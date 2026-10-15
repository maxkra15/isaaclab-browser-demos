# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Capture a Franka beam grasp and a clipped cloth with native Newton physics."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import newton
import newton._src.solvers.coupled.solver_coupled as coupled
import newton.ik as ik
import numpy as np
import trimesh
import warp as wp
from bundle import write_binary_files
from isaaclab_newton.physics import NewtonManager
from isaaclab_newton.sim.schemas import MujocoRigidBodyCfg
from manipulation import _append_mesh, _joint_coordinates, _pack_visuals
from newton_web import Parameter, export_graph
from scenes import package_bundle, validate_toolchain

from isaaclab.app import launch_simulation
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.sim import get_current_stage

from isaaclab_tasks.utils.parse_cfg import parse_env_cfg


def record_bundle_hashes(bundle: Path, web: Path) -> None:
    """Bind native trajectory evidence to the exact compiled artifact, including its mesh."""
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(web.iterdir()) if p.is_file()}
    (bundle / "validation.json").write_text(json.dumps({"files": hashes}, indent=2) + "\n")


@wp.kernel
def _gather_1d(source: wp.array(dtype=Any), mapping: wp.array(dtype=int), destination: wp.array(dtype=Any)):
    i = wp.tid()
    destination[i] = source[mapping[i]]


@wp.kernel
def _gather_2d(source: wp.array2d(dtype=Any), mapping: wp.array(dtype=int), destination: wp.array2d(dtype=Any)):
    i, j = wp.tid()
    destination[i, j] = source[mapping[i], j]


@contextmanager
def portable_coupler_controls():
    """Capture Newton's indexed control copies as equivalent explicit gather kernels.

    Warp 1.17 CPU APIC cannot record ``wp.copy(dst, src[indices])``. Keep this
    adapter build-local; it neither changes Newton's controls nor approximates physics.
    """
    original = coupled._copy_mapped_control_attribute

    def copy(source_control, destination_control, name, mapping):
        source = coupled._nested_attribute_value(source_control, name)
        destination = coupled._nested_attribute_value(destination_control, name)
        if not isinstance(source, wp.array) or not isinstance(destination, wp.array) or not destination.shape[0]:
            original(source_control, destination_control, name, mapping)
            return
        if source.ndim not in (1, 2):
            raise ValueError(f"Unreviewed coupled control shape: {name}, {source.shape}")
        wp.launch(
            _gather_1d if source.ndim == 1 else _gather_2d,
            destination.shape,
            [source, mapping, destination],
            device=destination.device,
        )

    coupled._copy_mapped_control_attribute = copy
    try:
        yield
    finally:
        coupled._copy_mapped_control_attribute = original


@wp.kernel
def _grasp_site(q: wp.array(dtype=wp.vec3), clock: wp.array(dtype=float), site: wp.array(dtype=wp.vec3), cloth: int):
    if clock[0] < 4.0:
        low = wp.vec3(1.0e6)
        high = wp.vec3(-1.0e6)
        selected = wp.vec3(0.48, 0.0, 0.15)
        best = float(-1.0e6)
        for i in range(q.shape[0]):
            p = q[i]
            low = wp.min(low, p)
            high = wp.max(high, p)
            # Grasp the exposed cloth corner, outside the support blocks.
            if p[0] > 0.465 and p[0] < 0.51:
                score = p[2] - 2.0 * wp.abs(p[0] - 0.485)
                if score > best:
                    best = score
                    selected = p
        if cloth != 0:
            site[0] = selected - wp.vec3(0.0, 0.0, 0.015)
        else:
            site[0] = 0.5 * (low + high) - wp.vec3(0.0, 0.0, 0.01)


@wp.kernel
def _trajectory(
    clock: wp.array(dtype=float),
    height: wp.array(dtype=float),
    release: wp.array(dtype=int),
    goal: wp.array(dtype=wp.vec3),
    grip: wp.array(dtype=float),
    site: wp.array(dtype=wp.vec3),
    closed: float,
    dt: float,
):
    t = clock[0]
    grasp_z = site[0][2]
    above = grasp_z + 0.13
    z = above
    opening = float(0.04)
    if t >= 2.0:
        z = wp.lerp(above, grasp_z, wp.clamp((t - 2.0) / 2.0, 0.0, 1.0))
    if t >= 4.0:
        opening = wp.lerp(0.04, closed, wp.clamp((t - 4.0) / 1.5, 0.0, 1.0))
    if t >= 5.5:
        z = wp.lerp(grasp_z, height[0], wp.clamp((t - 5.5) / 2.5, 0.0, 1.0))
    if release[0] != 0:
        opening = 0.04
    desired = wp.vec3(site[0][0], site[0][1], z)
    displacement = desired - goal[0]
    goal[0] += displacement * wp.min(1.0, 0.2 * dt / wp.max(wp.length(displacement), 1.0e-6))
    grip[0] = opening
    clock[0] = wp.min(t + dt, 9.0)


@wp.kernel
def _robot_targets(
    q: wp.array2d(dtype=float),
    coordinates: wp.array(dtype=int),
    grip: wp.array(dtype=float),
    targets: wp.array2d(dtype=float),
):
    i = wp.tid()
    if i < 7:
        targets[0, i] = q[0, coordinates[i]]
    else:
        targets[0, i] = grip[0]


@wp.kernel
def _volume_material(scale: wp.array(dtype=float), initial: wp.array2d(dtype=float), material: wp.array2d(dtype=float)):
    i, j = wp.tid()
    material[i, j] = initial[i, j] * wp.where(j < 2, scale[0], 1.0)


@wp.kernel
def _cloth_material(scale: wp.array(dtype=float), initial: wp.array2d(dtype=float), material: wp.array2d(dtype=float)):
    i, j = wp.tid()
    material[i, j] = initial[i, j] * wp.where(j == 0, scale[0], 1.0)


@wp.kernel
def _drag_object(
    index: wp.array(dtype=int),
    target: wp.array(dtype=wp.vec3),
    q: wp.array(dtype=wp.vec3),
    qd: wp.array(dtype=wp.vec3),
    force: wp.array(dtype=wp.vec3),
):
    i = index[0]
    if i >= 0:
        f = 20.0 * (target[0] - q[i]) - 0.15 * qd[i]
        force[i] += f * wp.min(1.0, 2.0 / wp.max(wp.length(f), 1.0e-6))


@wp.kernel
def _corner_clip(
    clock: wp.array(dtype=float),
    release: wp.array(dtype=int),
    attached: wp.array(dtype=int),
    ids: wp.array(dtype=int),
    offsets: wp.array(dtype=wp.vec3),
    body_q: wp.array(dtype=wp.transform),
    q: wp.array(dtype=wp.vec3),
    qd: wp.array(dtype=wp.vec3),
    flags: wp.array(dtype=int),
    inv_mass: wp.array(dtype=float),
    soft_flags: wp.array(dtype=int),
    soft_inv_mass: wp.array(dtype=float),
    original_inv_mass: wp.array(dtype=float),
    original_flags: wp.array(dtype=int),
    hand: int,
):
    pose = body_q[hand]
    if clock[0] >= 4.0 and release[0] == 0 and attached[0] == 0:
        tcp = wp.transform_point(pose, wp.vec3(0.0, 0.0, 0.107))
        for slot in range(3):
            best = float(1.0e10)
            selected = int(0)
            for i in range(q.shape[0]):
                used = bool(False)
                for previous in range(slot):
                    if ids[previous] == i:
                        used = True
                distance = wp.length_sq(q[i] - tcp)
                if not used and distance < best:
                    selected = i
                    best = distance
            ids[slot] = selected
            offsets[slot] = wp.transform_point(wp.transform_inverse(pose), q[selected])
            if slot == 0 and best > 0.06 * 0.06:
                return
        attached[0] = 1
    if attached[0] != 0:
        for slot in range(3):
            i = ids[slot]
            if release[0] != 0:
                inv_mass[i] = original_inv_mass[i]
                flags[i] = original_flags[i]
                soft_inv_mass[i] = inv_mass[i]
                soft_flags[i] = flags[i]
            else:
                # Match Isaac Lab's enforce_kinematic_targets: retain physical mass,
                # set zero inverse mass and clear ACTIVE, then prescribe the pose.
                inv_mass[i] = 0.0
                flags[i] = 0
                soft_inv_mass[i] = 0.0
                soft_flags[i] = 0
                q[i] = wp.transform_point(pose, offsets[slot])
                qd[i] = wp.vec3(0.0)
        if release[0] != 0:
            attached[0] = 0


def pack_workbench(stage, model, output: Path, cloth: bool, extra_roots=()) -> dict:
    """Use the native robot/support visuals and a light visual bench over the native support collider."""
    roots = ["/World/envs/env_0/Robot"]
    if cloth:
        roots += ["/World/envs/env_0/SupportNegY", "/World/envs/env_0/SupportPosY"]
    visuals = _pack_visuals(stage, roots + list(extra_roots), model.body_label, output)
    data = bytearray(output.read_bytes())
    pieces = [((1.3, 0.9, 0.045), (0.5, 0, -0.0225), (0.24, 0.31, 0.34))]
    pieces += [((0.055, 0.055, 1.0), (x, y, -0.55), (0.065, 0.085, 0.10)) for x in (-0.08, 1.08) for y in (-0.38, 0.38)]
    for index, (size, center, color) in enumerate(pieces):
        mesh = trimesh.creation.box(extents=size)
        vertices = (mesh.vertices + center).astype("<f4")
        _append_mesh(data, visuals["meshes"], vertices, mesh.faces, -1, f"workbench_{index}", color)
    return batch_visuals(visuals, bytes(data), output)


def batch_visuals(visuals: dict, raw: bytes, output: Path) -> dict:
    # All geometry here has a body-local transform and one flat material.
    # Batch by body/material at build time to reduce browser draw calls.
    groups = {}
    for mesh in visuals["meshes"]:
        key = (mesh["body"], tuple(mesh["color"]))
        groups.setdefault(key, []).append(mesh)
    data, merged = bytearray(), []
    for index, ((body, color), parts) in enumerate(groups.items()):
        vertices, faces, count = [], [], 0
        for part in parts:
            vertices.append(
                np.frombuffer(raw, dtype="<f4", count=part["vertexCount"] * 3, offset=part["vertexOffset"]).reshape(
                    -1, 3
                )
            )
            faces.append(
                np.frombuffer(raw, dtype="<u2", count=part["indexCount"], offset=part["indexOffset"])
                .reshape(-1, 3)
                .astype(np.int32)
                + count
            )
            count += part["vertexCount"]
        vertices = np.concatenate(vertices)
        faces = np.concatenate(faces)
        if body < 0:
            # Bench boxes have planar faces, rather than smoothed corner normals.
            vertices = vertices[faces.reshape(-1)]
            faces = np.arange(len(vertices)).reshape(-1, 3)
        _append_mesh(data, merged, vertices, faces, body, f"body_{body}_{index}", color)
    output.write_bytes(data)
    return {"file": output.name, "byteLength": len(data), "meshes": merged}


def export(kind: str, output: Path) -> None:
    versions = validate_toolchain()
    cloth = kind == "cloth"
    task = f"Isaac-Lift-{'Cloth' if cloth else 'Soft'}-Franka"
    cfg = parse_env_cfg(task, device="cpu", num_envs=1)
    cfg.play_mode()
    # One reproducible reset; preserve startup terms but omit task reset randomization.
    for name, term in vars(cfg.events).items():
        if getattr(term, "mode", None) == "reset":
            setattr(cfg.events, name, None)
    # The native task requests gravity-free arms using a PhysX field. Author its
    # Newton equivalent so gravity sag does not move the fingers below the table.
    cfg.scene.robot.spawn.rigid_props = {"/.*": [cfg.scene.robot.spawn.rigid_props, MujocoRigidBodyCfg(gravcomp=1.0)]}
    cfg.seed = 42
    cfg.scene.deformable.spawn.edge_refinement = 3 if cloth else 8
    if not cloth:
        cfg.scene.deformable.spawn.physics_material.density = 1000.0
    cfg.sim.dt = 1.0 / 120.0
    cfg.sim.physics.use_cuda_graph = False
    cfg.sim.physics.num_substeps = 2
    cfg.sim.physics.solver_cfg.entries[0].solver_cfg.use_mujoco_contacts = True
    if cloth:
        # The corner fixture prescribes the cloth boundary. Transfer the support
        # blocks to VBD, excluding hand/cloth contact around that fixture.
        mapping = cfg.sim.physics.solver_cfg.proxies[0]
        mapping.bodies = [selector for selector in mapping.bodies if "Support" in str(selector)]
        if not mapping.bodies:
            raise ValueError("Cloth export requires the native support-block proxy mapping")
    for entry in cfg.sim.physics.solver_cfg.entries:
        if hasattr(entry.solver_cfg, "particle_enable_tile_solve"):
            entry.solver_cfg.particle_enable_tile_solve = False
    with launch_simulation(cfg, {"visualizer": ["none"], "device": "cpu"}):
        env = ManagerBasedRLEnv(cfg)
        try:
            env.reset(seed=42)
            model, state = NewtonManager.get_model(), NewtonManager.get_state_0()
            robot = env.scene["robot"]
            robot.write_joint_state_to_sim_index(
                position=robot.data.default_joint_pos.torch, velocity=robot.data.default_joint_vel.torch
            )
            robot.actuators.target_command.position.warp.assign(robot.data.default_joint_pos.torch.numpy())
            for _ in range(5):
                env.scene.write_data_to_sim()
                env.sim.step(render=False)
            coordinates, dofs = _joint_coordinates(model, robot.joint_names)
            hand = next(i for i, name in enumerate(model.body_label) if name.endswith("/panda_hand"))
            root = next(i for i, name in enumerate(model.body_label) if name.endswith("/panda_link0"))
            hand_pose = wp.transform(*state.body_q.numpy()[hand])
            tcp = wp.transform_point(hand_pose, wp.vec3(0, 0, 0.107))
            goal = wp.array([tcp], dtype=wp.vec3, device="cpu")
            q_ik = wp.clone(state.joint_q).reshape((1, model.joint_coord_count))
            coordinate = wp.array(coordinates, dtype=int, device="cpu")
            mask = np.zeros(model.joint_dof_count, dtype=bool)
            mask[dofs[:7]] = True
            inverse = ik.IKSolver(
                model,
                1,
                [
                    ik.IKObjectivePosition(hand, wp.vec3(0, 0, 0.107), goal),
                    ik.IKObjectiveRotation(
                        hand, wp.quat_identity(), wp.array([wp.vec4(1, 0, 0, 0)], dtype=wp.vec4, device="cpu")
                    ),
                    ik.IKObjectiveJointLimit(model.joint_limit_lower, model.joint_limit_upper, weight=5.0),
                ],
                jacobian_mode="analytic",
                joint_dof_mask=wp.array(mask, dtype=bool, device="cpu"),
            )
            clock = wp.zeros(1, device="cpu")
            site = wp.zeros(1, dtype=wp.vec3, device="cpu")
            height = wp.array([0.32 if cloth else 0.18], dtype=float, device="cpu")
            release = wp.zeros(1, dtype=int, device="cpu")
            grip = wp.array([0.04], dtype=float, device="cpu")
            drag_index = wp.array([-1], dtype=int, device="cpu")
            drag_target = wp.zeros(1, dtype=wp.vec3, device="cpu")
            NewtonManager.register_state_force_callback(
                lambda s: wp.launch(
                    _drag_object, 1, [drag_index, drag_target, s.particle_q, s.particle_qd, s.particle_f], device="cpu"
                )
            )
            scale = wp.array([0.05 if cloth else 1.0], dtype=float, device="cpu")
            material = model.edge_bending_properties if cloth else model.tet_materials
            initial_material = wp.clone(material)
            apply_material = _cloth_material if cloth else _volume_material
            # Native proxy submodels retain their own material buffers; update the soft entry too.
            physics_solver = NewtonManager._solver
            soft_model = physics_solver.solver("soft").model
            soft_material = soft_model.edge_bending_properties if cloth else soft_model.tet_materials
            if soft_material.shape != material.shape:
                raise ValueError("Unexpected proxy material mapping")
            if cloth:
                attached = wp.zeros(1, dtype=int, device="cpu")
                clip_ids = wp.array([-1, -1, -1], dtype=int, device="cpu")
                clip_offsets = wp.zeros(3, dtype=wp.vec3, device="cpu")
                original_inv_mass = wp.clone(model.particle_inv_mass)
                original_flags = wp.clone(model.particle_flags)
                clip_states = {
                    state.particle_q.ptr: state
                    for state in (
                        NewtonManager.get_state_0(),
                        NewtonManager.get_state_1(),
                        physics_solver.entry_state("soft", "input"),
                        physics_solver.entry_state("soft", "output"),
                    )
                }

                def apply_clip(current):
                    # Match the native nodal-target path: every state buffer must
                    # receive the prescribed position before either solver swaps it.
                    for destination in clip_states.values():
                        wp.launch(
                            _corner_clip,
                            1,
                            [
                                clock,
                                release,
                                attached,
                                clip_ids,
                                clip_offsets,
                                current.body_q,
                                destination.particle_q,
                                destination.particle_qd,
                                model.particle_flags,
                                model.particle_inv_mass,
                                soft_model.particle_flags,
                                soft_model.particle_inv_mass,
                                original_inv_mass,
                                original_flags,
                                hand,
                            ],
                            device="cpu",
                        )

                NewtonManager.register_state_force_callback(apply_clip)
            with portable_coupler_controls(), wp.ScopedCapture(device="cpu", apic=True) as capture:
                wp.launch(apply_material, material.shape, [scale, initial_material, material], device="cpu")
                if soft_material.ptr != material.ptr:
                    wp.launch(apply_material, material.shape, [scale, initial_material, soft_material], device="cpu")
                wp.launch(_grasp_site, 1, [state.particle_q, clock, site, int(cloth)], device="cpu")
                wp.launch(
                    _trajectory,
                    1,
                    [clock, height, release, goal, grip, site, 0.0 if cloth else 0.01, cfg.sim.dt],
                    device="cpu",
                )
                inverse.step(q_ik, q_ik, iterations=3)
                wp.launch(
                    _robot_targets,
                    len(coordinates),
                    [q_ik, coordinate, grip, robot.actuators.target_command.position.warp],
                    device="cpu",
                )
                env.scene.write_data_to_sim()
                env.sim.step(render=False)
            regions = {}
            for key, region in capture.graph._apic_capture._regions.items():
                if region[0] not in regions or region[2] > regions[region[0]][1][2]:
                    regions[region[0]] = (key, region)
            capture.graph._apic_capture._regions = dict(regions.values())
            bindings = {
                n: getattr(state, n) for n in ("body_q", "body_qd", "joint_q", "joint_qd", "particle_q", "particle_qd")
            }
            display = SimpleNamespace(
                body_count=model.body_count,
                particle_count=model.particle_count,
                tri_count=model.tri_count,
                tri_indices=model.tri_indices,
                tri_color=model.tri_color,
                shape_type=wp.array([int(newton.GeoType.PLANE)], dtype=int, device="cpu"),
                shape_body=wp.array([-1], dtype=int, device="cpu"),
                shape_scale=wp.array([wp.vec3(8, 8, 0)], dtype=wp.vec3, device="cpu"),
                shape_transform=wp.array(
                    [wp.transform(wp.vec3(0, 0, -1.05), wp.quat_identity())], dtype=wp.transform, device="cpu"
                ),
            )
            export_graph(
                capture.graph,
                model=display,
                inputs={
                    **bindings,
                    "height": height,
                    "material": scale,
                    "release": release,
                    "clock": clock,
                    "drag_index": drag_index,
                    "drag_target": drag_target,
                },
                outputs=bindings,
                output=output,
                timestep=cfg.sim.dt,
                persistent=("height", "material"),
                parameters=(
                    Parameter("height", 0, "Lift height [m]", 0.22 if cloth else 0.06, 0.4, 0.005),
                    Parameter(
                        "material",
                        0,
                        "Bending scale" if cloth else "Elastic stiffness scale",
                        0.002 if cloth else 0.1,
                        2.0 if cloth else 10.0,
                        0.001,
                    ),
                ),
            )
            visuals = pack_workbench(get_current_stage(), model, output / "visuals.bin", cloth)
            path = output / "manifest.json"
            manifest = json.loads(path.read_text())
            manifest["isaacLabDemo"] = {
                "kind": "franka_" + kind,
                "title": "Franka cloth clip" if cloth else "Franka soft-body grasp",
                "viewer": "robot",
                "controls": "parameters",
                "visuals": visuals,
                "rootBody": root,
                "cameraTarget": [0.35, 0.2, 0],
                "cameraRadius": 1.85,
                "cameraAzimuth": -2.1,
                "groundHeight": -1.05,
                "toneMappingExposure": 1.15,
                "logParameters": ["material"],
                "maxStepsPerFrame": 8,
                "renderFPS": 30,
                "softDrag": True,
                "boundaryNote": (
                    "Corner clip: three vertices follow the gripper. "
                    "Hand/cloth contact and reaction forces are omitted."
                    if cloth
                    else "Contact grasp · Scripted Newton IK, no trained policy"
                ),
                "actions": [
                    {"label": "Pick & lift", "reset": True},
                    {"label": "Release", "binding": "release", "value": 1},
                ],
                "source": {
                    "task": task,
                    "versions": versions,
                    "controller": "Newton analytic IK; scripted approach, grasp and lift; no policy",
                },
            }
            path.write_text(json.dumps(manifest, indent=2) + "\n")
            rows, particles, frames = [], [], []
            frames_per_second = round(1.0 / cfg.sim.dt)
            for frame in range(14 * frames_per_second):
                wp.capture_launch(capture.graph)
                if frame < 60 or (frame + 1) % (frames_per_second // 2) == 0:
                    frames.append(frame + 1)
                    rows.append(state.body_q.numpy().copy())
                    particles.append(state.particle_q.numpy().copy())
                if frame % frames_per_second == frames_per_second - 1:
                    poses = state.body_q.numpy()
                    q = state.particle_q.numpy()
                    actual = wp.transform_point(wp.transform(*poses[hand]), wp.vec3(0, 0, 0.107))
                    print(
                        "TIME",
                        (frame + 1) * cfg.sim.dt,
                        "TCP",
                        list(actual),
                        "OBJECT",
                        q.mean(axis=0),
                        "MINZ",
                        q[:, 2].min(),
                        flush=True,
                    )
                if not all(np.isfinite(getattr(state, name).numpy()).all() for name in bindings):
                    raise RuntimeError("Non-finite coupled trajectory")
            final_height = float(state.particle_q.numpy()[:, 2].mean())
            if final_height < (0.15 if cloth else 0.1):
                raise RuntimeError(f"{task}: grasp did not hold the object, mean height {final_height:.3f} m")
            np.savez(output / "reference.npz", frames=frames, body_q=rows, particle_q=particles)
            print("VERIFIED", task, model.body_count, "bodies", model.particle_count, "particles", flush=True)
        finally:
            env.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("kind", choices=("soft", "cloth"))
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--emxx", default="em++")
    a = p.parse_args()
    bundle = a.output / ("franka_" + a.kind)
    web = a.output / ("franka_" + a.kind + "-web")
    export(a.kind, bundle)
    manifest = package_bundle(bundle, web, a.emxx)
    files = write_binary_files(gzip.compress((bundle / "visuals.bin").read_bytes(), mtime=0), web, "visuals.bin.gz")
    manifest["isaacLabDemo"]["visuals"].update(file="visuals.bin.gz", files=files)
    (web / "manifest.json").write_text(json.dumps(manifest, separators=(",", ":")) + "\n")
    record_bundle_hashes(bundle, web)


if __name__ == "__main__":
    main()
