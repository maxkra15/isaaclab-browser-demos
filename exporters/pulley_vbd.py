# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Capture instrumented VBD capsule cables in contact with passive sheaves.

Only the test handle has prescribed motion. The load, wheel and every cable
segment advance through native Newton VBD, including cylinder/capsule contacts.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

import newton
import numpy as np
import warp as wp
from newton._src.solvers.vbd.rigid_vbd_kernels import (
    evaluate_linear_constraint_force_hessian,
)
from newton_web import Parameter, export_graph
from pulley import _payload, _restore, _snapshot
from scenes import FRAME_DT, package_bundle, validate_toolchain

ROPE_RADIUS = 0.006
SEGMENT_LENGTH = 0.035
TOP = 1.35
MOVING_Z = 0.85
LOAD_Z = 0.55
HANDLE_Z = 0.64
SUBSTEPS = 16
ITERATIONS = 16
STRETCH_STIFFNESS = 1.0e4
FRICTION = 0.002
GUIDE_DAMPING = 20.0
CONTACTS_PER_BODY = 256


@wp.kernel
def _move_handle(
    pull: wp.array(dtype=float),
    command: wp.array(dtype=float),
    q: wp.array(dtype=wp.transform),
    handle: int,
    origin: wp.vec3,
    dt: float,
):
    error = wp.clamp(pull[0], 0.0, 0.5) - command[0]
    wanted_velocity = wp.sign(error) * wp.min(0.12, wp.sqrt(2.0 * 0.4 * wp.abs(error)))
    velocity = command[1] + wp.clamp(wanted_velocity - command[1], -0.4 * dt, 0.4 * dt)
    displacement = velocity * dt
    if wp.abs(displacement) >= wp.abs(error):
        displacement = error
        velocity = 0.0
    command[0] = command[0] + displacement
    command[1] = velocity
    q[handle] = wp.transform(origin - wp.vec3(0.0, 0.0, command[0]), wp.quat_identity())


@wp.kernel
def _measure(
    q: wp.array(dtype=wp.transform),
    q_prev: wp.array(dtype=wp.transform),
    child_frame: wp.transform,
    row: int,
    joint: int,
    handle: int,
    cable: int,
    load: int,
    rho: wp.array(dtype=float),
    material: wp.array(dtype=float),
    reaction: wp.array(dtype=wp.vec3),
    initial_error: wp.array(dtype=wp.vec3),
    mass: wp.array(dtype=float),
    moving_mass: float,
    strands: int,
    alpha: float,
    dt: float,
    telemetry: wp.array(dtype=float),
):
    parent = q[handle]
    child = q[cable]
    # Evaluate the native BALL attachment force with exactly the solver's ALM
    # state. This reads a reaction; it does not update the dual a second time.
    # Damping is zero, so VBD's advanced q_prev snapshot does not affect it.
    force, _torque, _hll, _hal, _haa = evaluate_linear_constraint_force_hessian(
        parent,
        child * child_frame,
        q_prev[handle],
        q_prev[cable] * child_frame,
        parent,
        child,
        wp.vec3(0.0),
        wp.vec3(0.0),
        True,
        rho[row],
        material[row],
        wp.identity(3, float),
        reaction[joint],
        initial_error[joint],
        alpha,
        0.0,
        1,
        dt,
    )
    telemetry[0] = telemetry[0] + force[2] / float(SUBSTEPS)
    telemetry[1] = (mass[0] + moving_mass) * 9.81 / float(strands)
    telemetry[2] = wp.transform_get_translation(q[load])[2]
    telemetry[3] = wp.transform_get_translation(parent)[2]
    telemetry[4] = telemetry[4] + wp.length(force) / float(SUBSTEPS)
    telemetry[5] = wp.length(wp.transform_get_translation(child * child_frame) - wp.transform_get_translation(parent))


def build_system(strands: int = 1) -> SimpleNamespace:
    """Build a finite-mass elastic rod, load, revolute sheave and test handle."""
    builder = newton.ModelBuilder()
    builder.rigid_gap = 0.001
    radius = 0.115 if strands == 1 else 0.075
    wrap_radius = radius + 1.1 * ROPE_RADIUS
    top = 1.20 if strands == 4 else TOP
    lower = [4 * wrap_radius * (i - 0.5 * (strands // 2 - 1)) for i in range(strands // 2)]
    upper = [x + 2 * wrap_radius for x in lower] if lower else [0.0]
    load_origin = wp.vec3(-wrap_radius if strands == 1 else 0.0, 0.0, LOAD_Z)
    load = builder.add_link(xform=wp.transform(load_origin, wp.quat_identity()), label="load")
    builder.add_shape_box(
        load,
        hx=0.1,
        hy=0.07,
        hz=0.1,
        cfg=newton.ModelBuilder.ShapeConfig(density=1.0 / (8 * 0.1 * 0.07 * 0.1), has_shape_collision=False),
    )
    load_joint = builder.add_joint_prismatic(
        -1,
        load,
        axis=wp.vec3(0, 0, 1),
        parent_xform=wp.transform(load_origin, wp.quat_identity()),
        target_kd=GUIDE_DAMPING,
    )
    align_y = wp.quat_from_axis_angle(wp.vec3(1, 0, 0), -math.pi * 0.5)
    moving, fixed = [], []
    load_joints = [load_joint]
    fixed_joints = []
    for x, is_moving in [*((x, True) for x in lower), *((x, False) for x in upper)]:
        wheel_origin = wp.vec3(x, 0, MOVING_Z if is_moving else top)
        wheel = builder.add_link(
            xform=wp.transform(wheel_origin, wp.quat_identity()),
            label="moving_sheave" if is_moving else "fixed_sheave",
        )
        axle = builder.add_joint_revolute(
            load if is_moving else -1,
            wheel,
            axis=wp.vec3(0, 1, 0),
            parent_xform=wp.transform(wheel_origin - (load_origin if is_moving else wp.vec3(0.0)), wp.quat_identity()),
            armature=1e-5,
        )
        if is_moving:
            moving.append(wheel)
            load_joints.append(axle)
        else:
            fixed.append(wheel)
            fixed_joints.append(axle)
        for offset, shape_radius, width in (
            (0.0, radius, 0.012),
            (-0.016, radius + 0.014, 0.004),
            (0.016, radius + 0.014, 0.004),
        ):
            builder.add_shape_cylinder(
                wheel,
                xform=wp.transform(wp.vec3(0, offset, 0), align_y),
                radius=shape_radius,
                half_height=width,
                cfg=newton.ModelBuilder.ShapeConfig(density=100.0, ke=1e5, kd=0.0, mu=FRICTION),
            )
    builder.add_articulation(load_joints)
    for axle in fixed_joints:
        builder.add_articulation([axle])
    handle_origin = wp.vec3(upper[-1] + wrap_radius, 0, HANDLE_Z)
    handle = builder.add_body(
        xform=wp.transform(handle_origin, wp.quat_identity()),
        is_kinematic=True,
        label="test_handle",
    )
    arc_length = math.pi * wrap_radius
    anchor_origin = wp.vec3(-wrap_radius, 0, LOAD_Z + 0.1) if strands == 1 else wp.vec3(lower[0] - wrap_radius, 0, top)
    ordered = (
        [(0.0, top, False)]
        if strands == 1
        else [wheel for pair in zip(lower, upper) for wheel in ((pair[0], MOVING_Z, True), (pair[1], top, False))]
    )
    # Route a single continuous rope through alternating lower and upper wraps.
    # Each piece stores its analytic arclength; all physical rod segments have
    # the same rest length, including those that cross piece boundaries.
    pieces = []
    previous_z = float(anchor_origin[2])
    for x, z, is_moving in ordered:
        pieces.append((abs(z - previous_z), x - wrap_radius, previous_z, z, 0))
        pieces.append((arc_length, x, z, z, -1 if is_moving else 1))
        previous_z = z
    pieces.append((top - HANDLE_Z, float(handle_origin[0]), top, HANDLE_Z, 0))
    length = sum(piece[0] for piece in pieces)
    count = math.ceil(length / SEGMENT_LENGTH)
    spacing = length / count
    route = []
    for index in range(count + 1):
        distance = index * spacing
        for piece in pieces:
            piece_length, x, z0, z1, side = piece
            if distance <= piece_length or piece is pieces[-1]:
                distance = min(distance, piece_length)
                if side:
                    angle = math.pi - distance / wrap_radius
                    route.append(
                        wp.vec3(x + wrap_radius * math.cos(angle), 0, z0 + side * wrap_radius * math.sin(angle))
                    )
                else:
                    route.append(wp.vec3(x, 0, z0 + (z1 - z0) * distance / piece_length))
                break
            distance -= piece_length
    orientations = newton.Rod(route).quaternions
    straight = newton.Rod.create_straight(
        start=route[0], direction=wp.vec3(1, 0, 0), length=length, segment_count=count, radius=ROPE_RADIUS
    )
    cable, joints = builder.add_rod(
        rod=straight,
        body_frame_origin="com",
        wrap_in_articulation=False,
        stretch_stiffness=STRETCH_STIFFNESS,
        shear_stiffness=STRETCH_STIFFNESS,
        bend_stiffness=0.0,
        twist_stiffness=0.0,
        cfg=newton.ModelBuilder.ShapeConfig(density=50.0, ke=1e5, kd=0.0, mu=FRICTION, gap=0.001),
        label="cable",
    )
    anchor = builder.add_joint_ball(
        load if strands == 1 else -1,
        cable[0],
        parent_xform=wp.transform(wp.vec3(0, 0, 0.1) if strands == 1 else route[0], wp.quat_identity()),
        child_xform=wp.transform(wp.vec3(0, 0, -0.5 * spacing), wp.quat_identity()),
        label="load_attachment" if strands == 1 else "world_anchor",
    )
    sensor = builder.add_joint_ball(
        handle,
        cable[-1],
        child_xform=wp.transform(wp.vec3(0, 0, 0.5 * spacing), wp.quat_identity()),
        label="handle_load_cell",
    )
    builder.add_articulation(joints)
    for index, a in enumerate(cable):
        for b in cable[index + 1 :]:
            builder.add_shape_collision_filter_pair(builder.body_shapes[a][0], builder.body_shapes[b][0])
    builder.color()
    model = builder.finalize(device="cpu")
    solver = newton.solvers.SolverVBD(
        model,
        iterations=ITERATIONS,
        rigid_compliant_alm=True,
        rigid_contact_hard=False,
        rigid_body_contact_buffer_size=CONTACTS_PER_BODY,
    )
    pipeline = newton.CollisionPipeline(model, broad_phase="explicit")
    state, next_state = model.state(), model.state()
    wrapped = model.body_q.numpy()
    for index, body in enumerate(cable):
        wrapped[body] = (*((route[index] + route[index + 1]) * 0.5), *orientations[index])
    state.body_q.assign(wrapped)
    next_state.body_q.assign(wrapped)
    child_frame = model.joint_X_c.numpy()[sensor]
    return SimpleNamespace(
        model=model,
        solver=solver,
        pipeline=pipeline,
        contacts=pipeline.contacts(),
        state=state,
        next_state=next_state,
        control=model.control(),
        load=load,
        moving=moving,
        fixed=fixed,
        strands=strands,
        radius=radius,
        wrap_radius=wrap_radius,
        top=top,
        lower=lower,
        anchor_origin=anchor_origin,
        moving_mass=float(model.body_mass.numpy()[moving].sum()),
        handle=handle,
        handle_origin=handle_origin,
        cable=cable,
        anchor=anchor,
        sensor=sensor,
        spacing=spacing,
        child_frame=wp.transform(wp.vec3(*child_frame[:3]), wp.quat(*child_frame[3:])),
        sensor_row=int(solver.joint_constraint_start.numpy()[sensor]),
        reference_inertia=wp.mat33(model.body_inertia.numpy()[load]),
        command=wp.zeros(2, dtype=float, device="cpu"),
        values={
            "pull": wp.array([0.0], dtype=float, device="cpu"),
            "load": wp.array([1.0], dtype=float, device="cpu"),
            "telemetry": wp.zeros(6, dtype=float, device="cpu"),
        },
    )


def frame(scene: SimpleNamespace) -> None:
    m, s, values = scene.model, scene.solver, scene.values
    values["telemetry"].zero_()
    wp.launch(
        _payload,
        1,
        [
            values["load"],
            m.body_mass,
            m.body_inv_mass,
            m.body_inertia,
            m.body_inv_inertia,
            scene.reference_inertia,
            scene.load,
        ],
        device="cpu",
    )
    s.notify_model_changed(newton.ModelFlags.BODY_INERTIAL_PROPERTIES)
    dt = FRAME_DT / SUBSTEPS
    for _ in range(SUBSTEPS):
        scene.state.clear_forces()
        wp.launch(
            _move_handle,
            1,
            [values["pull"], scene.command, scene.state.body_q, scene.handle, scene.handle_origin, dt],
            device="cpu",
        )
        scene.pipeline.collide(scene.state, scene.contacts)
        s.step(scene.state, scene.next_state, scene.control, scene.contacts, dt)
        wp.copy(scene.state.body_q, scene.next_state.body_q)
        wp.copy(scene.state.body_qd, scene.next_state.body_qd)
        wp.launch(
            _measure,
            1,
            [
                scene.state.body_q,
                s.body_q_prev,
                scene.child_frame,
                scene.sensor_row,
                scene.sensor,
                scene.handle,
                scene.cable[-1],
                scene.load,
                s.joint_rho,
                s.joint_material_k,
                s.joint_lambda_lin,
                s.joint_C0_lin,
                values["load"],
                scene.moving_mass,
                scene.strands,
                s.rigid_joint_alpha,
                dt,
                values["telemetry"],
            ],
            device="cpu",
        )


def metadata(scene: SimpleNamespace) -> dict:
    m = scene.model
    shapes = m.shape_body.numpy()
    scales, transforms = m.shape_scale.numpy(), m.shape_transform.numpy()
    cable = []
    for body in scene.cable:
        shape = int(np.flatnonzero(shapes == body)[0])
        cable.append(
            {
                "body": body,
                "radius": float(scales[shape, 0]),
                "length": float(2 * scales[shape, 1]),
                "transform": transforms[shape].tolist(),
            }
        )
    return {
        "kind": "pulley",
        "title": "VBD cable pulley",
        "viewer": "pulley",
        "controls": "parameters",
        "ropeModel": "vbd",
        "loadBody": scene.load,
        "loadSize": [0.2, 0.14, 0.2],
        "handleBody": scene.handle,
        "supportingStrands": scene.strands,
        "movingSheaveMass": scene.moving_mass,
        "handleMass": 0.0,
        "telemetryFields": ["tension", "ideal", "loadHeight", "handleHeight", "reactionNorm", "attachmentGap"],
        "cable": cable,
        "ropeAnchor": {"body": scene.load, "position": [0, 0, 0.1]}
        if scene.strands == 1
        else {"body": -1, "position": list(scene.anchor_origin)},
        "variants": [],
        "pulleys": [
            {
                "body": wheel,
                "radius": scene.radius,
                "halfWidth": 0.02,
                "grooveHalfWidth": 0.012,
                "flangeHalfWidth": 0.004,
                "flangeOffset": 0.016,
                "flangeRadius": scene.radius + 0.014,
                "moving": wheel in scene.moving,
            }
            for wheel in [*scene.moving, *scene.fixed]
        ],
        "supportStrands": [
            {
                "top": [x + side * scene.wrap_radius, 0, scene.top],
                "bottom": [x + side * scene.wrap_radius, 0, MOVING_Z - LOAD_Z],
            }
            for x in scene.lower
            for side in (-1, 1)
        ],
        "cameraTarget": [0 if scene.strands == 1 else scene.wrap_radius, 0.8, 0],
        "cameraRadius": 2.0,
        "cameraAzimuth": math.pi / 2,
        "cameraElevation": 0.08,
        "maxStepsPerFrame": 2,
        "renderFPS": 30,
        "source": {
            "scene": "exporters/pulley_vbd.py",
            "referenceScene": "examples/demos/newton_viewer_block_and_tackle.py",
            "solver": "Newton VBD",
            "ropeModel": "Finite-mass capsule rod with real sheave contacts; cable self-collision filtered",
            "measurement": (
                "Native BALL-attachment reaction, signed vertical component averaged over all physics substeps"
            ),
            "handle": "Prescribed kinematic test fixture; load, cable and sheave remain dynamically simulated",
            "physicsSubsteps": SUBSTEPS,
            "iterations": ITERATIONS,
            "stretchStiffness": STRETCH_STIFFNESS,
            "friction": FRICTION,
            "guideDamping": GUIDE_DAMPING,
            "contactsPerBody": CONTACTS_PER_BODY,
            "cableMass": float(m.body_mass.numpy()[scene.cable].sum()),
        },
        "boundaryNote": (
            "VBD cable contacts · prescribed test handle · finite rope mass, elasticity and contact friction"
        ),
    }


def export(output: Path, emxx: str, native_only: bool = False, strands: int = 1) -> None:
    versions = validate_toolchain()
    scene = build_system(strands)
    with wp.ScopedCapture(device="cpu", apic=True) as capture:
        frame(scene)
    for _ in range(240):
        wp.capture_launch(capture.graph)
    with wp.ScopedCapture(device="cpu", apic=True) as capture:
        frame(scene)
    name = "pulley_vbd" if strands == 1 else f"pulley_vbd_{strands}"
    bundle = output / name
    display = SimpleNamespace(
        body_count=scene.model.body_count,
        particle_count=0,
        tri_count=0,
        shape_type=wp.array([int(newton.GeoType.PLANE)], dtype=int, device="cpu"),
        shape_body=wp.array([-1], dtype=int, device="cpu"),
        shape_scale=wp.array([wp.vec3(4, 4, 0)], dtype=wp.vec3, device="cpu"),
        shape_transform=wp.array([wp.transform_identity()], dtype=wp.transform, device="cpu"),
    )
    bindings = {"body_q": scene.state.body_q, "body_qd": scene.state.body_qd, "telemetry": scene.values["telemetry"]}
    export_graph(
        capture.graph,
        model=display,
        inputs={**bindings, "pull": scene.values["pull"], "load": scene.values["load"]},
        outputs=bindings,
        output=bundle,
        timestep=FRAME_DT,
        parameters=(
            Parameter("pull", 0, "Pull distance [m]", 0, 0.5, 0.005),
            Parameter("load", 0, "Payload [kg]", 0.5, 5, 0.1),
        ),
        persistent=("load",),
    )
    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["isaacLabDemo"] = {**metadata(scene), "versions": versions}
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    snapshot = _snapshot(capture.graph)
    results = []
    for mass in (0.5, 1.0, 5.0):
        _restore(snapshot)
        scene.values["load"].assign([mass])
        for _ in range(360):
            wp.capture_launch(capture.graph)
        baseline = float(scene.values["telemetry"].numpy()[2])
        rows, samples = [], []
        contact_peak = 0
        start = time.perf_counter()
        for step in range(540):
            if step == 60:
                scene.values["pull"].assign([0.5])
            wp.capture_launch(capture.graph)
            rows.append(scene.state.body_q.numpy().copy())
            samples.append(scene.values["telemetry"].numpy().copy())
            contact_peak = max(contact_peak, int(scene.solver.body_body_contact_counts.numpy().max()))
            if contact_peak > CONTACTS_PER_BODY:
                raise RuntimeError(f"VBD contact capacity exceeded: {contact_peak} > {CONTACTS_PER_BODY}")
        rows = np.asarray(rows)
        samples = np.asarray(samples)
        force = float(samples[-60:, 0].mean())
        ideal = float(samples[-1, 1])
        rise = float(samples[-1, 2] - baseline)
        gap = float(samples[:, 5].max())
        span = float(np.ptp(samples[-60:, 2]))
        wrap_height = float(rows[:, scene.cable, 2].max(axis=1).min())
        for wheel in [*scene.moving, *scene.fixed]:
            offset = rows[:, scene.cable, :3] - rows[:, wheel, None, :3]
            side = -1 if wheel in scene.moving else 1
            on_wrap = (
                (np.abs(offset[:, :, 0]) < 0.8 * scene.radius)
                & (np.abs(offset[:, :, 1]) < 0.02)
                & (side * offset[:, :, 2] > 0.4 * scene.radius)
                & (side * offset[:, :, 2] < 1.5 * scene.radius)
            )
            if not on_wrap.any(axis=1).all():
                raise RuntimeError(f"VBD cable escaped sheave {wheel} at {mass} kg")
        if not np.isfinite(rows).all() or not np.isfinite(samples).all():
            raise RuntimeError("Non-finite VBD cable trajectory")
        if abs(force / ideal - 1) > 0.1:
            raise RuntimeError(f"VBD static force differs from the ideal reference by over 10% at {mass} kg")
        if (
            samples[:, 2].min() < 0.05
            or abs(rise - 0.5 / strands) > 0.02 / strands
            or gap > 0.02
            or span > 0.002
            or wrap_height < scene.top + 0.4 * scene.radius
        ):
            raise RuntimeError(f"VBD cable lost its load or attachment at {mass} kg: rise={rise},gap={gap}")
        result = {
            "payload": mass,
            "tension": force,
            "idealMasslessForce": ideal,
            "forceRelativeError": force / ideal - 1,
            "rise": rise,
            "idealRise": 0.5 / strands,
            "maxAttachmentGap": gap,
            "settledPositionSpan": span,
            "minimumWrapHeight": wrap_height,
            "peakContactsPerBody": contact_peak,
            "millisecondsPerFrame": 1000 * (time.perf_counter() - start) / len(samples),
        }
        print("VALIDATED_VBD", json.dumps(result), flush=True)
        results.append(result)
        if mass == 1.0:
            np.savez(bundle / "reference.npz", body_q=rows, telemetry=samples)
    (bundle / "validation.json").write_text(json.dumps(results, indent=2) + "\n")
    _restore(snapshot)
    if not native_only:
        package_bundle(bundle, output / f"{name}-web", emxx)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--emxx", default="em++")
    parser.add_argument("--native-only", action="store_true")
    parser.add_argument("--strands", type=int, choices=(1, 2, 4), default=1)
    args = parser.parse_args()
    export(args.output, args.emxx, args.native_only, args.strands)


if __name__ == "__main__":
    main()
