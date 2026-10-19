# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Capture an instrumented Newton/MJWarp spatial-tendon block and tackle.

The rope is massless, with a unilateral length constraint and frictionless
cylindrical wrapping. The measured tension comes from MJWarp's solved generalized
constraint force on the handle, whose tendon Jacobian has unit magnitude. Gravity/theory are
never used to generate the measured force or to prescribe the load's position.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

import mujoco
import newton
import numpy as np
import warp as wp
from newton_web import Parameter, export_graph
from scenes import FRAME_DT, package_bundle, validate_toolchain

RADIUS = 0.075
ROPE_RADIUS = 0.006
TOP = 1.35
BOTTOM = 0.95
LOAD_Z = 0.65
HANDLE_Z = 0.64
DEFAULT_MASS = 1.0
SHEAVE_MASS = 0.02
HANDLE_MASS = 0.01
SUBSTEPS = 8


@wp.kernel
def _payload(
    value: wp.array(dtype=float),
    mass: wp.array(dtype=float),
    inverse_mass: wp.array(dtype=float),
    inertia: wp.array(dtype=wp.mat33),
    inverse_inertia: wp.array(dtype=wp.mat33),
    reference_inertia: wp.mat33,
    load: int,
):
    m = wp.clamp(value[0], 0.5, 5.0)
    mass[load] = m
    inverse_mass[load] = 1.0 / m
    inertia[load] = reference_inertia * (m / DEFAULT_MASS)
    inverse_inertia[load] = wp.inverse(inertia[load])


@wp.kernel
def _drive(
    pull: wp.array(dtype=float),
    command: wp.array(dtype=float),
    target_q: wp.array(dtype=float),
    target_qd: wp.array(dtype=float),
    coordinate: int,
    dof: int,
    dt: float,
):
    error = -wp.clamp(pull[0], 0.0, 0.5) - command[0]
    wanted_velocity = wp.sign(error) * wp.min(0.12, wp.sqrt(2.0 * 0.4 * wp.abs(error)))
    velocity = command[1] + wp.clamp(wanted_velocity - command[1], -0.4 * dt, 0.4 * dt)
    displacement = velocity * dt
    if wp.abs(displacement) >= wp.abs(error):
        displacement = error
        velocity = 0.0
    command[0] = command[0] + displacement
    command[1] = velocity
    target_q[coordinate] = command[0]
    target_qd[dof] = velocity


@wp.kernel
def _measure(
    q: wp.array(dtype=wp.transform),
    qd: wp.array(dtype=wp.spatial_vector),
    constraint_force: wp.array2d(dtype=float),
    actuator_force: wp.array2d(dtype=float),
    tendon_length: wp.array2d(dtype=float),
    mass: wp.array(dtype=float),
    load: int,
    handle: int,
    handle_dof: int,
    strands: int,
    moving_mass: float,
    telemetry: wp.array(dtype=float),
):
    # No contacts, joint limits, friction-loss rows or other constraints exist.
    # A vertical outgoing strand gives dL/dz_handle=-1, hence +T on the handle.
    tension = constraint_force[0, handle_dof]
    telemetry[0] = telemetry[0] + tension / float(SUBSTEPS)
    telemetry[1] = (mass[0] + moving_mass) * 9.81 / float(strands)
    telemetry[2] = wp.transform_get_translation(q[load])[2]
    telemetry[3] = wp.transform_get_translation(q[handle])[2]
    telemetry[4] = telemetry[4] + tension / float(SUBSTEPS)
    telemetry[5] = telemetry[5] - actuator_force[0, handle_dof] / float(SUBSTEPS)
    telemetry[6] = wp.spatial_top(qd[load])[2]
    telemetry[7] = tendon_length[0, 0]


def _scene_xml(strands: int, radius: float, load_z: float) -> tuple[str, list[float], list[float], float]:
    """Place real wrapping cylinders and route one spatial tendon through them."""
    wrap_radius = radius + ROPE_RADIUS
    count = strands // 2
    lower = [4.0 * wrap_radius * (i - 0.5 * (count - 1)) for i in range(count)]
    upper = [x + 2.0 * wrap_radius for x in lower] if count else [0.0]
    load_x = 0.0 if count else -wrap_radius

    def site(name: str, x: float, z: float) -> str:
        return f'<site name="{name}" pos="{x} 0 {z}" size=".005"/>'

    def wheel(name: str, x: float, z: float, moving: bool) -> str:
        return (
            f'<body name="{name}" pos="{x} 0 {z}">'
            f'<joint name="{name}_hinge" axis="0 1 0" type="hinge"/>'
            f'<geom name="{name}_wrap" type="cylinder" size="{wrap_radius} .012" '
            f'quat=".707106781 .707106781 0 0" mass="{SHEAVE_MASS}"/>'
            + site(name + "_side", 0.0, -0.16 if moving else 0.16)
            + "</body>"
        )

    load = (
        f'<body name="load" pos="{load_x} 0 {load_z}">'
        '<joint name="load_slide" type="slide" axis="0 0 1"/>'
        f'<geom type="box" size=".1 .07 .1" mass="{DEFAULT_MASS}"/>'
        + site("load_anchor", 0.0, 0.1)
        + "".join(wheel(f"moving{i}", x, BOTTOM - load_z, True) for i, x in enumerate(lower))
        + "</body>"
    )
    fixed = "".join(wheel(f"fixed{i}", x, TOP, False) for i, x in enumerate(upper))
    handle_x = upper[-1] + wrap_radius
    handle = (
        f'<body name="handle" pos="{handle_x} 0 {HANDLE_Z}">'
        '<joint name="handle_slide" type="slide" axis="0 0 1"/>'
        f'<geom type="box" size=".025 .025 .035" mass="{HANDLE_MASS}"/>' + site("handle_end", 0.0, 0.0) + "</body>"
    )
    path = ['<site site="anchor"/>'] if count else ['<site site="load_anchor"/>']
    guides = []
    for index, x in enumerate(lower):
        upper_x = upper[index]
        path.append(f'<geom geom="moving{index}_wrap" sidesite="moving{index}_side"/>')
        guides.append(site(f"up{index}", x + wrap_radius, TOP - 0.01))
        path += [f'<site site="up{index}"/>', f'<geom geom="fixed{index}_wrap" sidesite="fixed{index}_side"/>']
        if index < count - 1:
            guides.append(site(f"down{index}", upper_x + wrap_radius, TOP - 0.01))
            path.append(f'<site site="down{index}"/>')
    if not count:
        path.append('<geom geom="fixed0_wrap" sidesite="fixed0_side"/>')
    path.append('<site site="handle_end"/>')
    anchor = site("anchor", lower[0] - wrap_radius, TOP) if count else ""
    text = (
        '<mujoco><compiler angle="radian"/>'
        '<option integrator="implicitfast"/>'
        '<default><geom contype="0" conaffinity="0"/><joint damping=".001"/></default>'
        "<worldbody>" + load + fixed + handle + anchor + "".join(guides) + "</worldbody>"
        '<tendon><spatial name="rope" springlength="-1">' + "".join(path) + "</spatial></tendon>"
        '<actuator><position name="handle_drive" joint="handle_slide" kp="2000" kv="100"/></actuator>'
        "</mujoco>"
    )
    native = mujoco.MjModel.from_xml_string(text)
    data = mujoco.MjData(native)
    mujoco.mj_forward(native, data)
    length = float(data.ten_length[0])
    # The only constraint is a unilateral upper bound on the full wrapped length.
    text = text.replace(
        'springlength="-1"',
        f'limited="true" range="0 {length}"',
    )
    return text, lower, upper, length


def build_system(strands: int) -> SimpleNamespace:
    """Use Newton's MJCF import and native MJWarp tendon constraint solver."""
    # A fixed pulley needs room between the payload and the hand grip, including
    # clearance beneath its rim at the maximum 0.5 m pull distance.
    radius = 0.115 if strands == 1 else RADIUS
    load_z = LOAD_Z - 0.10 if strands == 1 else LOAD_Z
    text, lower, upper, length = _scene_xml(strands, radius, load_z)
    builder = newton.ModelBuilder()
    newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
    builder.add_mjcf(text)
    model = builder.finalize(device="cpu")
    solver = newton.solvers.SolverMuJoCo(model, iterations=12, nconmax=1, njmax=8, disable_contacts=True)
    load = next(i for i, name in enumerate(model.body_label) if name.endswith("/load"))
    handle = next(i for i, name in enumerate(model.body_label) if name.endswith("/handle"))
    handle_joint = next(i for i, name in enumerate(model.joint_label) if name.endswith("/handle_slide"))
    handle_dof = int(model.joint_qd_start.numpy()[handle_joint])
    mj_handle_dof = int(np.flatnonzero(solver.mjc_dof_to_newton_dof.numpy()[0] == handle_dof)[0])
    moving = [i for i, name in enumerate(model.body_label) if "/moving" in name]
    fixed = [i for i, name in enumerate(model.body_label) if "/fixed" in name]
    if solver.mjw_model.ntendon != 1 or solver.mjw_model.nv != model.joint_dof_count:
        raise RuntimeError("Unexpected tendon or coordinate layout")
    return SimpleNamespace(
        model=model,
        solver=solver,
        state=model.state(),
        next_state=model.state(),
        control=model.control(),
        load=load,
        handle=handle,
        handle_dof=handle_dof,
        mj_handle_dof=mj_handle_dof,
        handle_coordinate=int(model.joint_target_q_start.numpy()[handle_joint]),
        moving=moving,
        fixed=fixed,
        lower=lower,
        upper=upper,
        strands=strands,
        radius=radius,
        wrap_radius=radius + ROPE_RADIUS,
        load_z=load_z,
        moving_mass=float(sum(model.body_mass.numpy()[moving])),
        length=length,
        reference_inertia=wp.mat33(model.body_inertia.numpy()[load]),
        command=wp.zeros(2, dtype=float, device="cpu"),
        values={
            "pull": wp.array([0.0], dtype=float, device="cpu"),
            "load": wp.array([DEFAULT_MASS], dtype=float, device="cpu"),
            "telemetry": wp.zeros(8, dtype=float, device="cpu"),
        },
        xml=text,
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
        wp.launch(
            _drive,
            1,
            [
                values["pull"],
                scene.command,
                scene.control.joint_target_q,
                scene.control.joint_target_qd,
                scene.handle_coordinate,
                scene.handle_dof,
                dt,
            ],
            device="cpu",
        )
        scene.state.clear_forces()
        s.step(scene.state, scene.next_state, scene.control, None, dt)
        for name in ("body_q", "body_qd", "joint_q", "joint_qd"):
            wp.copy(getattr(scene.state, name), getattr(scene.next_state, name))
        wp.launch(
            _measure,
            1,
            [
                scene.state.body_q,
                scene.state.body_qd,
                s.mjw_data.qfrc_constraint,
                s.mjw_data.qfrc_actuator,
                s.mjw_data.ten_length,
                values["load"],
                scene.load,
                scene.handle,
                scene.mj_handle_dof,
                scene.strands,
                scene.moving_mass,
                values["telemetry"],
            ],
            device="cpu",
        )


def metadata(scene: SimpleNamespace) -> dict:
    path = []
    for moving, fixed in zip(scene.moving, scene.fixed, strict=False):
        path += [
            {"body": moving, "lower": True, "wrapRadius": scene.wrap_radius},
            {"body": fixed, "lower": False, "wrapRadius": scene.wrap_radius},
        ]
    if not scene.moving:
        path = [{"body": scene.fixed[0], "lower": False, "wrapRadius": scene.wrap_radius}]
    return {
        "kind": "pulley",
        "title": "Pulley mechanical advantage",
        "viewer": "pulley",
        "controls": "parameters",
        "loadBody": scene.load,
        "loadSize": [0.2, 0.14, 0.2],
        "handleBody": scene.handle,
        "supportingStrands": scene.strands,
        "movingSheaveMass": scene.moving_mass,
        "handleMass": HANDLE_MASS,
        "telemetryFields": [
            "tension",
            "ideal",
            "loadHeight",
            "handleHeight",
            "verticalForce",
            "handleForce",
            "loadSpeed",
            "tendonLength",
        ],
        "cable": [],
        "ropeRadius": ROPE_RADIUS,
        "ropePath": path,
        "ropeAnchor": {"body": -1, "position": [scene.lower[0] - scene.wrap_radius, 0, TOP]}
        if scene.moving
        else {"body": scene.load, "position": [0, 0, 0.1]},
        "ropeRestLength": scene.length,
        "pulleys": [
            {
                "body": body,
                "radius": scene.radius,
                "halfWidth": 0.02,
                "grooveHalfWidth": 0.012,
                "flangeHalfWidth": 0.004,
                "flangeOffset": 0.016,
                "flangeRadius": scene.radius + 0.014,
                "moving": body in scene.moving,
            }
            for body in [*scene.moving, *scene.fixed]
        ],
        "supportStrands": [
            {
                "top": [x + side * scene.wrap_radius, 0, TOP],
                "bottom": [x + side * scene.wrap_radius, 0, BOTTOM - scene.load_z],
            }
            for x in scene.lower
            for side in (-1, 1)
        ],
        "variants": [
            {"label": f"{n}:1", "strands": n, "source": f"../pulley{'' if n == 4 else '_' + str(n)}/manifest.json"}
            for n in (1, 2, 4, 6)
        ],
        "cameraTarget": [0.08, 0.83, 0],
        "cameraRadius": 2.0,
        "cameraAzimuth": math.pi / 2,
        "cameraElevation": 0.08,
        "maxStepsPerFrame": 4,
        "renderFPS": 30,
        "source": {
            "scene": "exporters/pulley.py",
            "referenceScene": "examples/demos/newton_viewer_block_and_tackle.py",
            "solver": "Newton MJWarp",
            "ropeModel": "Massless frictionless spatial tendon; unilateral length constraint",
            "measurement": "Native qfrc_constraint on the handle; averaged over the physics substeps",
            "physicsSubsteps": SUBSTEPS,
        },
        "boundaryNote": "Massless frictionless rope · native MJWarp constraint tension · moving sheave weight included",
    }


def _snapshot(graph) -> list[tuple[int, bytes]]:
    return [(r[1], ctypes.string_at(r[1], r[2])) for r in graph._apic_capture._regions.values()]


def _restore(snapshot: list[tuple[int, bytes]]) -> None:
    for pointer, data in snapshot:
        ctypes.memmove(pointer, data, len(data))


def export_scene(strands: int, output: Path, emxx: str, native_only: bool = False) -> None:
    versions = validate_toolchain()
    scene = build_system(strands)
    with wp.ScopedCapture(device="cpu", apic=True) as capture:
        frame(scene)
    for _ in range(240):
        wp.capture_launch(capture.graph)
    with wp.ScopedCapture(device="cpu", apic=True) as capture:
        frame(scene)
    name = "pulley" if strands == 4 else f"pulley_{strands}"
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
    bindings = {k: getattr(scene.state, k) for k in ("body_q", "body_qd", "joint_q", "joint_qd")}
    bindings["telemetry"] = scene.values["telemetry"]
    bindings["tendon_length"] = scene.solver.mjw_data.ten_length
    export_graph(
        capture.graph,
        model=display,
        inputs={**bindings, "pull": scene.values["pull"], "load": scene.values["load"]},
        outputs=bindings,
        output=bundle,
        timestep=FRAME_DT,
        parameters=(
            Parameter("pull", 0, "Pull distance [m]", 0.0, 0.5, 0.005),
            Parameter("load", 0, "Payload [kg]", 0.5, 5.0, 0.1),
        ),
        persistent=("load",),
    )
    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["isaacLabDemo"] = {**metadata(scene), "versions": versions}
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    (bundle / "scene.xml").write_text(scene.xml + "\n")
    snapshot = _snapshot(capture.graph)
    results = []
    for mass in (0.5, DEFAULT_MASS, 5.0):
        _restore(snapshot)
        scene.values["load"].assign([mass])
        for _ in range(240):
            wp.capture_launch(capture.graph)
        baseline = float(scene.values["telemetry"].numpy()[2])
        rows, samples = [], []
        start = time.perf_counter()
        for step in range(360):
            if step == 60:
                scene.values["pull"].assign([0.3])
            wp.capture_launch(capture.graph)
            rows.append(scene.state.body_q.numpy().copy())
            samples.append(scene.values["telemetry"].numpy().copy())
        samples = np.asarray(samples)
        force = float(samples[-60:, 0].mean())
        ideal = float(samples[-1, 1])
        travel = float(samples[-1, 2] - baseline)
        max_speed = float(np.max(np.abs(np.diff(samples[-60:, 2]) / FRAME_DT)))
        if not np.isfinite(rows).all() or not np.isfinite(samples).all():
            raise RuntimeError(f"Non-finite {strands}:1 pulley trajectory")
        settled_span = float(np.ptp(samples[-60:, 2]))
        if abs(force / ideal - 1.0) > 0.02 or abs(travel - 0.3 / strands) > 0.002 or settled_span > 0.001:
            raise RuntimeError(
                f"Inaccurate {strands}:1 pulley at {mass} kg: force={force}/{ideal}, "
                f"rise={travel}/{0.3 / strands}, speed={max_speed}, span={settled_span}"
            )
        result = {
            "payload": mass,
            "tension": force,
            "ideal": ideal,
            "rise": travel,
            "idealRise": 0.3 / strands,
            "maxSettledSpeed": max_speed,
            "settledPositionSpan": settled_span,
            "millisecondsPerFrame": 1000 * (time.perf_counter() - start) / len(samples),
        }
        print("VALIDATED", strands, result, flush=True)
        results.append(result)
        if mass == DEFAULT_MASS:
            np.savez(bundle / "reference.npz", body_q=rows, telemetry=samples)
    (bundle / "validation.json").write_text(json.dumps(results, indent=2) + "\n")
    _restore(snapshot)
    if not native_only:
        package_bundle(bundle, output / (name + "-web"), emxx)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--strands", type=int, choices=(1, 2, 4, 6), nargs="+", default=[1, 2, 4, 6])
    parser.add_argument("--emxx", default="em++")
    parser.add_argument("--native-only", action="store_true")
    args = parser.parse_args()
    for strands in args.strands:
        export_scene(strands, args.output, args.emxx, args.native_only)


if __name__ == "__main__":
    main()
