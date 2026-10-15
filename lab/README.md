<!--
Copyright (c) 2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
All rights reserved.

SPDX-License-Identifier: BSD-3-Clause
-->

# Interactive lab

Personal experimental preview: <https://maxkra15.github.io/isaaclab-browser-demos/lab/>.
It reuses the released gallery and assets. Ant is hidden here; versioned releases
remain immutable. Only the selected simulation is loaded.

| Scene | Physics and interaction | Preparation for this preview |
| --- | --- | --- |
| VBD cable bending | Clamped rod with live bending stiffness and tip load | Ten segments, 0.45 m long, 24 VBD iterations and eight substeps; actual rigid root clamp |
| Franka soft-body grasp | MJWarp arm coupled to a VBD tetrahedral beam; stiffness, lift height, release and bounded mouse pulling | Native `Isaac-Lift-Soft-Franka`, original coarse refinement, deterministic initial joints, gravity compensation, Newton IK trajectory |
| Franka cloth clip | MJWarp arm and VBD sheet; bending, lift height, release and bounded mouse pulling | Native `Isaac-Lift-Cloth-Franka` scene with a 5 × 5 particle grid and three vertices prescribed relative to the hand; hand/cloth contacts and reaction feedback omitted around the clip; support contacts remain |
| Franka cube stacking | Native contrib scene; contact grasp, placement X/Y and early release | `IsaacContrib-Stack-Cube-Franka`, MuJoCo asset variant, one-world clone plan, unused frame sensor/observations omitted, Newton IK trajectory |

The Franka demos use scripted IK, not trained policies or recorded animation.
All bodies and deformables advance in the captured Newton solvers. The cloth
clip is an explicit one-way boundary condition, not a validated free grasp.
Visual bench legs are decorative; the native support collider is unchanged.
Mouse pulling adds a spring force capped at 2 N; it does not edit particle poses.

Physics stays at 120 Hz with two substeps for deformables. Their renderer runs
at 30 Hz to leave CPU time for physics. Playback speed still depends on the device.
The beam uses the native coarse tetrahedralization; more vertices materially
increase the browser CPU cost. Published meshes are fixed in each bundle.

## Rebuild

From the Isaac Lab checkout used for these captures (`380ddac0`), use the
Newton 1.7.0.dev0 / Warp 1.17.0 / MuJoCo Warp 3.14.0 environment.
`scenes.validate_toolchain()` checks these versions and Newton Web revision
`b0795fbe6b46e08a1fea2425699421415ca4cdf1`. Use Emscripten 5.0.3.
Follow the [export guide](../exporters/README.md) for compiler installation.

```bash
export PYTHONPATH=/path/to/newton-web/src:/path/to/IsaacLab/source/isaaclab_teleop
demo_repo=/path/to/isaaclab-browser-demos
build_dir=/tmp/isaaclab-browser-lab
emxx=/path/to/emsdk/upstream/emscripten/em++
uv run --no-sync python "$demo_repo/exporters/scenes.py" cable_bending --output "$build_dir" --emxx "$emxx"
uv run --no-sync --with 'pytetwild[all]>=0.3,<0.4' python "$demo_repo/exporters/deformable.py" soft --output "$build_dir" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/deformable.py" cloth --output "$build_dir" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/stacking.py" --output "$build_dir" --emxx "$emxx"
```

Copy each resulting `<name>-web/` to `site/lab/browser_demos/<name>/`.
Coupled control copies use a build-local Warp gather adapter because Warp 1.17
CPU graph capture cannot record the original indexed `wp.copy`. It implements
the same gather and restores Newton's helper after capture.

Exports fail if positions or velocities become non-finite, the default beam or
cloth lift is not held through 14 seconds, or the default cube stack fails.
Native sparse checkpoints cover approach, grasp, lift and hold. `validation.json`
records hashes of the exact browser files associated with `reference.npz`;
keep those build artifacts together when comparing native and WASM execution.
Tetrahedralization is not guaranteed to reproduce an identical mesh on rebuild.
In a browser, also verify pause/reset, slider persistence, early release,
mouse picking and blur/cancel cleanup, material endpoints, and playback speed.

## Other contrib candidates

| Candidate | Remaining work before a useful browser demo |
| --- | --- |
| Shoelace | Dual-arm ADMM coupling, contact/sensor bookkeeping and collision preparation need their own CPU/export and performance validation |
| Franka pouring / UR10 particle pushing | Sparse MPM grid operations and collider assets need export support and a measured CPU budget; particle count alone does not bound grid work |
| Factory / assembly | Backend-specific contacts and task controllers need a separate port; no browser parity has been established |

These are source reviews, not claims that those tasks currently export. The
small stack and native VBD lift scenes are the first validated additions here.
