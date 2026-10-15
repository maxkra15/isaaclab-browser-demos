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
| Franka soft-body policy | Pretrained policy controls an MJWarp arm coupled to a VBD tetrahedral beam; stiffness, target height and bounded mouse pulling | Native `Isaac-Lift-Soft-Franka` observations, relative joint actions, actuators and contacts; published Newton checkpoint |
| Franka cloth policy | Pretrained policy grasps and lifts a VBD sheet through hand/finger contacts; bending, target height and bounded mouse pulling | Native `Isaac-Lift-Cloth-Franka` with a compact 5 × 5 particle grid; published Newton checkpoint; 20 observed nodes as in training |
| Franka cube stacking | Native contrib scene; contact grasp, placement X/Y and early release | `IsaacContrib-Stack-Cube-Franka`, MuJoCo asset variant, one-world clone plan, unused frame sensor/observations omitted, Newton IK trajectory |

The beam and cloth demos run their published RSL-RL actors in the browser using
the existing MLP evaluator. The captured graph builds and normalizes all 93
observations, applies the eight actions and advances native physics. Both grasp
through contacts; the cloth has no attachment. Cube stacking still uses scripted IK.
Visual bench legs are decorative; the native support collider is unchanged.
Mouse pulling adds a spring force capped at 2 N; it does not edit particle poses.
Open gripper temporarily overrides the learned gripper target; Policy grip returns
control to the actor. Changing the material tests policy generalization, not
performance on its original training distribution.

Physics stays at 120 Hz with two substeps for deformables. Each captured control
step includes four physics steps and one 30 Hz policy decision. Relative arm targets
are reapplied at each native action boundary, preserving the task's update schedule.
The five-second training horizon repeats. Rendering runs at up to 30 Hz; playback
speed still depends on the device.
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
checkpoint_root=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/IsaacLab/PretrainedCheckpoints/rsl_rl
curl -fL "$checkpoint_root/Isaac-Lift-Soft-Franka_newtonmjwarpvbdproxy_none_rsl_rl.pt" -o /tmp/vbd-soft.pt
curl -fL "$checkpoint_root/Isaac-Lift-Cloth-Franka_newtonmjwarpvbdproxy_none_rsl_rl.pt" -o /tmp/vbd-cloth.pt
uv run --no-sync python "$demo_repo/exporters/scenes.py" cable_bending --output "$build_dir" --emxx "$emxx"
uv run --no-sync --with 'pytetwild[all]>=0.3,<0.4' python "$demo_repo/exporters/deformable.py" soft --checkpoint /tmp/vbd-soft.pt --output "$build_dir" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/deformable.py" cloth --checkpoint /tmp/vbd-cloth.pt --output "$build_dir" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/stacking.py" --output "$build_dir" --emxx "$emxx"
```

Copy the beam/cloth `<name>-web/` outputs to `site/lab/browser_demos/policy-v1/<name>/`.
Keep earlier bundles for cached previews. Other scenes use `site/lab/browser_demos/<name>/`.
Coupled control copies use a build-local Warp gather adapter because Warp 1.17
CPU graph capture cannot record the original indexed `wp.copy`. It implements
the same gather and restores Newton's helper after capture.

Checkpoint hashes and layer widths are pinned in the exporter. Exports compare
the observation/action adapters to the actual task managers through a native
policy rollout. Float32 operation order and relative-target update boundaries
match the task. They fail if the default policy cannot lift, positions/velocities
become non-finite, or CPU capture differs from eager playback by more than 1 μm.
Native trajectory references cover approach, grasp, lift and hold. `validation.json`
records hashes of the exact browser files associated with `reference.npz`;
keep those build artifacts together when comparing native and WASM execution.
Tetrahedralization is not guaranteed to reproduce an identical mesh on rebuild.
Browser checks verify the actor output, observation formula on the browser's own
state, and learned lifting/tracking with closed-loop inference. Contact trajectories
can diverge between native CPU and WASM due to floating-point differences;
full per-particle equality after grasp is not a suitable cross-platform gate.
In a browser, also verify pause/reset, slider persistence, early release,
mouse picking and blur/cancel cleanup, material endpoints, and playback speed.

## Other contrib candidates

| Candidate | Remaining work before a useful browser demo |
| --- | --- |
| Shoelace | Dual-arm ADMM coupling, contact/sensor bookkeeping and collision preparation need their own CPU/export and performance validation |
| Franka pouring / UR10 particle pushing | Sparse MPM grid operations and collider assets need export support and a measured CPU budget; particle count alone does not bound grid work |
| Factory / assembly | Backend-specific contacts and task controllers need a separate port; no browser parity has been established |

These are source reviews, not claims that those tasks currently export. The
small stack and native VBD policy scenes are the first validated additions here.
