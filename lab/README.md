<!--
Copyright (c) 2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
All rights reserved.

SPDX-License-Identifier: BSD-3-Clause
-->

# Interactive lab

Demos with a default ground plane use the current official Isaac Lab checker
textures from `browser_demos/shared/`. MicroDuck uses its authored USD warehouse
floor and materials. The MPM tub and Franka tables retain their original surfaces
and ground heights.

Personal experimental preview: <https://maxkra15.github.io/isaaclab-browser-demos/lab/>.
It reuses the released gallery and assets. Ant is hidden here; versioned releases
remain immutable. Only the selected simulation is loaded.

The gallery filters by activity (tuning, policy inference or scripted scenes) and
physics (rigid bodies, VBD or MPM). Categories live in `demos.js`; coupled scenes
can have multiple physics tags. Filters combine, update the URL and survive the
return from a demo. For example, `?activity=policy&physics=vbd` selects the VBD policies.

## Embed the gallery in documentation

The same page serves a lightweight gallery in an iframe. Use
`?embed=1&demos=all`, or pass a comma-separated subset such as
`?embed=1&demos=stiffness,cloth_bending,cable_bending,pulley`.
The catalog, posters, categories and controls stay in this app. Documentation
only needs its heading, the hosted URL, a fallback link and the selected IDs.
No simulation module, policy or WASM is loaded until a reader selects a card.
A single player opens below the filters; switching or closing disposes the
previous simulation. Filtering out the active demo also closes its player.
Modified clicks and **Open full view** open the standalone demo.

`demo=<id>` highlights and focuses a matching card in embed mode without
starting physics. The standalone page keeps its existing direct-play URLs.
Unknown subset IDs display an error instead of silently changing the selection.

The optional parent bridge uses messages with
`namespace: "isaaclab-demos"`. The child validates the sender against
`window.parent` and the HTTP(S) origin in `document.referrer`; if the referrer
is unavailable, the docs host can supply its origin as `parentOrigin` in the
iframe URL.

- Child → parent: `{namespace, type: "ready"}` after the gallery shell;
  `{namespace, type: "resize", height}` for natural content height (at most
  12,000 px); `{namespace, type: "error", message}` if the gallery or a
  selected simulation fails.
- Parent → child: `{namespace, type: "theme", theme: "light" | "dark"}`;
  `{namespace, type: "select", demo: "<id>"}` to reveal, highlight and focus a
  card without starting it.

The parent must validate both the child's origin and `event.source` before
using these messages. Keep the iframe visible while it starts: simulation
loading is triggered by the widget's visibility observer. Child height comes
from natural content rather than `100vh`, so resizing the iframe does not
create a size feedback loop. Gallery mode and the bridge require no external
metadata API or network access during a Sphinx build.

| Scene | Physics and interaction | Preparation for this preview |
| --- | --- | --- |
| VBD cable bending | Clamped rod with live bending stiffness and tip load | Ten segments, 0.45 m long, 24 VBD iterations and eight substeps; actual rigid root clamp |
| Pulley workshop | Try a VBD cable over one, two or four wheels or a massless 1:1, 2:1, 4:1 or 6:1 arrangement; drag the handle and compare forces | Native VBD rod contacts or MJWarp spatial tendons; actual handle reactions and a separate ideal static reference |
| Franka soft-body policy | Pretrained policy controls an MJWarp arm coupled to a VBD tetrahedral beam; stiffness, target height and bounded mouse pulling | Native `Isaac-Lift-Soft-Franka` observations, relative joint actions, actuators and contacts; published Newton checkpoint |
| Franka cloth policy | Pretrained policy grasps and lifts a VBD sheet through hand/finger contacts; bending, target height and bounded mouse pulling | Native `Isaac-Lift-Cloth-Franka` with a compact 5 × 5 particle grid; published Newton checkpoint; 20 observed nodes as in training |
| Franka cable policy | Pretrained policy grasps and lifts a 12-segment cable; change the height or take over with keyboard IK | Native `Isaac-Lift-Cable-Franka` with its published 69-input Newton policy, four substeps, VBD/MJWarp coupling and native capsule visuals |
| Franka cube stacking | Scripted contact grasp and stacking, or manual end-effector control | Contrib Franka scene with aligned 50 mm, 20 g procedural cubes and a matching tabletop collider; native Newton IK sequence |
| MPM material response | Sand, snow, clay and water references falling into a tub | 2,744 particles, 14% smaller radii than the initial preview, at the same block volume and density; finite contact clearance and checker ground beneath the tub base |

The beam and cloth demos run their published RSL-RL actors in the browser using
the existing MLP evaluator. The captured graph builds and normalizes all 93
observations, applies the eight actions and advances native physics. Both grasp
through contacts; the cloth has no attachment. Cube stacking still uses scripted IK.
The visible tabletop matches the native support collider; bench legs are decorative.
Mouse pulling adds a spring force capped at 2 N; it does not edit particle poses.
Changing the material tests policy generalization, not performance on its
original training distribution. Material settings are available in either mode.

For the beam and cloth, physics stays at 120 Hz with two substeps for deformables. Each captured control
step includes four physics steps and one 30 Hz policy decision. Relative arm targets
are reapplied at each native action boundary, preserving the task's update schedule.
Policy mode repeats the five-second training horizon. Manual mode continues until
reset. Rendering runs at up to 30 Hz; playback speed still depends on the device.
The beam uses the native coarse tetrahedralization; more vertices materially
increase the browser CPU cost. Published meshes are fixed in each bundle.

The cable policy uses its original 12-segment rod and 69 normalized observations,
including every segment position. Its published actor has widths
69–256–128–64–8. The graph keeps the native four physics steps per policy update
and four substeps per physics step. Capsules are packed from the native collider
transforms because the USD cable is authored as a curve, rather than a mesh.
Native and browser checks cover a real contact grasp, lift and hold, Manual
takeover, and handoff back to Policy. The checkpoint and source URL are pinned
in the manifest. This scene does not replace the cable with a particle mesh or
attach it to the gripper.

## Pulley workshop

One gallery entry offers two rope models. **Massless rope** uses a Newton/MJWarp
spatial tendon in 1:1, 2:1, 4:1 and 6:1 arrangements. **VBD cable** uses a finite-mass
rod contacting one fixed wheel, a fixed and moving wheel in 2:1, or two fixed and
two moving wheels in 4:1. Each model and topology is a separate
captured scene: selecting it disposes the current simulation and loads only the
selected bundle. Payload is preserved; the scene and force history reset.
Link directly to the VBD model with `?demo=pulley&rope=vbd`; add `&wheels=2`
for the two-wheel scene, or `&wheels=4` for four wheels. Changing rope models preserves the wheel count when
supported. Available models and wheel counts live in `demos.js`.

The **Payload** slider updates mass and inertia on the next physics step without
resetting the simulation or force history, so the plot shows the load change.

Drag the yellow handle or use **Pull down**. The Pull label highlights while
dragging. A bolted clamp marks the native cable attachment; it follows the payload
in 1:1. The single fixed wheel changes the pull direction without reducing ideal
static tension. Its geometry separates the payload and handle and leaves
clearance below the wheel over the full pull range.

The massless model has frictionless wrapping and a position-driven handle. Its
sensor reads the native tendon constraint reaction. The ideal reference is the
combined weight of the payload and moving sheaves divided by the number of
supporting strands. The displayed line follows the engine's wrapping geometry.

The VBD model renders the native capsule shapes at their simulated body poses,
including bending and wheel contacts. The handle is a prescribed kinematic test
fixture; the cable, load and wheel remain dynamically simulated. The force sensor
reads the signed vertical BALL-attachment reaction using the solver's own force
evaluation. The ideal line assumes a massless, frictionless rope; actual cable
mass, elasticity, friction and motion can produce differences. The vertical load
guide uses the native example's viscous damping. All three VBD arrangements expose
0.5–5 kg, with live mass changes that preserve the cable state and force history.
Force estimates remain affected by finite solver convergence. It is a compact
adaptation of `examples/demos/newton_viewer_block_and_tackle.py`, rather than an
export of the native example's full four-strand scene.

The two-wheel VBD scene has a native revolute sheave attached above the guided
load. Its ideal static force includes that sheave's actual mass and divides by
two: pulling the handle 50 cm should lift the load about 25 cm. Four wheels use
two native moving sheaves, include both masses and divide by four: a 50 cm pull
should lift the load about 12.5 cm. The four-wheel frame is shorter to reduce
cable length and browser cost while preserving clearance at full pull. The
visible carrier is massless scenery connecting the native axles to the load.

Both plots average the native physics substeps without further smoothing and use
simulation time. Neither computes the measured line from mechanical advantage.
Wheel rotation comes from the simulated hinges. The gantry is visual scenery.

## Control the Franka

Reach, lift, drawer opening, beam, cloth and cable lifting offer **Policy / Manual**.
Cube stacking offers **Scripted / Manual**. The selected mode owns the arm and gripper;
there are no separate policy-grip or release overrides. The reach task has no gripper action,
so it shows movement controls only.

In Manual, focus the viewport and hold **WASD / arrow keys** to move relative to the
camera, **R/F** to raise/lower, **Q/E** to turn, and **G** to open/close the gripper.
**Shift** gives finer movement; **Esc** stops. Releasing movement keys or touch buttons
stops the requested motion. Small screens show a movement pad and gripper button.
Pause and reset stay in the viewport. Reset keeps the selected mode; returning to
Policy hands over the current scene, while selecting Scripted restarts its sequence.

The shared browser IK controller uses exported native joint frames, axes, limits,
mass/COM and actuator stiffness. It writes the existing action inputs, with gravity
compensation and velocity damping; physics and contact handling stay in the same WASM
simulation. The blue marker is the manual target. Joint limits, contacts and actuator
dynamics constrain the actual motion. No joint, rigid-body or particle poses are edited.

The legacy drawer bundle has an overlapping reset pose. Before displaying it, the
controller performs a short IK retreat through 90 steps of the original physics.
Reset performs the same initialization. Its original absolute-action offsets are
retained; the current registered task's EMA controller is a different action profile.

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
curl -fL "$checkpoint_root/Isaac-Lift-Cable-Franka_newtonmjwarpvbdproxy_none_rsl_rl.pt" -o /tmp/vbd-cable.pt
uv run --no-sync python "$demo_repo/exporters/scenes.py" cable_bending --output "$build_dir" --emxx "$emxx"
uv run --no-sync --with 'pytetwild[all]>=0.3,<0.4' python "$demo_repo/exporters/deformable.py" soft --checkpoint /tmp/vbd-soft.pt --output "$build_dir" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/deformable.py" cloth --checkpoint /tmp/vbd-cloth.pt --output "$build_dir" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/deformable.py" cable --checkpoint /tmp/vbd-cable.pt --output "$build_dir" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/pulley.py" --output "$build_dir/pulleys" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/pulley_vbd.py" --output "$build_dir/pulley-vbd" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/pulley_vbd.py" --strands 2 --output "$build_dir/pulley-vbd" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/pulley_vbd.py" --strands 4 --output "$build_dir/pulley-vbd" --emxx "$emxx"
uv run --no-sync python "$demo_repo/exporters/stacking.py" --output "$build_dir" --emxx "$emxx"
for kind in reach lift drawer soft cloth stack; do
    uv run --no-sync --with 'pytetwild[all]>=0.3,<0.4' python "$demo_repo/exporters/franka_control.py" \
        "$kind" --output "$build_dir/control/franka_$kind.json"
done
```

Copy the beam/cloth `<name>-web/` outputs to `site/lab/browser_demos/policy-v1/<name>/`,
stacking to `site/lab/browser_demos/teleop-v1/franka_stack/`, and the six control JSON
files to `site/lab/browser_demos/teleop-v1/`. Fresh captures emit `manual-control.json`
beside their build data; use that metadata from the same capture when publishing new
physics. The standalone command above prepares the existing preview profiles.
Copy `franka_cable-web/` into `site/lab/browser_demos/cable-policy-v1/franka_cable/`
and its captured `manual-control.json` into the same asset root as `franka_cable.json`.
Copy the four pulley web bundles into `site/lab/browser_demos/pulley-v1/`, preserving
the names `pulley_1`, `pulley_2`, `pulley` and `pulley_6`.
Copy `pulley_vbd-web/` into `site/lab/browser_demos/pulley-vbd-v1/pulley_vbd/`.
Copy `pulley_vbd_2-web/` into `site/lab/browser_demos/pulley-vbd-v1/pulley_vbd_2/`.
Copy `pulley_vbd_4-web/` into `site/lab/browser_demos/pulley-vbd-v1/pulley_vbd_4/`.
Use a new asset path and update the module and gallery-link revisions when replacing
a bundle, so cached files do not mix with a new simulation. Remove unused bundles
from the mutable preview; retain published version directories. Other scenes use
`site/lab/browser_demos/<name>/`.
MPM retains the pinned Newton 1.6.0 / MuJoCo Warp 3.12.0 export toolchain in the
export guide. Rebuild it separately and publish `mpm-web/` under
`site/lab/browser_demos/grains-v1/mpm/`; the lab viewer places its checker ground
beneath the tub while the inner floor and collision boundary stay at z=0.
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
In a browser, verify controller handoff, manual pose holding and movement, keyboard
and touch release, gripper contacts, pause/reset, slider persistence, mouse picking,
blur/cancel cleanup, material endpoints, and playback speed. Compare FK with native
reference poses along motion, not only at reset. Stacking must keep the blue cube
at 25 mm and leave the red cube 50 mm above it, within contact-solver tolerance.

## Other contrib candidates

| Candidate | Remaining work before a useful browser demo |
| --- | --- |
| Shoelace | Dual-arm ADMM coupling, contact/sensor bookkeeping and collision preparation need their own CPU/export and performance validation |
| Franka pouring / UR10 particle pushing | Sparse MPM grid operations and collider assets need export support and a measured CPU budget; particle count alone does not bound grid work |
| Factory / assembly | Backend-specific contacts and task controllers need a separate port; no browser parity has been established |

These are source reviews, not claims that those tasks currently export. The
small stack and native VBD policy scenes are the first validated additions here.
