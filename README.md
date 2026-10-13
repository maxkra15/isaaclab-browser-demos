<!--
Copyright (c) 2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
All rights reserved.

SPDX-License-Identifier: BSD-3-Clause
-->

# Isaac Lab browser demos

Static deployment mirror for the interactive Newton demos linked from Isaac Lab
documentation. Physics and policy inference run locally in the browser.

[Open the public gallery](https://maxkra15.github.io/isaaclab-browser-demos/v1/).

The NVIDIA-maintained source project is
[Isaac Lab Browser Demos](https://gitlab-master.nvidia.com/omniverse/isaac/isaaclab-browser-demos/-/tree/pages).
The public [reference source](https://github.com/isaac-sim/IsaacLab/pull/7974)
records the original exporters and scene adaptations.

Published version directories are retained unchanged. Synchronize the reviewed
`site/` output from the source project when adding a new version. GitHub Pages
publishes this repository's `pages` branch with no build dependencies.

## Publish a reviewed release

From this mirror's checkout, copy the static output from the source checkout:

```bash
cp -a /path/to/isaaclab-browser-demos/site/. .
git diff --check
git status --short
```

Review and commit the new version, then push to `pages`. Preserve existing version
directories and the third-party license notices. No compiler or simulator is
required to publish an already reviewed release.
