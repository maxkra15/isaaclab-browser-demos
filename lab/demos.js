// Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

import { demos as releasedDemos } from '../v6/demos.js';

const existing = releasedDemos.filter((demo) => !['ant', 'cable_bending'].includes(demo.id)).map((demo) => ({
  ...demo,
  assetRoot: demo.id === 'dominoes' ? '../v6/browser_demos' : demo.assetRoot,
}));

export const demos = [
  { id: 'cable_bending', title: 'VBD cable bending', group: 'Deformable lab', description: 'A shorter clamped cable that settles. Tune stiffness and the tip load.', assetRoot: './browser_demos' },
  ...existing,
];
