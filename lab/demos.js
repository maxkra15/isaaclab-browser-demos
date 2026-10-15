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
  { id: 'franka_soft', title: 'Franka soft-body grasp', group: 'Deformable lab', description: 'Lift a flexible beam with native coupled physics. Change stiffness and lift height, or drag the beam.', assetRoot: './browser_demos' },
  { id: 'franka_cloth', title: 'Franka cloth clip', group: 'Deformable lab', description: 'Lift and drape a VBD cloth with a corner clip. Tune bending, change lift height, or pull the sheet.', assetRoot: './browser_demos' },
  { id: 'franka_stack', title: 'Franka cube stacking', group: 'Manipulation', description: 'Pick and stack with real contacts. Change the placement target or release the cube early.', assetRoot: './browser_demos' },
  ...existing,
];
