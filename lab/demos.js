// Copyright (c) 2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

const response = await fetch(new URL('./catalog.json', import.meta.url));
if (!response.ok) throw new Error(`Could not load demo catalog (${response.status})`);
const catalog = await response.json();
if (!Array.isArray(catalog.demos)) throw new Error('The demo catalog must contain a demos array.');

export const activities = catalog.activities || [
  { id: 'tuning', label: 'Tuning', description: 'Change material, solver or actuator settings and compare the response.' },
  { id: 'policy', label: 'Policy inference', description: 'Steer pretrained policies, move their targets or test their response to a disturbance.' },
  { id: 'scripted', label: 'Scripted scenes', description: 'Explore contact interactions with a programmed controller.' },
];

export const physics = catalog.physics || [
  { id: 'rigid', label: 'Rigid', description: 'Rigid bodies and robots' },
  { id: 'vbd', label: 'VBD', description: 'Soft bodies, cloth and cables' },
  { id: 'mpm', label: 'MPM', description: 'Particle materials' },
];

export const demos = catalog.demos.map((demo) => ({ assetRoot: './demos', ...demo }));
