// Copyright (c) 2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

import * as THREE from '../vendor/three.module.min.js';
import { OrbitCamera } from './browser-orbit.js';
import { addIsaacLabGround } from './browser-ground.js';
import { PulleyPlot } from './pulley-plot.js';
import { demos } from '../demos.js';

export class PulleyViewer {
  static async load(canvas, simulation) {
    const viewer = new PulleyViewer(canvas, simulation);
    try {
      viewer.disposeGround = await addIsaacLabGround(viewer.scene);
      return viewer;
    } catch (error) {
      viewer.dispose();
      throw error;
    }
  }

  constructor(canvas, simulation) {
    this.canvas = canvas;
    this.simulation = simulation;
    this.demo = simulation.manifest.isaacLabDemo;
    if (this.demo.ropeModel === 'newton' && !this.demo.ropePointsBinding) {
      throw new Error('Newton tendon visualization requires its native accepted rope path');
    }
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'low-power' });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = .95;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color('#e8eef1');
    this.scene.fog = new THREE.Fog('#e8eef1', 7, 16);
    this.camera = new THREE.PerspectiveCamera(36, 1, .03, 30);
    this.target = new THREE.Vector3(0, .95, 0);
    this.orbit = new OrbitCamera(canvas, this.camera,
      Math.PI / 2 - .12, .13, 3.7);
    this.scene.add(new THREE.HemisphereLight('#f7fbff', '#a8b6bd', 1.35));
    const key = new THREE.DirectionalLight('#fff8ec', 2.1);
    key.position.set(-2.5, 4, 3);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    key.shadow.camera.left = key.shadow.camera.bottom = -2;
    key.shadow.camera.right = key.shadow.camera.top = 2;
    key.shadow.normalBias = .002;
    key.shadow.bias = -.0001;
    this.scene.add(key);
    const fill = new THREE.DirectionalLight('#d6e9ff', .55);
    fill.position.set(2, 2, -2);
    this.scene.add(fill);
    this.stage = new THREE.Group();
    this.stage.rotation.x = -Math.PI / 2;
    this.scene.add(this.stage);
    this.geometries = [];
    this.materials = [];
    this.labels = [];
    this.bodies = Array.from({ length: simulation.binding('body_q').length / 7 }, () => {
      const body = new THREE.Group();
      this.stage.add(body);
      return body;
    });
    const material = (color, roughness = .5, metalness = .2) => {
      const result = new THREE.MeshStandardMaterial({ color, roughness, metalness });
      this.materials.push(result);
      return result;
    };
    const frame = material('#354955', .48, .35);
    const fixed = material('#526a79', .38, .4);
    const moving = material('#138fa1', .35, .3);
    const silver = material('#bdcbd1', .32, .65);
    const rubber = material('#23333e', .82, 0);
    const rope = material('#e7ac32', .6, 0);
    const load = material('#ce704e', .48, .15);
    const loadInset = material('#a44f35', .6, .1);
    const grip = material('#eebd45', .42, .15);
    this.handleMeshes = [];
    const add = (geometry, surface, parent, position = [0, 0, 0]) => {
      this.geometries.push(geometry);
      const mesh = new THREE.Mesh(geometry, surface);
      mesh.position.fromArray(position);
      mesh.castShadow = mesh.receiveShadow = true;
      parent.add(mesh);
      return mesh;
    };
    const poses = simulation.binding('body_q');
    const gantryHeight = Math.max(...this.demo.pulleys.map((wheel) => poses[wheel.body * 7 + 2])) + .25;
    const left = Math.min(poses[this.demo.loadBody * 7] - this.demo.loadSize[0] / 2,
      ...this.demo.pulleys.map((wheel) => poses[wheel.body * 7] - wheel.flangeRadius)) - .20;
    const right = Math.max(poses[this.demo.handleBody * 7] + .073,
      ...this.demo.pulleys.map((wheel) => poses[wheel.body * 7] + wheel.flangeRadius)) + .20;
    const center = (left + right) / 2;
    this.target.x = center;
    this.target.y = gantryHeight / 2;
    this.frameWidth = right - left + .4;
    this.frameHeight = gantryHeight;
    this.fitDistance = gantryHeight / .52;
    this.orbit.distance = this.fitDistance;
    for (const x of [left, right]) {
      add(new THREE.BoxGeometry(.055, .07, gantryHeight), frame, this.stage, [x, .12, gantryHeight / 2]);
      add(new THREE.BoxGeometry(.22, .38, .035), frame, this.stage, [x, .08, .018]);
      for (const y of [-.065, .225]) {
        add(new THREE.BoxGeometry(.18, .055, .009), rubber, this.stage, [x, y, .005]);
      }
      add(new THREE.BoxGeometry(.013, .002, gantryHeight - .08), silver, this.stage,
        [x, .084, gantryHeight / 2]);
      for (const z of [.08, gantryHeight - .055]) {
        add(new THREE.CylinderGeometry(.009, .009, .006, 6), silver, this.stage, [x, .081, z]);
      }
    }
    add(new THREE.BoxGeometry(right - left + .11, .09, .08), frame, this.stage, [center, .12, gantryHeight]);
    for (const wheel of this.demo.pulleys) {
      const body = this.bodies[wheel.body];
      const surface = wheel.moving ? moving : fixed;
      add(new THREE.CylinderGeometry(wheel.radius, wheel.radius, 2 * wheel.grooveHalfWidth, 48), surface, body);
      for (const side of [-1, 1]) {
        add(new THREE.CylinderGeometry(wheel.flangeRadius, wheel.flangeRadius, 2 * wheel.flangeHalfWidth, 48), surface, body,
          [0, side * wheel.flangeOffset, 0]);
        const rim = add(new THREE.TorusGeometry(wheel.flangeRadius - .004, .003, 6, 48), silver, body,
          [0, side * (wheel.flangeOffset + wheel.flangeHalfWidth + .001), 0]);
        rim.rotation.x = Math.PI / 2;
        rim.castShadow = false;
        add(new THREE.CylinderGeometry(.026, .026, .009, 24), rubber, body,
          [0, side * (wheel.halfWidth + .007), 0]);
        add(new THREE.CylinderGeometry(.016, .016, .014, 20), silver, body,
          [0, side * (wheel.halfWidth + .014), 0]);
        add(new THREE.CylinderGeometry(.008, .008, .003, 6), frame, body,
          [0, side * (wheel.halfWidth + .023), 0]);
      }
      for (const angle of [0, Math.PI * 2 / 3, Math.PI * 4 / 3]) {
        const spoke = add(new THREE.BoxGeometry(wheel.radius * .62, .003, .007), silver, body,
          [Math.cos(angle) * wheel.radius * .48, -wheel.halfWidth - .005, Math.sin(angle) * wheel.radius * .48]);
        spoke.rotation.y = -angle;
        spoke.castShadow = false;
      }
      if (!wheel.moving) {
        add(new THREE.BoxGeometry(.03, .05, gantryHeight - poses[wheel.body * 7 + 2]), frame, this.stage,
          [poses[wheel.body * 7], .085, (gantryHeight + poses[wheel.body * 7 + 2]) / 2]);
      }
    }
    if (this.demo.supportingStrands > 1) {
      const anchor = this.demo.ropeAnchor.position;
      add(new THREE.BoxGeometry(.018, .03, gantryHeight - anchor[2]), silver, this.stage,
        [anchor[0], 0, (gantryHeight + anchor[2]) / 2]);
    }
    const anchor = this.demo.ropeAnchor;
    this.anchorClamp = new THREE.Group();
    this.anchorClamp.name = 'fixed-cable-end';
    this.anchorClamp.position.fromArray(anchor.position);
    (anchor.body < 0 ? this.stage : this.bodies[anchor.body]).add(this.anchorClamp);
    for (const side of [-1, 1]) {
      add(new THREE.BoxGeometry(.065, .014, .065), fixed, this.anchorClamp, [0, side * .014, .025]);
      for (const x of [-.022, .022]) {
        add(new THREE.CylinderGeometry(.006, .006, .009, 6), silver, this.anchorClamp,
          [x, side * .026, .025]);
      }
    }
    const loadBody = this.bodies[this.demo.loadBody];
    add(new THREE.BoxGeometry(...this.demo.loadSize), load, loadBody);
    add(new THREE.BoxGeometry(this.demo.loadSize[0] * .76, .002, this.demo.loadSize[2] * .66), loadInset, loadBody,
      [0, -this.demo.loadSize[1] / 2 - .001, 0]);
    for (const x of [-1, 1]) {
      for (const z of [-1, 1]) {
        add(new THREE.CylinderGeometry(.005, .005, .003, 6), silver, loadBody,
          [x * this.demo.loadSize[0] * .32, -this.demo.loadSize[1] / 2 - .003, z * this.demo.loadSize[2] * .25]);
      }
    }
    const movingWheels = this.demo.pulleys.filter((wheel) => wheel.moving);
    if (movingWheels.length) {
      const min = Math.min(...movingWheels.map((wheel) => poses[wheel.body * 7]));
      const max = Math.max(...movingWheels.map((wheel) => poses[wheel.body * 7]));
      const barHeight = poses[movingWheels[0].body * 7 + 2] - poses[this.demo.loadBody * 7 + 2];
      add(new THREE.BoxGeometry(max - min + .08, .06, .035), moving, loadBody,
        [(min + max) / 2 - poses[this.demo.loadBody * 7], .08, barHeight]);
      const length = barHeight - this.demo.loadSize[2] / 2;
      add(new THREE.BoxGeometry(.03, .04, length), moving, loadBody,
        [0, .08, (barHeight + this.demo.loadSize[2] / 2) / 2]);
    }
    const handle = this.bodies[this.demo.handleBody];
    const handleGeometry = new THREE.CapsuleGeometry(.023, .10, 6, 16);
    handleGeometry.rotateZ(Math.PI / 2);
    this.handleMeshes.push(add(handleGeometry, grip, handle));
    for (const x of [-.043, -.025, -.008, .008, .025, .043]) {
      const ring = add(new THREE.TorusGeometry(.0235, .0016, 5, 16), rubber, handle, [x, 0, 0]);
      ring.rotation.y = Math.PI / 2;
      ring.castShadow = false;
    }
    if (this.demo.cable.length) {
      const segment = this.demo.cable[0];
      // Equal authored segments can differ by float32 roundoff in the capture.
      if (!this.demo.cable.every((entry) => Math.abs(entry.length - segment.length) < 1e-6 && entry.radius === segment.radius)) {
        throw new Error('Pulley cable segments must have matching capsule dimensions');
      }
      const geometry = new THREE.CapsuleGeometry(segment.radius, segment.length, 4, 10);
      geometry.rotateX(Math.PI / 2);
      this.geometries.push(geometry);
      this.cable = new THREE.InstancedMesh(geometry, rope, this.demo.cable.length);
      this.cable.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
      this.cable.castShadow = this.cable.receiveShadow = true;
      this.cable.frustumCulled = false;
      this.stage.add(this.cable);
      this.cableSegments = this.demo.cable.map((entry) => ({
        body: entry.body,
        transform: new THREE.Matrix4().compose(new THREE.Vector3().fromArray(entry.transform),
          new THREE.Quaternion().fromArray(entry.transform, 3), new THREE.Vector3(1, 1, 1)),
      }));
    }
    if (this.demo.ropePointsBinding || this.demo.ropePath) {
      const radius = this.demo.ropeRadius;
      this.ropeBinding = this.demo.ropePointsBinding;
      let count;
      if (this.ropeBinding) {
        count = simulation.binding(this.ropeBinding).length / 3;
        if (!Number.isInteger(count) || count < 2) throw new Error('Invalid native rope path capacity');
      } else {
        this.ropePoints = [{ body: this.demo.ropeAnchor.body, offset: this.demo.ropeAnchor.position }];
        for (const wheel of this.demo.ropePath) for (let index = 0; index <= 16; index++) {
          const angle = Math.PI + (wheel.lower ? 1 : -1) * Math.PI * index / 16;
          this.ropePoints.push({ body: wheel.body, offset: [wheel.wrapRadius * Math.cos(angle), 0, wheel.wrapRadius * Math.sin(angle)] });
        }
        this.ropePoints.push({ body: this.demo.handleBody, offset: [0, 0, 0] });
        count = this.ropePoints.length;
      }
      const geometry = new THREE.CylinderGeometry(radius, radius, 1, 10);
      const capGeometry = new THREE.SphereGeometry(radius, 8, 6);
      this.geometries.push(geometry, capGeometry);
      this.rope = new THREE.InstancedMesh(geometry, rope, count - 1);
      this.ropeCaps = new THREE.InstancedMesh(capGeometry, rope, count);
      for (const mesh of [this.rope, this.ropeCaps]) {
        mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        mesh.castShadow = mesh.receiveShadow = true;
        mesh.frustumCulled = false;
        this.stage.add(mesh);
      }
      this.ropeWorldPoints = Array.from({ length: count }, () => new THREE.Vector3());
      this.direction = new THREE.Vector3();
      this.up = new THREE.Vector3(0, 1, 0);
    }
    this.matrix = new THREE.Matrix4();
    this.position = new THREE.Vector3();
    this.quaternion = new THREE.Quaternion();
    this.scale = new THREE.Vector3(1, 1, 1);
    this.addHandleDrag();
  }

  setupControls(host, panel) {
    this.host = host;
    host.viewport = document.createElement('div');
    host.viewport.className = 'browser-demo-viewport browser-demo-pulley-viewport';
    this.canvas.before(host.viewport);
    host.viewport.append(this.canvas);
    const motion = document.createElement('section');
    motion.className = 'browser-demo-pulley-motion';
    motion.setAttribute('aria-label', 'Move the load');
    const load = this.simulation.manifest.parameters.find((parameter) => parameter.binding === 'load');
    if (host.hasAttribute('load-mass')) this.simulation.binding('load')[0] =
      Math.max(load.minimum, Math.min(load.maximum, Number(host.getAttribute('load-mass'))));
    const pull = this.simulation.manifest.parameters.find((parameter) => parameter.binding === 'pull');
    this.pullActions = [];
    const updatePull = (value) => {
      if (host.ready && !host.running && Math.abs(this.simulation.binding('pull')[0] - value) > 1e-8) {
        this.pullWhilePaused = true;
      }
      this.simulation.binding('pull')[0] = value;
      this.pullSlider.style.setProperty('--range-fill', `${100 * (value - pull.minimum) / (pull.maximum - pull.minimum)}%`);
      this.pullSlider.setAttribute('aria-valuetext', `${Math.round(value * 100)} centimeters down`);
      for (const { button, target } of this.pullActions) button.disabled = Math.abs(value - target) < pull.step / 2;
    };
    this.pullSlider = host.addSlider(motion, 'Pull down', pull.minimum, pull.maximum, pull.step,
      this.simulation.binding('pull')[0], updatePull, (value) => `${(value * 100).toFixed(0)} cm`);
    this.pullSlider.setAttribute('aria-label', 'Pull the handle downward');
    this.pullSlider.parentElement.classList.add('browser-demo-pulley-pull-slider');
    const actions = document.createElement('div');
    actions.className = 'browser-demo-pulley-actions';
    for (const [label, target] of [['Pull ↓', pull.maximum], ['Release ↑', pull.minimum]]) {
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = label;
      button.addEventListener('click', () => {
        this.pullSlider.value = target;
        this.pullSlider.dispatchEvent(new Event('input'));
      });
      this.pullActions.push({ button, target });
      actions.append(button);
    }
    updatePull(Number(this.pullSlider.value));
    motion.append(actions);
    this.pullHint = document.createElement('small');
    this.pullHint.className = 'browser-demo-pulley-pause-hint';
    this.pullHint.setAttribute('role', 'status');
    motion.append(this.pullHint);
    host.addSlider(motion, 'Payload', load.minimum, load.maximum, load.step, this.simulation.binding('load')[0],
      (value) => { this.simulation.binding('load')[0] = value; }, (value) => `${value.toFixed(1)} kg`);
    panel.append(motion);
    const heading = document.createElement('div');
    heading.className = 'browser-demo-pulley-heading';
    const eyebrow = document.createElement('small');
    eyebrow.textContent = 'IDEAL RATIO';
    this.ratioLabel = document.createElement('strong');
    this.ratioLabel.textContent = `${this.demo.supportingStrands}:1`;
    const explanation = document.createElement('span');
    explanation.textContent = this.demo.supportingStrands === 1 ? 'Changes the pull direction'
      : `${this.demo.supportingStrands} supporting strands`;
    heading.append(eyebrow, this.ratioLabel, explanation);
    host.viewport.append(heading);
    const setup = document.createElement('section');
    setup.className = 'browser-demo-pulley-setup';
    const setupTitle = document.createElement('strong');
    setupTitle.className = 'browser-demo-pulley-section-title';
    setupTitle.textContent = 'Cable model';
    setup.append(setupTitle);
    const models = document.createElement('div');
    models.className = 'browser-demo-pulley-configurations browser-demo-pulley-models';
    models.setAttribute('role', 'group');
    models.setAttribute('aria-label', 'Rope model');
    const ropeModels = demos.find((entry) => entry.id === this.demo.kind).ropeModels;
    const activeModel = ropeModels.find((model) => model.id === (this.demo.ropeModel || 'tendon'));
    for (const model of ropeModels) {
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = model.label;
      button.title = model.description;
      const selected = model.id === (this.demo.ropeModel || 'tendon');
      button.setAttribute('aria-pressed', String(selected));
      button.addEventListener('click', () => {
        if (selected) return;
        const strands = model.sources[this.demo.supportingStrands] ? this.demo.supportingStrands : 1;
        this.switchScene(model.sources[strands], strands, model.id);
      });
      models.append(button);
    }
    setup.append(models);
    const modelDescription = document.createElement('p');
    modelDescription.className = 'browser-demo-pulley-model-description';
    modelDescription.textContent = activeModel.description;
    setup.append(modelDescription);
    const configuration = document.createElement('div');
    configuration.className = 'browser-demo-pulley-configurations';
    configuration.setAttribute('role', 'group');
    configuration.setAttribute('aria-label', 'Pulley arrangement');
    for (const [count, source] of Object.entries(activeModel.sources)) {
      const strands = Number(count);
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = `${strands} ${strands === 1 ? 'wheel' : 'wheels'}`;
      button.title = strands === 1 ? 'One fixed wheel changes the direction of the pull'
        : `${strands / 2} fixed and ${strands / 2} moving wheels`;
      button.setAttribute('aria-pressed', String(strands === this.demo.supportingStrands));
      button.addEventListener('click', () => {
        if (strands === this.demo.supportingStrands) return;
        this.switchScene(source, strands);
      });
      configuration.append(button);
    }
    if (Object.keys(activeModel.sources).length > 1) setup.append(configuration);
    panel.append(setup);
    if (this.demo.ropeModel === 'newton') this.setupTendonControls(host, panel);
    this.plot = new PulleyPlot(panel.parentElement);
    this.hint = document.createElement('div');
    this.hint.className = 'browser-demo-pulley-hint';
    this.hint.textContent = 'Drag the gold handle to pull · Drag the scene to orbit';
    host.viewport.append(this.hint);
    this.loadLabel = this.addLabel('Load', this.demo.loadBody, [0, 0, -.17]);
    this.addLabel('Drag ↕', this.demo.handleBody, [.1, 0, 0]).classList.add('browser-demo-pulley-pull');
    const about = document.createElement('details');
    about.className = 'browser-demo-pulley-about';
    const summary = document.createElement('summary');
    summary.textContent = 'About the forces';
    about.append(summary);
    const note = document.createElement('small');
    note.className = 'browser-demo-pulley-note';
    const ideal = `Ideal static force = (payload + ${this.demo.movingSheaveMass.toFixed(3)} kg of moving wheels) × gravity ÷ ${this.demo.supportingStrands}.`;
    note.textContent = `${ideal} The reference assumes a massless, frictionless rope at rest. ${this.demo.ropeModel === 'newton'
      ? 'Measured pull is axial tension in a massless tendon sliding over non-rotating guides.'
      : this.demo.ropeModel === 'vbd'
      ? 'Measured pull is the vertical handle reaction, including cable weight, friction, elasticity and motion.'
      : 'Measured pull is the constraint reaction; acceleration can change it.'}`;
    about.append(note);
    panel.append(about);
    host.addEventListener('browser-demo-ready', () => {
      if (host.boundaryNote) about.append(host.boundaryNote);
    }, { once: true });
  }

  setupTendonControls(host, panel) {
    const section = document.createElement('section');
    section.className = 'browser-demo-tendon-settings';
    section.setAttribute('aria-label', 'Newton tendon properties');
    const header = document.createElement('div');
    header.className = 'browser-demo-tendon-heading';
    const title = document.createElement('strong');
    title.textContent = 'Tendon material';
    const status = document.createElement('span');
    status.setAttribute('aria-live', 'polite');
    header.append(title, status);
    section.append(header);
    const presets = this.demo.tendonPresets || [];
    const buttons = [];
    this.tendonSliders = [];
    const updatePresets = () => {
      let selected;
      for (const { preset, button } of buttons) {
        const matches = Object.entries(preset.values).every(([name, value]) => {
          const actual = this.simulation.binding(name)[0];
          return Math.abs(actual - value) <= Math.max(1e-12, Math.abs(value) * 1e-5);
        });
        button.setAttribute('aria-pressed', String(matches));
        if (matches) selected = preset;
      }
      const label = selected?.label || 'Custom';
      if (status.textContent !== label) status.textContent = label;
    };
    if (presets.length) {
      const row = document.createElement('div');
      row.className = 'browser-demo-tendon-presets';
      row.setAttribute('role', 'group');
      row.setAttribute('aria-label', 'Tendon reference presets');
      for (const preset of presets) {
        const button = document.createElement('button');
        button.type = 'button';
        button.textContent = preset.label;
        button.title = preset.description || preset.label;
        button.addEventListener('click', () => {
          for (const { parameter } of this.tendonSliders) {
            const value = preset.values[parameter.binding];
            if (!Number.isFinite(value) || value < parameter.minimum || value > parameter.maximum) {
              throw new Error(`Invalid tendon preset value: ${parameter.binding}`);
            }
            this.simulation.binding(parameter.binding)[parameter.index] = value;
          }
          this.refreshTendonControls();
        });
        buttons.push({ preset, button });
        row.append(button);
      }
      section.append(row);
    }
    const controls = document.createElement('div');
    controls.className = 'browser-demo-tendon-sliders';
    const more = document.createElement('details');
    more.className = 'browser-demo-tendon-advanced';
    const summary = document.createElement('summary');
    summary.textContent = 'Stiffness & damping';
    const advanced = document.createElement('div');
    advanced.className = 'browser-demo-tendon-sliders';
    more.append(summary, advanced);
    for (const [name, label, logarithmic, format] of [
      ['friction', 'Guide friction', false, (value) => `μ ${value.toFixed(value > 0 && value < .01 ? 3 : 2)}`],
      ['compliance', 'Span stiffness', true,
        (value) => `${(1 / value / 1000).toLocaleString(undefined, { maximumSignificantDigits: 3 })} kN/m`],
      ['damping', 'Damping', false, (value) => `${value.toFixed(2)} N·s/m`],
    ]) {
      const parameter = this.simulation.manifest.parameters.find((entry) => entry.binding === name);
      if (!parameter || parameter.index !== 0 || (logarithmic && parameter.minimum <= 0)) {
        throw new Error(`Missing or invalid Newton tendon parameter: ${name}`);
      }
      const attribute = `tendon-${name}`;
      if (host.hasAttribute(attribute)) {
        const value = Number(host.getAttribute(attribute));
        if (Number.isFinite(value)) this.simulation.binding(name)[0] =
          Math.max(parameter.minimum, Math.min(parameter.maximum, value));
      }
      const logMinimum = logarithmic ? -Math.log10(parameter.maximum) : 0;
      const logSpan = logarithmic ? Math.log10(parameter.maximum / parameter.minimum) : 0;
      const toSlider = (value) => logarithmic ? 1000 * (-Math.log10(value) - logMinimum) / logSpan : value;
      const physical = (value) => logarithmic ? 10 ** -(logMinimum + value * logSpan / 1000) : value;
      const initial = this.simulation.binding(name)[0];
      const slider = host.addSlider(name === 'friction' ? controls : advanced, label,
        logarithmic ? 0 : parameter.minimum,
        logarithmic ? 1000 : parameter.maximum,
        logarithmic ? 1 : parameter.step,
        toSlider(initial),
        (value) => {
          this.simulation.binding(name)[0] = physical(value);
          slider.setAttribute('aria-valuetext', format(physical(value)));
          updatePresets();
        },
        (value) => format(physical(value)));
      slider.setAttribute('aria-valuetext', format(initial));
      this.tendonSliders.push({ parameter, slider, toSlider, format });
    }
    const help = document.createElement('small');
    help.textContent = 'More friction needs more pull.';
    section.append(controls, help, more);
    panel.append(section);
    this.updateTendonPresets = updatePresets;
    updatePresets();
  }

  refreshTendonControls() {
    for (const { parameter, slider, toSlider, format } of this.tendonSliders || []) {
      const value = this.simulation.binding(parameter.binding)[parameter.index];
      slider.value = toSlider(value);
      slider.parentElement.querySelector('output').value = format(value);
      slider.setAttribute('aria-valuetext', format(value));
    }
    this.updateTendonPresets?.();
  }

  switchScene(source, strands, model = this.demo.ropeModel || 'tendon') {
    const url = new URL(location.href);
    url.searchParams.set('rope', model);
    url.searchParams.set('wheels', strands);
    history.replaceState(null, '', url);
    const next = this.host.cloneNode(false);
    const scene = new URL(source, new URL('../', import.meta.url));
    scene.search = this.simulation.url.search;
    next.setAttribute('src', scene.href);
    next.setAttribute('load-mass', String(this.simulation.binding('load')[0]));
    if (this.demo.ropeModel === 'newton') for (const name of ['friction', 'compliance', 'damping']) {
      next.setAttribute(`tendon-${name}`, String(this.simulation.binding(name)[0]));
    }
    const paused = !this.host.running;
    const focused = this.host.contains(document.activeElement)
      ? document.activeElement.closest('[role="group"]')?.getAttribute('aria-label') : null;
    next.addEventListener('browser-demo-ready', () => {
      if (paused && next.running) next.querySelector('.browser-demo-pause').click();
      if (focused === 'Rope model' || focused === 'Pulley arrangement') {
        next.querySelector(`[aria-label="${focused}"] [aria-pressed="true"]`)?.focus({ preventScroll: true });
      }
    }, { once: true });
    this.host.replaceWith(next);
  }

  addLabel(text, body, offset) {
    const label = document.createElement('span');
    label.className = 'browser-demo-pulley-label';
    label.textContent = text;
    this.host.viewport.append(label);
    this.labels.push({ label, body, offset: new THREE.Vector3(...offset) });
    return label;
  }

  addHandleDrag() {
    const raycaster = new THREE.Raycaster();
    const plane = new THREE.Plane();
    const point = new THREE.Vector3();
    const handleWorld = new THREE.Vector3();
    const handleScreen = new THREE.Vector3();
    let drag = null;
    const nearHandle = (event) => {
      const rect = this.canvas.getBoundingClientRect();
      this.bodies[this.demo.handleBody].getWorldPosition(handleWorld);
      handleScreen.copy(handleWorld).project(this.camera);
      const x = rect.left + (handleScreen.x + 1) * rect.width / 2;
      const y = rect.top + (1 - handleScreen.y) * rect.height / 2;
      return handleScreen.z < 1 && Math.abs(event.clientX - x) <= 26 && Math.abs(event.clientY - y) <= 26;
    };
    const project = (event) => {
      const rect = this.canvas.getBoundingClientRect();
      raycaster.setFromCamera(new THREE.Vector2(2 * (event.clientX - rect.left) / rect.width - 1,
        1 - 2 * (event.clientY - rect.top) / rect.height), this.camera);
    };
    const down = (event) => {
      if (event.button !== 0 || drag || !this.pullSlider) return;
      project(event);
      const hit = raycaster.intersectObjects(this.handleMeshes)[0];
      if (!hit && !nearHandle(event)) return;
      event.preventDefault();
      event.stopImmediatePropagation();
      const grabbed = hit?.point || handleWorld;
      plane.setFromNormalAndCoplanarPoint(this.camera.getWorldDirection(new THREE.Vector3()), grabbed);
      if (!raycaster.ray.intersectPlane(plane, point)) return;
      drag = { id: event.pointerId, y: point.y, pull: this.simulation.binding('pull')[0] };
      this.canvas.setPointerCapture(event.pointerId);
      this.canvas.classList.add('browser-demo-pushing');
    };
    const move = (event) => {
      if (!drag) {
        this.canvas.classList.toggle('browser-demo-handle-hover', nearHandle(event));
        return;
      }
      if (event.pointerId !== drag?.id) return;
      project(event);
      if (!raycaster.ray.intersectPlane(plane, point)) return;
      this.pullSlider.value = Math.max(Number(this.pullSlider.min), Math.min(Number(this.pullSlider.max), drag.pull + drag.y - point.y));
      this.pullSlider.dispatchEvent(new Event('input'));
    };
    const release = (event) => {
      if (event?.pointerId !== undefined && event.pointerId !== drag?.id) return;
      const id = drag?.id;
      drag = null;
      if (id !== undefined && this.canvas.hasPointerCapture(id)) this.canvas.releasePointerCapture(id);
      this.canvas.classList.remove('browser-demo-pushing', 'browser-demo-handle-hover');
    };
    const leave = () => this.canvas.classList.remove('browser-demo-handle-hover');
    this.canvas.addEventListener('pointerdown', down, true);
    this.canvas.addEventListener('pointermove', move, true);
    this.canvas.addEventListener('pointerleave', leave);
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) this.canvas.addEventListener(name, release, true);
    window.addEventListener('blur', release);
    this.releasePush = () => release();
    this.disposeDrag = () => {
      release();
      this.canvas.removeEventListener('pointerdown', down, true);
      this.canvas.removeEventListener('pointermove', move, true);
      this.canvas.removeEventListener('pointerleave', leave);
      for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) this.canvas.removeEventListener(name, release, true);
      window.removeEventListener('blur', release);
    };
  }

  sample(time) {
    const values = this.simulation.binding('telemetry');
    const fields = this.demo.telemetryFields;
    this.plot.add(time, values[fields.indexOf('ideal')], values[fields.indexOf('tension')]);
  }

  reset() {
    this.pullWhilePaused = false;
    this.plot?.reset();
    if (this.pullSlider) {
      this.pullSlider.value = this.simulation.binding('pull')[0];
      this.pullSlider.dispatchEvent(new Event('input'));
    }
    this.refreshTendonControls();
  }

  render() {
    const poses = this.simulation.binding('body_q');
    if (!poses.every(Number.isFinite)) throw new Error('Non-finite pulley pose');
    this.bodies.forEach((body, index) => {
      body.position.fromArray(poses, index * 7);
      body.quaternion.fromArray(poses, index * 7 + 3);
      body.updateMatrix();
    });
    if (this.cable) {
      this.cableSegments.forEach(({ body, transform }, index) => {
        this.cable.setMatrixAt(index, this.matrix.multiplyMatrices(this.bodies[body].matrix, transform));
      });
      this.cable.instanceMatrix.needsUpdate = true;
    }
    if (this.rope) {
      this.quaternion.identity();
      this.scale.set(1, 1, 1);
      const points = this.ropeBinding ? this.simulation.binding(this.ropeBinding) : null;
      const count = this.demo.ropePointCountBinding
        ? this.simulation.binding(this.demo.ropePointCountBinding)[0] : this.ropeWorldPoints.length;
      if (!Number.isInteger(count) || count < 2 || count > this.ropeWorldPoints.length) {
        throw new Error('Invalid native rope path point count');
      }
      this.rope.count = count - 1;
      this.ropeCaps.count = count;
      for (let index = 0; index < count; index++) {
        const point = this.ropeWorldPoints[index];
        if (points) {
          point.fromArray(points, index * 3);
          if (![point.x, point.y, point.z].every(Number.isFinite)) throw new Error('Non-finite native rope path');
        } else {
          const { body, offset } = this.ropePoints[index];
          point.fromArray(offset);
          if (body >= 0) point.add(this.bodies[body].position);
        }
        this.ropeCaps.setMatrixAt(index, this.matrix.compose(point, this.quaternion, this.scale));
      }
      for (let index = 1; index < count; index++) {
        const start = this.ropeWorldPoints[index - 1];
        const end = this.ropeWorldPoints[index];
        this.direction.subVectors(end, start);
        const length = this.direction.length();
        this.quaternion.setFromUnitVectors(this.up, this.direction.divideScalar(Math.max(length, 1e-9)));
        this.position.addVectors(start, end).multiplyScalar(.5);
        this.scale.set(1, length, 1);
        this.rope.setMatrixAt(index - 1, this.matrix.compose(this.position, this.quaternion, this.scale));
      }
      this.rope.instanceMatrix.needsUpdate = true;
      this.ropeCaps.instanceMatrix.needsUpdate = true;
    }
    const width = this.canvas.clientWidth;
    const height = this.canvas.clientHeight;
    if (!width || !height) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    if (this.canvas.width !== Math.round(width * ratio) || this.canvas.height !== Math.round(height * ratio)) {
      this.renderer.setPixelRatio(ratio);
      this.renderer.setSize(width, height, false);
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      const fitDistance = Math.max(this.frameHeight / .52,
        this.frameWidth / (2 * Math.tan(this.camera.fov * Math.PI / 360) * this.camera.aspect));
      this.orbit.distance *= fitDistance / this.fitDistance;
      this.fitDistance = fitDistance;
    }
    this.orbit.update(this.target);
    this.renderer.render(this.scene, this.camera);
    for (const { label, body, offset, topHeight } of this.labels) {
      this.position.copy(offset);
      if (body >= 0) this.position.applyQuaternion(this.bodies[body].quaternion).add(this.bodies[body].position);
      if (topHeight !== undefined) this.position.z = (this.bodies[body].position.z + offset.z + topHeight) / 2;
      this.position.set(this.position.x, this.position.z, -this.position.y).project(this.camera);
      label.hidden = Math.abs(this.position.x) > 1 || Math.abs(this.position.y) > 1 || this.position.z > 1;
      label.style.left = `${(this.position.x + 1) * width / 2}px`;
      label.style.top = `${(1 - this.position.y) * height / 2}px`;
    }
    if (this.loadLabel) this.loadLabel.textContent = `${this.simulation.binding('load')[0].toFixed(1)} kg payload`;
    if (this.pullHint) {
      if (this.host.running) this.pullWhilePaused = false;
      const message = this.pullWhilePaused ? 'Paused · press Play to move the cable.' : '';
      if (this.pullHint.textContent !== message) this.pullHint.textContent = message;
      this.pullHint.hidden = !message;
    }
    this.plot?.draw();
  }

  dispose() {
    this.disposeDrag?.();
    this.disposeGround?.();
    this.plot?.dispose();
    this.orbit?.dispose();
    this.geometries.forEach((geometry) => geometry.dispose());
    this.materials.forEach((material) => material.dispose());
    this.renderer.dispose();
  }
}
