// Copyright (c) 2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

import * as THREE from '../../v1/vendor/three.module.min.js';
import { OrbitCamera } from '../../v6/css/browser-orbit.js';
import { addIsaacLabGround } from './browser-ground.js?rev=01ee9fecd932';
import { PulleyPlot } from './pulley-plot.js?rev=01ee9fecd932';
import { demos } from '../demos.js?rev=01ee9fecd932';

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
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'low-power' });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.2;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color('#f1f5f6');
    this.scene.fog = new THREE.Fog('#f1f5f6', 7, 16);
    this.camera = new THREE.PerspectiveCamera(36, 1, .03, 30);
    this.target = new THREE.Vector3(0, .95, 0);
    this.orbit = new OrbitCamera(canvas, this.camera,
      Math.PI / 2 - (this.demo.supportingStrands === 1 ? .22 : .06), .1, 3.7);
    this.scene.add(new THREE.HemisphereLight('#ffffff', '#b3c4c9', 2));
    const key = new THREE.DirectionalLight('#ffffff', 2.5);
    key.position.set(-2, 4, 3);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    key.shadow.camera.left = key.shadow.camera.bottom = -2;
    key.shadow.camera.right = key.shadow.camera.top = 2;
    key.shadow.normalBias = .002;
    this.scene.add(key);
    const fill = new THREE.DirectionalLight('#d7eaf0', .7);
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
    const frame = material('#415966', .4, .5);
    const fixed = material('#365362', .35, .5);
    const moving = material('#218ca6', .35, .4);
    const silver = material('#c8d3d5', .3, .7);
    const rope = material('#e5b44d', .75, 0);
    const load = material('#d36d47', .48, .1);
    const grip = material('#f4c44f', .5, .1);
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
    }
    add(new THREE.BoxGeometry(right - left + .11, .09, .08), frame, this.stage, [center, .12, gantryHeight]);
    for (const wheel of this.demo.pulleys) {
      const body = this.bodies[wheel.body];
      const surface = wheel.moving ? moving : fixed;
      add(new THREE.CylinderGeometry(wheel.radius, wheel.radius, 2 * wheel.grooveHalfWidth, 48), surface, body);
      for (const side of [-1, 1]) {
        add(new THREE.CylinderGeometry(wheel.flangeRadius, wheel.flangeRadius, 2 * wheel.flangeHalfWidth, 48), surface, body,
          [0, side * wheel.flangeOffset, 0]);
        add(new THREE.CylinderGeometry(.018, .018, .012, 20), silver, body,
          [0, side * (wheel.halfWidth + .014), 0]);
      }
      const spoke = add(new THREE.BoxGeometry(wheel.radius * 1.5, .005, .009), silver, body, [0, -wheel.halfWidth - .01, 0]);
      spoke.castShadow = false;
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
    if (this.demo.ropePath) {
      const radius = this.demo.ropeRadius;
      this.ropePoints = [{ body: this.demo.ropeAnchor.body, offset: this.demo.ropeAnchor.position }];
      for (const wheel of this.demo.ropePath) for (let index = 0; index <= 16; index++) {
        const angle = Math.PI + (wheel.lower ? 1 : -1) * Math.PI * index / 16;
        this.ropePoints.push({ body: wheel.body, offset: [wheel.wrapRadius * Math.cos(angle), 0, wheel.wrapRadius * Math.sin(angle)] });
      }
      this.ropePoints.push({ body: this.demo.handleBody, offset: [0, 0, 0] });
      const geometry = new THREE.CylinderGeometry(radius, radius, 1, 10);
      const capGeometry = new THREE.SphereGeometry(radius, 8, 6);
      this.geometries.push(geometry, capGeometry);
      this.rope = new THREE.InstancedMesh(geometry, rope, this.ropePoints.length - 1);
      this.ropeCaps = new THREE.InstancedMesh(capGeometry, rope, this.ropePoints.length);
      for (const mesh of [this.rope, this.ropeCaps]) {
        mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        mesh.castShadow = mesh.receiveShadow = true;
        mesh.frustumCulled = false;
        this.stage.add(mesh);
      }
      this.ropeWorldPoints = this.ropePoints.map(() => new THREE.Vector3());
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
    const heading = document.createElement('div');
    heading.className = 'browser-demo-pulley-heading';
    const eyebrow = document.createElement('small');
    eyebrow.textContent = this.demo.ropeModel === 'vbd' ? 'VBD CABLE' : 'MECHANICAL ADVANTAGE';
    this.ratioLabel = document.createElement('strong');
    this.ratioLabel.textContent = `${this.demo.supportingStrands}:1`;
    const explanation = document.createElement('span');
    explanation.textContent = this.demo.supportingStrands === 1 ? 'Changes the pull direction'
      : `${this.demo.supportingStrands} supporting strands`;
    heading.append(eyebrow, this.ratioLabel, explanation);
    host.viewport.append(heading);
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
      const selected = model.id === (this.demo.ropeModel || 'tendon');
      button.setAttribute('aria-pressed', String(selected));
      button.addEventListener('click', () => {
        if (selected) return;
        const strands = model.sources[this.demo.supportingStrands] ? this.demo.supportingStrands : 1;
        this.switchScene(model.sources[strands], strands, model.id);
      });
      models.append(button);
    }
    panel.append(models);
    const configuration = document.createElement('div');
    configuration.className = 'browser-demo-pulley-configurations';
    configuration.setAttribute('role', 'group');
    configuration.setAttribute('aria-label', 'Pulley arrangement');
    for (const [count, source] of Object.entries(activeModel.sources)) {
      const strands = Number(count);
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = `${strands} ${strands === 1 ? 'wheel' : 'wheels'} · ${strands}:1`;
      button.title = strands === 1 ? 'One fixed wheel changes the direction of the pull'
        : `${strands / 2} fixed and ${strands / 2} moving wheels`;
      button.setAttribute('aria-pressed', String(strands === this.demo.supportingStrands));
      button.addEventListener('click', () => {
        if (strands === this.demo.supportingStrands) return;
        this.switchScene(source, strands);
      });
      configuration.append(button);
    }
    if (Object.keys(activeModel.sources).length > 1) panel.append(configuration);
    const load = this.simulation.manifest.parameters.find((parameter) => parameter.binding === 'load');
    if (host.hasAttribute('load-mass')) this.simulation.binding('load')[0] =
      Math.max(load.minimum, Math.min(load.maximum, Number(host.getAttribute('load-mass'))));
    const pull = this.simulation.manifest.parameters.find((parameter) => parameter.binding === 'pull');
    this.pullSlider = host.addSlider(panel, 'Pull down', pull.minimum, pull.maximum, pull.step,
      this.simulation.binding('pull')[0], (value) => { this.simulation.binding('pull')[0] = value; },
      (value) => `${(value * 100).toFixed(0)} cm`);
    this.pullSlider.setAttribute('aria-label', 'Pull the handle downward');
    host.addSlider(panel, 'Payload', load.minimum, load.maximum, load.step, this.simulation.binding('load')[0],
      (value) => { this.simulation.binding('load')[0] = value; }, (value) => `${value.toFixed(1)} kg`);
    this.plot = new PulleyPlot(host.viewport, this.demo.ropeModel === 'vbd'
      ? { title: 'Handle force', ideal: 'Ideal · massless rope', measured: 'Measured handle' } : {});
    this.hint = document.createElement('div');
    this.hint.className = 'browser-demo-pulley-hint';
    this.hint.textContent = 'Drag the yellow handle down · Drag the background to orbit';
    host.viewport.append(this.hint);
    this.loadLabel = this.addLabel('Load', this.demo.loadBody, [0, 0, -.17]);
    this.addLabel('Pull ↓', this.demo.handleBody, [.1, 0, 0]).classList.add('browser-demo-pulley-pull');
    const anchor = this.demo.ropeAnchor;
    if (anchor.body < 0) this.addLabel('Fixed end', anchor.body,
      [anchor.position[0] - .15, anchor.position[1] - .04, anchor.position[2] + .025]);
    for (const [index, strand] of this.demo.supportStrands.entries()) {
      this.addLabel(String(index + 1), this.demo.loadBody, [strand.bottom[0], -.045, strand.bottom[2]]);
      this.labels.at(-1).topHeight = strand.top[2];
    }
    const note = document.createElement('small');
    note.className = 'browser-demo-pulley-note';
    const ideal = this.demo.supportingStrands === 1 ? 'Ideal at rest: payload × gravity.'
      : `Ideal at rest: (payload + ${this.demo.movingSheaveMass.toFixed(3)} kg of moving wheels) × gravity ÷ ${this.demo.supportingStrands}.`;
    note.textContent = `${ideal} ${this.demo.ropeModel === 'vbd'
      ? 'Measured vertical handle reaction includes rope mass, friction, guide damping and motion. The test handle follows the commanded pull.'
      : 'Frictionless wrapping and a massless rope; the measured constraint force also responds to acceleration.'}`;
    panel.append(note);
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
    let drag = null;
    const project = (event) => {
      const rect = this.canvas.getBoundingClientRect();
      raycaster.setFromCamera(new THREE.Vector2(2 * (event.clientX - rect.left) / rect.width - 1,
        1 - 2 * (event.clientY - rect.top) / rect.height), this.camera);
    };
    const down = (event) => {
      if (event.button !== 0 || drag || !this.pullSlider) return;
      project(event);
      const hit = raycaster.intersectObjects(this.handleMeshes)[0];
      if (!hit) return;
      event.preventDefault();
      event.stopImmediatePropagation();
      plane.setFromNormalAndCoplanarPoint(this.camera.getWorldDirection(new THREE.Vector3()), hit.point);
      drag = { id: event.pointerId, y: hit.point.y, pull: this.simulation.binding('pull')[0] };
      this.canvas.setPointerCapture(event.pointerId);
      this.canvas.classList.add('browser-demo-pushing');
    };
    const move = (event) => {
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
      this.canvas.classList.remove('browser-demo-pushing');
    };
    this.canvas.addEventListener('pointerdown', down, true);
    this.canvas.addEventListener('pointermove', move, true);
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) this.canvas.addEventListener(name, release, true);
    window.addEventListener('blur', release);
    this.releasePush = () => release();
    this.disposeDrag = () => {
      release();
      this.canvas.removeEventListener('pointerdown', down, true);
      this.canvas.removeEventListener('pointermove', move, true);
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
    this.plot?.reset();
    if (this.pullSlider) {
      this.pullSlider.value = this.simulation.binding('pull')[0];
      this.pullSlider.dispatchEvent(new Event('input'));
    }
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
      this.ropePoints.forEach(({ body, offset }, index) => {
        const point = this.ropeWorldPoints[index];
        point.fromArray(offset);
        if (body >= 0) point.add(this.bodies[body].position);
        this.ropeCaps.setMatrixAt(index, this.matrix.compose(point, this.quaternion, this.scale));
      });
      for (let index = 1; index < this.ropeWorldPoints.length; index++) {
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
