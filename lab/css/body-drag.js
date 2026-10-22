// Copyright (c) 2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

import * as THREE from '../vendor/three.module.min.js';

export class BodyDrag {
  constructor(canvas, simulation, camera, scene, bodies, allowedBodies) {
    if (!Array.isArray(allowedBodies) || allowedBodies.some((body) =>
      !Number.isInteger(body) || body < 0 || body >= bodies.length)) {
      throw new Error('Invalid draggable body list');
    }
    if (simulation.binding('drag_body').length !== 1
        || simulation.binding('drag_local_anchor').length !== 3
        || simulation.binding('drag_target').length !== 3) {
      throw new Error('Invalid rigid-body drag inputs');
    }
    this.canvas = canvas;
    this.simulation = simulation;
    this.camera = camera;
    this.scene = scene;
    this.targets = new Map(allowedBodies.map((body) => [bodies[body], body]));
    this.raycaster = new THREE.Raycaster();
    this.plane = new THREE.Plane();
    this.pointer = new THREE.Vector2();
    this.point = new THREE.Vector3();
    this.position = new THREE.Vector3();
    this.rotation = new THREE.Quaternion();
    this.anchor = new THREE.Vector3();
    this.target = new THREE.Vector3();
    this.body = -1;
    this.pointerId = null;

    this.visual = new THREE.Group();
    this.visual.visible = false;
    this.marker = new THREE.Mesh(new THREE.SphereGeometry(.0075, 12, 8),
      new THREE.MeshBasicMaterial({ color: '#76b900', depthTest: false, depthWrite: false, toneMapped: false }));
    this.marker.renderOrder = 1;
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3).setUsage(THREE.DynamicDrawUsage));
    this.tether = new THREE.Line(geometry,
      new THREE.LineBasicMaterial({ color: '#76b900', transparent: true, opacity: .85, depthTest: false,
        depthWrite: false, toneMapped: false }));
    this.tether.renderOrder = 1;
    this.tether.frustumCulled = false;
    this.visual.add(this.marker, this.tether);
    scene.add(this.visual);

    this.onDown = (event) => {
      if (this.pointerId !== null) {
        event.stopImmediatePropagation();
        return;
      }
      if (event.button !== 0 || event.shiftKey || !this.project(event)) return;
      const hit = this.raycaster.intersectObjects(bodies, true)[0];
      if (!hit) return;
      let group = hit.object;
      while (group && !this.targets.has(group)) group = group.parent;
      if (!group) return;
      event.preventDefault();
      event.stopImmediatePropagation();
      this.body = this.targets.get(group);
      const poses = simulation.binding('body_q');
      this.position.fromArray(poses, this.body * 7);
      this.rotation.fromArray(poses, this.body * 7 + 3).invert();
      this.anchor.set(hit.point.x, -hit.point.z, hit.point.y).sub(this.position).applyQuaternion(this.rotation);
      this.target.copy(hit.point);
      this.plane.setFromNormalAndCoplanarPoint(camera.getWorldDirection(this.point), hit.point);
      simulation.binding('drag_local_anchor').set(this.anchor.toArray());
      simulation.binding('drag_target').set([hit.point.x, -hit.point.z, hit.point.y]);
      simulation.binding('drag_body')[0] = this.body;
      this.pointerId = event.pointerId;
      canvas.setPointerCapture(this.pointerId);
      canvas.classList.add('browser-demo-pushing');
      this.visual.visible = true;
      this.render();
    };
    this.onMove = (event) => {
      if (event.pointerId !== this.pointerId || !this.project(event)
          || !this.raycaster.ray.intersectPlane(this.plane, this.point)) return;
      this.target.copy(this.point);
      simulation.binding('drag_target').set([this.point.x, -this.point.z, this.point.y]);
    };
    this.onRelease = (event) => this.release(event);
    canvas.addEventListener('pointerdown', this.onDown, { capture: true });
    canvas.addEventListener('pointermove', this.onMove);
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) canvas.addEventListener(name, this.onRelease);
    window.addEventListener('blur', this.onRelease);
  }

  project(event) {
    const rect = this.canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return false;
    this.pointer.set(2 * (event.clientX - rect.left) / rect.width - 1,
      1 - 2 * (event.clientY - rect.top) / rect.height);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    return true;
  }

  release(event) {
    if (event?.pointerId !== undefined && event.pointerId !== this.pointerId) return;
    if (this.pointerId === null && this.body < 0) return;
    const pointerId = this.pointerId;
    this.pointerId = null;
    this.body = -1;
    this.simulation.binding('drag_body')[0] = -1;
    this.visual.visible = false;
    this.canvas.classList.remove('browser-demo-pushing');
    if (pointerId !== null && this.canvas.hasPointerCapture(pointerId)) this.canvas.releasePointerCapture(pointerId);
  }

  render() {
    if (this.body < 0) return;
    const poses = this.simulation.binding('body_q');
    this.position.fromArray(poses, this.body * 7);
    this.rotation.fromArray(poses, this.body * 7 + 3);
    this.point.copy(this.anchor).applyQuaternion(this.rotation).add(this.position);
    this.marker.position.set(this.point.x, this.point.z, -this.point.y);
    const positions = this.tether.geometry.attributes.position;
    positions.setXYZ(0, this.marker.position.x, this.marker.position.y, this.marker.position.z);
    positions.setXYZ(1, this.target.x, this.target.y, this.target.z);
    positions.needsUpdate = true;
  }

  dispose() {
    this.release();
    this.canvas.removeEventListener('pointerdown', this.onDown, { capture: true });
    this.canvas.removeEventListener('pointermove', this.onMove);
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) this.canvas.removeEventListener(name, this.onRelease);
    window.removeEventListener('blur', this.onRelease);
    this.scene.remove(this.visual);
    this.marker.geometry.dispose();
    this.marker.material.dispose();
    this.tether.geometry.dispose();
    this.tether.material.dispose();
  }
}
