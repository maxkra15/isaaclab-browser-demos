// Copyright (c) 2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

export class OrbitCamera {
  constructor(canvas, camera, azimuth, elevation, distance, enablePan = false) {
    this.canvas = canvas;
    this.camera = camera;
    this.azimuth = azimuth;
    this.elevation = elevation;
    this.distance = distance;
    this.panOffset = camera.position.clone().set(0, 0, 0);
    this.focus = camera.position.clone();
    this.onPointerDown = (event) => {
      if (this.drag || (event.button !== 0 && !(enablePan && [1, 2].includes(event.button)))) return;
      event.preventDefault();
      this.drag = { x: event.clientX, y: event.clientY, id: event.pointerId,
        pan: enablePan && (event.button !== 0 || event.shiftKey) };
      canvas.setPointerCapture(event.pointerId);
    };
    this.onPointerMove = (event) => {
      if (!this.drag || event.pointerId !== this.drag.id) return;
      const dx = event.clientX - this.drag.x, dy = event.clientY - this.drag.y;
      if (this.drag.pan) {
        const scale = 2 * this.distance * Math.tan(camera.fov * Math.PI / 360) / canvas.clientHeight;
        const basis = camera.matrixWorld.elements;
        this.panOffset.x += scale * (-dx * basis[0] + dy * basis[4]);
        this.panOffset.y += scale * (-dx * basis[1] + dy * basis[5]);
        this.panOffset.z += scale * (-dx * basis[2] + dy * basis[6]);
      } else {
        this.azimuth -= dx * 0.006;
        this.elevation = Math.max(0.05, Math.min(1.25, this.elevation + dy * 0.006));
      }
      this.drag.x = event.clientX;
      this.drag.y = event.clientY;
    };
    this.onPointerUp = (event) => { if (event.pointerId === this.drag?.id) this.drag = null; };
    this.onWheel = (event) => {
      event.preventDefault();
      this.distance = Math.max(0.35, Math.min(6, this.distance * Math.exp(event.deltaY * 0.001)));
    };
    this.onContextMenu = (event) => { if (enablePan) event.preventDefault(); };
    this.onDoubleClick = () => { if (enablePan) this.panOffset.set(0, 0, 0); };
    this.onBlur = () => {
      const id = this.drag?.id;
      this.drag = null;
      if (id !== undefined && canvas.hasPointerCapture(id)) canvas.releasePointerCapture(id);
    };
    canvas.addEventListener('pointerdown', this.onPointerDown);
    canvas.addEventListener('pointermove', this.onPointerMove);
    canvas.addEventListener('pointerup', this.onPointerUp);
    canvas.addEventListener('pointercancel', this.onPointerUp);
    canvas.addEventListener('lostpointercapture', this.onPointerUp);
    canvas.addEventListener('wheel', this.onWheel, { passive: false });
    canvas.addEventListener('contextmenu', this.onContextMenu);
    canvas.addEventListener('dblclick', this.onDoubleClick);
    window.addEventListener('blur', this.onBlur);
  }

  update(target) {
    this.focus.copy(target).add(this.panOffset);
    const horizontal = this.distance * Math.cos(this.elevation);
    this.camera.position.set(
      this.focus.x + horizontal * Math.cos(this.azimuth),
      this.focus.y + this.distance * Math.sin(this.elevation),
      this.focus.z + horizontal * Math.sin(this.azimuth),
    );
    this.camera.lookAt(this.focus);
  }

  dispose() {
    this.onBlur();
    this.canvas.removeEventListener('pointerdown', this.onPointerDown);
    this.canvas.removeEventListener('pointermove', this.onPointerMove);
    this.canvas.removeEventListener('pointerup', this.onPointerUp);
    this.canvas.removeEventListener('pointercancel', this.onPointerUp);
    this.canvas.removeEventListener('lostpointercapture', this.onPointerUp);
    this.canvas.removeEventListener('wheel', this.onWheel);
    this.canvas.removeEventListener('contextmenu', this.onContextMenu);
    this.canvas.removeEventListener('dblclick', this.onDoubleClick);
    window.removeEventListener('blur', this.onBlur);
  }
}
