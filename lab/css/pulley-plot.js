// Copyright (c) 2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

export class PulleyPlot {
  constructor(parent, labels = {}) {
    this.samples = [];
    this.container = document.createElement('section');
    this.container.className = 'browser-demo-pulley-plot';
    this.container.setAttribute('aria-label', 'Measured cable tension and ideal static force');
    this.canvas = document.createElement('canvas');
    this.canvas.setAttribute('aria-label', 'Cable force in newtons over the last eight seconds of simulation');
    this.legend = document.createElement('div');
    this.legend.className = 'browser-demo-pulley-legend';
    this.values = {};
    for (const [key, title] of [['ideal', labels.ideal || 'Ideal at rest'], ['measured', labels.measured || 'Measured cable']]) {
      const item = document.createElement('div');
      item.className = `browser-demo-pulley-${key}`;
      const label = document.createElement('span');
      label.textContent = title;
      const value = document.createElement('output');
      value.value = '— N';
      item.append(label, value);
      this.legend.append(item);
      this.values[key] = value;
    }
    const heading = document.createElement('strong');
    heading.textContent = labels.title || 'Cable force';
    const note = document.createElement('small');
    note.textContent = 'Step average · 8 s of simulation time';
    const header = document.createElement('div');
    header.className = 'browser-demo-pulley-plot-heading';
    header.append(heading, note);
    this.container.append(header, this.legend, this.canvas);
    parent.append(this.container);
  }

  add(time, ideal, measured) {
    if (![time, ideal, measured].every(Number.isFinite)) throw new Error('Non-finite pulley force sample');
    this.samples.push({ time, ideal, measured });
    while (this.samples.length && this.samples[0].time < time - 8) this.samples.shift();
  }

  reset() {
    this.samples.length = 0;
    for (const value of Object.values(this.values)) value.value = '— N';
    this.draw();
  }

  draw() {
    const width = this.canvas.clientWidth;
    const height = this.canvas.clientHeight;
    if (!width || !height) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    if (this.canvas.width !== Math.round(width * ratio) || this.canvas.height !== Math.round(height * ratio)) {
      this.canvas.width = Math.round(width * ratio);
      this.canvas.height = Math.round(height * ratio);
    }
    const context = this.canvas.getContext('2d');
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, width, height);
    const left = 39;
    const right = width - 12;
    const top = 12;
    const bottom = height - 22;
    const end = Math.max(8, this.samples.at(-1)?.time || 0);
    let minimum = 0;
    let maximum = 5;
    for (const sample of this.samples) {
      minimum = Math.min(minimum, sample.ideal, sample.measured);
      maximum = Math.max(maximum, sample.ideal, sample.measured);
    }
    const interval = 10 ** Math.floor(Math.log10(Math.max(maximum - minimum, 1))) / 2;
    minimum = Math.floor(minimum / interval) * interval;
    maximum = Math.ceil(maximum * 1.08 / interval) * interval;
    const x = (time) => left + (time - end + 8) / 8 * (right - left);
    const y = (force) => bottom - (force - minimum) / (maximum - minimum) * (bottom - top);
    context.font = '10px system-ui, sans-serif';
    context.textBaseline = 'middle';
    context.fillStyle = '#738790';
    context.lineWidth = 1;
    for (let tick = 0; tick <= 2; tick++) {
      const force = minimum + tick * (maximum - minimum) / 2;
      context.strokeStyle = '#dbe5e8';
      context.beginPath();
      context.moveTo(left, y(force));
      context.lineTo(right, y(force));
      context.stroke();
      context.fillText(force.toFixed(Math.abs(force) < 10 ? 1 : 0), 3, y(force));
    }
    context.fillText('N', 3, height - 9);
    for (let tick = 0; tick <= 4; tick++) {
      const position = left + tick / 4 * (right - left);
      context.beginPath();
      context.moveTo(position, top);
      context.lineTo(position, bottom);
      context.stroke();
      context.fillText(`${(end - 8 + tick * 2).toFixed(0)} s`, position - (tick === 4 ? 16 : 0), height - 9);
    }
    for (const [key, color] of [['ideal', '#609000'], ['measured', '#168bb1']]) {
      context.strokeStyle = color;
      context.lineWidth = 2;
      context.lineJoin = 'round';
      context.setLineDash(key === 'ideal' ? [5, 4] : []);
      context.beginPath();
      this.samples.forEach((sample, index) => {
        if (index) context.lineTo(x(sample.time), y(sample[key]));
        else context.moveTo(x(sample.time), y(sample[key]));
      });
      context.stroke();
    }
    context.setLineDash([]);
    const latest = this.samples.at(-1);
    if (latest) for (const key of ['ideal', 'measured']) this.values[key].value = `${latest[key].toFixed(2)} N`;
  }

  dispose() {
    this.container.remove();
  }
}
