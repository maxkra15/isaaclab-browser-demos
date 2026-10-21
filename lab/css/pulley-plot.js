// Copyright (c) 2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

export class PulleyPlot {
  constructor(parent, labels = {}) {
    this.samples = [];
    this.container = document.createElement('section');
    this.container.className = 'browser-demo-pulley-plot';
    this.container.setAttribute('aria-label', labels.ariaLabel || 'Measured pull force and ideal static force');
    this.canvas = document.createElement('canvas');
    this.canvas.setAttribute('role', 'img');
    this.canvas.setAttribute('aria-label', 'Cable force in newtons over the last eight seconds of simulation');
    this.legend = document.createElement('div');
    this.legend.className = 'browser-demo-pulley-legend';
    this.values = {};
    for (const [key, title] of [['measured', labels.measured || 'Measured pull'], ['ideal', labels.ideal || 'Ideal pull']]) {
      const item = document.createElement('div');
      item.className = `browser-demo-pulley-${key}`;
      const label = document.createElement('span');
      label.textContent = title;
      const value = document.createElement('output');
      value.setAttribute('aria-label', `${title} in newtons`);
      value.setAttribute('aria-live', 'off');
      value.value = '— N';
      item.append(label, value);
      this.legend.append(item);
      this.values[key] = value;
    }
    const heading = document.createElement('strong');
    heading.textContent = labels.title || 'Pull force';
    const note = document.createElement('small');
    note.textContent = labels.note || 'Last 8 s · simulation time';
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
    const left = 43;
    const right = width - 14;
    const top = 13;
    const bottom = height - 25;
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
    context.font = '11px system-ui, sans-serif';
    context.textBaseline = 'middle';
    context.fillStyle = '#536d78';
    context.lineWidth = 1;
    for (let tick = 0; tick <= 2; tick++) {
      const force = minimum + tick * (maximum - minimum) / 2;
      context.strokeStyle = '#dfe8eb';
      context.beginPath();
      context.moveTo(left, y(force));
      context.lineTo(right, y(force));
      context.stroke();
      context.textAlign = 'right';
      context.fillText(force.toFixed(Math.abs(force) < 10 ? 1 : 0), left - 9, y(force));
    }
    context.fillText('N', left - 9, height - 10);
    const timeTicks = width < 480 ? 2 : 4;
    for (let tick = 0; tick <= timeTicks; tick++) {
      const position = left + tick / timeTicks * (right - left);
      context.textAlign = tick === 0 ? 'left' : tick === timeTicks ? 'right' : 'center';
      context.fillText(`${(end - 8 + tick * 8 / timeTicks).toFixed(0)} s`, position, height - 10);
    }

    context.save();
    context.beginPath();
    context.rect(left, top, right - left, bottom - top);
    context.clip();
    if (this.samples.length > 1) {
      const fill = context.createLinearGradient(0, top, 0, bottom);
      fill.addColorStop(0, '#168bb11c');
      fill.addColorStop(1, '#168bb103');
      context.fillStyle = fill;
      context.beginPath();
      context.moveTo(x(this.samples[0].time), y(0));
      for (const sample of this.samples) context.lineTo(x(sample.time), y(sample.measured));
      context.lineTo(x(this.samples.at(-1).time), y(0));
      context.closePath();
      context.fill();
    }
    for (const [key, color] of [['ideal', '#609000'], ['measured', '#168bb1']]) {
      context.strokeStyle = color;
      context.lineWidth = key === 'measured' ? 2.5 : 1.75;
      context.lineJoin = 'round';
      context.lineCap = 'round';
      context.setLineDash(key === 'ideal' ? [5, 4] : []);
      context.beginPath();
      this.samples.forEach((sample, index) => {
        if (index) context.lineTo(x(sample.time), y(sample[key]));
        else context.moveTo(x(sample.time), y(sample[key]));
      });
      context.stroke();
    }
    context.setLineDash([]);
    context.restore();
    const latest = this.samples.at(-1);
    if (latest) {
      for (const [key, color] of [['ideal', '#609000'], ['measured', '#168bb1']]) {
        context.beginPath();
        context.arc(x(latest.time), y(latest[key]), key === 'measured' ? 3.5 : 3, 0, Math.PI * 2);
        context.fillStyle = key === 'measured' ? color : '#ffffff';
        context.strokeStyle = key === 'measured' ? '#ffffff' : color;
        context.lineWidth = 1.5;
        context.fill();
        context.stroke();
        this.values[key].value = `${latest[key].toFixed(2)} N`;
      }
    }
  }

  dispose() {
    this.container.remove();
  }
}
