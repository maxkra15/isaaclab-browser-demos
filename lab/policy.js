// Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

const sizeOf = (shape) => shape.reduce((size, dimension) => size * dimension, 1);
const finite = (values, label) => {
  if (!values.every(Number.isFinite)) throw new Error(`${label} contains non-finite values.`);
};

export class Policy {
  static async load(description, base, simulation) {
    const ort = await import('./vendor/ort/ort.wasm.min.mjs');
    ort.env.wasm.wasmPaths = new URL('./vendor/ort/', import.meta.url).href;
    ort.env.wasm.numThreads = 1;
    const response = await fetch(new URL(description.file, base));
    if (!response.ok) throw new Error(`Could not load policy: ${response.status}`);
    const bytes = await response.arrayBuffer();
    if (description.sha256) {
      const hash = [...new Uint8Array(await crypto.subtle.digest('SHA-256', bytes))]
        .map((byte) => byte.toString(16).padStart(2, '0')).join('');
      if (hash !== description.sha256) throw new Error('Policy checksum does not match its manifest.');
    }
    const session = await ort.InferenceSession.create(bytes, { executionProviders: ['wasm'] });
    try {
      return new Policy(ort, session, description, simulation);
    } catch (error) {
      await session.release();
      throw error;
    }
  }

  constructor(ort, session, description, simulation) {
    this.ort = ort;
    this.session = session;
    this.inputs = description.inputs || [{ name: description.inputName, shape: description.inputShape,
      binding: description.observationBinding || 'observation' }];
    this.outputs = description.outputs || [{ name: description.outputName, shape: description.outputShape,
      binding: description.actionBinding || 'action' }];
    // Older single-input metadata can contain a derived schema without wiring.
    if (this.inputs.length === 1 && !this.inputs[0].binding && !description.feedback?.length) {
      this.inputs = [{ ...this.inputs[0], binding: description.observationBinding || 'observation' }];
    }
    if (this.outputs.length === 1 && !this.outputs[0].binding && !description.feedback?.length) {
      this.outputs = [{ ...this.outputs[0], binding: description.actionBinding || 'action' }];
    }
    this.feedback = description.feedback || [];
    for (const [entries, names] of [[this.inputs, session.inputNames], [this.outputs, session.outputNames]]) {
      if (entries.length !== names.length || entries.some((entry) => !names.includes(entry.name))) {
        throw new Error('Policy tensor names do not match the ONNX session.');
      }
      for (const entry of entries) {
        if (!entry.shape?.length || entry.shape.some((dimension) => !Number.isInteger(dimension) || dimension <= 0)
            || (entry.dtype && entry.dtype !== 'float32')) {
          throw new Error(`Policy tensor ${entry.name} needs a fixed float32 shape.`);
        }
        if (entry.binding && simulation.binding(entry.binding).length !== sizeOf(entry.shape)) {
          throw new Error(`Policy tensor ${entry.name} does not fit binding ${entry.binding}.`);
        }
      }
    }
    for (const feedback of this.feedback) {
      const input = this.inputs.find((entry) => entry.name === feedback.input);
      const output = this.outputs.find((entry) => entry.name === feedback.output);
      if (!input || !output || sizeOf(input.shape) !== sizeOf(output.shape)
          || (feedback.shape && JSON.stringify(feedback.shape) !== JSON.stringify(input.shape))) {
        throw new Error(`Invalid feedback mapping for ${feedback.input}.`);
      }
    }
    for (const input of this.inputs) {
      if (!input.binding && !this.feedback.some((feedback) => feedback.input === input.name)) {
        throw new Error(`Policy input ${input.name} has no simulation binding or feedback.`);
      }
    }
    this.reset();
  }

  reset() {
    this.state = new Map(this.feedback.map((feedback) => {
      const input = this.inputs.find((entry) => entry.name === feedback.input);
      const values = new Float32Array(feedback.initial || sizeOf(input.shape));
      if (values.length !== sizeOf(input.shape)) throw new Error(`Invalid initial state for ${input.name}.`);
      finite(values, input.name);
      return [input.name, values];
    }));
  }

  async evaluate(simulation) {
    const feeds = {};
    for (const input of this.inputs) {
      const values = new Float32Array(this.state.get(input.name) || simulation.binding(input.binding));
      finite(values, input.name);
      feeds[input.name] = new this.ort.Tensor('float32', values, input.shape);
    }
    this.pending = this.session.run(feeds);
    try {
      return await this.pending;
    } finally {
      this.pending = null;
    }
  }

  apply(result, simulation) {
    for (const output of this.outputs) {
      const tensor = result[output.name];
      if (tensor.type !== 'float32' || tensor.data.length !== sizeOf(output.shape)) {
        throw new Error(`Invalid policy output ${output.name}.`);
      }
      finite(tensor.data, output.name);
    }
    for (const output of this.outputs) {
      if (output.binding) simulation.set(output.binding, result[output.name].data);
    }
    for (const feedback of this.feedback) {
      this.state.set(feedback.input, new Float32Array(result[feedback.output].data));
    }
  }

  async dispose() {
    await this.pending?.catch(() => {});
    await this.session.release();
  }
}
