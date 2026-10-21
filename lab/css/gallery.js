// Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

import { demos, activities, physics } from '../demos.js';

const params = new URLSearchParams(location.search);
const embedded = params.get('embed') === '1';
const contextual = embedded && params.has('demos') && params.get('demos') !== 'all';
const content = document.querySelector('#content');
const player = document.querySelector('#inline-player');
const playerHost = document.querySelector('#player-host');
const playerStatus = document.querySelector('#player-status');
let parentOrigin;
let highlightDemo;

function httpOrigin(value) {
  try {
    const url = new URL(value);
    return ['http:', 'https:'].includes(url.protocol) ? url.origin : null;
  } catch { return null; }
}

if (embedded && window.parent !== window) {
  parentOrigin = httpOrigin(document.referrer) || httpOrigin(params.get('parentOrigin'));
}
function notify(type, fields = {}) {
  if (parentOrigin) window.parent.postMessage({ namespace: 'isaaclab-demos', type, ...fields }, parentOrigin);
}

// Natural content height avoids depending on the iframe height sent by its host.
let resizeFrame = 0;
let previousHeight = 0;
function resize() {
  if (!embedded || resizeFrame) return;
  resizeFrame = requestAnimationFrame(() => {
    resizeFrame = 0;
    // Leave two pixels for fractional layout rounding at iframe boundaries.
    const height = Math.max(1, Math.ceil(document.body.getBoundingClientRect().height) + 2);
    if (height !== previousHeight) {
      previousHeight = height;
      notify('resize', { height });
    }
  });
}

if (embedded) {
  const observer = new ResizeObserver(resize);
  observer.observe(document.body);
  window.addEventListener('message', (event) => {
    if (!parentOrigin || event.source !== window.parent || event.origin !== parentOrigin
        || event.data?.namespace !== 'isaaclab-demos') return;
    if (event.data.type === 'theme' && ['light', 'dark'].includes(event.data.theme)) {
      document.documentElement.dataset.theme = event.data.theme;
      const properties = {
        background: ['bg', 'color'], surface: ['surface', 'color'],
        mutedSurface: ['muted-surface', 'color'], text: ['text', 'color'],
        muted: ['muted', 'color'], border: ['border', 'color'], accent: ['accent', 'color'],
        fontFamily: ['font-family', 'font-family'], fontSize: ['font-size', 'font-size'],
        lineHeight: ['line-height', 'line-height'],
      };
      for (const [key, [name, property]] of Object.entries(properties)) {
        const value = event.data.styles?.[key];
        if (typeof value === 'string' && CSS.supports(property, value)) {
          document.documentElement.style.setProperty(`--gallery-${name}`, value);
        }
      }
      resize();
    } else if (event.data.type === 'select' && typeof event.data.demo === 'string') {
      highlightDemo?.(event.data.demo, true);
    }
  });
  const preference = matchMedia('(prefers-color-scheme: dark)');
  document.documentElement.dataset.theme = preference.matches ? 'dark' : 'light';
}

function standaloneURL(demo, query = new URLSearchParams(location.search)) {
  const url = new URL(location.href);
  url.search = query.toString();
  for (const key of ['embed', 'demos', 'parentOrigin']) url.searchParams.delete(key);
  url.searchParams.set('demo', demo.id);
  if (!demo.ropeModels) for (const key of ['rope', 'wheels']) url.searchParams.delete(key);
  return url;
}

// Variants replace the widget, so observe its bubbling events on the persistent host.
playerHost.addEventListener('browser-demo-ready', () => { playerStatus.textContent = ''; resize(); });
playerHost.addEventListener('browser-demo-error', (event) => {
  const message = event.detail?.message || 'Could not start this simulation.';
  playerStatus.textContent = `${message} Try opening the full view.`;
  notify('error', { message });
  resize();
});

function simulationSource(demo, query) {
  const rope = demo.ropeModels?.find((model) => model.id === query.get('rope'));
  const source = rope?.sources[query.get('wheels')] || rope?.sources[1]
    || demo.manifest || `${demo.assetRoot || './demos'}/${demo.id}/manifest.json`;
  const url = new URL(source, new URL('../', import.meta.url));
  return url;
}

async function showStandalone(demo) {
  document.title = `${demo.title} · Isaac Lab`;
  document.querySelector('h1').textContent = demo.title;
  document.querySelector('#description').textContent = `${demo.description} Drag to orbit, scroll to zoom, and use the controls to pause or reset.`;
  await import('./browser-demo.js');
  const widget = document.createElement('isaaclab-browser-demo');
  widget.setAttribute('src', simulationSource(demo, params));
  content.append(widget);
}

function showGallery(pool) {
  document.documentElement.classList.toggle('contextual', contextual);
  content.classList.toggle('single-demo', embedded && pool.length === 1);
  document.querySelector('#gallery-controls').hidden = false;
  const choices = { activity: activities, physics };
  const readFilters = () => {
    const query = new URLSearchParams(location.search);
    return Object.fromEntries(Object.entries(choices).map(([key, values]) =>
      [key, values.some((item) => item.id === query.get(key)) ? query.get(key) : 'all']));
  };
  let filters = readFilters();
  let activeDemo = null;
  let highlighted = null;
  let loadGeneration = 0;
  const cards = [];
  const sections = [];
  const buttons = [];
  const clear = document.querySelector('.clear-filters');
  const matches = (demo, selection = filters) =>
    (selection.activity === 'all' || demo.activity === selection.activity)
    && (selection.physics === 'all' || demo.physics.includes(selection.physics));

  function closePlayer(focus = false) {
    loadGeneration += 1;
    playerHost.replaceChildren(); // disconnectedCallback releases WebGL, WASM and input listeners.
    player.hidden = true;
    if (focus && activeDemo) cards.find(({ item }) => item.id === activeDemo)?.card.focus({ preventScroll: true });
    activeDemo = null;
    for (const { card } of cards) {
      card.classList.remove('is-playing');
      card.removeAttribute('aria-current');
    }
    resize();
  }
  document.querySelector('#close-player').addEventListener('click', () => closePlayer(true));

  async function startDemo(item, card) {
    closePlayer();
    activeDemo = item.id;
    const generation = loadGeneration;
    card.classList.add('is-playing');
    card.setAttribute('aria-current', 'true');
    document.querySelector('#player-title').textContent = item.title;
    document.querySelector('#player-description').textContent = item.description;
    document.querySelector('#player-standalone').href = card.href;
    playerStatus.textContent = 'Loading simulation…';
    player.hidden = false;
    player.focus({ preventScroll: true });
    player.scrollIntoView({ block: 'start', behavior: 'instant' });
    resize();
    try {
      await import('./browser-demo.js');
      if (generation !== loadGeneration) return;
      const widget = document.createElement('isaaclab-browser-demo');
      widget.setAttribute('src', simulationSource(item, new URL(card.href).searchParams));
      playerHost.append(widget);
      const query = new URLSearchParams(location.search);
      query.set('demo', item.id);
      history.replaceState(null, '', `?${query}`);
    } catch (error) {
      if (generation !== loadGeneration) return;
      playerStatus.textContent = `Could not start: ${error.message}. Try opening the full view.`;
      notify('error', { message: error.message });
      resize();
    }
  }

  for (const activity of activities) {
    const members = pool.filter((demo) => demo.activity === activity.id);
    if (!members.length) continue;
    const section = document.createElement('section');
    section.className = 'demo-section';
    const heading = document.createElement('h2');
    heading.id = `section-${activity.id}`;
    section.setAttribute('aria-labelledby', heading.id);
    heading.textContent = activity.label;
    const count = document.createElement('span');
    count.className = 'section-count';
    heading.append(count);
    const help = document.createElement('p');
    help.className = 'section-description';
    help.textContent = activity.description;
    const grid = document.createElement('div');
    grid.className = 'grid';
    for (const item of members) {
      const card = document.querySelector('#demo-card').content.firstElementChild.cloneNode(true);
      card.dataset.demo = item.id;
      card.id = `demo-${item.id}`;
      card.querySelector('img').src = item.preview || `${item.assetRoot || './demos'}/${item.id}/preview.jpg`;
      card.querySelector('strong').textContent = item.title;
      card.querySelector('p').textContent = item.description;
      if (embedded) {
        card.querySelector('.card-arrow').textContent = '\u25B6\uFE0E';
        card.setAttribute('aria-label', `Run ${item.title}`);
        card.addEventListener('click', (event) => {
          if (event.button !== 0 || event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
          event.preventDefault();
          startDemo(item, card);
        });
      }
      const tags = [
        { label: activity.label, className: `badge badge-${activity.id}` },
        ...item.physics.map((id) => ({ label: physics.find((value) => value.id === id).label, className: 'badge' })),
      ];
      if (item.manualControl) tags.push({ label: 'Manual control', className: 'badge badge-manual' });
      for (const tag of tags) {
        const badge = document.createElement('span');
        badge.className = tag.className;
        badge.textContent = tag.label;
        card.querySelector('.card-tags').append(badge);
      }
      grid.append(card);
      cards.push({ item, card });
    }
    section.append(heading, help, grid);
    content.append(section);
    sections.push({ activity, section, count });
  }
  const empty = document.createElement('p');
  empty.className = 'empty';
  empty.textContent = 'No demos match these filters. Clear the filters to explore all demos.';
  content.append(empty);

  function updateGallery() {
    const query = new URLSearchParams(location.search);
    for (const key of ['demo', 'rope', 'wheels']) query.delete(key);
    for (const key of Object.keys(choices)) {
      filters[key] === 'all' ? query.delete(key) : query.set(key, filters[key]);
    }
    let count = 0;
    for (const { item, card } of cards) {
      card.hidden = !matches(item);
      if (!card.hidden) count += 1;
      const link = new URLSearchParams(query);
      if (filters.physics === 'vbd' && item.ropeModels) link.set('rope', 'vbd');
      card.href = standaloneURL(item, link);
    }
    for (const { activity, section, count: label } of sections) {
      const visible = pool.filter((item) => item.activity === activity.id && matches(item)).length;
      section.hidden = visible === 0;
      label.textContent = visible;
    }
    for (const { key, option, button, label } of buttons) {
      const available = pool.filter((item) => matches(item, { ...filters, [key]: option.id })).length;
      button.setAttribute('aria-pressed', String(filters[key] === option.id));
      button.setAttribute('aria-label', `${option.label}: ${available} ${available === 1 ? 'demo' : 'demos'}`);
      button.disabled = available === 0 && option.id !== 'all' && filters[key] !== option.id;
      label.textContent = available;
    }
    document.querySelector('#result-count').textContent = `${count} of ${pool.length} demos${embedded && !contextual ? ' · Select one to run' : ''}`;
    clear.hidden = Object.values(filters).every((value) => value === 'all');
    document.querySelector('#gallery-controls').hidden = contextual
      && document.querySelector('.filters').hidden && count === pool.length;
    empty.hidden = count > 0;
    if (activeDemo && !matches(pool.find((item) => item.id === activeDemo))) closePlayer();
    resize();
    return query;
  }
  for (const [key, values] of Object.entries(choices)) {
    const available = values.filter((value) => pool.some((item) => key === 'physics' ? item.physics.includes(value.id) : item.activity === value.id));
    const row = document.querySelector(`#${key}-filters`).closest('.filter-row');
    row.hidden = embedded && available.length < 2;
    for (const option of [{ id: 'all', label: 'All' }, ...(embedded ? available : values)]) {
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'filter-pill';
      button.textContent = option.label;
      if (option.description) button.title = option.description;
      const label = document.createElement('span');
      label.className = 'filter-count';
      button.append(label);
      button.addEventListener('click', () => {
        if (filters[key] === option.id) return;
        filters[key] = option.id;
        history.pushState(null, '', `?${updateGallery()}`);
      });
      document.querySelector(`#${key}-filters`).append(button);
      buttons.push({ key, option, button, label });
    }
  }
  document.querySelector('.filters').hidden = embedded
    && [...document.querySelectorAll('.filter-row')].every((row) => row.hidden);
  clear.addEventListener('click', () => {
    filters = { activity: 'all', physics: 'all' };
    history.pushState(null, '', `?${updateGallery()}`);
  });
  window.addEventListener('popstate', () => { filters = readFilters(); updateGallery(); });

  highlightDemo = (id, scroll = false) => {
    const selected = cards.find(({ item }) => item.id === id);
    if (!selected) return;
    if (activeDemo && activeDemo !== id) closePlayer();
    if (!matches(selected.item)) {
      filters = { activity: 'all', physics: 'all' };
      history.replaceState(null, '', `?${updateGallery()}`);
    }
    highlighted?.classList.remove('is-highlighted');
    highlighted = selected.card;
    highlighted.classList.add('is-highlighted');
    highlighted.focus({ preventScroll: true });
    if (scroll) highlighted.scrollIntoView({ block: 'nearest', behavior: 'instant' });
    resize();
  };
  updateGallery();
  if (embedded && params.has('demo')) highlightDemo(params.get('demo'));
}

try {
  const galleryQuery = new URLSearchParams(params);
  for (const key of ['demo', 'rope', 'wheels']) galleryQuery.delete(key);
  document.querySelector('#gallery-link').href = `?${galleryQuery}`;
  const selected = demos.find((item) => item.id === params.get('demo'));
  if (!embedded && selected) await showStandalone(selected);
  else {
    let pool = demos;
    if (embedded && params.has('demos') && params.get('demos') !== 'all') {
      const ids = new Set(params.get('demos').split(',').map((id) => id.trim()).filter(Boolean));
      const unknown = [...ids].filter((id) => !demos.some((item) => item.id === id));
      if (!ids.size || unknown.length) throw new Error(`Unknown demo selection${unknown.length ? `: ${unknown.join(', ')}` : ''}`);
      pool = demos.filter((demo) => ids.has(demo.id));
    }
    showGallery(pool);
    notify('ready');
    resize();
  }
} catch (error) {
  const message = document.createElement('p');
  message.className = 'empty gallery-error';
  message.setAttribute('role', 'alert');
  message.textContent = `Could not load the demo gallery: ${error.message}`;
  content.replaceChildren(message);
  notify('error', { message: error.message });
  resize();
}
