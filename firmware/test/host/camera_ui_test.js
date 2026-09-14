'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

class FakeClassList {
  constructor(element) { this.element = element; }
  values() { return this.element.className.split(/\s+/).filter(Boolean); }
  contains(name) { return this.values().includes(name); }
  add(name) {
    if (!this.contains(name)) this.element.className = [...this.values(), name].join(' ');
  }
  remove(name) { this.element.className = this.values().filter((item) => item !== name).join(' '); }
  toggle(name, force) {
    const enabled = force === undefined ? !this.contains(name) : !!force;
    if (enabled) this.add(name); else this.remove(name);
    return enabled;
  }
}

function matches(element, selector) {
  if (selector === '*') return true;
  if (selector.startsWith('#')) return element.id === selector.slice(1);
  if (selector.startsWith('.')) return element.classList.contains(selector.slice(1));
  const dataMatch = selector.match(/^\[data-([a-z-]+)\]$/);
  if (dataMatch) return Object.prototype.hasOwnProperty.call(element.dataset, dataMatch[1]);
  return element.tagName.toLowerCase() === selector.toLowerCase();
}

class FakeElement {
  constructor(tagName, document) {
    this.tagName = tagName.toUpperCase();
    this.ownerDocument = document;
    this.children = [];
    this.parentNode = null;
    this.id = '';
    this.className = '';
    this.dataset = {};
    this.attributes = new Map();
    this.listeners = new Map();
    this.hidden = false;
    this.disabled = false;
    this.draggable = true;
    this._textContent = '';
    this.classList = new FakeClassList(this);
  }

  appendChild(child) {
    child.parentNode = this;
    this.children.push(child);
    return child;
  }

  insertBefore(child, reference) {
    child.parentNode = this;
    const index = this.children.indexOf(reference);
    if (index < 0) this.children.push(child); else this.children.splice(index, 0, child);
    return child;
  }

  remove() {
    if (!this.parentNode) return;
    const index = this.parentNode.children.indexOf(this);
    if (index >= 0) this.parentNode.children.splice(index, 1);
    this.parentNode = null;
  }

  setAttribute(name, value) {
    const text = String(value);
    this.attributes.set(name, text);
    if (name === 'id') this.id = text;
    if (name === 'class') this.className = text;
    if (name.startsWith('data-')) this.dataset[name.slice(5)] = text;
  }

  removeAttribute(name) {
    this.attributes.delete(name);
    if (name === 'src') this._src = '';
  }

  get src() { return this._src || ''; }
  set src(value) { this._src = String(value); this.attributes.set('src', this._src); }

  get textContent() {
    return this._textContent + this.children.map((child) => child.textContent).join('');
  }
  set textContent(value) { this._textContent = String(value); }

  set innerHTML(html) {
    this.children = [];
    const stack = [this];
    for (const token of html.match(/<[^>]+>|[^<]+/g) || []) {
      if (token.startsWith('</')) {
        stack.pop();
        continue;
      }
      if (!token.startsWith('<')) {
        const text = token.trim();
        if (text) stack[stack.length - 1]._textContent += text;
        continue;
      }
      const tagMatch = token.match(/^<([a-z0-9-]+)/i);
      if (!tagMatch) continue;
      const element = this.ownerDocument.createElement(tagMatch[1]);
      const attributeText = token.slice(tagMatch[0].length, token.length - 1);
      const attributePattern = /([:\w-]+)(?:="([^"]*)")?/g;
      let match;
      while ((match = attributePattern.exec(attributeText))) element.setAttribute(match[1], match[2] || '');
      stack[stack.length - 1].appendChild(element);
      if (!token.endsWith('/>')) stack.push(element);
    }
  }

  querySelectorAll(selector) {
    const result = [];
    const visit = (element) => {
      for (const child of element.children) {
        if (matches(child, selector)) result.push(child);
        visit(child);
      }
    };
    visit(this);
    return result;
  }

  querySelector(selector) { return this.querySelectorAll(selector)[0] || null; }

  addEventListener(type, listener) {
    if (!this.listeners.has(type)) this.listeners.set(type, []);
    this.listeners.get(type).push(listener);
  }

  dispatchEvent(event) {
    event.target = this;
    if (!event.preventDefault) event.preventDefault = () => { event.defaultPrevented = true; };
    for (const listener of this.listeners.get(event.type) || []) listener.call(this, event);
    return !event.defaultPrevented;
  }

  click() { this.dispatchEvent({type: 'click'}); }
}

class FakeDocument {
  constructor() {
    this.head = new FakeElement('head', this);
    this.body = new FakeElement('body', this);
    this.hidden = false;
    this.listeners = new Map();
    const header = this.createElement('div');
    header.className = 'header-actions-right';
    this.body.appendChild(header);
  }
  createElement(tagName) { return new FakeElement(tagName, this); }
  getElementById(id) {
    return [this.head, this.body].flatMap((root) => [root, ...root.querySelectorAll('*')]).find((item) => item.id === id) || null;
  }
  querySelectorAll(selector) {
    return [...this.head.querySelectorAll(selector), ...this.body.querySelectorAll(selector)];
  }
  querySelector(selector) { return this.querySelectorAll(selector)[0] || null; }
  addEventListener(type, listener) {
    if (!this.listeners.has(type)) this.listeners.set(type, []);
    this.listeners.get(type).push(listener);
  }
  dispatchEvent(event) {
    for (const listener of this.listeners.get(event.type) || []) listener.call(this, event);
  }
}

async function flushPromises() {
  await Promise.resolve();
  await Promise.resolve();
}

async function main() {
  const document = new FakeDocument();
  const timers = new Map();
  let nextTimerId = 1;
  let caps = {
    peripherals: {camera: {available: true, streamUrl: 'http://192.168.4.2:81/stream', activeProfile: 'qvga', profilePending: false}}
  };
  const movements = [];
  let stops = 0;

  const window = {
    setInterval: () => 1,
    addEventListener: () => {},
    CameraUi: null
  };
  const context = {
    window,
    document,
    console,
    Date,
    setTimeout(callback) {
      const id = nextTimerId++;
      timers.set(id, callback);
      return id;
    },
    clearTimeout(id) { timers.delete(id); },
    fetch: async () => ({ok: true, json: async () => caps})
  };
  vm.createContext(context);
  const source = fs.readFileSync(path.join(__dirname, '../../data/camera_ui.js'), 'utf8');
  vm.runInContext(source, context, {filename: 'camera_ui.js'});

  window.CameraUi.init({
    sendMovement: (mode) => movements.push(mode),
    sendSpeed: () => {},
    stop: () => { stops += 1; },
    guardLowBattery: () => false,
    getMotionButtonMode: () => 'continuous'
  });
  await flushPromises();
  window.CameraUi.applyCaps(caps);

  const entry = document.getElementById('cameraEntry');
  assert(entry && !entry.hidden, 'camera entry should be visible');
  assert.deepEqual(
    document.querySelectorAll('[data-motion]').map((button) => Number(button.dataset.motion)).sort((a, b) => a - b),
    [1, 3, 4, 5, 6, 7]
  );

  entry.click();
  const firstStream = document.getElementById('cameraStream');
  assert(firstStream && firstStream.src.startsWith('http://192.168.4.2:81/stream?t='));
  firstStream.dispatchEvent({type: 'load'});
  assert.equal(document.getElementById('cameraStatus').textContent, '图传已连接');

  document.querySelector('[data-motion]').click();
  assert.deepEqual(movements, [1]);
  document.querySelector('[data-motion]').dispatchEvent({type: 'pointerup'});
  assert.equal(stops, 0, 'button release must not stop continuous or single-cycle commands');

  window.CameraUi.setMotionButtonMode('single_cycle');
  assert.equal(document.getElementById('cameraMotionMode').textContent, '当前：单周期');
  document.querySelectorAll('[data-motion]').find((button) => button.dataset.motion === '3').click();
  assert.deepEqual(movements, [1, 3]);
  assert.equal(stops, 0);

  document.getElementById('cameraExit').click();
  assert.equal(stops, 1);
  assert.equal(document.getElementById('cameraStream'), null);
  entry.click();
  const secondStream = document.getElementById('cameraStream');
  assert(secondStream && secondStream !== firstStream, 're-entry must create a new image element');
  firstStream.dispatchEvent({type: 'error'});
  assert.equal(stops, 1, 'stale stream errors must not affect the current session');
  assert.equal(document.getElementById('cameraStream'), secondStream);

  caps = {
    peripherals: {camera: {available: true, streamUrl: 'http://192.168.4.3:81/stream', activeProfile: 'qvga', profilePending: false}}
  };
  window.CameraUi.applyCaps(caps);
  const changedStream = document.getElementById('cameraStream');
  assert(changedStream && changedStream !== secondStream);
  assert(changedStream.src.startsWith('http://192.168.4.3:81/stream?t='));

  changedStream.dispatchEvent({type: 'error'});
  assert.equal(stops, 2, 'current stream errors should stop robot motion');
  assert.equal(document.getElementById('cameraStream'), null);
  assert.equal(timers.size, 1);
  const retry = timers.values().next().value;
  timers.clear();
  retry();
  assert(document.getElementById('cameraStream'), 'retry should create a fresh stream element');

  console.log('camera_ui_test: PASS');
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
