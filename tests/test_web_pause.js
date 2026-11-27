// Exercise the actual browser client using a small DOM and WebSocket harness.
const assert = require('assert');
const fs = require('fs');
const vm = require('vm');

class Element {
  constructor(tag = 'div') {
    this.tagName = tag.toUpperCase(); this.children = []; this.listeners = {};
    this.dataset = {}; this.hidden = false; this.disabled = false; this.value = '';
    this.textContent = ''; this.className = ''; this.attributes = {}; this.style = {};
    this.classList = {toggle() {}};
  }
  addEventListener(type, f) { (this.listeners[type] ||= []).push(f); }
  dispatch(type, details = {}) {
    const event = {type, preventDefault() {}, ...details};
    for (const fn of this.listeners[type] || []) fn(event);
  }
  append(...items) { this.children.push(...items); }
  replaceChildren(...items) { this.children = [...items]; }
  setAttribute(k, v) { this.attributes[k] = v; }
  removeAttribute(k) { delete this.attributes[k]; }
  querySelector() { return new Element(); }
  focus() {}
}
const elements = new Map();
const cards = ['operator', 'observer', 'admin'].map(role => {
  const e = new Element('button'); e.dataset.role = role; return e;
});
const document = {
  hidden: false, activeElement: null,
  getElementById(id) { if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id); },
  createElement(tag) { return new Element(tag); },
  querySelectorAll(selector) { return selector === '.mode-card' ? cards : []; },
  querySelector(selector) { const m = selector.match(/data-role="(\w+)"/); return cards.find(x => x.dataset.role === m?.[1]) || new Element(); },
  addEventListener() {},
};
class FakeSocket {
  static OPEN = 1; static CLOSING = 2; static sockets = [];
  constructor() { this.listeners = {}; this.sent = []; this.readyState = 1; FakeSocket.sockets.push(this); }
  addEventListener(type, f) { (this.listeners[type] ||= []).push(f); }
  dispatch(type, event = {}) { for (const f of this.listeners[type] || []) f(event); }
  send(payload) { this.sent.push(JSON.parse(payload)); }
  close() { this.readyState = 3; }
  message(value) { this.dispatch('message', {data: JSON.stringify(value)}); }
}
const sandbox = {
  document, WebSocket: FakeSocket, location: {protocol: 'https:', host: 'test'},
  window: {addEventListener() {}}, performance: {now: () => 0},
  fetch: async () => ({ok: true, json: async () => ({mode: 'simulation'})}),
  setInterval: () => 1, clearInterval() {}, setTimeout: () => 1, clearTimeout() {},
  confirm: () => true, console,
};
vm.runInNewContext(fs.readFileSync('web/app.js', 'utf8'), sandbox);
const id = (name) => elements.get(name);
const axes = (assigned = false) => Object.fromEntries(['X','Y','Z','ROLL','PITCH','YAW','PINZA'].map(x => [x, {assigned: (x === 'X' || x === 'PINZA') && assigned, generation: assigned && (x === 'X' || x === 'PINZA') ? 1 : null, available:true}]));
const modes = {operator: true, observer: true, admin: true};
const welcome = {type:'welcome', session_id:'session', state:'READY', physical_session:false, read_only:false};
const room = (state, revision, assigned) => ({type:'room_state', state, revision, axes:axes(assigned), modes});

setImmediate(() => {
  const observer = FakeSocket.sockets.at(-1);
  observer.dispatch('open'); observer.message(welcome); observer.message(room('READY', 1, false));
  observer.message({type:'telemetry', state:'READY', telemetry_available:true, finger_source:'simulated', finger_positions:[0,0,0], pose_age_sec:0});
  cards[0].dispatch('click'); id('confirm-mode').dispatch('click');
  assert.strictEqual(id('axis-options').children.length, 7);
  const gripper = id('axis-options').children[6];
  assert.strictEqual(gripper.children[0].textContent, 'PINZA · SIMULADA');
  assert.strictEqual(gripper.disabled, false);
  assert.match(gripper.children[1].textContent, /Disponible/);
  id('axis-options').children[0].dispatch('click');
  id('axis-options').children[6].dispatch('click'); id('enter').dispatch('click');
  const operator = FakeSocket.sockets.at(-1);
  assert.notStrictEqual(operator, observer);
  operator.dispatch('open'); operator.message(welcome);
  operator.message({type:'telemetry', state:'READY', telemetry_available:true, finger_source:'simulated', finger_positions:[0,0,0], pose_age_sec:0});
  operator.message({type:'claim_granted', revision:2, leases:[{axis:'X', lease_id:'lease', generation:1},{axis:'PINZA', lease_id:'finger-lease', generation:1}]});
  operator.message(room('ACTIVE', 2, true));
  operator.message({type:'telemetry', state:'ACTIVE', telemetry_available:true, finger_source:'simulated', finger_positions:[0,0,0], pose_age_sec:0,
    virtual_task:{total:5, delivered:2, held:2, objects:[0,1,2,3,4].map(i=>({id:i,delivered:i<2}))}});
  assert.strictEqual(id('task-progress').textContent, '2 / 5');
  assert.strictEqual(id('task-items').children.length, 5);
  assert.match(id('task-items').children[2].children[1].textContent, /en pinza/);
  assert.match(id('task-note').textContent, /Sujetando Sol/);
  assert.match(id('task-note').textContent, /pulsa −/);
  operator.message({type:'telemetry', state:'ACTIVE', telemetry_available:true, finger_source:'simulated', finger_positions:[0,0,0], pose_age_sec:0, virtual_task:{total:5, delivered:0, held:null, release_reason:'impacto', objects:[0,1,2,3,4].map(i=>({id:i, state:i===0?'caido':'apoyado'}))}});
  assert.match(id('task-items').children[0].children[1].textContent, /caído/);
  assert.match(id('task-items').children[1].children[1].textContent, /apoyado/);
  assert.match(id('task-note').textContent, /presión o impacto/);
  const reset = id('task-items').children[0].children[2];
  assert.strictEqual(reset.disabled, false);
  reset.dispatch('click');
  assert.strictEqual(operator.sent.at(-1).type, 'reset_object');
  assert.strictEqual(operator.sent.at(-1).object_id, 0);
  operator.message({type:'object_reset', object_id:0});
  assert.match(id('dashboard-message').textContent, /Luna volvió/);
  assert.strictEqual(id('controls').children.length, 2);
  let row = id('controls').children[0];
  assert.strictEqual(row.children[2].disabled, false);
  row.children[2].dispatch('pointerdown');
  assert(operator.sent.some(m => m.type === 'input' && m.values.X > 0));
  id('controls').children[1].children[2].dispatch('pointerdown');
  assert(operator.sent.at(-1).values.PINZA > 0);
  const unavailable = room('ACTIVE', 3, true);
  unavailable.axes.PINZA.available = false;
  operator.message(unavailable);
  assert.strictEqual(id('controls').children[1].children[2].disabled, true);
  operator.message(room('ACTIVE', 4, true));
  assert.strictEqual(id('controls').children[1].children[2].disabled, false);
  id('controls').children[1].children[2].dispatch('pointerdown');
  operator.message({type:'telemetry', state:'PAUSED', telemetry_available:true, finger_source:'simulated', finger_positions:[0,0,0], pose_age_sec:0});
  operator.message(room('PAUSED', 5, true));
  row = id('controls').children[0];
  assert.strictEqual(id('pause-banner').hidden, false);
  assert.strictEqual(id('controls').children.length, 2);
  assert.strictEqual(row.children[1].disabled, true);
  assert.strictEqual(row.children[2].disabled, true);
  assert.strictEqual(row.children[3].children[1].disabled, true);
  assert.strictEqual(id('controls').children[1].children[2].disabled, true);
  const during = operator.sent.length;
  row.children[2].dispatch('pointerdown');
  assert.strictEqual(operator.sent.length, during);
  operator.message(room('ACTIVE', 6, true));
  row = id('controls').children[0];
  assert.strictEqual(id('pause-banner').hidden, true);
  assert.strictEqual(row.children[2].disabled, false);
  const after = operator.sent.slice(during).filter(m => m.type === 'input');
  assert(after.length && after.every(m => m.values.X === 0 && m.values.PINZA === 0));
  assert.strictEqual(FakeSocket.sockets.at(-1), operator);
  row.children[2].dispatch('pointerdown');
  assert(operator.sent.at(-1).values.X > 0);
  const simOnly = room('SIM_ONLY', 7, true);
  for (const axis of ['X','Y','Z','ROLL','PITCH','YAW']) {
    simOnly.axes[axis].assigned = false;
    simOnly.axes[axis].available = false;
  }
  operator.message(simOnly);
  assert.strictEqual(id('controls').children.length, 1);
  assert.strictEqual(id('controls').children[0].children[0].children[0].textContent, 'PINZA · SIMULADA');
  assert.strictEqual(id('controls').children[0].children[2].disabled, false);
  id('controls').children[0].children[2].dispatch('pointerdown');
  assert(operator.sent.at(-1).values.PINZA > 0);
  assert.strictEqual(operator.sent.at(-1).values.X, undefined);
  console.log('OK: controles visibles y deshabilitados en pausa, reanudación neutral y misma conexión');
});
