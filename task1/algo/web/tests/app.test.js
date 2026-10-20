// Regression tests for the real algo/web/app.js, driven through a minimal
// fake DOM, a controlled `fetch` and controlled timers.
//
// Run with:
//   node --test algo/web/tests/app.test.js
//
// Zero dependencies (Node built-ins only), no server, no network. Each test
// loads a fresh instance of app.js (a unique import URL gives it its own
// module state) and restores every global it touched afterwards.

import { test, describe, beforeEach, afterEach } from 'node:test';
import assert from 'node:assert/strict';
import { setImmediate as nextTick } from 'node:timers/promises';
import { pathToFileURL } from 'node:url';
import { resolve } from 'node:path';

const APP_URL = pathToFileURL(resolve(import.meta.dirname, '../app.js')).href;
const GLOBALS = ['document', 'fetch', 'setInterval', 'clearInterval'];

// ---------------------------------------------------------------------------
// Fixtures: minimal but schema-shaped responses
// ---------------------------------------------------------------------------

const pose = (y) => ({ x_cm: 20, y_cm: y, heading_deg: 90 });

/** 5 waypoints; two MOVs sharing boundary waypoint 2..4, then a SNAP that
 * reuses the last waypoint => 7 preview steps. */
function v2Plan(missionId) {
  return {
    type: 'PLAN_RESULT',
    protocol_version: 2,
    mission_id: missionId,
    mode: 'B2',
    valid: true,
    complete: true,
    obstacle_order: [1],
    skipped_obstacle_ids: [],
    target_poses: [pose(60)],
    final_pose: pose(60),
    actions: [
      {
        seq: 0, type: 'MOV', dir: 'F', value: 20, unit: 'cm', stm_command: 'MOV,F,20',
        start_pose: pose(20), end_pose: pose(40), waypoint_start_index: 0, waypoint_end_index: 2,
      },
      {
        seq: 1, type: 'MOV', dir: 'F', value: 20, unit: 'cm', stm_command: 'MOV,F,20',
        start_pose: pose(40), end_pose: pose(60), waypoint_start_index: 2, waypoint_end_index: 4,
      },
      { seq: 2, type: 'SNAP', obstacle_id: 1, pose: pose(60) },
    ],
    waypoints: [20, 30, 40, 50, 60].map(pose),
    total_distance_cm: 40,
    total_cost: 40,
    failure_reason: null,
  };
}
const V2_STEPS = '7';

/** Two obstacles: MOV (steps 0-2), SNAP O1 (3), MOV (4-6), SNAP O2 (7). */
function v2PlanTwoObstacles(missionId) {
  const plan = v2Plan(missionId);
  const mov = (seq, from, to, start, end) => ({
    seq, type: 'MOV', dir: 'F', value: 20, unit: 'cm', stm_command: 'MOV,F,20',
    start_pose: pose(start), end_pose: pose(end), waypoint_start_index: from, waypoint_end_index: to,
  });
  return {
    ...plan,
    obstacle_order: [1, 2],
    target_poses: [pose(40), pose(60)],
    actions: [
      mov(0, 0, 2, 20, 40),
      { seq: 1, type: 'SNAP', obstacle_id: 1, pose: pose(40) },
      mov(2, 2, 4, 40, 60),
      { seq: 3, type: 'SNAP', obstacle_id: 2, pose: pose(60) },
    ],
  };
}
const TWO_OBSTACLE_STEPS = '8';

/** 4 waypoints => "n / 4" in the step readout. */
function v1Plan() {
  const p = (y) => ({ x_cm: 20, y_cm: y, heading_rad: Math.PI / 2 });
  return {
    type: 'PLAN_RESULT',
    protocol_version: 1,
    valid: true,
    complete: true,
    mode: 'B2',
    obstacle_order: [1],
    skipped_obstacle_ids: [],
    target_poses: [p(60)],
    waypoints: [20, 30, 40, 60].map(p),
    commands: [{ type: 'FW', value: 40 }, { type: 'SNAP', obstacle_id: 1 }],
    estimated_time_s: 5,
  };
}
const V1_STEPS = '4';

// ---------------------------------------------------------------------------
// Minimal fake browser environment
// ---------------------------------------------------------------------------

const jsonResponse = (status, body) => ({ status, ok: status < 400, json: async () => body });
const missionOf = (opts) => JSON.parse(opts.body).mission_id;

function deferred() {
  let resolveFn;
  const promise = new Promise((r) => (resolveFn = r));
  return { promise, resolve: resolveFn };
}

async function startApp() {
  const saved = Object.fromEntries(GLOBALS.map((k) => [k, globalThis[k]]));
  const elements = new Map();
  const created = [];
  const htmlWrites = [];
  const timers = new Map();
  const queues = { '/plan': [], '/plan/v2': [] };
  let timerSeq = 0;

  class FakeElement {
    constructor(tag) {
      this.tag = tag;
      this.children = [];
      this.attrs = {};
      this.style = {};
      this.dataset = {};
      this.handlers = {};
      this.textContent = '';
      this.className = '';
      this.hidden = false;
      this.disabled = false;
      this.checked = false;
      this.value = '';
      const self = this;
      this.classList = {
        add(c) { if (!self.className.split(' ').includes(c)) self.className += ` ${c}`; },
        remove(c) { self.className = self.className.split(' ').filter((x) => x !== c).join(' '); },
        toggle(c, on) { if (on) this.add(c); else this.remove(c); },
      };
    }
    set innerHTML(html) { htmlWrites.push(html); this.children = []; }
    get innerHTML() { return htmlWrites.at(-1) ?? ''; }
    setAttribute(k, v) { this.attrs[k] = v; }
    appendChild(child) { this.children.push(child); return child; }
    append(...kids) { this.children.push(...kids); }
    replaceChildren(...kids) { this.children = [...kids]; }
    addEventListener(type, fn) { this.handlers[type] = fn; }
    getBoundingClientRect() { return { left: 0, top: 0, width: 1000, height: 1000 }; }
  }

  const make = (tag) => { const el = new FakeElement(tag); created.push(el); return el; };
  const engineButtons = ['v1', 'v2'].map((engine) => Object.assign(make('button'), { dataset: { engine } }));
  const faceButtons = ['N', 'E', 'S', 'W'].map((face) => Object.assign(make('button'), { dataset: { face } }));

  globalThis.document = {
    getElementById(id) {
      if (!elements.has(id)) elements.set(id, make('element'));
      return elements.get(id);
    },
    createElement: make,
    createElementNS: (_ns, tag) => make(tag),
    querySelectorAll(selector) {
      if (selector === '[data-engine]') return engineButtons;
      if (selector === '[data-face]') return faceButtons;
      return [];
    },
  };
  globalThis.setInterval = (fn) => { timers.set(++timerSeq, fn); return timerSeq; };
  globalThis.clearInterval = (id) => { timers.delete(id); };
  // Only /plan and /plan/v2 are scripted; the state pollers get a harmless 404.
  globalThis.fetch = async (url, opts) => {
    const queue = queues[url];
    if (!queue) return jsonResponse(404, {});
    const handler = queue.shift();
    if (!handler) throw new Error(`unexpected ${url} request`);
    return handler(opts);
  };

  await import(`${APP_URL}?instance=${startApp.count++}`);

  const $ = (id) => globalThis.document.getElementById(id);
  return {
    $,
    created,
    htmlWrites,
    timers,
    queue: (url, handler) => queues[url].push(handler),
    // Cell (cx, cy) is 10 cm; click its centre.
    clickCell: (cx, cy) => $('map').handlers.click({ clientX: cx * 50 + 5, clientY: 1000 - (cy * 50 + 5) }),
    chooseEngine: (engine) => engineButtons.find((b) => b.dataset.engine === engine).handlers.click(),
    run: () => $('runB2').onclick(),
    settle: async () => { await nextTick(); await nextTick(); },
    commandRows: () => $('commands').children.map((row) => row.children.map((cell) => cell.textContent)),
    // 'active' | 'done' | 'pending' for every rendered command row.
    commandStates: () =>
      $('commands').children.map((row) => {
        const classes = row.className.split(' ');
        return classes.includes('active') ? 'active' : classes.includes('done') ? 'done' : 'pending';
      }),
    scrub: (position) => { $('timeline').value = String(position); $('timeline').handlers.input(); },
    restore() {
      timers.clear();
      for (const key of GLOBALS) {
        if (saved[key] === undefined) delete globalThis[key];
        else globalThis[key] = saved[key];
      }
    },
  };
}
startApp.count = 0;

/** Plans a successful v2 route with one obstacle and waits for it to show. */
async function planV2Successfully(app, { plan = v2Plan, cells = [[6, 10]], steps = V2_STEPS } = {}) {
  app.chooseEngine('v2');
  for (const [cx, cy] of cells) app.clickCell(cx, cy);
  app.queue('/plan/v2', async (opts) => jsonResponse(200, plan(missionOf(opts))));
  app.run();
  await app.settle();
  assert.equal(app.$('status').textContent, 'Route ready');
  assert.equal(app.$('step').textContent, `1 / ${steps}`);
}

let app;
beforeEach(async () => { app = await startApp(); });
afterEach(() => { app.restore(); });

// ---------------------------------------------------------------------------
// Stale-response guard
// ---------------------------------------------------------------------------

describe('stale v2 responses', () => {
  test('a map change while planning discards the old response', async () => {
    app.chooseEngine('v2');
    app.clickCell(6, 10);
    const gate = deferred();
    app.queue('/plan/v2', async (opts) => { await gate.promise; return jsonResponse(200, v2Plan(missionOf(opts))); });
    app.run();
    await app.settle();
    assert.equal(app.$('runB2').disabled, true, 'submitting while the request is pending');

    app.clickCell(12, 5); // map mutation invalidates the pending request
    gate.resolve();
    await app.settle();

    assert.equal(app.$('resultMessage').textContent, 'Map changed while planning; result discarded.');
    assert.equal(app.$('step').textContent, '0 / 0');
    assert.equal(app.$('status').textContent, 'Ready');
    assert.equal(app.$('runB2').disabled, false, 'submitting released');
    assert.equal(app.$('commands').children[0].textContent, 'No actions.');
  });

  test('an older response cannot modify a newer request', async () => {
    app.chooseEngine('v2');
    app.clickCell(6, 10);
    const oldGate = deferred();
    const newGate = deferred();
    app.queue('/plan/v2', async (opts) => { await oldGate.promise; return jsonResponse(200, v2Plan(missionOf(opts))); });
    app.run(); // request A
    await app.settle();

    app.clickCell(12, 5); // invalidates A
    app.queue('/plan/v2', async (opts) => { await newGate.promise; return jsonResponse(200, v2Plan(missionOf(opts))); });
    app.run(); // request B, started while A is unresolved
    await app.settle();

    oldGate.resolve(); // A answers first
    await app.settle();
    assert.equal(app.$('planningStatusBadge').textContent, 'planning');
    assert.ok(app.$('resultMessage').textContent.startsWith('Planning'));
    assert.equal(app.$('step').textContent, '0 / 0', 'A drew nothing');
    assert.equal(app.$('runB2').disabled, true, 'A did not release B\'s submitting state');

    newGate.resolve();
    await app.settle();
    assert.equal(app.$('status').textContent, 'Route ready');
    assert.equal(app.$('step').textContent, `1 / ${V2_STEPS}`);
  });
});

// ---------------------------------------------------------------------------
// Engine switching during playback
// ---------------------------------------------------------------------------

describe('engine switching', () => {
  test('stops the old engine and an old callback cannot advance the new one', async () => {
    app.clickCell(6, 10);
    app.queue('/plan', async () => jsonResponse(200, v1Plan()));
    app.run(); // v1 is the default engine
    await app.settle();
    assert.equal(app.$('step').textContent, `1 / ${V1_STEPS}`);

    app.chooseEngine('v2');
    app.queue('/plan/v2', async (opts) => jsonResponse(200, v2Plan(missionOf(opts))));
    app.run();
    await app.settle();

    const before = new Set(app.timers.keys());
    app.$('play').onclick();
    const [timerId] = [...app.timers.keys()].filter((id) => !before.has(id));
    const staleCallback = app.timers.get(timerId);
    staleCallback();
    staleCallback();
    assert.equal(app.$('step').textContent, `3 / ${V2_STEPS}`, 'v2 playback advances');

    app.chooseEngine('v1');
    assert.equal(app.timers.has(timerId), false, 'old engine timer cleared');
    assert.equal(app.$('step').textContent, `1 / ${V1_STEPS}`);
    assert.equal(app.$('play').textContent, 'Play');

    staleCallback();
    staleCallback();
    staleCallback();
    assert.equal(app.$('step').textContent, `1 / ${V1_STEPS}`, 'v1 not advanced by the stale v2 callback');

    app.chooseEngine('v2');
    assert.equal(app.$('play').textContent, 'Play', 'v2 playback is stopped, not still "playing"');
  });
});

// ---------------------------------------------------------------------------
// Plain-text rendering
// ---------------------------------------------------------------------------

describe('backend text is never injected as HTML', () => {
  test('HTML-like text in a model error renders as text and creates no elements', async () => {
    const evil = '<img src=x onerror=alert(1)>';
    app.chooseEngine('v2');
    app.clickCell(6, 10);
    app.queue('/plan/v2', async (opts) =>
      jsonResponse(200, { ...v2Plan(missionOf(opts)), waypoints: [], actions: [{ seq: 0, type: evil }] }),
    );
    app.run();
    await app.settle();

    assert.equal(app.$('status').textContent, 'Planning request failed');
    const box = app.$('commands');
    assert.equal(box.children.length, 1);
    assert.ok(box.children[0].textContent.includes(evil), 'shown literally');
    assert.equal(box.children[0].children.length, 0, 'no child elements');
    assert.equal(app.created.some((el) => el.tag === 'img'), false, 'no <img> element created');
    assert.equal(app.htmlWrites.some((html) => html.includes('<img') || html.includes('onerror')), false, 'never via innerHTML');
  });
});

// ---------------------------------------------------------------------------
// Failed attempts clear the previous route
// ---------------------------------------------------------------------------

describe('failed v2 attempts', () => {
  const cases = [
    ['PLAN_ERROR', async () => jsonResponse(400, { type: 'PLAN_ERROR', error: 'bad request' }), 'Planning request failed: bad request'],
    ['HTTP 409', async () => jsonResponse(409, { type: 'PLAN_ERROR', error: 'busy' }), 'planner is busy'],
    ['non-JSON body', async () => ({ status: 200, ok: true, json: async () => { throw new SyntaxError('x'); } }), 'not valid JSON'],
    ['network failure', async () => { throw new TypeError('fetch failed'); }, 'Could not connect to planner: fetch failed'],
    [
      'malformed schema',
      async (opts) => jsonResponse(200, { ...v2Plan(missionOf(opts)), waypoints: 'not-an-array' }),
      'Planning request failed: unexpected response',
    ],
    ['wrong response type', async () => jsonResponse(200, { type: 'SOMETHING_ELSE' }), 'not a PLAN_RESULT'],
    [
      'mismatched mission_id',
      async () => jsonResponse(200, { ...v2Plan('some-other-mission') }),
      'mission_id does not match',
    ],
  ];

  for (const [name, handler, expectedText] of cases) {
    test(`${name} clears the previous route and releases submitting`, async () => {
      await planV2Successfully(app);
      assert.ok(app.$('copyActions').disabled === false, 'a route exists before the failure');

      app.queue('/plan/v2', handler);
      app.run();
      await app.settle();

      assert.equal(app.$('status').textContent, 'Planning request failed');
      assert.ok(app.$('resultMessage').textContent.includes(expectedText), app.$('resultMessage').textContent);
      assert.equal(app.$('step').textContent, '0 / 0', 'preview cleared');
      assert.equal(app.$('route').children.length, 0, 'obstacle order cleared');
      assert.equal(app.$('copyActions').disabled, true, 'no actions left to copy');
      assert.equal(app.$('runB2').disabled, false, 'submitting released');
      assert.equal(app.$('planningStatusBadge').textContent, 'error');
      assert.equal(app.$('commands').children[0].textContent, app.$('resultMessage').textContent);
    });
  }
});

// ---------------------------------------------------------------------------
// Command rows and labels
// ---------------------------------------------------------------------------

describe('command rows', () => {
  test('a missing stm_command renders a dash, never "undefined"', async () => {
    app.chooseEngine('v2');
    app.clickCell(6, 10);
    app.queue('/plan/v2', async (opts) => {
      const plan = v2Plan(missionOf(opts));
      delete plan.actions[0].stm_command;
      return jsonResponse(200, plan);
    });
    app.run();
    await app.settle();

    const rows = app.commandRows();
    assert.deepEqual(rows[0], ['#0 MOV', '— · waypoints 0-2']);
    assert.deepEqual(rows[1], ['#1 MOV', 'MOV,F,20 · waypoints 2-4']);
    assert.deepEqual(rows[2], ['#2 SNAP', 'obstacle 1'], 'SNAP has no waypoint range');
    assert.equal(rows.flat().some((text) => /undefined|NaN/.test(text)), false);
  });
});

describe('visited label', () => {
  test('is "Visited (preview)" for v2 and "Visited" for v1', () => {
    assert.equal(app.$('visitedLabel').textContent, 'Visited');
    app.chooseEngine('v2');
    assert.equal(app.$('visitedLabel').textContent, 'Visited (preview)');
    app.chooseEngine('v1');
    assert.equal(app.$('visitedLabel').textContent, 'Visited');
  });
});

// ---------------------------------------------------------------------------
// Command highlighting follows the preview position
// ---------------------------------------------------------------------------

describe('command highlighting follows the v2 preview', () => {
  // v2Plan steps: MOV #0 = 0-2, MOV #1 = 3-5, SNAP #2 = 6.
  test('scrubbing from command 0 into later commands moves active/done', async () => {
    await planV2Successfully(app);
    assert.deepEqual(app.commandStates(), ['active', 'pending', 'pending']);

    app.scrub(2);
    assert.deepEqual(app.commandStates(), ['active', 'pending', 'pending']);
    app.scrub(3);
    assert.deepEqual(app.commandStates(), ['done', 'active', 'pending']);
    app.scrub(5);
    assert.deepEqual(app.commandStates(), ['done', 'active', 'pending']);
    app.scrub(0);
    assert.deepEqual(app.commandStates(), ['active', 'pending', 'pending'], 'scrubbing back un-completes');
  });

  test('Next and Previous update the highlighting', async () => {
    await planV2Successfully(app);
    for (let i = 0; i < 3; i++) app.$('next').onclick();
    assert.equal(app.$('step').textContent, `4 / ${V2_STEPS}`);
    assert.deepEqual(app.commandStates(), ['done', 'active', 'pending']);

    app.$('previous').onclick();
    assert.deepEqual(app.commandStates(), ['active', 'pending', 'pending']);
  });

  test('Play updates the highlighting step by step and finishes with everything done', async () => {
    await planV2Successfully(app);
    const before = new Set(app.timers.keys());
    app.$('play').onclick();
    const [timerId] = [...app.timers.keys()].filter((id) => !before.has(id));
    const tick = app.timers.get(timerId);

    for (let i = 0; i < 3; i++) tick();
    assert.equal(app.$('step').textContent, `4 / ${V2_STEPS}`);
    assert.deepEqual(app.commandStates(), ['done', 'active', 'pending']);

    for (let i = 0; i < 3; i++) tick();
    assert.equal(app.$('step').textContent, `7 / ${V2_STEPS}`);
    assert.deepEqual(app.commandStates(), ['done', 'done', 'done']);
  });

  test('Reset returns the highlighting to the first command', async () => {
    await planV2Successfully(app);
    app.scrub(5);
    assert.deepEqual(app.commandStates(), ['done', 'active', 'pending']);

    app.$('resetAnimation').onclick();
    assert.equal(app.$('step').textContent, `1 / ${V2_STEPS}`);
    assert.deepEqual(app.commandStates(), ['active', 'pending', 'pending']);
  });
});

describe('final SNAP completion', () => {
  test('the final preview step completes the final SNAP and counts the obstacle visited', async () => {
    await planV2Successfully(app);
    app.scrub(5);
    assert.equal(app.$('visited').textContent, '0 / 1');

    app.scrub(6);
    assert.equal(app.$('step').textContent, `7 / ${V2_STEPS}`);
    assert.deepEqual(app.commandStates(), ['done', 'done', 'done']);
    assert.equal(app.$('visited').textContent, '1 / 1');
    assert.equal(app.$('visitedLabel').textContent, 'Visited (preview)');
    assert.ok(app.$('route').children[0].className.includes('done'), 'obstacle order shows it done');
  });

  test('an intermediate SNAP behaves as before: active on its own step, visited once passed', async () => {
    await planV2Successfully(app, { plan: v2PlanTwoObstacles, cells: [[6, 10], [12, 10]], steps: TWO_OBSTACLE_STEPS });
    // steps: MOV = 0-2, SNAP O1 = 3, MOV = 4-6, SNAP O2 = 7
    app.scrub(2);
    assert.deepEqual(app.commandStates(), ['active', 'pending', 'pending', 'pending']);
    assert.equal(app.$('visited').textContent, '0 / 2');

    app.scrub(3);
    assert.deepEqual(app.commandStates(), ['done', 'active', 'pending', 'pending']);
    assert.equal(app.$('visited').textContent, '0 / 2', 'a SNAP is not visited while it is the active step');

    app.scrub(4);
    assert.deepEqual(app.commandStates(), ['done', 'done', 'active', 'pending']);
    assert.equal(app.$('visited').textContent, '1 / 2');

    app.scrub(7);
    assert.deepEqual(app.commandStates(), ['done', 'done', 'done', 'done']);
    assert.equal(app.$('visited').textContent, '2 / 2');
  });
});

describe('legacy v1 is unaffected', () => {
  test('v1 scrubbing keeps its own visited count/label and never marks command rows active or done', async () => {
    app.clickCell(6, 10);
    app.queue('/plan', async () => jsonResponse(200, v1Plan()));
    app.run();
    await app.settle();
    assert.equal(app.$('step').textContent, `1 / ${V1_STEPS}`);
    assert.equal(app.$('visitedLabel').textContent, 'Visited');
    assert.equal(app.$('visited').textContent, '0 / 1');

    for (const position of [1, 2, 3]) {
      app.scrub(position);
      assert.equal(app.$('step').textContent, `${position + 1} / ${V1_STEPS}`);
      assert.equal(app.commandStates().every((s) => s === 'pending'), true);
    }
    assert.equal(app.$('visited').textContent, '1 / 1');
    assert.deepEqual(app.commandRows(), [['FW', '40.00 cm'], ['SNAP', 'obstacle 1']]);
  });
});
