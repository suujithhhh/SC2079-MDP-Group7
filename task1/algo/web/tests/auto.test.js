// Tests for the real display-only page: algo/web/index.html + auto.js, driven
// through a minimal fake DOM, a fake clock and a scripted planner server
// (support.js). The page never edits the map and never sends a planning
// request: it only shows what the RPi asked the planner for, read from
// /state/v2. Also checks that /manual.html keeps the manual interface.
//
// Run with:  node --test algo/web/tests/auto.test.js

import { test, describe, afterEach } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { resolve } from 'node:path';

import { FakeClock, makeServer, noRoutePlan, rpiRequest, stmSetupFor, validPlan } from './support.js';

const WEB = resolve(import.meta.dirname, '..');
const AUTO_URL = pathToFileURL(resolve(WEB, 'auto.js')).href;
const read = (name) => readFileSync(resolve(WEB, name), 'utf8');
const GLOBALS = ['document', 'fetch', 'setTimeout', 'clearTimeout', 'setInterval', 'clearInterval', 'location'];

const ROUTE_STROKE = '#1f5eff';
const OBSTACLE_FILL = '#25334d';

let loads = 0;
let active = null;

async function loadPage({ search = '', initialState = null } = {}) {
  const saved = Object.fromEntries(GLOBALS.map((k) => [k, globalThis[k]]));
  const savedNow = Date.now;
  const clock = new FakeClock();
  const server = makeServer();
  if (initialState) server.state = initialState;
  const elements = new Map();
  const created = [];
  const htmlWrites = [];
  const fetchLog = [];
  const docHandlers = {};
  let imageList = { type: 'IMAGE_LIST', mission_id: null, revision: 0, images: [] };
  let imagesDown = false;
  let versionsReply = { type: 'PLAN_VERSIONS', last_request: null };
  let versionsDown = false;

  class FakeElement {
    constructor(tag) {
      this.tag = tag;
      this.tagName = tag.toUpperCase();
      this.children = [];
      this.attrs = {};
      this.style = {};
      this.dataset = {};
      this.handlers = {};
      this.textContent = '';
      this.className = '';
      this.hidden = false;
      this.disabled = false;
      this.value = '';
      this.onclick = null;
      const self = this;
      this.classList = {
        add(c) { if (!self.className.split(' ').includes(c)) self.className = `${self.className} ${c}`.trim(); },
        remove(c) { self.className = self.className.split(' ').filter((x) => x && x !== c).join(' '); },
        toggle(c, on) { if (on) this.add(c); else this.remove(c); },
      };
    }
    set innerHTML(html) { htmlWrites.push(html); this.children = []; }
    get innerHTML() { return ''; }
    setAttribute(k, v) { this.attrs[k] = v; }
    appendChild(child) { this.children.push(child); return child; }
    append(...kids) { this.children.push(...kids); }
    replaceChildren(...kids) { this.children = [...kids]; }
    addEventListener(type, fn) { this.handlers[type] = fn; }
    getBoundingClientRect() { return { left: 0, top: 0, width: 1000, height: 1000 }; }
  }

  const make = (tag) => {
    const el = new FakeElement(tag);
    created.push(el);
    return el;
  };
  const faceButtons = ['N', 'E', 'S', 'W'].map((face) => Object.assign(make('button'), { dataset: { face } }));
  const initial = { startX: '2', startY: '1', startHeading: 'N' };

  globalThis.document = {
    getElementById(id) {
      if (!elements.has(id)) {
        const el = make(id === 'startHeading' ? 'select' : id.startsWith('start') ? 'input' : 'element');
        if (id in initial) el.value = initial[id];
        elements.set(id, el);
      }
      return elements.get(id);
    },
    createElement: make,
    createElementNS: (_ns, tag) => make(tag),
    querySelectorAll: (selector) => (selector === '[data-face]' ? faceButtons : []),
    addEventListener: (type, fn) => { docHandlers[type] = fn; },
  };
  globalThis.location = { search };
  globalThis.setTimeout = clock.setTimeout;
  globalThis.clearTimeout = clock.clearTimeout;
  globalThis.setInterval = clock.setInterval;
  globalThis.clearInterval = clock.clearInterval;
  Date.now = clock.now;
  globalThis.fetch = async (url, opts = {}) => {
    fetchLog.push({ url, method: opts.method || 'GET' });
    if (url === '/state/v2') {
      const snapshot = await server.transport.getState();
      return { status: 200, ok: true, json: async () => snapshot };
    }
    if (url === '/images') {
      if (imagesDown) throw new TypeError('network down');
      return { status: 200, ok: true, json: async () => structuredClone(imageList) };
    }
    if (url === '/state/versions') {
      if (versionsDown) throw new TypeError('network down');
      return { status: 200, ok: true, json: async () => structuredClone(versionsReply) };
    }
    if (url === '/plan/v2') {
      const reply = await server.transport.postPlan(JSON.parse(opts.body), opts.signal);
      return {
        status: reply.httpStatus,
        ok: reply.httpStatus < 400,
        json: async () => {
          if (!reply.jsonOk) throw new SyntaxError('Unexpected token');
          return reply.body;
        },
      };
    }
    return { status: 404, ok: false, json: async () => ({}) };
  };

  await import(`${AUTO_URL}?instance=${loads++}`);
  await clock.advance(0);

  const $ = (id) => globalThis.document.getElementById(id);
  const textOf = (el) => [el.textContent, ...el.children.map(textOf)].join(' ').replace(/\s+/g, ' ').trim();
  const page = {
    $,
    clock,
    server,
    created,
    htmlWrites,
    fetchLog,
    docHandlers,
    textOf,
    posts: () => fetchLog.filter((f) => f.url === '/plan/v2'),
    setImages(list) { imageList = list; },
    setImagesDown(down) { imagesDown = down; },
    /** The newest request the server saw, as `GET /state/versions` reports it (null: none). */
    setLastRequest(last) { versionsReply = { type: 'PLAN_VERSIONS', last_request: last }; },
    setVersionsReply(reply) { versionsReply = reply; },
    setVersionsDown(down) { versionsDown = down; },
    tiles: () => $('gallery').children.map((tile) => ({
      src: tile.children[0].src,
      alt: tile.children[0].alt,
      caption: tile.children[1].textContent,
    })),
    routePaths: () => $('map').children.filter((el) => el.tag === 'path' && el.attrs.stroke === ROUTE_STROKE),
    obstacleRects: () => $('map').children.filter((el) => el.tag === 'rect' && el.attrs.fill === OBSTACLE_FILL),
    commandRows: () => $('commands').children.filter((row) => row.className.split(' ').includes('command')),
    /** The RPi asked the planner for `request`; let the page's next poll see it. */
    async rpiPlans(missionId, status, result = null, request = rpiRequest(missionId)) {
      server.external(missionId, request, status, result);
      await clock.advance(3000);
    },
    setupRows: () => $('commands').children.filter((row) => row.className === 'setup-row').map(textOf),
    commandBoxClasses: () => $('commands').children.map((row) => row.className),
    restore() {
      for (const key of GLOBALS) {
        if (saved[key] === undefined) delete globalThis[key];
        else globalThis[key] = saved[key];
      }
      Date.now = savedNow;
    },
  };
  active = page;
  return page;
}

afterEach(() => {
  active?.restore();
  active = null;
});

// ---------------------------------------------------------------------------
// Page structure
// ---------------------------------------------------------------------------

const FORBIDDEN_IDS = [
  'engineButtons', 'runB2', 'runB3', 'play', 'previous', 'next', 'timeline', 'resetAnimation',
  'speed', 'allowAlternativeViewing', 'copyActions', 'downloadActions', 'missionIdReadout', 'setStart',
];

/** index.html without the two `hidden` wrappers: what a person can actually see. */
function visibleHtml() {
  return read('index.html')
    .replace(/<div id="hiddenPanels" hidden>[\s\S]*<\/div>\s*<\/aside>/, '</aside>')
    .replace(/<div hidden>\s*<div id="observeBanner"[\s\S]*?<div id="externalNote"[^>]*><\/div>\s*<\/div>/, '');
}

describe('page structure', () => {
  test('the visible page has no control of any kind: only the map and the Planning card', () => {
    const html = visibleHtml();
    for (const tag of ['<button', '<input', '<select', '<textarea', '<a ', '<form']) {
      assert.ok(!html.includes(tag), `the visible page must not contain ${tag}`);
    }
    assert.ok(html.includes('id="map"') && html.includes('id="planStatus"') && html.includes('id="status"'));
    for (const id of ['startX', 'startY', 'obstacles', 'commands', 'totalCost', 'observeBanner', 'externalNote']) {
      assert.ok(!html.includes(`id="${id}"`), `#${id} must not be visible`);
    }
    assert.ok(!html.includes('manual.html'));
  });

  test('the editing cards and the old controls exist only inside hidden wrappers', () => {
    const html = read('index.html');
    assert.match(html, /<div id="hiddenPanels" hidden>/);
    for (const id of FORBIDDEN_IDS) assert.ok(!html.includes(`id="${id}"`), `index.html must not contain #${id}`);
    assert.ok(!html.includes('data-engine'));
    assert.ok(!/<input[^>]*type="range"/.test(html) && !/<input[^>]*type="checkbox"/.test(html));
    assert.match(html, /<script type="module" src="\/auto\.js"><\/script>/);
    for (const name of ['index.html', 'auto.js', 'autoplan.js']) assert.ok(!read(name).includes('manual.html'));
  });

  test('/manual.html keeps its manual v1/v2 interface', () => {
    const html = read('manual.html');
    for (const id of FORBIDDEN_IDS.filter((id) => id !== 'missionIdReadout')) {
      assert.ok(html.includes(`id="${id}"`), `manual.html keeps #${id}`);
    }
    assert.ok(html.includes('id="missionIdReadout"'));
    assert.ok(html.includes('data-engine="v1"') && html.includes('data-engine="v2"'));
    assert.match(html, /<script type="module" src="\/app\.js"><\/script>/);
  });

  test('auto.js registers no input handler of any kind', () => {
    const source = read('auto.js');
    assert.ok(!/addEventListener|\.onclick|\.oninput|\.onchange|\.onkeydown|\.onpointer/.test(source));
    for (const method of ['clickCell', 'chooseFace', 'selectObstacle', 'deleteObstacle', 'clearMap', 'setStartInput', 'noteActivity']) {
      assert.ok(!source.includes(`controller.${method}(`), `auto.js must not call controller.${method}()`);
    }
  });
});

// ---------------------------------------------------------------------------
// Behaviour through the real DOM wiring
// ---------------------------------------------------------------------------

describe('display-only page', () => {
  test('nothing on the page is wired up, and it never sends a planning request', async () => {
    const page = await loadPage();
    assert.deepEqual(Object.keys(page.docHandlers), []);
    for (const el of page.created) {
      assert.deepEqual(Object.keys(el.handlers), [], `${el.tag} has no event handler`);
      assert.equal(el.onclick, null);
    }
    // The map cannot be clicked: there is no handler to call.
    assert.equal($click(page), false);
    await page.clock.advance(300_000);
    assert.equal(page.posts().length, 0);
    assert.ok(page.fetchLog.every((f) => f.method === 'GET' && ['/state/v2', '/images', '/state/versions'].includes(f.url)));
  });

  test('a ?observe=1 or any other address does not change that', async () => {
    const page = await loadPage({ search: '?observe=1&edit=1' });
    assert.deepEqual(Object.keys(page.$('map').handlers), []);
    assert.equal(page.posts().length, 0);
  });

  test('idle: the Planning card says it is waiting for the RPi', async () => {
    const page = await loadPage();
    assert.equal(page.$('status').textContent, 'Waiting');
    assert.equal(page.$('planStatus').textContent, 'Waiting for the RPi to request a plan.');
    assert.equal(page.obstacleRects().length, 0);
    assert.equal(page.routePaths().length, 0);
  });

  test('while the RPi\'s plan runs: a status, a timer, and the map shows its obstacles', async () => {
    const page = await loadPage();
    await page.rpiPlans('rpi-1', 'planning');
    assert.equal(page.$('status').textContent, 'Planning');
    assert.match(page.$('planStatus').textContent, /^External mission rpi-1 planning… ≈\d+ s \(approximate, observed by this page\)$/);
    assert.equal(page.obstacleRects().length, 1, 'the planned map is drawn while it plans');
    assert.equal(page.routePaths().length, 0, 'no route before there is one');
    const seconds = () => Number(/≈(\d+) s/.exec(page.$('planStatus').textContent)[1]);
    const before = seconds();
    await page.clock.advance(10_000);
    assert.ok(seconds() >= before + 9, 'the timer keeps counting');
    assert.equal(page.posts().length, 0);
  });

  test('when the RPi\'s plan is ready: Route ready, the static route and obstacles are on the map', async () => {
    const page = await loadPage();
    await page.rpiPlans('rpi-2', 'planning');
    await page.rpiPlans('rpi-2', 'complete', validPlan('rpi-2', [1]));
    assert.equal(page.$('status').textContent, 'Route ready');
    assert.equal(page.$('planStatus').textContent, 'Best safe complete route found within the planning window — optimality not proven');
    assert.equal(page.routePaths().length, 2, 'the full static route is drawn at once');
    assert.equal(page.obstacleRects().length, 1);
    assert.equal(page.posts().length, 0);
  });

  test('a proven-optimal result is labelled so, and only because the backend says it is proven', async () => {
    const page = await loadPage();
    await page.rpiPlans('rpi-3', 'complete', validPlan('rpi-3', [1], { optimality_proven: true }));
    assert.equal(page.$('planStatus').textContent, 'Optimal route (proven by search)');
  });

  test('a failed plan shows a red failure on the Planning card and no route', async () => {
    const page = await loadPage();
    await page.rpiPlans('rpi-4', 'complete', noRoutePlan('rpi-4', [1]));
    assert.equal(page.$('status').textContent, 'No route');
    assert.equal(page.$('status').className.trim(), 'badge error');
    assert.equal(page.$('planStatus').textContent, 'No executable route');
    assert.equal(page.$('planStatus').className.includes('error-text'), true);
    assert.equal(page.$('planDetail').textContent, 'No complete route was found within the planning window; non-existence was not proven.');
    assert.equal(page.routePaths().length, 0);
    assert.equal(page.commandRows().length, 0);
  });

  test('malformed plan data fails closed: no route is drawn', async () => {
    const page = await loadPage();
    const bad = validPlan('rpi-5', [1]);
    bad.actions[1] = { ...bad.actions[1], pose: { ...bad.actions[1].pose, y_cm: 41 } };
    await page.rpiPlans('rpi-5', 'complete', bad);
    assert.equal(page.$('status').textContent, 'Failed');
    assert.equal(page.routePaths().length, 0);
    assert.equal(page.commandRows().length, 0);
  });

  test('a reload shows the latest planner state at once, without planning anything', async () => {
    const request = rpiRequest('rpi-6');
    const page = await loadPage({
      initialState: { mission_id: 'rpi-6', request, status: 'complete', result: validPlan('rpi-6', [1]), error: null },
    });
    assert.equal(page.$('status').textContent, 'Route ready');
    assert.equal(page.routePaths().length, 2);
    await page.clock.advance(120_000);
    assert.equal(page.posts().length, 0);
  });

  test('an RPi map in centimetres, off the 10 cm grid, is drawn as it is', async () => {
    const page = await loadPage();
    const request = rpiRequest('rpi-7', { start: { x: 23.5, y: 10, heading_deg: 45 } });
    await page.rpiPlans('rpi-7', 'complete', validPlan('rpi-7', [1]), request);
    assert.equal(page.$('status').textContent, 'Route ready');
    assert.equal(page.obstacleRects().length, 1);
    assert.equal(page.posts().length, 0);
  });

  test('a new mission from the RPi replaces the previous one on screen', async () => {
    const page = await loadPage();
    await page.rpiPlans('rpi-8', 'complete', validPlan('rpi-8', [1]));
    assert.equal(page.$('status').textContent, 'Route ready');
    await page.rpiPlans('rpi-9', 'planning', null, rpiRequest('rpi-9', { obstacles: [{ id: 1, x: 50, y: 50, face: 'N' }, { id: 2, x: 120, y: 80, face: 'W' }] }));
    assert.equal(page.$('status').textContent, 'Planning');
    assert.equal(page.routePaths().length, 0, 'the old route is gone');
    assert.equal(page.obstacleRects().length, 2, 'the new map is shown');
  });

  test('backend text is rendered only with textContent, never as HTML', async () => {
    const evil = '<img src=x onerror="alert(1)">';
    const page = await loadPage();
    await page.rpiPlans('rpi-10', 'complete', noRoutePlan('rpi-10', [1], { message: evil }));
    assert.equal(page.$('failureBackend').textContent, `Planner: ${evil}`);
    assert.deepEqual(page.htmlWrites, []);
    assert.equal(page.created.filter((el) => el.tag === 'img').length, 0);
  });
});

describe('STM setup rows', () => {
  test('a STANDARD plan shows the STM setup first, before the first MOV', async () => {
    const page = await loadPage();
    await page.rpiPlans('rpi-11', 'complete', validPlan('rpi-11', [1]));
    assert.deepEqual(page.setupRows(), [
      'SETUP: set PROFILE,STANDARD \u2192 PROFILE,OK',
      'SETUP: verify PROFILE,GET \u2192 PROFILE,STANDARD',
    ]);
    // Header, the two setup rows, then the actions: the setup precedes every MOV.
    assert.deepEqual(page.commandBoxClasses(), ['setup-header', 'setup-row', 'setup-row', 'command', 'command snap']);
    assert.equal(page.textOf(page.$('commands').children[0]), 'STM setup \u2014 the RPi sends this before the first MOV');
    // Setup rows are not actions: the count and the action rows are unchanged.
    assert.equal(page.$('actionCount').textContent, '2');
    assert.deepEqual(page.commandRows().map(page.textOf), ['#0 MOV MOV,F,20', '#1 SNAP obstacle 1']);
    assert.equal(page.posts().length, 0);
  });

  test('a PRECISION plan from the RPi shows its own PRECISION setup', async () => {
    const page = await loadPage();
    await page.rpiPlans('rpi-12', 'complete', validPlan('rpi-12', [1], {
      motion_profile: 'PRECISION', stm_setup: stmSetupFor('PRECISION'),
    }));
    assert.deepEqual(page.setupRows(), [
      'SETUP: set PROFILE,PRECISION \u2192 PROFILE,OK',
      'SETUP: verify PROFILE,GET \u2192 PROFILE,PRECISION',
    ]);
  });

  test('a setup that does not match its profile fails closed: no setup rows, no commands, no route', async () => {
    const page = await loadPage();
    await page.rpiPlans('rpi-13', 'complete', validPlan('rpi-13', [1], {
      motion_profile: 'STANDARD', stm_setup: stmSetupFor('PRECISION'),
    }));
    assert.equal(page.$('status').textContent, 'Failed');
    assert.deepEqual(page.setupRows(), []);
    assert.equal(page.commandRows().length, 0);
    assert.equal(page.routePaths().length, 0);
  });

  test('a no-route result shows no setup rows', async () => {
    const page = await loadPage();
    await page.rpiPlans('rpi-14', 'complete', noRoutePlan('rpi-14', [1]));
    assert.deepEqual(page.setupRows(), []);
    assert.equal(page.commandRows().length, 0);
  });
});


describe('camera gallery', () => {
  const photo = (obstacleId, seq, at) => ({ image_id: `${obstacleId}-${seq}`, obstacle_id: obstacleId, bytes: 5000, received_at_s: at });
  const list = (missionId, images, revision = images.length) => ({ type: 'IMAGE_LIST', mission_id: missionId, revision, images });
  /** The RPi's plan is on screen, then the page's next photo poll runs. */
  async function planOnScreen(page, missionId = 'rpi-1') {
    await page.rpiPlans(missionId, 'complete', validPlan(missionId, [1]));
  }

  test('before any photo: the Camera card says so, and shows no tile', async () => {
    const page = await loadPage();
    await planOnScreen(page);
    assert.equal(page.$('galleryEmpty').hidden, false);
    assert.equal(page.$('galleryCount').textContent, '');
    assert.deepEqual(page.tiles(), []);
  });

  test('a photo appears after the next poll, labelled with its obstacle, and is not clickable', async () => {
    const page = await loadPage();
    await planOnScreen(page);
    page.setImages(list('rpi-1', [photo(3, 1, 100)]));
    await page.clock.advance(1100);
    assert.deepEqual(page.tiles(), [{ src: '/images/3-1', alt: 'Obstacle 3 photo', caption: 'Obstacle 3' }]);
    assert.equal(page.$('galleryEmpty').hidden, true);
    assert.equal(page.$('galleryCount').textContent, '1 photo');
    for (const el of page.created.filter((e) => e.tag === 'img' || e.tag === 'figure')) {
      assert.deepEqual(Object.keys(el.handlers), [], 'a photo has no event handler');
      assert.equal(el.onclick, null);
    }
  });

  test('photos are tiled in the order they were taken, one more after each snap', async () => {
    const page = await loadPage();
    await planOnScreen(page);
    page.setImages(list('rpi-1', [photo(5, 1, 10)]));
    await page.clock.advance(1100);
    page.setImages(list('rpi-1', [photo(5, 1, 10), photo(2, 2, 20)]));
    await page.clock.advance(1100);
    page.setImages(list('rpi-1', [photo(5, 1, 10), photo(2, 2, 20), photo(7, 3, 30)]));
    await page.clock.advance(1100);
    assert.deepEqual(page.tiles().map((t) => t.caption), ['Obstacle 5', 'Obstacle 2', 'Obstacle 7']);
    assert.equal(page.$('galleryCount').textContent, '3 photos');
  });

  test("a retake replaces that obstacle's tile; an unchanged list does not rebuild the tiles", async () => {
    const page = await loadPage();
    await planOnScreen(page);
    page.setImages(list('rpi-1', [photo(4, 1, 10)]));
    await page.clock.advance(1100);
    const firstTile = page.$('gallery').children[0];
    await page.clock.advance(5000); // several polls with the same list
    assert.equal(page.$('gallery').children[0], firstTile, 'the same tile element: no flicker');
    page.setImages(list('rpi-1', [photo(4, 2, 40)]));
    await page.clock.advance(1100);
    assert.deepEqual(page.tiles().map((t) => t.src), ['/images/4-2']);
    assert.equal(page.tiles().length, 1);
  });

  test("photos of another mission are never shown, and they vanish when a new plan starts", async () => {
    const page = await loadPage();
    page.setImages(list('rpi-old', [photo(1, 1, 10)]));
    await planOnScreen(page, 'rpi-new');
    await page.clock.advance(1100);
    assert.deepEqual(page.tiles(), [], "the old mission's photo is not shown for the new plan");
    page.setImages(list('rpi-new', [photo(2, 2, 20)]));
    await page.clock.advance(1100);
    assert.deepEqual(page.tiles().map((t) => t.caption), ['Obstacle 2']);
    // The RPi starts another plan: the previous run's photos disappear at once.
    await page.rpiPlans('rpi-next', 'planning', null, rpiRequest('rpi-next'));
    assert.deepEqual(page.tiles(), []);
    assert.equal(page.$('galleryEmpty').hidden, false);
  });

  test('with no plan on screen yet, nothing is shown even if photos exist', async () => {
    const page = await loadPage();
    page.setImages(list('rpi-1', [photo(1, 1, 10)]));
    await page.clock.advance(3000);
    assert.deepEqual(page.tiles(), []);
  });

  test('a malformed reply is ignored whole: no tile, no error', async () => {
    const page = await loadPage();
    await planOnScreen(page);
    page.setImages(list('rpi-1', [{ ...photo(3, 1, 10), image_id: '../../etc/passwd' }]));
    await page.clock.advance(1100);
    assert.deepEqual(page.tiles(), []);
    page.setImages({ type: 'IMAGE_ERROR', error: 'x' });
    await page.clock.advance(1100);
    assert.deepEqual(page.tiles(), []);
  });

  test('a network failure keeps what is on screen, and the next good poll updates it', async () => {
    const page = await loadPage();
    await planOnScreen(page);
    page.setImages(list('rpi-1', [photo(3, 1, 10)]));
    await page.clock.advance(1100);
    page.setImagesDown(true);
    await page.clock.advance(5000);
    assert.deepEqual(page.tiles().map((t) => t.caption), ['Obstacle 3']);
    page.setImagesDown(false);
    page.setImages(list('rpi-1', [photo(3, 1, 10), photo(6, 2, 20)]));
    await page.clock.advance(1100);
    assert.deepEqual(page.tiles().map((t) => t.caption), ['Obstacle 3', 'Obstacle 6']);
  });

  test('the page never uploads anything and the gallery adds no POST', async () => {
    const page = await loadPage();
    await planOnScreen(page);
    page.setImages(list('rpi-1', [photo(3, 1, 10)]));
    await page.clock.advance(10_000);
    assert.equal(page.fetchLog.filter((f) => f.method !== 'GET').length, 0);
  });
});

/** A map click: only possible if the page registered a handler for it. */
function $click(page) {
  return typeof page.$('map').handlers.click === 'function';
}


// ---------------------------------------------------------------------------
// Planner version indicator: which planner the newest request used (V1 or V2),
// and which one this page draws (always V2)
// ---------------------------------------------------------------------------

describe('planner version indicator', () => {
  const lit = (page, id) => page.$(id).className.split(' ').some((c) => c.startsWith('on-'));
  const last = (over = {}) => ({ protocol_version: 2, source: 'rpi', mission_id: 'task1-run-001', age_s: 0.2, ...over });

  test('the card is on the visible page, display only', () => {
    const html = visibleHtml();
    for (const id of ['reqV1', 'reqV2', 'shownV1', 'shownV2', 'reqText', 'shownText', 'versionWarning']) {
      assert.ok(html.includes(`id="${id}"`), `#${id} is visible`);
    }
    assert.match(html, /<h2>Planner version<\/h2>/);
  });

  test('no request yet: nothing lit in the request row, V2 lit in the page row', async () => {
    const page = await loadPage();
    assert.equal(lit(page, 'reqV1'), false);
    assert.equal(lit(page, 'reqV2'), false);
    assert.equal(page.$('reqText').textContent, 'No plan request yet.');
    assert.equal(page.$('shownV1').className, 'vcircle');
    assert.equal(page.$('shownV2').className, 'vcircle on-v2');
    assert.match(page.$('shownText').textContent, /V2 only/);
    assert.equal(page.$('versionWarning').hidden, true);
  });

  test('the RPi asked for V2: the request V2 circle is lit, V1 is not, no warning', async () => {
    const page = await loadPage();
    page.setLastRequest(last());
    await page.clock.advance(1000);
    assert.equal(page.$('reqV2').className, 'vcircle on-v2');
    assert.equal(page.$('reqV1').className, 'vcircle');
    assert.equal(page.$('reqV2').attrs['aria-label'], 'V2 active');
    assert.equal(page.$('reqV1').attrs['aria-label'], 'V1 not active');
    assert.match(page.$('reqText').textContent, /^Robot \(RPi\) · V2 \(\/plan\/v2\) · .* · mission task1-run-001$/);
    assert.equal(page.$('versionWarning').hidden, true);
  });

  test('the RPi asked for V1: the request V1 circle is lit and the page says it cannot draw it', async () => {
    const page = await loadPage();
    page.setLastRequest(last({ protocol_version: 1, mission_id: null }));
    await page.clock.advance(1000);
    assert.equal(page.$('reqV1').className, 'vcircle on-v1');
    assert.equal(page.$('reqV2').className, 'vcircle');
    assert.match(page.$('reqText').textContent, /^Robot \(RPi\) · V1 \(\/plan\)/);
    assert.equal(page.$('versionWarning').hidden, false);
    assert.match(page.$('versionWarning').textContent, /last request used V1/);
    assert.equal(page.$('shownV2').className, 'vcircle on-v2', 'the page still draws V2 only');
  });

  test('the request row names the mode; a mode other than B3 raises a warning', async () => {
    const page = await loadPage();
    page.setLastRequest(last({ mode: 'B3' }));
    await page.clock.advance(1000);
    assert.match(page.$('reqText').textContent, /^Robot \(RPi\) · V2 \(\/plan\/v2\) · B3 · .* · mission task1-run-001$/);
    assert.equal(page.$('versionWarning').hidden, true);
    page.setLastRequest(last({ mode: 'B2' }));
    await page.clock.advance(1000);
    assert.match(page.$('reqText').textContent, /· B2 ·/);
    assert.equal(page.$('versionWarning').hidden, false);
    assert.match(page.$('versionWarning').textContent, /mode B2, not B3/);
  });

  test('a request from a browser page is labelled as such, not as the robot', async () => {
    const page = await loadPage();
    page.setLastRequest(last({ source: 'browser' }));
    await page.clock.advance(1000);
    assert.match(page.$('reqText').textContent, /^A browser page · V2/);
  });

  test('a V2 mission other than the one on screen raises a warning', async () => {
    const page = await loadPage();
    await page.rpiPlans('mission-on-screen', 'planning');
    page.setLastRequest(last({ mission_id: 'mission-newer' }));
    await page.clock.advance(1000);
    assert.equal(page.$('versionWarning').hidden, false);
    assert.match(page.$('versionWarning').textContent, /mission-on-screen.*mission-newer/);
    page.setLastRequest(last({ mission_id: 'mission-on-screen' }));
    await page.clock.advance(1000);
    assert.equal(page.$('versionWarning').hidden, true);
  });

  test('a malformed reply is ignored: nothing lit, nothing claimed', async () => {
    const page = await loadPage();
    page.setLastRequest(last());
    await page.clock.advance(1000);
    assert.equal(lit(page, 'reqV2'), true);
    page.setVersionsReply({ type: 'PLAN_VERSIONS', last_request: last({ protocol_version: 3 }) });
    await page.clock.advance(1000);
    assert.equal(lit(page, 'reqV1') || lit(page, 'reqV2'), false);
    assert.equal(page.$('reqText').textContent, 'Waiting for the planner.');
  });

  test('when the poll fails the last reply stays and its age keeps growing', async () => {
    const page = await loadPage();
    page.setLastRequest(last({ age_s: 0 }));
    await page.clock.advance(1000);
    page.setVersionsDown(true);
    await page.clock.advance(30_000);
    assert.equal(page.$('reqV2').className, 'vcircle on-v2');
    assert.match(page.$('reqText').textContent, /· (3\d) s ago ·/);
  });

  test('the page only reads: every version request is a GET', async () => {
    const page = await loadPage();
    await page.clock.advance(3000);
    const calls = page.fetchLog.filter((f) => f.url === '/state/versions');
    assert.ok(calls.length >= 3);
    assert.ok(calls.every((f) => f.method === 'GET'));
  });
});
