// Tests for algo/web/autoplan.js: the automatic page's pure planning rules,
// driven by a fake clock and a scripted single-flight server (support.js).
//
// Run with:  node --test algo/web/tests/autoplan.test.js

import { test, describe } from 'node:test';
import assert from 'node:assert/strict';

import {
  AUTO_MOTION_PROFILE,
  AUTO_PLAN_IDLE_MS,
  BROWSER_TIMEOUT_MS,
  AutoPlanController,
  addObstacle,
  buildAutoRequest,
  describeRequestMap,
  emptyMap,
  evaluatePlanResponse,
  lowestFreeObstacleId,
  mapFingerprint,
  matchesAutoDefaults,
  parseStartInput,
  responseMatchesLatest,
} from '../autoplan.js';
import { FakeClock, makeServer, noRoutePlan, rpiRequest, stmSetupFor, validPlan } from './support.js';

function setup({ observeOnly = false, initialState = null } = {}) {
  const clock = new FakeClock();
  const server = makeServer();
  if (initialState) server.state = initialState;
  let n = 0;
  const ctl = new AutoPlanController({
    observeOnly,
    transport: server.transport,
    clock,
    generateId: (prefix) => `${prefix}-${++n}`,
  });
  ctl.start();
  return { clock, server, ctl };
}

const ids = (payload) => payload.obstacles.map((o) => o.id);
const FACE_CYCLE = ['E', 'S', 'W', 'N'];

/** Places obstacles and lets the countdown expire, leaving one POST pending. */
async function planOnce(env, cells = [[5, 10]]) {
  await env.clock.advance(0);
  for (const [x, y] of cells) env.ctl.clickCell(x, y);
  await env.clock.advance(AUTO_PLAN_IDLE_MS);
  const entry = env.server.pending();
  assert.ok(entry, 'expected one pending POST');
  return entry;
}

// ---------------------------------------------------------------------------
// Map model
// ---------------------------------------------------------------------------

describe('map editing', () => {
  test('the lowest free obstacle ID is reused, so IDs stay within 1-8', () => {
    let map = emptyMap();
    for (let i = 0; i < 3; i++) map = addObstacle(map, i, 10, 'N').map;
    assert.deepEqual(map.obstacles.map((o) => o.id), [1, 2, 3]);
    map = { ...map, obstacles: map.obstacles.filter((o) => o.id !== 2) };
    const added = addObstacle(map, 9, 9, 'E');
    assert.equal(added.id, 2);
    let full = emptyMap();
    for (let i = 0; i < 8; i++) full = addObstacle(full, i, 12, 'S').map;
    assert.equal(lowestFreeObstacleId(full.obstacles), null);
    assert.equal(addObstacle(full, 15, 15, 'S'), null, 'a ninth obstacle is refused');
    assert.equal(addObstacle(full, 0, 12, 'S'), null, 'an occupied cell is refused');
  });

  test('start input is validated as typed; problems never mutate the accepted map', () => {
    assert.match(parseStartInput('', '1', 'N').problem, /Enter the start X/);
    assert.match(parseStartInput('2.5', '1', 'N').problem, /whole number/);
    assert.match(parseStartInput('2', '20', 'N').problem, /whole number/);
    assert.match(parseStartInput('-1', '1', 'N').problem, /whole number/);
    assert.match(parseStartInput('2', '1', 'Q').problem, /heading/);
    assert.deepEqual(parseStartInput(' 3 ', '4', 'W').start, { x: 3, y: 4, heading: 'W' });

    const { ctl } = setup();
    const revision = ctl.revision;
    assert.match(ctl.setStartInput('1', '', 'N'), /Enter the start Y/);
    assert.equal(ctl.revision, revision);
    assert.deepEqual(ctl.localMap.start, { x: 2, y: 1, heading: 'N' });
    assert.equal(ctl.view().startProblem, ctl.startProblem);
    assert.equal(ctl.setStartInput('3', '2', 'E'), null);
    assert.deepEqual(ctl.localMap.start, { x: 3, y: 2, heading: 'E' });
    assert.equal(ctl.revision, revision + 1);
    ctl.dispose();
  });

  test('a selected obstacle changes facing in place (same ID, no re-create)', () => {
    const { ctl } = setup();
    ctl.clickCell(4, 4);
    ctl.clickCell(8, 8);
    ctl.clickCell(4, 4); // select obstacle 1
    assert.equal(ctl.view().selectedId, 1);
    ctl.chooseFace('W');
    assert.deepEqual(ctl.localMap.obstacles.find((o) => o.id === 1), { id: 1, x: 4, y: 4, face: 'W' });
    assert.equal(ctl.localMap.obstacles.length, 2);
    ctl.deleteSelected();
    assert.deepEqual(ctl.localMap.obstacles.map((o) => o.id), [2]);
    ctl.dispose();
  });

  test('the fixed request: v2, B3, grid, no alternative viewing, STANDARD, web-auto mission', () => {
    let map = addObstacle(emptyMap(), 7, 9, 'E').map;
    map = addObstacle(map, 3, 3, 'N').map;
    const request = buildAutoRequest(map, 'web-auto-x');
    assert.deepEqual(request, {
      type: 'PLAN_REQUEST',
      protocol_version: 2,
      mission_id: 'web-auto-x',
      mode: 'B3',
      coordinate_units: 'grid',
      allow_alternative_viewing: false,
      start: { x: 2, y: 1, heading_deg: 90 },
      obstacles: [{ id: 1, x: 7, y: 9, face: 'E' }, { id: 2, x: 3, y: 3, face: 'N' }],
      motion_profile: 'STANDARD',
    });
    assert.ok(!mapFingerprint(map).includes('web-auto-x'), 'the fingerprint excludes the mission ID');
  });
});

// ---------------------------------------------------------------------------
// Countdown
// ---------------------------------------------------------------------------

describe('inactivity countdown', () => {
  test('never plans with zero obstacles', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(AUTO_PLAN_IDLE_MS * 4);
    ctl.clickCell(5, 5);
    ctl.deleteObstacle(1);
    await clock.advance(AUTO_PLAN_IDLE_MS * 4);
    assert.equal(server.posts.length, 0);
    assert.equal(ctl.view().status.text, 'Place 1–8 obstacles to plan a route.');
    ctl.dispose();
  });

  test('exactly one plan fires exactly at the inactivity deadline, with the fixed values', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.clickCell(5, 10);
    assert.equal(ctl.view().status.text, 'Planning starts in 5 s — keep editing to postpone');
    await clock.advance(1500);
    assert.equal(ctl.view().status.text, 'Planning starts in 4 s — keep editing to postpone');
    await clock.advance(AUTO_PLAN_IDLE_MS - 1500 - 1);
    assert.equal(server.posts.length, 0);
    await clock.advance(1);
    assert.equal(server.posts.length, 1);
    const { payload } = server.posts[0];
    assert.equal(payload.protocol_version, 2);
    assert.equal(payload.mode, 'B3');
    assert.equal(payload.coordinate_units, 'grid');
    assert.equal(payload.allow_alternative_viewing, false);
    assert.equal(payload.motion_profile, 'STANDARD');
    assert.match(payload.mission_id, /^web-auto-/);
    assert.equal(ctl.view().status.text, 'Planning… 0 s · target ≈63 s · hard stop 66 s · reply by 70 s');
    await clock.advance(12_000);
    assert.equal(ctl.view().status.text, 'Planning… 12 s · target ≈63 s · hard stop 66 s · reply by 70 s');
    assert.match(ctl.view().status.detail, /server-owned 70 s window after the server accepts it/);
    assert.equal(server.posts.length, 1);
    ctl.dispose();
  });

  test('every accepted edit resets the timer', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.clickCell(5, 10); // add obstacle 1
    await clock.advance(3000);
    ctl.selectObstacle(1);
    ctl.chooseFace('E'); // facing edit of the selected obstacle
    await clock.advance(3000);
    ctl.setStartInput('3', '1', 'N'); // start edit
    await clock.advance(3000);
    ctl.clickCell(9, 9); // add obstacle 2
    await clock.advance(3000);
    ctl.deleteObstacle(2); // delete (obstacle 1 remains)
    await clock.advance(AUTO_PLAN_IDLE_MS - 1);
    assert.equal(server.posts.length, 0, 'each edit restarted the countdown');
    await clock.advance(1);
    assert.equal(server.posts.length, 1);
    assert.deepEqual(server.posts[0].payload.start, { x: 3, y: 1, heading_deg: 90 });
    assert.deepEqual(server.posts[0].payload.obstacles, [{ id: 1, x: 5, y: 10, face: 'E' }]);
    ctl.dispose();
  });
});

// ---------------------------------------------------------------------------
// D1: an invalid or partial start input blocks planning
// ---------------------------------------------------------------------------

describe('start-input problems block planning', () => {
  test('invalid before the pending deadline: nothing is posted at or after the old deadline', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.clickCell(5, 10);
    await clock.advance(3000);
    const revision = ctl.revision;
    assert.match(ctl.setStartInput('', '1', 'N'), /Enter the start X/);
    assert.equal(ctl.revision, revision, 'the accepted map is not mutated');
    assert.deepEqual(ctl.localMap.start, { x: 2, y: 1, heading: 'N' });
    assert.equal(ctl.view().status.kind, 'invalid-start');
    assert.equal(ctl.view().status.text, 'Fix the start position to continue.');
    await clock.advance(2000); // the old deadline
    await clock.advance(1);
    assert.equal(server.posts.length, 0, 'not at the old deadline');
    await clock.advance(120_000);
    assert.equal(server.posts.length, 0, 'and not afterwards');
    // Partial and out-of-range values block alike.
    for (const [x, y] of [['2', ''], ['2', '2x'], ['20', '1'], ['-1', '1']]) {
      assert.ok(ctl.setStartInput(x, y, 'N'));
      await clock.advance(30_000);
    }
    assert.equal(server.posts.length, 0);
    ctl.dispose();
  });

  test('editing the map while the start is invalid still does not post', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.setStartInput('', '1', 'N');
    ctl.clickCell(5, 10);
    ctl.clickCell(6, 10);
    await clock.advance(120_000);
    assert.equal(server.posts.length, 0);
    assert.equal(ctl.view().status.kind, 'invalid-start');
    ctl.setStartInput('2', '1', 'N');
    assert.equal(ctl.view().status.kind, 'countdown', 'valid again: planning resumes with a fresh countdown');
    await clock.advance(AUTO_PLAN_IDLE_MS);
    assert.equal(server.posts.length, 1);
    assert.deepEqual(ids(server.posts[0].payload), [1, 2]);
    ctl.dispose();
  });

  test('invalid during an in-flight request: its result is not displayed', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    const revision = ctl.revision;
    ctl.setStartInput('3', '', 'N');
    assert.equal(ctl.revision, revision);
    assert.equal(ctl.view().shown, null);
    assert.match(ctl.view().status.detail, /will not be shown while the start is invalid/);
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(0);
    assert.equal(ctl.view().shown, null, 'not displayed while the input stays invalid');
    assert.equal(ctl.view().status.kind, 'invalid-start');
    await clock.advance(AUTO_PLAN_IDLE_MS * 3);
    assert.equal(server.posts.length, 1);
    ctl.dispose();
  });

  test('a restored valid, changed start gets exactly one fresh countdown and one request', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.clickCell(5, 10);
    await clock.advance(2000);
    ctl.setStartInput('3', '', 'N');
    await clock.advance(10_000);
    assert.equal(server.posts.length, 0);
    ctl.setStartInput('3', '4', 'E');
    assert.equal(ctl.view().status.text, 'Planning starts in 5 s — keep editing to postpone');
    await clock.advance(AUTO_PLAN_IDLE_MS - 1);
    assert.equal(server.posts.length, 0);
    await clock.advance(1);
    assert.equal(server.posts.length, 1);
    assert.deepEqual(server.posts[0].payload.start, { x: 3, y: 4, heading_deg: 0 });
    await clock.advance(60_000);
    assert.equal(server.posts.length, 1);
    ctl.dispose();
  });

  test('restoring the unchanged start restarts one fresh countdown (the old one is gone)', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.clickCell(5, 10);
    ctl.setStartInput('', '1', 'N');
    await clock.advance(8000);
    ctl.setStartInput('2', '1', 'N'); // identical to the accepted start
    await clock.advance(AUTO_PLAN_IDLE_MS - 1);
    assert.equal(server.posts.length, 0);
    await clock.advance(1);
    assert.equal(server.posts.length, 1);
    assert.deepEqual(server.posts[0].payload.start, { x: 2, y: 1, heading_deg: 90 });
    await clock.advance(60_000);
    assert.equal(server.posts.length, 1);
    ctl.dispose();
  });

  test('restoring the exact map in flight reattaches to it: no duplicate, matching result shown', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    ctl.setStartInput('', '1', 'N');
    await clock.advance(8000);
    ctl.setStartInput('2', '1', 'N');
    assert.equal(ctl.view().status.kind, 'planning');
    await clock.advance(AUTO_PLAN_IDLE_MS * 2);
    assert.equal(server.posts.length, 1, 'no duplicate request');
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(0);
    assert.equal(ctl.view().shown.kind, 'result');
    assert.equal(ctl.view().shown.missionId, first.payload.mission_id);
    await clock.advance(60_000);
    assert.equal(server.posts.length, 1);
    ctl.dispose();
  });

  test('a changed valid start makes the old in-flight response stale: never shown, one follow-up', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    ctl.setStartInput('', '1', 'N');
    ctl.setStartInput('4', '1', 'N');
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(0);
    assert.equal(ctl.view().shown, null);
    await clock.advance(AUTO_PLAN_IDLE_MS + 1000);
    assert.equal(server.posts.length, 2);
    assert.deepEqual(server.posts[1].payload.start, { x: 4, y: 1, heading_deg: 90 });
    ctl.dispose();
  });

  test('a response that arrived while invalid shows only once the exact map is restored', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    ctl.setStartInput('', '1', 'N');
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(0);
    assert.equal(ctl.view().shown, null);
    ctl.setStartInput('2', '1', 'N');
    assert.equal(ctl.view().shown.kind, 'result');
    await clock.advance(60_000);
    assert.equal(server.posts.length, 1);
    ctl.dispose();
  });

  test('a displayed route disappears the moment the start input becomes invalid', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(0);
    assert.equal(ctl.view().shown.kind, 'result');
    ctl.setStartInput('2', '', 'N'); // no clock advance: immediate
    assert.equal(ctl.view().shown, null);
    assert.equal(ctl.view().status.kind, 'invalid-start');
    ctl.setStartInput('2', '1', 'N');
    assert.equal(ctl.view().shown.kind, 'result', 'the accepted map is unchanged, so its route returns');
    await clock.advance(60_000);
    assert.equal(server.posts.length, 1, 'and nothing is replanned');
    ctl.dispose();
  });
});

// ---------------------------------------------------------------------------
// D2: sequential placement; D3: editing activity postpones a pending countdown
// ---------------------------------------------------------------------------

describe('sequential obstacle placement', () => {
  test('a new obstacle is not left selected: the next facing never alters the previous one', () => {
    const { ctl } = setup();
    ctl.chooseFace('N');
    ctl.clickCell(4, 4); // obstacle 1, North
    assert.equal(ctl.view().selectedId, null);
    ctl.chooseFace('E');
    ctl.clickCell(8, 8); // obstacle 2, East
    assert.equal(ctl.view().selectedId, null);
    assert.deepEqual(ctl.localMap.obstacles, [
      { id: 1, x: 4, y: 4, face: 'N' },
      { id: 2, x: 8, y: 8, face: 'E' },
    ]);
    assert.equal(ctl.view().activeFace, 'E', 'the chosen facing stays for the next placement');
    ctl.clickCell(12, 12);
    assert.equal(ctl.localMap.obstacles[2].face, 'E');
    ctl.dispose();
  });

  test('explicit selection (map or list) still corrects or deletes, leaving the next facing alone', () => {
    const { ctl } = setup();
    ctl.chooseFace('E');
    ctl.clickCell(4, 4);
    ctl.clickCell(8, 8);
    ctl.clickCell(4, 4); // select obstacle 1 on the map
    assert.equal(ctl.view().selectedId, 1);
    assert.equal(ctl.view().activeFace, 'E');
    ctl.chooseFace('W');
    assert.deepEqual(ctl.localMap.obstacles.map((o) => o.face), ['W', 'E']);
    ctl.selectObstacle(2); // select obstacle 2 from the list
    ctl.chooseFace('S');
    assert.deepEqual(ctl.localMap.obstacles.map((o) => o.face), ['W', 'S']);
    ctl.deleteSelected();
    assert.deepEqual(ctl.localMap.obstacles.map((o) => o.id), [1]);
    assert.equal(ctl.view().selectedId, null, 'deleting the selection leaves nothing selected');
    ctl.clickCell(4, 4); // select obstacle 1 again
    ctl.clickCell(4, 4); // and deselect it
    assert.equal(ctl.view().selectedId, null);
    assert.equal(ctl.view().activeFace, 'E', 'correcting obstacles never changed the next-placement facing');
    ctl.dispose();
  });
});

describe('editing activity postpones a pending countdown', () => {
  test('selection and facing activity postpone an existing deadline without changing the map', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.clickCell(5, 10); // unplanned edit: countdown to t+5000
    const revision = ctl.revision;
    await clock.advance(4000);
    ctl.clickCell(5, 10); // select
    await clock.advance(4000);
    ctl.clickCell(5, 10); // deselect
    await clock.advance(4000);
    ctl.chooseFace('S'); // facing for the next obstacle
    await clock.advance(4000);
    ctl.selectObstacle(1);
    await clock.advance(4000);
    ctl.selectObstacle(null);
    assert.equal(ctl.revision, revision, 'activity is not a map edit');
    assert.equal(server.posts.length, 0, 'the original deadline long passed, yet nothing was posted');
    assert.equal(ctl.view().status.text, 'Planning starts in 5 s — keep editing to postpone');
    await clock.advance(AUTO_PLAN_IDLE_MS - 1);
    assert.equal(server.posts.length, 0);
    await clock.advance(1);
    assert.equal(server.posts.length, 1);
    assert.deepEqual(server.posts[0].payload.obstacles, [{ id: 1, x: 5, y: 10, face: 'N' }], 'nothing was edited');
    ctl.dispose();
  });

  test('repeated activity still produces only one eventual request', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.clickCell(5, 10);
    for (let i = 0; i < 12; i++) {
      await clock.advance(3000);
      ctl.clickCell(5, 10); // select
      ctl.clickCell(5, 10); // deselect, so the facing below is for the next obstacle
      ctl.chooseFace(FACE_CYCLE[i % 4]);
    }
    assert.equal(server.posts.length, 0);
    await clock.advance(AUTO_PLAN_IDLE_MS);
    assert.equal(server.posts.length, 1);
    await clock.advance(120_000);
    assert.equal(server.posts.length, 1);
    ctl.dispose();
  });

  test('activity after a completed result does not submit the same map again', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(0);
    assert.equal(ctl.view().shown.kind, 'result');
    ctl.clickCell(5, 10); // select
    ctl.clickCell(5, 10); // deselect
    ctl.chooseFace('W');
    ctl.selectObstacle(1);
    ctl.selectObstacle(null);
    ctl.noteActivity();
    assert.notEqual(ctl.view().status.kind, 'countdown', 'no countdown without an unplanned edit');
    await clock.advance(120_000);
    assert.equal(server.posts.length, 1);
    assert.equal(ctl.view().shown.kind, 'result', 'the completed result stays displayed');
    ctl.dispose();
  });

  test('activity does not start a countdown when there is nothing to plan', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.chooseFace('E');
    ctl.selectObstacle(3);
    ctl.noteActivity();
    assert.equal(ctl.view().status.kind, 'empty');
    await clock.advance(120_000);
    assert.equal(server.posts.length, 0);
    ctl.dispose();
  });

  test('activity postpones a follow-up that is already waiting for the planner', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    ctl.clickCell(11, 11);
    await clock.advance(AUTO_PLAN_IDLE_MS + 1000); // queued behind the request in flight
    ctl.selectObstacle(2); // activity postpones it
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(AUTO_PLAN_IDLE_MS - 1);
    assert.equal(server.posts.length, 1);
    await clock.advance(1000);
    assert.equal(server.posts.length, 2);
    assert.deepEqual(ids(server.posts[1].payload), [1, 2]);
    assert.equal(server.count409(), 0);
    ctl.dispose();
  });

  test('activity never starts a countdown while the start input is invalid', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.clickCell(5, 10);
    ctl.setStartInput('', '1', 'N');
    ctl.clickCell(5, 10);
    ctl.chooseFace('E');
    assert.equal(ctl.view().status.kind, 'invalid-start');
    await clock.advance(120_000);
    assert.equal(server.posts.length, 0);
    ctl.dispose();
  });
});

// ---------------------------------------------------------------------------
// One request in flight, stale responses, no duplicates
// ---------------------------------------------------------------------------

describe('submission rules', () => {
  test('an identical accepted map is never submitted twice (result or failure)', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(0);
    assert.equal(ctl.view().shown.kind, 'result');

    ctl.clickCell(12, 12); // a different map...
    assert.equal(ctl.view().shown, null, 'the old route is cleared at once');
    ctl.deleteObstacle(2); // ...and back to the planned one
    assert.equal(ctl.view().shown.kind, 'result');
    assert.equal(ctl.view().shown.fromCache, true);
    await clock.advance(AUTO_PLAN_IDLE_MS * 3);
    assert.equal(server.posts.length, 1);

    ctl.setStartInput('4', '1', 'N');
    await clock.advance(AUTO_PLAN_IDLE_MS);
    const second = server.pending();
    server.finish(second, noRoutePlan(second.payload.mission_id, [1]));
    await clock.advance(0);
    assert.equal(ctl.view().shown.kind, 'failure');
    ctl.setStartInput('2', '1', 'N');
    ctl.setStartInput('4', '1', 'N'); // back to the failed map: no automatic retry
    await clock.advance(AUTO_PLAN_IDLE_MS * 3);
    assert.equal(server.posts.length, 2);
    assert.equal(ctl.view().shown.kind, 'failure');
    ctl.dispose();
  });

  test('only one request is ever in flight', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    await planOnce(env);
    for (let i = 0; i < 4; i++) {
      ctl.clickCell(10 + i, 3);
      await clock.advance(AUTO_PLAN_IDLE_MS + 1000);
    }
    assert.equal(server.posts.length, 1);
    assert.equal(server.posts.filter((p) => !p.settled).length, 1);
    assert.equal(ctl.view().status.kind, 'stale');
    assert.equal(ctl.view().status.text, 'Map changed — the current plan will be discarded.');
    assert.match(ctl.view().status.detail, /newest map is queued/);
    ctl.dispose();
  });

  test('edits during planning discard the stale response and queue exactly one latest follow-up', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    ctl.clickCell(11, 11); // B
    await clock.advance(2000);
    ctl.clickCell(12, 12); // C
    await clock.advance(AUTO_PLAN_IDLE_MS + 2000); // countdown expired while busy
    ctl.clickCell(12, 12); // select obstacle 3 (activity: postpones the queued follow-up)
    ctl.chooseFace('W'); // D: correct its facing; folds into the same follow-up
    assert.equal(ctl.view().shown, null);

    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(0);
    assert.equal(ctl.view().shown, null, 'the stale result is never displayed');
    assert.equal(server.posts.length, 1, 'the follow-up waits for the restarted countdown');

    await clock.advance(AUTO_PLAN_IDLE_MS + 1000);
    assert.equal(server.posts.length, 2, 'exactly one follow-up');
    const followUp = server.posts[1];
    assert.deepEqual(ids(followUp.payload), [1, 2, 3]);
    assert.equal(followUp.payload.obstacles.find((o) => o.id === 3).face, 'W');
    server.finish(followUp, validPlan(followUp.payload.mission_id, [1, 2, 3]));
    await clock.advance(AUTO_PLAN_IDLE_MS * 3);
    assert.equal(ctl.view().shown.kind, 'result');
    assert.deepEqual(ctl.view().shown.result.obstacle_order, [1, 2, 3]);
    assert.equal(server.posts.length, 2);
    assert.equal(server.count409(), 0, 'never a self-inflicted 409');
    ctl.dispose();
  });

  test('a follow-up whose countdown already expired goes out right after the stale reply', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    ctl.clickCell(11, 11);
    await clock.advance(AUTO_PLAN_IDLE_MS + 1000); // queued behind the in-flight request
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(1000); // includes the short grace period
    assert.equal(server.posts.length, 2);
    assert.deepEqual(ids(server.posts[1].payload), [1, 2]);
    assert.equal(server.count409(), 0);
    ctl.dispose();
  });

  test('returning to the map in flight makes its response current again (no duplicate POST)', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    ctl.clickCell(11, 11);
    ctl.deleteObstacle(2);
    assert.equal(ctl.view().status.kind, 'planning');
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(AUTO_PLAN_IDLE_MS * 2);
    assert.equal(ctl.view().shown.kind, 'result');
    assert.equal(server.posts.length, 1);
    ctl.dispose();
  });

  test('the guard needs mission ID, revision and fingerprint to all match', () => {
    const latest = { missionId: 'web-auto-2', revision: 7, fingerprint: 'F' };
    assert.equal(responseMatchesLatest({ ...latest }, latest), true);
    assert.equal(responseMatchesLatest({ ...latest, missionId: 'web-auto-1' }, latest), false);
    assert.equal(responseMatchesLatest({ ...latest, revision: 6 }, latest), false);
    assert.equal(responseMatchesLatest({ ...latest, fingerprint: 'G' }, latest), false);
  });

  test('a reply echoing another mission ID is rejected as malformed', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    server.finish(first, validPlan('web-auto-someone-else', [1]));
    await clock.advance(0);
    const { shown } = ctl.view();
    assert.equal(shown.kind, 'failure');
    assert.match(shown.message, /Malformed plan data: response mission_id does not match the request/);
    assert.equal(shown.result, undefined);
    ctl.dispose();
  });
});

// ---------------------------------------------------------------------------
// /state/v2: reloads, external missions, observe-only, 409, timeouts
// ---------------------------------------------------------------------------

describe('external missions and reloads', () => {
  test('a reload displays the latest snapshot and never re-plans it', async () => {
    const request = buildAutoRequest(addObstacle(emptyMap(), 6, 12, 'S').map, 'web-auto-earlier');
    const { clock, server, ctl } = setup({
      initialState: { mission_id: 'web-auto-earlier', request, status: 'complete', result: validPlan('web-auto-earlier', [1]), error: null },
    });
    await clock.advance(0);
    const view = ctl.view();
    assert.equal(view.shown.kind, 'result');
    assert.equal(view.shown.source, 'external');
    assert.equal(view.externalMissionId, 'web-auto-earlier');
    assert.equal(view.editable, true, 'an on-grid map is adopted for editing');
    assert.deepEqual(view.localObstacles, [{ id: 1, x: 6, y: 12, face: 'S' }]);
    await clock.advance(60_000);
    assert.equal(server.posts.length, 0);
    ctl.dispose();
  });

  test('an external mission cancels the local countdown and is adopted without another POST', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.clickCell(5, 5);
    await clock.advance(1000);
    server.external('rpi-mission-1', rpiRequest('rpi-mission-1'), 'planning');
    await clock.advance(3000); // the next idle poll sees it
    let view = ctl.view();
    assert.equal(view.status.kind, 'external-planning');
    assert.match(view.status.text, /^External mission rpi-mission-1 planning… ≈\d+ s \(approximate, observed by this page\)$/);
    assert.deepEqual(view.localObstacles, [{ id: 1, x: 10, y: 10, face: 'S' }]);
    assert.equal(view.externalMissionId, 'rpi-mission-1');

    await clock.advance(5000);
    server.external('rpi-mission-1', rpiRequest('rpi-mission-1'), 'complete', validPlan('rpi-mission-1', [1]));
    await clock.advance(2000);
    view = ctl.view();
    assert.equal(view.shown.kind, 'result');
    assert.equal(view.shown.source, 'external');
    await clock.advance(60_000);
    assert.equal(server.posts.length, 0, 'the countdown never fired and nothing was re-sent');
    ctl.dispose();
  });

  test('an off-grid or non-cardinal external map is drawn read-only; clearing leaves it', async () => {
    const request = rpiRequest('rpi-off-grid', { start: { x: 23.5, y: 10, heading_deg: 45 } });
    const { clock, server, ctl } = setup({
      initialState: { mission_id: 'rpi-off-grid', request, status: 'complete', result: validPlan('rpi-off-grid', [1]), error: null },
    });
    await clock.advance(0);
    let view = ctl.view();
    assert.equal(view.editable, false);
    assert.equal(view.readOnlyReason, 'off-grid');
    assert.deepEqual(view.map.start, { x_cm: 23.5, y_cm: 10, heading_deg: 45 });
    ctl.clickCell(3, 3);
    assert.equal(ctl.view().map.obstacles.length, 1, 'clicks never edit a read-only map');
    ctl.clearMap();
    view = ctl.view();
    assert.equal(view.editable, true);
    assert.equal(view.obstacleCount, 0);
    await clock.advance(30_000);
    assert.equal(server.posts.length, 0);
    ctl.dispose();
  });

  test('describeRequestMap keeps exact grid maps editable and everything else read-only', () => {
    const grid = describeRequestMap(rpiRequest('a', { units: 'grid', start: { x: 2, y: 1, heading_deg: 450 }, obstacles: [{ id: 3, x: 5, y: 6, face: 'W' }] }));
    assert.deepEqual(grid.editableMap, { start: { x: 2, y: 1, heading: 'N' }, obstacles: [{ id: 3, x: 5, y: 6, face: 'W' }] });
    assert.equal(describeRequestMap(rpiRequest('b', { obstacles: [{ id: 1, x: 105, y: 100, face: 'S' }] })).editableMap, null);
    assert.equal(describeRequestMap(rpiRequest('c', { obstacles: [{ id: 9, x: 100, y: 100, face: 'S' }] })).editableMap, null);
    assert.equal(describeRequestMap(rpiRequest('d', { obstacles: [{ id: 1, x: 100, y: 100, face: 'Q' }] })), null);
    assert.equal(describeRequestMap(null), null);
  });

  test('?observe=1 is display-only: it never sends a POST, yet shows external missions', async () => {
    const { clock, server, ctl } = setup({ observeOnly: true });
    await clock.advance(0);
    ctl.clickCell(5, 5);
    ctl.setStartInput('4', '4', 'E');
    assert.equal(ctl.view().editable, false);
    assert.equal(ctl.view().obstacleCount, 0);
    server.external('rpi-observed', rpiRequest('rpi-observed'), 'planning');
    await clock.advance(3000);
    assert.equal(ctl.view().status.kind, 'external-planning');
    assert.match(ctl.view().status.text, /approximate/);
    assert.equal(ctl.view().readOnlyReason, 'observe');
    server.external('rpi-observed', rpiRequest('rpi-observed'), 'complete', validPlan('rpi-observed', [1]));
    await clock.advance(3000);
    assert.equal(ctl.view().shown.kind, 'result');
    await clock.advance(120_000);
    assert.equal(server.posts.length, 0);
    ctl.dispose();
  });

  test('HTTP 409 switches to observing and keeps at most one newest local map', async () => {
    const { clock, server, ctl } = setup();
    await clock.advance(0);
    ctl.clickCell(5, 5);
    ctl.clickCell(6, 6);
    server.busyOnNextPost = { missionId: 'rpi-race', request: rpiRequest('rpi-race') };
    await clock.advance(AUTO_PLAN_IDLE_MS + 1500);
    assert.equal(server.count409(), 1);
    let view = ctl.view();
    assert.equal(view.status.kind, 'busy');
    assert.match(view.status.text, /External mission rpi-race planning/);
    assert.match(view.status.detail, /newest map is queued/);
    assert.equal(view.readOnlyReason, 'busy');
    assert.equal(view.externalMissionId, 'rpi-race');

    await clock.advance(10_000);
    assert.equal(server.posts.length, 1, 'nothing is posted while the planner is busy');
    server.external('rpi-race', rpiRequest('rpi-race'), 'complete', validPlan('rpi-race', [1]));
    await clock.advance(3000);
    assert.equal(server.posts.length, 2, 'the queued map follows once the planner is free');
    assert.deepEqual(ids(server.posts[1].payload), [1, 2]);
    view = ctl.view();
    assert.equal(view.status.kind, 'planning');
    assert.equal(view.editable, true);
    await clock.advance(10_000);
    assert.equal(server.posts.length, 2, 'exactly one queued submission');
    ctl.dispose();
  });

  test('edits while an external mission plans wait for it instead of earning a 409', async () => {
    const { clock, server, ctl } = setup();
    const request = rpiRequest('rpi-long', { units: 'grid', start: { x: 2, y: 1, heading_deg: 90 }, obstacles: [{ id: 1, x: 10, y: 10, face: 'S' }] });
    server.external('rpi-long', request, 'planning');
    await clock.advance(3000); // the next idle poll adopts it
    assert.equal(ctl.view().externalMissionId, 'rpi-long');
    assert.equal(ctl.view().editable, true, 'an exact grid map is adopted for editing');
    ctl.clickCell(3, 15); // edit the adopted map
    await clock.advance(AUTO_PLAN_IDLE_MS + 3000);
    assert.equal(server.posts.length, 0);
    assert.equal(ctl.view().status.kind, 'busy');
    server.external('rpi-long', rpiRequest('rpi-long'), 'complete', validPlan('rpi-long', [1]));
    await clock.advance(3000);
    assert.equal(server.posts.length, 1);
    assert.deepEqual(ids(server.posts[0].payload), [1, 2]);
    assert.equal(server.count409(), 0);
    ctl.dispose();
  });

  test('a browser timeout consults /state/v2 for the same mission before failing', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    // The server finished, but its reply never reached the browser.
    server.state = { ...server.state, status: 'complete', result: validPlan(first.payload.mission_id, [1]) };
    const callsBefore = server.stateCalls;
    await clock.advance(BROWSER_TIMEOUT_MS);
    assert.equal(first.signal.aborted, true);
    assert.ok(server.stateCalls > callsBefore);
    assert.equal(ctl.view().shown.kind, 'result', 'recovered from /state/v2');
    ctl.dispose();
  });

  test('a browser timeout with no finished mission in /state/v2 is a clear failure, not retried', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    await planOnce(env);
    server.state = { mission_id: 'someone-else', request: null, status: 'complete', result: null, error: null };
    await clock.advance(BROWSER_TIMEOUT_MS);
    const { shown } = ctl.view();
    assert.equal(shown.kind, 'failure');
    assert.match(shown.message, /No reply from the planner within 75 s, and \/state\/v2 holds no finished result/);
    await clock.advance(AUTO_PLAN_IDLE_MS * 4);
    assert.equal(server.posts.length, 1);
    ctl.dispose();
  });
});

// ---------------------------------------------------------------------------
// Outcomes
// ---------------------------------------------------------------------------

describe('evaluatePlanResponse', () => {
  const ok = (body, httpStatus = 200) => ({ httpStatus, body, jsonOk: true });

  test('an executable result is shown with the proof-driven label and backend text', () => {
    const outcome = evaluatePlanResponse(ok(validPlan('m', [1, 2])), 'm');
    assert.equal(outcome.kind, 'result');
    assert.equal(outcome.label, 'Best safe complete route found within the planning window — optimality not proven');
    assert.equal(outcome.backendMessage, 'Best complete verified route found. Optimality not proven.');
    const proven = evaluatePlanResponse(ok(validPlan('m', [1], { optimality_proven: true })), 'm');
    assert.equal(proven.label, 'Optimal route (proven by search)');
    const truthy = evaluatePlanResponse(ok(validPlan('m', [1], { optimality_proven: 'true' })), 'm');
    assert.notEqual(truthy.label, 'Optimal route (proven by search)');
  });

  test('every failure carries no route, order or commands', () => {
    const cases = [
      evaluatePlanResponse(ok(noRoutePlan('m')), 'm'),
      evaluatePlanResponse(ok(validPlan('m', [1], { complete: false })), 'm'),
      evaluatePlanResponse(ok(validPlan('m', [1], { actions: [], waypoints: [] })), 'm'),
      evaluatePlanResponse(ok({ ...validPlan('m', [1]), actions: validPlan('m', [1]).actions.map((a) => (a.type === 'SNAP' ? { ...a, pose: { ...a.pose, x_cm: 21 } } : a)) }), 'm'),
      evaluatePlanResponse(ok({ type: 'PLAN_ERROR', error: 'bad start' }, 400), 'm'),
      evaluatePlanResponse({ httpStatus: 200, body: null, jsonOk: false }, 'm'),
      evaluatePlanResponse(ok({ type: 'SOMETHING' }), 'm'),
    ];
    for (const outcome of cases) {
      assert.equal(outcome.kind, 'failure');
      assert.equal(outcome.result, undefined);
      assert.equal(typeof outcome.message, 'string');
    }
  });

  test('impossibility is claimed only when complete_route_proven_impossible === true', () => {
    const notProven = evaluatePlanResponse(ok(noRoutePlan('m')), 'm');
    assert.equal(notProven.message, 'No complete route was found within the planning window; non-existence was not proven.');
    const proven = evaluatePlanResponse(ok(noRoutePlan('m', [1], { complete_route_proven_impossible: true })), 'm');
    assert.match(proven.message, /^No complete route exists for this map \(proven by search/);
    const truthy = evaluatePlanResponse(ok(noRoutePlan('m', [1], { complete_route_proven_impossible: 1 })), 'm');
    assert.equal(truthy.message, notProven.message);
  });

  test('HTTP 409 is "busy", never a failure of the map', () => {
    assert.deepEqual(evaluatePlanResponse(ok({ type: 'PLAN_ERROR' }, 409), 'm'), { kind: 'busy' });
  });

  test('only HTTP 200 may be evaluated as a PLAN_RESULT: other statuses fail closed', () => {
    const valid = (id = 'm') => validPlan(id, [1, 2]);
    for (const status of [500, 502, 503, 201, 202, 204, 301, 404]) {
      const outcome = evaluatePlanResponse(ok(valid(), status), 'm');
      assert.equal(outcome.kind, 'failure', `HTTP ${status}`);
      assert.equal(outcome.cacheable, false, `HTTP ${status} is never cached`);
      assert.equal(outcome.title, 'Planning request failed');
      assert.ok(outcome.message.includes(`Unexpected HTTP status ${status}`), outcome.message);
      assert.equal(outcome.result, undefined, `HTTP ${status} exposes no result`);
      assert.equal(outcome.label, undefined);
      assert.equal(outcome.backendMessage, null);
      assert.ok(!JSON.stringify(outcome).includes('MOV,F,20'), 'no command text leaks');
    }
    // A valid-looking body with an unexpected 400 (not a PLAN_ERROR) is no result either.
    const odd400 = evaluatePlanResponse(ok(valid(), 400), 'm');
    assert.equal(odd400.kind, 'failure');
    assert.equal(odd400.cacheable, false);
    assert.equal(odd400.result, undefined);
    // The same body at 200 is still the one accepted shape.
    assert.equal(evaluatePlanResponse(ok(valid(), 200), 'm').kind, 'result');
  });

  test('HTTP 500 carrying a PLAN_ERROR, or a non-JSON body, is a non-cacheable failure', () => {
    const planError = evaluatePlanResponse(ok({ type: 'PLAN_ERROR', error: 'planner failed unexpectedly' }, 500), 'm');
    assert.equal(planError.kind, 'failure');
    assert.equal(planError.cacheable, false);
    assert.match(planError.message, /^Unexpected HTTP status 500 from the planner: planner failed unexpectedly\.$/);
    const html = evaluatePlanResponse({ httpStatus: 500, body: null, jsonOk: false }, 'm');
    assert.equal(html.kind, 'failure');
    assert.equal(html.cacheable, false);
    assert.match(html.message, /Unexpected HTTP status 500/);
  });

  test('an ordinary HTTP 400 PLAN_ERROR stays a rejected request', () => {
    const rejected = evaluatePlanResponse(ok({ type: 'PLAN_ERROR', error: 'bad start' }, 400), 'm');
    assert.equal(rejected.kind, 'failure');
    assert.equal(rejected.cacheable, true);
    assert.equal(rejected.message, 'The planner rejected the request: bad start');
  });

  test('through the controller: an HTTP 500 with a valid-looking result shows no route and is not cached', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    server.finish(first, validPlan(first.payload.mission_id, [1]), 500);
    await clock.advance(0);
    const { shown } = ctl.view();
    assert.equal(shown.kind, 'failure');
    assert.equal(shown.result, undefined);
    assert.match(shown.message, /Unexpected HTTP status 500/);
    await clock.advance(120_000);
    assert.equal(server.posts.length, 1, 'a failed map is not retried automatically');
    // Not cached: going away and coming back to the same map plans it afresh.
    ctl.clickCell(12, 12);
    ctl.deleteObstacle(2);
    await clock.advance(AUTO_PLAN_IDLE_MS + 1000);
    assert.equal(server.posts.length, 2);
    ctl.dispose();
  });
});

// ---------------------------------------------------------------------------
// STM motion profile: the page requests STANDARD and shows the setup first
// ---------------------------------------------------------------------------

describe('STM profile setup', () => {
  const ok = (body, httpStatus = 200) => ({ httpStatus, body, jsonOk: true });
  const planWith = (extra) => validPlan('m', [1, 2], extra);
  const SETUP_STANDARD = [
    { command: 'PROFILE,STANDARD', expect: 'PROFILE,OK' },
    { command: 'PROFILE,GET', expect: 'PROFILE,STANDARD' },
  ];

  test('the page names STANDARD itself, and the profile is part of the map fingerprint', () => {
    assert.equal(AUTO_MOTION_PROFILE, 'STANDARD');
    const map = addObstacle(emptyMap(), 5, 5, 'N').map;
    assert.equal(buildAutoRequest(map, 'x').motion_profile, 'STANDARD');
    assert.ok(JSON.parse(mapFingerprint(map)).motion_profile === 'STANDARD');
  });

  test('an executable STANDARD result carries the two setup steps, set first then verify', () => {
    const outcome = evaluatePlanResponse(ok(planWith({})), 'm', AUTO_MOTION_PROFILE);
    assert.equal(outcome.kind, 'result');
    assert.deepEqual(outcome.setupRows, SETUP_STANDARD);
  });

  test('a reply that echoes a different profile than the one requested fails closed', () => {
    const precision = planWith({ motion_profile: 'PRECISION', stm_setup: stmSetupFor('PRECISION') });
    const outcome = evaluatePlanResponse(ok(precision), 'm', AUTO_MOTION_PROFILE);
    assert.equal(outcome.kind, 'failure');
    assert.equal(outcome.cacheable, false);
    assert.match(outcome.message, /motion_profile PRECISION does not match the requested STANDARD/);
    assert.equal(outcome.result, undefined);
    assert.equal(outcome.setupRows, undefined);
  });

  test('a missing, altered or extended stm_setup fails closed, exposing no commands', () => {
    const good = stmSetupFor('STANDARD');
    const { stm_setup: _drop, ...withoutSetup } = planWith({});
    const variants = {
      missing: withoutSetup,
      null: planWith({ stm_setup: null }),
      array: planWith({ stm_setup: [good.set_command, good.verify_command] }),
      wrongSet: planWith({ stm_setup: { ...good, set_command: 'PROFILE,PRECISION' } }),
      newline: planWith({ stm_setup: { ...good, set_command: 'PROFILE,STANDARD\n' } }),
      lowercase: planWith({ stm_setup: { ...good, set_command: 'profile,standard' } }),
      wrongSetReply: planWith({ stm_setup: { ...good, expected_set_reply: 'OK' } }),
      wrongVerifyCommand: planWith({ stm_setup: { ...good, verify_command: 'PROFILE,SET' } }),
      wrongVerifyReply: planWith({ stm_setup: { ...good, expected_verify_reply: 'PROFILE,PRECISION' } }),
      extraField: planWith({ stm_setup: { ...good, extra: 'x' } }),
      missingField: planWith({ stm_setup: { set_command: good.set_command } }),
    };
    for (const [name, body] of Object.entries(variants)) {
      const outcome = evaluatePlanResponse(ok(body), 'm', AUTO_MOTION_PROFILE);
      assert.equal(outcome.kind, 'failure', name);
      assert.equal(outcome.result, undefined, name);
      assert.equal(outcome.setupRows, undefined, name);
      assert.match(outcome.message, /^Malformed plan data: stm_setup does not match the exact STANDARD setup$/, name);
    }
  });

  test('an unknown, lowercase, CUSTOM or missing motion_profile fails closed', () => {
    for (const profile of ['precision', 'standard', 'CUSTOM', 'FAST', '', null, undefined, 1]) {
      const body = planWith({ motion_profile: profile });
      const outcome = evaluatePlanResponse(ok(body), 'm', AUTO_MOTION_PROFILE);
      assert.equal(outcome.kind, 'failure', String(profile));
      assert.match(outcome.message, /unknown motion_profile/, String(profile));
      assert.equal(outcome.setupRows, undefined);
    }
  });

  test('a no-route result exposes no setup rows', () => {
    const outcome = evaluatePlanResponse(ok(noRoutePlan('m', [1])), 'm', AUTO_MOTION_PROFILE);
    assert.equal(outcome.kind, 'failure');
    assert.equal(outcome.setupRows, undefined);
    assert.equal(outcome.result, undefined);
  });

  test('an external mission is only checked for self-consistency: its own PRECISION setup is shown as PRECISION', () => {
    const precision = planWith({ motion_profile: 'PRECISION', stm_setup: stmSetupFor('PRECISION') });
    const outcome = evaluatePlanResponse(ok(precision), 'm');
    assert.equal(outcome.kind, 'result');
    assert.deepEqual(outcome.setupRows, [
      { command: 'PROFILE,PRECISION', expect: 'PROFILE,OK' },
      { command: 'PROFILE,GET', expect: 'PROFILE,PRECISION' },
    ]);
    // ...but a PRECISION result carrying the STANDARD setup contradicts itself.
    const mixed = planWith({ motion_profile: 'PRECISION', stm_setup: stmSetupFor('STANDARD') });
    assert.equal(evaluatePlanResponse(ok(mixed), 'm').kind, 'failure');
  });

  test('an external request only stands in for this page\'s own request when it names STANDARD', () => {
    const base = { mode: 'B3', allow_alternative_viewing: false };
    assert.equal(matchesAutoDefaults({ ...base, motion_profile: 'STANDARD' }), true);
    assert.equal(matchesAutoDefaults({ ...base, motion_profile: 'PRECISION' }), false);
    assert.equal(matchesAutoDefaults(base), false, 'an omitted profile is never assumed to match');
    assert.equal(matchesAutoDefaults({ ...base, mode: 'B2', motion_profile: 'STANDARD' }), false);
  });

  test('through the controller: a reply with the wrong profile is shown as a failure, never as commands', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    assert.equal(first.payload.motion_profile, 'STANDARD');
    server.finish(first, validPlan(first.payload.mission_id, [1], {
      motion_profile: 'PRECISION', stm_setup: stmSetupFor('PRECISION'),
    }));
    await clock.advance(0);
    const { shown } = ctl.view();
    assert.equal(shown.kind, 'failure');
    assert.equal(shown.result, undefined);
    assert.equal(shown.setupRows, undefined);
    ctl.dispose();
  });

  test('through the controller: recovering from a browser timeout checks the profile too', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    server.state = {
      ...server.state,
      status: 'complete',
      result: validPlan(first.payload.mission_id, [1], { motion_profile: 'PRECISION', stm_setup: stmSetupFor('PRECISION') }),
    };
    await clock.advance(BROWSER_TIMEOUT_MS);
    assert.equal(ctl.view().shown.kind, 'failure');
    assert.equal(ctl.view().shown.setupRows, undefined);
    ctl.dispose();
  });

  test('through the controller: a valid STANDARD result, and an external mission, both carry the setup rows', async () => {
    const env = setup();
    const { clock, server, ctl } = env;
    const first = await planOnce(env);
    server.finish(first, validPlan(first.payload.mission_id, [1]));
    await clock.advance(0);
    assert.deepEqual(ctl.view().shown.setupRows, SETUP_STANDARD);
    ctl.dispose();

    const other = setup();
    other.server.external('rpi-9', rpiRequest('rpi-9'), 'complete', validPlan('rpi-9', [1]));
    await other.clock.advance(3000);
    assert.deepEqual(other.ctl.view().shown.setupRows, SETUP_STANDARD);
    other.ctl.dispose();
  });
});
