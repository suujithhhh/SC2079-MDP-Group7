// Shared helpers for the automatic-page tests (not a test file itself: the
// `npm test` glob only picks up *.test.js). Node built-ins only.

/** Flushes pending promise callbacks (real `setImmediate`, never faked). */
export async function flush(rounds = 25) {
  for (let i = 0; i < rounds; i++) await new Promise((resolve) => setImmediate(resolve));
}

/** Deterministic clock: timers fire only inside `advance()`, in time order,
 * with promise callbacks flushed between them. */
export class FakeClock {
  constructor(start = 1_000_000) {
    this.t = start;
    this.timers = new Map();
    this.seq = 0;
    this.now = () => this.t;
    this.setTimeout = (fn, ms = 0) => this.add(fn, ms, null);
    this.clearTimeout = (id) => { this.timers.delete(id); };
    this.setInterval = (fn, ms = 0) => this.add(fn, ms, Math.max(1, ms));
    this.clearInterval = (id) => { this.timers.delete(id); };
  }

  add(fn, ms, interval) {
    const id = ++this.seq;
    this.timers.set(id, { at: this.t + Math.max(0, ms || 0), fn, interval });
    return id;
  }

  async advance(ms) {
    const end = this.t + ms;
    for (;;) {
      await flush();
      let next = null;
      for (const [id, timer] of this.timers) {
        if (timer.at > end) continue;
        if (!next || timer.at < next.timer.at || (timer.at === next.timer.at && id < next.id)) next = { id, timer };
      }
      if (!next) break;
      this.t = next.timer.at;
      if (next.timer.interval) next.timer.at += next.timer.interval;
      else this.timers.delete(next.id);
      next.timer.fn();
    }
    this.t = end;
    await flush();
  }
}

const IDLE = { mission_id: null, request: null, status: 'idle', result: null, error: null };

/**
 * A scripted stand-in for `server.py`'s v2 endpoints with the same
 * single-flight rule: a POST while `/state/v2` says `planning` gets HTTP 409
 * and leaves the state untouched; otherwise the state becomes `planning` for
 * that mission until the test finishes it.
 */
export function makeServer() {
  const server = {
    state: { ...IDLE },
    posts: [],
    stateCalls: 0,
    busyOnNextPost: null, // { missionId, request }: an external mission wins the race
    transport: {
      postPlan(payload, signal) {
        return new Promise((resolve, reject) => {
          const entry = { payload, signal, settled: false, httpStatus: null };
          entry.reply = (httpStatus, body, jsonOk = true) => {
            if (entry.settled) return;
            entry.settled = true;
            entry.httpStatus = httpStatus;
            resolve({ httpStatus, body, jsonOk });
          };
          entry.fail = (error) => {
            if (entry.settled) return;
            entry.settled = true;
            reject(error);
          };
          signal?.addEventListener('abort', () => entry.fail(Object.assign(new Error('The operation was aborted'), { name: 'AbortError' })));
          server.posts.push(entry);
          if (server.busyOnNextPost) {
            const { missionId, request } = server.busyOnNextPost;
            server.busyOnNextPost = null;
            server.state = { mission_id: missionId, request, status: 'planning', result: null, error: null };
          }
          if (server.state.status === 'planning') {
            entry.reply(409, { type: 'PLAN_ERROR', protocol_version: 2, valid: false, error: 'planner is busy processing another v2 request' });
            return;
          }
          server.state = { mission_id: payload.mission_id, request: payload, status: 'planning', result: null, error: null };
        });
      },
      async getState() {
        server.stateCalls += 1;
        return structuredClone(server.state);
      },
    },
    /** The in-flight own POST (last one that is not settled). */
    pending() {
      return [...server.posts].reverse().find((p) => !p.settled) || null;
    },
    /** Completes a POST: state becomes `complete`, then the reply is sent. */
    finish(entry, body, httpStatus = 200) {
      server.state = { mission_id: entry.payload.mission_id, request: entry.payload, status: 'complete', result: body, error: null };
      entry.reply(httpStatus, body);
    },
    external(missionId, request, status = 'planning', result = null, error = null) {
      server.state = { mission_id: missionId, request, status, result, error };
    },
    count409() {
      return server.posts.filter((p) => p.httpStatus === 409).length;
    },
  };
  return server;
}

const pose = (i) => ({ x_cm: 20, y_cm: 20 + 10 * i, heading_deg: 90 });

/** The exact `stm_setup` the planner returns for a profile (`PROTOCOL.md` §4.1),
 * spelled out here independently of the code under test. */
export const stmSetupFor = (profile) => ({
  set_command: `PROFILE,${profile}`,
  expected_set_reply: 'PROFILE,OK',
  verify_command: 'PROFILE,GET',
  expected_verify_reply: `PROFILE,${profile}`,
});

/** A schema-shaped, internally consistent executable v2 result: per obstacle
 * one MOV over three waypoints, then a SNAP reusing the MOV's last waypoint. */
export function validPlan(missionId, ids = [1], extra = {}) {
  const waypoints = [pose(0)];
  const actions = [];
  let seq = 0;
  let cursor = 0;
  for (const id of ids) {
    waypoints.push(pose(cursor + 1), pose(cursor + 2));
    actions.push({
      seq: seq++, type: 'MOV', dir: 'F', value: 20, unit: 'cm', stm_command: 'MOV,F,20',
      start_pose: pose(cursor), end_pose: pose(cursor + 2),
      waypoint_start_index: cursor, waypoint_end_index: cursor + 2,
    });
    cursor += 2;
    actions.push({ seq: seq++, type: 'SNAP', obstacle_id: id, pose: pose(cursor) });
  }
  return {
    type: 'PLAN_RESULT',
    protocol_version: 2,
    mission_id: missionId,
    mode: 'B3',
    termination: 'search_finished',
    delivery: 'on_time',
    optimality_proven: false,
    complete_route_proven_impossible: false,
    search_space_exhausted: false,
    message: 'Best complete verified route found. Optimality not proven.',
    motion_profile: 'STANDARD',
    stm_setup: stmSetupFor('STANDARD'),
    valid: true,
    complete: true,
    final_pose: pose(cursor),
    obstacle_order: [...ids],
    skipped_obstacle_ids: [],
    target_poses: ids.map((_, k) => pose(2 * k + 2)),
    actions,
    waypoints,
    total_distance_cm: 20 * ids.length,
    total_cost: 20 * ids.length,
    failure_reason: null,
    ...extra,
  };
}

/** The bounded no-route shape: valid/complete false, no executable geometry. */
export function noRoutePlan(missionId, ids = [1], extra = {}) {
  const message = 'No complete route found within budget (planning budget); non-existence is not proven.';
  return {
    type: 'PLAN_RESULT',
    protocol_version: 2,
    mission_id: missionId,
    mode: 'B3',
    termination: 'planning_budget',
    delivery: 'on_time',
    optimality_proven: false,
    complete_route_proven_impossible: false,
    search_space_exhausted: false,
    message,
    motion_profile: 'STANDARD',
    stm_setup: stmSetupFor('STANDARD'),
    valid: false,
    complete: false,
    final_pose: null,
    obstacle_order: [],
    skipped_obstacle_ids: [...ids],
    target_poses: [],
    actions: [],
    waypoints: [],
    total_distance_cm: 0,
    total_cost: 0,
    failure_reason: message,
    error: message,
    ...extra,
  };
}

/** A v2 request as an RPi might send it (centimetres by default). It names no
 * motion profile, as a real RPi request may not. */
export function rpiRequest(missionId, { units = 'cm', start = { x: 20, y: 10, heading_deg: 90 }, obstacles = [{ id: 1, x: 100, y: 100, face: 'S' }] } = {}) {
  return {
    type: 'PLAN_REQUEST',
    protocol_version: 2,
    mission_id: missionId,
    mode: 'B3',
    coordinate_units: units,
    allow_alternative_viewing: false,
    start,
    obstacles,
  };
}
