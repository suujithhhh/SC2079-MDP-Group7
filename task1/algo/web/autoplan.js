/**
 * Automatic planning for the normal page (`/`, wired up by `auto.js`).
 *
 * No DOM access. Time, timers and HTTP are injected into
 * `AutoPlanController`, so Node's test runner can drive every rule here with a
 * fake clock and a scripted transport (`tests/autoplan.test.js`).
 *
 * Rules (the page states them to the user):
 * - The user edits only the start pose and 1-8 obstacles. Every real edit
 *   restarts a visible `AUTO_PLAN_IDLE_MS` countdown; when it expires, exactly
 *   one bounded protocol-v2 request is sent with fixed values (B3, grid units,
 *   no alternative viewing, STANDARD, a `web-auto-...` mission ID).
 * - The countdown is client-side and lies outside the server's request
 *   window. The bounded request itself returns within its server-owned 90 s
 *   window after the server accepts it.
 * - One request in flight at a time. An edit during planning marks that
 *   response stale (it is never shown) and queues one follow-up for the newest
 *   map; further edits fold into that same follow-up. A response is shown only
 *   when its mission ID, map revision and request fingerprint all match the
 *   latest accepted local map.
 * - The same accepted map is never submitted twice: an identical map reuses
 *   the request already in flight or the outcome already received.
 * - `/state/v2` is the authority for global planner state. Missions this page
 *   did not create are adopted and drawn, never re-submitted. Observe-only
 *   mode never sends a planning request.
 * - Executable results pass the shared fail-closed gate
 *   (`validateV2Result()` in `planv2.js`); every failure exposes no route,
 *   order or commands.
 */

import {
  MAX_OBSTACLES_V2,
  buildPlanV2Request,
  generateMissionId,
  headingLetterToDeg,
  isExecutableV2Result,
  noRouteExplanationV2,
  normalizeHeadingDeg,
  optimalityLabelV2,
  validateStmSetup,
  validateV2Result,
} from './planv2.js';
import { CELL_CM, GRID } from './arena.js';

export const AUTO_PLAN_IDLE_MS = 5000;
/** The STM motion profile this page requests by name (never left to the
 * server default, so the page's own plans do not depend on it). */
export const AUTO_MOTION_PROFILE = 'STANDARD';
/** Browser-side abort: the server's 90 s ceiling plus 5 s of transport margin
 * (the same margin `rpi_test_client_v2.py` uses). */
export const BROWSER_TIMEOUT_MS = 95000;
export const POLL_PLANNING_MS = 1000;
export const POLL_IDLE_MS = 2500;
/** Pause after a response before a queued follow-up is sent, so the server
 * has released its single-flight guard (it writes the reply first). */
export const FOLLOW_UP_GRACE_MS = 500;
const OUTCOME_CACHE_LIMIT = 10;

export const FACES = ['N', 'E', 'S', 'W'];
const HEADING_LETTER_BY_DEG = { 0: 'E', 90: 'N', 180: 'W', 270: 'S' };

export const PLANNING_TEXT = (seconds) =>
  `Planning… ${seconds} s · target ≈67 s · hard stop 81 s · reply by 90 s`;
export const COUNTDOWN_TEXT = (seconds) => `Planning starts in ${seconds} s — keep editing to postpone`;
export const WINDOW_NOTE =
  'The bounded request returns within its server-owned 90 s window after the server accepts it; '
  + 'the 5 s editing countdown is not part of that window.';

// ---------------------------------------------------------------------------
// Map model (grid cells) and pure edits. Every edit returns a new map, or
// `null` when it would change nothing (a non-edit never restarts the timer).
// ---------------------------------------------------------------------------

export function emptyMap() {
  // Rear reference (rear-axle midpoint) in grid cells: (2, 1) = (20, 10) cm.
  return { start: { x: 2, y: 1, heading: 'N' }, obstacles: [] };
}

/** Lowest ID in 1..MAX_OBSTACLES_V2 not used by `obstacles`, or null. */
export function lowestFreeObstacleId(obstacles) {
  for (let id = 1; id <= MAX_OBSTACLES_V2; id++) {
    if (!obstacles.some((o) => o.id === id)) return id;
  }
  return null;
}

export function obstacleAt(map, x, y) {
  return map.obstacles.find((o) => o.x === x && o.y === y) || null;
}

/** Adds an obstacle at cell (x, y). Returns `{ map, id }`, or null when the
 * cell is taken or the map already holds the maximum. */
export function addObstacle(map, x, y, face) {
  if (!FACES.includes(face) || obstacleAt(map, x, y)) return null;
  const id = lowestFreeObstacleId(map.obstacles);
  if (id === null) return null;
  return { map: { ...map, obstacles: [...map.obstacles, { id, x, y, face }] }, id };
}

export function setObstacleFace(map, id, face) {
  const target = map.obstacles.find((o) => o.id === id);
  if (!target || !FACES.includes(face) || target.face === face) return null;
  return { ...map, obstacles: map.obstacles.map((o) => (o.id === id ? { ...o, face } : o)) };
}

export function removeObstacle(map, id) {
  if (!map.obstacles.some((o) => o.id === id)) return null;
  return { ...map, obstacles: map.obstacles.filter((o) => o.id !== id) };
}

export function clearObstacles(map) {
  return map.obstacles.length ? { ...map, obstacles: [] } : null;
}

export function setStart(map, start) {
  const s = map.start;
  if (s.x === start.x && s.y === start.y && s.heading === start.heading) return null;
  return { ...map, start: { ...start } };
}

/** Validates the start inputs exactly as typed. Returns `{ start }` or
 * `{ problem }`; a problem never changes the accepted map. */
export function parseStartInput(xText, yText, heading) {
  const cell = (text, axis) => {
    const t = String(text ?? '').trim();
    if (t === '') return { problem: `Enter the start ${axis} cell (0–19).` };
    if (!/^\d+$/.test(t) || Number(t) > GRID - 1) {
      return { problem: `Start ${axis} must be a whole number of cells from 0 to 19.` };
    }
    return { value: Number(t) };
  };
  const x = cell(xText, 'X');
  if (x.problem) return { problem: x.problem };
  const y = cell(yText, 'Y');
  if (y.problem) return { problem: y.problem };
  if (!FACES.includes(heading)) return { problem: 'Choose a start heading.' };
  return { start: { x: x.value, y: y.value, heading } };
}

/** Grid map -> centimetre map for drawing (`arena.js` draws in cm). */
export function toDisplayMap(map) {
  return {
    start: { x_cm: map.start.x * CELL_CM, y_cm: map.start.y * CELL_CM, heading_deg: headingLetterToDeg(map.start.heading) },
    obstacles: map.obstacles.map((o) => ({ id: o.id, x: o.x * CELL_CM, y: o.y * CELL_CM, face: o.face })),
  };
}

// ---------------------------------------------------------------------------
// Request and fingerprint
// ---------------------------------------------------------------------------

/** The exact `/plan/v2` request this page sends: fixed B3, grid units, no
 * alternative viewing, explicit STANDARD profile. Obstacles are sorted by ID
 * so the request and its fingerprint are independent of placement order. */
export function buildAutoRequest(map, missionId) {
  const obstacles = [...map.obstacles].sort((a, b) => a.id - b.id);
  return {
    ...buildPlanV2Request(
      { mode: 'B3', coordinateUnits: 'grid', allowAlternativeViewing: false, start: map.start, obstacles },
      missionId,
    ),
    motion_profile: AUTO_MOTION_PROFILE,
  };
}

/** Identity of an accepted map: the auto request without its mission ID. */
export function mapFingerprint(map) {
  const { mission_id: _missionId, ...request } = buildAutoRequest(map, 'fingerprint');
  return JSON.stringify(request);
}

/** True when an external request asked for what this page would ask for, so
 * its outcome may stand in for this page's own request of the same map. The
 * profile must be named: an omitted one is never assumed to match. */
export function matchesAutoDefaults(request) {
  return request?.mode === 'B3'
    && (request.allow_alternative_viewing ?? false) === false
    && request.motion_profile === AUTO_MOTION_PROFILE;
}

/** The response guard: shown only when mission ID, revision and fingerprint
 * all match the latest accepted local map. */
export function responseMatchesLatest(request, latest) {
  return request.missionId === latest.missionId
    && request.revision === latest.revision
    && request.fingerprint === latest.fingerprint;
}

// ---------------------------------------------------------------------------
// External requests (from /state/v2): drawn generically, editable only when
// they are exactly representable in this page's grid model.
// ---------------------------------------------------------------------------

const finite = (v) => typeof v === 'number' && Number.isFinite(v);

/** Returns `{ displayMap, editableMap }` for a v2 request (`editableMap` is
 * null for off-grid centimetres, non-cardinal headings or IDs outside 1-8),
 * or null when the request cannot be drawn at all. */
export function describeRequestMap(request) {
  if (!request || typeof request !== 'object') return null;
  const units = request.coordinate_units;
  if (units !== 'grid' && units !== 'cm') return null;
  const scale = units === 'grid' ? CELL_CM : 1;
  const s = request.start;
  if (!s || !finite(s.x) || !finite(s.y) || !finite(s.heading_deg)) return null;
  if (!Array.isArray(request.obstacles)) return null;
  for (const o of request.obstacles) {
    if (!o || !Number.isInteger(o.id) || !finite(o.x) || !finite(o.y) || !FACES.includes(o.face)) return null;
  }
  const displayMap = {
    start: { x_cm: s.x * scale, y_cm: s.y * scale, heading_deg: normalizeHeadingDeg(s.heading_deg) },
    obstacles: request.obstacles.map((o) => ({ id: o.id, x: o.x * scale, y: o.y * scale, face: o.face })),
  };
  return { displayMap, editableMap: toEditableMap(displayMap) };
}

function toEditableMap(displayMap) {
  const cell = (cm) => {
    const c = cm / CELL_CM;
    return Number.isInteger(c) && c >= 0 && c < GRID ? c : null;
  };
  const heading = HEADING_LETTER_BY_DEG[displayMap.start.heading_deg];
  const x = cell(displayMap.start.x_cm);
  const y = cell(displayMap.start.y_cm);
  if (!heading || x === null || y === null || displayMap.obstacles.length > MAX_OBSTACLES_V2) return null;
  const obstacles = [];
  for (const o of displayMap.obstacles) {
    const ox = cell(o.x);
    const oy = cell(o.y);
    if (ox === null || oy === null || o.id < 1 || o.id > MAX_OBSTACLES_V2) return null;
    if (obstacles.some((p) => p.id === o.id || (p.x === ox && p.y === oy))) return null;
    obstacles.push({ id: o.id, x: ox, y: oy, face: o.face });
  }
  return { start: { x, y, heading }, obstacles };
}

// ---------------------------------------------------------------------------
// Outcomes
// ---------------------------------------------------------------------------

/**
 * Turns one `/plan/v2` reply into what the page may show:
 * - `{ kind: 'busy' }` for HTTP 409 (the request never ran);
 * - `{ kind: 'result', result, label, backendMessage, setupRows }` only for an
 *   HTTP 200 executable result that passed `validateV2Result()` and whose STM
 *   setup is exactly the one required for its profile (`validateStmSetup()`;
 *   `expectedProfile`, when given, is the profile this page requested and the
 *   reply must echo it);
 * - otherwise `{ kind: 'failure', title, message, backendMessage, cacheable }`,
 *   which carries no route, order, setup or commands.
 * Only HTTP 200 is ever evaluated as a `PLAN_RESULT`. An ordinary HTTP 400
 * `PLAN_ERROR` is a rejected request; every other non-200 status is a
 * non-cacheable failure even when its body looks like a valid result.
 * `cacheable` marks a definitive planner answer for this exact request (safe
 * to reuse instead of re-submitting the same map); transport problems,
 * malformed data and internal errors are not cached.
 */
export function evaluatePlanResponse(reply, missionId, expectedProfile = null) {
  const { httpStatus, body, jsonOk } = reply;
  if (httpStatus === 409) return { kind: 'busy' };
  const failure = (title, message, extra = {}) => ({
    kind: 'failure', title, message, backendMessage: null, cacheable: false, ...extra,
  });
  if (httpStatus !== 200) {
    const rejected = httpStatus === 400 && jsonOk && body?.type === 'PLAN_ERROR';
    if (!rejected) {
      const detail = jsonOk && body?.type === 'PLAN_ERROR' && typeof body.error === 'string' ? `: ${body.error}` : '';
      return failure('Planning request failed', `Unexpected HTTP status ${httpStatus} from the planner${detail}.`);
    }
    return failure('Planning request failed', `The planner rejected the request: ${body.error || 'no reason given'}`, {
      cacheable: true,
    });
  }
  if (!jsonOk) return failure('Planning request failed', 'The planner reply was not valid JSON.');
  if (body?.type === 'PLAN_ERROR') {
    return failure('Planning request failed', `The planner rejected the request: ${body.error || 'no reason given'}`);
  }
  let result;
  let setup = null;
  try {
    ({ result } = validateV2Result(body, missionId));
    if (isExecutableV2Result(result)) setup = validateStmSetup(result, expectedProfile);
  } catch (error) {
    return failure('Planning request failed', `Malformed plan data: ${error.message}`);
  }
  const backendMessage = typeof result.message === 'string' ? result.message : null;
  if (setup) {
    return {
      kind: 'result', result, label: optimalityLabelV2(result), backendMessage, cacheable: true, setupRows: stmSetupRows(setup),
    };
  }
  return failure('No executable route', noRouteExplanationV2(result), { backendMessage, cacheable: true });
}

/** The two STM setup steps, in the order the RPi sends them before the first
 * `MOV`: set the profile, then verify it. Takes an already validated setup. */
export function stmSetupRows(setup) {
  return [
    { command: setup.set_command, expect: setup.expected_set_reply },
    { command: setup.verify_command, expect: setup.expected_verify_reply },
  ];
}

/** Outcome of a finished `/state/v2` snapshot (`complete` or `error`).
 * `expectedProfile` is the profile this page requested, for its own missions;
 * an external mission is only checked for self-consistency. */
export function evaluateSnapshot(snapshot, expectedProfile = null) {
  if (snapshot.status === 'error') {
    return {
      kind: 'failure',
      title: 'Planning request failed',
      message: `The planner reported an error: ${snapshot.error || 'unknown error'}`,
      backendMessage: null,
      cacheable: false,
    };
  }
  return evaluatePlanResponse({ httpStatus: 200, body: snapshot.result, jsonOk: true }, snapshot.mission_id, expectedProfile);
}

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

export class AutoPlanController {
  /**
   * @param {object} deps
   * @param {boolean} [deps.observeOnly]  never send a planning request
   * @param {{postPlan(payload, signal): Promise<{httpStatus, body, jsonOk}>,
   *          getState(): Promise<object|null>}} deps.transport
   * @param {{now(): number, setTimeout(fn, ms): any, clearTimeout(id): void}} deps.clock
   * @param {() => void} [deps.onChange]  called after every visible change
   * @param {(prefix: string) => string} [deps.generateId]
   */
  constructor({ observeOnly = false, transport, clock, onChange = () => {}, generateId = generateMissionId }) {
    this.observeOnly = observeOnly;
    this.transport = transport;
    this.clock = clock;
    this.onChange = onChange;
    this.generateId = generateId;

    this.localMap = emptyMap();
    this.revision = 0;
    this.selectedId = null;
    this.newFace = 'N';
    this.startProblem = null;
    this.startEpoch = 0; // bumps when the start inputs must be rewritten from the accepted start

    this.pendingEdit = false; // the accepted local map still needs a plan
    this.queued = false; // the countdown expired but the planner was busy
    this.countdownDeadline = null;
    this.countdownTimer = null;
    this.graceUntil = 0;
    this.graceTimer = null;
    this.submitting = false;
    this.inFlight = null; // { missionId, revision, fingerprint, sentAt, controller, timer, timedOut }
    this.latestSubmittedMissionId = null;
    this.ownMissionIds = new Set();
    this.outcomes = new Map(); // fingerprint -> cacheable outcome (LRU)

    this.shown = null; // outcome on screen: { kind: 'result'|'failure', ..., source, missionId }
    this.readOnlyView = null; // { displayMap, missionId, reason } shown instead of the local map
    this.externalLabel = null; // mission ID of an adopted external mission, until the next local edit
    this.external = null; // { missionId, firstSeenAt, sawPlanning, adoptedRevision, fingerprint }
    this.lastExternalKey = null;
    this.lastSnapshot = null;

    this.pollTimer = null;
    this.polling = false;
    this.disposed = false;
  }

  start() {
    this.poll();
  }

  dispose() {
    this.disposed = true;
    for (const timer of [this.pollTimer, this.countdownTimer, this.graceTimer, this.inFlight?.timer]) {
      if (timer != null) this.clock.clearTimeout(timer);
    }
  }

  // -- editing --------------------------------------------------------------

  canEdit() {
    return !this.observeOnly && !this.readOnlyView;
  }

  /** Clearing is also how a read-only external map is left (it is never
   * edited in place), except while a queued local map waits for the planner. */
  canClear() {
    if (this.observeOnly) return false;
    if (this.readOnlyView) return !(this.queued && this.pendingEdit);
    return this.localMap.obstacles.length > 0;
  }

  /** Editing activity that is not itself a map edit (selecting, choosing the
   * facing for the next obstacle, pressing on the map): while an unplanned
   * edit is pending (countdown running, or waiting for the planner) the
   * five-second inactivity period starts over. It never touches the map
   * revision, never submits anything, and never starts a countdown when
   * nothing needs planning (a completed identical map is not replanned). */
  noteActivity() {
    if (!this.canEdit() || !this.pendingEdit || this.startProblem) return;
    this.queued = false;
    this.restartCountdown();
    this.emit();
  }

  /** Click on grid cell (x, y): selects an existing obstacle (or deselects
   * it); otherwise adds a new one with the facing chosen for the next
   * placement. A new obstacle is never left selected, so the next facing
   * choice cannot alter it. */
  clickCell(x, y) {
    if (!this.canEdit()) return;
    const existing = obstacleAt(this.localMap, x, y);
    if (existing) {
      this.selectedId = this.selectedId === existing.id ? null : existing.id;
      this.noteActivity();
      this.emit();
      return;
    }
    const added = addObstacle(this.localMap, x, y, this.newFace);
    if (!added) {
      this.noteActivity(); // e.g. a click on a full map: still editing activity
      return;
    }
    this.applyEdit(added.map);
  }

  selectObstacle(id) {
    if (!this.canEdit()) return;
    const next = id !== null && this.localMap.obstacles.some((o) => o.id === id) ? id : null;
    const changed = next !== this.selectedId;
    this.selectedId = next;
    if (changed) this.noteActivity();
    this.emit();
  }

  /** Facing buttons. With an obstacle selected they correct that obstacle
   * (an edit, or mere activity when the facing is unchanged) and leave the
   * facing for the next placement alone. With nothing selected they choose
   * the facing for the next placement (activity, not an edit). */
  chooseFace(face) {
    if (!this.canEdit() || !FACES.includes(face)) return;
    if (this.selectedId !== null) {
      const edited = setObstacleFace(this.localMap, this.selectedId, face);
      if (edited) this.applyEdit(edited);
      else this.noteActivity();
    } else {
      this.newFace = face;
      this.noteActivity();
    }
    this.emit();
  }

  deleteObstacle(id) {
    if (!this.canEdit()) return;
    if (this.selectedId === id) this.selectedId = null;
    this.applyEdit(removeObstacle(this.localMap, id));
  }

  deleteSelected() {
    if (this.selectedId !== null) this.deleteObstacle(this.selectedId);
  }

  clearMap() {
    if (!this.canClear()) return;
    this.selectedId = null;
    if (this.readOnlyView) {
      this.readOnlyView = null;
      this.startProblem = null;
      this.startEpoch += 1;
      this.applyEdit({ ...this.localMap, obstacles: [] }, { force: true });
      return;
    }
    this.applyEdit(clearObstacles(this.localMap));
  }

  /** Start inputs apply as soon as they are valid; returns the problem text
   * (also kept in `view().startProblem`) or null.
   *
   * While any start field is empty, partial or invalid the accepted map is
   * left alone but planning is blocked: the countdown is cancelled, the
   * displayed route and commands are hidden, an in-flight result cannot be
   * displayed, and nothing is submitted (`trySubmit()` refuses). When every
   * field is valid again the start is applied (a new countdown when a new
   * plan is needed) or, for the exact map still in flight, reattached to it. */
  setStartInput(xText, yText, heading) {
    if (!this.canEdit()) return null;
    const hadProblem = this.startProblem !== null;
    const parsed = parseStartInput(xText, yText, heading);
    this.startProblem = parsed.problem || null;
    if (parsed.problem) {
      this.stopCountdown();
      this.queued = false;
    } else {
      const changed = setStart(this.localMap, parsed.start);
      if (changed) this.applyEdit(changed);
      else if (hadProblem) this.resumeAfterStartProblem();
    }
    this.emit();
    return this.startProblem;
  }

  /** The start is valid again and equals the accepted one. */
  resumeAfterStartProblem() {
    if (this.localMap.obstacles.length === 0) return;
    const fingerprint = mapFingerprint(this.localMap);
    if (this.inFlight && this.inFlight.fingerprint === fingerprint) {
      // Exactly the map being planned: reattach to that request, no duplicate.
      this.inFlight.revision = this.revision;
      this.stopCountdown();
      this.pendingEdit = false;
      this.queued = false;
    } else if (this.pendingEdit) {
      this.queued = false;
      this.restartCountdown();
    }
  }

  /** One accepted edit: new revision, route cleared at once, then either a
   * reuse (identical to the request in flight or an earlier outcome) or a
   * restarted countdown. */
  applyEdit(newMap, { force = false } = {}) {
    if (!newMap || (!force && !this.canEdit())) return;
    this.localMap = newMap;
    this.revision += 1;
    this.externalLabel = null;
    this.external = this.external && { ...this.external, adoptedRevision: null };
    this.shown = null;
    if (this.selectedId !== null && !newMap.obstacles.some((o) => o.id === this.selectedId)) this.selectedId = null;

    const fingerprint = mapFingerprint(newMap);
    if (newMap.obstacles.length === 0) {
      this.stopCountdown();
      this.pendingEdit = false;
      this.queued = false;
    } else if (this.inFlight && this.inFlight.fingerprint === fingerprint) {
      // Back to exactly the map being planned: its response is current again.
      this.inFlight.revision = this.revision;
      this.stopCountdown();
      this.pendingEdit = false;
      this.queued = false;
    } else if (this.outcomes.has(fingerprint)) {
      this.restoreOutcome(fingerprint);
    } else {
      this.pendingEdit = true;
      this.queued = false;
      // A start-input problem keeps planning paused; the countdown starts
      // when the input becomes valid (`resumeAfterStartProblem`).
      if (this.startProblem) this.stopCountdown();
      else this.restartCountdown();
    }
    this.emit();
  }

  restoreOutcome(fingerprint) {
    const outcome = this.outcomes.get(fingerprint);
    this.outcomes.delete(fingerprint);
    this.outcomes.set(fingerprint, outcome);
    this.shown = { ...outcome, fromCache: true };
    this.stopCountdown();
    this.pendingEdit = false;
    this.queued = false;
  }

  cacheOutcome(fingerprint, outcome) {
    if (!outcome.cacheable) return;
    this.outcomes.delete(fingerprint);
    this.outcomes.set(fingerprint, outcome);
    while (this.outcomes.size > OUTCOME_CACHE_LIMIT) this.outcomes.delete(this.outcomes.keys().next().value);
  }

  // -- countdown and submission ------------------------------------------------

  restartCountdown() {
    this.stopCountdown();
    this.countdownDeadline = this.clock.now() + AUTO_PLAN_IDLE_MS;
    this.countdownTimer = this.clock.setTimeout(() => {
      this.countdownTimer = null;
      this.countdownDeadline = null;
      this.trySubmit();
    }, AUTO_PLAN_IDLE_MS);
  }

  stopCountdown() {
    if (this.countdownTimer !== null) this.clock.clearTimeout(this.countdownTimer);
    this.countdownTimer = null;
    this.countdownDeadline = null;
  }

  latest() {
    return { missionId: this.latestSubmittedMissionId, revision: this.revision, fingerprint: mapFingerprint(this.localMap) };
  }

  async trySubmit() {
    if (this.disposed || this.observeOnly || !this.pendingEdit || this.submitting || this.countdownDeadline !== null) return;
    // Nothing is submitted while a start field is empty, partial or invalid.
    if (this.startProblem) return;
    if (this.localMap.obstacles.length === 0) {
      this.pendingEdit = false;
      this.emit();
      return;
    }
    if (this.inFlight) {
      // One request at a time: the newest map follows once this one returns.
      this.queued = true;
      this.emit();
      return;
    }
    const fingerprint = mapFingerprint(this.localMap);
    if (this.outcomes.has(fingerprint)) {
      this.restoreOutcome(fingerprint);
      this.emit();
      return;
    }
    const now = this.clock.now();
    if (now < this.graceUntil) {
      this.queued = true;
      if (this.graceTimer === null) {
        this.graceTimer = this.clock.setTimeout(() => {
          this.graceTimer = null;
          this.trySubmit();
        }, this.graceUntil - now);
      }
      this.emit();
      return;
    }

    // Never post into a busy planner: that would only earn a 409.
    this.submitting = true;
    this.emit();
    let snapshot = null;
    try {
      snapshot = await this.transport.getState();
    } catch {
      snapshot = null;
    }
    this.submitting = false;
    if (this.disposed) return;
    if (snapshot?.status === 'planning') {
      this.queued = true;
      this.handleSnapshot(snapshot);
      this.schedulePoll(POLL_PLANNING_MS);
      this.emit();
      return;
    }
    if (snapshot) this.handleSnapshot(snapshot);
    // An edit during the check restarted the countdown; it will submit. A
    // start problem that appeared during the check blocks submission.
    if (!this.pendingEdit || this.countdownDeadline !== null || this.inFlight || this.startProblem) return;
    this.send();
  }

  async send() {
    if (this.startProblem) return;
    const missionId = this.generateId('web-auto');
    this.ownMissionIds.add(missionId);
    const payload = buildAutoRequest(this.localMap, missionId);
    const controller = new AbortController();
    const request = {
      missionId,
      revision: this.revision,
      fingerprint: mapFingerprint(this.localMap),
      sentAt: this.clock.now(),
      controller,
      timer: null,
      timedOut: false,
    };
    request.timer = this.clock.setTimeout(() => {
      request.timedOut = true;
      controller.abort();
    }, BROWSER_TIMEOUT_MS);
    this.inFlight = request;
    this.latestSubmittedMissionId = missionId;
    this.pendingEdit = false;
    this.queued = false;
    this.shown = null;
    this.externalLabel = null;
    this.readOnlyView = null; // a queued local map leaves the busy external view
    this.emit();
    this.schedulePoll(POLL_PLANNING_MS);

    let reply;
    try {
      reply = await this.transport.postPlan(payload, controller.signal);
    } catch (error) {
      reply = { transportError: error };
    }
    await this.settle(request, reply);
  }

  async settle(request, reply) {
    this.clock.clearTimeout(request.timer);
    if (this.inFlight === request) this.inFlight = null;
    if (this.disposed) return;
    this.graceUntil = this.clock.now() + FOLLOW_UP_GRACE_MS;

    const outcome = reply.transportError
      ? await this.recoverFromState(request, reply.transportError)
      : evaluatePlanResponse(reply, request.missionId, AUTO_MOTION_PROFILE);
    if (this.disposed) return;

    if (outcome.kind === 'busy') {
      // The request never ran. Keep exactly one newest local map for later
      // and observe whoever holds the planner.
      if (this.localMap.obstacles.length > 0) {
        this.pendingEdit = true;
        this.queued = true;
      }
      this.emit();
      this.schedulePoll(0);
      return;
    }

    this.cacheOutcome(request.fingerprint, outcome);
    if (responseMatchesLatest(request, this.latest())) {
      this.shown = { ...outcome, source: 'local', missionId: request.missionId };
    }
    this.emit();
    this.trySubmit();
  }

  /** Abort or network failure: the server may still have finished, so
   * consult `/state/v2` for the same mission before reporting a failure. */
  async recoverFromState(request, error) {
    let snapshot = null;
    try {
      snapshot = await this.transport.getState();
    } catch {
      snapshot = null;
    }
    if (snapshot?.mission_id === request.missionId && (snapshot.status === 'complete' || snapshot.status === 'error')) {
      return evaluateSnapshot(snapshot, AUTO_MOTION_PROFILE);
    }
    const reason = request.timedOut
      ? `No reply from the planner within ${BROWSER_TIMEOUT_MS / 1000} s`
      : `Could not reach the planner (${error?.message || 'network error'})`;
    return {
      kind: 'failure',
      title: 'Planning request failed',
      message: `${reason}, and /state/v2 holds no finished result for this mission.`,
      backendMessage: null,
      cacheable: false,
    };
  }

  // -- /state/v2 ----------------------------------------------------------------

  schedulePoll(delayMs) {
    if (this.disposed) return;
    if (this.pollTimer !== null) this.clock.clearTimeout(this.pollTimer);
    this.pollTimer = this.clock.setTimeout(() => {
      this.pollTimer = null;
      this.poll();
    }, delayMs);
  }

  plannerBusy() {
    return Boolean(this.inFlight || this.submitting || this.lastSnapshot?.status === 'planning');
  }

  async poll() {
    if (this.disposed) return;
    if (!this.polling) {
      this.polling = true;
      let snapshot = null;
      try {
        snapshot = await this.transport.getState();
      } catch {
        snapshot = null; // transient: the next poll retries
      }
      this.polling = false;
      if (this.disposed) return;
      if (snapshot) this.handleSnapshot(snapshot);
    }
    if (this.pollTimer === null) this.schedulePoll(this.plannerBusy() ? POLL_PLANNING_MS : POLL_IDLE_MS);
  }

  handleSnapshot(snapshot) {
    if (!snapshot || typeof snapshot !== 'object') return;
    this.lastSnapshot = snapshot;
    const id = snapshot.mission_id;
    // This page's own missions are settled by their own reply, never here.
    if (typeof id === 'string' && id && !this.ownMissionIds.has(id)) this.observeExternal(snapshot);
    this.emit();
    if (snapshot.status !== 'planning' && this.queued && this.pendingEdit) this.trySubmit();
  }

  observeExternal(snapshot) {
    const key = `${snapshot.mission_id}|${snapshot.status}`;
    if (key === this.lastExternalKey) return;
    this.lastExternalKey = key;

    if (this.external?.missionId !== snapshot.mission_id) {
      this.external = {
        missionId: snapshot.mission_id,
        firstSeenAt: this.clock.now(),
        sawPlanning: snapshot.status === 'planning',
        adoptedRevision: null,
        fingerprint: null,
      };
      const described = describeRequestMap(snapshot.request);
      const keepQueued = this.queued && this.pendingEdit;
      // The page now shows another mission's start: drop any typed-but-invalid
      // start text so it can neither block planning nor linger in the form.
      this.startProblem = null;
      this.startEpoch += 1;
      if (!keepQueued) {
        this.stopCountdown();
        this.pendingEdit = false;
        this.queued = false;
      }
      if (!keepQueued && !this.observeOnly && described?.editableMap) {
        this.localMap = described.editableMap;
        this.revision += 1;
        this.readOnlyView = null;
        if (matchesAutoDefaults(snapshot.request)) this.external.fingerprint = mapFingerprint(this.localMap);
      } else {
        let reason = 'malformed';
        if (described) reason = keepQueued ? 'busy' : this.observeOnly ? 'observe' : 'off-grid';
        this.readOnlyView = { displayMap: described?.displayMap ?? null, missionId: snapshot.mission_id, reason };
      }
      this.external.adoptedRevision = this.revision;
      this.selectedId = null;
      this.externalLabel = snapshot.mission_id;
      this.shown = null;
    }

    if (snapshot.status === 'planning') {
      if (!this.external.sawPlanning) {
        this.external.sawPlanning = true;
        this.external.firstSeenAt = this.clock.now();
      }
      return;
    }
    if (snapshot.status === 'complete' || snapshot.status === 'error') {
      const outcome = evaluateSnapshot(snapshot);
      if (this.external.fingerprint) this.cacheOutcome(this.external.fingerprint, outcome);
      // Shown unless the user has edited the adopted map since.
      if (this.revision === this.external.adoptedRevision || this.readOnlyView?.missionId === snapshot.mission_id) {
        this.shown = { ...outcome, source: 'external', missionId: snapshot.mission_id };
      }
    }
  }

  // -- view -------------------------------------------------------------------

  emit() {
    if (!this.disposed) this.onChange();
  }

  /** True while the status line shows a running clock. */
  needsTick() {
    return Boolean(this.countdownDeadline !== null || this.inFlight || this.lastSnapshot?.status === 'planning');
  }

  view() {
    const now = this.clock.now();
    const displayMap = this.readOnlyView ? this.readOnlyView.displayMap : toDisplayMap(this.localMap);
    const selected = this.canEdit() ? this.localMap.obstacles.find((o) => o.id === this.selectedId) || null : null;
    return {
      observeOnly: this.observeOnly,
      editable: this.canEdit(),
      canClear: this.canClear(),
      map: displayMap,
      readOnlyReason: this.readOnlyView?.reason ?? null,
      localStart: { ...this.localMap.start },
      startEpoch: this.startEpoch,
      startProblem: this.startProblem,
      selectedId: selected ? selected.id : null,
      activeFace: selected ? selected.face : this.newFace,
      obstacleCount: displayMap ? displayMap.obstacles.length : 0,
      localObstacles: this.localMap.obstacles.map((o) => ({ ...o })),
      maxObstacles: MAX_OBSTACLES_V2,
      externalMissionId: this.externalLabel,
      // The mission on screen (from /state/v2): the camera gallery shows only its photos.
      missionId: typeof this.lastSnapshot?.mission_id === 'string' ? this.lastSnapshot.mission_id : null,
      status: this.statusView(now),
      // No route or commands are displayed while the start input is invalid;
      // the accepted map's outcome is kept and returns once it is valid again.
      shown: this.startProblem ? null : this.shown,
    };
  }

  statusView(now) {
    const countdownSeconds = () => Math.max(1, Math.ceil((this.countdownDeadline - now) / 1000));
    const externalPlanning = this.lastSnapshot?.status === 'planning'
      && typeof this.lastSnapshot.mission_id === 'string'
      && !this.ownMissionIds.has(this.lastSnapshot.mission_id);
    const externalSeconds = () => Math.max(0, Math.floor((now - (this.external?.firstSeenAt ?? now)) / 1000));
    const externalText = () =>
      `External mission ${this.lastSnapshot.mission_id} planning… ≈${externalSeconds()} s (approximate, observed by this page)`;

    if (this.startProblem && this.canEdit()) {
      return {
        kind: 'invalid-start',
        badge: 'Check start',
        text: 'Fix the start position to continue.',
        detail: this.inFlight
          ? 'A plan already running will not be shown while the start is invalid.'
          : 'Automatic planning is paused until the start position is valid.',
      };
    }
    if (this.inFlight) {
      if (this.inFlight.revision !== this.revision) {
        let detail = 'Place 1–8 obstacles to plan a route.';
        if (this.countdownDeadline !== null) detail = `Then replanning the newest map. ${COUNTDOWN_TEXT(countdownSeconds())}`;
        else if (this.pendingEdit) detail = 'The newest map is queued and will be planned when the planner is free.';
        return { kind: 'stale', badge: 'Map changed', text: 'Map changed — the current plan will be discarded.', detail };
      }
      const seconds = Math.max(0, Math.floor((now - this.inFlight.sentAt) / 1000));
      return { kind: 'planning', badge: 'Planning', text: PLANNING_TEXT(seconds), detail: WINDOW_NOTE };
    }
    if (this.submitting) {
      return { kind: 'planning', badge: 'Planning', text: 'Checking that the planner is free…', detail: WINDOW_NOTE };
    }
    if (this.countdownDeadline !== null) {
      return { kind: 'countdown', badge: 'Editing', text: COUNTDOWN_TEXT(countdownSeconds()), detail: WINDOW_NOTE };
    }
    if (this.queued && this.pendingEdit) {
      return {
        kind: 'busy',
        badge: 'Waiting',
        text: externalPlanning ? externalText() : 'Waiting for the planner to become free…',
        detail: 'Your newest map is queued and will be planned when the planner is free.',
      };
    }
    if (externalPlanning) {
      return { kind: 'external-planning', badge: 'Planning', text: externalText(), detail: '' };
    }
    if (this.shown?.kind === 'result') {
      return { kind: 'result', badge: 'Route ready', text: this.shown.label, detail: '' };
    }
    if (this.shown?.kind === 'failure') {
      const badge = this.shown.title === 'No executable route' ? 'No route' : 'Failed';
      return { kind: 'failure', badge, text: this.shown.title, detail: this.shown.message };
    }
    if (this.observeOnly) {
      return { kind: 'idle', badge: 'Waiting', text: 'Waiting for the RPi to request a plan.', detail: '' };
    }
    if (!this.readOnlyView && this.localMap.obstacles.length === 0) {
      return { kind: 'empty', badge: 'Ready', text: 'Place 1–8 obstacles to plan a route.', detail: '' };
    }
    return { kind: 'idle', badge: 'Ready', text: 'Edit the map to plan a route.', detail: '' };
  }
}
