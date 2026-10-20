import {
  headingLetterToDeg,
  generateMissionId,
  buildPlanV2Request,
  classifyActionsAtPreviewStep,
  mergeV2State,
  normalizeV1Result,
  validateV2Result,
  PreviewModelError,
  MAX_OBSTACLES_V2,
  isExecutableV2Result,
} from './planv2.js';

import {
  GRID,
  CELL_CM,
  svgPoint,
  drawArenaGrid,
  drawArenaBorder,
  drawObstaclesLayer,
  drawFullAndCompletedPath,
  drawRobot,
  drawTargetRing,
  drawSnapMarker,
  cellFromClientPoint,
  createSvgAdder,
} from './arena.js';

const MAX_OBSTACLES_V1 = 5;

const state = {
  engine: 'v1', // 'v1' | 'v2' -- never mixed within one request/response cycle
  obstacles: [],
  nextId: 1,
  face: 'N',
  mode: 'B3',
  // Rear reference (rear-axle midpoint) in grid cells: (2, 1) = (20, 10) cm.
  start: { x: 2, y: 1, heading: 'N' },
  submitting: false,

  v1: {
    result: null, // normalized: waypoints/target_poses use heading_deg
    pathIndex: 0,
    playing: false,
    timer: null,
    speed: 1,
  },

  v2: {
    result: null, // normalized: waypoints/target_poses/final_pose use heading_deg
    previewSteps: [],
    previewStepIndex: 0,
    playing: false,
    timer: null,
    speed: 1,
    allowAlternativeViewing: false,
    missionId: null,
    lastSubmittedMissionId: null,
    // Identity of the one request whose response may still mutate state.
    // Null when nothing is in flight or the map invalidated the request.
    pendingMissionId: null,
    planningStatus: 'idle',
    // `{ text, failure }` shown when there is no result: a failed attempt
    // (failure: true) or a discarded stale response (failure: false).
    notice: null,
  },
};

const $ = (id) => document.getElementById(id);

function format(n, digits = 1) {
  return Number.isFinite(n) ? n.toFixed(digits) : '—';
}

function startPoseFallback() {
  return {
    x_cm: state.start.x * CELL_CM,
    y_cm: state.start.y * CELL_CM,
    heading_deg: headingLetterToDeg(state.start.heading),
  };
}

// ---------------------------------------------------------------------------
// Current pose (per engine)
// ---------------------------------------------------------------------------

function currentWaypointIndexV2() {
  const step = state.v2.previewSteps[state.v2.previewStepIndex];
  return step ? step.waypointIndex : 0;
}

function currentPose() {
  if (state.engine === 'v1') {
    const waypoints = state.v1.result?.waypoints;
    if (!waypoints?.length) return startPoseFallback();
    return waypoints[state.v1.pathIndex];
  }
  const waypoints = state.v2.result?.waypoints;
  if (!waypoints?.length) return startPoseFallback();
  return waypoints[currentWaypointIndexV2()];
}

// ---------------------------------------------------------------------------
// Drawing primitives (pure DOM, take `add` + explicit data -- no globals)
// ---------------------------------------------------------------------------

/** Amber overlay for the MOV action currently active in the preview --
 * always its full inclusive waypoint range, regardless of the exact
 * sub-step the scrubber is on within it. */
function drawAmberOverlay(add, waypoints, startIndex, endIndex) {
  const segment = waypoints.slice(startIndex, endIndex + 1);
  if (segment.length < 2) return;
  const d = segment.map((p, i) => `${i ? 'L' : 'M'} ${svgPoint(p.x_cm, p.y_cm).join(' ')}`).join(' ');
  add('path', {
    d,
    fill: 'none',
    stroke: '#f59e0b',
    'stroke-width': 10,
    'stroke-linecap': 'round',
    'stroke-linejoin': 'round',
  });
}

// ---------------------------------------------------------------------------
// Per-engine route/target rendering
// ---------------------------------------------------------------------------

function completedCountV1() {
  const result = state.v1.result;
  if (!result) return 0;
  const targets = result.target_poses || [];
  if (!targets.length) return 0;

  let count = 0;
  for (const target of targets) {
    const idx = (result.waypoints || []).findIndex(
      (p) =>
        Math.abs(p.x_cm - target.x_cm) < 1e-6 &&
        Math.abs(p.y_cm - target.y_cm) < 1e-6 &&
        Math.abs(p.heading_deg - target.heading_deg) < 1e-4,
    );
    if (idx >= 0 && state.v1.pathIndex >= idx) count++;
  }
  return count;
}

function v2Classification() {
  const result = state.v2.result;
  if (!result) return null;
  return classifyActionsAtPreviewStep(result.actions, state.v2.previewSteps, state.v2.previewStepIndex);
}

function drawRouteV1(add) {
  const result = state.v1.result;
  if (!result?.waypoints?.length) return;
  drawFullAndCompletedPath(add, result.waypoints, state.v1.pathIndex);

  const done = completedCountV1();
  (result.target_poses || []).forEach((p, index) => {
    drawTargetRing(add, p, index, result.obstacle_order[index], index < done);
  });
}

function drawRouteV2(add) {
  const result = state.v2.result;
  if (!result?.waypoints?.length) return;

  drawFullAndCompletedPath(add, result.waypoints, currentWaypointIndexV2());

  const classification = v2Classification();
  const activeMov = classification?.find((c) => c.status === 'active' && c.action.type === 'MOV');
  if (activeMov) {
    drawAmberOverlay(add, result.waypoints, activeMov.action.waypoint_start_index, activeMov.action.waypoint_end_index);
  }

  (result.target_poses || []).forEach((p, index) => {
    const obstacleId = result.obstacle_order[index];
    const snapEntry = classification?.find((c) => c.action.type === 'SNAP' && c.action.obstacle_id === obstacleId);
    drawTargetRing(add, p, index, obstacleId, snapEntry?.status === 'completed');
  });

  (classification || [])
    .filter((c) => c.action.type === 'SNAP')
    .forEach((c) => drawSnapMarker(add, c.action.pose, c.action.obstacle_id, c.status));
}

function visitedIdsV1() {
  const result = state.v1.result;
  if (!result?.obstacle_order) return new Set();
  return new Set(result.obstacle_order.slice(0, completedCountV1()));
}

function visitedIdsV2() {
  const classification = v2Classification();
  if (!classification) return new Set();
  return new Set(
    classification.filter((c) => c.action.type === 'SNAP' && c.status === 'completed').map((c) => c.action.obstacle_id),
  );
}

// ---------------------------------------------------------------------------
// Top-level render
// ---------------------------------------------------------------------------

function render() {
  const svg = $('map');
  svg.replaceChildren();
  const add = createSvgAdder(svg);

  drawArenaGrid(add);

  if (state.engine === 'v1') {
    drawRouteV1(add);
    drawObstaclesLayer(add, state.obstacles, visitedIdsV1());
  } else {
    drawRouteV2(add);
    drawObstaclesLayer(add, state.obstacles, visitedIdsV2());
  }

  drawRobot(add, currentPose());
  drawArenaBorder(add);

  updatePoseDisplay();
}

function updatePoseDisplay() {
  const p = currentPose();
  $('pose').textContent = `x ${format(p.x_cm)} cm · y ${format(p.y_cm)} cm · heading ${format(p.heading_deg)}°`;

  if (state.engine === 'v1') {
    const total = state.v1.result?.waypoints?.length || 0;
    $('step').textContent = total ? `${state.v1.pathIndex + 1} / ${total}` : '0 / 0';
  } else {
    const total = state.v2.previewSteps.length;
    $('step').textContent = total ? `${state.v2.previewStepIndex + 1} / ${total}` : '0 / 0';
  }
}

// ---------------------------------------------------------------------------
// Sidebar panels
// ---------------------------------------------------------------------------

function renderObstacles() {
  const list = $('obstacles');
  list.replaceChildren();

  if (!state.obstacles.length) {
    const empty = document.createElement('div');
    empty.className = 'muted';
    empty.textContent = 'Click an empty grid cell to add an obstacle.';
    list.appendChild(empty);
    return;
  }

  state.obstacles.forEach((o) => {
    const row = document.createElement('div');
    row.className = 'obstacle-item';

    const id = document.createElement('div');
    id.className = 'obstacle-id';
    id.textContent = o.id;

    const text = document.createElement('div');
    text.textContent = `(${o.x / CELL_CM}, ${o.y / CELL_CM}) · face ${o.face}`;

    const del = document.createElement('button');
    del.className = 'danger';
    del.textContent = 'Delete';
    del.onclick = () => {
      state.obstacles = state.obstacles.filter((item) => item.id !== o.id);
      invalidateResults();
      renderObstacles();
      renderAll();
    };

    row.append(id, text, del);
    list.appendChild(row);
  });
}

function renderRoute() {
  const route = $('route');
  route.replaceChildren();

  const order = state.engine === 'v1' ? state.v1.result?.obstacle_order : state.v2.result?.obstacle_order;
  if (!order?.length) {
    route.innerHTML = '<span class="muted">No route planned.</span>';
    return;
  }

  const doneCount = state.engine === 'v1' ? completedCountV1() : visitedIdsV2().size;
  order.forEach((id, i) => {
    const s = document.createElement('div');
    s.className = 'route-step';
    if (i < doneCount) s.classList.add('done');
    if (i === doneCount && doneCount < order.length) s.classList.add('current');
    s.textContent = id;
    route.appendChild(s);
  });
}

function renderCommandsV1() {
  const box = $('commands');
  box.replaceChildren();
  const commands = state.v1.result?.commands;
  if (!commands?.length) {
    box.innerHTML = '<div class="muted" style="padding:9px">No commands.</div>';
    return;
  }
  commands.forEach((command) => {
    const row = document.createElement('div');
    row.className = 'command';
    if (command.type === 'SNAP') row.classList.add('snap');
    const left = document.createElement('span');
    left.textContent = command.type;
    const right = document.createElement('span');
    right.textContent = command.type === 'SNAP' ? `obstacle ${command.obstacle_id}` : `${format(command.value, 2)} cm`;
    row.append(left, right);
    box.appendChild(row);
  });
}

/** `waypoint_start_index`/`waypoint_end_index` (PROTOCOL.md §7) are
 * inclusive indices into the response-wide `waypoints` array. Returns a
 * `" · waypoints A-B"` suffix, or `''` when either index is missing or
 * not a valid inclusive range -- never renders `undefined`/`NaN`/a
 * guessed value. */
function formatWaypointRangeSuffix(action) {
  const start = action.waypoint_start_index;
  const end = action.waypoint_end_index;
  if (!Number.isInteger(start) || !Number.isInteger(end) || start < 0 || end < start) return '';
  return ` · waypoints ${start}-${end}`;
}

// One entry per rendered v2 action row, in action order, so preview movement
// can update classes without rebuilding the list.
let commandRowsV2 = [];

/** Sets active/done classes on the existing v2 rows from the current preview
 * step, then keeps the active (or, at the very end, the last completed) row
 * visible by scrolling only the command box -- never the page. */
function updateCommandStatesV2() {
  const classification = v2Classification();
  if (!classification || classification.length !== commandRowsV2.length) return;

  let target = null;
  classification.forEach(({ status }, index) => {
    const row = commandRowsV2[index];
    row.classList.toggle('active', status === 'active');
    row.classList.toggle('done', status === 'completed');
    if (status === 'active' || (status === 'completed' && !target?.active)) target = { row, active: status === 'active' };
  });
  if (!target) return;

  const box = $('commands');
  const boxRect = box.getBoundingClientRect();
  const rowRect = target.row.getBoundingClientRect();
  if (rowRect.top < boxRect.top) box.scrollTop -= boxRect.top - rowRect.top;
  else if (rowRect.bottom > boxRect.bottom) box.scrollTop += rowRect.bottom - boxRect.bottom;
}

/** Everything that must follow a change of the preview position. */
function refreshPreview() {
  renderRoute();
  updateStats();
  render();
  if (state.engine === 'v2') updateCommandStatesV2();
}

function renderCommandsV2() {
  const box = $('commands');
  box.replaceChildren();
  commandRowsV2 = [];
  const result = state.v2.result;
  if (!result?.actions?.length) {
    const empty = document.createElement('div');
    const failure = state.v2.notice?.failure;
    empty.className = failure ? 'error' : 'muted';
    empty.style.margin = '9px';
    empty.textContent = failure ? state.v2.notice.text : 'No actions.';
    box.appendChild(empty);
    return;
  }
  result.actions.forEach((action) => {
    const row = document.createElement('div');
    row.className = 'command';
    if (action.type === 'SNAP') row.classList.add('snap');

    const left = document.createElement('span');
    left.textContent = `#${action.seq} ${action.type}`;
    const right = document.createElement('span');
    const stmCommand = typeof action.stm_command === 'string' && action.stm_command ? action.stm_command : '—';
    right.textContent =
      action.type === 'SNAP' ? `obstacle ${action.obstacle_id}` : `${stmCommand}${formatWaypointRangeSuffix(action)}`;
    row.append(left, right);
    box.appendChild(row);
    commandRowsV2.push(row);
  });
  updateCommandStatesV2();
}

function renderCommands() {
  if (state.engine === 'v1') renderCommandsV1();
  else renderCommandsV2();
}

function updateStatsV1() {
  const result = state.v1.result;
  $('status').textContent = result ? (result.valid ? (result.complete ? 'Route ready' : 'Partial route ready') : 'Planning failed') : 'Ready';
  $('status').className = `badge ${result?.valid ? 'ok' : result ? 'error' : ''}`;
  $('mode').textContent = result?.mode || state.mode;
  $('time').textContent = result ? `${format(result.estimated_time_s, 2)} s` : '—';
  $('visited').textContent = result ? `${completedCountV1()} / ${result.obstacle_order.length}` : '0 / 0';
  $('distance').textContent = result
    ? `${format(
        result.waypoints.reduce((sum, p, i, arr) => (i ? sum + Math.hypot(p.x_cm - arr[i - 1].x_cm, p.y_cm - arr[i - 1].y_cm) : 0), 0),
        1,
      )} cm`
    : '—';
  const skipped = result?.skipped_obstacle_ids || [];
  $('resultMessage').textContent = result?.valid
    ? result.complete
      ? 'The returned route covers all requested obstacles.'
      : `Partial route. Skipped obstacles: ${skipped.join(', ') || 'none'}.`
    : result?.error || '';
  $('resultMessage').className = result && !result.valid ? 'error' : 'help';
}

function updateStatsV2() {
  const result = state.v2.result;
  $('missionIdReadout').textContent = state.v2.missionId || '—';
  $('planningStatusBadge').textContent = state.v2.planningStatus;
  $('planningStatusBadge').className = `badge ${state.v2.planningStatus === 'error' ? 'error' : state.v2.planningStatus === 'complete' ? 'ok' : ''}`;

  if (!result && state.v2.notice) {
    const { text, failure } = state.v2.notice;
    $('status').textContent = failure ? 'Planning request failed' : 'Ready';
    $('status').className = `badge ${failure ? 'error' : ''}`;
    $('resultMessage').textContent = text;
    $('resultMessage').className = failure ? 'error' : 'help';
    $('mode').textContent = state.mode;
    $('totalCost').textContent = '—';
    $('visited').textContent = '0 / 0';
    $('distance').textContent = '—';
    return;
  }

  $('status').textContent = result ? (result.valid ? (result.complete ? 'Route ready' : 'Partial route ready') : 'Planning failed') : 'Ready';
  $('status').className = `badge ${result?.valid ? 'ok' : result ? 'error' : ''}`;
  $('mode').textContent = result?.mode || state.mode;
  $('totalCost').textContent = result?.valid ? format(result.total_cost, 2) : '—';
  $('visited').textContent = result?.valid ? `${visitedIdsV2().size} / ${result.obstacle_order.length}` : '0 / 0';
  $('distance').textContent = result?.valid ? `${format(result.total_distance_cm, 1)} cm` : '—';

  const skipped = result?.skipped_obstacle_ids || [];
  $('resultMessage').textContent = result
    ? result.valid
      ? result.complete
        ? 'The returned mission covers all requested obstacles (plan preview only -- not executed).'
        : `Partial route. ${result.failure_reason || ''} Skipped obstacles: ${skipped.join(', ') || 'none'}.`
      : result.error || ''
    : state.submitting && state.v2.pendingMissionId
      ? 'Planning… the bounded planner replies within about 90 seconds.'
      : 'Configure obstacles and run the planner.';
  $('resultMessage').className = result && !result.valid ? 'error' : 'help';
}

function updateStats() {
  if (state.engine === 'v1') updateStatsV1();
  else updateStatsV2();
}

function updateTimeline() {
  const max = state.engine === 'v1' ? Math.max(0, (state.v1.result?.waypoints?.length || 1) - 1) : Math.max(0, state.v2.previewSteps.length - 1);
  const value = state.engine === 'v1' ? state.v1.pathIndex : state.v2.previewStepIndex;
  $('timeline').max = max;
  $('timeline').value = Math.min(value, max);
  const playing = state.engine === 'v1' ? state.v1.playing : state.v2.playing;
  $('play').textContent = playing ? 'Pause' : 'Play';
}

function updateEngineVisibility() {
  document.querySelectorAll('[data-engine-panel]').forEach((el) => {
    el.hidden = el.dataset.enginePanel !== state.engine;
  });
  document.querySelectorAll('[data-engine]').forEach((button) => {
    button.classList.toggle('selected', button.dataset.engine === state.engine);
  });
  $('visitedLabel').textContent = state.engine === 'v2' ? 'Visited (preview)' : 'Visited';
}

function updateControls() {
  const busy = state.submitting;
  const maxObstacles = state.engine === 'v2' ? MAX_OBSTACLES_V2 : MAX_OBSTACLES_V1;
  const countInvalid = state.obstacles.length === 0 || state.obstacles.length > maxObstacles;
  $('runB2').disabled = countInvalid || busy || currentEnginePlaying();
  $('runB3').disabled = countInvalid || busy || currentEnginePlaying();
  $('deleteAll').disabled = state.obstacles.length === 0;
  $('obstacleCount').textContent = `${state.obstacles.length} / ${maxObstacles}`;
  $('allowAlternativeViewing').checked = state.v2.allowAlternativeViewing;
  $('allowAlternativeViewing').disabled = busy;

  document.querySelectorAll('[data-face]').forEach((button) => {
    button.classList.toggle('selected', button.dataset.face === state.face);
  });

  updateEngineVisibility();

  const hasV2Actions = state.engine === 'v2' && isExecutableV2Result(state.v2.result);
  $('copyActions').disabled = !hasV2Actions;
  $('downloadActions').disabled = !hasV2Actions;
}

function currentEnginePlaying() {
  return state.engine === 'v1' ? state.v1.playing : state.v2.playing;
}

function renderAll() {
  renderRoute();
  renderCommands();
  updateTimeline();
  updateStats();
  render();
  updateControls();
}

function invalidateResults() {
  state.v1.result = null;
  state.v2.result = null;
  state.v2.previewSteps = [];
  state.v2.previewStepIndex = 0;
  state.v2.notice = null;
  // Any v2 request still in flight is now stale; its response is discarded.
  state.v2.pendingMissionId = null;
  resetAnimation();
}

// ---------------------------------------------------------------------------
// Playback
// ---------------------------------------------------------------------------

function stopPlayback(engineKey) {
  const engineState = state[engineKey];
  engineState.playing = false;
  if (engineState.timer) clearInterval(engineState.timer);
  engineState.timer = null;
}

function resetAnimation() {
  stopPlayback(state.engine);
  if (state.engine === 'v1') state.v1.pathIndex = 0;
  else state.v2.previewStepIndex = 0;
  renderAll();
}

function engineTotal(engineKey) {
  return engineKey === 'v1' ? state.v1.result?.waypoints?.length || 0 : state.v2.previewSteps.length;
}

function engineIndex(engineKey) {
  return engineKey === 'v1' ? state.v1.pathIndex : state.v2.previewStepIndex;
}

function setEngineIndex(engineKey, value) {
  if (engineKey === 'v1') state.v1.pathIndex = value;
  else state.v2.previewStepIndex = value;
}

function setIndex(value) {
  setEngineIndex(state.engine, value);
}

// The interval is bound to the engine it was created for (`engineKey`); it
// never reads `state.engine`, so it can only ever advance its own engine.
function play() {
  const engineKey = state.engine;
  const engineState = state[engineKey];
  const total = engineTotal(engineKey);
  if (!total) return;

  if (engineIndex(engineKey) >= total - 1) {
    setEngineIndex(engineKey, 0);
  }

  engineState.playing = true;
  updateTimeline();
  updateControls();

  const intervalMs = 35 / engineState.speed;
  engineState.timer = setInterval(() => {
    const limit = engineTotal(engineKey);
    const idx = engineIndex(engineKey);
    if (!limit || idx >= limit - 1) {
      stopPlayback(engineKey);
      if (state.engine === engineKey) {
        updateTimeline();
        updateControls();
      }
      return;
    }
    setEngineIndex(engineKey, idx + 1);
    if (state.engine === engineKey) {
      $('timeline').value = idx + 1;
      refreshPreview();
    }
  }, intervalMs);
}

function togglePlay() {
  const engineState = state.engine === 'v1' ? state.v1 : state.v2;
  if (engineState.playing) {
    engineState.playing = false;
    clearInterval(engineState.timer);
    engineState.timer = null;
    updateTimeline();
    updateControls();
  } else {
    play();
  }
}

// ---------------------------------------------------------------------------
// Map interaction
// ---------------------------------------------------------------------------

function addObstacleFromCell(cellX, cellY) {
  const maxObstacles = state.engine === 'v2' ? MAX_OBSTACLES_V2 : MAX_OBSTACLES_V1;
  if (state.obstacles.length >= maxObstacles) return;
  if (state.obstacles.some((o) => o.x / CELL_CM === cellX && o.y / CELL_CM === cellY)) return;

  state.obstacles.push({ id: state.nextId++, x: cellX * CELL_CM, y: cellY * CELL_CM, face: state.face });
  invalidateResults();
  renderObstacles();
}

function cellFromPointer(event) {
  return cellFromClientPoint($('map').getBoundingClientRect(), event.clientX, event.clientY);
}

// ---------------------------------------------------------------------------
// Planning: v1
// ---------------------------------------------------------------------------

async function runPlanV1(mode) {
  state.mode = mode;
  $('resultMessage').className = 'help';
  $('resultMessage').textContent = 'Planning…';
  updateControls();

  const payload = {
    type: 'PLAN_REQUEST',
    protocol_version: 1,
    mode,
    coordinate_units: 'grid',
    start: { x: state.start.x, y: state.start.y, heading: state.start.heading },
    obstacles: state.obstacles.map((o) => ({ id: o.id, x: o.x / CELL_CM, y: o.y / CELL_CM, face: o.face })),
  };

  try {
    const response = await fetch('/plan', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const result = await response.json();
    state.v1.result = normalizeV1Result(result);
    state.v1.pathIndex = 0;
    state.v1.playing = false;
    if (state.v1.timer) clearInterval(state.v1.timer);
    state.v1.timer = null;
    renderAll();
  } catch (error) {
    state.v1.result = { valid: false, error: `Could not connect to planner: ${error.message}` };
    updateStats();
    updateControls();
  }
}

// ---------------------------------------------------------------------------
// Planning: v2
// ---------------------------------------------------------------------------

function v2UiState(mode) {
  return {
    mode,
    coordinateUnits: 'grid',
    start: { x: state.start.x, y: state.start.y, heading: state.start.heading },
    obstacles: state.obstacles.map((o) => ({ id: o.id, x: o.x / CELL_CM, y: o.y / CELL_CM, face: o.face })),
    allowAlternativeViewing: state.v2.allowAlternativeViewing,
  };
}

/** Ends the current v2 attempt as a failure: nothing from a previous (or
 * partially normalised) result stays visible or scrubbable. */
function failV2Attempt(message) {
  stopPlayback('v2');
  state.v2.result = null;
  state.v2.previewSteps = [];
  state.v2.previewStepIndex = 0;
  state.v2.pendingMissionId = null;
  state.v2.planningStatus = 'error';
  state.v2.notice = { text: message, failure: true };
  state.submitting = false;
  renderAll();
}

async function runPlanV2(mode) {
  state.mode = mode;
  state.submitting = true;
  const missionId = generateMissionId();
  state.v2.lastSubmittedMissionId = missionId;
  state.v2.pendingMissionId = missionId;
  state.v2.missionId = missionId;
  state.v2.planningStatus = 'planning';
  state.v2.notice = null;
  $('resultMessage').className = 'help';
  $('resultMessage').textContent = 'Planning… the bounded planner replies within about 90 seconds.';
  updateControls();
  updateStatsV2();

  // Browser-side stale-result guard only; the backend computation is not
  // cancelled. Returns true when this response must be dropped: either the
  // map/start changed (pending id cleared) or a newer request owns state.
  const dropIfStale = () => {
    if (state.v2.pendingMissionId === missionId) return false;
    if (state.v2.pendingMissionId === null) {
      state.submitting = false;
      state.v2.planningStatus = 'idle';
      state.v2.notice = { text: 'Map changed while planning; result discarded.', failure: false };
      renderAll();
    }
    return true;
  };

  let payload;
  try {
    payload = buildPlanV2Request(v2UiState(mode), missionId);
  } catch (error) {
    failV2Attempt(`Planning request failed: ${error.message}`);
    return;
  }

  let response;
  try {
    response = await fetch('/plan/v2', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
  } catch (error) {
    if (!dropIfStale()) failV2Attempt(`Could not connect to planner: ${error.message}`);
    return;
  }

  let body;
  let jsonFailed = false;
  try {
    body = await response.json();
  } catch (error) {
    jsonFailed = true;
  }

  if (dropIfStale()) return;

  if (response.status === 409) {
    failV2Attempt('Planning request failed: planner is busy with another v2 request — try again shortly.');
  } else if (jsonFailed) {
    failV2Attempt('Planning request failed: response was not valid JSON.');
  } else if (body?.type === 'PLAN_ERROR') {
    failV2Attempt(`Planning request failed: ${body.error || 'planner rejected the request.'}`);
  } else {
    applyV2Result(body, missionId);
  }
}

function applyV2Result(rawResult, missionId) {
  let normalized;
  let previewSteps = [];
  try {
    ({ result: normalized, previewSteps } = validateV2Result(rawResult, missionId));
  } catch (error) {
    const detail = error instanceof PreviewModelError ? `Malformed plan data: ${error.message}` : `unexpected response (${error.message})`;
    failV2Attempt(`Planning request failed: ${detail}`);
    return;
  }

  stopPlayback('v2');
  state.v2.result = normalized;
  state.v2.previewSteps = previewSteps;
  state.v2.previewStepIndex = 0;
  state.v2.pendingMissionId = null;
  state.v2.notice = null;
  state.submitting = false;
  // Server-side `/state/v2` status is "complete" for any planning outcome
  // that isn't a transport/validation PLAN_ERROR -- including a legitimate
  // no-route result (valid:false is still a completed planning attempt,
  // not a server error). Mirror that distinction here rather than
  // conflating "the planner finished" with "a route was found".
  state.v2.planningStatus = 'complete';

  renderAll();
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

function runPlan(mode) {
  const maxObstacles = state.engine === 'v2' ? MAX_OBSTACLES_V2 : MAX_OBSTACLES_V1;
  if (!state.obstacles.length) return;
  if (state.obstacles.length > maxObstacles) {
    $('resultMessage').className = 'error';
    $('resultMessage').textContent = `This planner accepts at most ${maxObstacles} obstacles.`;
    return;
  }
  if (state.engine === 'v1') runPlanV1(mode);
  else runPlanV2(mode);
}

function setStart() {
  const x = Number($('startX').value);
  const y = Number($('startY').value);
  const heading = $('startHeading').value;

  if (!Number.isInteger(x) || !Number.isInteger(y) || x < 0 || x >= GRID || y < 0 || y >= GRID) {
    $('resultMessage').className = 'error';
    $('resultMessage').textContent = 'Start rear reference must be grid cells from 0 to 19.';
    return;
  }

  state.start = { x, y, heading };
  invalidateResults();
  $('resultMessage').textContent = 'Start pose updated.';
}

function clearAll() {
  state.obstacles = [];
  state.nextId = 1;
  invalidateResults();
  renderObstacles();
  $('resultMessage').textContent = 'Map cleared.';
}

// ---------------------------------------------------------------------------
// State polling (planning status only -- never execution/robot position)
// ---------------------------------------------------------------------------

let lastV1ResultKey = null;

async function pollServerStateV1() {
  try {
    const response = await fetch('/state', { cache: 'no-store' });
    if (!response.ok) return;
    const body = await response.json();
    if (!body.request || !body.result) return;

    const resultKey = JSON.stringify(body.result);
    if (resultKey === lastV1ResultKey) return;
    lastV1ResultKey = resultKey;

    state.mode = body.request.mode || state.mode;
    if (body.request.start) {
      state.start = { x: body.request.start.x, y: body.request.start.y, heading: body.request.start.heading };
    }
    if (Array.isArray(body.request.obstacles)) {
      state.obstacles = body.request.obstacles.map((o) => ({ id: o.id, x: o.x * CELL_CM, y: o.y * CELL_CM, face: o.face }));
      state.nextId = state.obstacles.reduce((max, o) => Math.max(max, o.id), 0) + 1;
    }
    state.v1.result = normalizeV1Result(body.result);
    state.v1.pathIndex = 0;
    state.v1.playing = false;
    if (state.v1.timer) clearInterval(state.v1.timer);
    state.v1.timer = null;

    renderObstacles();
    if (state.engine === 'v1') renderAll();
  } catch (error) {
    /* transient network error -- next poll will retry */
  }
}

/** `/state/v2` reports planning status only (idle/planning/complete/error)
 * plus the mission id it belongs to -- it is never used to move the
 * preview cursor or to imply execution/robot-position progress, since no
 * RPi execution exists. The actual v2 result this tab renders always comes
 * from the direct `fetch('/plan/v2')` response in `runPlanV2()`; this
 * poller only keeps the status badge honest (e.g. if a very slow request
 * is still running) and reflects another tab's busy mission for context. */
async function pollServerStateV2() {
  try {
    const response = await fetch('/state/v2', { cache: 'no-store' });
    if (!response.ok) return;
    const snapshot = await response.json();
    const merged = mergeV2State(
      { status: state.v2.planningStatus, missionId: state.v2.missionId },
      snapshot,
      state.v2.lastSubmittedMissionId,
    );
    if (merged.status !== state.v2.planningStatus || merged.missionId !== state.v2.missionId) {
      state.v2.planningStatus = merged.status;
      if (!state.submitting) state.v2.missionId = merged.missionId;
      if (state.engine === 'v2') updateStatsV2();
    }
  } catch (error) {
    /* transient network error -- next poll will retry */
  }
}

function pollActiveEngineState() {
  if (state.engine === 'v1') pollServerStateV1();
  else pollServerStateV2();
}

// ---------------------------------------------------------------------------
// Copy / download mission actions
// ---------------------------------------------------------------------------

function fallbackCopyToClipboard(text) {
  const textarea = document.createElement('textarea');
  textarea.value = text;
  textarea.style.position = 'fixed';
  textarea.style.opacity = '0';
  document.body.appendChild(textarea);
  textarea.focus();
  textarea.select();
  let ok = false;
  try {
    ok = document.execCommand('copy');
  } catch (error) {
    ok = false;
  }
  document.body.removeChild(textarea);
  return ok;
}

async function copyActionsJson() {
  if (!isExecutableV2Result(state.v2.result)) return;
  const actions = state.v2.result.actions;
  const text = JSON.stringify(actions, null, 2);
  if (navigator.clipboard?.writeText) {
    try {
      await navigator.clipboard.writeText(text);
      $('resultMessage').textContent = 'Mission actions copied to clipboard.';
      $('resultMessage').className = 'help';
      return;
    } catch (error) {
      /* fall through to the legacy fallback below */
    }
  }
  const ok = fallbackCopyToClipboard(text);
  $('resultMessage').textContent = ok ? 'Mission actions copied to clipboard.' : 'Copy unavailable in this browser — use Download instead.';
  $('resultMessage').className = ok ? 'help' : 'error';
}

function downloadActionsJson() {
  if (!isExecutableV2Result(state.v2.result)) return;
  const actions = state.v2.result.actions;
  const blob = new Blob([JSON.stringify(actions, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `${state.v2.missionId || 'mission'}-actions.json`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

$('map').addEventListener('click', (event) => {
  const cell = cellFromPointer(event);
  addObstacleFromCell(cell.x, cell.y);
});

document.querySelectorAll('[data-face]').forEach((button) => {
  button.addEventListener('click', () => {
    state.face = button.dataset.face;
    updateControls();
  });
});

document.querySelectorAll('[data-engine]').forEach((button) => {
  button.addEventListener('click', () => {
    if (state.engine === button.dataset.engine) return;
    stopPlayback(state.engine);
    state.engine = button.dataset.engine;
    renderAll();
  });
});

$('allowAlternativeViewing').addEventListener('change', () => {
  state.v2.allowAlternativeViewing = $('allowAlternativeViewing').checked;
});

$('runB2').onclick = () => runPlan('B2');
$('runB3').onclick = () => runPlan('B3');
$('deleteAll').onclick = clearAll;
$('setStart').onclick = setStart;
$('play').onclick = togglePlay;
$('resetAnimation').onclick = resetAnimation;
$('copyActions').onclick = copyActionsJson;
$('downloadActions').onclick = downloadActionsJson;

$('timeline').addEventListener('input', () => {
  const engineState = state.engine === 'v1' ? state.v1 : state.v2;
  setIndex(Number($('timeline').value));
  engineState.playing = false;
  if (engineState.timer) clearInterval(engineState.timer);
  engineState.timer = null;
  refreshPreview();
  updateTimeline();
  updateControls();
});

$('speed').addEventListener('change', () => {
  const engineState = state.engine === 'v1' ? state.v1 : state.v2;
  engineState.speed = Number($('speed').value);
  if (engineState.playing) {
    clearInterval(engineState.timer);
    engineState.timer = null;
    engineState.playing = false;
    play();
  }
});

$('next').onclick = () => {
  const total = state.engine === 'v1' ? state.v1.result?.waypoints?.length || 0 : state.v2.previewSteps.length;
  if (!total) return;
  const current = state.engine === 'v1' ? state.v1.pathIndex : state.v2.previewStepIndex;
  setIndex(Math.min(current + 1, total - 1));
  refreshPreview();
  updateTimeline();
};

$('previous').onclick = () => {
  const current = state.engine === 'v1' ? state.v1.pathIndex : state.v2.previewStepIndex;
  setIndex(Math.max(0, current - 1));
  refreshPreview();
  updateTimeline();
};

renderObstacles();
renderAll();
setInterval(pollActiveEngineState, 750);
pollActiveEngineState();
