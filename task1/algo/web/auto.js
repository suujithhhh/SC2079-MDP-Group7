/**
 * DOM wiring for the display-only planner page (`/`). All planning rules live
 * in `autoplan.js` (pure, tested in Node); this file only renders
 * `controller.view()`. Backend text is set with `textContent` only, never as
 * HTML.
 *
 * The page is a pure display: the controller always runs in observe-only mode
 * (it never sends a planning request) and this file registers NO input
 * handlers, so nothing on the page can edit the map or start a plan. It shows
 * what the planner is doing for the RPi, read from `/state/v2`. The RPi, not
 * this page, asks for plans.
 */

import { AutoPlanController } from './autoplan.js';
import { countText, galleryKey, imageUrl, normalizeImageList, selectGalleryImages, tileCaption } from './gallery.js';
import { VERSION_POLL_MS, normalizeVersions, versionIndicator } from './versions.js';
import {
  CELL_CM,
  ARENA_CM,
  svgPoint,
  drawArenaGrid,
  drawArenaBorder,
  drawObstaclesLayer,
  drawFullAndCompletedPath,
  drawRobot,
  drawTargetRing,
  drawSnapMarker,
  createSvgAdder,
} from './arena.js';

const $ = (id) => document.getElementById(id);
// Always display-only: never sends a planning request, never edits the map.
const observeOnly = true;

const transport = {
  async postPlan(payload, signal) {
    const response = await fetch('/plan/v2', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
      signal,
    });
    let body = null;
    let jsonOk = true;
    try {
      body = await response.json();
    } catch (error) {
      if (signal?.aborted) throw error; // a timeout, not a malformed reply
      jsonOk = false;
    }
    return { httpStatus: response.status, body, jsonOk };
  },
  async getState() {
    const response = await fetch('/state/v2', { cache: 'no-store' });
    if (!response.ok) return null;
    return response.json();
  },
};

const clock = {
  now: () => Date.now(),
  setTimeout: (fn, ms) => globalThis.setTimeout(fn, ms),
  clearTimeout: (id) => globalThis.clearTimeout(id),
};

const controller = new AutoPlanController({ observeOnly, transport, clock, onChange: render });

function format(n, digits) {
  return Number.isFinite(n) ? n.toFixed(digits) : '—';
}

function show(el, visible) {
  el.hidden = !visible;
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

function render() {
  const view = controller.view();
  renderStatus(view);
  renderGallery(view);
  renderVersions(view);
  renderStart(view);
  renderObstacles(view);
  renderResult(view);
  renderMap(view);
}

function renderStatus(view) {
  const { status } = view;
  $('status').textContent = status.badge;
  const tone = { result: 'ok', failure: 'error' }[status.kind] || '';
  $('status').className = `badge ${tone}`;
  $('planStatus').textContent = status.text;
  $('planStatus').className = `plan-status ${status.kind === 'failure' ? 'error-text' : ''}`;
  $('planDetail').textContent = status.detail;
  show($('planDetail'), Boolean(status.detail));

  show($('observeBanner'), view.observeOnly);
  const notes = [];
  if (view.externalMissionId) notes.push(`External mission ${view.externalMissionId}.`);
  if (view.readOnlyReason === 'off-grid') {
    notes.push('Its map is not on this page’s 10 cm grid (or its heading is not N/E/S/W), so it is read-only. Clear the map to plan your own.');
  } else if (view.readOnlyReason === 'busy') {
    notes.push('Shown while the planner works on it; your newest map is queued.');
  } else if (view.readOnlyReason === 'malformed') {
    notes.push('Its request could not be drawn.');
  }
  $('externalNote').textContent = notes.join(' ');
  show($('externalNote'), notes.length > 0);
}

let renderedStartKey = null;

function renderStart(view) {
  const inputs = [$('startX'), $('startY'), $('startHeading')];
  inputs.forEach((el) => { el.disabled = !view.editable; });

  // Rewrite the inputs only when the accepted start changed from outside
  // (e.g. an adopted external map), never over the user's own typing.
  const key = JSON.stringify([view.localStart, view.startEpoch]);
  if (key !== renderedStartKey) {
    renderedStartKey = key;
    if (!view.startProblem) {
      const values = [String(view.localStart.x), String(view.localStart.y), view.localStart.heading];
      inputs.forEach((el, i) => { if (el.value !== values[i]) el.value = values[i]; });
    }
  }

  $('startProblem').textContent = view.startProblem || '';
  show($('startProblem'), Boolean(view.startProblem) && view.editable);

  const start = view.map?.start;
  const note = !view.editable && start
    ? `Showing x ${format(start.x_cm, 1)} cm · y ${format(start.y_cm, 1)} cm · heading ${format(start.heading_deg, 1)}° (read-only).`
    : '';
  $('startNote').textContent = note;
  show($('startNote'), Boolean(note));
}

function renderObstacles(view) {
  $('obstacleCount').textContent = `${view.obstacleCount} / ${view.maxObstacles}`;
  document.querySelectorAll('[data-face]').forEach((button) => {
    button.classList.toggle('selected', button.dataset.face === view.activeFace);
    button.disabled = !view.editable;
  });
  $('faceHint').textContent = !view.editable
    ? 'Read-only map.'
    : view.selectedId !== null
      ? `Facing buttons change obstacle ${view.selectedId}. Press Delete to remove it, Esc to deselect.`
      : 'Click an empty grid cell to place an obstacle facing the selected side. Click an obstacle to select and correct it.';
  $('clearMap').disabled = !view.canClear;

  const list = $('obstacles');
  list.replaceChildren();
  const obstacles = view.editable
    ? view.localObstacles.map((o) => ({ id: o.id, label: `(${o.x}, ${o.y}) · face ${o.face}` }))
    : (view.map?.obstacles || []).map((o) => ({ id: o.id, label: `(${format(o.x, 1)}, ${format(o.y, 1)}) cm · face ${o.face}` }));
  if (!obstacles.length) {
    const empty = document.createElement('div');
    empty.className = 'muted';
    empty.textContent = view.editable ? 'Click an empty grid cell to add an obstacle.' : 'No obstacles.';
    list.appendChild(empty);
    return;
  }
  [...obstacles].sort((a, b) => a.id - b.id).forEach((o) => {
    const row = document.createElement('div');
    row.className = 'obstacle-item obstacle-item-auto';
    if (o.id === view.selectedId) row.classList.add('selected');

    const select = document.createElement('button');
    select.className = 'obstacle-select';
    select.disabled = !view.editable;
    const id = document.createElement('span');
    id.className = 'obstacle-id';
    id.textContent = o.id;
    const text = document.createElement('span');
    text.textContent = o.label;
    select.append(id, text);

    const del = document.createElement('button');
    del.className = 'danger';
    del.textContent = 'Delete';
    del.disabled = !view.editable;

    row.append(select, del);
    list.appendChild(row);
  });
}

function renderResult(view) {
  const shown = view.shown;
  const result = shown?.kind === 'result' ? shown.result : null;

  $('resultLabel').textContent = result ? shown.label : '';
  show($('resultLabel'), Boolean(result));
  $('backendMessage').textContent = result && shown.backendMessage ? `Planner: ${shown.backendMessage}` : '';
  show($('backendMessage'), Boolean(result && shown.backendMessage));

  const failure = shown?.kind === 'failure' ? shown : null;
  $('failureTitle').textContent = failure ? failure.title : '';
  $('failureMessage').textContent = failure ? failure.message : '';
  $('failureBackend').textContent = failure?.backendMessage ? `Planner: ${failure.backendMessage}` : '';
  show($('failure'), Boolean(failure));

  $('totalCost').textContent = result ? format(result.total_cost, 2) : '—';
  $('distance').textContent = result ? `${format(result.total_distance_cm, 1)} cm` : '—';
  $('actionCount').textContent = result ? String(result.actions.length) : '—';

  const order = $('order');
  order.replaceChildren();
  if (result) {
    result.obstacle_order.forEach((id) => {
      const chip = document.createElement('div');
      chip.className = 'route-step';
      chip.textContent = id;
      order.appendChild(chip);
    });
  } else {
    const none = document.createElement('span');
    none.className = 'muted';
    none.textContent = 'No route.';
    order.appendChild(none);
  }

  const box = $('commands');
  box.replaceChildren();
  if (!result) {
    const none = document.createElement('div');
    none.className = 'muted';
    none.style.padding = '9px';
    none.textContent = 'No commands.';
    box.appendChild(none);
    return;
  }
  // STM setup first: the RPi sends these (profile set, then verify) once, with
  // the STM idle, before the first MOV. They are not actions and not counted.
  if (shown.setupRows?.length) {
    const header = document.createElement('div');
    header.className = 'setup-header';
    header.textContent = 'STM setup — the RPi sends this before the first MOV';
    box.appendChild(header);
    shown.setupRows.forEach((step, index) => {
      const row = document.createElement('div');
      row.className = 'setup-row';
      const left = document.createElement('span');
      left.textContent = index === 0 ? 'SETUP: set' : 'SETUP: verify';
      const right = document.createElement('span');
      right.textContent = `${step.command} → ${step.expect}`;
      row.append(left, right);
      box.appendChild(row);
    });
  }
  result.actions.forEach((action) => {
    const row = document.createElement('div');
    row.className = 'command';
    if (action.type === 'SNAP') row.classList.add('snap');
    const left = document.createElement('span');
    left.textContent = `#${action.seq} ${action.type}`;
    const right = document.createElement('span');
    right.textContent = action.type === 'SNAP'
      ? `obstacle ${action.obstacle_id}`
      : typeof action.stm_command === 'string' && action.stm_command ? action.stm_command : '—';
    row.append(left, right);
    box.appendChild(row);
  });
}

function renderMap(view) {
  const svg = $('map');
  svg.replaceChildren();
  const add = createSvgAdder(svg);
  drawArenaGrid(add);

  const result = view.shown?.kind === 'result' ? view.shown.result : null;
  if (result) {
    // Static: the whole route at once, no preview cursor.
    drawFullAndCompletedPath(add, result.waypoints, result.waypoints.length - 1);
    result.target_poses.forEach((pose, index) => drawTargetRing(add, pose, index, result.obstacle_order[index], false));
    result.actions
      .filter((action) => action.type === 'SNAP')
      .forEach((action) => drawSnapMarker(add, action.pose, action.obstacle_id, 'remaining'));
  }

  if (view.map) {
    drawObstaclesLayer(add, view.map.obstacles, new Set());
    const selected = view.map.obstacles.find((o) => o.id === view.selectedId);
    if (selected) {
      const [x, y] = svgPoint(selected.x, selected.y + CELL_CM);
      const size = (CELL_CM / ARENA_CM) * 1000;
      add('rect', { x: x - 6, y: y - 6, width: size + 12, height: size + 12, rx: 8, fill: 'none', stroke: '#f59e0b', 'stroke-width': 5 });
    }
    drawRobot(add, view.map.start);
  }
  drawArenaBorder(add);
}

// ---------------------------------------------------------------------------
// Camera gallery (PROTOCOL.md section 7.1): the RPi's photo after each SNAP.
// Display only: no handler on any tile. Rebuilt only when the set of photos or
// the mission on screen changes, so images never flicker between polls.
// ---------------------------------------------------------------------------

const IMAGE_POLL_MS = 1000;
let imageList = null; // normalized `GET /images` reply, or null (nothing valid yet)
let renderedGalleryKey = null;

function renderGallery(view = controller.view()) {
  const images = selectGalleryImages(imageList, view.missionId);
  const key = galleryKey(images, view.missionId);
  if (key === renderedGalleryKey) return;
  renderedGalleryKey = key;

  $('galleryCount').textContent = countText(images);
  $('galleryEmpty').hidden = images.length > 0;
  const grid = $('gallery');
  grid.replaceChildren();
  images.forEach((image) => {
    const tile = document.createElement('figure');
    tile.className = 'gallery-tile';
    const picture = document.createElement('img');
    picture.className = 'gallery-img';
    picture.alt = `${tileCaption(image)} photo`;
    picture.src = imageUrl(image);
    const caption = document.createElement('figcaption');
    caption.textContent = tileCaption(image);
    tile.append(picture, caption);
    grid.appendChild(tile);
  });
}

async function pollImages() {
  try {
    const response = await fetch('/images', { cache: 'no-store' });
    if (!response.ok) return;
    imageList = normalizeImageList(await response.json());
  } catch (error) {
    return; // transient: the next poll retries, what is shown stays
  }
  renderGallery();
}

// ---------------------------------------------------------------------------
// Planner version (versions.js): which planner the newest request used, and
// which one this page draws. Display only: `GET /state/versions`, never posts.
// ---------------------------------------------------------------------------

let versionInfo = null; // { versions: normalized reply or null, fetchedAt } once polled

/** The last reply, with the time since it arrived added to the request's age. */
function currentVersions() {
  if (!versionInfo?.versions?.last) return versionInfo?.versions ?? null;
  const { last } = versionInfo.versions;
  return { last: { ...last, ageS: last.ageS + Math.max(0, Date.now() - versionInfo.fetchedAt) / 1000 } };
}

function setCircle(id, label, on, onClass) {
  $(id).className = on ? `vcircle ${onClass}` : 'vcircle';
  $(id).setAttribute('aria-label', `${label} ${on ? 'active' : 'not active'}`);
}

function renderVersions(view = controller.view()) {
  const indicator = versionIndicator(currentVersions(), view.missionId ?? null);
  setCircle('reqV1', 'V1', indicator.request.active === 1, 'on-v1');
  setCircle('reqV2', 'V2', indicator.request.active === 2, 'on-v2');
  setCircle('shownV1', 'V1', indicator.shown.active === 1, 'on-v1');
  setCircle('shownV2', 'V2', indicator.shown.active === 2, 'on-v2');
  $('reqText').textContent = indicator.request.text;
  $('shownText').textContent = indicator.shown.text;
  $('versionWarning').textContent = indicator.warning;
  show($('versionWarning'), indicator.warning !== '');
}

async function pollVersions() {
  try {
    const response = await fetch('/state/versions', { cache: 'no-store' });
    if (response.ok) versionInfo = { versions: normalizeVersions(await response.json()), fetchedAt: Date.now() };
  } catch (error) {
    // transient: keep what is shown; its age keeps growing from the last reply
  }
  renderVersions();
}

// The status line shows running clocks (countdown, planning elapsed time).
globalThis.setInterval(() => {
  if (controller.needsTick()) renderStatus(controller.view());
}, 250);

render();
controller.start();
pollImages();
globalThis.setInterval(pollImages, IMAGE_POLL_MS);
pollVersions();
globalThis.setInterval(pollVersions, VERSION_POLL_MS);
