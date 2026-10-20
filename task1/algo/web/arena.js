/**
 * Arena drawing shared by the automatic page (`auto.js`) and the manual/debug
 * page (`app.js`). Moved verbatim from `app.js`; behaviour is unchanged.
 *
 * Pure DOM helpers: every function takes an `add(tag, attrs, parent?)` SVG
 * element factory plus explicit data, never module or page state.
 */

export const ARENA_CM = 200;
export const GRID = 20;
export const CELL_CM = 10;
// Measured robot geometry about the rear reference point (the pose).
// Mirrors algo/config.py; keep the two in sync.
export const ROBOT_FRONT_EXTENT_CM = 24;
export const ROBOT_REAR_EXTENT_CM = 0;
export const ROBOT_WIDTH_CM = 19;
export const CAMERA_FORWARD_OFFSET_CM = 15.25;
export const CAMERA_LATERAL_OFFSET_CM = 0;
export const SVG_PER_CM = 1000 / ARENA_CM;
// The arena is drawn in 0..1000 user units. The axis numbers sit outside it
// (left and below), so the display page's SVG `viewBox` is the arena plus
// that margin: "-40 -10 1050 1050" in `index.html` (square, nothing moves).
export const GRID_THICK_EVERY = 5;

export function svgPoint(xCm, yCm) {
  const x = (xCm / ARENA_CM) * 1000;
  const y = 1000 - (yCm / ARENA_CM) * 1000;
  return [x, y];
}

/**
 * The grid: 20 x 20 cells, a heavier line every 5 cells, and the cell numbers
 * 0-19 (cell index, not cm) centred on each column below and each row to the
 * left of the arena.
 */
export function drawArenaGrid(add) {
  add('rect', { x: 0, y: 0, width: 1000, height: 1000, fill: '#ffffff' });

  for (let i = 0; i <= GRID; i++) {
    const p = (i / GRID) * 1000;
    const thick = i % GRID_THICK_EVERY === 0;
    const line = thick
      ? { stroke: '#c9d0db', 'stroke-width': 2 }
      : { stroke: '#e7ebf1', 'stroke-width': 1 };
    add('line', { x1: p, y1: 0, x2: p, y2: 1000, ...line });
    add('line', { x1: 0, y1: p, x2: 1000, y2: p, ...line });
  }

  add('rect', {
    x: 0,
    y: 1000 - 200,
    width: 200,
    height: 200,
    fill: '#eef3ff',
    stroke: '#b9c9f7',
    'stroke-width': 2,
  });
  const zoneText = add('text', {
    x: 100,
    y: 970,
    'text-anchor': 'middle',
    fill: '#7180a0',
    'font-size': 20,
    'font-weight': 650,
  });
  zoneText.textContent = 'START ZONE';

  const cell = 1000 / GRID;
  for (let i = 0; i < GRID; i++) {
    const tx = add('text', {
      x: (i + 0.5) * cell,
      y: 1000 + 24,
      'text-anchor': 'middle',
      fill: '#8a94a6',
      'font-size': 16,
    });
    tx.textContent = `${i}`;
    const ty = add('text', {
      x: -12,
      y: 1000 - (i + 0.5) * cell + 5,
      'text-anchor': 'end',
      fill: '#8a94a6',
      'font-size': 16,
    });
    ty.textContent = `${i}`;
  }
}

export function drawArenaBorder(add) {
  add('rect', {
    x: 1,
    y: 1,
    width: 998,
    height: 998,
    fill: 'none',
    stroke: '#aeb7c6',
    'stroke-width': 3,
  });
}

export function drawObstaclesLayer(add, obstacles, visitedIds) {
  obstacles.forEach((o) => {
    const x = (o.x / ARENA_CM) * 1000;
    const y = 1000 - ((o.y + CELL_CM) / ARENA_CM) * 1000;
    const visited = visitedIds.has(o.id);

    add('rect', {
      x,
      y,
      width: 50,
      height: 50,
      rx: 4,
      fill: visited ? '#a7d8bd' : '#25334d',
      stroke: visited ? '#2b8a57' : '#172033',
      'stroke-width': 2,
    });

    const label = add('text', {
      x: x + 25,
      y: y + 31,
      'text-anchor': 'middle',
      fill: '#fff',
      'font-size': 17,
      'font-weight': 800,
    });
    label.textContent = o.id;

    const side = {
      N: [x + 25, y + 4, x + 25, y - 13],
      S: [x + 25, y + 46, x + 25, y + 63],
      E: [x + 46, y + 25, x + 63, y + 25],
      W: [x + 4, y + 25, x - 13, y + 25],
    }[o.face];
    add('line', {
      x1: side[0],
      y1: side[1],
      x2: side[2],
      y2: side[3],
      stroke: '#e05252',
      'stroke-width': 6,
      'stroke-linecap': 'round',
    });
  });
}

/** Draws the translucent full route plus a solid "completed" overlay up to
 * (and including) `uptoIndex`. Shared by both engines -- takes plain
 * `{x_cm, y_cm}` points, never schema-specific fields. */
export function drawFullAndCompletedPath(add, waypoints, uptoIndex) {
  if (waypoints.length < 2) return;

  const pathD = (points) =>
    points.map((p, i) => `${i ? 'L' : 'M'} ${svgPoint(p.x_cm, p.y_cm).join(' ')}`).join(' ');

  add('path', {
    d: pathD(waypoints),
    fill: 'none',
    stroke: '#1f5eff',
    'stroke-width': 7,
    'stroke-linecap': 'round',
    'stroke-linejoin': 'round',
    opacity: 0.22,
  });

  const travelled = waypoints.slice(0, uptoIndex + 1);
  if (travelled.length > 1) {
    add('path', {
      d: pathD(travelled),
      fill: 'none',
      stroke: '#1f5eff',
      'stroke-width': 9,
      'stroke-linecap': 'round',
      'stroke-linejoin': 'round',
    });
  }
}

export function drawRobot(add, poseDeg) {
  const [cx, cy] = svgPoint(poseDeg.x_cm, poseDeg.y_cm);
  const angleDeg = 90 - poseDeg.heading_deg;
  const robot = add('g', { transform: `translate(${cx} ${cy}) rotate(${angleDeg})` });
  const front = ROBOT_FRONT_EXTENT_CM * SVG_PER_CM;
  const rear = ROBOT_REAR_EXTENT_CM * SVG_PER_CM;
  const halfWidth = (ROBOT_WIDTH_CM / 2) * SVG_PER_CM;

  add(
    'rect',
    {
      x: -halfWidth,
      y: -front,
      width: halfWidth * 2,
      height: front + rear,
      rx: 10,
      fill: '#f59e0b',
      stroke: '#9a6500',
      'stroke-width': 4,
    },
    robot,
  );
  add('path', { d: `M 0 ${-front - 18} L -14 ${-front + 6} L 14 ${-front + 6} Z`, fill: '#9a6500' }, robot);
  // Camera lens.
  add(
    'circle',
    {
      cx: -CAMERA_LATERAL_OFFSET_CM * SVG_PER_CM,
      cy: -CAMERA_FORWARD_OFFSET_CM * SVG_PER_CM,
      r: 7,
      fill: '#fde047',
      stroke: '#9a6500',
      'stroke-width': 2,
    },
    robot,
  );
  // Rear reference point (planner pose).
  add('circle', { cx: 0, cy: 0, r: 7, fill: '#9a6500', stroke: '#fff', 'stroke-width': 2 }, robot);
  const robotText = add(
    'text',
    { x: 0, y: -front / 2 + 8, 'text-anchor': 'middle', fill: '#fff', 'font-size': 23, 'font-weight': 850 },
    robot,
  );
  robotText.textContent = 'R';
}

// White outline drawn behind label text so it stays readable over the path,
// grid lines and neighbouring markers.
export const LABEL_HALO = { stroke: '#fff', 'stroke-width': 4, 'stroke-linejoin': 'round', 'paint-order': 'stroke' };

export function drawTargetRing(add, poseCm, index, obstacleId, done) {
  const [x, y] = svgPoint(poseCm.x_cm, poseCm.y_cm);
  add('circle', {
    cx: x,
    cy: y,
    r: 13,
    fill: done ? '#2b8a57' : '#fff',
    stroke: done ? '#2b8a57' : '#1f5eff',
    'stroke-width': 4,
    opacity: done ? 1 : 0.55,
  });
  const t = add('text', {
    x,
    y: y + 6,
    'text-anchor': 'middle',
    fill: done ? '#fff' : '#1f5eff',
    'font-size': 15,
    'font-weight': 800,
    opacity: done ? 1 : 0.7,
  });
  t.textContent = index + 1;
  const label = add('text', { x: x + 18, y: y - 15, fill: '#536075', 'font-size': 14, 'font-weight': 700, ...LABEL_HALO });
  label.textContent = `O${obstacleId}`;
}

/** SNAP's *actual* estimated capture pose -- distinct from the faint
 * "ideal target" ring above, since v2 explicitly allows the two to differ
 * (a SNAP's pose is the leg's estimated final pose, never the ideal
 * target_pose). Styled as a small camera glyph, colored by preview status. */
export function drawSnapMarker(add, poseCm, obstacleId, status) {
  const [x, y] = svgPoint(poseCm.x_cm, poseCm.y_cm);
  const colors = {
    completed: { fill: '#2b8a57', stroke: '#1d5f3c' },
    active: { fill: '#fde047', stroke: '#9a6500' },
    remaining: { fill: '#fff', stroke: '#9aa4b8' },
  }[status];
  add('circle', { cx: x, cy: y, r: 9, fill: colors.fill, stroke: colors.stroke, 'stroke-width': 3 });
  // Below the marker (clear of the "O<n>" label above-right); above it only
  // when the marker is too close to the arena's bottom edge. Label position
  // only -- the marker itself stays exactly at the estimated capture pose.
  const label = add('text', {
    x,
    y: y < 950 ? y + 28 : y - 32,
    'text-anchor': 'middle',
    fill: '#3f4c63',
    'font-size': 13,
    'font-weight': 750,
    ...LABEL_HALO,
  });
  label.textContent = `SNAP O${obstacleId}`;
}

/** Grid cell under a pointer event, from the SVG's client rectangle. Shared
 * by both pages; the same arithmetic `app.js` used inline before. */
export function cellFromClientPoint(rect, clientX, clientY) {
  const x = ((clientX - rect.left) / rect.width) * ARENA_CM;
  const y = ARENA_CM - ((clientY - rect.top) / rect.height) * ARENA_CM;
  return {
    x: Math.max(0, Math.min(GRID - 1, Math.floor(x / CELL_CM))),
    y: Math.max(0, Math.min(GRID - 1, Math.floor(y / CELL_CM))),
  };
}

/** `add(tag, attrs, parent = svg)` factory used by every drawing helper. */
export function createSvgAdder(svg) {
  const ns = 'http://www.w3.org/2000/svg';
  return (tag, attrs, parent = svg) => {
    const el = document.createElementNS(ns, tag);
    for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
    parent.appendChild(el);
    return el;
  };
}
