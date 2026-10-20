// Tests for the grid drawn by algo/web/arena.js: 20 x 20 cells, a heavier line
// every 5 cells, and the cell numbers 0-19 outside the arena.
//
// Run with:  node --test algo/web/tests/arena.test.js

import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

import { GRID, drawArenaGrid } from '../arena.js';

function drawn() {
  const elements = [];
  const add = (tag, attrs) => {
    const el = { tag, attrs, textContent: '' };
    elements.push(el);
    return el;
  };
  drawArenaGrid(add);
  return elements;
}

describe('drawArenaGrid', () => {
  const elements = drawn();
  const vertical = elements.filter((e) => e.tag === 'line' && e.attrs.x1 === e.attrs.x2);
  const labels = elements.filter((e) => e.tag === 'text' && /^\d+$/.test(e.textContent));

  test('21 vertical lines, with a heavier line at cells 5, 10 and 15', () => {
    assert.equal(vertical.length, GRID + 1);
    const heavy = vertical.filter((e) => e.attrs['stroke-width'] > 1).map((e) => e.attrs.x1);
    assert.deepEqual(heavy, [0, 250, 500, 750, 1000]);
  });

  test('the column numbers are the cell indices 0-19, centred under each cell', () => {
    const columns = labels.filter((e) => e.attrs.y > 1000);
    assert.deepEqual(columns.map((e) => e.textContent), Array.from({ length: GRID }, (_, i) => `${i}`));
    columns.forEach((e, i) => {
      assert.equal(e.attrs.x, i * 50 + 25);
      assert.equal(e.attrs['text-anchor'], 'middle');
    });
  });

  test('the row numbers are the cell indices 0-19, bottom to top, left of the arena', () => {
    const rows = labels.filter((e) => e.attrs.x < 0);
    assert.deepEqual(rows.map((e) => e.textContent), Array.from({ length: GRID }, (_, i) => `${i}`));
    rows.forEach((e, i) => {
      assert.ok(Math.abs(e.attrs.y - (1000 - (i * 50 + 25) + 5)) < 1e-9);
      assert.equal(e.attrs['text-anchor'], 'end');
    });
  });

  test('there are no other numeric labels (the old 0/50/100/150/200 cm axis is gone)', () => {
    assert.equal(labels.length, 2 * GRID);
  });

  test('the START ZONE is still 200 x 200 units in the lower-left corner', () => {
    const zone = elements.find((e) => e.tag === 'rect' && e.attrs.fill === '#eef3ff');
    assert.deepEqual([zone.attrs.x, zone.attrs.y, zone.attrs.width, zone.attrs.height], [0, 800, 200, 200]);
  });
});

describe('the display page frame', () => {
  test('index.html shows the arena plus the label margin, square, so nothing is clipped', () => {
    const html = readFileSync(new URL('../index.html', import.meta.url), 'utf8');
    const match = /<svg[^>]*\bviewBox="(-?\d+) (-?\d+) (\d+) (\d+)"/s.exec(html);
    assert.ok(match, 'index.html has an svg viewBox');
    const [x, y, w, h] = match.slice(1).map(Number);
    assert.equal(w, h, 'square, so the page layout is unchanged');
    assert.ok(x <= -34 && x + w >= 1000 && y <= 0 && y + h >= 1000 + 30, 'arena and both axes fit');
  });
});
