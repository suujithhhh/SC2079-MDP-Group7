// Tests for versions.js: pure planner-version indicator helpers (no DOM, no fetch).

import { describe, test } from 'node:test';
import assert from 'node:assert/strict';

import { VERSION_POLL_MS, agoText, normalizeVersions, versionIndicator } from '../versions.js';

const wire = (over = {}) => ({
  type: 'PLAN_VERSIONS',
  last_request: { protocol_version: 2, source: 'rpi', mission_id: 'task1-run-001', age_s: 4.2, ...over },
});
const lastOf = (over = {}) => normalizeVersions(wire(over));

describe('normalizeVersions', () => {
  test('no request yet is { last: null }', () => {
    assert.deepEqual(normalizeVersions({ type: 'PLAN_VERSIONS', last_request: null }), { last: null });
  });

  test('a valid request is renamed to camelCase (an absent mode is null)', () => {
    assert.deepEqual(lastOf(), {
      last: { protocolVersion: 2, source: 'rpi', missionId: 'task1-run-001', mode: null, ageS: 4.2 },
    });
    assert.deepEqual(lastOf({ protocol_version: 1, mission_id: null, source: 'browser' }).last,
      { protocolVersion: 1, source: 'browser', missionId: null, mode: null, ageS: 4.2 });
  });

  test('the mode is upper-cased, null stays null, and a non-string or odd mode is rejected', () => {
    assert.equal(lastOf({ mode: 'b3' }).last.mode, 'B3');
    assert.equal(lastOf({ mode: 'B2' }).last.mode, 'B2');
    assert.equal(lastOf({ mode: null }).last.mode, null);
    for (const mode of [3, true, [], {}, '', 'x'.repeat(17)]) {
      assert.equal(normalizeVersions(wire({ mode })), null, JSON.stringify(mode));
    }
  });

  test('anything off is rejected whole', () => {
    const bad = [
      null, undefined, 'x', 5, [], {}, { type: 'IMAGE_LIST', last_request: null },
      { type: 'PLAN_VERSIONS' }, { type: 'PLAN_VERSIONS', last_request: 'v2' },
      wire({ protocol_version: 3 }), wire({ protocol_version: '2' }), wire({ protocol_version: 0 }),
      wire({ source: 'robot' }), wire({ source: undefined }), wire({ source: '__proto__' }),
      wire({ mission_id: '' }), wire({ mission_id: 7 }),
      wire({ age_s: -1 }), wire({ age_s: '3' }), wire({ age_s: NaN }), wire({ age_s: Infinity }),
    ];
    for (const raw of bad) assert.equal(normalizeVersions(raw), null, JSON.stringify(raw));
  });
});

describe('agoText', () => {
  test('just now, seconds, minutes', () => {
    assert.equal(agoText(0), 'just now');
    assert.equal(agoText(1.4), 'just now');
    assert.equal(agoText(2), '2 s ago');
    assert.equal(agoText(59.6), '60 s ago');
    assert.equal(agoText(89), '89 s ago');
    assert.equal(agoText(90), '2 min ago');
    assert.equal(agoText(600), '10 min ago');
  });
});

describe('versionIndicator', () => {
  test('nothing valid yet: nothing lit in the request row, V2 in the page row', () => {
    const waiting = versionIndicator(null);
    assert.equal(waiting.request.active, null);
    assert.equal(waiting.request.text, 'Waiting for the planner.');
    assert.equal(waiting.shown.active, 2);
    assert.equal(waiting.warning, '');

    const none = versionIndicator({ last: null });
    assert.equal(none.request.active, null);
    assert.equal(none.request.text, 'No plan request yet.');
  });

  test('the page row is always V2, whatever the request used', () => {
    for (const version of [1, 2]) {
      assert.equal(versionIndicator(lastOf({ protocol_version: version })).shown.active, 2);
    }
    assert.match(versionIndicator(null).shown.text, /cannot draw V1/);
  });

  test('a V2 request lights V2 and names the sender, endpoint, age and mission', () => {
    const indicator = versionIndicator(lastOf(), null);
    assert.equal(indicator.request.active, 2);
    assert.equal(indicator.request.text, 'Robot (RPi) · V2 (/plan/v2) · 4 s ago · mission task1-run-001');
    assert.equal(indicator.warning, '');
  });

  test('a V1 request lights V1 and warns that this page cannot draw it', () => {
    const indicator = versionIndicator(lastOf({ protocol_version: 1, mission_id: null }), 'mission-on-screen');
    assert.equal(indicator.request.active, 1);
    assert.equal(indicator.request.text, 'Robot (RPi) · V1 (/plan) · 4 s ago');
    assert.match(indicator.warning, /last request used V1 \(\/plan\)/);
    assert.match(indicator.warning, /only shows V2 plans/);
  });

  test('the row names the mode, and only a mode other than B3 warns', () => {
    const b3 = versionIndicator(lastOf({ mode: 'B3' }), null);
    assert.equal(b3.request.text, 'Robot (RPi) · V2 (/plan/v2) · B3 · 4 s ago · mission task1-run-001');
    assert.equal(b3.warning, '');
    const b2 = versionIndicator(lastOf({ mode: 'B2' }), null);
    assert.match(b2.request.text, /V2 \(\/plan\/v2\) · B2 · 4 s ago/);
    assert.equal(b2.warning, 'The last request used mode B2, not B3 (the team default).');
    assert.equal(versionIndicator(lastOf({ mode: null }), null).warning, '', 'an unknown mode is not a warning');
  });

  test('a V1 request in B2 shows both warnings', () => {
    const both = versionIndicator(lastOf({ protocol_version: 1, mission_id: null, mode: 'B2' }), null);
    assert.match(both.warning, /last request used V1/);
    assert.match(both.warning, /used mode B2, not B3/);
  });

  test('a browser sender is not called the robot', () => {
    assert.match(versionIndicator(lastOf({ source: 'browser' })).request.text, /^A browser page · V2/);
  });

  test('a V2 mission other than the one on screen warns; the same one, or none on screen, does not', () => {
    const newer = versionIndicator(lastOf({ mission_id: 'mission-newer' }), 'mission-shown');
    assert.match(newer.warning, /mission-shown.*mission-newer/);
    assert.equal(versionIndicator(lastOf({ mission_id: 'same' }), 'same').warning, '');
    assert.equal(versionIndicator(lastOf({ mission_id: 'm' }), null).warning, '');
    assert.equal(versionIndicator(lastOf({ mission_id: null }), 'shown').warning, '');
  });

  test('the poll interval is one second', () => {
    assert.equal(VERSION_POLL_MS, 1000);
  });
});
