// Original Roadmap Step 5 (web) -- tests for algo/web/planv2.js.
//
// Run with:
//   node --test algo/web/tests/planv2.test.js
//
// Zero dependencies: Node's own built-in `node:test`/`node:assert/strict`.
// Mirrors several test names in algo/tests/test_planning_service_v2.py so
// the server-side and client-side suites can be read side by side.

import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import {
  headingLetterToDeg,
  normalizeHeadingDeg,
  generateMissionId,
  buildPlanV2Request,
  buildPreviewSteps,
  classifyActionsAtPreviewStep,
  mergeV2State,
  normalizeV1Result,
  normalizeV2Result,
  PreviewModelError,
  serializedPoseEquals,
  MAX_OBSTACLES_V2,
  isExecutableV2Result,
  validateV2Result,
  optimalityLabelV2,
  noRouteExplanationV2,
  MOTION_PROFILES,
  expectedStmSetup,
  validateStmSetup,
} from '../planv2.js';

// ---------------------------------------------------------------------------
// Heading conversion
// ---------------------------------------------------------------------------

test('bounded v2 accepts up to eight obstacles', () => {
  assert.equal(MAX_OBSTACLES_V2, 8);
});

describe('isExecutableV2Result', () => {
  const actions = [{ seq: 0, type: 'MOV', stm_command: 'MOV,F,5' }];

  test('only valid and complete with actions is executable', () => {
    assert.equal(isExecutableV2Result({ valid: true, complete: true, actions }), true);
  });

  test('a clear bounded failure is never executable', () => {
    assert.equal(isExecutableV2Result({ valid: false, complete: false, actions: [] }), false);
  });

  test('valid without complete, or complete without valid, is not executable', () => {
    assert.equal(isExecutableV2Result({ valid: true, complete: false, actions }), false);
    assert.equal(isExecutableV2Result({ valid: false, complete: true, actions }), false);
  });

  test('missing flags, odd types, empty actions and no result are not executable', () => {
    assert.equal(isExecutableV2Result({ valid: true, actions }), false);
    assert.equal(isExecutableV2Result({ valid: 'true', complete: 'true', actions }), false);
    assert.equal(isExecutableV2Result({ valid: true, complete: true, actions: [] }), false);
    assert.equal(isExecutableV2Result({ valid: true, complete: true }), false);
    assert.equal(isExecutableV2Result(null), false);
    assert.equal(isExecutableV2Result(undefined), false);
  });
});

describe('headingLetterToDeg', () => {
  test('all four compass letters', () => {
    assert.equal(headingLetterToDeg('E'), 0);
    assert.equal(headingLetterToDeg('N'), 90);
    assert.equal(headingLetterToDeg('W'), 180);
    assert.equal(headingLetterToDeg('S'), 270);
  });

  test('rejects an unknown letter', () => {
    assert.throws(() => headingLetterToDeg('Q'), RangeError);
  });
});

describe('normalizeHeadingDeg', () => {
  test('already-normalised values pass through', () => {
    assert.equal(normalizeHeadingDeg(0), 0);
    assert.equal(normalizeHeadingDeg(90), 90);
    assert.equal(normalizeHeadingDeg(359.9), 359.9);
  });

  test('negative values wrap into [0, 360)', () => {
    assert.equal(normalizeHeadingDeg(-90), 270);
  });

  test('values at or above 360 wrap down', () => {
    assert.equal(normalizeHeadingDeg(360), 0);
    assert.equal(normalizeHeadingDeg(450), 90);
  });

  test('negative zero is normalised to positive zero', () => {
    const result = normalizeHeadingDeg(-0);
    assert.equal(result, 0);
    assert.equal(Object.is(result, -0), false);
  });
});

// ---------------------------------------------------------------------------
// Mission ID
// ---------------------------------------------------------------------------

describe('generateMissionId', () => {
  test('uses crypto.randomUUID when available and is prefixed clearly', () => {
    const fakeCrypto = { randomUUID: () => '11111111-2222-3333-4444-555555555555' };
    const id = generateMissionId('web-preview', fakeCrypto);
    assert.match(id, /^web-preview-[0-9a-z]+-11111111$/);
  });

  test('falls back to a non-crypto id when randomUUID is unavailable', () => {
    const id = generateMissionId('web-preview', undefined);
    assert.match(id, /^web-preview-[0-9a-z]+-[0-9a-z]+$/);
  });

  test('two calls never collide', () => {
    const a = generateMissionId('web-preview', undefined);
    const b = generateMissionId('web-preview', undefined);
    assert.notEqual(a, b);
  });
});

// ---------------------------------------------------------------------------
// Request construction
// ---------------------------------------------------------------------------

describe('buildPlanV2Request', () => {
  const uiState = {
    mode: 'B2',
    coordinateUnits: 'grid',
    start: { x: 2, y: 1, heading: 'N' },
    obstacles: [{ id: 1, x: 5, y: 5, face: 'S' }],
    allowAlternativeViewing: false,
  };

  test('exact request shape', () => {
    const request = buildPlanV2Request(uiState, 'web-preview-abc-123');
    assert.deepEqual(request, {
      type: 'PLAN_REQUEST',
      protocol_version: 2,
      mission_id: 'web-preview-abc-123',
      mode: 'B2',
      coordinate_units: 'grid',
      allow_alternative_viewing: false,
      start: { x: 2, y: 1, heading_deg: 90 },
      obstacles: [{ id: 1, x: 5, y: 5, face: 'S' }],
    });
  });

  test('never emits a heading or heading_rad key', () => {
    const request = buildPlanV2Request(uiState, 'm-1');
    assert.equal('heading' in request.start, false);
    assert.equal('heading_rad' in request.start, false);
  });

  test('rejects an empty mission id', () => {
    assert.throws(() => buildPlanV2Request(uiState, ''), RangeError);
  });

  test('rejects a non-boolean allow_alternative_viewing', () => {
    const bad = { ...uiState, allowAlternativeViewing: 'false' };
    assert.throws(() => buildPlanV2Request(bad, 'm-1'), RangeError);
  });

  test('allow_alternative_viewing true is carried through', () => {
    const request = buildPlanV2Request({ ...uiState, allowAlternativeViewing: true }, 'm-1');
    assert.equal(request.allow_alternative_viewing, true);
  });
});

// ---------------------------------------------------------------------------
// Preview-step model
// ---------------------------------------------------------------------------

function pose(x, y, headingDeg) {
  return { x_cm: x, y_cm: y, heading_deg: headingDeg };
}

function mov(seq, start, end) {
  return { seq, type: 'MOV', waypoint_start_index: start, waypoint_end_index: end };
}

function snap(seq, obstacleId, poseValue) {
  return { seq, type: 'SNAP', obstacle_id: obstacleId, pose: poseValue };
}

describe('buildPreviewSteps: MOV ranges', () => {
  test('a single MOV over 3 waypoints creates 3 ordered steps', () => {
    const waypoints = [pose(0, 0, 0), pose(1, 0, 0), pose(2, 0, 0)];
    const actions = [mov(0, 0, 2)];
    const steps = buildPreviewSteps(actions, waypoints);
    assert.deepEqual(steps, [
      { actionIndex: 0, actionSeq: 0, actionType: 'MOV', waypointIndex: 0 },
      { actionIndex: 0, actionSeq: 0, actionType: 'MOV', waypointIndex: 1 },
      { actionIndex: 0, actionSeq: 0, actionType: 'MOV', waypointIndex: 2 },
    ]);
  });

  test('a shared MOV/MOV boundary produces two ordered preview events at the same waypoint', () => {
    const waypoints = [pose(0, 0, 0), pose(1, 0, 0), pose(2, 0, 0)];
    const actions = [mov(0, 0, 1), mov(1, 1, 2)];
    const steps = buildPreviewSteps(actions, waypoints);
    // Both actions touch waypointIndex=1, but as two distinct, separately
    // addressable preview-step entries (different actionIndex).
    const atBoundary = steps.filter((s) => s.waypointIndex === 1);
    assert.equal(atBoundary.length, 2);
    assert.equal(atBoundary[0].actionIndex, 0);
    assert.equal(atBoundary[1].actionIndex, 1);
  });
});

describe('buildPreviewSteps: SNAP handling', () => {
  test('a zero-motion leg containing only SNAP gets its own preview step', () => {
    const shared = pose(0, 0, 0);
    const waypoints = [shared];
    const actions = [snap(0, 7, shared)];
    const steps = buildPreviewSteps(actions, waypoints);
    assert.deepEqual(steps, [{ actionIndex: 0, actionSeq: 0, actionType: 'SNAP', waypointIndex: 0 }]);
  });

  test('SNAP followed by MOV at the same waypoint: MOV continues from the SNAP index', () => {
    const shared = pose(0, 0, 0);
    const next = pose(1, 0, 0);
    const waypoints = [shared, next];
    const actions = [snap(0, 1, shared), mov(1, 0, 1)];
    const steps = buildPreviewSteps(actions, waypoints);
    assert.deepEqual(steps, [
      { actionIndex: 0, actionSeq: 0, actionType: 'SNAP', waypointIndex: 0 },
      { actionIndex: 1, actionSeq: 1, actionType: 'MOV', waypointIndex: 0 },
      { actionIndex: 1, actionSeq: 1, actionType: 'MOV', waypointIndex: 1 },
    ]);
  });

  test('consecutive SNAP actions at the same pose share a waypoint but stay distinct steps', () => {
    const shared = pose(3, 3, 90);
    const waypoints = [shared];
    const actions = [snap(0, 1, shared), snap(1, 2, shared)];
    const steps = buildPreviewSteps(actions, waypoints);
    assert.equal(steps.length, 2);
    assert.equal(steps[0].waypointIndex, 0);
    assert.equal(steps[1].waypointIndex, 0);
    assert.notEqual(steps[0].actionIndex, steps[1].actionIndex);
  });

  test('a SNAP with a genuinely new pose appends and consumes the next waypoint', () => {
    const first = pose(0, 0, 0);
    const second = pose(5, 5, 0);
    const waypoints = [first, second];
    const actions = [snap(0, 1, first), snap(1, 2, second)];
    const steps = buildPreviewSteps(actions, waypoints);
    assert.equal(steps[0].waypointIndex, 0);
    assert.equal(steps[1].waypointIndex, 1);
  });

  test('non-consecutive revisit to an earlier physical pose gets a distinct later index', () => {
    // p_start -> p_mid (MOV), then p_mid -> p_far -> p_mid (two MOVs),
    // mirroring test_revisiting_a_previous_physical_pose_is_not_deduplicated_across_legs.
    const pStart = pose(0, 0, 0);
    const pMid = pose(5, 0, 0);
    const pFar = pose(10, 0, 0);
    const waypoints = [pStart, pMid, pFar, pMid];
    const actions = [
      mov(0, 0, 1),
      mov(1, 1, 2),
      mov(2, 2, 3),
      snap(3, 9, pMid),
    ];
    const steps = buildPreviewSteps(actions, waypoints);
    const snapStep = steps.find((s) => s.actionType === 'SNAP');
    assert.equal(snapStep.waypointIndex, 3, 'the revisit must reuse the later, distinct index, not the earlier one');
  });
});

describe('buildPreviewSteps: malformed/inconsistent data', () => {
  test('a MOV range out of bounds is rejected', () => {
    const waypoints = [pose(0, 0, 0), pose(1, 0, 0)];
    const actions = [mov(0, 0, 5)];
    assert.throws(() => buildPreviewSteps(actions, waypoints), PreviewModelError);
  });

  test('a MOV that does not continue from the previous cursor is rejected', () => {
    const waypoints = [pose(0, 0, 0), pose(1, 0, 0), pose(2, 0, 0)];
    const actions = [mov(0, 0, 1), mov(1, 0, 2)]; // should start at 1, not 0
    assert.throws(() => buildPreviewSteps(actions, waypoints), PreviewModelError);
  });

  test('a SNAP pose inconsistent with the waypoint it would consume is rejected', () => {
    const waypoints = [pose(0, 0, 0), pose(99, 99, 0)]; // second entry doesn't match the SNAP
    const actions = [snap(0, 1, pose(0, 0, 0)), snap(1, 2, pose(1, 1, 0))];
    assert.throws(() => buildPreviewSteps(actions, waypoints), PreviewModelError);
  });

  test('leftover unexplained waypoints are rejected', () => {
    const waypoints = [pose(0, 0, 0), pose(1, 0, 0), pose(2, 0, 0)];
    const actions = [mov(0, 0, 1)]; // never accounts for waypoints[2]
    assert.throws(() => buildPreviewSteps(actions, waypoints), PreviewModelError);
  });

  test('an unknown action type is rejected', () => {
    const waypoints = [pose(0, 0, 0)];
    const actions = [{ seq: 0, type: 'STOP' }];
    assert.throws(() => buildPreviewSteps(actions, waypoints), PreviewModelError);
  });
});

// ---------------------------------------------------------------------------
// Action classification
// ---------------------------------------------------------------------------

describe('classifyActionsAtPreviewStep', () => {
  test('remaining / active / completed at ordinary positions', () => {
    const waypoints = [pose(0, 0, 0), pose(1, 0, 0), pose(2, 0, 0)];
    const actions = [mov(0, 0, 1), mov(1, 1, 2)];
    const steps = buildPreviewSteps(actions, waypoints);

    const atStart = classifyActionsAtPreviewStep(actions, steps, 0);
    assert.equal(atStart[0].status, 'active');
    assert.equal(atStart[1].status, 'remaining');

    const secondActionStart = steps.findIndex((s) => s.actionIndex === 1);
    const inSecond = classifyActionsAtPreviewStep(actions, steps, secondActionStart);
    assert.equal(inSecond[0].status, 'completed');
    assert.equal(inSecond[1].status, 'active');
  });

  test('the final preview step completes the final action, leaving nothing active', () => {
    const waypoints = [pose(0, 0, 0), pose(1, 0, 0), pose(2, 0, 0)];
    const actions = [mov(0, 0, 1), mov(1, 1, 2)];
    const steps = buildPreviewSteps(actions, waypoints);

    const atEnd = classifyActionsAtPreviewStep(actions, steps, steps.length - 1);
    assert.deepEqual(atEnd.map((c) => c.status), ['completed', 'completed']);
  });

  test('an intermediate SNAP is active only on its own step; the final SNAP completes at the end', () => {
    const p0 = pose(0, 0, 0);
    const p1 = pose(1, 0, 0);
    const p2 = pose(2, 0, 0);
    const actions = [mov(0, 0, 1), snap(1, 1, p1), mov(2, 1, 2), snap(3, 2, p2)];
    const steps = buildPreviewSteps(actions, [p0, p1, p2]);
    // steps: MOV0 = 0,1 | SNAP1 = 2 | MOV2 = 3,4 | SNAP3 = 5
    const statuses = (position) => classifyActionsAtPreviewStep(actions, steps, position).map((c) => c.status);

    assert.deepEqual(statuses(1), ['active', 'remaining', 'remaining', 'remaining']);
    assert.deepEqual(statuses(2), ['completed', 'active', 'remaining', 'remaining']);
    assert.deepEqual(statuses(3), ['completed', 'completed', 'active', 'remaining']);
    assert.deepEqual(statuses(4), ['completed', 'completed', 'active', 'remaining']);
    assert.deepEqual(statuses(5), ['completed', 'completed', 'completed', 'completed']);
  });

  test('a shared MOV/MOV boundary yields exactly one active action, never two', () => {
    const waypoints = [pose(0, 0, 0), pose(1, 0, 0), pose(2, 0, 0)];
    const actions = [mov(0, 0, 1), mov(1, 1, 2)];
    const steps = buildPreviewSteps(actions, waypoints);
    const boundaryStepPos = steps.findIndex((s) => s.actionIndex === 0 && s.waypointIndex === 1);

    const classification = classifyActionsAtPreviewStep(actions, steps, boundaryStepPos);
    const activeCount = classification.filter((c) => c.status === 'active').length;
    assert.equal(activeCount, 1);
    assert.equal(classification[0].status, 'active');
    assert.equal(classification[1].status, 'remaining');
  });

  test('partial plan classification stops cleanly at the last real action', () => {
    const waypoints = [pose(0, 0, 0), pose(1, 0, 0)];
    const actions = [mov(0, 0, 1)];
    const steps = buildPreviewSteps(actions, waypoints);
    const midway = classifyActionsAtPreviewStep(actions, steps, 0);
    assert.equal(midway[0].status, 'active');
    const classification = classifyActionsAtPreviewStep(actions, steps, steps.length - 1);
    assert.equal(classification.length, 1);
    assert.equal(classification[0].status, 'completed');
  });
});

// ---------------------------------------------------------------------------
// serializedPoseEquals
// ---------------------------------------------------------------------------

describe('serializedPoseEquals', () => {
  test('exact matches are equal', () => {
    assert.equal(serializedPoseEquals(pose(1, 2, 3), pose(1, 2, 3)), true);
  });

  test('any differing field makes poses unequal', () => {
    assert.equal(serializedPoseEquals(pose(1, 2, 3), pose(1, 2, 3.0001)), false);
  });
});

// ---------------------------------------------------------------------------
// /state/v2 merge guard
// ---------------------------------------------------------------------------

describe('mergeV2State', () => {
  test('a same-mission poll updates the local status', () => {
    const current = { status: 'planning', missionId: 'm-1' };
    const polled = { status: 'complete', mission_id: 'm-1' };
    const merged = mergeV2State(current, polled, 'm-1');
    assert.equal(merged.status, 'complete');
    assert.equal(merged.missionId, 'm-1');
  });

  test('a foreign mission poll is ignored entirely', () => {
    const current = { status: 'planning', missionId: 'm-1' };
    const polled = { status: 'complete', mission_id: 'm-2' };
    const merged = mergeV2State(current, polled, 'm-1');
    assert.deepEqual(merged, current);
  });

  test('with no local submission yet, any snapshot is adopted (bootstrap/reload case)', () => {
    const current = { status: 'idle', missionId: null };
    const polled = { status: 'complete', mission_id: 'm-9' };
    const merged = mergeV2State(current, polled, null);
    assert.equal(merged.status, 'complete');
    assert.equal(merged.missionId, 'm-9');
  });
});

// ---------------------------------------------------------------------------
// Result normalization
// ---------------------------------------------------------------------------

describe('normalizeV1Result', () => {
  test('heading_rad is converted to a normalised heading_deg', () => {
    const raw = {
      waypoints: [{ x_cm: 1, y_cm: 2, heading_rad: Math.PI / 2 }],
      target_poses: [{ x_cm: 3, y_cm: 4, heading_rad: -Math.PI / 2 }],
    };
    const normalized = normalizeV1Result(raw);
    assert.equal(normalized.waypoints[0].heading_deg, 90);
    assert.equal(normalized.target_poses[0].heading_deg, 270);
  });

  test('x_cm/y_cm are never altered', () => {
    const raw = { waypoints: [{ x_cm: 12.34, y_cm: 56.78, heading_rad: 0 }], target_poses: [] };
    const normalized = normalizeV1Result(raw);
    assert.equal(normalized.waypoints[0].x_cm, 12.34);
    assert.equal(normalized.waypoints[0].y_cm, 56.78);
  });

  test('null input passes through unchanged', () => {
    assert.equal(normalizeV1Result(null), null);
  });
});

describe('normalizeV2Result', () => {
  test('heading_deg is re-validated into [0, 360)', () => {
    const raw = {
      waypoints: [{ x_cm: 1, y_cm: 2, heading_deg: 450 }],
      target_poses: [],
      final_pose: { x_cm: 0, y_cm: 0, heading_deg: -90 },
    };
    const normalized = normalizeV2Result(raw);
    assert.equal(normalized.waypoints[0].heading_deg, 90);
    assert.equal(normalized.final_pose.heading_deg, 270);
  });

  test('x_cm/y_cm are never altered', () => {
    const raw = { waypoints: [{ x_cm: 12.34, y_cm: 56.78, heading_deg: 0 }], target_poses: [] };
    const normalized = normalizeV2Result(raw);
    assert.equal(normalized.waypoints[0].x_cm, 12.34);
    assert.equal(normalized.waypoints[0].y_cm, 56.78);
  });

  test('every nested action pose is normalized like waypoints (MOV start/end, SNAP pose)', () => {
    const raw = {
      waypoints: [],
      target_poses: [],
      actions: [
        {
          seq: 0, type: 'MOV', dir: 'F', value: 5, unit: 'cm', stm_command: 'MOV,F,5',
          start_pose: { x_cm: 1.5, y_cm: 2.5, heading_deg: 450 },
          end_pose: { x_cm: 3.5, y_cm: 4.5, heading_deg: -90 },
          waypoint_start_index: 0, waypoint_end_index: 1,
        },
        { seq: 1, type: 'SNAP', obstacle_id: 7, pose: { x_cm: 3.5, y_cm: 4.5, heading_deg: 2.5444437451708134e-14 } },
      ],
    };
    const before = JSON.stringify(raw);
    const normalized = normalizeV2Result(raw);
    const [mov, snap] = normalized.actions;
    assert.deepEqual(mov.start_pose, { x_cm: 1.5, y_cm: 2.5, heading_deg: 90 });
    assert.deepEqual(mov.end_pose, { x_cm: 3.5, y_cm: 4.5, heading_deg: 270 });
    assert.deepEqual(snap.pose, { x_cm: 3.5, y_cm: 4.5, heading_deg: 0 });
    // Everything that is not a pose passes through untouched.
    assert.equal(mov.stm_command, 'MOV,F,5');
    assert.equal(mov.waypoint_end_index, 1);
    assert.equal(snap.obstacle_id, 7);
    assert.equal('stm_command' in snap, false);
    // The input response object is never mutated.
    assert.equal(JSON.stringify(raw), before);
  });

  test('a response without actions is passed through (no crash, no invented actions)', () => {
    assert.equal(normalizeV2Result({ waypoints: [], target_poses: [] }).actions, undefined);
    assert.deepEqual(normalizeV2Result({ waypoints: [], target_poses: [], actions: [] }).actions, []);
  });
});

describe('near-zero heading SNAP regression (A_spread_08 B3)', () => {
  // The raw heading shared by a waypoint and the SNAP that reuses it. Waypoint
  // normalization turns it into exactly 0; the SNAP pose used to keep it.
  const TINY = 2.5444437451708134e-14;
  const mov = (seq, startIndex, endIndex, start, end) => ({
    seq, type: 'MOV', dir: 'F', value: 5, unit: 'cm', stm_command: 'MOV,F,5',
    start_pose: start, end_pose: end,
    waypoint_start_index: startIndex, waypoint_end_index: endIndex,
  });
  const route = (snapPose) => {
    const w0 = { x_cm: 0, y_cm: 0, heading_deg: 350 };
    const w1 = { x_cm: 5, y_cm: 0, heading_deg: TINY };
    const w2 = { x_cm: 10, y_cm: 0, heading_deg: 90 };
    return {
      type: 'PLAN_RESULT', valid: true, complete: true, target_poses: [],
      waypoints: [w0, w1, w2],
      actions: [
        mov(0, 0, 1, w0, w1),
        { seq: 1, type: 'SNAP', obstacle_id: 3, pose: snapPose },
        mov(2, 1, 2, w1, w2),
        { seq: 3, type: 'SNAP', obstacle_id: 4, pose: w2 },
      ],
    };
  };

  test('reproduces the original rejection when only waypoints are normalized', () => {
    const raw = route({ x_cm: 5, y_cm: 0, heading_deg: TINY });
    const waypointsOnly = { ...raw, waypoints: normalizeV2Result(raw).waypoints };
    assert.throws(
      () => buildPreviewSteps(waypointsOnly.actions, waypointsOnly.waypoints),
      (error) => error instanceof PreviewModelError && /SNAP pose does not match waypoints\[2\]/.test(error.message),
    );
  });

  test('a valid near-zero-heading response now builds its preview steps', () => {
    const raw = route({ x_cm: 5, y_cm: 0, heading_deg: TINY });
    const normalized = normalizeV2Result(raw);
    assert.equal(normalized.waypoints[1].heading_deg, 0);
    assert.equal(normalized.actions[1].pose.heading_deg, 0);
    const steps = buildPreviewSteps(normalized.actions, normalized.waypoints);
    const snapSteps = steps.filter((s) => s.actionType === 'SNAP');
    assert.deepEqual(snapSteps.map((s) => s.waypointIndex), [1, 2]);
    // MOV 0..1 (2 steps) + SNAP + MOV 1..2 (2 steps) + SNAP: a SNAP sharing a waypoint is never merged.
    assert.equal(steps.length, 6);
  });

  test('strict checking is preserved: a genuinely different SNAP pose is still rejected', () => {
    // A real (non-collapsing) difference in heading, x or y must still fail:
    // no epsilon was introduced.
    for (const bad of [
      { x_cm: 5, y_cm: 0, heading_deg: 1e-6 },
      { x_cm: 5.000001, y_cm: 0, heading_deg: TINY },
      { x_cm: 5, y_cm: 0.000001, heading_deg: TINY },
    ]) {
      const normalized = normalizeV2Result(route(bad));
      assert.throws(
        () => buildPreviewSteps(normalized.actions, normalized.waypoints),
        PreviewModelError,
      );
    }
  });

  test('the shared gate validateV2Result accepts it and rejects real pose mismatches', () => {
    const withId = (snapPose) => ({ ...route(snapPose), mission_id: 'm-8' });
    const { result, previewSteps } = validateV2Result(withId({ x_cm: 5, y_cm: 0, heading_deg: TINY }), 'm-8');
    assert.equal(result.actions[1].pose.heading_deg, 0);
    assert.equal(previewSteps.length, 6);
    for (const bad of [
      { x_cm: 5, y_cm: 0, heading_deg: 1e-6 },
      { x_cm: 5.000001, y_cm: 0, heading_deg: TINY },
      { x_cm: 5, y_cm: 0.000001, heading_deg: TINY },
    ]) {
      assert.throws(() => validateV2Result(withId(bad), 'm-8'), PreviewModelError);
    }
  });
});

describe('validateV2Result (shared fail-closed gate)', () => {
  const pose = (y) => ({ x_cm: 20, y_cm: y, heading_deg: 90 });
  const plan = (extra = {}) => ({
    type: 'PLAN_RESULT',
    mission_id: 'm',
    valid: true,
    complete: true,
    waypoints: [pose(20), pose(30), pose(40)],
    actions: [
      { seq: 0, type: 'MOV', start_pose: pose(20), end_pose: pose(40), waypoint_start_index: 0, waypoint_end_index: 2 },
      { seq: 1, type: 'SNAP', obstacle_id: 1, pose: pose(40) },
    ],
    ...extra,
  });

  test('rejects a non-PLAN_RESULT and a mismatched mission ID', () => {
    assert.throws(() => validateV2Result(null, 'm'), /not a PLAN_RESULT/);
    assert.throws(() => validateV2Result({ ...plan(), type: 'PLAN_ERROR' }, 'm'), /not a PLAN_RESULT/);
    assert.throws(() => validateV2Result(plan(), 'other'), /mission_id does not match/);
  });

  test('an executable result gets strict preview steps; a non-executable one gets none', () => {
    assert.equal(validateV2Result(plan(), 'm').previewSteps.length, 4);
    const failed = validateV2Result(plan({ valid: false, complete: false, actions: [], waypoints: [] }), 'm');
    assert.deepEqual(failed.previewSteps, []);
    assert.equal(isExecutableV2Result(failed.result), false);
  });

  test('a MOV range that does not chain is rejected', () => {
    const broken = plan();
    broken.actions[0] = { ...broken.actions[0], waypoint_start_index: 1 };
    assert.throws(() => validateV2Result(broken, 'm'), PreviewModelError);
  });
});

describe('proof-flag labels', () => {
  test('"optimal" only when optimality_proven === true', () => {
    assert.equal(optimalityLabelV2({ optimality_proven: true }), 'Optimal route (proven by search)');
    for (const flag of [false, undefined, null, 1, 'true']) {
      assert.equal(
        optimalityLabelV2({ optimality_proven: flag }),
        'Best safe complete route found within the planning window — optimality not proven',
      );
    }
  });

  test('"impossible" only when complete_route_proven_impossible === true', () => {
    assert.match(noRouteExplanationV2({ complete_route_proven_impossible: true }), /^No complete route exists/);
    for (const flag of [false, undefined, null, 1, 'true']) {
      assert.equal(
        noRouteExplanationV2({ complete_route_proven_impossible: flag }),
        'No complete route was found within the planning window; non-existence was not proven.',
      );
    }
  });
});

describe('STM setup validation', () => {
  test('the two profiles and their exact setup strings (PROTOCOL.md section 4.1)', () => {
    assert.deepEqual(MOTION_PROFILES, ['PRECISION', 'STANDARD']);
    assert.deepEqual(expectedStmSetup('STANDARD'), {
      set_command: 'PROFILE,STANDARD',
      expected_set_reply: 'PROFILE,OK',
      verify_command: 'PROFILE,GET',
      expected_verify_reply: 'PROFILE,STANDARD',
    });
    assert.equal(expectedStmSetup('PRECISION').set_command, 'PROFILE,PRECISION');
  });

  test('accepts exactly the setup for the result\'s own profile, and for a requested one', () => {
    for (const profile of MOTION_PROFILES) {
      const result = { motion_profile: profile, stm_setup: expectedStmSetup(profile) };
      assert.deepEqual(validateStmSetup(result), expectedStmSetup(profile));
      assert.deepEqual(validateStmSetup(result, profile), expectedStmSetup(profile));
    }
  });

  test('rejects a profile other than the requested one, an unknown profile, and any altered setup', () => {
    const precision = { motion_profile: 'PRECISION', stm_setup: expectedStmSetup('PRECISION') };
    assert.throws(() => validateStmSetup(precision, 'STANDARD'), PreviewModelError);
    for (const motion_profile of ['standard', 'CUSTOM', '', null, undefined, 0]) {
      assert.throws(() => validateStmSetup({ motion_profile, stm_setup: expectedStmSetup('STANDARD') }), /unknown motion_profile/);
    }
    const good = expectedStmSetup('STANDARD');
    for (const stm_setup of [
      undefined, null, 'PROFILE,STANDARD', [], {},
      { ...good, set_command: 'PROFILE,STANDARD ' },
      { ...good, expected_set_reply: 'PROFILE,OK\n' },
      { ...good, verify_command: 'PROFILE,get' },
      { ...good, expected_verify_reply: 'PROFILE,PRECISION' },
      { ...good, extra: true },
    ]) {
      assert.throws(
        () => validateStmSetup({ motion_profile: 'STANDARD', stm_setup }, 'STANDARD'),
        (error) => error instanceof PreviewModelError && /exact STANDARD setup/.test(error.message),
      );
    }
    assert.throws(() => validateStmSetup(null), PreviewModelError);
  });
});
