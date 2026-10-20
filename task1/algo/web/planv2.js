/**
 * Original Roadmap Step 5 (web) -- pure-function helpers for the
 * experimental protocol-v2 planning preview.
 *
 * No DOM access, no `fetch`: every export here is a plain function of its
 * arguments, so it can be imported unchanged by both the browser
 * (`app.js`, loaded as an ES module) and Node's built-in test runner
 * (`algo/web/tests/planv2.test.js`). Keeping this split is what lets the
 * request/response/preview logic be tested without a browser.
 *
 * This module never talks to the RPi/STM execution path -- there is none
 * yet. Everything here is planning-time only: building a `/plan/v2`
 * request, and turning its response into an ordered, scrubbable preview
 * of the mission the RPi would eventually execute. See `renderAtPreviewStep`
 * in `app.js` for where a future RPi ACK/SNAP execution-progress feed
 * would plug into the same preview-step shape this module produces.
 */

export const PROTOCOL_VERSION_V2 = 2;
export const MAX_OBSTACLES_V2 = 8;

/**
 * True only for a bounded-v2 result the RPi may execute: `valid` and `complete`
 * are both exactly true and at least one action exists. Every clear failure
 * (`valid:false`, `complete:false`, empty actions) is non-executable, and a
 * result with a missing or odd flag is never treated as executable.
 */
export function isExecutableV2Result(result) {
  return Boolean(
    result
    && result.valid === true
    && result.complete === true
    && Array.isArray(result.actions)
    && result.actions.length > 0,
  );
}

// ---------------------------------------------------------------------------
// Heading conversion
// ---------------------------------------------------------------------------

const COMPASS_TO_DEG = { E: 0, N: 90, W: 180, S: 270 };

/** Convert a compass letter (the web UI's start-heading control) to the
 * degrees protocol v2 requires at `start.heading_deg`. v2 has no compass-
 * letter key; this conversion must happen client-side before the request
 * is sent, unlike v1 where the server accepts the letter directly. */
export function headingLetterToDeg(letter) {
  if (!(letter in COMPASS_TO_DEG)) {
    throw new RangeError(`unknown heading letter: ${letter}`);
  }
  return COMPASS_TO_DEG[letter];
}

/** Normalise any finite degree value to the half-open range [0, 360),
 * matching `planning_service_v2.py::_normalized_heading_deg` exactly,
 * including never emitting `-0`. */
export function normalizeHeadingDeg(deg) {
  let normalized = ((deg % 360) + 360) % 360;
  if (normalized === 0) normalized = 0; // collapse -0 to 0
  return normalized;
}

// ---------------------------------------------------------------------------
// Mission ID (browser-generated, dev/preview only)
// ---------------------------------------------------------------------------

/** Generates a development/preview `mission_id`. This is a convenience for
 * exercising `/plan/v2` from the browser only -- in the final integrated
 * system, `mission_id` is assigned once by the RPi per real mission
 * (`PROTOCOL.md` §6). Never treat a value from this function as anything
 * but a throwaway id for manual testing.
 *
 * Uses `crypto.randomUUID()` when available (requires a secure context:
 * HTTPS or `localhost`). The dev server binds `0.0.0.0` and is commonly
 * reached over a plain-HTTP LAN address, which is not a secure context in
 * most browsers, so a non-cryptographic fallback is used there -- fine for
 * a disposable preview id, never a security boundary. */
export function generateMissionId(prefix = 'web-preview', randomSource = globalThis.crypto) {
  const timePart = Date.now().toString(36);
  let randomPart;
  if (randomSource && typeof randomSource.randomUUID === 'function') {
    randomPart = randomSource.randomUUID().slice(0, 8);
  } else {
    randomPart = Math.random().toString(36).slice(2, 10);
  }
  return `${prefix}-${timePart}-${randomPart}`;
}

// ---------------------------------------------------------------------------
// Request construction
// ---------------------------------------------------------------------------

/**
 * Builds the exact protocol-v2 `PLAN_REQUEST` body from the web UI's own
 * state shape (mirrors `state.start`/`state.obstacles` in `app.js`).
 *
 * `uiState` shape: `{ mode, coordinateUnits, start: {x, y, heading},
 * obstacles: [{id, x, y, face}], allowAlternativeViewing }` -- `x`/`y` are
 * already in whatever unit `coordinateUnits` declares (grid cells or cm),
 * matching the existing v1 request builder's convention.
 */
export function buildPlanV2Request(uiState, missionId) {
  if (typeof missionId !== 'string' || missionId.length === 0) {
    throw new RangeError('missionId must be a non-empty string');
  }
  if (typeof uiState.allowAlternativeViewing !== 'boolean') {
    throw new RangeError('allowAlternativeViewing must be a boolean');
  }
  return {
    type: 'PLAN_REQUEST',
    protocol_version: PROTOCOL_VERSION_V2,
    mission_id: missionId,
    mode: uiState.mode,
    coordinate_units: uiState.coordinateUnits,
    allow_alternative_viewing: uiState.allowAlternativeViewing,
    start: {
      x: uiState.start.x,
      y: uiState.start.y,
      heading_deg: headingLetterToDeg(uiState.start.heading),
    },
    obstacles: uiState.obstacles.map((o) => ({
      id: o.id,
      x: o.x,
      y: o.y,
      face: o.face,
    })),
  };
}

// ---------------------------------------------------------------------------
// Preview-step model
// ---------------------------------------------------------------------------

export class PreviewModelError extends Error {
  constructor(message) {
    super(message);
    this.name = 'PreviewModelError';
  }
}

/** Exact equality on every serialized pose field -- mirrors the server's
 * own `Pose` dataclass equality (`planning_service_v2.py`'s
 * `waypoints[-1] != snap_pose` check) bit-for-bit. `x_cm`/`y_cm` round-trip
 * losslessly through JSON as IEEE-754 doubles, and `heading_deg` is a
 * deterministic function of the same underlying value the server compared,
 * so an exact `===` on the three serialized fields reproduces the server's
 * decision precisely. A tolerance/epsilon comparison here would silently
 * diverge from what the server actually decided and must not be used. */
export function serializedPoseEquals(a, b) {
  return a.x_cm === b.x_cm && a.y_cm === b.y_cm && a.heading_deg === b.heading_deg;
}

/**
 * Builds the ordered list of preview steps for a v2 result's `actions` and
 * `waypoints`. Every action contributes at least one step, in mission
 * order; a MOV contributes one step per waypoint in its inclusive range
 * (so a long straight/turn can be scrubbed mid-segment); a SNAP always
 * contributes exactly one step, even when it shares a physical waypoint
 * with the action before or after it -- the two are still distinct entries
 * in the returned array, distinguished by `actionIndex`, never merged.
 *
 * Never performs a pose search over the whole `waypoints` array: a SNAP's
 * own index is derived sequentially from the running cursor (the last
 * waypoint index any preceding action has claimed), exactly mirroring the
 * server's append-or-reuse construction. Throws `PreviewModelError` on any
 * range that doesn't validate against `waypoints` -- this is treated as
 * malformed response data, never silently drawn as if it were correct.
 *
 * Returns `Array<{actionIndex, actionSeq, actionType, waypointIndex}>`.
 */
export function buildPreviewSteps(actions, waypoints) {
  if (!Array.isArray(actions) || !Array.isArray(waypoints)) {
    throw new PreviewModelError('actions and waypoints must both be arrays');
  }

  const steps = [];
  let cursor = -1; // last waypoint index any action so far has claimed

  actions.forEach((action, actionIndex) => {
    if (action.type === 'MOV') {
      const start = action.waypoint_start_index;
      const end = action.waypoint_end_index;
      if (
        !Number.isInteger(start) ||
        !Number.isInteger(end) ||
        start < 0 ||
        end > waypoints.length - 1 ||
        start > end
      ) {
        throw new PreviewModelError(
          `action seq ${action.seq}: MOV waypoint range [${start}, ${end}] is invalid for ${waypoints.length} waypoint(s)`,
        );
      }
      const expectedStart = cursor === -1 ? 0 : cursor;
      if (start !== expectedStart) {
        throw new PreviewModelError(
          `action seq ${action.seq}: MOV waypoint_start_index ${start} does not continue from the previous action's last waypoint (expected ${expectedStart})`,
        );
      }
      for (let waypointIndex = start; waypointIndex <= end; waypointIndex++) {
        steps.push({ actionIndex, actionSeq: action.seq, actionType: 'MOV', waypointIndex });
      }
      cursor = end;
    } else if (action.type === 'SNAP') {
      const reused = cursor >= 0 && serializedPoseEquals(waypoints[cursor], action.pose);
      const waypointIndex = reused ? cursor : cursor + 1;
      if (waypointIndex > waypoints.length - 1) {
        throw new PreviewModelError(
          `action seq ${action.seq}: SNAP needs waypoint index ${waypointIndex}, but only ${waypoints.length} waypoint(s) were supplied`,
        );
      }
      if (!reused && !serializedPoseEquals(waypoints[waypointIndex], action.pose)) {
        throw new PreviewModelError(
          `action seq ${action.seq}: SNAP pose does not match waypoints[${waypointIndex}]; response is inconsistent`,
        );
      }
      steps.push({ actionIndex, actionSeq: action.seq, actionType: 'SNAP', waypointIndex });
      cursor = waypointIndex;
    } else {
      throw new PreviewModelError(`action seq ${action.seq}: unknown action type ${action.type}`);
    }
  });

  if (actions.length > 0 && cursor !== waypoints.length - 1) {
    throw new PreviewModelError(
      `mission actions account for waypoints up to index ${cursor}, but ${waypoints.length} waypoint(s) were supplied`,
    );
  }

  return steps;
}

/**
 * Classifies every action as `'remaining' | 'active' | 'completed'` at a
 * given position in the `previewSteps` array (not a waypoint index -- the
 * slider scrubs preview steps, never raw waypoints, precisely so a shared
 * physical waypoint between two actions never has to be split by a
 * tie-break: each action owns its own distinct range of preview-step
 * positions, so exactly one action is ever 'active' for any given
 * `previewStepIndex`).
 *
 * A non-final action becomes 'completed' only once the preview moves past
 * its last step (its boundary waypoint is shared with the next action). The
 * final action has no successor, so reaching the preview's last step is what
 * completes it: at the end of the preview nothing is left 'active' and the
 * final SNAP counts as done.
 *
 * Returns `Array<{action, actionIndex, status}>`, one entry per action in
 * `actions`, in the original mission order.
 */
export function classifyActionsAtPreviewStep(actions, previewSteps, previewStepIndex) {
  const lastStepPosition = previewSteps.length - 1;
  const bounds = new Array(actions.length).fill(null);
  previewSteps.forEach((step, stepPos) => {
    const existing = bounds[step.actionIndex];
    if (existing === null) {
      bounds[step.actionIndex] = { first: stepPos, last: stepPos };
    } else {
      existing.last = stepPos;
    }
  });

  return actions.map((action, actionIndex) => {
    const b = bounds[actionIndex];
    let status = 'remaining';
    if (b) {
      const finishesPreview = b.last === lastStepPosition && previewStepIndex >= lastStepPosition;
      if (previewStepIndex > b.last || finishesPreview) status = 'completed';
      else if (previewStepIndex >= b.first) status = 'active';
    }
    return { action, actionIndex, status };
  });
}

// ---------------------------------------------------------------------------
// /state/v2 merge guard
// ---------------------------------------------------------------------------

/**
 * Decides whether a polled `/state/v2` snapshot should update the locally
 * displayed planning status. `/state/v2` is a single, global, server-side
 * snapshot -- a second tab/window can legitimately overwrite it with an
 * unrelated mission once this tab's own request has finished (the
 * single-flight guard guarantees at most one mission is ever in flight at
 * a time, so mission-id equality alone is sufficient: two genuinely
 * concurrent missions from this tab's perspective cannot both be
 * in-flight). A snapshot whose `mission_id` doesn't match the last mission
 * this tab itself submitted is informational only and must never replace
 * the locally displayed result or move the preview cursor.
 */
export function mergeV2State(current, polled, lastSubmittedMissionId) {
  if (lastSubmittedMissionId && polled.mission_id !== lastSubmittedMissionId) {
    return current;
  }
  return { ...current, status: polled.status, missionId: polled.mission_id };
}

// ---------------------------------------------------------------------------
// Result normalization (geometry only -- schema-specific fields such as
// `actions`/`commands` are deliberately left untouched and handled per
// engine in app.js, never merged into one shape).
// ---------------------------------------------------------------------------

function headingRadToNormalizedDeg(rad) {
  return normalizeHeadingDeg((rad * 180) / Math.PI);
}

/** Converts every `{x_cm, y_cm, heading_rad}` pose in a v1 result to
 * `{x_cm, y_cm, heading_deg}`, so the shared rendering code only ever
 * handles one heading convention. Never touches `x_cm`/`y_cm`, and never
 * invents or alters any radius/geometry constant. */
export function normalizeV1Result(raw) {
  if (!raw) return raw;
  const toPose = (p) => ({ x_cm: p.x_cm, y_cm: p.y_cm, heading_deg: headingRadToNormalizedDeg(p.heading_rad) });
  return {
    ...raw,
    waypoints: (raw.waypoints || []).map(toPose),
    target_poses: (raw.target_poses || []).map(toPose),
  };
}

/** Pose-bearing fields of a v2 action: a MOV's `start_pose`/`end_pose` and a
 * SNAP's `pose`. */
const ACTION_POSE_FIELDS = ['start_pose', 'end_pose', 'pose'];

/** v2 poses are already `heading_deg`; this only re-validates the range so
 * downstream code can assume `[0, 360)` unconditionally, same as v1 after
 * normalization. Never touches `x_cm`/`y_cm`.
 *
 * EVERY pose-bearing field goes through the same normalization: `waypoints`,
 * `target_poses`, `final_pose` and each action's `start_pose`/`end_pose`/
 * `pose`. They must agree, because `buildPreviewSteps()` compares a SNAP's
 * pose with a waypoint using strict equality. A raw heading such as
 * 2.5e-14 deg collapses to 0 when normalized; normalizing waypoints alone
 * made an identical SNAP pose look different and a valid response was
 * rejected. The strict comparison itself is unchanged. */
export function normalizeV2Result(raw) {
  if (!raw) return raw;
  const toPose = (p) => ({ x_cm: p.x_cm, y_cm: p.y_cm, heading_deg: normalizeHeadingDeg(p.heading_deg) });
  const normalizeAction = (action) => {
    if (!action || typeof action !== 'object') return action;
    const normalized = { ...action };
    for (const field of ACTION_POSE_FIELDS) {
      if (action[field]) normalized[field] = toPose(action[field]);
    }
    return normalized;
  };
  return {
    ...raw,
    waypoints: (raw.waypoints || []).map(toPose),
    target_poses: (raw.target_poses || []).map(toPose),
    final_pose: raw.final_pose ? toPose(raw.final_pose) : raw.final_pose,
    actions: Array.isArray(raw.actions) ? raw.actions.map(normalizeAction) : raw.actions,
  };
}

// ---------------------------------------------------------------------------
// Shared fail-closed result gate (manual page and automatic page)
// ---------------------------------------------------------------------------

/**
 * The one gate every displayed v2 result goes through, on both pages. Throws
 * `PreviewModelError` unless `rawResult` is a `PLAN_RESULT` echoing
 * `missionId`; normalizes every pose; and, for an executable result
 * (`isExecutableV2Result`), runs the strict action/waypoint validation of
 * `buildPreviewSteps()`, which also throws `PreviewModelError`. Any other
 * exception (e.g. a malformed pose) propagates unchanged; callers treat every
 * throw as a failure. A non-executable result passes with no preview steps.
 *
 * Returns `{ result, previewSteps }` (`result` normalized).
 */
export function validateV2Result(rawResult, missionId) {
  if (!rawResult || rawResult.type !== 'PLAN_RESULT') {
    throw new PreviewModelError('response is not a PLAN_RESULT');
  }
  if (rawResult.mission_id !== missionId) {
    throw new PreviewModelError('response mission_id does not match the request');
  }
  const result = normalizeV2Result(rawResult);
  const previewSteps = isExecutableV2Result(result) ? buildPreviewSteps(result.actions, result.waypoints) : [];
  return { result, previewSteps };
}

/** The two STM motion profiles a request may name (`PROTOCOL.md` §4.1). */
export const MOTION_PROFILES = ['PRECISION', 'STANDARD'];

/** The exact `stm_setup` `PROTOCOL.md` §4.1 requires for `profile`. Spelled
 * out here independently of `planning_service_v2.py`, on purpose: this is a
 * black-box check of the server's reply, not a second consumer of its helper. */
export function expectedStmSetup(profile) {
  return {
    set_command: `PROFILE,${profile}`,
    expected_set_reply: 'PROFILE,OK',
    verify_command: 'PROFILE,GET',
    expected_verify_reply: `PROFILE,${profile}`,
  };
}

/**
 * Fail-closed check of an executable result's STM setup. Throws
 * `PreviewModelError` unless `motion_profile` is one of the two known profiles
 * (and equals `expectedProfile` when one is given, i.e. the profile this page
 * itself requested) and `stm_setup` is exactly the four strings required for
 * that profile, with no missing, extra or altered field. Returns the validated
 * setup. The RPi, not this page, sends these commands to the STM.
 */
export function validateStmSetup(result, expectedProfile = null) {
  const profile = result?.motion_profile;
  if (!MOTION_PROFILES.includes(profile)) {
    throw new PreviewModelError(`unknown motion_profile ${JSON.stringify(profile)}`);
  }
  if (expectedProfile !== null && profile !== expectedProfile) {
    throw new PreviewModelError(`motion_profile ${profile} does not match the requested ${expectedProfile}`);
  }
  const expected = expectedStmSetup(profile);
  const got = result.stm_setup;
  const isObject = got !== null && typeof got === 'object' && !Array.isArray(got);
  const keys = isObject ? Object.keys(got) : [];
  const exact = isObject
    && keys.length === Object.keys(expected).length
    && Object.keys(expected).every((key) => got[key] === expected[key]);
  if (!exact) throw new PreviewModelError(`stm_setup does not match the exact ${profile} setup`);
  return expected;
}

/** Route-quality wording driven only by the proof flag: "optimal" is claimed
 * only when the backend reports `optimality_proven === true`. */
export function optimalityLabelV2(result) {
  return result?.optimality_proven === true
    ? 'Optimal route (proven by search)'
    : 'Best safe complete route found within the planning window — optimality not proven';
}

/** Explains a non-executable result. Impossibility is claimed only when the
 * backend reports `complete_route_proven_impossible === true`. */
export function noRouteExplanationV2(result) {
  return result?.complete_route_proven_impossible === true
    ? 'No complete route exists for this map (proven by search under the supplied viewing candidates).'
    : 'No complete route was found within the planning window; non-existence was not proven.';
}
