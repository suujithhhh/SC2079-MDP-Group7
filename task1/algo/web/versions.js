/**
 * Planner-version indicator helpers for the display-only page.
 *
 * No DOM access and no `fetch`: pure functions, imported by `auto.js` in the
 * browser and by Node's test runner (`tests/versions.test.js`).
 *
 * The PC server runs two planners side by side: protocol v1 at `POST /plan`
 * and protocol v2 at `POST /plan/v2`. The URL the caller posts to decides
 * which one plans, not the `protocol_version` field inside the body. This page
 * only ever draws v2 plans (it reads `/state/v2`), so a request that used v1 is
 * never what is drawn here. `GET /state/versions` reports the newest plan
 * request's version, sender and mission id; rules here:
 * - a malformed reply is ignored entirely (fail closed): nothing is shown;
 * - the "last request" circles light exactly the version the server saw, and
 *   the row names the mode it asked for (B3 is the team default);
 * - the "page shows" circles always light V2, the only version drawn here;
 * - a warning appears when the newest request was V1, when it was a V2
 *   mission other than the one on screen, or when its mode was not B3.
 */

export const VERSION_POLL_MS = 1000;

const SOURCES = { rpi: 'Robot (RPi)', browser: 'A browser page' };
export const TEAM_MODE = 'B3';
const MAX_MODE_CHARS = 16;
const PATHS = { 1: '/plan', 2: '/plan/v2' };

const isFiniteNumber = (v) => typeof v === 'number' && Number.isFinite(v);

/**
 * Validates a `GET /state/versions` body. Returns `{ last: null }` when the
 * server has seen no plan request yet, `{ last: { protocolVersion, source,
 * missionId, mode, ageS } }` for a valid one (`mode` is an upper-case string,
 * or null when the server did not record one), or `null` when anything is off.
 */
export function normalizeVersions(raw) {
  if (!raw || typeof raw !== 'object' || raw.type !== 'PLAN_VERSIONS') return null;
  const last = raw.last_request;
  if (last === null) return { last: null };
  if (!last || typeof last !== 'object') return null;
  if (last.protocol_version !== 1 && last.protocol_version !== 2) return null;
  if (!Object.hasOwn(SOURCES, last.source)) return null;
  if (last.mission_id !== null && (typeof last.mission_id !== 'string' || last.mission_id === '')) return null;
  if (last.mode !== undefined && last.mode !== null
      && (typeof last.mode !== 'string' || last.mode === '' || last.mode.length > MAX_MODE_CHARS)) return null;
  if (!isFiniteNumber(last.age_s) || last.age_s < 0) return null;
  return {
    last: {
      protocolVersion: last.protocol_version,
      source: last.source,
      missionId: last.mission_id,
      mode: typeof last.mode === 'string' ? last.mode.toUpperCase() : null,
      ageS: last.age_s,
    },
  };
}

/** "just now", "12 s ago", "3 min ago". */
export function agoText(ageS) {
  if (ageS < 1.5) return 'just now';
  if (ageS < 90) return `${Math.round(ageS)} s ago`;
  return `${Math.round(ageS / 60)} min ago`;
}

/**
 * What the page draws: which circle is lit in each row, the row texts and an
 * optional warning. `versions` is a `normalizeVersions()` result or `null`
 * (nothing valid yet); `shownMissionId` is the mission on screen, or `null`.
 */
export function versionIndicator(versions, shownMissionId = null) {
  const shown = {
    active: 2,
    text: 'V2 only: this page reads /state/v2 and cannot draw V1 plans.',
  };
  const last = versions?.last ?? null;
  if (!last) {
    return {
      request: { active: null, text: versions ? 'No plan request yet.' : 'Waiting for the planner.' },
      shown,
      warning: '',
    };
  }

  const who = SOURCES[last.source];
  const mission = last.missionId ? ` · mission ${last.missionId}` : '';
  const mode = last.mode ? ` · ${last.mode}` : '';
  const request = {
    active: last.protocolVersion,
    text: `${who} · V${last.protocolVersion} (${PATHS[last.protocolVersion]})${mode} · ${agoText(last.ageS)}${mission}`,
  };

  const warnings = [];
  if (last.protocolVersion === 1) {
    warnings.push(`The last request used V1 (${PATHS[1]}). This page only shows V2 plans, `
      + 'so the route drawn here is not from that request.');
  } else if (last.missionId && shownMissionId && last.missionId !== shownMissionId) {
    warnings.push(`This page shows mission ${shownMissionId}, not the latest request (${last.missionId}).`);
  }
  if (last.mode && last.mode !== TEAM_MODE) {
    warnings.push(`The last request used mode ${last.mode}, not ${TEAM_MODE} (the team default).`);
  }
  return { request, shown, warning: warnings.join(' ') };
}
