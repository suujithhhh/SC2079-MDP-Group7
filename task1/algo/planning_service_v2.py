"""Original Roadmap Step 5 -- protocol-v2 HTTP planning service (experimental).

Exposes the bounded prefix-tree executable planner (`bounded_prefix_planning.py`)
over HTTP as a separate schema (`protocol_version: 2`), without touching
`planning_service.py` (protocol v1). Complete routes still use the established
state-lattice -> compression -> STM-command pipeline. Direct executable-route
imports remain only as server-owned serializer test seams; the live endpoint
always uses the bounded executor.

Experimental scope -- read before treating this as production-ready
----------------------------------------------------------------------
This endpoint accepts **1-8 obstacles**. The bounded executor targets planning
by R0+67s, kills the contained worker tree at R0+81s, withholds anything
assembled after R0+88s, and reserves the R0+90s HTTP ceiling. It returns only
a complete independently verified route or a clear non-executable failure;
partial geometry never crosses this boundary. Bounded validation passed the
tested 6-8 obstacle cases, containment and soak cases, but the original 4-8
adoption gate still fails because `A_spread_04` found no route within the
planning budget. This remains an experimental integration, not a claim that
every Task 1 layout succeeds.

Request: degrees, not radians
----------------------------------
Unlike protocol v1 (which accepts `start.heading` as a compass letter or a
raw radians float), protocol v2 requires `start.heading_deg`: a numeric,
finite, non-boolean value in **degrees**. A plain numeric `start.heading`
value is never accepted as radians here -- there is no such key in this
schema, so degrees and radians are never silently mixed. Degrees are
converted to internal radians, then normalised through the same
`motion_primitives.normalize_heading()` every other pipeline module already
uses, at this single parsing boundary only -- `Pose.heading` stays radians
everywhere else in Python, exactly as elsewhere in this subsystem.

Response: every pose uses heading_deg, normalised to [0.0, 360.0)
-----------------------------------------------------------------------
Every pose object emitted by this module (`target_poses`, `final_pose`,
every action's `start_pose`/`end_pose`, every SNAP's `pose`, every entry of
`waypoints`) is `{"x_cm", "y_cm", "heading_deg"}` -- never `heading_rad`.
`heading_deg` is always normalised to the half-open range `[0.0, 360.0)`
(so East=0, North=90, West=180, South=270), and `-0.0` is never emitted --
it is serialised as `0.0`. This differs from v1's `heading_rad` convention
by design (`PROTOCOL.md`'s own worked v2 example already uses degrees).

Required mission_id, never generated here
----------------------------------------------
`PROTOCOL.md` assigns `mission_id` to the RPi, once per plan request; the
planner only echoes it back unchanged. Since no RPi exists yet, this module
requires the caller to supply a non-empty string `mission_id` and never
invents one. Missing, empty, or non-string `mission_id` is an ordinary
`RequestErrorV2` (HTTP 400) -- no UUID format or length requirement is
imposed, since nothing in `PROTOCOL.md` or elsewhere requires one.

No timing fields
--------------------
No calibrated timing model exists for the executable pipeline. This module
never emits `estimated_time_s`/`estimated_time_status`, and `total_cost`
(purely geometric length plus `reverse_penalty`) is never described as, or
convertible to, time.

Selectable STM motion profile (setup metadata, not a path action)
-----------------------------------------------------------------------
A request may carry an optional `motion_profile`: exactly `"PRECISION"` or
`"STANDARD"` (no case folding). Absent resolves to `"PRECISION"`, the team's
default (also the STM's own power-on default; the RPi's setup command still
sets and verifies whichever profile is used). Any
other value -- lowercase, unknown names, `"CUSTOM"` (the STM can report it
but has no SET form for it), the empty string, `null`, numbers, booleans,
lists, objects -- is a `RequestErrorV2` (`PLAN_ERROR`). A caller may name
`"STANDARD"` explicitly; the backend never picks a profile from the route,
mode or distance.

Every `PLAN_RESULT` (complete, partial and no-route alike) echoes the
resolved `motion_profile` and carries a top-level `stm_setup` object built
by `_stm_setup_v2()`: the exact strings the RPi must send and expect
(`PROTOCOL.md` §4.1/§6) -- no newline; the RPi appends it. `stm_setup` is
deliberately *not* an entry of `actions`: no `seq`, pose or waypoint range.
`PLAN_ERROR` carries neither field (no profile was resolved).

The profile is never passed to the route planner and touches nothing the
planner computes: obstacle order, geometry, the four turn radii, collision
checking, `MOV` values, estimated poses, waypoint ranges and `seq` numbers
are identical for both profiles. On the STM, applying a profile restores its
complete set of profile-owned settings, but today PRECISION and STANDARD
differ only in three straight-motion values (SPCTRL_TARGET_SPEED,
SPPROF_DECEL_START_COUNTS, STOPCTRL_BRAKE_ENTRY_SPEED; `PROTOCOL.md` §4.1).
PRESTEER, TURNSPEED, ACKERMANN, TURNBRAKE, the servo endpoints and the turn
radii are not profile-owned. No timing field is added and nothing here claims
a mission-time difference between profiles -- no calibrated PRECISION vs
STANDARD timing model exists.

SNAP poses are estimated, not ideal
----------------------------------------
A SNAP action's `pose` is always the corresponding leg's estimated
`command_plan.final_pose` -- the modelled pose the search actually reached
-- never the leg's ideal `target_pose`. SNAP entries carry no `stm_command`,
`value`, `unit`, or waypoint-index fields: they are not STM commands
(`PROTOCOL.md` §6 item 5).

Waypoint index mapping
---------------------------
See `_build_actions_and_waypoints()` for the exact construction rule: one
global, deduplicated `waypoints` array is built by walking every leg's
commands in order, and every MOV action records an inclusive
`waypoint_start_index`/`waypoint_end_index` range into that one array --
including its shared starting pose, not just newly appended samples. This
gives an unambiguous mapping even when the route revisits an earlier
physical pose (deduplication only ever happens at the immediate boundary
between two consecutive commands/legs, never by searching the array for an
existing occurrence of the same pose).
"""

import math
import time
from typing import Any, Callable, Optional

from bounded_prefix_planning import (
    BoundedPlanOutcome,
    BoundedPrefixPlanner,
    PlanInput,
)
from config import DEFAULT_PLANNER_CONFIG, GRID_CELL_CM, PlannerConfig
from executable_route_optimizer import (
    ExecutableRouteResult,
    find_executable_b2_route,
    optimise_executable_b3_route,
)
from models import Obstacle, Pose
from motion_primitives import normalize_heading

PROTOCOL_VERSION = 2
FACES = ("N", "E", "S", "W")

# Sentinel distinguishing "key absent from the request" (use the default)
# from "key present with an explicit JSON null" (a strict-boolean field
# must reject this, not silently treat it as absent).
_MISSING = object()

# The adopted bounded prefix-search layer is defined and validated for at
# most eight obstacles. Protocol v1 deliberately keeps its independent
# five-obstacle limit.
MAX_OBSTACLES = 8

# STM motion profiles selectable per mission (PROTOCOL.md §4.1). A request that
# names none resolves to PRECISION (team decision; the STM itself also boots in
# PRECISION, and the RPi's PROFILE setup command sets and verifies it).
MOTION_PROFILES = ("PRECISION", "STANDARD")
DEFAULT_MOTION_PROFILE = "PRECISION"

RouteFinder = Callable[..., ExecutableRouteResult]

# One long-lived executor per service process. Its health is intentionally
# sticky: if fail-closed cleanup cannot be verified, later requests keep
# returning a clear executor failure until the service is restarted.
_DEFAULT_BOUNDED_PLANNER = BoundedPrefixPlanner()


class RequestErrorV2(ValueError):
    """Raised on a malformed protocol-v2 request. Never raised for an
    ordinary no-route/partial-route planning outcome -- those are reported
    through `ExecutableRouteResult.is_valid`/`is_complete`, not as request
    errors."""


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RequestErrorV2(f"{field} must be a number")
    if not math.isfinite(value):
        raise RequestErrorV2(f"{field} must be finite")
    return float(value)


def _coordinate(value: Any, field: str, units: str) -> float:
    coordinate = _number(value, field)
    return coordinate * GRID_CELL_CM if units == "grid" else coordinate


def _heading_rad_from_deg(value: Any, field: str) -> float:
    """Convert a required `*_deg` request field to internal radians, the
    only place in this module degrees become radians, then normalise
    through the same convention every other pipeline module uses."""
    degrees = _number(value, field)
    return normalize_heading(math.radians(degrees))


def _bool(value: Any, field: str, default: bool) -> bool:
    """Strict optional boolean: absent (`_MISSING`) uses `default`; present
    with any non-boolean value, including an explicit JSON `null`, is
    rejected rather than silently treated as absent."""
    if value is _MISSING:
        return default
    if isinstance(value, bool):
        return value
    raise RequestErrorV2(f"{field} must be a boolean")


def _mission_id(payload: dict) -> str:
    value = payload.get("mission_id")
    if not isinstance(value, str) or not value:
        raise RequestErrorV2("mission_id must be a non-empty string")
    return value


def _motion_profile(value: Any) -> str:
    """Strict optional enum: absent (`_MISSING`) resolves to
    `DEFAULT_MOTION_PROFILE`; present, it must be a `str` exactly equal to
    one of `MOTION_PROFILES` -- no case folding, and an explicit JSON `null`
    is rejected rather than treated as absent."""
    if value is _MISSING:
        return DEFAULT_MOTION_PROFILE
    if isinstance(value, str) and value in MOTION_PROFILES:
        return value
    raise RequestErrorV2("motion_profile must be PRECISION or STANDARD")


def _stm_setup_v2(motion_profile: str) -> dict[str, str]:
    """Exact STM profile setup/verification strings for one resolved
    profile (`PROTOCOL.md` §4.1). Pure and independent of the route, so it
    is identical for every result of a given profile."""
    return {
        "set_command": f"PROFILE,{motion_profile}",
        "expected_set_reply": "PROFILE,OK",
        "verify_command": "PROFILE,GET",
        "expected_verify_reply": f"PROFILE,{motion_profile}",
    }


def parse_plan_request_v2(
    payload: Any,
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
) -> tuple[str, str, Pose, list[Obstacle], bool, str]:
    """Validate a protocol-v2 `PLAN_REQUEST` payload.

    Returns `(mission_id, mode, start, obstacles, allow_alternative_viewing,
    motion_profile)`.
    Raises `RequestErrorV2` on any malformed field. Never mutates `payload`.
    """
    if not isinstance(payload, dict):
        raise RequestErrorV2("request must be a JSON object")
    if payload.get("type") != "PLAN_REQUEST":
        raise RequestErrorV2("type must be PLAN_REQUEST")
    version = payload.get("protocol_version")
    if version != PROTOCOL_VERSION:
        raise RequestErrorV2(f"unsupported protocol_version: {version!r}")

    mission_id = _mission_id(payload)

    if "mode" not in payload:
        raise RequestErrorV2("mode is required")
    mode = payload.get("mode")
    if not isinstance(mode, str) or mode.upper() not in ("B2", "B3"):
        raise RequestErrorV2("mode must be B2 or B3")
    mode = mode.upper()

    if "coordinate_units" not in payload:
        raise RequestErrorV2("coordinate_units is required")
    units = payload.get("coordinate_units")
    if units not in ("cm", "grid"):
        raise RequestErrorV2("coordinate_units must be cm or grid")

    allow_alternative_viewing = _bool(
        payload.get("allow_alternative_viewing", _MISSING),
        "allow_alternative_viewing", False,
    )
    motion_profile = _motion_profile(payload.get("motion_profile", _MISSING))

    start_data = payload.get("start")
    if not isinstance(start_data, dict):
        raise RequestErrorV2("start must be an object")
    if "heading_deg" not in start_data:
        raise RequestErrorV2("start.heading_deg is required")
    start = Pose(
        _coordinate(start_data.get("x"), "start.x", units),
        _coordinate(start_data.get("y"), "start.y", units),
        _heading_rad_from_deg(start_data.get("heading_deg"), "start.heading_deg"),
    )

    obstacle_data = payload.get("obstacles")
    if not isinstance(obstacle_data, list) or not 1 <= len(obstacle_data) <= MAX_OBSTACLES:
        raise RequestErrorV2(
            f"obstacles must contain between 1 and {MAX_OBSTACLES} items"
        )
    obstacles: list[Obstacle] = []
    identifiers: set[int] = set()
    for index, item in enumerate(obstacle_data):
        if not isinstance(item, dict):
            raise RequestErrorV2(f"obstacles[{index}] must be an object")
        identifier = item.get("id")
        if isinstance(identifier, bool) or not isinstance(identifier, int) or identifier <= 0:
            raise RequestErrorV2(f"obstacles[{index}].id must be a positive integer")
        if identifier in identifiers:
            raise RequestErrorV2(f"duplicate obstacle id: {identifier}")
        face = item.get("face")
        if not isinstance(face, str) or face.upper() not in FACES:
            raise RequestErrorV2(f"obstacles[{index}].face must be N, E, S or W")
        obstacle = Obstacle(
            identifier,
            _coordinate(item.get("x"), f"obstacles[{index}].x", units),
            _coordinate(item.get("y"), f"obstacles[{index}].y", units),
            face.upper(),
        )
        if not (
            0 <= obstacle.x <= config.arena_cm - config.obstacle_cm
            and 0 <= obstacle.y <= config.arena_cm - config.obstacle_cm
        ):
            raise RequestErrorV2(f"obstacle {identifier} lies outside the arena")
        identifiers.add(identifier)
        obstacles.append(obstacle)

    return mission_id, mode, start, obstacles, allow_alternative_viewing, motion_profile


def _normalized_heading_deg(heading_rad: float) -> float:
    """Degrees in the half-open range [0.0, 360.0); never -0.0."""
    degrees = math.degrees(heading_rad) % 360.0
    if degrees == 0.0:
        degrees = 0.0
    return degrees


def _pose_json(pose: Pose) -> dict:
    return {
        "x_cm": pose.x,
        "y_cm": pose.y,
        "heading_deg": _normalized_heading_deg(pose.heading),
    }


def _build_actions_and_waypoints(result: ExecutableRouteResult) -> tuple[list, list]:
    """Build the `actions` list and one global, deduplicated `waypoints`
    array from `result.legs`.

    Every MOV action's `waypoint_start_index`/`waypoint_end_index` is an
    inclusive range into `waypoints`, covering its *entire* swept path,
    including a shared boundary pose it did not itself append (that pose
    was already appended by the previous command or leg's SNAP). SNAP
    actions never carry these fields ("no swept path"), and their `pose` is
    the leg's estimated final pose, added to `waypoints` only if it is not
    already the array's last entry (deduplication only at the immediate
    consecutive boundary -- never by searching for an earlier occurrence,
    so a route that revisits a physical pose still gets two distinct
    entries and two distinct index ranges for it).
    """
    waypoints: list[Pose] = []
    actions: list[dict] = []
    seq = 0

    for leg in result.legs:
        for command in leg.command_plan.commands:
            if waypoints:
                start_index = len(waypoints) - 1
                waypoints.extend(command.samples[1:])
            else:
                start_index = 0
                waypoints.extend(command.samples)
            end_index = len(waypoints) - 1
            actions.append({
                "seq": seq,
                "type": "MOV",
                "dir": command.direction,
                "value": command.value,
                "unit": command.unit,
                "stm_command": command.stm_command,
                "start_pose": _pose_json(command.start_pose),
                "end_pose": _pose_json(command.end_pose),
                "waypoint_start_index": start_index,
                "waypoint_end_index": end_index,
            })
            seq += 1

        snap_pose = leg.command_plan.final_pose
        if not waypoints or waypoints[-1] != snap_pose:
            waypoints.append(snap_pose)
        actions.append({
            "seq": seq,
            "type": "SNAP",
            "obstacle_id": leg.obstacle_id,
            "pose": _pose_json(snap_pose),
        })
        seq += 1

    return actions, [_pose_json(pose) for pose in waypoints]


def _order_counts(result: ExecutableRouteResult) -> dict:
    return {
        "orders_considered": result.orders_considered,
        "valid_orders": result.valid_orders,
        "rejected_orders": result.rejected_orders,
        "pruned_orders": result.pruned_orders,
    }


def _result_payload_v2(
    mission_id: str, mode: str, result: ExecutableRouteResult, config: PlannerConfig,
    motion_profile: str,
) -> dict[str, Any]:
    if not result.is_valid:
        return {
            "type": "PLAN_RESULT",
            "protocol_version": PROTOCOL_VERSION,
            "mission_id": mission_id,
            "mode": mode,
            "motion_profile": motion_profile,
            "stm_setup": _stm_setup_v2(motion_profile),
            "valid": False,
            "error": result.failure_reason or "No complete executable route visits all obstacles.",
            **_order_counts(result),
        }

    # `result.failure_reason` is None for a fully complete route and a
    # descriptive string for a partial one (is_valid=True, is_complete=False)
    # -- passed through verbatim, never forced to None.

    actions, waypoints = _build_actions_and_waypoints(result)
    return {
        "type": "PLAN_RESULT",
        "protocol_version": PROTOCOL_VERSION,
        "mission_id": mission_id,
        "motion_model": "stm-4radius",
        "turn_radii_cm": {
            "FL": config.turn_radius_fl_cm,
            "FR": config.turn_radius_fr_cm,
            "BL": config.turn_radius_bl_cm,
            "BR": config.turn_radius_br_cm,
        },
        "mode": mode,
        "motion_profile": motion_profile,
        "stm_setup": _stm_setup_v2(motion_profile),
        "valid": True,
        "complete": result.is_complete,
        "final_pose": _pose_json(result.final_pose),
        "obstacle_order": list(result.obstacle_order),
        "skipped_obstacle_ids": list(result.skipped_obstacle_ids),
        "target_poses": [_pose_json(pose) for pose in result.target_poses],
        "actions": actions,
        "waypoints": waypoints,
        "total_distance_cm": result.total_distance_cm,
        "total_cost": result.total_cost,
        **_order_counts(result),
        "failure_reason": result.failure_reason,
    }


def _bounded_facts(outcome: BoundedPlanOutcome) -> dict[str, Any]:
    """Public, non-timing facts from one bounded execution.

    Cleanup and deadline offsets remain server diagnostics.  The RPi only
    needs to know whether a complete route is executable and, when it is not,
    why planning ended.  In particular, no diagnostic/partial geometry ever
    crosses this boundary.
    """
    return {
        "termination": outcome.termination,
        "delivery": outcome.delivery,
        "route_source": outcome.route_source,
        "incumbents_received": outcome.incumbents_received,
        "optimality_proven": outcome.optimality_proven,
        "complete_route_proven_impossible": outcome.complete_route_proven_impossible,
        "search_space_exhausted": outcome.search_space_exhausted,
        "message": outcome.execution_message,
    }


def _bounded_result_payload_v2(
    mission_id: str,
    mode: str,
    outcome: BoundedPlanOutcome,
    obstacles: list[Obstacle],
    config: PlannerConfig,
    motion_profile: str,
) -> dict[str, Any]:
    """Serialize the fail-closed bounded outcome into protocol v2.

    `valid` means there is a complete, independently verified, on-time route
    safe to execute.  Every other outcome is a normal HTTP-200 PLAN_RESULT
    with empty executable geometry and a clear failure message.
    """
    common = {
        "type": "PLAN_RESULT",
        "protocol_version": PROTOCOL_VERSION,
        "mission_id": mission_id,
        "motion_model": "stm-4radius",
        "turn_radii_cm": {
            "FL": config.turn_radius_fl_cm,
            "FR": config.turn_radius_fr_cm,
            "BL": config.turn_radius_bl_cm,
            "BR": config.turn_radius_br_cm,
        },
        "mode": mode,
        "motion_profile": motion_profile,
        "stm_setup": _stm_setup_v2(motion_profile),
        **_bounded_facts(outcome),
    }

    if not outcome.deliverable or outcome.route is None:
        return {
            **common,
            "valid": False,
            "complete": False,
            "final_pose": None,
            "obstacle_order": [],
            "skipped_obstacle_ids": [obstacle.obstacle_id for obstacle in obstacles],
            "target_poses": [],
            "actions": [],
            "waypoints": [],
            "total_distance_cm": 0.0,
            "total_cost": 0.0,
            "failure_reason": outcome.execution_message,
            "error": outcome.execution_message,
        }

    route = outcome.route
    actions, waypoints = _build_actions_and_waypoints(route)
    return {
        **common,
        "valid": True,
        "complete": True,
        "final_pose": _pose_json(route.final_pose),
        "obstacle_order": list(route.obstacle_order),
        "skipped_obstacle_ids": [],
        "target_poses": [_pose_json(pose) for pose in route.target_poses],
        "actions": actions,
        "waypoints": waypoints,
        "total_distance_cm": route.total_distance_cm,
        "total_cost": route.total_cost,
        "failure_reason": None,
    }


def plan_request_v2(
    payload: Any,
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
    b2_planner: Optional[RouteFinder] = None,
    b3_planner: Optional[RouteFinder] = None,
    *,
    bounded_planner: Optional[BoundedPrefixPlanner] = None,
    accepted_at: Optional[float] = None,
) -> dict[str, Any]:
    """Validate and plan one protocol-v2 request.

    Catches only `RequestErrorV2` (malformed request -> `PLAN_ERROR` dict).
    Every internal-invariant exception the executable pipeline can raise
    (`ExecutableRouteInvariantError`, `InvalidExecutableRouteInputError`,
    `InvalidLatticePlanError`, `CompressionInvariantError`,
    `STMCommandGenerationError`) is deliberately left to propagate --
    those represent a genuine pipeline bug, never an ordinary "no route"
    outcome, and must reach the caller's own error handling unchanged.
    """
    # R0 is captured before request validation so parsing, worker start-up,
    # search, containment, verification and response assembly all consume the
    # same server-owned budget.  `accepted_at` is an injection seam for tests;
    # no request field can influence it.
    if accepted_at is None:
        accepted_at = time.monotonic()

    try:
        (
            mission_id, mode, start, obstacles, allow_alternative_viewing, motion_profile,
        ) = parse_plan_request_v2(payload, config)
    except RequestErrorV2 as error:
        return {
            "type": "PLAN_ERROR",
            "protocol_version": PROTOCOL_VERSION,
            "valid": False,
            "error": str(error),
        }

    # Preserve the established direct-planner seams for focused serializer
    # tests only.  The live endpoint supplies neither and therefore always
    # uses the bounded prefix-search executor below.
    if b2_planner is not None or b3_planner is not None:
        planner = b2_planner if mode == "B2" else b3_planner
        if planner is None:
            planner = find_executable_b2_route if mode == "B2" else optimise_executable_b3_route
        result = planner(
            start, obstacles, config, allow_alternative_viewing=allow_alternative_viewing
        )
        return _result_payload_v2(mission_id, mode, result, config, motion_profile)

    executor = bounded_planner
    if executor is None:
        executor = (_DEFAULT_BOUNDED_PLANNER if config is DEFAULT_PLANNER_CONFIG
                    else BoundedPrefixPlanner(config=config))
    outcome = executor.plan(
        PlanInput(start, tuple(obstacles), mode, allow_alternative_viewing),
        accepted_at=accepted_at,
    )
    return _bounded_result_payload_v2(
        mission_id, mode, outcome, obstacles, config, motion_profile,
    )
