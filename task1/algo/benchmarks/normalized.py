"""Benchmark-only normalized route record (measurement tooling only).

`route_replay` and the comparisons in `report` work on this plain, JSON-
serialisable dict, never on a result object, so they cannot come to depend on
`ExecutableRouteResult` or `PrefixSearchResult` by accident. Each result type
has one mapping below. The original raw metadata is kept alongside, never
replaced.

    {
      "search":              "ero_b2" | "ero_b3" | "prefix_b2" | "prefix_b3",
      "alternatives":        bool,
      "start_pose":          [x, y, heading],
      "route_kind":          "complete" | "partial" | "none",
      "obstacle_order":      [id, ...],           # visited, in order
      "skipped_obstacle_ids": [id, ...],
      "candidate_indices":   [int, ...],          # index into the safety-filtered candidates
      "legs": [{"obstacle_id", "target_pose", "final_pose",
                "total_cost", "total_distance_cm",
                "commands": [{"direction", "value", "unit", "stm_command",
                              "start_pose", "end_pose", "length_cm"}]}],
      "total_cost", "total_distance_cm": float,
      "final_pose":          [x, y, heading] | null,
      "hash":                sha256 hex
    }

A leg's start pose is not stored: it is the route start for leg 0 and the
previous leg's `final_pose` afterwards, which is exactly what the replay
re-derives. Floats round-trip exactly through JSON.

`hash` digests the same fields as `bench_executable_planner.route_hash()`
(validity, completeness, order, skipped ids, `repr(cost)`, the exact `MOV`
strings and the final pose), so an old-search route and a prefix-search route
with the same content hash identically.
"""

import dataclasses
import hashlib
import json
import math

from config import DEFAULT_PLANNER_CONFIG
from executable_route_optimizer import _candidate_targets

SEARCHES = ("ero_b2", "ero_b3", "prefix_b2", "prefix_b3")
ROUTE_KINDS = ("complete", "partial", "none")


def digest(payload) -> str:
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def json_safe(value):
    """Make a value strictly JSON-serialisable (no NaN/Infinity, no tuples)."""
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def pose_list(pose):
    return None if pose is None else [pose.x, pose.y, pose.heading]


def _command_dict(command) -> dict:
    return {
        "direction": command.direction, "value": command.value, "unit": command.unit,
        "stm_command": command.stm_command, "start_pose": pose_list(command.start_pose),
        "end_pose": pose_list(command.end_pose), "length_cm": command.length_cm,
    }


def _leg_dict(leg) -> dict:
    plan = leg.command_plan
    return {
        "obstacle_id": leg.obstacle_id, "target_pose": pose_list(leg.target_pose),
        "final_pose": pose_list(plan.final_pose), "total_cost": plan.total_cost,
        "total_distance_cm": plan.total_distance_cm,
        "commands": [_command_dict(c) for c in plan.commands],
    }


def route_hash(kind, order, skipped, total_cost, legs, final_pose) -> str:
    return digest({
        "valid": kind != "none", "complete": kind == "complete",
        "order": list(order), "skipped": list(skipped), "cost": repr(total_cost),
        "commands": [[c["stm_command"] for c in leg["commands"]] for leg in legs],
        "final_pose": None if final_pose is None else [repr(v) for v in final_pose],
    })


def _record(search, alternatives, start_pose, kind, order, skipped, indices, legs,
            total_cost, total_distance_cm, final_pose) -> dict:
    return {
        "search": search, "alternatives": bool(alternatives), "start_pose": pose_list(start_pose),
        "route_kind": kind, "obstacle_order": list(order), "skipped_obstacle_ids": list(skipped),
        "candidate_indices": list(indices), "legs": legs, "total_cost": total_cost,
        "total_distance_cm": total_distance_cm, "final_pose": final_pose,
        "hash": route_hash(kind, order, skipped, total_cost, legs, final_pose),
    }


def _none_record(search, alternatives, start_pose) -> dict:
    return _record(search, alternatives, start_pose, "none", (), (), (), [], 0.0, 0.0, None)


def normalize_ero_result(result, *, search, start_pose, obstacles, alternatives,
                         config=DEFAULT_PLANNER_CONFIG) -> dict:
    """`ExecutableRouteResult` -> normalized record. `candidate_indices` are
    DERIVED (ero stores only the target pose): the position of each leg's
    target in the safety-filtered candidate tuple, or -1 if it is absent
    (which `route_replay` reports)."""
    if search not in ("ero_b2", "ero_b3"):
        raise ValueError(f"search must be ero_b2 or ero_b3, got {search!r}")
    if not result.is_valid:
        return _none_record(search, alternatives, start_pose)
    obstacles = tuple(obstacles)
    by_id = {o.obstacle_id: o for o in obstacles}
    candidates = {}
    indices = []
    for leg in result.legs:
        if leg.obstacle_id not in candidates:
            obstacle = by_id.get(leg.obstacle_id)
            candidates[leg.obstacle_id] = (
                () if obstacle is None else _candidate_targets(obstacle, obstacles, config, alternatives))
        found = candidates[leg.obstacle_id]
        indices.append(found.index(leg.target_pose) if leg.target_pose in found else -1)
    kind = "complete" if result.is_complete else "partial"
    return _record(
        search, alternatives, start_pose, kind, result.obstacle_order, result.skipped_obstacle_ids,
        indices, [_leg_dict(leg) for leg in result.legs], result.total_cost,
        result.total_distance_cm, pose_list(result.final_pose),
    )


def normalize_prefix_result(result, *, search, start_pose, alternatives) -> dict:
    """`PrefixSearchResult` -> normalized record of the route it returned:
    `complete_route`, else `best_partial_route`, else none."""
    if search not in ("prefix_b2", "prefix_b3"):
        raise ValueError(f"search must be prefix_b2 or prefix_b3, got {search!r}")
    if result.complete_route is not None:
        return normalize_prefix_route(result.complete_route, "complete", search=search,
                                      start_pose=start_pose, alternatives=alternatives)
    if result.best_partial_route is not None:
        return normalize_prefix_route(result.best_partial_route, "partial", search=search,
                                      start_pose=start_pose, alternatives=alternatives)
    return _none_record(search, alternatives, start_pose)


def normalize_prefix_route(route, kind, *, search, start_pose, alternatives) -> dict:
    """A `PrefixRoute` (for example B3's `b2_route`) -> normalized record;
    `candidate_indices` are copied from the route."""
    if route is None:
        return _none_record(search, alternatives, start_pose)
    return _record(
        search, alternatives, start_pose, kind, route.obstacle_order, route.skipped_obstacle_ids,
        route.candidate_indices, [_leg_dict(leg) for leg in route.legs], route.total_cost,
        route.total_distance_cm, pose_list(route.final_pose),
    )


def ero_raw(result) -> dict:
    return json_safe({
        "is_valid": result.is_valid, "is_complete": result.is_complete,
        "orders_considered": result.orders_considered, "valid_orders": result.valid_orders,
        "rejected_orders": result.rejected_orders, "pruned_orders": result.pruned_orders,
        "failure_reason": result.failure_reason,
    })


def prefix_raw(result) -> dict:
    """Every scalar field of a `PrefixSearchResult` plus its work counters."""
    return json_safe({
        "mode": result.mode, "route_status": result.route_status, "stop_reason": result.stop_reason,
        "work_limit_hit": result.work_limit_hit,
        "search_space_exhausted": result.search_space_exhausted,
        "complete_route_proven_impossible": result.complete_route_proven_impossible,
        "optimality_proven": result.optimality_proven,
        "empty_candidate_obstacle_ids": result.empty_candidate_obstacle_ids,
        "filtered_primary_obstacle_ids": result.filtered_primary_obstacle_ids,
        "unresolved_branch_count": result.unresolved_branch_count,
        "min_unresolved_lower_bound": result.min_unresolved_lower_bound,
        "unresolved_bound_known": result.unresolved_bound_known,
        "lower_bound_certified": result.lower_bound_certified,
        "bound_violation_count": result.bound_violation_count,
        "bound_pruning_active": result.bound_pruning_active,
        "root_lower_bound": result.root_lower_bound, "message": result.message,
        "elapsed_s": result.elapsed_s,
        "work": dataclasses.asdict(result.work),
        "b2_work": None if result.b2_work is None else dataclasses.asdict(result.b2_work),
    })
