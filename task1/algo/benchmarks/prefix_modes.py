"""Worker side of the prefix-search benchmark modes (measurement only).

`pb2`/`pb3` run `route_prefix_search.find_prefix_b2_route` /
`optimise_prefix_b3_route` with the REAL default leg planner, wrapped so the
existing `RouteSearchStats.plan_leg` records per-leg timing (the lattice versus
compression/generation split, expansions, outcome). The wrapper makes exactly
the calls `plan_executable_leg` makes and changes nothing. The prefix search
calls its planner only on a cache miss, so every wrapped call is a miss; the
injected planner is not told a tree depth, so all wrapped legs are recorded at
depth 0 and per-depth statistics are not meaningful for these modes.

Limits are benchmark-operator inputs. Correctness runs (exactness, determinism,
proof, unlimited search) pass NONE: no planner checkpoint and no work limit, so
the isolated worker's hard timeout is their only containment. Checkpoint,
cancellation and work limits are supplied explicitly, only for budget,
cancellation, frontier and overshoot experiments. A planner checkpoint stop
(honest planner metadata), a harness hard timeout (the worker is killed; no
planner metadata) and the future production hard backstop are three different
things and are never conflated.

The normalisation and metadata checks here run in the worker AFTER the timed
search, so `planning_time_s` never includes them; replay runs later, in the
parent.
"""

import executable_route_optimizer as ero
import route_prefix_search as rps
from benchmarks import normalized as nz
from benchmarks.route_replay import invariant_violations
from config import DEFAULT_PLANNER_CONFIG

PREFIX_MODES = ("pb2", "pb3")
LIMIT_KEYS = ("max_leg_calls", "max_expansions", "max_nodes", "max_frontier", "checkpoint_s",
              "cancel_after_legs")


def is_prefix_mode(mode) -> bool:
    return mode in PREFIX_MODES


def limits_from_spec(spec: dict) -> rps.SearchLimits:
    kwargs = {}
    for source, target in (("max_leg_calls", "max_leg_calls"), ("max_expansions", "max_lattice_expansions"),
                           ("max_nodes", "max_nodes"), ("max_frontier", "max_frontier_entries"),
                           ("checkpoint_s", "wall_clock_checkpoint_limit_s")):
        if spec.get(source) is not None:
            kwargs[target] = spec[source]
    return rps.SearchLimits(**kwargs)


def limits_record(spec: dict) -> dict:
    """The effective limits, for the record: every field, `None` when unset."""
    limits = limits_from_spec(spec)
    return {
        "max_leg_calls": limits.max_leg_calls, "max_lattice_expansions": limits.max_lattice_expansions,
        "max_nodes": limits.max_nodes, "max_frontier_entries": limits.max_frontier_entries,
        "wall_clock_checkpoint_limit_s": limits.wall_clock_checkpoint_limit_s,
        "cancel_after_legs": spec.get("cancel_after_legs"),
    }


def run_prefix_search(fixture, mode: str, alternatives: bool, spec: dict, stats):
    """The timed part: run the search and return its `PrefixSearchResult`."""
    def timed_leg(start, target, obstacles, config):
        stats.note_request(0, False)
        return stats.plan_leg(ero.plan_executable_leg, start, target, obstacles, config, 0)

    cancel_after = spec.get("cancel_after_legs")
    cancel = None if cancel_after is None else (lambda: stats.cache_misses >= cancel_after)
    function = rps.find_prefix_b2_route if mode == "pb2" else rps.optimise_prefix_b3_route
    stats.begin(mode)
    try:
        return function(fixture.start, fixture.obstacles, config=DEFAULT_PLANNER_CONFIG,
                        leg_planner=timed_leg, allow_alternative_viewing=alternatives,
                        limits=limits_from_spec(spec), cancel_requested=cancel)
    finally:
        stats.end()


def _summary(norm: dict, result) -> dict:
    commands = [c for leg in norm["legs"] for c in leg["commands"]]
    return {
        "is_valid": norm["route_kind"] != "none", "is_complete": norm["route_kind"] == "complete",
        "route_status": result.route_status, "obstacle_order": norm["obstacle_order"],
        "skipped_obstacle_ids": norm["skipped_obstacle_ids"], "total_cost": norm["total_cost"],
        "total_distance_cm": norm["total_distance_cm"], "n_legs": len(norm["legs"]),
        "n_commands": len(commands),
        "reverse_commands": sum(1 for c in commands if c["direction"] in ("B", "BL", "BR")),
        "stop_reason": result.stop_reason, "work_limit_hit": result.work_limit_hit,
        "optimality_proven": result.optimality_proven,
        "complete_route_proven_impossible": result.complete_route_proven_impossible,
        "unresolved_branch_count": result.unresolved_branch_count, "message": result.message,
    }


def _blank_validation(norm, raw) -> dict:
    return {"normalized": norm, "b2_normalized": None, "raw": raw, "invariant_violations": None,
            "b3_not_worse_than_b2": None, "overshoot": None,
            "replay_violations": None, "b2_replay_violations": None,
            "timing": {"normalize_s": None, "replay_s": None}}


def seed_route_kind(route, fixture) -> str:
    """Honest label of B3's B2 seed route, from the route's own metadata: `complete`
    only if it visits every fixture obstacle and skips none, else `partial` (for
    example when an obstacle has no safe viewing candidate). Never inferred from
    the final B3 result."""
    if not route.skipped_obstacle_ids and len(route.obstacle_order) == len(fixture.obstacles):
        return "complete"
    return "partial"


def prefix_validation(result, fixture, mode: str, alternatives: bool, spec: dict, stats):
    """After the timed search: `(status, summary, hash, validation)`."""
    search = "prefix_" + mode[1:]
    norm = nz.normalize_prefix_result(result, search=search, start_pose=fixture.start,
                                      alternatives=alternatives)
    validation = _blank_validation(norm, nz.prefix_raw(result))
    validation["invariant_violations"] = invariant_violations(result)
    if mode == "pb3":
        validation["b2_normalized"] = nz.normalize_prefix_route(
            result.b2_route, seed_route_kind(result.b2_route, fixture), search="prefix_b2",
            start_pose=fixture.start, alternatives=alternatives) if result.b2_route is not None else None
        if result.complete_route is not None and result.b2_route is not None:
            validation["b3_not_worse_than_b2"] = result.complete_route.total_cost <= result.b2_route.total_cost
    checkpoint = spec.get("checkpoint_s")
    if checkpoint is not None:
        last_leg = stats.leg_records[-1].elapsed_s if stats.leg_records else None
        hit = result.stop_reason == rps.STOP_WALL_CLOCK_CHECKPOINT
        validation["overshoot"] = {
            "checkpoint_s": checkpoint, "checkpoint_stop": hit, "search_elapsed_s": result.elapsed_s,
            "overshoot_s": result.elapsed_s - checkpoint if hit else None,
            "last_planned_leg_s": last_leg,
        }
    status = {"complete": "complete", "partial": "partial", "none": "no_route"}[norm["route_kind"]]
    return status, _summary(norm, result), norm["hash"], validation


def ero_validation(result, fixture, mode: str, alternatives: bool):
    """Normalized record for an old-search result (`b2`/`b3`)."""
    search = "ero_" + mode
    norm = nz.normalize_ero_result(result, search=search, start_pose=fixture.start,
                                   obstacles=fixture.obstacles, alternatives=alternatives)
    return _blank_validation(norm, nz.ero_raw(result))
