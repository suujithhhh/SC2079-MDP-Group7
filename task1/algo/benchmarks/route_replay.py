"""Independent route replay and metadata invariant checks (measurement only).

`replay_violations()` re-checks a NORMALIZED route (`benchmarks/normalized.py`,
a plain dict, never a result object) against a fixture, without trusting the
search that produced it. It returns a list of `"<code>: <message>"` strings;
an empty list means the route replays cleanly. Codes:

  structure            malformed record
  order                visited/skipped ids are not a permutation of the fixture ids
  command.legality     direction/value not an STM primitive (via `primitive_length_cm`)
  command.unit         unit does not match the direction
  command.wire         `stm_command` is not exactly `MOV,<direction>,<value>`
  command.length       `length_cm` is not the primitive's own length
  chain.leg_start      first command does not start at the leg's start pose (exactly)
  chain.command        a command does not start where the previous one ended (exactly)
  chain.propagate      `propagate()` from the start does not reproduce `end_pose`
  chain.leg_final      leg `final_pose` is not its last command's `end_pose` (exactly)
  chain.route_final    route `final_pose` is not the last leg's `final_pose`
  collision            the whole swept path of a command is not safe
  target.tolerance     leg end is not within the goal tolerance of its target
  target.candidate     target is not the safety-filtered candidate at its index
  accounting.distance  distance is not the UNWEIGHTED ordered sum of `primitive_length_cm`
  accounting.cost      cost is not the ordered sum of length x `reverse_penalty` for B/BL/BR only

Distance and cost are computed independently and never from each other, so a
value cannot pass by matching the other formula. Leg totals are checked at the
planner's strict round-off tolerance; the route totals are the exact ordered
sums of the leg totals, as both searches aggregate them. Pose links use exact
equality; only re-propagation uses the strict tolerance.

`invariant_violations()` checks the cross-field metadata guarantees of a
`PrefixSearchResult` (proof conditions, wording, B3 not worse than B2, ...).
"""

import dataclasses

import route_prefix_search as rps
from config import DEFAULT_PLANNER_CONFIG
from executable_route_optimizer import _candidate_targets
from models import Pose
from motion_primitives import primitive_length_cm, propagate
from state_lattice import _within_goal_tolerance
from stm_command_generation import _STRICT_VALUE_TOL_CM, _poses_match_strict, _wire_string
from swept_collision import safe_primitive_samples

from benchmarks.normalized import ROUTE_KINDS, pose_list

DIRECTIONS = ("F", "B", "FL", "FR", "BL", "BR")
STRAIGHT = ("F", "B")
REVERSE = ("B", "BL", "BR")


def _pose(values):
    return None if values is None else Pose(*values)


def replay_violations(normalized, fixture, config=DEFAULT_PLANNER_CONFIG) -> list:
    """Problems found replaying a normalized route against `fixture` (empty = clean)."""
    if not isinstance(normalized, dict):
        raise TypeError(f"replay works on normalized dict records, got {type(normalized).__name__}")
    found = []

    def add(code, message):
        found.append(f"{code}: {message}")

    try:
        _replay(normalized, fixture, config, add)
    except (KeyError, TypeError, ValueError, IndexError) as error:
        add("structure", f"{type(error).__name__}: {error}")
    return found


def _replay(n, fixture, config, add):
    kind = n["route_kind"]
    if kind not in ROUTE_KINDS:
        add("structure", f"route_kind {kind!r}")
        return
    obstacles = tuple(fixture.obstacles)
    fixture_ids = [o.obstacle_id for o in obstacles]
    alternatives = bool(n["alternatives"])
    start = n["start_pose"]
    legs, order, skipped, indices = n["legs"], n["obstacle_order"], n["skipped_obstacle_ids"], n["candidate_indices"]

    if kind == "none":
        if legs or order or skipped or indices or n["final_pose"] is not None:
            add("structure", "a 'none' route must have no legs, order, skipped ids or final pose")
        return

    # -- order ---------------------------------------------------------------
    if len(set(order)) != len(order) or len(set(skipped)) != len(skipped) or set(order) & set(skipped) \
            or set(order) | set(skipped) != set(fixture_ids):
        add("order", f"visited {order} + skipped {skipped} is not a permutation of {fixture_ids}")
    if list(skipped) != [i for i in fixture_ids if i in set(skipped)]:
        add("order", f"skipped ids {skipped} are not in input order")
    if kind == "complete" and skipped:
        add("order", "a complete route has skipped obstacles")
    if not (len(legs) == len(order) == len(indices)):
        add("structure", f"legs/order/candidate_indices lengths differ: {len(legs)}/{len(order)}/{len(indices)}")
        return

    by_id = {o.obstacle_id: o for o in obstacles}
    candidate_cache = {}

    def candidates_of(obstacle_id):
        if obstacle_id not in candidate_cache:
            obstacle = by_id.get(obstacle_id)
            candidate_cache[obstacle_id] = () if obstacle is None else _candidate_targets(
                obstacle, obstacles, config, alternatives)
        return candidate_cache[obstacle_id]

    leg_start = start
    route_cost = 0.0
    route_distance = 0.0
    for i, leg in enumerate(legs):
        tag = f"leg {i} (obstacle {leg['obstacle_id']})"
        if leg["obstacle_id"] != order[i]:
            add("order", f"{tag}: obstacle id does not match obstacle_order[{i}]={order[i]}")
        commands = leg["commands"]
        distance = 0.0
        cost = 0.0
        computable = True
        previous_end = None
        for k, command in enumerate(commands):
            where = f"{tag} command {k}"
            direction, value = command["direction"], command["value"]
            length = None
            if direction not in DIRECTIONS or isinstance(value, bool) or not isinstance(value, int):
                add("command.legality", f"{where}: ({direction!r}, {value!r})")
            else:
                try:
                    length = primitive_length_cm(direction, value, config)
                except Exception as error:
                    add("command.legality", f"{where}: ({direction!r}, {value!r}): {error}")
            if direction in DIRECTIONS:
                expected_unit = "cm" if direction in STRAIGHT else "deg"
                if command["unit"] != expected_unit:
                    add("command.unit", f"{where}: unit {command['unit']!r}, expected {expected_unit!r}")
            if length is None:
                computable = False
            else:
                if command["stm_command"] != _wire_string(direction, value):
                    add("command.wire", f"{where}: {command['stm_command']!r}")
                if abs(command["length_cm"] - length) > _STRICT_VALUE_TOL_CM:
                    add("command.length", f"{where}: length_cm {command['length_cm']!r}, expected {length!r}")
                distance += length
                cost += length * config.reverse_penalty if direction in REVERSE else length
            expected_start = leg_start if k == 0 else previous_end
            if command["start_pose"] != expected_start:
                add("chain.leg_start" if k == 0 else "chain.command",
                    f"{where}: starts at {command['start_pose']}, expected {expected_start}")
            if length is not None:
                start_pose = _pose(command["start_pose"])
                end_pose = _pose(command["end_pose"])
                if not _poses_match_strict(propagate(start_pose, direction, value, config), end_pose):
                    add("chain.propagate", f"{where}: propagate() does not reproduce end_pose")
                if safe_primitive_samples(start_pose, direction, value, obstacles, config) is None:
                    add("collision", f"{where}: {command['stm_command']} sweeps an unsafe pose")
            previous_end = command["end_pose"]
        expected_final = previous_end if commands else leg_start
        if leg["final_pose"] != expected_final:
            add("chain.leg_final", f"{tag}: final_pose {leg['final_pose']}, expected {expected_final}")
        if computable:
            if abs(leg["total_distance_cm"] - distance) > _STRICT_VALUE_TOL_CM:
                add("accounting.distance", f"{tag}: total_distance_cm {leg['total_distance_cm']!r}, "
                    f"unweighted sum of lengths {distance!r}")
            if abs(leg["total_cost"] - cost) > _STRICT_VALUE_TOL_CM:
                add("accounting.cost", f"{tag}: total_cost {leg['total_cost']!r}, "
                    f"reverse-weighted sum {cost!r}")
        # -- target ----------------------------------------------------------
        target = leg["target_pose"]
        final_pose = leg["final_pose"]
        if final_pose is None or target is None or not _within_goal_tolerance(
                _pose(final_pose), _pose(target), config):
            add("target.tolerance", f"{tag}: final pose {final_pose} is not within the goal tolerance of {target}")
        found = candidates_of(leg["obstacle_id"])
        index = indices[i]
        if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(found) \
                or pose_list(found[index]) != target:
            add("target.candidate", f"{tag}: target {target} is not safety-filtered candidate {index}")
        route_cost += leg["total_cost"]
        route_distance += leg["total_distance_cm"]
        leg_start = final_pose

    expected_route_final = leg_start
    if n["final_pose"] != expected_route_final:
        add("chain.route_final", f"route final_pose {n['final_pose']}, expected {expected_route_final}")
    if n["total_distance_cm"] != route_distance:
        add("accounting.distance", f"route total_distance_cm {n['total_distance_cm']!r}, "
            f"ordered sum of leg distances {route_distance!r}")
    if n["total_cost"] != route_cost:
        add("accounting.cost", f"route total_cost {n['total_cost']!r}, ordered sum of leg costs {route_cost!r}")


def codes(violations) -> set:
    """The set of violation codes in a `replay_violations()` result."""
    return {v.split(":", 1)[0] for v in violations}


def invariant_violations(result) -> list:
    """Every cross-field guarantee a PrefixSearchResult must satisfy."""
    bad = []
    r = result
    closed = r.stop_reason in (rps.STOP_EXHAUSTED, rps.STOP_BOUND_CLOSED)
    limited = r.stop_reason in (rps.STOP_WORK_LIMIT, rps.STOP_WALL_CLOCK_CHECKPOINT, rps.STOP_CANCELLED)
    complete = r.route_status == rps.ROUTE_COMPLETE
    if r.optimality_proven and not (r.mode == rps.MODE_B3 and complete and closed
                                    and r.lower_bound_certified and r.bound_violation_count == 0):
        bad.append("optimality proven without an honest closed certified search")
    if r.optimality_proven and r.unresolved_branch_count and not (
            r.unresolved_bound_known and r.min_unresolved_lower_bound is not None
            and r.min_unresolved_lower_bound >= r.complete_route.total_cost):
        bad.append("optimality proven despite unresolved branches")
    if r.complete_route_proven_impossible and complete:
        bad.append("impossible but complete")
    if r.complete_route_proven_impossible and not r.empty_candidate_obstacle_ids and (
            r.stop_reason != rps.STOP_EXHAUSTED or r.unresolved_branch_count):
        bad.append("impossibility claimed without exhaustive definitive search")
    if limited and r.optimality_proven:
        bad.append("proof after a limit/cancel stop")
    if limited and r.complete_route_proven_impossible and not r.empty_candidate_obstacle_ids:
        bad.append("impossibility after a limit/cancel stop")
    if ("exists" in r.message) != r.complete_route_proven_impossible:
        bad.append("'exists' wording does not match the proof flag")
    if not complete and r.stop_reason in (rps.STOP_WORK_LIMIT, rps.STOP_WALL_CLOCK_CHECKPOINT) \
            and "within budget" not in r.message:
        bad.append("limit stop without 'within budget' wording")
    if "backstop" in r.message:
        bad.append("'backstop' wording")
    if complete != (r.complete_route is not None):
        bad.append("route_status/complete_route mismatch")
    if r.route_status == rps.ROUTE_PARTIAL_ONLY and r.best_partial_route is None:
        bad.append("partial status without a partial route")
    if r.route_status == rps.ROUTE_NONE and (r.complete_route or r.best_partial_route):
        bad.append("none status with a route")
    if complete and r.best_partial_route is not None:
        bad.append("partial route reported alongside a complete route")
    if r.mode == rps.MODE_B3 and complete and not r.empty_candidate_obstacle_ids:
        if r.b2_route is None:
            bad.append("B3 lost its B2 fallback")
        elif r.complete_route.total_cost > r.b2_route.total_cost:
            bad.append("B3 worse than B2")
    if not r.lower_bound_certified and (
            r.work.bound_prunes or r.stop_reason == rps.STOP_BOUND_CLOSED
            or r.root_lower_bound is not None or r.optimality_proven):
        bad.append("uncertified bound pruned, closed or proved")
    if r.bound_violation_count and r.bound_pruning_active:
        bad.append("pruning still active after an audit violation")
    if r.unresolved_branch_count == 0 and r.min_unresolved_lower_bound is not None:
        bad.append("min unresolved bound without unresolved branches")
    w = r.work
    if w.leg_requests != w.cache_hits + w.leg_calls:
        bad.append("leg request accounting")
    if w.complete_leaf_discoveries < w.complete_routes_found or w.unique_complete_orders > w.complete_routes_found:
        bad.append("complete-route counters inconsistent")
    if r.mode == rps.MODE_B2 and (w.frontier_peak or r.b2_work is not None or r.b2_route is not None):
        bad.append("B2 result carries B3 phase data")
    if r.b2_work is not None:
        for field in dataclasses.fields(rps.SearchWork):
            before, after = getattr(r.b2_work, field.name), getattr(w, field.name)
            if isinstance(before, int) and after < before:
                bad.append(f"cumulative work decreased: {field.name}")
    return bad
