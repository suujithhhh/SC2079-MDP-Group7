"""Standalone prefix-tree B2/B3 executable-route search (route_prefix_search.py).

Not connected to executable_route_optimizer.py's callers, planning_service_v2.py
or server.py. Almost every test injects a deterministic fake leg planner whose
costs respect the certified lower bound (`route_lower_bound`) and ero's own
`_heuristic_cm` bound, so pruning in both searches is sound and results can be
compared exactly. `RealPipelineSmoke` is the one small (1-obstacle) exception.

Scenario functions (`scenario_*`) return the list of violated guarantees; the
normal tests assert it is empty, and `MutantsAreDetected` patches one decision
seam at a time and asserts the same scenario then reports a violation.

No test sleeps or asserts on wall-clock time: an injected fake clock is used.

Run from algo/:
    python -m unittest tests.test_route_prefix_search -v
"""

import dataclasses
import math
import random
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import executable_route_optimizer as ero  # noqa: E402
import route_lower_bound as rlb  # noqa: E402
import route_prefix_search as rps  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
from models import Obstacle, Pose  # noqa: E402
from state_lattice import REASON_EXPANSION_LIMIT, REASON_NO_PATH, _heuristic_cm  # noqa: E402
from stm_command_generation import STMCommandPlan  # noqa: E402
from targets import viewing_target  # noqa: E402
from benchmarks.route_replay import invariant_violations  # noqa: E402

CFG = DEFAULT_PLANNER_CONFIG
MODEL = rlb.lower_bound_model_from_config(CFG)
END_OFFSET = (0.4, -0.3, 0.05)   # inside the 3 cm / 5 deg goal tolerance

START = Pose(100.0, 100.0, 0.0)
OBS_A = Obstacle(obstacle_id=1, x=150, y=100, image_face="W")
OBS_B = Obstacle(obstacle_id=2, x=100, y=150, image_face="S")
OBS_C = Obstacle(obstacle_id=3, x=100, y=60, image_face="N")
OBS_FAR = Obstacle(obstacle_id=4, x=100, y=190, image_face="S")
OBS_NEAR = Obstacle(obstacle_id=5, x=55, y=100, image_face="E")
OBS_UNSAFE = Obstacle(obstacle_id=42, x=0, y=100, image_face="W")   # no safe candidate at all
POOL = (OBS_A, OBS_B, OBS_C, OBS_FAR, OBS_NEAR)

# A_spread_08 geometry (benchmarks/fixtures/A_spread_08.json), copied inline.
SPREAD_START = Pose(20.0, 10.0, math.pi / 2)
SPREAD_08 = tuple(Obstacle(i, x, y, f) for i, x, y, f in (
    (1, 30, 100, "S"), (2, 100, 20, "W"), (3, 130, 50, "W"), (4, 80, 160, "E"),
    (5, 150, 140, "S"), (6, 170, 90, "N"), (7, 20, 150, "E"), (8, 100, 90, "N"),
))
# C_unsafe_primary_02 geometry (benchmarks/fixtures/C_unsafe_primary_02.json).
UNSAFE_PRIMARY_02 = (Obstacle(1, 30, 100, "S"), Obstacle(2, 15, 70, "E"))


def end_of(target: Pose) -> Pose:
    dx, dy, dh = END_OFFSET
    return Pose(target.x + dx, target.y + dy, target.heading + dh)


def admissible_cost(start: Pose, target: Pose) -> float:
    return max(rlb.pose_to_region_lower_bound(start, target, MODEL), _heuristic_cm(start, target, CFG))


def cands(obstacle, obstacles, alt):
    return ero._candidate_targets(obstacle, tuple(obstacles), CFG, alt)


def _hash01(*parts) -> float:
    return random.Random("|".join(repr(p) for p in parts)).random()


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeLegs:
    """Deterministic fake leg planner, a pure function of the exact
    (start, target) pair. Valid costs are `admissible_cost + extra`; the final
    pose is `end_of(target)` unless overridden. Records every call."""

    def __init__(self, extra=None, fail=None, final=None, cost=None, clock=None,
                 seconds_per_call=0.0, expansions=0):
        self.extra = extra or (lambda s, t: 1.0)
        self.fail = fail or (lambda s, t: None)
        self.final = final or (lambda s, t: end_of(t))
        self.cost = cost
        self.clock = clock
        self.seconds_per_call = seconds_per_call
        self.expansions = expansions
        self.calls = []
        self.counts = {}
        self.returned = {}

    def outcome(self, start, target):
        reason = self.fail(start, target)
        if reason is not None:
            return STMCommandPlan(False, (), (), 0.0, 0.0, self.expansions, None, reason)
        if self.cost is not None:
            value = self.cost(start, target)
        else:
            value = admissible_cost(start, target) + self.extra(start, target)
        return STMCommandPlan(True, (), (), value, value, self.expansions,
                              self.final(start, target), None)

    def __call__(self, start, target, obstacles, config=CFG):
        key = (start, target)
        self.calls.append(key)
        self.counts[key] = self.counts.get(key, 0) + 1
        if self.clock is not None:
            self.clock.now += self.seconds_per_call
        plan = self.outcome(start, target)
        self.returned.setdefault(key, []).append(plan)
        return plan


def seeded_legs(seed, fail_p=0.0, zero_p=0.0, max_extra=20.0):
    def extra(s, t):
        if _hash01(seed, "zero", s, t) < zero_p:
            return 0.0
        return max_extra * _hash01(seed, "extra", s, t)

    def fail(s, t):
        return REASON_NO_PATH if _hash01(seed, "fail", s, t) < fail_p else None

    return FakeLegs(extra=extra, fail=fail)


def brute_force(start, obstacles, alt, legs):
    """Exhaustive oracle: (best complete cost or None, (max depth, min cost at it))."""
    obstacles = tuple(obstacles)
    by_id = {o.obstacle_id: cands(o, obstacles, alt) for o in obstacles}
    best = [math.inf, 0, math.inf]

    def rec(pose, g, remaining, depth):
        if depth > best[1] or (depth == best[1] and g < best[2]):
            best[1], best[2] = depth, g
        if not remaining:
            best[0] = min(best[0], g)
            return
        for oid in remaining:
            for target in by_id[oid]:
                plan = legs.outcome(pose, target)
                if plan.is_valid:
                    rec(plan.final_pose, g + plan.total_cost, remaining - {oid}, depth + 1)

    rec(start, 0.0, frozenset(by_id), 0)
    return (None if best[0] == math.inf else best[0]), (best[1], best[2] if best[1] else 0.0)


def held_karp_oracle(start, obstacles, legs):
    """Exact optimum for primary-only candidates when the fake's end pose depends
    only on the target (so a leg's cost depends only on the previous candidate)."""
    obstacles = tuple(obstacles)
    targets = [cands(o, obstacles, False)[0] for o in obstacles]
    n = len(targets)
    dp = {}
    for i, t in enumerate(targets):
        dp[(1 << i, i)] = 0.0 + legs.outcome(start, t).total_cost
    for mask in range(1, 1 << n):
        for last in range(n):
            if (mask, last) not in dp:
                continue
            for nxt in range(n):
                if mask & (1 << nxt):
                    continue
                value = dp[(mask, last)] + legs.outcome(end_of(targets[last]), targets[nxt]).total_cost
                key = (mask | (1 << nxt), nxt)
                if value < dp.get(key, math.inf):
                    dp[key] = value
    return min(dp[((1 << n) - 1, last)] for last in range(n))


class ScaledBound:
    """Uncertified test bound: `scale * genuine + offset` (ordering/diagnostics only)."""

    def __init__(self, genuine, scale, offset):
        self.genuine = genuine
        self.scale = scale
        self.offset = offset
        self.obstacle_ids = genuine.obstacle_ids
        self.candidates = genuine.candidates

    def lower_bound(self, pose, remaining):
        return self.scale * self.genuine.lower_bound(pose, remaining) + self.offset

    def child_lower_bound(self, pose, oid, index, remaining):
        return self.scale * self.genuine.child_lower_bound(pose, oid, index, remaining) + self.offset


def scaled_factory(scale, offset):
    return lambda candidates, config: ScaledBound(
        rlb.build_remaining_route_bound(candidates, config), scale, offset)


ZERO_BOUND = scaled_factory(0.0, 0.0)
INFLATED_BOUND = scaled_factory(10.0, 1000.0)


def root_order(start, obstacles, alt=False):
    """Obstacle ids in the order B2 tries their rank-0 root children."""
    obstacles = tuple(obstacles)
    by_id = {o.obstacle_id: cands(o, obstacles, alt) for o in obstacles}
    bound = rlb.build_remaining_route_bound(by_id, CFG)
    ids = [o.obstacle_id for o in obstacles]
    keyed = [(bound.child_lower_bound(start, oid, 0, ids), i, oid) for i, oid in enumerate(ids)]
    return [oid for _f, _i, oid in sorted(keyed)]


class InvariantMixin:
    def assertInvariants(self, result):
        self.assertEqual(invariant_violations(result), [])


# --------------------------------------------------------------------------
# Scenario functions shared by the normal tests and the mutant self-checks
# --------------------------------------------------------------------------

def lazy_fallback_setup():
    """Obstacle X's rank-0 leg is valid but every continuation from it fails,
    and every leg from START to Y fails; X's rank-1 alternative completes."""
    obstacles = (OBS_A, OBS_FAR)
    x_id, y_id = root_order(START, obstacles, alt=True)
    by_id = {o.obstacle_id: cands(o, obstacles, True) for o in obstacles}
    x0 = by_id[x_id][0]
    y_targets = set(by_id[y_id])

    def fail(s, t):
        if t in y_targets and (s == START or s == end_of(x0)):
            return REASON_NO_PATH
        return None

    return obstacles, x_id, y_id, by_id, FakeLegs(fail=fail)


def scenario_lazy_fallback():
    obstacles, x_id, _y_id, by_id, legs = lazy_fallback_setup()
    result = rps.find_prefix_b2_route(START, obstacles, leg_planner=legs)
    bad = invariant_violations(result)
    if result.route_status != rps.ROUTE_COMPLETE:
        return bad + ["no complete route although an alternative completes"]
    route = result.complete_route
    x_pos = route.obstacle_order.index(x_id)
    if route.candidate_indices[x_pos] == 0:
        bad.append("rank-0 candidate cannot complete here")
    x0, x_alt = by_id[x_id][0], route.target_poses[x_pos]
    alt_call = legs.calls.index((START, x_alt))
    dead_calls = [i for i, (s, _t) in enumerate(legs.calls) if s == end_of(x0)]
    if not dead_calls or max(dead_calls) > alt_call:
        bad.append("alternative tried before the rank-0 subtree was exhausted")
    return bad


def budget_wording_setup():
    obstacles = (OBS_A, OBS_B)
    b_targets = set(cands(OBS_B, obstacles, False))
    return obstacles, FakeLegs(fail=lambda s, t: REASON_NO_PATH if t in b_targets else None)


def scenario_budget_wording():
    obstacles, legs = budget_wording_setup()
    exhaustive = rps.find_prefix_b2_route(START, obstacles, leg_planner=legs,
                                          allow_alternative_viewing=False)
    obstacles, legs = budget_wording_setup()
    limited = rps.find_prefix_b2_route(START, obstacles, leg_planner=legs,
                                       allow_alternative_viewing=False,
                                       limits=rps.SearchLimits(max_leg_calls=1))
    bad = invariant_violations(exhaustive) + invariant_violations(limited)
    if not exhaustive.complete_route_proven_impossible or "No complete route exists" not in exhaustive.message:
        bad.append("exhaustive definitive failure not proven")
    if limited.complete_route_proven_impossible or "exists" in limited.message:
        bad.append("budget-limited failure claimed non-existence")
    if "No complete route found within budget" not in limited.message:
        bad.append("budget-limited failure lacks 'within budget' wording")
    return bad


def expansion_limit_setup():
    obstacles = (OBS_A,)
    a0 = cands(OBS_A, obstacles, False)[0]
    return obstacles, FakeLegs(fail=lambda s, t: REASON_EXPANSION_LIMIT if (s, t) == (START, a0) else None)


def scenario_expansion_limit():
    obstacles, legs = expansion_limit_setup()
    result = rps.find_prefix_b2_route(START, obstacles, leg_planner=legs,
                                      allow_alternative_viewing=False)
    bad = invariant_violations(result)
    if result.complete_route_proven_impossible or result.search_space_exhausted:
        bad.append("expansion_limit treated as proof")
    if result.unresolved_branch_count != 1 or "inconclusive" not in result.message:
        bad.append("expansion_limit not reported as unresolved")
    return bad


def suboptimal_b2_setup(delta=0.5):
    """Two obstacles, primaries only; B2's (X, Y) route costs exactly `delta`
    more than the (Y, X) route, with the adjustment on a first leg."""
    obstacles = (OBS_A, OBS_FAR)
    x_id, y_id = root_order(START, obstacles)
    by_id = {o.obstacle_id: cands(o, obstacles, False)[0] for o in obstacles}
    x0, y0 = by_id[x_id], by_id[y_id]
    base = FakeLegs(extra=lambda s, t: 0.0)
    c_xy = base.outcome(START, x0).total_cost + base.outcome(end_of(x0), y0).total_cost
    c_yx = base.outcome(START, y0).total_cost + base.outcome(end_of(y0), x0).total_cost
    extras = {}
    if c_xy < c_yx + delta:
        extras[(START, x0)] = c_yx + delta - c_xy
    else:
        extras[(START, y0)] = c_xy - delta - c_yx
    return obstacles, x_id, FakeLegs(extra=lambda s, t: extras.get((s, t), 0.0))


def scenario_pruning_keeps_optimum():
    obstacles, x_id, legs = suboptimal_b2_setup()
    oracle, _ = brute_force(START, obstacles, False, legs)
    result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs,
                                          allow_alternative_viewing=False)
    bad = invariant_violations(result)
    if result.b2_route is None or result.b2_route.obstacle_order[0] != x_id:
        bad.append("scenario precondition: B2 should start with X")
    if result.complete_route is None or result.complete_route.total_cost != oracle:
        bad.append("B3 did not return the true optimum")
    if not result.optimality_proven:
        bad.append("honest exhaustive search not proven")
    for seed in range(30):
        obstacles = tuple(random.Random(seed).sample(POOL, 2 + seed % 3))
        alt = bool(seed % 2)
        legs = seeded_legs(seed, zero_p=0.5)
        oracle, _ = brute_force(START, obstacles, alt, legs)
        result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs,
                                              allow_alternative_viewing=alt)
        bad += invariant_violations(result)
        found = None if result.complete_route is None else result.complete_route.total_cost
        if found != oracle:
            bad.append(f"seed {seed}: B3 {found} != oracle {oracle}")
        if oracle is not None and not result.optimality_proven:
            bad.append(f"seed {seed}: honest exhaustive search not proven")
    return bad


def scenario_b3_never_worse():
    bad = []
    for seed in range(20):
        obstacles = tuple(random.Random(seed + 100).sample(POOL, 2 + seed % 3))
        alt = bool(seed % 2)
        for factory in (rlb.build_remaining_route_bound, ZERO_BOUND):
            legs = seeded_legs(seed, fail_p=0.15)
            result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs,
                                                  allow_alternative_viewing=alt,
                                                  bound_factory=factory)
            bad += invariant_violations(result)
            if result.complete_route is not None:
                if result.b2_route is None:
                    bad.append(f"seed {seed}: missing B2 fallback")
                elif result.complete_route.total_cost > result.b2_route.total_cost:
                    bad.append(f"seed {seed}: B3 worse than B2")
    return bad


def scenario_uncertified_cannot_prune():
    obstacles, _x_id, legs = suboptimal_b2_setup()
    oracle, _ = brute_force(START, obstacles, False, legs)
    result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs,
                                          allow_alternative_viewing=False,
                                          bound_factory=INFLATED_BOUND)
    bad = invariant_violations(result)
    if result.lower_bound_certified:
        bad.append("custom factory certified")
    if result.work.bound_prunes != 0 or result.stop_reason != rps.STOP_EXHAUSTED:
        bad.append("uncertified bound pruned or closed the search")
    if result.complete_route is None or result.complete_route.total_cost != oracle:
        bad.append("uncertified bound removed the optimum")
    if result.optimality_proven or result.root_lower_bound is not None:
        bad.append("uncertified bound supported a proof")
    return bad


def violation_setup():
    """Legs into obstacle C are inadmissibly cheap; every complete route
    visits C, so the audit fires during B2, before any B3 pruning."""
    obstacles = (OBS_A, OBS_B, OBS_C)
    c_targets = set(cands(OBS_C, obstacles, True))
    b2 = rps.find_prefix_b2_route(START, obstacles, leg_planner=FakeLegs(extra=lambda s, t: 0.0))
    first = (START, b2.complete_route.target_poses[0])

    def cost(s, t):
        if t in c_targets:
            return 0.1
        return admissible_cost(s, t) + (60.0 if (s, t) == first else 2.0)

    return obstacles, FakeLegs(cost=cost)


def scenario_violation_disables_pruning():
    obstacles, legs = violation_setup()
    oracle, _ = brute_force(START, obstacles, True, legs)
    result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs)
    bad = invariant_violations(result)
    if result.bound_violation_count == 0:
        bad.append("scenario precondition: audit violation expected")
    if result.bound_pruning_active or result.optimality_proven:
        bad.append("violation did not disable pruning/proof")
    if result.b2_route is None:
        bad.append("B2 fallback lost")
    if result.complete_route is None or result.complete_route.total_cost != oracle:
        bad.append("pruning after a violation removed the optimum")
    return bad


def scenario_exhausted_flag_on_limit():
    return scenario_budget_wording()


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------

class B2StraightforwardRoute(InvariantMixin, unittest.TestCase):
    def test_complete_route_chains_exact_final_poses(self):
        legs = FakeLegs()
        result = rps.find_prefix_b2_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=legs,
                                          allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertEqual(result.route_status, rps.ROUTE_COMPLETE)
        self.assertEqual(result.stop_reason, rps.STOP_FIRST_COMPLETE)
        self.assertFalse(result.optimality_proven)
        self.assertEqual(result.work.leg_calls, 3)
        route = result.complete_route
        for i, leg in enumerate(route.legs):
            expected_start = START if i == 0 else route.legs[i - 1].command_plan.final_pose
            self.assertEqual(legs.calls[i], (expected_start, leg.target_pose))
            self.assertNotEqual(leg.command_plan.final_pose, leg.target_pose)
        self.assertEqual(route.final_pose, route.legs[-1].command_plan.final_pose)
        self.assertEqual(route.total_cost, sum(leg.command_plan.total_cost for leg in route.legs))


class B2MultipleOrders(InvariantMixin, unittest.TestCase):
    def test_order_follows_lower_bound_guidance(self):
        obstacles = (OBS_A, OBS_B, OBS_C)
        legs = FakeLegs()
        result = rps.find_prefix_b2_route(START, obstacles, leg_planner=legs,
                                          allow_alternative_viewing=False)
        self.assertInvariants(result)
        by_id = {o.obstacle_id: cands(o, obstacles, False) for o in obstacles}
        bound = rlb.build_remaining_route_bound(by_id, CFG)
        pose, g, remaining, expected = START, 0.0, [1, 2, 3], []
        while remaining:
            best = min(remaining, key=lambda oid: (
                g + bound.child_lower_bound(pose, oid, 0, remaining), remaining.index(oid)))
            plan = legs.outcome(pose, by_id[best][0])
            pose, g = plan.final_pose, g + plan.total_cost
            expected.append(best)
            remaining.remove(best)
        self.assertEqual(result.complete_route.obstacle_order, tuple(expected))

    def test_ties_resolve_by_input_order(self):
        for obstacles in ((OBS_C, OBS_A, OBS_B), (OBS_B, OBS_C, OBS_A)):
            result = rps.find_prefix_b2_route(START, obstacles, leg_planner=FakeLegs(),
                                              allow_alternative_viewing=False,
                                              bound_factory=ZERO_BOUND)
            self.assertInvariants(result)
            self.assertEqual(result.complete_route.obstacle_order,
                             tuple(o.obstacle_id for o in obstacles))


class LazyFallbackAfterDownstreamDeadEnd(unittest.TestCase):
    def test_alternative_completes_after_rank0_subtree_dead_ends(self):
        self.assertEqual(scenario_lazy_fallback(), [])

    def test_whole_rank0_subtree_is_explored_first(self):
        obstacles, x_id, y_id, by_id, legs = lazy_fallback_setup()
        rps.find_prefix_b2_route(START, obstacles, leg_planner=legs)
        x0 = by_id[x_id][0]
        tried_from_x0 = {t for s, t in legs.calls if s == end_of(x0)}
        self.assertEqual(tried_from_x0, set(by_id[y_id]))


class PreferredCompletesNoAlternativesTried(InvariantMixin, unittest.TestCase):
    def test_no_rank1_leg_when_rank0_completes(self):
        obstacles = (OBS_A, OBS_B, OBS_C)
        legs = FakeLegs()
        result = rps.find_prefix_b2_route(START, obstacles, leg_planner=legs)
        self.assertInvariants(result)
        preferred = {cands(o, obstacles, True)[0] for o in obstacles}
        self.assertTrue(all(t in preferred for _s, t in legs.calls))
        self.assertEqual(result.complete_route.candidate_indices, (0, 0, 0))


class OtherNextObstacleSucceeds(InvariantMixin, unittest.TestCase):
    def test_different_next_obstacle_completes(self):
        obstacles = (OBS_A, OBS_B, OBS_C)
        first = root_order(START, obstacles)[0]
        x0 = cands(next(o for o in obstacles if o.obstacle_id == first), obstacles, False)[0]
        legs = FakeLegs(fail=lambda s, t: REASON_NO_PATH if s == end_of(x0) else None)
        result = rps.find_prefix_b2_route(START, obstacles, leg_planner=legs,
                                          allow_alternative_viewing=False)
        self.assertInvariants(result)
        order = result.complete_route.obstacle_order
        self.assertNotEqual(order[0], first)
        self.assertEqual(order[-1], first)
        self.assertEqual(legs.calls[0], (START, x0))


class EmptyCandidateGroup(InvariantMixin, unittest.TestCase):
    def test_empty_group_is_proven_even_with_tiny_budget(self):
        seen = []

        def spy(candidates, config):
            seen.append(tuple(sorted(candidates)))
            return rlb.build_remaining_route_bound(candidates, config)

        for search in (rps.find_prefix_b2_route, rps.optimise_prefix_b3_route):
            for limits in (rps.SearchLimits(), rps.SearchLimits(max_leg_calls=1)):
                result = search(START, [OBS_A, OBS_UNSAFE, OBS_B], leg_planner=FakeLegs(),
                                limits=limits, bound_factory=spy)
                self.assertInvariants(result)
                self.assertEqual(result.empty_candidate_obstacle_ids, (42,))
                self.assertTrue(result.complete_route_proven_impossible)
                self.assertNotEqual(result.route_status, rps.ROUTE_COMPLETE)
                self.assertIn("exists", result.message)
                self.assertIn("42", result.message)
        self.assertTrue(all(ids == (1, 2) for ids in seen))

    def test_best_partial_skips_the_empty_obstacle(self):
        result = rps.optimise_prefix_b3_route(START, [OBS_A, OBS_UNSAFE, OBS_B], leg_planner=FakeLegs())
        self.assertEqual(result.route_status, rps.ROUTE_PARTIAL_ONLY)
        self.assertEqual(set(result.best_partial_route.obstacle_order), {1, 2})
        self.assertEqual(result.best_partial_route.skipped_obstacle_ids, (42,))
        self.assertFalse(result.optimality_proven)

    def test_all_empty(self):
        for search in (rps.find_prefix_b2_route, rps.optimise_prefix_b3_route):
            result = search(START, [OBS_UNSAFE], leg_planner=FakeLegs())
            self.assertInvariants(result)
            self.assertEqual(result.route_status, rps.ROUTE_NONE)
            self.assertTrue(result.complete_route_proven_impossible)
            self.assertEqual(result.work.leg_calls, 0)


class ExactNoPathLeg(InvariantMixin, unittest.TestCase):
    def test_failure_closes_only_the_exact_edge(self):
        obstacles = (OBS_A, OBS_B)
        x_id, y_id = root_order(START, obstacles)
        x0 = cands(next(o for o in obstacles if o.obstacle_id == x_id), obstacles, False)[0]
        y0 = cands(next(o for o in obstacles if o.obstacle_id == y_id), obstacles, False)[0]
        legs = FakeLegs(fail=lambda s, t: REASON_NO_PATH if (s, t) == (START, x0) else None)
        result = rps.find_prefix_b2_route(START, obstacles, leg_planner=legs,
                                          allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertEqual(result.complete_route.obstacle_order, (y_id, x_id))
        self.assertIn((START, x0), legs.calls)
        self.assertIn((end_of(y0), x0), legs.calls)
        self.assertEqual(dict(result.work.leg_outcome_counts)[REASON_NO_PATH], 1)
        self.assertEqual(result.unresolved_branch_count, 0)


class ExactExpansionLimitLeg(InvariantMixin, unittest.TestCase):
    def test_b2_expansion_limit_is_inconclusive(self):
        self.assertEqual(scenario_expansion_limit(), [])
        obstacles, legs = expansion_limit_setup()
        result = rps.find_prefix_b2_route(START, obstacles, leg_planner=legs,
                                          allow_alternative_viewing=False)
        self.assertEqual(result.stop_reason, rps.STOP_EXHAUSTED)
        self.assertEqual(dict(result.work.leg_outcome_counts)[REASON_EXPANSION_LIMIT], 1)
        self.assertNotIn("exists", result.message)

    def test_b3_inconclusive_branch_below_incumbent_blocks_proof(self):
        obstacles = (OBS_A, OBS_FAR)
        x_id, y_id = root_order(START, obstacles)
        by_id = {o.obstacle_id: cands(o, obstacles, False)[0] for o in obstacles}
        x0, y0 = by_id[x_id], by_id[y_id]

        legs = FakeLegs(
            extra=lambda s, t: 200.0 if (s, t) == (end_of(x0), y0) else 0.0,
            fail=lambda s, t: REASON_EXPANSION_LIMIT if (s, t) == (START, y0) else None,
        )
        result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs,
                                              allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertEqual(result.route_status, rps.ROUTE_COMPLETE)
        self.assertFalse(result.optimality_proven)
        self.assertEqual(result.unresolved_branch_count, 1)
        self.assertTrue(result.unresolved_bound_known)
        self.assertLess(result.min_unresolved_lower_bound, result.complete_route.total_cost)
        self.assertIn("Optimality not proven", result.message)

    def test_proof_rule_for_inconclusive_roots(self):
        # In best-first order a popped inconclusive root has f < incumbent at
        # pop time, and any later incumbent's leaf has f >= it, so closure by
        # bound can only arise from exact ties; the rule is tested directly.
        flags = rps._finalise_flags
        B3, EXH = rps.MODE_B3, rps.STOP_EXHAUSTED
        self.assertTrue(flags(B3, True, EXH, True, 0, [150.0], 100.0, ())[2])
        self.assertTrue(flags(B3, True, EXH, True, 0, [100.0], 100.0, ())[2])
        self.assertFalse(flags(B3, True, EXH, True, 0, [99.0], 100.0, ())[2])
        self.assertFalse(flags(B3, True, EXH, True, 0, [150.0, None], 100.0, ())[2])
        self.assertFalse(flags(B3, True, EXH, False, 0, [], 100.0, ())[2])
        self.assertFalse(flags(B3, True, EXH, True, 1, [], 100.0, ())[2])
        self.assertFalse(flags(rps.MODE_B2, True, EXH, True, 0, [], 100.0, ())[2])
        for stop in (rps.STOP_WORK_LIMIT, rps.STOP_WALL_CLOCK_CHECKPOINT, rps.STOP_CANCELLED):
            self.assertEqual(flags(B3, True, stop, True, 0, [], 100.0, ())[:3], (False, False, False))
        self.assertEqual(flags(B3, False, EXH, True, 0, [], None, ())[:3], (True, True, False))
        self.assertEqual(flags(B3, False, EXH, True, 0, [None], None, ())[:3], (False, False, False))
        self.assertTrue(flags(B3, False, rps.STOP_WORK_LIMIT, True, 0, [None], None, (7,))[1])


class ExhaustiveVersusBudgetWording(unittest.TestCase):
    def test_wording(self):
        self.assertEqual(scenario_budget_wording(), [])


class BestPartialSelection(InvariantMixin, unittest.TestCase):
    def test_more_visited_then_lower_cost(self):
        obstacles = (OBS_A, OBS_B, OBS_C)
        c0 = cands(OBS_C, obstacles, False)[0]

        def make():
            return FakeLegs(extra=lambda s, t: 3.0 * _hash01("partial", s, t),
                            fail=lambda s, t: REASON_NO_PATH if t == c0 else None)

        _, (depth, cost) = brute_force(START, obstacles, False, make())
        first = rps.find_prefix_b2_route(START, obstacles, leg_planner=make(),
                                         allow_alternative_viewing=False, clock=FakeClock())
        again = rps.find_prefix_b2_route(START, obstacles, leg_planner=make(),
                                         allow_alternative_viewing=False, clock=FakeClock())
        self.assertInvariants(first)
        self.assertEqual(first.route_status, rps.ROUTE_PARTIAL_ONLY)
        partial = first.best_partial_route
        self.assertEqual(len(partial.obstacle_order), depth)
        self.assertEqual(depth, 2)
        self.assertEqual(partial.total_cost, cost)
        self.assertEqual(partial.skipped_obstacle_ids, (3,))
        self.assertTrue(first.complete_route_proven_impossible)
        self.assertEqual(first, again)


class B3NeverWorseThanB2(unittest.TestCase):
    def test_seeded(self):
        self.assertEqual(scenario_b3_never_worse(), [])


class B3ImprovesOnB2(InvariantMixin, unittest.TestCase):
    def test_improvement_is_found_and_proven(self):
        obstacles, _x_id, legs = suboptimal_b2_setup(delta=5.0)
        oracle, _ = brute_force(START, obstacles, False, legs)
        result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs,
                                              allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertEqual(result.complete_route.total_cost, oracle)
        self.assertLess(result.complete_route.total_cost, result.b2_route.total_cost)
        self.assertGreaterEqual(result.work.incumbent_updates, 1)
        self.assertTrue(result.optimality_proven)
        self.assertIn("Optimality proven", result.message)


class B3WorkBudgetReturnsIncumbent(InvariantMixin, unittest.TestCase):
    def test_leg_call_budget(self):
        legs = seeded_legs(3)
        result = rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=legs,
                                              limits=rps.SearchLimits(max_leg_calls=3))
        self.assertInvariants(result)
        self.assertEqual(result.b2_work.leg_calls, 3)
        self.assertEqual(result.work.leg_calls, 3)
        self.assertEqual(result.stop_reason, rps.STOP_WORK_LIMIT)
        self.assertEqual(result.work_limit_hit, rps.LIMIT_LEG_CALLS)
        self.assertEqual(result.complete_route, result.b2_route)
        self.assertFalse(result.optimality_proven)
        self.assertGreater(result.unresolved_branch_count, 0)
        self.assertIn("Optimality not proven (work limit max_leg_calls reached)", result.message)

    def test_expansion_and_node_budgets(self):
        legs = FakeLegs(expansions=7)
        result = rps.find_prefix_b2_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=legs,
                                          limits=rps.SearchLimits(max_lattice_expansions=14))
        self.assertInvariants(result)
        self.assertEqual(result.work_limit_hit, rps.LIMIT_LATTICE_EXPANSIONS)
        self.assertEqual(result.work.lattice_expansions, 14)
        result = rps.find_prefix_b2_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=FakeLegs(),
                                          limits=rps.SearchLimits(max_nodes=2))
        self.assertInvariants(result)
        self.assertEqual(result.work_limit_hit, rps.LIMIT_NODES)
        self.assertEqual(result.work.nodes_expanded, 2)
        self.assertIn("within budget (max_nodes)", result.message)


class WallClockCheckpointLimit(InvariantMixin, unittest.TestCase):
    def test_checkpoint_between_legs_with_fake_clock(self):
        clock = FakeClock()
        legs = FakeLegs(fail=lambda s, t: REASON_NO_PATH, clock=clock, seconds_per_call=10.0)
        result = rps.find_prefix_b2_route(
            START, [OBS_A, OBS_B, OBS_C], leg_planner=legs, clock=clock,
            limits=rps.SearchLimits(wall_clock_checkpoint_limit_s=45.0))
        self.assertInvariants(result)
        self.assertEqual(result.stop_reason, rps.STOP_WALL_CLOCK_CHECKPOINT)
        # The fifth leg starts at t=40 and ends at t=50: it is not interrupted,
        # so the limit is overshot by that leg; the next checkpoint stops.
        self.assertEqual(result.work.leg_calls, 5)
        self.assertEqual(result.elapsed_s, 50.0)
        self.assertEqual(dict(result.work.leg_outcome_counts), {REASON_NO_PATH: 5})
        self.assertIn("wall-clock checkpoint limit", result.message)
        self.assertIn("within budget", result.message)
        self.assertFalse(result.complete_route_proven_impossible)
        self.assertGreater(result.unresolved_branch_count, 0)

    def test_b3_keeps_incumbent_at_checkpoint_limit(self):
        clock = FakeClock()
        legs = seeded_legs(4)
        legs.clock, legs.seconds_per_call = clock, 10.0
        result = rps.optimise_prefix_b3_route(
            START, [OBS_A, OBS_B, OBS_C], leg_planner=legs, clock=clock,
            limits=rps.SearchLimits(wall_clock_checkpoint_limit_s=35.0))
        self.assertInvariants(result)
        self.assertEqual(result.stop_reason, rps.STOP_WALL_CLOCK_CHECKPOINT)
        self.assertEqual(result.complete_route, result.b2_route)
        self.assertFalse(result.optimality_proven)

    def test_cancellation(self):
        legs = FakeLegs(fail=lambda s, t: REASON_NO_PATH)
        result = rps.find_prefix_b2_route(START, [OBS_A, OBS_B], leg_planner=legs,
                                          cancel_requested=lambda: len(legs.calls) >= 2)
        self.assertInvariants(result)
        self.assertEqual(result.stop_reason, rps.STOP_CANCELLED)
        self.assertEqual(result.work.leg_calls, 2)
        self.assertIn("cancelled", result.message)
        self.assertFalse(result.complete_route_proven_impossible)


class StopPrecedence(InvariantMixin, unittest.TestCase):
    """Simultaneously applicable stop conditions report work limits first (leg
    calls, lattice expansions, nodes, frontier), then cancellation, then the
    wall-clock checkpoint limit. Each case is paired with a control showing the
    lower-precedence condition does fire when no work limit is reached."""

    OBSTACLES = (OBS_A, OBS_B, OBS_C)

    def timed(self, **legs_kwargs):
        clock = FakeClock()
        return clock, FakeLegs(clock=clock, seconds_per_call=10.0, **legs_kwargs)

    def run_b2(self, legs, clock, limits, cancel=None):
        result = rps.find_prefix_b2_route(START, self.OBSTACLES, leg_planner=legs, clock=clock,
                                          allow_alternative_viewing=False, limits=limits,
                                          cancel_requested=cancel)
        self.assertInvariants(result)
        return result

    def assertWorkLimit(self, result, limit):
        self.assertEqual(result.stop_reason, rps.STOP_WORK_LIMIT)
        self.assertEqual(result.work_limit_hit, limit)
        if result.complete_route is None:
            self.assertIn(f"within budget ({limit})", result.message)
        else:
            self.assertIn(f"work limit {limit} reached", result.message)

    def test_node_limit_beats_wall_clock(self):
        clock, legs = self.timed()
        result = self.run_b2(legs, clock, rps.SearchLimits(max_nodes=2, wall_clock_checkpoint_limit_s=10.0))
        self.assertWorkLimit(result, rps.LIMIT_NODES)
        self.assertEqual((result.work.nodes_expanded, result.work.leg_calls), (2, 1))
        self.assertEqual(result.elapsed_s, 10.0)
        clock, legs = self.timed()
        control = self.run_b2(legs, clock, rps.SearchLimits(wall_clock_checkpoint_limit_s=10.0))
        self.assertEqual(control.stop_reason, rps.STOP_WALL_CLOCK_CHECKPOINT)
        self.assertEqual((control.work.nodes_expanded, control.work.leg_calls), (2, 1))

    def test_node_limit_beats_cancellation(self):
        clock, legs = self.timed()
        result = self.run_b2(legs, clock, rps.SearchLimits(max_nodes=2),
                             cancel=lambda: len(legs.calls) >= 1)
        self.assertWorkLimit(result, rps.LIMIT_NODES)

    def test_leg_call_limit_beats_cancellation_and_wall_clock(self):
        clock, legs = self.timed()
        result = self.run_b2(legs, clock, rps.SearchLimits(max_leg_calls=1),
                             cancel=lambda: len(legs.calls) >= 1)
        self.assertWorkLimit(result, rps.LIMIT_LEG_CALLS)
        clock, legs = self.timed()
        result = self.run_b2(legs, clock, rps.SearchLimits(max_leg_calls=1, wall_clock_checkpoint_limit_s=10.0))
        self.assertWorkLimit(result, rps.LIMIT_LEG_CALLS)
        clock, legs = self.timed()
        control = self.run_b2(legs, clock, rps.SearchLimits(), cancel=lambda: len(legs.calls) >= 1)
        self.assertEqual(control.stop_reason, rps.STOP_CANCELLED)

    def test_expansion_limit_beats_cancellation_and_wall_clock(self):
        clock, legs = self.timed(expansions=7)
        result = self.run_b2(legs, clock, rps.SearchLimits(max_lattice_expansions=7),
                             cancel=lambda: len(legs.calls) >= 1)
        self.assertWorkLimit(result, rps.LIMIT_LATTICE_EXPANSIONS)
        clock, legs = self.timed(expansions=7)
        result = self.run_b2(legs, clock, rps.SearchLimits(max_lattice_expansions=7,
                                                           wall_clock_checkpoint_limit_s=10.0))
        self.assertWorkLimit(result, rps.LIMIT_LATTICE_EXPANSIONS)

    def test_cancellation_beats_wall_clock_without_a_work_limit(self):
        clock, legs = self.timed()
        result = self.run_b2(legs, clock, rps.SearchLimits(wall_clock_checkpoint_limit_s=10.0),
                             cancel=lambda: len(legs.calls) >= 1)
        self.assertEqual(result.stop_reason, rps.STOP_CANCELLED)
        self.assertIsNone(result.work_limit_hit)
        self.assertGreaterEqual(result.elapsed_s, 10.0)   # the wall-clock limit was reached too
        self.assertIn("cancelled", result.message)

    def test_work_limits_keep_their_own_order(self):
        # Leg calls before nodes, at the loop-top checkpoint (with the wall clock also reached) ...
        clock, legs = self.timed()
        result = self.run_b2(legs, clock, rps.SearchLimits(max_leg_calls=1, max_nodes=2,
                                                           wall_clock_checkpoint_limit_s=10.0))
        self.assertWorkLimit(result, rps.LIMIT_LEG_CALLS)
        # ... and at the node checkpoint itself.
        clock, legs = self.timed()
        result = self.run_b2(legs, clock, rps.SearchLimits(max_leg_calls=1, max_nodes=1))
        self.assertWorkLimit(result, rps.LIMIT_LEG_CALLS)
        # Expansions before nodes.
        clock, legs = self.timed(expansions=7)
        result = self.run_b2(legs, clock, rps.SearchLimits(max_lattice_expansions=7, max_nodes=1))
        self.assertWorkLimit(result, rps.LIMIT_LATTICE_EXPANSIONS)
        # Nodes alone.
        clock, legs = self.timed()
        result = self.run_b2(legs, clock, rps.SearchLimits(max_nodes=1))
        self.assertWorkLimit(result, rps.LIMIT_NODES)

    def test_frontier_limit_beats_cancellation_but_not_higher_work_limits(self):
        obstacles = (OBS_A, OBS_B)

        def b3(limits, **kwargs):
            legs = FakeLegs()
            result = rps.optimise_prefix_b3_route(
                START, obstacles, leg_planner=legs, allow_alternative_viewing=False, limits=limits,
                cancel_requested=lambda: len(legs.calls) >= 2, **kwargs)   # true once B2 is done
            self.assertInvariants(result)
            self.assertEqual(result.complete_route, result.b2_route)
            return result

        result = b3(rps.SearchLimits(max_frontier_entries=1))
        self.assertWorkLimit(result, rps.LIMIT_FRONTIER)
        self.assertEqual(b3(rps.SearchLimits()).stop_reason, rps.STOP_CANCELLED)
        # Leg calls (already 2) outrank the frontier at the same checkpoint.
        result = b3(rps.SearchLimits(max_frontier_entries=1, max_leg_calls=2))
        self.assertWorkLimit(result, rps.LIMIT_LEG_CALLS)

    def test_a_running_leg_is_never_interrupted(self):
        clock, legs = self.timed()
        result = self.run_b2(legs, clock, rps.SearchLimits(wall_clock_checkpoint_limit_s=1.0))
        self.assertEqual(result.stop_reason, rps.STOP_WALL_CLOCK_CHECKPOINT)
        self.assertEqual(result.work.leg_calls, 1)          # the first leg ran to completion
        self.assertEqual(result.elapsed_s, 10.0)            # ... overshooting the 1 s limit


class AuditBoundAccounting(InvariantMixin, unittest.TestCase):
    """The certified single-edge audit bound goes through the same counted,
    validated path as every other bound evaluation."""

    def spy_child_bound(self, override=None):
        calls = []
        original = rlb.RemainingRouteBound.child_lower_bound

        def spy(bound, pose, obstacle_id, candidate_index, remaining_ids):
            remaining = tuple(remaining_ids)
            calls.append((pose, obstacle_id, candidate_index, remaining))
            value = original(bound, pose, obstacle_id, candidate_index, remaining)
            return value if override is None else override(remaining, value)

        return calls, mock.patch.object(rlb.RemainingRouteBound, "child_lower_bound", spy)

    def test_audit_evaluations_are_counted_with_singleton_remaining_ids(self):
        calls, patch = self.spy_child_bound()
        with patch:
            one = rps.find_prefix_b2_route(START, [OBS_A], leg_planner=FakeLegs(),
                                           allow_alternative_viewing=False)
        self.assertInvariants(one)
        # Ordering the root's only child, then auditing the one valid leg.
        self.assertEqual(one.work.bound_evaluations, 2)
        self.assertEqual(calls, [(START, 1, 0, (1,)), (START, 1, 0, (1,))])

        calls, patch = self.spy_child_bound()
        with patch:
            two = rps.find_prefix_b2_route(START, [OBS_A, OBS_B], leg_planner=FakeLegs(),
                                           allow_alternative_viewing=False)
        self.assertInvariants(two)
        # Root ordering (2) + audit (1) + the child's ordering (1) + audit (1).
        self.assertEqual(two.work.bound_evaluations, 5)
        self.assertEqual(two.work.bound_evaluations, len(calls))
        self.assertEqual(sum(1 for c in calls if len(c[3]) == 2), 2)
        first = two.complete_route.obstacle_order[0]
        self.assertEqual(calls[2], (START, first, 0, (first,)))          # the first leg's audit

    def test_each_exact_leg_is_audited_once(self):
        audits = []
        original = rps._Search._audit_lb

        def counting(search, pose, active_index, cand_index):
            audits.append((pose, active_index, cand_index))
            return original(search, pose, active_index, cand_index)

        legs = FakeLegs()
        with mock.patch.object(rps._Search, "_audit_lb", counting):
            result = rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B], leg_planner=legs,
                                                  allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertGreater(result.work.cache_hits, 0)                  # B3 re-reads B2's legs ...
        self.assertEqual(len(audits), result.work.leg_calls)           # ... yet audits each valid leg once
        self.assertEqual(len(set(audits)), len(audits))
        self.assertEqual(result.bound_violation_count, 0)

    def test_invalid_audit_values_raise_instead_of_becoming_violations(self):
        for bad in (float("nan"), -1.0, "1", True, None):
            calls, patch = self.spy_child_bound(
                override=lambda remaining, value, bad=bad: bad if len(remaining) == 1 else value)
            with patch:
                with self.assertRaises(rps.InvalidPrefixSearchInputError, msg=repr(bad)):
                    rps.find_prefix_b2_route(START, [OBS_A, OBS_B], leg_planner=FakeLegs(),
                                             allow_alternative_viewing=False)
            # The two root ordering evaluations succeed; the first singleton call is the audit.
            self.assertEqual([len(c[3]) for c in calls], [2, 2, 1], repr(bad))

    def test_a_valid_but_too_high_audit_value_is_still_a_violation(self):
        calls, patch = self.spy_child_bound(
            override=lambda remaining, value: value + 1e6 if len(remaining) == 1 else value)
        with patch:
            result = rps.find_prefix_b2_route(START, [OBS_A, OBS_B], leg_planner=FakeLegs(),
                                              allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertGreaterEqual(result.bound_violation_count, 1)
        self.assertFalse(result.bound_pruning_active)


class OptimalityProvenOnlyWhenHonest(InvariantMixin, unittest.TestCase):
    def test_scenario_table(self):
        # (result, expected optimality_proven)
        cases = [
            (rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B], leg_planner=seeded_legs(1)), True),
            (rps.find_prefix_b2_route(START, [OBS_A, OBS_B], leg_planner=seeded_legs(1)), False),
            (rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B], leg_planner=seeded_legs(1),
                                          bound_factory=ZERO_BOUND), False),
            (rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=seeded_legs(1),
                                          limits=rps.SearchLimits(max_leg_calls=4)), False),
            (rps.optimise_prefix_b3_route(START, [OBS_A, OBS_UNSAFE], leg_planner=seeded_legs(1)), False),
        ]
        for result, expected in cases:
            self.assertInvariants(result)
            self.assertEqual(result.optimality_proven, expected, result.message)


class PruningKeepsTrueOptimum(unittest.TestCase):
    def test_bound_pruning_never_removes_the_optimum(self):
        self.assertEqual(scenario_pruning_keeps_optimum(), [])


class UncertifiedBoundsCannotPruneOrProve(InvariantMixin, unittest.TestCase):
    def test_inflated_uncertified_bound_only_orders(self):
        self.assertEqual(scenario_uncertified_cannot_prune(), [])

    def test_audit_violation_disables_pruning(self):
        self.assertEqual(scenario_violation_disables_pruning(), [])

    def test_violation_after_prunes_reopens_them(self):
        exercised = 0
        for seed in range(25):
            obstacles = tuple(random.Random(seed + 7).sample(POOL, 3))
            clean = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=seeded_legs(seed))
            route_pairs = set()
            pose = START
            for leg in clean.b2_route.legs:
                route_pairs.add((pose, leg.target_pose))
                pose = leg.command_plan.final_pose
            base = seeded_legs(seed)

            def cost(s, t, base=base, route_pairs=route_pairs, seed=seed):
                value = base.outcome(s, t).total_cost
                if (s, t) not in route_pairs and _hash01(seed, "cheap", s, t) < 0.3:
                    return 0.25 * value
                return value

            reopened = []
            original = rps._reopen_bound_prunes

            def spy(phase):
                reopened.append(len(phase.bound_pruned))
                original(phase)

            with mock.patch.object(rps, "_reopen_bound_prunes", spy):
                result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=FakeLegs(cost=cost))
            self.assertInvariants(result)
            if result.bound_violation_count:
                self.assertFalse(result.optimality_proven)
                self.assertFalse(result.bound_pruning_active)
                self.assertIsNotNone(result.b2_route)
            if reopened and reopened[0] > 0:
                exercised += 1
                self.assertGreaterEqual(result.unresolved_branch_count, reopened[0])
                self.assertFalse(result.unresolved_bound_known)
                self.assertIsNone(result.min_unresolved_lower_bound)
        self.assertGreater(exercised, 0, "no seed produced a violation after a prune")


class ExactPoseCacheKeys(InvariantMixin, unittest.TestCase):
    def test_nearly_equal_start_poses_are_distinct_queries(self):
        obstacles = (OBS_A, OBS_B, OBS_C)
        a0, b0, c0 = (cands(o, obstacles, False)[0] for o in obstacles)
        e1 = end_of(a0)
        e2 = Pose(e1.x + 1e-12, e1.y, e1.heading)
        self.assertNotEqual(e1, e2)

        def final(s, t):
            return e2 if (s, t) == (end_of(b0), a0) else end_of(t)

        legs = FakeLegs(final=final)
        result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs,
                                              allow_alternative_viewing=False,
                                              bound_factory=ZERO_BOUND)
        self.assertInvariants(result)
        self.assertEqual(legs.counts[(e1, c0)], 1)
        self.assertEqual(legs.counts[(e2, c0)], 1)

    def test_b2_prefix_rewalked_by_b3_is_a_cache_hit(self):
        legs = seeded_legs(9)
        result = rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B], leg_planner=legs,
                                              allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertEqual(max(legs.counts.values()), 1)
        self.assertGreater(result.work.cache_hits - result.b2_work.cache_hits, 0)

    def test_failures_are_cached_and_plans_shared_unmodified(self):
        obstacles = (OBS_A, OBS_B)
        x_id = root_order(START, obstacles)[0]
        x0 = cands(next(o for o in obstacles if o.obstacle_id == x_id), obstacles, False)[0]
        legs = FakeLegs(fail=lambda s, t: REASON_NO_PATH if (s, t) == (START, x0) else None)
        result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs,
                                              allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertEqual(legs.counts[(START, x0)], 1)
        self.assertEqual(max(legs.counts.values()), 1)
        pose = START
        for leg in result.complete_route.legs:
            first_returned = legs.returned[(pose, leg.target_pose)][0]
            self.assertIs(leg.command_plan, first_returned)
            self.assertEqual(dataclasses.astuple(leg.command_plan), dataclasses.astuple(first_returned))
            pose = leg.command_plan.final_pose


class UniqueOrderCounting(InvariantMixin, unittest.TestCase):
    def test_candidate_branches_do_not_inflate_order_counts(self):
        obstacles = (OBS_A, OBS_B)
        k_a, k_b = (len(cands(o, obstacles, True)) for o in obstacles)
        self.assertGreater(k_a * k_b, 1)
        result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=FakeLegs(),
                                              bound_factory=ZERO_BOUND)
        self.assertInvariants(result)
        self.assertEqual(result.work.unique_complete_orders, 2)
        self.assertEqual(result.work.unique_prefixes_reached, 4)
        self.assertEqual(result.work.complete_routes_found, 2 * k_a * k_b)
        # B3 rediscovers B2's leaf once: counted as a discovery, not a new route.
        self.assertEqual(result.work.complete_leaf_discoveries, 2 * k_a * k_b + 1)
        self.assertEqual(result.b2_work.complete_routes_found, 1)


class ImmutabilityAndDeterminism(InvariantMixin, unittest.TestCase):
    def test_inputs_untouched_and_results_repeatable(self):
        config_before = dataclasses.replace(CFG)
        obstacle_list = [OBS_A, OBS_B, OBS_C]
        snapshot = list(obstacle_list)
        results = []
        inputs = (lambda: obstacle_list, lambda: tuple(obstacle_list), lambda: (o for o in obstacle_list))
        for make_input in inputs:
            for search in (rps.find_prefix_b2_route, rps.optimise_prefix_b3_route):
                clock = FakeClock()
                legs = seeded_legs(11, fail_p=0.1)
                legs.clock, legs.seconds_per_call = clock, 1.0
                results.append(search(START, make_input(), leg_planner=legs, clock=clock))
        self.assertEqual(obstacle_list, snapshot)
        self.assertEqual(CFG, config_before)
        b2s, b3s = results[0::2], results[1::2]
        self.assertTrue(all(r == b2s[0] for r in b2s))
        self.assertTrue(all(r == b3s[0] for r in b3s))
        with self.assertRaises(dataclasses.FrozenInstanceError):
            b3s[0].optimality_proven = False
        with self.assertRaises(dataclasses.FrozenInstanceError):
            b3s[0].complete_route.total_cost = 0.0


class EmptyAndSingleObstacle(InvariantMixin, unittest.TestCase):
    def test_empty(self):
        calls = []
        factory = lambda c, cfg: calls.append(c)  # noqa: E731 - must never be invoked
        b2 = rps.find_prefix_b2_route(START, [], leg_planner=FakeLegs(), bound_factory=factory)
        b3 = rps.optimise_prefix_b3_route(START, [], leg_planner=FakeLegs())
        for result in (b2, b3):
            self.assertInvariants(result)
            self.assertEqual(result.route_status, rps.ROUTE_COMPLETE)
            self.assertEqual(result.complete_route.legs, ())
            self.assertEqual(result.complete_route.final_pose, START)
            self.assertEqual(result.complete_route.total_cost, 0.0)
        self.assertEqual(calls, [])
        self.assertFalse(b2.optimality_proven)
        self.assertTrue(b3.optimality_proven)

    def test_single(self):
        for search in (rps.find_prefix_b2_route, rps.optimise_prefix_b3_route):
            result = search(START, [OBS_A], leg_planner=FakeLegs(), allow_alternative_viewing=False)
            self.assertInvariants(result)
            self.assertEqual(result.complete_route.obstacle_order, (1,))
            self.assertEqual(len(result.complete_route.legs), 1)


class EightObstacleOperationCounts(InvariantMixin, unittest.TestCase):
    def test_b2_dives_with_exactly_eight_legs(self):
        for alt in (False, True):
            result = rps.find_prefix_b2_route(SPREAD_START, SPREAD_08, leg_planner=FakeLegs(),
                                              allow_alternative_viewing=alt)
            self.assertInvariants(result)
            self.assertEqual(result.route_status, rps.ROUTE_COMPLETE)
            self.assertEqual(result.work.leg_calls, 8)
            self.assertEqual(result.work.nodes_expanded, 8)

    def test_b3_primary_only_matches_held_karp_oracle(self):
        legs = seeded_legs(5, zero_p=0.3, max_extra=10.0)
        runs = [rps.optimise_prefix_b3_route(SPREAD_START, SPREAD_08, leg_planner=seeded_legs(5, zero_p=0.3, max_extra=10.0),
                                             allow_alternative_viewing=False,
                                             limits=rps.SearchLimits(max_leg_calls=3000))
                for _ in range(2)]
        result = runs[0]
        self.assertInvariants(result)
        self.assertEqual(runs[0].work, runs[1].work)
        self.assertLessEqual(result.work.leg_calls, 3000)
        if result.optimality_proven:
            self.assertEqual(result.complete_route.total_cost, held_karp_oracle(SPREAD_START, SPREAD_08, legs))
        else:
            self.assertEqual(result.stop_reason, rps.STOP_WORK_LIMIT)

    def test_b3_with_alternatives_respects_limits_deterministically(self):
        limits = rps.SearchLimits(max_leg_calls=300, max_frontier_entries=20_000)
        runs = [rps.optimise_prefix_b3_route(SPREAD_START, SPREAD_08, leg_planner=seeded_legs(6),
                                             limits=limits, clock=FakeClock())
                for _ in range(2)]
        self.assertEqual(runs[0], runs[1])
        result = runs[0]
        self.assertInvariants(result)
        self.assertLessEqual(result.work.leg_calls, 300)
        self.assertLessEqual(result.work.frontier_peak, 20_000)
        self.assertLessEqual(result.complete_route.total_cost, result.b2_route.total_cost)


class CandidateProvenance(InvariantMixin, unittest.TestCase):
    def test_filtered_primary_is_never_offered(self):
        legs = FakeLegs()
        result = rps.find_prefix_b2_route(SPREAD_START, UNSAFE_PRIMARY_02, leg_planner=legs)
        self.assertInvariants(result)
        primary = viewing_target(UNSAFE_PRIMARY_02[0], CFG)
        surviving = cands(UNSAFE_PRIMARY_02[0], UNSAFE_PRIMARY_02, True)
        self.assertEqual(result.filtered_primary_obstacle_ids, (1,))
        self.assertNotIn(primary, surviving)
        self.assertTrue(all(t != primary for _s, t in legs.calls))
        position = result.complete_route.obstacle_order.index(1)
        self.assertEqual(result.complete_route.candidate_indices[position], 0)
        self.assertEqual(result.complete_route.target_poses[position], surviving[0])

    def test_filtered_primary_without_alternatives_is_an_empty_group(self):
        result = rps.find_prefix_b2_route(SPREAD_START, UNSAFE_PRIMARY_02, leg_planner=FakeLegs(),
                                          allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertEqual(result.empty_candidate_obstacle_ids, (1,))
        self.assertEqual(result.filtered_primary_obstacle_ids, ())


class BoundFactoryContract(InvariantMixin, unittest.TestCase):
    def test_called_once_with_surviving_tuples(self):
        calls = []

        def spy(candidates, config):
            calls.append((dict(candidates), config))
            return rlb.build_remaining_route_bound(candidates, config)

        obstacles = (OBS_A, OBS_B)
        result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=FakeLegs(), bound_factory=spy)
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], {o.obstacle_id: cands(o, obstacles, True) for o in obstacles})
        self.assertIs(calls[0][1], CFG)
        self.assertFalse(result.lower_bound_certified)   # a wrapper, however genuine its result
        self.assertInvariants(result)

    def test_default_factory_is_certified(self):
        result = rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B], leg_planner=FakeLegs())
        self.assertTrue(result.lower_bound_certified)
        self.assertIsNotNone(result.root_lower_bound)

    def test_malformed_bounds_are_rejected(self):
        def genuine(c, cfg):
            return rlb.build_remaining_route_bound(c, cfg)

        class Missing:
            def __init__(self, g):
                self.obstacle_ids, self.candidates = g.obstacle_ids, g.candidates

        class Returns(ScaledBound):
            def __init__(self, g, value):
                super().__init__(g, 0.0, 0.0)
                self.value = value

            def lower_bound(self, pose, remaining):
                return self.value

            def child_lower_bound(self, pose, oid, index, remaining):
                return self.value

        def drop_one(c, cfg):
            g = genuine(c, cfg)
            b = ScaledBound(g, 1.0, 0.0)
            b.obstacle_ids = g.obstacle_ids[:-1]
            return b

        def reorder(c, cfg):
            g = genuine(c, cfg)
            b = ScaledBound(g, 1.0, 0.0)
            b.candidates = tuple(tuple(reversed(group)) for group in g.candidates)
            return b

        factories = [
            lambda c, cfg: Missing(genuine(c, cfg)),
            drop_one,
            reorder,
            lambda c, cfg: Returns(genuine(c, cfg), float("nan")),
            lambda c, cfg: Returns(genuine(c, cfg), -1.0),
            lambda c, cfg: Returns(genuine(c, cfg), "1"),
            lambda c, cfg: Returns(genuine(c, cfg), True),
        ]
        for factory in factories:
            with self.assertRaises(rps.InvalidPrefixSearchInputError):
                rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B], leg_planner=FakeLegs(),
                                             bound_factory=factory)


class PhaseSeparation(InvariantMixin, unittest.TestCase):
    def test_b2_work_is_the_exact_phase_boundary_snapshot(self):
        b2 = rps.find_prefix_b2_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=seeded_legs(2))
        b3 = rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=seeded_legs(2))
        self.assertInvariants(b3)
        self.assertEqual(b3.b2_work, b2.work)
        self.assertEqual(b3.b2_route, b2.complete_route)
        self.assertEqual(b3.b2_work.frontier_peak, 0)
        self.assertGreater(b3.work.frontier_peak, 0)

    def test_b2_untried_roots_do_not_become_unresolved(self):
        b2 = rps.find_prefix_b2_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=FakeLegs())
        self.assertEqual(b2.unresolved_branch_count, 0)
        b3 = rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=FakeLegs())
        self.assertEqual(b3.unresolved_branch_count, 0)
        self.assertTrue(b3.optimality_proven)

    def test_cached_inconclusive_leg_is_reclassified_in_b3(self):
        obstacles = (OBS_A, OBS_FAR)
        x_id = root_order(START, obstacles)[0]
        x0 = cands(next(o for o in obstacles if o.obstacle_id == x_id), obstacles, False)[0]
        legs = FakeLegs(fail=lambda s, t: REASON_EXPANSION_LIMIT if (s, t) == (START, x0) else None)
        result = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs,
                                              allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertEqual(legs.counts[(START, x0)], 1)
        self.assertEqual(result.unresolved_branch_count, 1)
        self.assertFalse(result.optimality_proven)


class UnresolvedAccounting(InvariantMixin, unittest.TestCase):
    def test_b2_limit_counts_untried_child_roots(self):
        obstacles = (OBS_A, OBS_B, OBS_C)
        for alt in (False, True):
            roots = sum(len(cands(o, obstacles, alt)) for o in obstacles)
            result = rps.find_prefix_b2_route(START, obstacles, leg_planner=FakeLegs(fail=lambda s, t: REASON_NO_PATH),
                                              allow_alternative_viewing=alt,
                                              limits=rps.SearchLimits(max_leg_calls=1))
            self.assertInvariants(result)
            self.assertEqual(result.unresolved_branch_count, roots - 1)

    def test_b3_frontier_cap_counts_unpushed_children(self):
        result = rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=seeded_legs(8),
                                              limits=rps.SearchLimits(max_frontier_entries=5))
        self.assertInvariants(result)
        self.assertEqual(result.work_limit_hit, rps.LIMIT_FRONTIER)
        self.assertEqual(result.work.frontier_peak, 5)
        self.assertGreaterEqual(result.unresolved_branch_count, 5)
        self.assertEqual(result.complete_route, result.b2_route)

    def test_uncertified_unresolved_bound_is_unknown(self):
        result = rps.optimise_prefix_b3_route(START, [OBS_A, OBS_B, OBS_C], leg_planner=seeded_legs(8),
                                              limits=rps.SearchLimits(max_leg_calls=4),
                                              bound_factory=ZERO_BOUND)
        self.assertInvariants(result)
        self.assertGreater(result.unresolved_branch_count, 0)
        self.assertIsNone(result.min_unresolved_lower_bound)
        self.assertFalse(result.unresolved_bound_known)


class InputValidation(unittest.TestCase):
    def test_limits(self):
        bad = [dict(max_leg_calls=True), dict(max_leg_calls=0), dict(max_nodes=-1),
               dict(max_lattice_expansions=1.5), dict(max_frontier_entries=0),
               dict(wall_clock_checkpoint_limit_s=0), dict(wall_clock_checkpoint_limit_s=-1.0),
               dict(wall_clock_checkpoint_limit_s=float("nan")),
               dict(wall_clock_checkpoint_limit_s=float("inf")),
               dict(wall_clock_checkpoint_limit_s=True)]
        for kwargs in bad:
            with self.assertRaises(rps.InvalidPrefixSearchInputError, msg=kwargs):
                rps.SearchLimits(**kwargs)

    def test_arguments(self):
        nine = tuple(Obstacle(i, 20 * i, 20, "N") for i in range(1, 10))
        cases = [
            dict(obstacles=nine),
            dict(clock=1.0),
            dict(cancel_requested=True),
            dict(limits={"max_leg_calls": 1}),
            dict(bound_factory=None),
            dict(start_pose=(100, 100, 0)),
        ]
        for override in cases:
            kwargs = dict(start_pose=START, obstacles=[OBS_A], leg_planner=FakeLegs())
            kwargs.update(override)
            start, obstacles = kwargs.pop("start_pose"), kwargs.pop("obstacles")
            with self.assertRaises(rps.InvalidPrefixSearchInputError, msg=override):
                rps.find_prefix_b2_route(start, obstacles, **kwargs)
        with self.assertRaises(ero.InvalidExecutableRouteInputError):
            rps.find_prefix_b2_route(START, [OBS_A, OBS_A], leg_planner=FakeLegs())

    def test_malformed_leg_result_raises(self):
        def planner(s, t, o, c=CFG):
            return STMCommandPlan(True, (), (), 1.0, float("nan"), 0, end_of(t), None)

        with self.assertRaises(ero.ExecutableRouteInvariantError):
            rps.find_prefix_b2_route(START, [OBS_A], leg_planner=planner)


class DifferentialAgainstExhaustiveOptimizer(unittest.TestCase):
    """Same fake legs through ero's permutation search, the new search and a
    brute-force oracle. `no_path` only (ero cannot tell inconclusive failures
    apart); B2 route identity is not compared (the orderings differ by design)."""

    def test_forty_seeded_scenarios(self):
        compared = complete_count = 0
        for seed in range(40):
            rng = random.Random(1000 + seed)
            obstacles = tuple(rng.sample(POOL, 1 + seed % 4))
            for alt in (False, True):
                def make():
                    return seeded_legs(seed, fail_p=0.2)

                old_b3 = ero.optimise_executable_b3_route(START, obstacles, leg_planner=make(),
                                                          allow_alternative_viewing=alt)
                old_b2 = ero.find_executable_b2_route(START, obstacles, leg_planner=make(),
                                                      allow_alternative_viewing=alt)
                new_b3 = rps.optimise_prefix_b3_route(START, obstacles, leg_planner=make(),
                                                      allow_alternative_viewing=alt)
                new_b2 = rps.find_prefix_b2_route(START, obstacles, leg_planner=make(),
                                                  allow_alternative_viewing=alt)
                oracle, (depth, partial_cost) = brute_force(START, obstacles, alt, make())
                label = f"seed={seed} alt={alt}"
                for result in (new_b2, new_b3):
                    self.assertEqual(invariant_violations(result), [], label)
                old_complete = old_b3.is_valid and old_b3.is_complete
                self.assertEqual(new_b3.route_status == rps.ROUTE_COMPLETE, old_complete, label)
                self.assertEqual(new_b2.route_status == rps.ROUTE_COMPLETE,
                                 old_b2.is_valid and old_b2.is_complete, label)
                self.assertEqual(old_complete, oracle is not None, label)
                if old_complete:
                    complete_count += 1
                    self.assertEqual(new_b3.complete_route.total_cost, old_b3.total_cost, label)
                    self.assertEqual(new_b3.complete_route.total_cost, oracle, label)
                    self.assertTrue(new_b3.optimality_proven, label)
                    replay = make()
                    pose, total = START, 0.0
                    for leg in new_b2.complete_route.legs:
                        plan = replay.outcome(pose, leg.target_pose)
                        self.assertTrue(plan.is_valid, label)
                        pose, total = plan.final_pose, total + plan.total_cost
                    self.assertEqual(total, new_b2.complete_route.total_cost, label)
                else:
                    self.assertTrue(new_b3.complete_route_proven_impossible, label)
                    self.assertTrue(new_b2.complete_route_proven_impossible, label)
                    if alt and old_b3.is_valid:
                        partial = new_b3.best_partial_route
                        self.assertEqual((len(partial.obstacle_order), partial.total_cost),
                                         (len(old_b3.obstacle_order), old_b3.total_cost), label)
                        self.assertEqual((len(partial.obstacle_order), partial.total_cost),
                                         (depth, partial_cost), label)
                compared += 1
        self.assertEqual(compared, 80)
        self.assertGreater(complete_count, 20)
        self.assertLess(complete_count, 80)


class MutantsAreDetected(unittest.TestCase):
    """Each decision seam is broken on purpose; the scenario guarding it must
    then report a violation, proving the guarantee is genuinely exercised."""

    def assertDetected(self, scenario, **patches):
        with mock.patch.multiple(rps, **patches):
            try:
                violations = scenario()
            except Exception as exc:  # a crash also counts as detection
                violations = [repr(exc)]
        self.assertNotEqual(violations, [], f"mutant {sorted(patches)} survived")

    def test_m1_lazy_fallback(self):
        self.assertDetected(scenario_lazy_fallback, _ordered_children=lambda children: sorted(
            (c for c in children if c[1] == 0), key=lambda c: (c[1], c[2], c[0])))

    def test_m2_completeness(self):
        self.assertDetected(scenario_expansion_limit, _is_definitive_failure=lambda reason: True)

    def test_m3_incumbent(self):
        self.assertDetected(scenario_b3_never_worse, _improves_incumbent=lambda g, cost: True)

    def test_m4_aggressive_pruning_add_to_f(self):
        self.assertDetected(scenario_pruning_keeps_optimum, _should_prune=lambda f, cost: f + 1.0 >= cost)

    def test_m4b_aggressive_pruning_lower_threshold(self):
        self.assertDetected(scenario_pruning_keeps_optimum, _should_prune=lambda f, cost: f >= cost - 1.0)

    def test_m5_budget_honesty(self):
        original = rps._finalise_flags

        def mutant(mode, complete, stop, certified, violations, unresolved, cost, empty):
            # Treat a limit stop as a clean exhaustive search.
            return original(mode, complete, rps.STOP_EXHAUSTED, certified, violations, [], cost, empty)

        self.assertDetected(scenario_budget_wording, _finalise_flags=mutant)

    def test_m6_uncertified_pruning(self):
        self.assertDetected(scenario_uncertified_cannot_prune,
                            _pruning_allowed=lambda certified, violations: violations == 0)

    def test_m7_violation_handling(self):
        self.assertDetected(scenario_violation_disables_pruning,
                            _pruning_allowed=lambda certified, violations: certified,
                            _reopen_bound_prunes=lambda phase: None)


class IncumbentObserver(InvariantMixin, unittest.TestCase):
    """`on_incumbent` only observes: it reports each complete incumbent and
    changes nothing the search computes."""

    @staticmethod
    def observed(search, obstacles, legs, **kwargs):
        seen = []
        result = search(START, obstacles, leg_planner=legs,
                        on_incumbent=lambda route, phase: seen.append((phase, route)), **kwargs)
        return result, seen

    def test_hook_changes_no_result_field(self):
        for seed in range(12):
            obstacles = tuple(random.Random(seed).sample(POOL, 2 + seed % 3))
            alt = bool(seed % 2)
            for search in (rps.find_prefix_b2_route, rps.optimise_prefix_b3_route):
                plain = search(START, obstacles, leg_planner=seeded_legs(seed, fail_p=0.1),
                               allow_alternative_viewing=alt)
                hooked, _seen = self.observed(search, obstacles, seeded_legs(seed, fail_p=0.1),
                                              allow_alternative_viewing=alt)
                self.assertEqual(dataclasses.replace(plain, elapsed_s=0.0),
                                 dataclasses.replace(hooked, elapsed_s=0.0))

    def test_b2_first_then_strictly_cheaper_b3_routes(self):
        obstacles, _x_id, legs = suboptimal_b2_setup(delta=5.0)
        result, seen = self.observed(rps.optimise_prefix_b3_route, obstacles, legs,
                                     allow_alternative_viewing=False)
        self.assertInvariants(result)
        self.assertEqual([phase for phase, _ in seen], ["b2", "b3"])
        self.assertEqual(seen[0][1], result.b2_route)
        self.assertEqual(seen[-1][1], result.complete_route)
        costs = [route.total_cost for _, route in seen]
        self.assertEqual(costs, sorted(costs, reverse=True))
        self.assertLess(costs[-1], costs[0])
        self.assertEqual(len(seen), 1 + result.work.incumbent_updates)

    def test_only_b2_is_reported_when_b3_cannot_improve(self):
        result, seen = self.observed(rps.optimise_prefix_b3_route, [OBS_A], FakeLegs(),
                                     allow_alternative_viewing=False)
        self.assertEqual([phase for phase, _ in seen], ["b2"])
        self.assertEqual(seen[0][1], result.complete_route)

    def test_b2_mode_reports_its_route_once(self):
        result, seen = self.observed(rps.find_prefix_b2_route, [OBS_A, OBS_B], FakeLegs())
        self.assertEqual([phase for phase, _ in seen], ["b2"])
        self.assertEqual(seen[0][1], result.complete_route)

    def test_no_report_without_a_complete_route(self):
        for search in (rps.find_prefix_b2_route, rps.optimise_prefix_b3_route):
            _result, seen = self.observed(
                search, [OBS_A, OBS_B], FakeLegs(fail=lambda s, t: REASON_NO_PATH))
            self.assertEqual(seen, [])
            _result, seen = self.observed(search, [OBS_A, OBS_UNSAFE, OBS_B], FakeLegs())
            self.assertEqual(seen, [])     # partial route only: an obstacle has no safe candidate
            _result, seen = self.observed(search, [], FakeLegs())
            self.assertEqual(seen, [])     # zero obstacles: nothing to report

    def test_a_hook_exception_propagates_and_earlier_reports_stay_delivered(self):
        obstacles, _x_id, legs = suboptimal_b2_setup(delta=5.0)
        seen = []

        def hook(route, phase):
            seen.append(phase)
            if phase == "b3":
                raise RuntimeError("observer failed")

        with self.assertRaises(RuntimeError):
            rps.optimise_prefix_b3_route(START, obstacles, leg_planner=legs, on_incumbent=hook,
                                         allow_alternative_viewing=False)
        self.assertEqual(seen, ["b2", "b3"])

    def test_non_callable_hook_is_rejected(self):
        with self.assertRaises(rps.InvalidPrefixSearchInputError):
            rps.find_prefix_b2_route(START, [OBS_A], leg_planner=FakeLegs(), on_incumbent=42)


class RealPipelineSmoke(unittest.TestCase):
    """One obstacle through the real lattice -> compression -> generation
    pipeline (fast); larger real-pipeline comparisons belong to the
    validation PR."""

    def test_matches_executable_route_optimizer(self):
        old_b2 = ero.find_executable_b2_route(START, [OBS_A], allow_alternative_viewing=False)
        old_b3 = ero.optimise_executable_b3_route(START, [OBS_A], allow_alternative_viewing=False)
        new_b2 = rps.find_prefix_b2_route(START, [OBS_A], allow_alternative_viewing=False)
        new_b3 = rps.optimise_prefix_b3_route(START, [OBS_A], allow_alternative_viewing=False)
        for old, new in ((old_b2, new_b2), (old_b3, new_b3)):
            self.assertEqual(invariant_violations(new), [])
            self.assertTrue(old.is_valid and old.is_complete)
            self.assertEqual(new.route_status, rps.ROUTE_COMPLETE)
            self.assertEqual(new.complete_route.total_cost, old.total_cost)
            old_commands = tuple(c.stm_command for leg in old.legs for c in leg.command_plan.commands)
            self.assertEqual(new.complete_route.stm_commands, old_commands)
            self.assertGreater(len(old_commands), 0)
            self.assertEqual(new.bound_violation_count, 0)
        self.assertTrue(new_b3.optimality_proven)


if __name__ == "__main__":
    unittest.main()
