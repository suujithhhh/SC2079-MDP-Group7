"""Original Roadmap Step 5 -- executable obstacle ordering.

Not connected to planner_interface.py, b3_optimizer.py, or any other
consumer. Most tests inject a deterministic `FakeLegPlanner` so route
logic (ordering, chaining, caching, partial-route selection, pruning,
validation) is tested quickly and precisely, without running the real
(and, for some pose pairs, slow) state-lattice search. RealPipelineIntegration
and GeneratedCommandsIndependentlyValid are the focused exceptions that
exercise the actual search -> compression -> generation pipeline against
DEFAULT_PLANNER_CONFIG, kept small (1-2 obstacles) to stay fast.

Tests that rely on the branch-and-bound admissible-lower-bound pruning
(`LegLowerBoundPruning`, and `B3LowestCost`) assign fake leg costs derived
from the real `state_lattice._heuristic_cm` lower bound plus a margin,
rather than arbitrary numbers -- an injected leg cost below that bound
would violate the same admissibility assumption the real planner
guarantees, and could be (correctly) pruned before ever being queried.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import dataclasses
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import executable_route_optimizer as ero  # noqa: E402
import motion_primitives as mp  # noqa: E402
import swept_collision as sc  # noqa: E402
import targets  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
from models import Obstacle, Pose  # noqa: E402
from state_lattice import _heuristic_cm  # noqa: E402
from stm_command_generation import STMCommandPlan  # noqa: E402

DEFAULT_CONFIG = DEFAULT_PLANNER_CONFIG
LOWER_BOUND_MARGIN = 2.0


def _min_admissible_cost(start: Pose, target: Pose, margin: float = LOWER_BOUND_MARGIN,
                         config=None) -> float:
    """A fake leg cost that respects the real admissible lower bound between
    two poses, plus a safety margin -- so branch-and-bound pruning never
    incorrectly drops a leg these tests expect to be queried."""
    return _heuristic_cm(start, target, config or DEFAULT_CONFIG) + margin


class FakeLegPlanner:
    """Deterministic, injectable stand-in for plan_executable_leg.

    Records every (start_pose, target_pose) query and how many times it
    was made, so tests can assert on caching. Unregistered pairs default to
    a valid leg (unless `default_valid=False`) whose final_pose is
    `target_pose` offset by a fixed, non-zero amount -- proving that any
    code chaining off this fake's result cannot possibly be silently
    substituting the ideal target. `default_valid=False` is for tests that
    must guarantee every candidate except a short, explicit allow-list is
    unreachable (e.g. partial-route selection with alternative viewing
    candidates enabled).
    """

    def __init__(self, default_cost: float = 10.0, default_offset=(0.4, -0.3, 0.05),
                 default_valid: bool = True):
        self.default_cost = default_cost
        self.default_offset = default_offset
        self.default_valid = default_valid
        self.responses: dict = {}
        self.calls: list = []
        self.call_counts: dict = {}

    def _offset_pose(self, target: Pose) -> Pose:
        dx, dy, dh = self.default_offset
        return Pose(target.x + dx, target.y + dy, target.heading + dh)

    def set_valid(self, start: Pose, target: Pose, cost: float = None, final_pose: Pose = None):
        cost = self.default_cost if cost is None else cost
        final_pose = self._offset_pose(target) if final_pose is None else final_pose
        plan = STMCommandPlan(
            is_valid=True, commands=(), waypoints=(), total_distance_cm=cost,
            total_cost=cost, expansions=0, final_pose=final_pose, reason=None,
        )
        self.responses[(start, target)] = plan
        return plan

    def set_invalid(self, start: Pose, target: Pose, reason: str = "no_path"):
        plan = STMCommandPlan(
            is_valid=False, commands=(), waypoints=(), total_distance_cm=0.0,
            total_cost=0.0, expansions=0, final_pose=None, reason=reason,
        )
        self.responses[(start, target)] = plan
        return plan

    def __call__(self, start_pose, target_pose, obstacles, config=DEFAULT_CONFIG):
        key = (start_pose, target_pose)
        self.calls.append(key)
        self.call_counts[key] = self.call_counts.get(key, 0) + 1
        if key in self.responses:
            return self.responses[key]
        if not self.default_valid:
            return STMCommandPlan(
                is_valid=False, commands=(), waypoints=(), total_distance_cm=0.0,
                total_cost=0.0, expansions=0, final_pose=None, reason="no_path",
            )
        return STMCommandPlan(
            is_valid=True, commands=(), waypoints=(), total_distance_cm=self.default_cost,
            total_cost=self.default_cost, expansions=0,
            final_pose=self._offset_pose(target_pose), reason=None,
        )


def always_invalid_planner(start_pose, target_pose, obstacles, config=DEFAULT_CONFIG):
    return STMCommandPlan(
        is_valid=False, commands=(), waypoints=(), total_distance_cm=0.0,
        total_cost=0.0, expansions=0, final_pose=None, reason="no_path",
    )


START = Pose(100.0, 100.0, 0.0)
OBSTACLE_A = Obstacle(obstacle_id=1, x=150, y=100, image_face="W")
OBSTACLE_B = Obstacle(obstacle_id=2, x=100, y=150, image_face="S")
OBSTACLE_C = Obstacle(obstacle_id=3, x=100, y=60, image_face="N")
# Far from START (~55 cm) but still footprint-safe, for lower-bound pruning tests.
FAR_OBSTACLE = Obstacle(obstacle_id=4, x=100, y=190, image_face="S")
# Very close to START and roughly on the way to FAR_OBSTACLE, so visiting
# it first is not a detour -- needed so a genuinely cheap two-leg route can
# undercut the single-leg lower bound straight to FAR_OBSTACLE (two goal-
# tolerance subtractions along a near-straight path beat one). Its own
# footprint stays clear of FAR_OBSTACLE and vice versa.
NEAR_OBSTACLE = Obstacle(obstacle_id=5, x=55, y=100, image_face="E")
# Facing away from the arena: its primary viewing pose is unsafe (out of bounds).
UNSAFE_OBSTACLE = Obstacle(obstacle_id=42, x=0, y=100, image_face="W")


class DefaultLegPlannerPipeline(unittest.TestCase):
    def test_calls_search_then_compress_via_generation(self):
        target = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        with mock.patch(
            "executable_route_optimizer.plan_lattice_path"
        ) as fake_search, mock.patch(
            "executable_route_optimizer.generate_stm_command_plan"
        ) as fake_generate:
            fake_search.return_value = "LATTICE_PLAN_SENTINEL"
            fake_generate.return_value = "COMMAND_PLAN_SENTINEL"
            result = ero.plan_executable_leg(START, target, [OBSTACLE_A], DEFAULT_CONFIG)

        fake_search.assert_called_once_with(START, target, (OBSTACLE_A,), DEFAULT_CONFIG)
        fake_generate.assert_called_once_with(
            "LATTICE_PLAN_SENTINEL", (OBSTACLE_A,), DEFAULT_CONFIG
        )
        self.assertEqual(result, "COMMAND_PLAN_SENTINEL")


class B2FirstCompleteOrder(unittest.TestCase):
    def test_returns_input_order_when_it_completes(self):
        fake = FakeLegPlanner()
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertTrue(result.is_valid)
        self.assertTrue(result.is_complete)
        self.assertEqual(result.obstacle_order, (1, 2))
        self.assertEqual(result.orders_considered, 1)
        self.assertEqual(result.pruned_orders, 0)


class B3LowestCost(unittest.TestCase):
    def test_chooses_the_cheaper_complete_order(self):
        # OBSTACLE_A and OBSTACLE_C sit at different distances from START, so
        # (unlike OBSTACLE_A/OBSTACLE_B, which are mirror-symmetric) the two
        # visiting orders have genuinely different admissible lower bounds --
        # letting fake costs respect real geometry while still differing.
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        target_c = targets.viewing_target(OBSTACLE_C, DEFAULT_CONFIG)
        fake = FakeLegPlanner()
        final_a = fake.set_valid(
            START, target_a, cost=_min_admissible_cost(START, target_a)
        ).final_pose
        final_c = fake.set_valid(
            START, target_c, cost=_min_admissible_cost(START, target_c)
        ).final_pose
        fake.set_valid(final_a, target_c, cost=_min_admissible_cost(final_a, target_c))  # (A,C)
        fake.set_valid(final_c, target_a, cost=_min_admissible_cost(final_c, target_a))  # (C,A)

        result = ero.optimise_executable_b3_route(
            START, [OBSTACLE_A, OBSTACLE_C], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertTrue(result.is_valid and result.is_complete)
        self.assertEqual(result.obstacle_order, (3, 1))  # (C,A) is the cheaper order
        self.assertEqual(result.valid_orders, 2)
        self.assertEqual(result.rejected_orders, 0)
        self.assertEqual(result.pruned_orders, 0)


class PrimaryTriedBeforeAlternatives(unittest.TestCase):
    def test_ties_resolve_to_the_primary_candidate(self):
        # All candidates default to the same cost -- the deterministic
        # tie-break must land on the primary (first-tried) candidate.
        fake = FakeLegPlanner(default_cost=10.0)
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A], DEFAULT_CONFIG, leg_planner=fake,
            allow_alternative_viewing=True,
        )
        self.assertTrue(result.is_valid)
        primary = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        self.assertEqual(result.legs[0].target_pose, primary)

    def test_alternative_used_when_primary_fails(self):
        primary = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        fake = FakeLegPlanner(default_cost=10.0)
        fake.set_invalid(START, primary)
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A], DEFAULT_CONFIG, leg_planner=fake,
            allow_alternative_viewing=True,
        )
        self.assertTrue(result.is_valid)
        self.assertNotEqual(result.legs[0].target_pose, primary)


class FailedLegPrunesPrefix(unittest.TestCase):
    def test_order_failing_partway_is_abandoned_for_a_working_one(self):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        target_b = targets.viewing_target(OBSTACLE_B, DEFAULT_CONFIG)
        fake = FakeLegPlanner()
        final_a = fake.set_valid(START, target_a).final_pose
        fake.set_invalid(final_a, target_b)  # (A,B) fails on its 2nd leg
        final_b = fake.set_valid(START, target_b).final_pose
        fake.set_valid(final_b, target_a)  # (B,A) fully works

        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertTrue(result.is_valid and result.is_complete)
        self.assertEqual(result.obstacle_order, (2, 1))
        self.assertEqual(result.rejected_orders, 1)


class NextLegUsesEstimatedFinalPose(unittest.TestCase):
    def test_second_leg_starts_at_first_legs_final_pose_not_its_target(self):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        target_b = targets.viewing_target(OBSTACLE_B, DEFAULT_CONFIG)
        fake = FakeLegPlanner()  # default offset makes final_pose != target
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertTrue(result.is_valid)
        leg_a_final = result.legs[0].command_plan.final_pose
        self.assertNotEqual(leg_a_final, target_a)  # proves the offset is real
        # The exact (start, target) pair recorded for leg B must show the
        # estimated final pose of leg A as its start -- not target_a.
        self.assertIn((leg_a_final, target_b), fake.calls)
        self.assertNotIn((target_a, target_b), fake.calls)


class LegCaching(unittest.TestCase):
    def test_identical_leg_queries_are_not_recomputed(self):
        fake = FakeLegPlanner()
        result = ero.optimise_executable_b3_route(
            START, [OBSTACLE_A, OBSTACLE_B, OBSTACLE_C], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertTrue(result.is_valid)
        # 3 obstacles -> 6 permutations, but only 3 distinct first-leg
        # queries (start -> A/B/C) are possible; each must be cached.
        self.assertTrue(all(count == 1 for count in fake.call_counts.values()))
        self.assertGreater(fake.call_counts[
            (START, targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG))
        ], 0)


class TotalsAndFinalPose(unittest.TestCase):
    def test_totals_equal_summed_leg_totals(self):
        fake = FakeLegPlanner()
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertAlmostEqual(
            result.total_distance_cm,
            sum(leg.command_plan.total_distance_cm for leg in result.legs),
            places=9,
        )
        self.assertAlmostEqual(
            result.total_cost,
            sum(leg.command_plan.total_cost for leg in result.legs),
            places=9,
        )

    def test_final_pose_is_last_legs_estimated_final_pose(self):
        fake = FakeLegPlanner()
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertEqual(result.final_pose, result.legs[-1].command_plan.final_pose)


class PartialRouteSelection(unittest.TestCase):
    """Alternatives must be enabled for a partial route to ever be
    returned (see PartialRouteRequiresAlternatives below), so these tests
    use `default_valid=False` fakes: every pair not explicitly registered
    -- including every alternate viewing candidate -- is unreachable,
    guaranteeing no order (and no fallback candidate) can accidentally
    complete."""

    def _configure_no_complete_route(self, fake):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        target_b = targets.viewing_target(OBSTACLE_B, DEFAULT_CONFIG)
        target_c = targets.viewing_target(OBSTACLE_C, DEFAULT_CONFIG)
        final_a = fake.set_valid(START, target_a, cost=1.0).final_pose
        final_b = fake.set_valid(START, target_b, cost=1.0).final_pose
        fake.set_valid(START, target_c, cost=1.0)
        fake.set_valid(final_a, target_b, cost=1.0)          # (A,B) len2 cost2
        fake.set_valid(final_b, target_a, cost=0.5)          # (B,A) len2 cost1.5
        # Everything else -- including every alternate viewing candidate for
        # every obstacle, and any length-3 continuation -- defaults to
        # invalid, so no complete route and no accidental alternate-based
        # route can ever form.
        return target_a, target_b, target_c

    def test_prefers_more_visited_obstacles_and_lower_cost_tiebreak(self):
        fake = FakeLegPlanner(default_valid=False)
        self._configure_no_complete_route(fake)
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A, OBSTACLE_B, OBSTACLE_C], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=True,
        )
        self.assertTrue(result.is_valid)
        self.assertFalse(result.is_complete)
        self.assertEqual(len(result.obstacle_order), 2)
        self.assertEqual(result.obstacle_order, (2, 1))  # (B,A): cheapest length-2 chain
        self.assertAlmostEqual(result.total_cost, 1.5, places=9)

    def test_skipped_obstacle_ids_are_correct_and_deterministic(self):
        fake = FakeLegPlanner(default_valid=False)
        self._configure_no_complete_route(fake)
        result = ero.optimise_executable_b3_route(
            START, [OBSTACLE_A, OBSTACLE_B, OBSTACLE_C], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=True,
        )
        self.assertFalse(result.is_complete)
        self.assertEqual(result.skipped_obstacle_ids, (3,))
        self.assertIsNotNone(result.failure_reason)


class PartialRouteRequiresAlternatives(unittest.TestCase):
    """With alternatives disabled, a reachable prefix must never be
    exposed as a partial route -- an incomplete order is simply invalid."""

    def _configure_reachable_prefix_then_dead_end(self, fake):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        target_b = targets.viewing_target(OBSTACLE_B, DEFAULT_CONFIG)
        fake.set_valid(START, target_a, cost=1.0)  # A alone is reachable
        fake.set_valid(START, target_b, cost=1.0)  # B alone is reachable
        # Neither order can ever extend past its first obstacle.
        return target_a, target_b

    def test_b2_returns_invalid_not_partial_when_alternatives_disabled(self):
        fake = FakeLegPlanner(default_valid=False)
        self._configure_reachable_prefix_then_dead_end(fake)
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertFalse(result.is_valid)
        self.assertFalse(result.is_complete)
        self.assertEqual(result.legs, ())
        self.assertEqual(result.skipped_obstacle_ids, ())
        # A and B are each individually reachable -- the failure message
        # must not falsely claim no obstacle can be reached at all.
        self.assertIn("No complete executable route", result.failure_reason)
        self.assertNotIn("No obstacle can be reached", result.failure_reason)

    def test_b3_returns_invalid_not_partial_when_alternatives_disabled(self):
        fake = FakeLegPlanner(default_valid=False)
        self._configure_reachable_prefix_then_dead_end(fake)
        result = ero.optimise_executable_b3_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertFalse(result.is_valid)
        self.assertFalse(result.is_complete)
        self.assertEqual(result.legs, ())
        self.assertEqual(result.skipped_obstacle_ids, ())
        # A and B are each individually reachable -- the failure message
        # must not falsely claim no obstacle can be reached at all.
        self.assertIn("No complete executable route", result.failure_reason)
        self.assertNotIn("No obstacle can be reached", result.failure_reason)


class NoReachableObstacle(unittest.TestCase):
    def test_returns_clear_invalid_result(self):
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=always_invalid_planner, allow_alternative_viewing=True,
        )
        self.assertFalse(result.is_valid)
        self.assertEqual(result.legs, ())
        self.assertIsNotNone(result.failure_reason)


class EmptyObstacles(unittest.TestCase):
    def test_returns_valid_complete_zero_leg_result(self):
        for search in (ero.find_executable_b2_route, ero.optimise_executable_b3_route):
            with self.subTest(search=search.__name__):
                result = search(START, [], DEFAULT_CONFIG)
                self.assertTrue(result.is_valid)
                self.assertTrue(result.is_complete)
                self.assertEqual(result.legs, ())
                self.assertEqual(result.final_pose, START)
                self.assertEqual(result.total_distance_cm, 0.0)
                self.assertEqual(result.total_cost, 0.0)
                self.assertEqual(result.rejected_orders, 0)
                self.assertEqual(result.pruned_orders, 0)
                self.assertEqual(
                    result.orders_considered,
                    result.valid_orders + result.rejected_orders + result.pruned_orders,
                )


class NoMutation(unittest.TestCase):
    def test_obstacle_list_generator_and_config_untouched(self):
        obstacles_list = [OBSTACLE_A, OBSTACLE_B]
        snapshot = list(obstacles_list)
        config_snapshot = dataclasses.replace(DEFAULT_CONFIG)
        fake = FakeLegPlanner()

        ero.find_executable_b2_route(
            START, obstacles_list, DEFAULT_CONFIG, leg_planner=fake,
            allow_alternative_viewing=False,
        )
        self.assertEqual(obstacles_list, snapshot)
        self.assertEqual(DEFAULT_CONFIG, config_snapshot)

        def obstacle_gen():
            return iter([OBSTACLE_A, OBSTACLE_B])

        fake2 = FakeLegPlanner()
        result = ero.find_executable_b2_route(
            START, obstacle_gen(), DEFAULT_CONFIG, leg_planner=fake2,
            allow_alternative_viewing=False,
        )
        self.assertTrue(result.is_valid)


class ImmutableResults(unittest.TestCase):
    def test_result_and_legs_are_immutable(self):
        fake = FakeLegPlanner()
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A], DEFAULT_CONFIG, leg_planner=fake,
            allow_alternative_viewing=False,
        )
        self.assertIsInstance(result.legs, tuple)
        self.assertIsInstance(result.obstacle_order, tuple)
        self.assertIsInstance(result.target_poses, tuple)
        with self.assertRaises(Exception):
            result.is_valid = False
        with self.assertRaises(Exception):
            result.legs[0].obstacle_id = 999


class DeterministicRepeatability(unittest.TestCase):
    def test_repeated_calls_produce_identical_results(self):
        result_a = ero.optimise_executable_b3_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=FakeLegPlanner(), allow_alternative_viewing=False,
        )
        result_b = ero.optimise_executable_b3_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=FakeLegPlanner(), allow_alternative_viewing=False,
        )
        self.assertEqual(result_a.obstacle_order, result_b.obstacle_order)
        self.assertEqual(result_a.total_cost, result_b.total_cost)
        self.assertEqual(result_a.final_pose, result_b.final_pose)


class NoForbiddenFields(unittest.TestCase):
    def test_no_mission_protocol_timing_or_snap_fields(self):
        result_fields = {f.name for f in dataclasses.fields(ero.ExecutableRouteResult)}
        leg_fields = {f.name for f in dataclasses.fields(ero.ExecutableRouteLeg)}
        forbidden = {
            "seq", "mission_id", "json", "estimated_time_s", "estimated_time_status",
            "timing", "ack", "retry", "snap",
        }
        self.assertEqual(result_fields & forbidden, set())
        self.assertEqual(leg_fields & forbidden, set())


class OrderAccountingInvariant(unittest.TestCase):
    """`orders_considered == valid_orders + rejected_orders + pruned_orders`
    must hold for every result, and a pruned order must never be double
    counted as rejected."""

    def test_accounting_holds_for_a_fully_rejected_search(self):
        result = ero.optimise_executable_b3_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            leg_planner=always_invalid_planner, allow_alternative_viewing=False,
        )
        self.assertFalse(result.is_valid)
        self.assertEqual(result.pruned_orders, 0)
        self.assertEqual(result.rejected_orders, result.orders_considered)
        self.assertEqual(
            result.orders_considered,
            result.valid_orders + result.rejected_orders + result.pruned_orders,
        )


class DuplicateObstacleIdValidation(unittest.TestCase):
    def test_duplicate_obstacle_id_raises(self):
        duplicate_id = Obstacle(obstacle_id=1, x=999, y=999, image_face="N")
        for search in (ero.find_executable_b2_route, ero.optimise_executable_b3_route):
            with self.subTest(search=search.__name__):
                with self.assertRaises(ero.InvalidExecutableRouteInputError):
                    search(START, [OBSTACLE_A, duplicate_id], DEFAULT_CONFIG)


class InjectedLegResultValidation(unittest.TestCase):
    """A `leg_planner` that claims `is_valid=True` but returns an internally
    malformed result must raise `ExecutableRouteInvariantError` rather than
    silently corrupting route aggregation or branch-and-bound."""

    def _planner_returning(self, plan):
        def planner(start_pose, target_pose, obstacles, config=DEFAULT_CONFIG):
            return plan
        return planner

    def _run(self, plan):
        return ero.find_executable_b2_route(
            START, [OBSTACLE_A], DEFAULT_CONFIG,
            leg_planner=self._planner_returning(plan),
            allow_alternative_viewing=False,
        )

    def test_missing_final_pose_raises(self):
        bad = STMCommandPlan(
            is_valid=True, commands=(), waypoints=(), total_distance_cm=1.0,
            total_cost=1.0, expansions=0, final_pose=None, reason=None,
        )
        with self.assertRaises(ero.ExecutableRouteInvariantError):
            self._run(bad)

    def test_negative_cost_raises(self):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        bad = STMCommandPlan(
            is_valid=True, commands=(), waypoints=(), total_distance_cm=1.0,
            total_cost=-1.0, expansions=0, final_pose=target_a, reason=None,
        )
        with self.assertRaises(ero.ExecutableRouteInvariantError):
            self._run(bad)

    def test_nan_cost_raises(self):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        bad = STMCommandPlan(
            is_valid=True, commands=(), waypoints=(), total_distance_cm=1.0,
            total_cost=float("nan"), expansions=0, final_pose=target_a, reason=None,
        )
        with self.assertRaises(ero.ExecutableRouteInvariantError):
            self._run(bad)

    def test_infinite_distance_raises(self):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        bad = STMCommandPlan(
            is_valid=True, commands=(), waypoints=(), total_distance_cm=float("inf"),
            total_cost=1.0, expansions=0, final_pose=target_a, reason=None,
        )
        with self.assertRaises(ero.ExecutableRouteInvariantError):
            self._run(bad)

    def test_boolean_cost_raises(self):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        bad = STMCommandPlan(
            is_valid=True, commands=(), waypoints=(), total_distance_cm=1.0,
            total_cost=True, expansions=0, final_pose=target_a, reason=None,
        )
        with self.assertRaises(ero.ExecutableRouteInvariantError):
            self._run(bad)

    def test_non_numeric_cost_raises(self):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        bad = STMCommandPlan(
            is_valid=True, commands=(), waypoints=(), total_distance_cm=1.0,
            total_cost="fast", expansions=0, final_pose=target_a, reason=None,
        )
        with self.assertRaises(ero.ExecutableRouteInvariantError):
            self._run(bad)

    def test_non_none_reason_raises(self):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        bad = STMCommandPlan(
            is_valid=True, commands=(), waypoints=(), total_distance_cm=1.0,
            total_cost=1.0, expansions=0, final_pose=target_a, reason="unsafe_goal",
        )
        with self.assertRaises(ero.ExecutableRouteInvariantError):
            self._run(bad)

    def test_ordinary_invalid_leg_does_not_raise(self):
        result = self._run(STMCommandPlan(
            is_valid=False, commands=(), waypoints=(), total_distance_cm=0.0,
            total_cost=0.0, expansions=0, final_pose=None, reason="no_path",
        ))
        self.assertFalse(result.is_valid)


class SafeTargetFiltering(unittest.TestCase):
    def test_unsafe_viewing_pose_is_filtered_without_invoking_leg_planner(self):
        fake = FakeLegPlanner()
        result = ero.find_executable_b2_route(
            START, [UNSAFE_OBSTACLE], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertFalse(result.is_valid)
        self.assertEqual(result.legs, ())
        self.assertEqual(fake.calls, [])


class LegLowerBoundPruning(unittest.TestCase):
    """B3's branch-and-bound must skip a leg whose admissible lower bound
    already cannot beat the incumbent -- without ever calling the leg
    planner for it -- while never skipping a leg that still could."""

    def test_branch_that_cannot_beat_incumbent_is_skipped_without_invoking_leg_planner(self):
        # A tighter margin than the module default: NEAR_OBSTACLE sits close
        # to the START-FAR_OBSTACLE line, so the genuine two-leg lower bound
        # already undercuts the direct one-leg bound by only a few cm (two
        # goal-tolerance subtractions beat one along a near-straight path) --
        # a smaller, still-admissible margin keeps that headroom positive.
        # This geometry's headroom was tuned at a 5 cm goal tolerance, so the
        # test pins it: it checks the pruning mechanism, not the default.
        config = dataclasses.replace(DEFAULT_CONFIG, lattice_goal_position_tolerance_cm=5.0)
        margin = 1.0
        target_near = targets.viewing_target(NEAR_OBSTACLE, config)
        target_far = targets.viewing_target(FAR_OBSTACLE, config)
        fake = FakeLegPlanner()
        final_near = fake.set_valid(
            START, target_near, cost=_min_admissible_cost(START, target_near, margin, config)
        ).final_pose
        fake.set_valid(
            final_near, target_far, cost=_min_admissible_cost(final_near, target_far, margin, config)
        )  # (NEAR, FAR): a cheap, complete incumbent route.

        # (FAR, NEAR) would ALSO complete if it were ever explored --
        # registering both its legs proves the prune below cuts a genuinely
        # valid, just costlier, order rather than a dead end.
        final_far = fake.set_valid(
            START, target_far, cost=_min_admissible_cost(START, target_far, margin, config)
        ).final_pose
        fake.set_valid(
            final_far, target_near, cost=_min_admissible_cost(final_far, target_near, margin, config)
        )

        result = ero.optimise_executable_b3_route(
            START, [NEAR_OBSTACLE, FAR_OBSTACLE], config,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertTrue(result.is_valid and result.is_complete)
        self.assertEqual(result.obstacle_order, (5, 4))  # (NEAR, FAR), tried first
        # The reverse order's first leg (START -> far target) has an
        # admissible lower bound that alone already exceeds the incumbent,
        # so neither of its legs may ever reach the leg planner.
        self.assertNotIn((START, target_far), fake.calls)
        self.assertNotIn((final_far, target_near), fake.calls)
        self.assertEqual(result.valid_orders, 1)
        self.assertEqual(result.pruned_orders, 1)
        self.assertEqual(result.rejected_orders, 0)
        self.assertEqual(
            result.orders_considered,
            result.valid_orders + result.rejected_orders + result.pruned_orders,
        )

    def test_branch_that_could_still_beat_incumbent_is_not_skipped(self):
        target_a = targets.viewing_target(OBSTACLE_A, DEFAULT_CONFIG)
        target_c = targets.viewing_target(OBSTACLE_C, DEFAULT_CONFIG)
        fake = FakeLegPlanner()
        final_a = fake.set_valid(
            START, target_a, cost=_min_admissible_cost(START, target_a)
        ).final_pose
        fake.set_valid(
            final_a, target_c, cost=_min_admissible_cost(final_a, target_c)
        )  # (A, C): the pricier complete order, tried first (input order).
        final_c = fake.set_valid(
            START, target_c, cost=_min_admissible_cost(START, target_c)
        ).final_pose
        fake.set_valid(
            final_c, target_a, cost=_min_admissible_cost(final_c, target_a)
        )  # (C, A): cheaper -- its lower bound must not exceed the (A, C)
           # incumbent, so the leg planner must still be invoked for it.

        result = ero.optimise_executable_b3_route(
            START, [OBSTACLE_A, OBSTACLE_C], DEFAULT_CONFIG,
            leg_planner=fake, allow_alternative_viewing=False,
        )
        self.assertIn((START, target_c), fake.calls)
        self.assertIn((final_c, target_a), fake.calls)
        self.assertEqual(result.obstacle_order, (3, 1))
        self.assertEqual(result.valid_orders, 2)
        self.assertEqual(result.pruned_orders, 0)
        self.assertEqual(result.rejected_orders, 0)


class RealPipelineIntegration(unittest.TestCase):
    """Small, fast cases using the actual search -> compression -> generation
    pipeline against DEFAULT_PLANNER_CONFIG (kept to 1-2 obstacles: some
    real leg queries are slow with the measured turn radii, a known
    tuning limitation already documented for the state-lattice search)."""

    def test_b2_real_pipeline_two_obstacles(self):
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A, OBSTACLE_B], DEFAULT_CONFIG,
            allow_alternative_viewing=False,
        )
        self.assertTrue(result.is_valid)
        self.assertTrue(result.is_complete)
        self.assertEqual(len(result.legs), 2)
        for leg in result.legs:
            self.assertTrue(leg.command_plan.is_valid)
            self.assertGreater(len(leg.command_plan.commands), 0)
        self.assertEqual(result.final_pose, result.legs[-1].command_plan.final_pose)

    def test_b3_real_pipeline_single_obstacle(self):
        result = ero.optimise_executable_b3_route(
            START, [OBSTACLE_A], DEFAULT_CONFIG, allow_alternative_viewing=False,
        )
        self.assertTrue(result.is_valid)
        self.assertTrue(result.is_complete)
        self.assertEqual(result.obstacle_order, (1,))


class GeneratedCommandsIndependentlyValid(unittest.TestCase):
    def test_every_command_replays_and_revalidates(self):
        result = ero.find_executable_b2_route(
            START, [OBSTACLE_A], DEFAULT_CONFIG, allow_alternative_viewing=False,
        )
        self.assertTrue(result.is_valid)
        obstacles = (OBSTACLE_A,)
        for leg in result.legs:
            for command in leg.command_plan.commands:
                replay = mp.propagate(
                    command.start_pose, command.direction, command.value, DEFAULT_CONFIG
                )
                self.assertAlmostEqual(replay.x, command.end_pose.x, places=6)
                self.assertAlmostEqual(replay.y, command.end_pose.y, places=6)
                revalidated = sc.safe_primitive_samples(
                    command.start_pose, command.direction, command.value,
                    obstacles, DEFAULT_CONFIG,
                )
                self.assertIsNotNone(revalidated)
                self.assertEqual(command.samples, revalidated)


if __name__ == "__main__":
    unittest.main()
