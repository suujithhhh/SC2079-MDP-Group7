"""Standalone STM-executable state-lattice search (state_lattice.py).

Not connected to planner_interface.py or any other consumer.

Most tests use a small, fast custom PlannerConfig (tiny footprint/radii,
coarse lattice steps) so the reachable state space stays small and
exhaustive/failure tests (impossible case, expansion limit) finish in well
under a second. This is a test-speed choice only.

Real DEFAULT_PLANNER_CONFIG geometry (the actual 24x19 cm footprint, 1 cm
margin, and all four measured turn radii) is also exercised directly: by
the one-step reachability/independent-radii tests below, and by
ObstacleDetourRealGeometry, a real-scale obstacle-detour regression. One
exploratory real-geometry detour attempt made during development did
exhaust the 20,000-expansion cap before finding a route (the large
measured turn radii can force a wide maneuver around even a small
obstacle) — that is a genuine tuning limitation to track, not evidence
that real radii generally prevent detours; ObstacleDetourRealGeometry
below is a case that does succeed, and is checked for validity and an
expansion bound rather than an exact action count.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import dataclasses
import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import motion_primitives as mp  # noqa: E402
import swept_collision as sc  # noqa: E402
import state_lattice as sl  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
from models import Obstacle, Pose  # noqa: E402

DEFAULT_CONFIG = DEFAULT_PLANNER_CONFIG

# A small, fast custom config: tiny footprint/radii and coarse lattice
# steps keep the reachable state space small so tests run in well under a
# second (see module docstring; this is a test-speed choice only).
SMALL_CONFIG = dataclasses.replace(
    DEFAULT_CONFIG,
    arena_cm=200.0,
    robot_front_extent_cm=5.0,
    robot_rear_extent_cm=0.0,
    robot_width_cm=4.0,
    safety_margin_cm=0.5,
    turn_radius_fl_cm=5.0,
    turn_radius_fr_cm=7.0,
    turn_radius_bl_cm=6.0,
    turn_radius_br_cm=4.0,
    sweep_max_point_step_cm=0.5,
    lattice_straight_step_cm=5,
    lattice_turn_step_deg=15,
    lattice_position_bin_cm=2.0,
    lattice_heading_bin_deg=15,
    lattice_goal_position_tolerance_cm=2.0,
    lattice_goal_heading_tolerance_deg=10.0,
    lattice_max_expansions=20000,
)

# A much smaller arena, used only by the enclosed/expansion-limit tests
# below, so the reachable free-space state space is small enough to
# exhaust (or hit a deliberately low cap) in well under a second.
ENCLOSED_CONFIG = dataclasses.replace(SMALL_CONFIG, arena_cm=30.0)


def assert_plan_internally_consistent(test, plan, obstacles, config):
    """Chain/endpoint/independent-revalidation invariants common to every success test."""
    pose = plan.actions[0].start_pose if plan.actions else plan.final_pose
    for action in plan.actions:
        test.assertEqual(action.start_pose, pose)
        expected_end = mp.propagate(action.start_pose, action.command, action.value, config)
        test.assertEqual(action.end_pose, expected_end)
        revalidated = sc.safe_primitive_samples(
            action.start_pose, action.command, action.value, obstacles, config
        )
        test.assertIsNotNone(revalidated)
        test.assertEqual(action.samples, revalidated)
        pose = action.end_pose
    test.assertEqual(pose, plan.final_pose)


class UnsafeStartRejected(unittest.TestCase):
    def test_start_overlapping_obstacle_rejected_before_search(self):
        start = Pose(15.0, 15.0, 0.0)
        obstacle = Obstacle(obstacle_id=1, x=14, y=12, image_face="N")
        goal = Pose(25.0, 15.0, 0.0)
        plan = sl.plan_lattice_path(start, goal, [obstacle], SMALL_CONFIG)
        self.assertFalse(plan.is_valid)
        self.assertEqual(plan.reason, sl.REASON_UNSAFE_START)
        self.assertEqual(plan.actions, ())
        self.assertEqual(plan.expansions, 0)
        self.assertIsNone(plan.final_pose)


class UnsafeGoalRejected(unittest.TestCase):
    def test_goal_overlapping_obstacle_rejected_before_search(self):
        start = Pose(5.0, 15.0, 0.0)
        goal = Pose(20.0, 15.0, 0.0)
        obstacle = Obstacle(obstacle_id=1, x=19, y=12, image_face="N")
        plan = sl.plan_lattice_path(start, goal, [obstacle], SMALL_CONFIG)
        self.assertFalse(plan.is_valid)
        self.assertEqual(plan.reason, sl.REASON_UNSAFE_GOAL)
        self.assertEqual(plan.actions, ())
        self.assertEqual(plan.expansions, 0)


class StartAlreadyAtGoal(unittest.TestCase):
    def test_start_within_tolerance_returns_zero_action_plan(self):
        start = Pose(10.0, 10.0, math.radians(3.0))
        goal = Pose(11.0, 10.5, math.radians(0.0))  # within 2cm/10deg tolerance
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        self.assertEqual(plan.actions, ())
        self.assertEqual(plan.waypoints, (start,))
        self.assertEqual(plan.total_distance_cm, 0.0)
        self.assertEqual(plan.total_cost, 0.0)
        self.assertEqual(plan.expansions, 0)
        self.assertEqual(plan.final_pose, start)


class OneStepReachability(unittest.TestCase):
    """Each of propagate()'s six one-step endpoints, reachable via the matching action."""

    def test_each_action_reaches_its_own_propagate_endpoint(self):
        start = Pose(100.0, 100.0, 0.0)
        for action in ("F", "B", "FL", "FR", "BL", "BR"):
            value = (
                DEFAULT_CONFIG.lattice_straight_step_cm
                if action in ("F", "B")
                else DEFAULT_CONFIG.lattice_turn_step_deg
            )
            with self.subTest(action=action):
                goal = mp.propagate(start, action, value, DEFAULT_CONFIG)
                plan = sl.plan_lattice_path(start, goal, [], DEFAULT_CONFIG)
                self.assertTrue(plan.is_valid)
                self.assertEqual(len(plan.actions), 1)
                self.assertEqual(plan.actions[0].command, action)
                self.assertEqual(plan.actions[0].value, value)
                self.assertEqual(plan.final_pose, goal)
                assert_plan_internally_consistent(self, plan, [], DEFAULT_CONFIG)


class EmittedCommandsAreValidSTMValues(unittest.TestCase):
    def test_every_action_value_is_integer_within_stm_limits_with_correct_unit(self):
        start = Pose(10.0, 15.0, 0.0)
        goal = Pose(22.0, 15.0, math.radians(30.0))
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        self.assertGreater(len(plan.actions), 0)
        for action in plan.actions:
            with self.subTest(command=action.command):
                self.assertIsInstance(action.value, int)
                self.assertNotIsInstance(action.value, bool)
                if action.command in ("F", "B"):
                    self.assertEqual(action.unit, "cm")
                    self.assertTrue(1 <= action.value <= 999)
                else:
                    self.assertIn(action.command, ("FL", "FR", "BL", "BR"))
                    self.assertEqual(action.unit, "deg")
                    self.assertTrue(1 <= action.value <= 360)


class ActionChainingAndFinalPose(unittest.TestCase):
    def test_each_actions_start_equals_previous_exact_end(self):
        start = Pose(10.0, 15.0, 0.0)
        goal = Pose(22.0, 15.0, math.radians(30.0))
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        self.assertGreaterEqual(len(plan.actions), 2)
        for prev, nxt in zip(plan.actions, plan.actions[1:]):
            self.assertEqual(prev.end_pose, nxt.start_pose)

    def test_final_pose_is_last_actions_end_not_the_requested_goal(self):
        start = Pose(10.0, 15.0, 0.0)
        goal = Pose(22.0, 15.0, math.radians(30.0))
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        self.assertEqual(plan.final_pose, plan.actions[-1].end_pose)
        self.assertNotEqual(plan.final_pose, goal)  # reached pose, not the ideal goal


class ContinuousStartPreserved(unittest.TestCase):
    def test_non_grid_start_is_not_snapped_to_a_bin(self):
        start = Pose(11.3, 14.7, math.radians(7.0))  # deliberately off any bin
        goal = mp.propagate(start, "F", SMALL_CONFIG.lattice_straight_step_cm, SMALL_CONFIG)
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        self.assertEqual(plan.actions[0].start_pose, start)  # exact, unsnapped


class ActionSamplesMatchSweptCollision(unittest.TestCase):
    def test_every_actions_samples_are_exactly_safe_primitive_samples(self):
        start = Pose(10.0, 15.0, 0.0)
        goal = Pose(22.0, 15.0, math.radians(30.0))
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        assert_plan_internally_consistent(self, plan, [], SMALL_CONFIG)


class StraightRoute(unittest.TestCase):
    def test_straight_route_in_empty_arena(self):
        start = Pose(5.0, 15.0, 0.0)
        goal = Pose(20.0, 15.0, 0.0)
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        self.assertTrue(any(a.command == "F" for a in plan.actions))
        # An unobstructed, directly-ahead goal should never need reversing;
        # the exact action mix (e.g. a cheap forward wiggle that lands
        # within tolerance for less total cost than one more full F step)
        # is left to the optimiser, not asserted here.
        self.assertTrue(all(a.command not in ("B", "BL", "BR") for a in plan.actions))
        assert_plan_internally_consistent(self, plan, [], SMALL_CONFIG)


class TurnRequiredRoute(unittest.TestCase):
    def test_route_requiring_a_turn(self):
        start = Pose(5.0, 5.0, 0.0)
        goal = Pose(15.0, 15.0, math.radians(90.0))
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        self.assertTrue(any(a.command in ("FL", "FR", "BL", "BR") for a in plan.actions))
        assert_plan_internally_consistent(self, plan, [], SMALL_CONFIG)


class ReverseRequiredRoute(unittest.TestCase):
    def test_reverse_route_applies_reverse_penalty(self):
        start = Pose(15.0, 15.0, 0.0)
        end = mp.propagate(mp.propagate(start, "B", 5, SMALL_CONFIG), "B", 5, SMALL_CONFIG)
        plan = sl.plan_lattice_path(start, end, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        self.assertTrue(all(a.command == "B" for a in plan.actions))
        self.assertEqual(plan.total_distance_cm, 10.0)
        self.assertAlmostEqual(
            plan.total_cost, 10.0 * SMALL_CONFIG.reverse_penalty, places=9
        )
        self.assertGreater(plan.total_cost, plan.total_distance_cm)  # penalty actually applied
        assert_plan_internally_consistent(self, plan, [], SMALL_CONFIG)


class ObstacleDetour(unittest.TestCase):
    def test_route_around_a_simple_obstacle(self):
        start = Pose(50.0, 50.0, 0.0)
        goal = Pose(85.0, 50.0, 0.0)
        obstacle = Obstacle(obstacle_id=1, x=62, y=48, image_face="N")

        # Confirm the direct primitive really is blocked, so a successful
        # plan here proves the search actually detoured.
        self.assertIsNone(sc.safe_primitive_samples(start, "F", 35, [obstacle], SMALL_CONFIG))

        t0 = time.time()
        plan = sl.plan_lattice_path(start, goal, [obstacle], SMALL_CONFIG)
        elapsed = time.time() - t0

        self.assertTrue(plan.is_valid)
        self.assertGreater(len(plan.actions), 1)
        assert_plan_internally_consistent(self, plan, [obstacle], SMALL_CONFIG)
        print(
            f"\n[perf] obstacle detour: actions={len(plan.actions)} "
            f"expansions={plan.expansions} time={elapsed:.3f}s"
        )


class ObstacleDetourRealGeometry(unittest.TestCase):
    """Same shape of test as ObstacleDetour, but with the real measured
    24x19 cm footprint, 1 cm margin, and all four measured turn radii
    (DEFAULT_PLANNER_CONFIG) instead of the small test config."""

    def test_route_around_a_simple_obstacle_with_default_geometry(self):
        start = Pose(30.0, 100.0, 0.0)
        goal = Pose(110.0, 100.0, 0.0)
        obstacle = Obstacle(obstacle_id=1, x=70, y=95, image_face="N")

        # Confirm the direct primitive really is blocked, so a successful
        # plan here proves the search actually detoured.
        self.assertIsNone(
            sc.safe_primitive_samples(start, "F", 80, [obstacle], DEFAULT_CONFIG)
        )

        t0 = time.time()
        plan = sl.plan_lattice_path(start, goal, [obstacle], DEFAULT_CONFIG)
        elapsed = time.time() - t0
        print(
            f"\n[perf] real-geometry obstacle detour: actions={len(plan.actions)} "
            f"expansions={plan.expansions} time={elapsed:.3f}s"
        )

        self.assertTrue(plan.is_valid)
        self.assertGreater(len(plan.actions), 1)
        self.assertTrue(any(a.command in ("FL", "FR", "BL", "BR") for a in plan.actions))
        assert_plan_internally_consistent(self, plan, [obstacle], DEFAULT_CONFIG)
        self.assertLess(plan.expansions, DEFAULT_CONFIG.lattice_max_expansions)

        distance_cm = math.hypot(plan.final_pose.x - goal.x, plan.final_pose.y - goal.y)
        self.assertLessEqual(distance_cm, DEFAULT_CONFIG.lattice_goal_position_tolerance_cm)
        heading_diff_deg = abs(
            math.degrees(mp.normalize_heading(plan.final_pose.heading - goal.heading))
        )
        heading_diff_deg = min(heading_diff_deg, 360.0 - heading_diff_deg)
        self.assertLessEqual(heading_diff_deg, DEFAULT_CONFIG.lattice_goal_heading_tolerance_deg)


class HeuristicAdmissibility(unittest.TestCase):
    def test_zero_for_a_pose_already_inside_the_position_tolerance(self):
        goal = Pose(50.0, 50.0, 0.0)
        pose = Pose(51.0, 50.5, math.radians(90.0))  # heading irrelevant to this heuristic
        self.assertLessEqual(
            math.hypot(pose.x - goal.x, pose.y - goal.y),
            SMALL_CONFIG.lattice_goal_position_tolerance_cm,
        )
        self.assertEqual(sl._heuristic_cm(pose, goal, SMALL_CONFIG), 0.0)

    def test_subtracts_the_goal_position_tolerance_outside_the_region(self):
        goal = Pose(0.0, 0.0, 0.0)
        pose = Pose(20.0, 0.0, 0.0)
        expected = 20.0 - SMALL_CONFIG.lattice_goal_position_tolerance_cm
        self.assertAlmostEqual(sl._heuristic_cm(pose, goal, SMALL_CONFIG), expected, places=9)

    def test_reverse_penalty_below_one_scales_the_lower_bound(self):
        goal = Pose(0.0, 0.0, 0.0)
        pose = Pose(20.0, 0.0, 0.0)
        config = dataclasses.replace(SMALL_CONFIG, reverse_penalty=0.5)
        expected = (20.0 - config.lattice_goal_position_tolerance_cm) * 0.5
        self.assertAlmostEqual(sl._heuristic_cm(pose, goal, config), expected, places=9)

    def test_reverse_penalty_of_one_or_above_does_not_inflate_the_bound(self):
        goal = Pose(0.0, 0.0, 0.0)
        pose = Pose(20.0, 0.0, 0.0)
        expected = 20.0 - SMALL_CONFIG.lattice_goal_position_tolerance_cm
        for penalty in (1.0, 1.08, 3.0):
            with self.subTest(reverse_penalty=penalty):
                config = dataclasses.replace(SMALL_CONFIG, reverse_penalty=penalty)
                self.assertAlmostEqual(sl._heuristic_cm(pose, goal, config), expected, places=9)

    def test_invalid_reverse_penalty_values_rejected(self):
        start = Pose(5.0, 5.0, 0.0)
        goal = Pose(10.0, 5.0, 0.0)
        for bad in (0, 0.0, -1.0, float("inf"), float("nan"), True, False, "1.08"):
            with self.subTest(reverse_penalty=bad):
                config = dataclasses.replace(SMALL_CONFIG, reverse_penalty=bad)
                with self.assertRaises(sl.InvalidLatticeConfigError):
                    sl.plan_lattice_path(start, goal, [], config)


class ImpossibleEnclosedCase(unittest.TestCase):
    """A full-width wall of obstacles genuinely separates start from goal."""

    WALL_OBSTACLES = tuple(
        Obstacle(obstacle_id=i + 1, x=10, y=10 * i, image_face="N") for i in range(3)
    )
    START = Pose(4.0, 15.0, 0.0)
    GOAL = Pose(22.5, 15.0, 0.0)

    def test_fully_enclosed_goal_returns_no_path(self):
        config = dataclasses.replace(ENCLOSED_CONFIG, lattice_max_expansions=20000)
        t0 = time.time()
        plan = sl.plan_lattice_path(self.START, self.GOAL, self.WALL_OBSTACLES, config)
        elapsed = time.time() - t0

        self.assertFalse(plan.is_valid)
        self.assertEqual(plan.reason, sl.REASON_NO_PATH)
        self.assertEqual(plan.actions, ())
        self.assertLess(plan.expansions, config.lattice_max_expansions)
        print(f"\n[perf] impossible/no_path: expansions={plan.expansions} time={elapsed:.3f}s")


class ExpansionLimitCase(unittest.TestCase):
    def test_low_expansion_cap_produces_bounded_failure(self):
        config = dataclasses.replace(ENCLOSED_CONFIG, lattice_max_expansions=50)
        t0 = time.time()
        plan = sl.plan_lattice_path(
            ImpossibleEnclosedCase.START,
            ImpossibleEnclosedCase.GOAL,
            ImpossibleEnclosedCase.WALL_OBSTACLES,
            config,
        )
        elapsed = time.time() - t0

        self.assertFalse(plan.is_valid)
        self.assertEqual(plan.reason, sl.REASON_EXPANSION_LIMIT)
        self.assertEqual(plan.expansions, 50)
        self.assertEqual(plan.actions, ())
        print(f"\n[perf] expansion_limit: expansions={plan.expansions} time={elapsed:.3f}s")


class DeterministicRepeatability(unittest.TestCase):
    def test_repeated_identical_requests_produce_identical_plans(self):
        start = Pose(10.0, 15.0, 0.0)
        goal = Pose(22.0, 15.0, math.radians(30.0))
        plan_a = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        plan_b = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)

        self.assertEqual(len(plan_a.actions), len(plan_b.actions))
        for a, b in zip(plan_a.actions, plan_b.actions):
            self.assertEqual(a.command, b.command)
            self.assertEqual(a.value, b.value)
            self.assertEqual(a.end_pose, b.end_pose)
        self.assertEqual(plan_a.final_pose, plan_b.final_pose)
        self.assertEqual(plan_a.expansions, plan_b.expansions)
        self.assertEqual(plan_a.total_cost, plan_b.total_cost)


class SearchKeyDistinguishesPreviousAction(unittest.TestCase):
    def test_same_quantised_pose_with_different_previous_action_is_a_different_key(self):
        pose = Pose(15.0, 15.0, 0.0)
        key_after_f = sl._state_key(pose, "F", SMALL_CONFIG)
        key_after_b = sl._state_key(pose, "B", SMALL_CONFIG)
        key_start = sl._state_key(pose, sl._START_ACTION_KEY, SMALL_CONFIG)
        self.assertNotEqual(key_after_f, key_after_b)
        self.assertNotEqual(key_after_f, key_start)
        self.assertNotEqual(key_after_b, key_start)
        # But the position/heading portion (everything but the previous
        # action) is identical, proving the difference is solely the
        # previous-action component of the key.
        self.assertEqual(key_after_f[:3], key_after_b[:3])
        self.assertEqual(key_after_f[:3], key_start[:3])


class InvalidLatticeConfigRejected(unittest.TestCase):
    def _plan(self, config):
        return sl.plan_lattice_path(Pose(5.0, 5.0, 0.0), Pose(10.0, 5.0, 0.0), [], config)

    def test_non_integer_or_out_of_range_straight_step_rejected(self):
        for bad in (0, -5, 1000, 12.5, True, "10"):
            with self.subTest(value=bad):
                config = dataclasses.replace(SMALL_CONFIG, lattice_straight_step_cm=bad)
                with self.assertRaises(sl.InvalidLatticeConfigError):
                    self._plan(config)

    def test_non_integer_or_out_of_range_turn_step_rejected(self):
        for bad in (0, -5, 361, 12.5, True):
            with self.subTest(value=bad):
                config = dataclasses.replace(SMALL_CONFIG, lattice_turn_step_deg=bad)
                with self.assertRaises(sl.InvalidLatticeConfigError):
                    self._plan(config)

    def test_non_positive_or_non_finite_position_bin_rejected(self):
        for bad in (0, -1.0, float("inf"), float("nan"), True):
            with self.subTest(value=bad):
                config = dataclasses.replace(SMALL_CONFIG, lattice_position_bin_cm=bad)
                with self.assertRaises(sl.InvalidLatticeConfigError):
                    self._plan(config)

    def test_heading_bin_not_dividing_360_rejected(self):
        for bad in (7, 0, -10, 360.0, True):
            with self.subTest(value=bad):
                config = dataclasses.replace(SMALL_CONFIG, lattice_heading_bin_deg=bad)
                with self.assertRaises(sl.InvalidLatticeConfigError):
                    self._plan(config)

    def test_negative_or_non_finite_tolerances_rejected(self):
        for field_name in (
            "lattice_goal_position_tolerance_cm", "lattice_goal_heading_tolerance_deg"
        ):
            for bad in (-1.0, float("inf"), float("nan"), True):
                with self.subTest(field=field_name, value=bad):
                    config = dataclasses.replace(SMALL_CONFIG, **{field_name: bad})
                    with self.assertRaises(sl.InvalidLatticeConfigError):
                        self._plan(config)

    def test_non_positive_or_boolean_max_expansions_rejected(self):
        for bad in (0, -1, True, 12.5):
            with self.subTest(value=bad):
                config = dataclasses.replace(SMALL_CONFIG, lattice_max_expansions=bad)
                with self.assertRaises(sl.InvalidLatticeConfigError):
                    self._plan(config)

    def test_non_positive_non_finite_boolean_or_nonnumeric_reverse_penalty_rejected(self):
        for bad in (0, 0.0, -1.0, float("inf"), float("nan"), True, False, "1.08"):
            with self.subTest(value=bad):
                config = dataclasses.replace(SMALL_CONFIG, reverse_penalty=bad)
                with self.assertRaises(sl.InvalidLatticeConfigError):
                    self._plan(config)


class IndependentRadiiUsed(unittest.TestCase):
    def test_each_turn_actions_arc_uses_its_own_configured_radius(self):
        start = Pose(100.0, 100.0, 0.0)
        radii = {
            "FL": DEFAULT_CONFIG.turn_radius_fl_cm,
            "FR": DEFAULT_CONFIG.turn_radius_fr_cm,
            "BL": DEFAULT_CONFIG.turn_radius_bl_cm,
            "BR": DEFAULT_CONFIG.turn_radius_br_cm,
        }
        for action, radius in radii.items():
            with self.subTest(action=action):
                value = DEFAULT_CONFIG.lattice_turn_step_deg
                goal = mp.propagate(start, action, value, DEFAULT_CONFIG)
                plan = sl.plan_lattice_path(start, goal, [], DEFAULT_CONFIG)
                self.assertTrue(plan.is_valid)
                expected_length = radius * math.radians(value)
                self.assertAlmostEqual(
                    plan.actions[0].length_cm, expected_length, places=9
                )


class NoMutation(unittest.TestCase):
    def test_inputs_are_not_mutated(self):
        start = Pose(10.0, 15.0, 0.0)
        goal = Pose(22.0, 15.0, math.radians(30.0))
        start_snapshot = Pose(start.x, start.y, start.heading)
        goal_snapshot = Pose(goal.x, goal.y, goal.heading)
        obstacles = [Obstacle(obstacle_id=1, x=1, y=1, image_face="N")]
        obstacles_snapshot = list(obstacles)
        config_snapshot = dataclasses.replace(SMALL_CONFIG)

        sl.plan_lattice_path(start, goal, obstacles, SMALL_CONFIG)

        self.assertEqual(start, start_snapshot)
        self.assertEqual(goal, goal_snapshot)
        self.assertEqual(obstacles, obstacles_snapshot)
        self.assertEqual(SMALL_CONFIG, config_snapshot)


if __name__ == "__main__":
    unittest.main()
