"""Standalone, safety-preserving command compression (command_compression.py).

Not connected to planner_interface.py or any other consumer. Most tests
build small hand-crafted or SMALL_CONFIG-driven LatticePlans for speed and
precise control; ObstacleDetourRealGeometryCompression exercises a plan
produced by the real DEFAULT_PLANNER_CONFIG measured-radius state-lattice
search, to prove integration with Stage 3A rather than just re-testing
compression in isolation. Run from algo/:
    python -m unittest discover -s tests -v
"""

import dataclasses
import math
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import command_compression as cc  # noqa: E402
import motion_primitives as mp  # noqa: E402
import state_lattice as sl  # noqa: E402
import swept_collision as sc  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
from models import Obstacle, Pose  # noqa: E402
from state_lattice import LatticeAction, LatticePlan  # noqa: E402

DEFAULT_CONFIG = DEFAULT_PLANNER_CONFIG

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

# Arena large enough for the 100x/37x atomic-action chains built by hand.
BIG_ARENA_CONFIG = dataclasses.replace(DEFAULT_CONFIG, arena_cm=2000.0)


def _make_action(start_pose, command, value, config):
    """Build one real, independently-safe LatticeAction (for hand-crafted plans)."""
    samples = sc.safe_primitive_samples(start_pose, command, value, config=config, obstacles=())
    assert samples is not None, f"test fixture {command},{value} from {start_pose} is unsafe"
    unit = "cm" if command in mp.STRAIGHT_ACTIONS else "deg"
    return LatticeAction(
        command=command,
        value=value,
        unit=unit,
        start_pose=start_pose,
        end_pose=samples[-1],
        length_cm=mp.primitive_length_cm(command, value, config),
        samples=samples,
    )


def _make_plan(start_pose, commands, config):
    """Build a real, chained LatticePlan from a list of (command, value) pairs."""
    actions = []
    pose = start_pose
    for command, value in commands:
        action = _make_action(pose, command, value, config)
        actions.append(action)
        pose = action.end_pose
    total_distance = sum(a.length_cm for a in actions)
    reverse = frozenset({"B", "BL", "BR"})
    total_cost = sum(
        a.length_cm * (config.reverse_penalty if a.command in reverse else 1.0) for a in actions
    )
    return LatticePlan(
        is_valid=True,
        actions=tuple(actions),
        waypoints=(),  # not exercised by these hand-built fixtures
        total_distance_cm=total_distance,
        total_cost=total_cost,
        expansions=0,
        final_pose=pose,
        reason=None,
    )


def assert_chain_and_revalidated(test, compressed, obstacles, config):
    pose = compressed.actions[0].start_pose if compressed.actions else compressed.final_pose
    for action in compressed.actions:
        test.assertEqual(action.start_pose, pose)
        revalidated = sc.safe_primitive_samples(
            action.start_pose, action.command, action.value, obstacles, config
        )
        test.assertIsNotNone(revalidated)
        test.assertEqual(action.samples, revalidated)
        pose = action.end_pose
    test.assertEqual(pose, compressed.final_pose)


class SimpleMerging(unittest.TestCase):
    def test_three_f10_actions_compress_to_f30(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10), ("F", 10)], DEFAULT_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual([(a.command, a.value) for a in compressed.actions], [("F", 30)])

    def test_each_command_compresses_when_repeated(self):
        start = Pose(100.0, 100.0, 0.0)
        for command in ("F", "B", "FL", "FR", "BL", "BR"):
            value = 10
            with self.subTest(command=command):
                plan = _make_plan(start, [(command, value)] * 3, DEFAULT_CONFIG)
                compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
                self.assertEqual(
                    [(a.command, a.value) for a in compressed.actions], [(command, value * 3)]
                )

    def test_different_adjacent_commands_are_never_merged(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("FL", 10), ("F", 10)], DEFAULT_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual(
            [(a.command, a.value) for a in compressed.actions],
            [("F", 10), ("FL", 10), ("F", 10)],
        )


class ValueLimitSplitting(unittest.TestCase):
    def test_100_straight_atoms_split_at_990_without_hardcoding_999(self):
        start = Pose(50.0, 50.0, 0.0)
        plan = _make_plan(start, [("F", 10)] * 100, BIG_ARENA_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], BIG_ARENA_CONFIG)
        # The split point is derived from motion_primitives' own validation
        # (via primitive_length_cm), never a copied "999" constant here.
        self.assertTrue(
            all(
                cc._combined_value_is_legal("F", a.value, BIG_ARENA_CONFIG)
                for a in compressed.actions
            )
        )
        self.assertFalse(
            cc._combined_value_is_legal(
                "F", sum(a.value for a in compressed.actions[:1]) + 1000, BIG_ARENA_CONFIG
            )
        )
        self.assertEqual([(a.command, a.value) for a in compressed.actions], [("F", 990), ("F", 10)])

    def test_37_turn_atoms_split_at_360_without_hardcoding_the_limit(self):
        start = Pose(50.0, 50.0, 0.0)
        plan = _make_plan(start, [("FL", 10)] * 37, BIG_ARENA_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], BIG_ARENA_CONFIG)
        self.assertEqual(
            [(a.command, a.value) for a in compressed.actions], [("FL", 360), ("FL", 10)]
        )


class PreservesSpecialPlans(unittest.TestCase):
    def test_failure_plan_preserved_without_inventing_actions(self):
        failure_plan = sl._failure(sl.REASON_NO_PATH, 42)
        compressed = cc.compress_lattice_plan(failure_plan, [], DEFAULT_CONFIG)
        self.assertIs(compressed, failure_plan)
        self.assertFalse(compressed.is_valid)
        self.assertEqual(compressed.actions, ())
        self.assertEqual(compressed.reason, sl.REASON_NO_PATH)
        self.assertEqual(compressed.expansions, 42)

    def test_zero_action_plan_preserved(self):
        start = Pose(10.0, 10.0, 0.0)
        zero_plan = LatticePlan(
            is_valid=True, actions=(), waypoints=(start,), total_distance_cm=0.0,
            total_cost=0.0, expansions=0, final_pose=start, reason=None,
        )
        compressed = cc.compress_lattice_plan(zero_plan, [], DEFAULT_CONFIG)
        self.assertIs(compressed, zero_plan)


class NoMutation(unittest.TestCase):
    def test_inputs_are_not_mutated(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        plan_snapshot_actions = tuple(plan.actions)
        obstacles = [Obstacle(obstacle_id=1, x=1, y=1, image_face="N")]
        obstacles_snapshot = list(obstacles)
        config_snapshot = dataclasses.replace(DEFAULT_CONFIG)

        cc.compress_lattice_plan(plan, obstacles, DEFAULT_CONFIG)

        self.assertEqual(plan.actions, plan_snapshot_actions)
        self.assertEqual(plan.is_valid, True)
        self.assertEqual(obstacles, obstacles_snapshot)
        self.assertEqual(DEFAULT_CONFIG, config_snapshot)


class Immutability(unittest.TestCase):
    def test_output_plan_and_actions_are_immutable(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
        self.assertIsInstance(compressed.actions, tuple)
        self.assertIsInstance(compressed.waypoints, tuple)
        with self.assertRaises(Exception):
            compressed.is_valid = False
        with self.assertRaises(Exception):
            compressed.actions[0].value = 999


class OutputValuesAreSTMValid(unittest.TestCase):
    def test_output_values_and_units_are_stm_valid(self):
        start = Pose(50.0, 50.0, 0.0)
        plan = _make_plan(start, [("F", 10)] * 100, BIG_ARENA_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], BIG_ARENA_CONFIG)
        for action in compressed.actions:
            self.assertIsInstance(action.value, int)
            self.assertNotIsInstance(action.value, bool)
            self.assertGreater(action.value, 0)
            self.assertEqual(action.unit, "cm" if action.command in ("F", "B") else "deg")


class SamplesAndChaining(unittest.TestCase):
    def test_every_actions_samples_match_independent_revalidation(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10), ("FL", 10), ("FL", 10)], DEFAULT_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
        assert_chain_and_revalidated(self, compressed, [], DEFAULT_CONFIG)

    def test_each_action_starts_at_previous_exact_end(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("FL", 10), ("F", 10), ("FR", 10)], DEFAULT_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
        for prev, nxt in zip(compressed.actions, compressed.actions[1:]):
            self.assertEqual(prev.end_pose, nxt.start_pose)


class FinalPosePreservation(unittest.TestCase):
    def test_final_pose_is_exact_compressed_replay_not_the_ideal_goal(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual(compressed.final_pose, compressed.actions[-1].end_pose)

    def test_final_pose_matches_atomic_plan_within_strict_tolerance(self):
        start = Pose(30.0, 100.0, 0.0)
        goal = Pose(60.0, 100.0, math.radians(30.0))
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        compressed = cc.compress_lattice_plan(plan, [], SMALL_CONFIG)
        self.assertAlmostEqual(compressed.final_pose.x, plan.final_pose.x, places=6)
        self.assertAlmostEqual(compressed.final_pose.y, plan.final_pose.y, places=6)
        self.assertAlmostEqual(
            mp.normalize_heading(compressed.final_pose.heading - plan.final_pose.heading),
            0.0,
            places=6,
        )


class DistanceAndCostPreservation(unittest.TestCase):
    def test_total_distance_preserved(self):
        start = Pose(30.0, 100.0, 0.0)
        goal = Pose(60.0, 100.0, math.radians(30.0))
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], SMALL_CONFIG)
        self.assertAlmostEqual(compressed.total_distance_cm, plan.total_distance_cm, places=6)

    def test_total_cost_including_reverse_penalty_preserved(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("B", 10), ("B", 10)], DEFAULT_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual(len(compressed.actions), 1)
        self.assertAlmostEqual(compressed.total_cost, plan.total_cost, places=9)
        self.assertAlmostEqual(
            compressed.total_cost, 20.0 * DEFAULT_CONFIG.reverse_penalty, places=9
        )
        self.assertGreater(compressed.total_cost, compressed.total_distance_cm)


class WaypointsRebuilt(unittest.TestCase):
    def test_waypoints_do_not_duplicate_action_boundaries(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10), ("F", 10)], DEFAULT_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual(len(compressed.actions), 1)
        expected = (start,) + compressed.actions[0].samples[1:]
        self.assertEqual(compressed.waypoints, expected)
        # No pose appears twice in a row (i.e. no duplicated boundary).
        for a, b in zip(compressed.waypoints, compressed.waypoints[1:]):
            self.assertNotEqual(a, b)


class ObstacleDetourRealGeometryCompression(unittest.TestCase):
    """Proves integration with a real Stage 3A DEFAULT_PLANNER_CONFIG plan."""

    def test_real_geometry_detour_plan_compresses_and_revalidates(self):
        start = Pose(30.0, 100.0, 0.0)
        goal = Pose(110.0, 100.0, 0.0)
        obstacle = Obstacle(obstacle_id=1, x=70, y=95, image_face="N")
        plan = sl.plan_lattice_path(start, goal, [obstacle], DEFAULT_CONFIG)
        self.assertTrue(plan.is_valid)
        self.assertGreater(len(plan.actions), 1)

        compressed = cc.compress_lattice_plan(plan, [obstacle], DEFAULT_CONFIG)

        self.assertTrue(compressed.is_valid)
        self.assertLessEqual(len(compressed.actions), len(plan.actions))
        assert_chain_and_revalidated(self, compressed, [obstacle], DEFAULT_CONFIG)
        self.assertAlmostEqual(compressed.total_distance_cm, plan.total_distance_cm, places=6)
        self.assertAlmostEqual(compressed.total_cost, plan.total_cost, places=6)
        self.assertAlmostEqual(compressed.final_pose.x, plan.final_pose.x, places=6)
        self.assertAlmostEqual(compressed.final_pose.y, plan.final_pose.y, places=6)
        print(
            f"\n[compression] real-geometry detour: {len(plan.actions)} -> "
            f"{len(compressed.actions)} actions: "
            f"{[(a.command, a.value) for a in compressed.actions]}"
        )


class OneShotObstacleIterable(unittest.TestCase):
    def test_one_shot_obstacle_iterable_works(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)

        def obstacle_iter():
            return iter([])

        compressed = cc.compress_lattice_plan(plan, obstacle_iter(), DEFAULT_CONFIG)
        self.assertEqual([(a.command, a.value) for a in compressed.actions], [("F", 20)])


class DeterministicRepeatability(unittest.TestCase):
    def test_repeated_compression_produces_identical_output(self):
        start = Pose(30.0, 100.0, 0.0)
        goal = Pose(60.0, 100.0, math.radians(30.0))
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        first = cc.compress_lattice_plan(plan, [], SMALL_CONFIG)
        second = cc.compress_lattice_plan(plan, [], SMALL_CONFIG)
        self.assertEqual(
            [(a.command, a.value) for a in first.actions],
            [(a.command, a.value) for a in second.actions],
        )
        self.assertEqual(first.final_pose, second.final_pose)
        self.assertEqual(first.total_cost, second.total_cost)


class MergeFallbackSimulated(unittest.TestCase):
    def test_merged_command_rejection_falls_back_to_original_actions(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        real_safe = sc.safe_primitive_samples

        def fake_safe(start_pose, action, value, obstacles, config):
            if action == "F" and value == 20:
                return None  # simulate: the merged F,20 would be unsafe
            return real_safe(start_pose, action, value, obstacles, config)

        with mock.patch(
            "command_compression.swept_collision.safe_primitive_samples", side_effect=fake_safe
        ):
            compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)

        self.assertEqual([(a.command, a.value) for a in compressed.actions], [("F", 10), ("F", 10)])
        self.assertAlmostEqual(compressed.final_pose.x, plan.final_pose.x, places=9)
        self.assertAlmostEqual(compressed.final_pose.y, plan.final_pose.y, places=9)


class FallbackFailureRaisesInvariantError(unittest.TestCase):
    def test_unrevalidatable_fallback_action_raises_dedicated_exception(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)

        with mock.patch(
            "command_compression.swept_collision.safe_primitive_samples", return_value=None
        ):
            with self.assertRaises(cc.CompressionInvariantError):
                cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)


class InvalidPlanRejected(unittest.TestCase):
    def test_broken_action_chain_rejected(self):
        start = Pose(100.0, 100.0, 0.0)
        a1 = _make_action(start, "F", 10, DEFAULT_CONFIG)
        # a2 does NOT start where a1 ends -- a deliberately broken chain.
        wrong_start = Pose(999.0, 999.0, 0.0)
        a2 = _make_action(Pose(50.0, 50.0, 0.0), "F", 10, DEFAULT_CONFIG)
        broken_plan = LatticePlan(
            is_valid=True, actions=(a1, a2), waypoints=(), total_distance_cm=20.0,
            total_cost=20.0, expansions=0, final_pose=a2.end_pose, reason=None,
        )
        with self.assertRaises(cc.InvalidLatticePlanError):
            cc.compress_lattice_plan(broken_plan, [], DEFAULT_CONFIG)

    def test_mismatched_final_pose_rejected(self):
        start = Pose(100.0, 100.0, 0.0)
        a1 = _make_action(start, "F", 10, DEFAULT_CONFIG)
        wrong_final = Pose(0.0, 0.0, 0.0)
        broken_plan = LatticePlan(
            is_valid=True, actions=(a1,), waypoints=(), total_distance_cm=10.0,
            total_cost=10.0, expansions=0, final_pose=wrong_final, reason=None,
        )
        with self.assertRaises(cc.InvalidLatticePlanError):
            cc.compress_lattice_plan(broken_plan, [], DEFAULT_CONFIG)


def _single_action_plan(start, command, value, config, **overrides):
    """One-action plan built from a real, valid LatticeAction, with fields
    of the action and/or the plan optionally overridden to be invalid."""
    action = _make_action(start, command, value, config)
    action_overrides = {k: v for k, v in overrides.items() if k in LatticeAction.__dataclass_fields__}
    plan_overrides = {k: v for k, v in overrides.items() if k not in action_overrides}
    if action_overrides:
        action = dataclasses.replace(action, **action_overrides)
    plan = LatticePlan(
        is_valid=True,
        actions=(action,),
        waypoints=(),
        total_distance_cm=action.length_cm,
        total_cost=action.length_cm
        * (config.reverse_penalty if command in ("B", "BL", "BR") else 1.0),
        expansions=0,
        final_pose=action.end_pose,
        reason=None,
    )
    return dataclasses.replace(plan, **plan_overrides) if plan_overrides else plan


class InputValidationRegressions(unittest.TestCase):
    """Each of these mirrors a real bug class the strengthened validator
    closes: a stored field that quietly disagrees with what the action
    actually is, rather than a merge-time or replay-time failure."""

    START = Pose(100.0, 100.0, 0.0)

    def test_wrong_action_unit_rejected(self):
        plan = _single_action_plan(self.START, "F", 10, DEFAULT_CONFIG, unit="deg")
        with self.assertRaises(cc.InvalidLatticePlanError):
            cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)

    def test_wrong_stored_action_length_rejected(self):
        plan = _single_action_plan(self.START, "F", 10, DEFAULT_CONFIG, length_cm=999.0)
        with self.assertRaises(cc.InvalidLatticePlanError):
            cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)

    def test_wrong_stored_total_distance_rejected(self):
        plan = _single_action_plan(self.START, "F", 10, DEFAULT_CONFIG, total_distance_cm=999.0)
        with self.assertRaises(cc.InvalidLatticePlanError):
            cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)

    def test_wrong_stored_total_cost_rejected(self):
        plan = _single_action_plan(self.START, "F", 10, DEFAULT_CONFIG, total_cost=999.0)
        with self.assertRaises(cc.InvalidLatticePlanError):
            cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)

    def test_non_finite_action_length_rejected(self):
        for bad in (float("inf"), float("nan"), float("-inf")):
            with self.subTest(length_cm=bad):
                plan = _single_action_plan(self.START, "F", 10, DEFAULT_CONFIG, length_cm=bad)
                with self.assertRaises(cc.InvalidLatticePlanError):
                    cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)

    def test_non_finite_total_distance_rejected(self):
        for bad in (float("inf"), float("nan")):
            with self.subTest(total_distance_cm=bad):
                plan = _single_action_plan(
                    self.START, "F", 10, DEFAULT_CONFIG, total_distance_cm=bad
                )
                with self.assertRaises(cc.InvalidLatticePlanError):
                    cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)

    def test_non_finite_total_cost_rejected(self):
        for bad in (float("inf"), float("nan")):
            with self.subTest(total_cost=bad):
                plan = _single_action_plan(self.START, "F", 10, DEFAULT_CONFIG, total_cost=bad)
                with self.assertRaises(cc.InvalidLatticePlanError):
                    cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)

    def test_invalid_command_exposed_as_invalid_lattice_plan_error(self):
        valid_action = _make_action(self.START, "F", 10, DEFAULT_CONFIG)
        tampered = dataclasses.replace(valid_action, command="FW")
        plan = LatticePlan(
            is_valid=True, actions=(tampered,), waypoints=(),
            total_distance_cm=tampered.length_cm, total_cost=tampered.length_cm,
            expansions=0, final_pose=tampered.end_pose, reason=None,
        )
        # Never a bare motion_primitives exception leaking through.
        try:
            cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
            self.fail("expected InvalidLatticePlanError")
        except mp.UnsupportedActionError:
            self.fail("a lower-level motion_primitives exception leaked through")
        except cc.InvalidLatticePlanError:
            pass

    def test_invalid_value_exposed_as_invalid_lattice_plan_error(self):
        valid_action = _make_action(self.START, "F", 10, DEFAULT_CONFIG)
        tampered = dataclasses.replace(valid_action, value=1000)
        plan = LatticePlan(
            is_valid=True, actions=(tampered,), waypoints=(),
            total_distance_cm=tampered.length_cm, total_cost=tampered.length_cm,
            expansions=0, final_pose=tampered.end_pose, reason=None,
        )
        try:
            cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
            self.fail("expected InvalidLatticePlanError")
        except mp.InvalidCommandValueError:
            self.fail("a lower-level motion_primitives exception leaked through")
        except cc.InvalidLatticePlanError:
            pass

    def test_missing_samples_rejected(self):
        plan = _single_action_plan(self.START, "F", 10, DEFAULT_CONFIG, samples=())
        with self.assertRaises(cc.InvalidLatticePlanError):
            cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)

    def test_sample_endpoints_inconsistent_with_action_rejected(self):
        action = _make_action(self.START, "F", 10, DEFAULT_CONFIG)
        wrong_samples = (Pose(0.0, 0.0, 0.0),) + action.samples[1:]
        plan = _single_action_plan(self.START, "F", 10, DEFAULT_CONFIG, samples=wrong_samples)
        with self.assertRaises(cc.InvalidLatticePlanError):
            cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)


class NoWireOrTimingFields(unittest.TestCase):
    def test_no_command_string_json_or_timing_field_introduced(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)

        plan_fields = {f.name for f in dataclasses.fields(compressed)}
        action_fields = {f.name for f in dataclasses.fields(compressed.actions[0])}
        forbidden = {
            "stm_command", "json", "mission_id", "seq",
            "estimated_time_s", "estimated_time_status", "timing",
        }
        self.assertEqual(plan_fields & forbidden, set())
        self.assertEqual(action_fields & forbidden, set())
        self.assertNotIn("SNAP", [a.command for a in compressed.actions])


if __name__ == "__main__":
    unittest.main()
