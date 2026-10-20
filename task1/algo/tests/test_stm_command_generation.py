"""Standalone STM wire-command generation (stm_command_generation.py).

Not connected to planner_interface.py or any other consumer. Most tests
build small hand-crafted LatticePlans (via command_compression's own test
fixtures) for speed and precise control over boundary values;
RealGeometryDetourGeneration exercises the full state-lattice search ->
command compression -> STM command generation pipeline against
DEFAULT_PLANNER_CONFIG to prove integration rather than re-testing
generation in isolation. Run from algo/:
    python -m unittest discover -s tests -v
"""

import dataclasses
import math
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import motion_primitives as mp  # noqa: E402
import state_lattice as sl  # noqa: E402
import stm_command_generation as sg  # noqa: E402
import swept_collision as sc  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
from models import Obstacle, Pose  # noqa: E402
from state_lattice import LatticeAction, LatticePlan  # noqa: E402

DEFAULT_CONFIG = DEFAULT_PLANNER_CONFIG
BIG_ARENA_CONFIG = dataclasses.replace(DEFAULT_CONFIG, arena_cm=2000.0)

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


def _make_action(start_pose, command, value, config):
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
        waypoints=(),
        total_distance_cm=total_distance,
        total_cost=total_cost,
        expansions=0,
        final_pose=pose,
        reason=None,
    )


class ExactWireStrings(unittest.TestCase):
    def test_exact_wire_string_for_every_direction(self):
        start = Pose(100.0, 100.0, 0.0)
        expected = {
            "F": "MOV,F,10", "B": "MOV,B,10", "FL": "MOV,FL,10",
            "FR": "MOV,FR,10", "BL": "MOV,BL,10", "BR": "MOV,BR,10",
        }
        for direction, wire in expected.items():
            with self.subTest(direction=direction):
                plan = _make_plan(start, [(direction, 10)], DEFAULT_CONFIG)
                result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
                self.assertTrue(result.is_valid)
                self.assertEqual(len(result.commands), 1)
                self.assertEqual(result.commands[0].stm_command, wire)


class CorrectUnits(unittest.TestCase):
    def test_cm_for_straight_deg_for_turns(self):
        start = Pose(100.0, 100.0, 0.0)
        for direction, expected_unit in (
            ("F", "cm"), ("B", "cm"), ("FL", "deg"), ("FR", "deg"),
            ("BL", "deg"), ("BR", "deg"),
        ):
            with self.subTest(direction=direction):
                plan = _make_plan(start, [(direction, 10)], DEFAULT_CONFIG)
                result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
                self.assertEqual(result.commands[0].unit, expected_unit)


class ExactIntegerPreservation(unittest.TestCase):
    def test_no_float_formatting_in_wire_string(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10), ("F", 10)], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual(len(result.commands), 1)
        command = result.commands[0]
        self.assertEqual(command.value, 30)
        self.assertIsInstance(command.value, int)
        self.assertNotIsInstance(command.value, bool)
        self.assertEqual(command.stm_command, "MOV,F,30")
        self.assertNotIn(".", command.stm_command)
        self.assertNotIn(" ", command.stm_command)


class BoundaryValues(unittest.TestCase):
    def test_straight_upper_boundary_999(self):
        start = Pose(500.0, 500.0, 0.0)
        plan = _make_plan(start, [("F", 999)], BIG_ARENA_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], BIG_ARENA_CONFIG)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.commands[0].stm_command, "MOV,F,999")

    def test_turn_upper_boundary_360(self):
        start = Pose(500.0, 500.0, 0.0)
        plan = _make_plan(start, [("FL", 360)], BIG_ARENA_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], BIG_ARENA_CONFIG)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.commands[0].stm_command, "MOV,FL,360")


class CompressionIntegration(unittest.TestCase):
    def test_adjacent_f10_actions_compress_to_a_single_command(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10), ("F", 10)], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual([c.stm_command for c in result.commands], ["MOV,F,30"])

    def test_legal_limit_splitting_990_plus_10(self):
        start = Pose(50.0, 50.0, 0.0)
        plan = _make_plan(start, [("F", 10)] * 100, BIG_ARENA_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], BIG_ARENA_CONFIG)
        self.assertEqual(
            [c.stm_command for c in result.commands], ["MOV,F,990", "MOV,F,10"]
        )

    def test_no_merging_across_different_commands(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("FL", 10), ("F", 10)], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual(
            [c.stm_command for c in result.commands],
            ["MOV,F,10", "MOV,FL,10", "MOV,F,10"],
        )


class PosePreservation(unittest.TestCase):
    def test_expected_start_and_end_pose_preserved(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual(result.commands[0].start_pose, start)
        self.assertEqual(result.commands[0].end_pose, plan.actions[-1].end_pose)

    def test_every_command_independently_reproduces_its_end_pose(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("FL", 10), ("F", 10)], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
        for command in result.commands:
            replay = mp.propagate(command.start_pose, command.direction, command.value, DEFAULT_CONFIG)
            self.assertAlmostEqual(replay.x, command.end_pose.x, places=9)
            self.assertAlmostEqual(replay.y, command.end_pose.y, places=9)
            self.assertAlmostEqual(
                mp.normalize_heading(replay.heading - command.end_pose.heading), 0.0, places=9
            )

    def test_exact_chaining_between_generated_commands(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("FL", 10), ("F", 10), ("FR", 10)], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
        for prev, nxt in zip(result.commands, result.commands[1:]):
            self.assertEqual(prev.end_pose, nxt.start_pose)


class SamplesAndWaypointsPreserved(unittest.TestCase):
    def test_samples_match_the_compressed_action(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        import command_compression as cc
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual(len(result.commands), len(compressed.actions))
        for command, action in zip(result.commands, compressed.actions):
            self.assertEqual(command.samples, action.samples)

    def test_waypoints_preserved_from_compressed_plan(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        import command_compression as cc
        compressed = cc.compress_lattice_plan(plan, [], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual(result.waypoints, compressed.waypoints)


class TotalsPreservation(unittest.TestCase):
    def test_distance_cost_expansions_final_pose_preserved(self):
        start = Pose(30.0, 100.0, 0.0)
        goal = Pose(60.0, 100.0, math.radians(30.0))
        plan = sl.plan_lattice_path(start, goal, [], SMALL_CONFIG)
        self.assertTrue(plan.is_valid)
        result = sg.generate_stm_command_plan(plan, [], SMALL_CONFIG)
        self.assertAlmostEqual(result.total_distance_cm, plan.total_distance_cm, places=6)
        self.assertAlmostEqual(result.total_cost, plan.total_cost, places=6)
        self.assertEqual(result.expansions, plan.expansions)
        self.assertAlmostEqual(result.final_pose.x, plan.final_pose.x, places=6)
        self.assertAlmostEqual(result.final_pose.y, plan.final_pose.y, places=6)

    def test_reverse_cost_preserved(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("B", 10), ("B", 10)], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
        self.assertEqual(len(result.commands), 1)
        self.assertAlmostEqual(
            result.total_cost, 20.0 * DEFAULT_CONFIG.reverse_penalty, places=9
        )
        self.assertGreater(result.total_cost, result.total_distance_cm)


class FailureAndZeroAction(unittest.TestCase):
    def test_failure_plan_yields_invalid_result_with_no_commands(self):
        failure_plan = sl._failure(sl.REASON_NO_PATH, 42)
        result = sg.generate_stm_command_plan(failure_plan, [], DEFAULT_CONFIG)
        self.assertFalse(result.is_valid)
        self.assertEqual(result.commands, ())
        self.assertEqual(result.reason, sl.REASON_NO_PATH)
        self.assertEqual(result.expansions, 42)
        self.assertIsNone(result.final_pose)

    def test_valid_zero_action_plan_preserved(self):
        start = Pose(10.0, 10.0, 0.0)
        zero_plan = LatticePlan(
            is_valid=True, actions=(), waypoints=(start,), total_distance_cm=0.0,
            total_cost=0.0, expansions=0, final_pose=start, reason=None,
        )
        result = sg.generate_stm_command_plan(zero_plan, [], DEFAULT_CONFIG)
        self.assertTrue(result.is_valid)
        self.assertEqual(result.commands, ())
        self.assertEqual(result.final_pose, start)
        self.assertEqual(result.total_distance_cm, 0.0)
        self.assertEqual(result.total_cost, 0.0)


class Immutability(unittest.TestCase):
    def test_output_is_immutable(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)
        self.assertIsInstance(result.commands, tuple)
        self.assertIsInstance(result.waypoints, tuple)
        with self.assertRaises(Exception):
            result.is_valid = False
        with self.assertRaises(Exception):
            result.commands[0].value = 999


class NoMutation(unittest.TestCase):
    def test_inputs_are_not_mutated(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        plan_actions_snapshot = tuple(plan.actions)
        obstacles = [Obstacle(obstacle_id=1, x=1, y=1, image_face="N")]
        obstacles_snapshot = list(obstacles)
        config_snapshot = dataclasses.replace(DEFAULT_CONFIG)

        sg.generate_stm_command_plan(plan, obstacles, DEFAULT_CONFIG)

        self.assertEqual(plan.actions, plan_actions_snapshot)
        self.assertEqual(obstacles, obstacles_snapshot)
        self.assertEqual(DEFAULT_CONFIG, config_snapshot)


class OneShotObstacleIterable(unittest.TestCase):
    def test_one_shot_iterable_works(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)

        def obstacle_iter():
            return iter([])

        result = sg.generate_stm_command_plan(plan, obstacle_iter(), DEFAULT_CONFIG)
        self.assertEqual([c.stm_command for c in result.commands], ["MOV,F,20"])


class MalformedPlanRejected(unittest.TestCase):
    def test_malformed_plan_rejected_via_compression_validation(self):
        start = Pose(100.0, 100.0, 0.0)
        a1 = _make_action(start, "F", 10, DEFAULT_CONFIG)
        wrong_final = Pose(0.0, 0.0, 0.0)
        broken_plan = LatticePlan(
            is_valid=True, actions=(a1,), waypoints=(), total_distance_cm=10.0,
            total_cost=10.0, expansions=0, final_pose=wrong_final, reason=None,
        )
        import command_compression as cc
        with self.assertRaises(cc.InvalidLatticePlanError):
            sg.generate_stm_command_plan(broken_plan, [], DEFAULT_CONFIG)


class GenerationInvariantFailure(unittest.TestCase):
    def test_broken_chain_between_compressed_actions_raises(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)

        import command_compression as cc

        real_compress = cc.compress_lattice_plan

        def fake_compress(p, obstacles, config):
            compressed = real_compress(p, obstacles, config)
            # Tamper with the compressed result to break the chain, the way
            # a bug elsewhere might -- generation must not trust it blindly.
            tampered_action = dataclasses.replace(
                compressed.actions[0], start_pose=Pose(999.0, 999.0, 0.0)
            )
            return dataclasses.replace(compressed, actions=(tampered_action,))

        with mock.patch(
            "stm_command_generation.command_compression.compress_lattice_plan",
            side_effect=fake_compress,
        ):
            with self.assertRaises(sg.STMCommandGenerationError):
                sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)


class NoForbiddenFields(unittest.TestCase):
    def test_no_mission_protocol_or_timing_fields_introduced(self):
        start = Pose(100.0, 100.0, 0.0)
        plan = _make_plan(start, [("F", 10), ("F", 10)], DEFAULT_CONFIG)
        result = sg.generate_stm_command_plan(plan, [], DEFAULT_CONFIG)

        plan_fields = {f.name for f in dataclasses.fields(result)}
        command_fields = {f.name for f in dataclasses.fields(result.commands[0])}
        forbidden = {
            "seq", "mission_id", "json", "estimated_time_s",
            "estimated_time_status", "timing", "obstacle_id",
        }
        self.assertEqual(plan_fields & forbidden, set())
        self.assertEqual(command_fields & forbidden, set())
        self.assertNotIn("SNAP", [c.direction for c in result.commands])


class RealGeometryDetourGeneration(unittest.TestCase):
    """Full state-lattice search -> command compression -> STM command
    generation pipeline against DEFAULT_PLANNER_CONFIG, reproducing the
    18-action-plan -> 9-command example (it was 16 -> 6 before the safety margin
    was raised from 1 cm to 3 cm, and 17 -> 9 at the 5 cm goal tolerance before
    it became 3 cm)."""

    def test_real_geometry_detour_yields_nine_exact_stm_strings(self):
        start = Pose(30.0, 100.0, 0.0)
        goal = Pose(110.0, 100.0, 0.0)
        obstacle = Obstacle(obstacle_id=1, x=70, y=95, image_face="N")

        plan = sl.plan_lattice_path(start, goal, [obstacle], DEFAULT_CONFIG)
        self.assertTrue(plan.is_valid)
        self.assertEqual(len(plan.actions), 18)

        result = sg.generate_stm_command_plan(plan, [obstacle], DEFAULT_CONFIG)

        self.assertTrue(result.is_valid)
        self.assertEqual(
            [c.stm_command for c in result.commands],
            [
                "MOV,BR,10", "MOV,FR,20", "MOV,F,20",
                "MOV,FL,10", "MOV,F,10", "MOV,FL,60",
                "MOV,F,10", "MOV,FR,30", "MOV,BR,10",
            ],
        )
        for prev, nxt in zip(result.commands, result.commands[1:]):
            self.assertEqual(prev.end_pose, nxt.start_pose)
        self.assertAlmostEqual(result.final_pose.x, plan.final_pose.x, places=6)
        self.assertAlmostEqual(result.final_pose.y, plan.final_pose.y, places=6)
        print(
            f"\n[stm generation] real-geometry detour: "
            f"{[c.stm_command for c in result.commands]}"
        )


if __name__ == "__main__":
    unittest.main()
