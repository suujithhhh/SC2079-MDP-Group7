"""Swept-path collision checking for the six STM primitives (swept_collision.py).

Not connected to planner_interface.py or any other consumer. Scenarios
below are analytically constructed (start/end footprints, obstacle cells
and arena bounds are computed or verified by hand, not fudged to pass).
Run from algo/:
    python -m unittest discover -s tests -v
"""

import dataclasses
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import footprint  # noqa: E402
import motion_primitives as mp  # noqa: E402
import swept_collision as sc  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
from models import Obstacle, Pose  # noqa: E402

CONFIG = DEFAULT_PLANNER_CONFIG
ALL_ACTIONS = ("F", "B", "FL", "FR", "BL", "BR")
TURN_ACTIONS = ("FL", "FR", "BL", "BR")


class SampleEndpoints(unittest.TestCase):
    def test_first_sample_is_exact_start(self):
        start = Pose(12.0, 34.0, math.radians(17.0))
        for action in ALL_ACTIONS:
            value = 20 if action in ("F", "B") else 90
            with self.subTest(action=action):
                samples = sc.sample_primitive(start, action, value, CONFIG)
                self.assertEqual(samples[0], start)

    def test_last_sample_matches_propagate_exactly(self):
        start = Pose(12.0, 34.0, math.radians(17.0))
        for action in ALL_ACTIONS:
            value = 20 if action in ("F", "B") else 90
            with self.subTest(action=action):
                samples = sc.sample_primitive(start, action, value, CONFIG)
                expected = mp.propagate(start, action, value, CONFIG)
                self.assertEqual(samples[-1], expected)


class ImmutableReturns(unittest.TestCase):
    def test_sample_primitive_returns_a_tuple(self):
        samples = sc.sample_primitive(Pose(50.0, 50.0, 0.0), "F", 20, CONFIG)
        self.assertIsInstance(samples, tuple)

    def test_safe_primitive_samples_returns_a_tuple_or_none(self):
        result = sc.safe_primitive_samples(Pose(50.0, 50.0, 0.0), "F", 20, [], CONFIG)
        self.assertIsInstance(result, tuple)

    def test_inputs_are_not_mutated(self):
        start = Pose(50.0, 50.0, 0.0)
        start_snapshot = Pose(start.x, start.y, start.heading)
        obstacles = [Obstacle(obstacle_id=1, x=100, y=100, image_face="N")]
        obstacles_snapshot = list(obstacles)
        config_snapshot = dataclasses.replace(CONFIG)

        sc.sample_primitive(start, "FL", 90, CONFIG)
        sc.primitive_is_safe(start, "FL", 90, obstacles, CONFIG)

        self.assertEqual(start, start_snapshot)
        self.assertEqual(obstacles, obstacles_snapshot)
        self.assertEqual(CONFIG, config_snapshot)


class StraightSampleSpacing(unittest.TestCase):
    def test_adjacent_rear_reference_spacing_respects_the_bound(self):
        start = Pose(0.0, 0.0, math.radians(23.0))
        for value in (1, 7, 20, 55, 200, 999):
            for action in ("F", "B"):
                with self.subTest(action=action, value=value):
                    samples = sc.sample_primitive(start, action, value, CONFIG)
                    for a, b in zip(samples, samples[1:]):
                        step = math.hypot(b.x - a.x, b.y - a.y)
                        self.assertLessEqual(
                            step, CONFIG.sweep_max_point_step_cm + 1e-9
                        )


class TurnCornerDisplacement(unittest.TestCase):
    def test_every_corner_respects_the_configured_bound_between_samples(self):
        start = Pose(50.0, 50.0, math.radians(10.0))
        for action in TURN_ACTIONS:
            for value in (5, 45, 90, 181, 300, 360):
                with self.subTest(action=action, value=value):
                    samples = sc.sample_primitive(start, action, value, CONFIG)
                    corner_lists = [
                        footprint.footprint_corners(pose, CONFIG) for pose in samples
                    ]
                    for corners_a, corners_b in zip(corner_lists, corner_lists[1:]):
                        for (ax, ay), (bx, by) in zip(corners_a, corners_b):
                            step = math.hypot(bx - ax, by - ay)
                            self.assertLessEqual(
                                step, CONFIG.sweep_max_point_step_cm + 1e-6
                            )

    def test_all_four_turns_use_their_own_independent_radius(self):
        start = Pose(50.0, 50.0, 0.0)
        radii = {
            "FL": CONFIG.turn_radius_fl_cm,
            "FR": CONFIG.turn_radius_fr_cm,
            "BL": CONFIG.turn_radius_bl_cm,
            "BR": CONFIG.turn_radius_br_cm,
        }
        for action, radius in radii.items():
            with self.subTest(action=action):
                cx, cy = mp.turn_center_cm(start, action, CONFIG)
                self.assertAlmostEqual(
                    math.hypot(start.x - cx, start.y - cy), radius, places=9
                )


class SafeEmptyArena(unittest.TestCase):
    def test_every_action_is_safe_with_no_obstacles_well_inside_the_arena(self):
        start = Pose(100.0, 100.0, 0.0)
        for action in ALL_ACTIONS:
            value = 20 if action in ("F", "B") else 90
            with self.subTest(action=action):
                self.assertTrue(sc.primitive_is_safe(start, action, value, [], CONFIG))


class StartAndEndpointCollisions(unittest.TestCase):
    def test_start_pose_collision_is_rejected(self):
        start = Pose(50.0, 50.0, 0.0)
        # Obstacle overlapping the start footprint itself.
        obstacle = Obstacle(obstacle_id=1, x=55, y=45, image_face="N")
        self.assertTrue(footprint.footprint_hits_obstacle(start, obstacle, CONFIG))
        self.assertFalse(sc.primitive_is_safe(start, "F", 20, [obstacle], CONFIG))
        self.assertIsNone(
            sc.safe_primitive_samples(start, "F", 20, [obstacle], CONFIG)
        )

    def test_endpoint_collision_is_rejected(self):
        start = Pose(50.0, 50.0, 0.0)
        end = mp.propagate(start, "F", 20, CONFIG)
        obstacle = Obstacle(obstacle_id=2, x=82, y=45, image_face="N")
        self.assertFalse(footprint.footprint_hits_obstacle(start, obstacle, CONFIG))
        self.assertTrue(footprint.footprint_hits_obstacle(end, obstacle, CONFIG))
        self.assertFalse(sc.primitive_is_safe(start, "F", 20, [obstacle], CONFIG))


class MidPathOnlyCollisions(unittest.TestCase):
    """Collisions that occur strictly between the start and end poses.

    Each fixture is verified directly against footprint.footprint_hits_obstacle
    at start/mid/end before being used, so the test does not merely trust
    swept_collision's own sampling to have picked the "right" obstacle.
    """

    def test_straight_motion_mid_path_collision_is_rejected(self):
        start = Pose(50.0, 50.0, 0.0)
        value = 50  # longer than the ~26 cm inflated footprint length, so
        # start's and end's footprint windows do not overlap and leave a
        # gap only a middle sample can cover.
        end = mp.propagate(start, "F", value, CONFIG)
        mid = mp.pose_at_fraction(start, "F", value, 0.5, CONFIG)
        obstacle = Obstacle(obstacle_id=3, x=82, y=45, image_face="N")

        self.assertFalse(footprint.footprint_hits_obstacle(start, obstacle, CONFIG))
        self.assertFalse(footprint.footprint_hits_obstacle(end, obstacle, CONFIG))
        self.assertTrue(footprint.footprint_hits_obstacle(mid, obstacle, CONFIG))

        self.assertFalse(sc.primitive_is_safe(start, "F", value, [obstacle], CONFIG))

    def test_curved_motion_mid_path_collision_is_rejected(self):
        start = Pose(50.0, 50.0, 0.0)
        end = mp.propagate(start, "FL", 90, CONFIG)
        mid = mp.pose_at_fraction(start, "FL", 90, 0.5, CONFIG)
        obstacle = Obstacle(obstacle_id=4, x=87, y=61, image_face="N")

        self.assertFalse(footprint.footprint_hits_obstacle(start, obstacle, CONFIG))
        self.assertFalse(footprint.footprint_hits_obstacle(end, obstacle, CONFIG))
        self.assertTrue(footprint.footprint_hits_obstacle(mid, obstacle, CONFIG))

        self.assertFalse(sc.primitive_is_safe(start, "FL", 90, [obstacle], CONFIG))

    def test_coarse_custom_sweep_step_cannot_tunnel_through_the_obstacle(self):
        # An absurdly coarse step collapses sampling to just start+end (2
        # samples), which footprint_hits_obstacle alone would call safe
        # (checked above) -- the anti-tunnelling guard must still reject it.
        coarse_config = dataclasses.replace(CONFIG, sweep_max_point_step_cm=1000.0)
        start = Pose(50.0, 50.0, 0.0)
        obstacle = Obstacle(obstacle_id=4, x=87, y=61, image_face="N")

        samples = sc.sample_primitive(start, "FL", 90, coarse_config)
        self.assertEqual(len(samples), 2)  # confirms the coarse step really
        # did collapse sampling to just the two endpoints.

        self.assertFalse(
            sc.primitive_is_safe(start, "FL", 90, [obstacle], coarse_config)
        )


class ArenaBoundaryDuringCurve(unittest.TestCase):
    def test_mid_arc_arena_exit_is_rejected_even_when_endpoints_are_safe(self):
        # Found by search: start and end footprints are both fully inside
        # the 200x200 cm arena, but the swept arc's front-right corner dips
        # below y=0 partway through (verified directly below).
        start = Pose(13.0, 30.0, math.radians(300.0))
        action, value = "FL", 60

        self.assertTrue(footprint.footprint_in_arena(start, CONFIG))
        end = mp.propagate(start, action, value, CONFIG)
        self.assertTrue(footprint.footprint_in_arena(end, CONFIG))

        mid = mp.pose_at_fraction(start, action, value, 0.5, CONFIG)
        self.assertFalse(footprint.footprint_in_arena(mid, CONFIG))

        self.assertFalse(sc.primitive_is_safe(start, action, value, [], CONFIG))
        self.assertIsNone(
            sc.safe_primitive_samples(start, action, value, [], CONFIG)
        )


class InvalidSweepStepRejected(unittest.TestCase):
    def test_zero_negative_and_non_finite_steps_rejected(self):
        start = Pose(50.0, 50.0, 0.0)
        for bad_step in (0, 0.0, -1.0, float("inf"), float("nan"), float("-inf")):
            with self.subTest(step=bad_step):
                bad_config = dataclasses.replace(
                    CONFIG, sweep_max_point_step_cm=bad_step
                )
                with self.assertRaises(sc.InvalidSweepStepError):
                    sc.sample_primitive(start, "F", 20, bad_config)

    def test_boolean_step_rejected(self):
        start = Pose(50.0, 50.0, 0.0)
        bad_config = dataclasses.replace(CONFIG, sweep_max_point_step_cm=True)
        with self.assertRaises(sc.InvalidSweepStepError):
            sc.sample_primitive(start, "F", 20, bad_config)


class ExistingPrimitiveValidationStillEffective(unittest.TestCase):
    def test_unsupported_action_rejected(self):
        start = Pose(50.0, 50.0, 0.0)
        with self.assertRaises(mp.UnsupportedActionError):
            sc.sample_primitive(start, "FW", 20, CONFIG)
        with self.assertRaises(mp.UnsupportedActionError):
            sc.primitive_is_safe(start, "FW", 20, [], CONFIG)

    def test_invalid_value_rejected(self):
        start = Pose(50.0, 50.0, 0.0)
        for bad_value in (0, -5, 12.5, "30", True, 1000):
            with self.subTest(value=bad_value):
                with self.assertRaises(mp.InvalidCommandValueError):
                    sc.sample_primitive(start, "F", bad_value, CONFIG)

    def test_turn_specific_range_violation_rejected(self):
        start = Pose(50.0, 50.0, 0.0)
        with self.assertRaises(mp.InvalidCommandValueError):
            sc.sample_primitive(start, "FL", 361, CONFIG)


if __name__ == "__main__":
    unittest.main()
