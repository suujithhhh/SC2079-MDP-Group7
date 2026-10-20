"""Pure motion-primitive geometry (STM-executable F/B/FL/FR/BL/BR).

Not connected to planner_interface.py or any other consumer; see
motion_primitives.py's module docstring. Run from algo/:
    python -m unittest discover -s tests -v
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
from models import Pose  # noqa: E402
from motion_primitives import (  # noqa: E402
    InvalidCommandValueError,
    InvalidFractionError,
    UnsupportedActionError,
    normalize_heading,
    pose_at_fraction,
    primitive_length_cm,
    propagate,
    turn_center_cm,
    turn_radius_cm,
)

CONFIG = DEFAULT_PLANNER_CONFIG
EPS = 1e-9
ANGLE_EPS = 1e-6


def assert_pose_close(test, pose, x, y, heading, *, places=6):
    test.assertAlmostEqual(pose.x, x, places=places)
    test.assertAlmostEqual(pose.y, y, places=places)
    test.assertAlmostEqual(
        normalize_heading(pose.heading - heading), 0.0, places=places
    )


class StraightMotion(unittest.TestCase):
    def test_forward_from_east(self):
        start = Pose(0.0, 0.0, 0.0)
        end = propagate(start, "F", 30, CONFIG)
        assert_pose_close(self, end, 30.0, 0.0, 0.0)

    def test_backward_from_east(self):
        start = Pose(0.0, 0.0, 0.0)
        end = propagate(start, "B", 30, CONFIG)
        assert_pose_close(self, end, -30.0, 0.0, 0.0)

    def test_forward_from_north(self):
        start = Pose(5.0, 5.0, math.pi / 2)
        end = propagate(start, "F", 10, CONFIG)
        assert_pose_close(self, end, 5.0, 15.0, math.pi / 2)

    def test_backward_from_north(self):
        start = Pose(5.0, 5.0, math.pi / 2)
        end = propagate(start, "B", 10, CONFIG)
        assert_pose_close(self, end, 5.0, -5.0, math.pi / 2)

    def test_forward_from_non_cardinal_heading(self):
        heading = math.radians(37.0)
        start = Pose(1.0, -2.0, heading)
        end = propagate(start, "F", 12, CONFIG)
        assert_pose_close(
            self,
            end,
            1.0 + 12 * math.cos(heading),
            -2.0 + 12 * math.sin(heading),
            heading,
        )

    def test_backward_from_non_cardinal_heading_does_not_change_heading(self):
        heading = math.radians(-110.0)
        start = Pose(0.0, 0.0, heading)
        end = propagate(start, "B", 7, CONFIG)
        self.assertAlmostEqual(
            normalize_heading(end.heading - heading), 0.0, places=6
        )


class TurnEachUsesItsOwnRadius(unittest.TestCase):
    def test_turn_radius_lookup_matches_config(self):
        self.assertEqual(turn_radius_cm("FL", CONFIG), CONFIG.turn_radius_fl_cm)
        self.assertEqual(turn_radius_cm("FR", CONFIG), CONFIG.turn_radius_fr_cm)
        self.assertEqual(turn_radius_cm("BL", CONFIG), CONFIG.turn_radius_bl_cm)
        self.assertEqual(turn_radius_cm("BR", CONFIG), CONFIG.turn_radius_br_cm)

    def test_radii_are_independent_not_mirrored(self):
        # All four configured radii must remain independently addressable:
        # a bug that mirrors or swaps one command's radius onto another's
        # would collapse two of these values together.
        radii = (
            CONFIG.turn_radius_fl_cm,
            CONFIG.turn_radius_fr_cm,
            CONFIG.turn_radius_bl_cm,
            CONFIG.turn_radius_br_cm,
        )
        self.assertEqual(len(set(radii)), len(radii))


class NinetyDegreeEndpoints(unittest.TestCase):
    """PROTOCOL.md §0/§7's qualitative 90-degree results, from (0,0) facing +x."""

    def test_fl_90_forward_left_heading_plus_90(self):
        start = Pose(0.0, 0.0, 0.0)
        end = propagate(start, "FL", 90, CONFIG)
        r = CONFIG.turn_radius_fl_cm
        assert_pose_close(self, end, r, r, math.pi / 2)
        self.assertGreater(end.x, 0.0)
        self.assertGreater(end.y, 0.0)

    def test_fr_90_forward_right_heading_minus_90(self):
        start = Pose(0.0, 0.0, 0.0)
        end = propagate(start, "FR", 90, CONFIG)
        r = CONFIG.turn_radius_fr_cm
        assert_pose_close(self, end, r, -r, -math.pi / 2)
        self.assertGreater(end.x, 0.0)
        self.assertLess(end.y, 0.0)

    def test_bl_90_backward_heading_increases_rear_moves_back_right(self):
        start = Pose(0.0, 0.0, 0.0)
        end = propagate(start, "BL", 90, CONFIG)
        r = CONFIG.turn_radius_bl_cm
        assert_pose_close(self, end, -r, -r, math.pi / 2)
        self.assertLess(end.x, 0.0)  # backward
        self.assertLess(end.y, 0.0)  # right (of an east-facing robot)

    def test_br_90_backward_heading_decreases_rear_moves_back_left(self):
        start = Pose(0.0, 0.0, 0.0)
        end = propagate(start, "BR", 90, CONFIG)
        r = CONFIG.turn_radius_br_cm
        assert_pose_close(self, end, -r, r, -math.pi / 2)
        self.assertLess(end.x, 0.0)  # backward
        self.assertGreater(end.y, 0.0)  # left (of an east-facing robot)


class TurnHeadingSigns(unittest.TestCase):
    def test_fl_and_bl_increase_heading(self):
        for action in ("FL", "BL"):
            with self.subTest(action=action):
                start = Pose(0.0, 0.0, 0.0)
                end = propagate(start, action, 30, CONFIG)
                self.assertAlmostEqual(end.heading, math.radians(30), places=6)

    def test_fr_and_br_decrease_heading(self):
        for action in ("FR", "BR"):
            with self.subTest(action=action):
                start = Pose(0.0, 0.0, 0.0)
                end = propagate(start, action, 30, CONFIG)
                self.assertAlmostEqual(end.heading, math.radians(-30), places=6)


class TurnAnglesSweep(unittest.TestCase):
    ANGLES = (10, 90, 180, 270, 360)
    ACTIONS = ("FL", "FR", "BL", "BR")

    def test_heading_delta_matches_signed_angle_for_every_action_and_angle(self):
        sign = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
        for action in self.ACTIONS:
            for degrees in self.ANGLES:
                with self.subTest(action=action, degrees=degrees):
                    start = Pose(0.0, 0.0, 0.0)
                    end = propagate(start, action, degrees, CONFIG)
                    expected = normalize_heading(
                        sign[action] * math.radians(degrees)
                    )
                    self.assertAlmostEqual(
                        normalize_heading(end.heading - expected), 0.0, places=6
                    )

    def test_360_degree_turn_returns_approximately_to_start_position(self):
        for action in self.ACTIONS:
            with self.subTest(action=action):
                start = Pose(3.0, -4.0, math.radians(50))
                end = propagate(start, action, 360, CONFIG)
                self.assertAlmostEqual(end.x, start.x, places=6)
                self.assertAlmostEqual(end.y, start.y, places=6)
                self.assertAlmostEqual(
                    normalize_heading(end.heading - start.heading), 0.0, places=6
                )

    def test_endpoint_lies_on_the_commands_own_radius_circle(self):
        # For every turn, the straight-line distance from start to end must
        # be the chord of an arc of that command's own radius and angle —
        # this fails if radii are swapped or mirrored between commands.
        radii = {
            "FL": CONFIG.turn_radius_fl_cm,
            "FR": CONFIG.turn_radius_fr_cm,
            "BL": CONFIG.turn_radius_bl_cm,
            "BR": CONFIG.turn_radius_br_cm,
        }
        for action in self.ACTIONS:
            for degrees in (10, 90, 180, 270):
                with self.subTest(action=action, degrees=degrees):
                    start = Pose(0.0, 0.0, 0.0)
                    end = propagate(start, action, degrees, CONFIG)
                    r = radii[action]
                    theta = math.radians(degrees)
                    expected_chord = 2 * r * math.sin(theta / 2)
                    actual_chord = math.hypot(end.x - start.x, end.y - start.y)
                    self.assertAlmostEqual(actual_chord, expected_chord, places=6)


class SequentialPropagation(unittest.TestCase):
    """F,20 -> FL,90 -> F,15 from (0,0,0), checked against hand-derived poses.

    Each expected pose below is computed independently of propagate()'s own
    arc math, so this proves each command starts from the exact prior
    endpoint (no re-snapping) rather than merely that propagate() agrees
    with itself.
    """

    def test_each_step_matches_the_analytically_derived_pose(self):
        r = CONFIG.turn_radius_fl_cm

        pose = Pose(0.0, 0.0, 0.0)

        # F,20 from (0,0,0) facing +x: straight line along heading 0.
        pose = propagate(pose, "F", 20, CONFIG)
        assert_pose_close(self, pose, 20.0, 0.0, 0.0)

        # FL,90 from (20,0,0): facing +x, the centre of curvature is directly
        # left, in the +y direction, at distance r, i.e. (20, r). Rotating
        # (20,0)-(20,r) by +90 deg around that centre lands at (20+r, r),
        # heading +90 deg -- the same FL-90 shape as the origin case,
        # offset by the F,20.
        pose = propagate(pose, "FL", 90, CONFIG)
        assert_pose_close(self, pose, 20.0 + r, r, math.pi / 2)

        # F,15 from (20+r, r, 90 deg): straight line along heading +90 deg,
        # i.e. pure +y motion.
        pose = propagate(pose, "F", 15, CONFIG)
        assert_pose_close(self, pose, 20.0 + r, r + 15.0, math.pi / 2)


class HeadingNormalization(unittest.TestCase):
    def test_normalize_heading_range(self):
        for degrees in range(-720, 721, 15):
            angle = normalize_heading(math.radians(degrees))
            self.assertGreater(angle, -math.pi - ANGLE_EPS)
            self.assertLessEqual(angle, math.pi + ANGLE_EPS)

    def test_wraparound_turn_stays_normalized(self):
        start = Pose(0.0, 0.0, math.radians(170))
        end = propagate(start, "FL", 60, CONFIG)  # crosses +/-180 degrees
        self.assertAlmostEqual(end.heading, math.radians(-130), places=6)

    def test_wraparound_negative_side(self):
        start = Pose(0.0, 0.0, math.radians(-170))
        end = propagate(start, "FR", 60, CONFIG)  # crosses -180/+180 degrees
        self.assertAlmostEqual(end.heading, math.radians(130), places=6)

    def test_positive_pi_normalizes_to_negative_pi(self):
        # normalize_heading's range is the half-open interval [-pi, pi):
        # +pi is out of range and wraps to the interval's own -pi boundary.
        self.assertAlmostEqual(normalize_heading(math.pi), -math.pi, places=12)

    def test_negative_pi_remains_negative_pi(self):
        # -pi is already inside [-pi, pi) and must be returned unchanged.
        self.assertAlmostEqual(normalize_heading(-math.pi), -math.pi, places=12)


class InputValidation(unittest.TestCase):
    def test_unsupported_action_names_rejected(self):
        start = Pose(0.0, 0.0, 0.0)
        for bad_action in ("FW", "BW", "L", "R", "MOV", "fl", ""):
            with self.subTest(action=bad_action):
                with self.assertRaises(UnsupportedActionError):
                    propagate(start, bad_action, 10, CONFIG)

    def test_zero_value_rejected(self):
        start = Pose(0.0, 0.0, 0.0)
        for action in ("F", "B", "FL", "FR", "BL", "BR"):
            with self.subTest(action=action):
                with self.assertRaises(InvalidCommandValueError):
                    propagate(start, action, 0, CONFIG)

    def test_negative_value_rejected(self):
        start = Pose(0.0, 0.0, 0.0)
        with self.assertRaises(InvalidCommandValueError):
            propagate(start, "F", -5, CONFIG)
        with self.assertRaises(InvalidCommandValueError):
            propagate(start, "FL", -90, CONFIG)

    def test_fractional_value_rejected(self):
        start = Pose(0.0, 0.0, 0.0)
        with self.assertRaises(InvalidCommandValueError):
            propagate(start, "F", 12.5, CONFIG)
        with self.assertRaises(InvalidCommandValueError):
            propagate(start, "FL", 90.0, CONFIG)  # integral float still rejected

    def test_string_value_rejected(self):
        start = Pose(0.0, 0.0, 0.0)
        with self.assertRaises(InvalidCommandValueError):
            propagate(start, "F", "30", CONFIG)

    def test_boolean_value_rejected(self):
        start = Pose(0.0, 0.0, 0.0)
        with self.assertRaises(InvalidCommandValueError):
            propagate(start, "F", True, CONFIG)
        with self.assertRaises(InvalidCommandValueError):
            propagate(start, "FL", False, CONFIG)

    def test_straight_commands_accept_999_and_reject_1000(self):
        start = Pose(0.0, 0.0, 0.0)
        for action in ("F", "B"):
            with self.subTest(action=action):
                propagate(start, action, 999, CONFIG)  # must not raise
                with self.assertRaises(InvalidCommandValueError):
                    propagate(start, action, 1000, CONFIG)

    def test_turn_commands_accept_360_and_reject_361(self):
        start = Pose(0.0, 0.0, 0.0)
        for action in ("FL", "FR", "BL", "BR"):
            with self.subTest(action=action):
                propagate(start, action, 360, CONFIG)  # must not raise
                with self.assertRaises(InvalidCommandValueError):
                    propagate(start, action, 361, CONFIG)

    def test_half_steer_and_non_stm_actions_rejected(self):
        start = Pose(0.0, 0.0, 0.0)
        for bad_action in ("TURN,L", "TURN,R", "SNAP"):
            with self.subTest(action=bad_action):
                with self.assertRaises(UnsupportedActionError):
                    propagate(start, bad_action, 10, CONFIG)


class PoseImmutability(unittest.TestCase):
    def test_input_pose_is_not_mutated(self):
        start = Pose(1.0, 2.0, math.radians(45))
        snapshot = Pose(start.x, start.y, start.heading)
        propagate(start, "FL", 90, CONFIG)
        self.assertEqual(start.x, snapshot.x)
        self.assertEqual(start.y, snapshot.y)
        self.assertEqual(start.heading, snapshot.heading)


class PoseAtFraction(unittest.TestCase):
    """Fraction 0/0.5/1 across straight and all four turn directions."""

    def test_fraction_zero_returns_start_pose_unchanged(self):
        start = Pose(3.0, -4.0, math.radians(50))
        for action, value in (("F", 20), ("B", 20), ("FL", 90), ("FR", 90),
                              ("BL", 90), ("BR", 90)):
            with self.subTest(action=action):
                result = pose_at_fraction(start, action, value, 0.0, CONFIG)
                self.assertEqual(result, start)

    def test_fraction_one_matches_propagate_exactly(self):
        start = Pose(3.0, -4.0, math.radians(50))
        for action, value in (("F", 20), ("B", 20), ("FL", 90), ("FR", 90),
                              ("BL", 90), ("BR", 90)):
            with self.subTest(action=action):
                expected = propagate(start, action, value, CONFIG)
                result = pose_at_fraction(start, action, value, 1.0, CONFIG)
                self.assertEqual(result, expected)

    def test_fraction_half_straight_is_halfway_along_heading(self):
        start = Pose(0.0, 0.0, 0.0)
        for action, sign in (("F", 1), ("B", -1)):
            with self.subTest(action=action):
                half = pose_at_fraction(start, action, 20, 0.5, CONFIG)
                assert_pose_close(self, half, sign * 10.0, 0.0, 0.0)

    def test_fraction_half_turn_is_the_half_angle_arc(self):
        start = Pose(0.0, 0.0, 0.0)
        sign = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}
        for action in ("FL", "FR", "BL", "BR"):
            with self.subTest(action=action):
                half = pose_at_fraction(start, action, 90, 0.5, CONFIG)
                expected_heading = normalize_heading(sign[action] * math.radians(45))
                self.assertAlmostEqual(
                    normalize_heading(half.heading - expected_heading), 0.0, places=9
                )
                full = propagate(start, action, 90, CONFIG)
                # Half-fraction endpoint must sit on the same radius circle,
                # around the same fixed centre, as the full command.
                r = turn_radius_cm(action, CONFIG)
                cx, cy = turn_center_cm(start, action, CONFIG)
                self.assertAlmostEqual(
                    math.hypot(half.x - cx, half.y - cy), r, places=6
                )
                self.assertNotEqual(half, full)

    def test_fraction_rejects_booleans_nan_inf_and_out_of_range(self):
        start = Pose(0.0, 0.0, 0.0)
        for bad_fraction in (True, False, float("nan"), float("inf"),
                             float("-inf"), -0.1, 1.1, 2, "0.5"):
            with self.subTest(fraction=bad_fraction):
                with self.assertRaises(InvalidFractionError):
                    pose_at_fraction(start, "F", 20, bad_fraction, CONFIG)

    def test_fraction_still_validates_action_and_value(self):
        start = Pose(0.0, 0.0, 0.0)
        with self.assertRaises(UnsupportedActionError):
            pose_at_fraction(start, "FW", 20, 0.5, CONFIG)
        with self.assertRaises(InvalidCommandValueError):
            pose_at_fraction(start, "F", 0, 0.5, CONFIG)


class ArcLength(unittest.TestCase):
    def test_straight_length_is_centimetres(self):
        self.assertEqual(primitive_length_cm("F", 42, CONFIG), 42.0)
        self.assertEqual(primitive_length_cm("B", 7, CONFIG), 7.0)

    def test_turn_length_is_radius_times_radians(self):
        for action, attr in (
            ("FL", "turn_radius_fl_cm"),
            ("FR", "turn_radius_fr_cm"),
            ("BL", "turn_radius_bl_cm"),
            ("BR", "turn_radius_br_cm"),
        ):
            with self.subTest(action=action):
                radius = getattr(CONFIG, attr)
                expected = radius * math.radians(90)
                self.assertAlmostEqual(
                    primitive_length_cm(action, 90, CONFIG), expected, places=9
                )

    def test_arc_length_rejects_invalid_values_like_propagate(self):
        with self.assertRaises(InvalidCommandValueError):
            primitive_length_cm("F", 0, CONFIG)
        with self.assertRaises(UnsupportedActionError):
            primitive_length_cm("FW", 10, CONFIG)


if __name__ == "__main__":
    unittest.main()
