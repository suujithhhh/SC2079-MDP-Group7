"""Rear-reference footprint and camera viewing-pose geometry.

Run from algo/:  python -m unittest discover -s tests -v
"""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import DEFAULT_PLANNER_CONFIG, PlannerConfig  # noqa: E402
from footprint import (  # noqa: E402
    camera_position,
    footprint_corners,
    footprint_hits_obstacle,
    footprint_in_arena,
    local_bounds,
    pose_is_safe,
)
from models import Obstacle, Pose  # noqa: E402
from targets import FACE_OUTWARD, face_is_available, viewing_target  # noqa: E402

CONFIG = DEFAULT_PLANNER_CONFIG
EPS = 1e-9


def bounds(points):
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    return min(xs), max(xs), min(ys), max(ys)


class MeasuredDefaults(unittest.TestCase):
    def test_measured_values_are_the_defaults(self):
        self.assertEqual(CONFIG.robot_front_extent_cm, 24.0)
        self.assertEqual(CONFIG.robot_rear_extent_cm, 0.0)
        self.assertEqual(CONFIG.robot_width_cm, 19.0)
        self.assertEqual(CONFIG.safety_margin_cm, 3.0)
        self.assertEqual(CONFIG.camera_forward_offset_cm, 15.25)
        self.assertEqual(CONFIG.camera_lateral_offset_cm, 0.0)
        self.assertEqual(CONFIG.camera_standoff_cm, 25.0)


class FootprintShape(unittest.TestCase):
    def test_heading_0_raw_footprint(self):
        pose = Pose(50.0, 60.0, 0.0)
        min_x, max_x, min_y, max_y = bounds(
            footprint_corners(pose, CONFIG, include_margin=False))
        self.assertAlmostEqual(min_x, 50.0)
        self.assertAlmostEqual(max_x, 74.0)
        self.assertAlmostEqual(min_y, 50.5)
        self.assertAlmostEqual(max_y, 69.5)

    def test_heading_90_rotates_asymmetric_footprint(self):
        pose = Pose(50.0, 60.0, math.pi / 2)
        min_x, max_x, min_y, max_y = bounds(
            footprint_corners(pose, CONFIG, include_margin=False))
        self.assertAlmostEqual(min_x, 40.5)
        self.assertAlmostEqual(max_x, 59.5)
        self.assertAlmostEqual(min_y, 60.0)   # rear edge at the reference
        self.assertAlmostEqual(max_y, 84.0)   # front edge 24 cm ahead

    def test_heading_180_and_270(self):
        west = bounds(footprint_corners(
            Pose(50.0, 60.0, math.pi), CONFIG, include_margin=False))
        for got, expected in zip(west, (26.0, 50.0, 50.5, 69.5)):
            self.assertAlmostEqual(got, expected)
        south = bounds(footprint_corners(
            Pose(50.0, 60.0, -math.pi / 2), CONFIG, include_margin=False))
        for got, expected in zip(south, (40.5, 59.5, 36.0, 60.0)):
            self.assertAlmostEqual(got, expected)

    def test_margin_applied_exactly_once(self):
        self.assertEqual(local_bounds(CONFIG, include_margin=False),
                         (-0.0, 24.0, -9.5, 9.5))
        self.assertEqual(local_bounds(CONFIG), (-3.0, 27.0, -12.5, 12.5))
        min_x, max_x, min_y, max_y = bounds(
            footprint_corners(Pose(50.0, 60.0, 0.0), CONFIG))
        self.assertAlmostEqual(min_x, 47.0)
        self.assertAlmostEqual(max_x, 77.0)
        self.assertAlmostEqual(min_y, 47.5)
        self.assertAlmostEqual(max_y, 72.5)

    def test_no_hidden_half_length_assumption(self):
        config = PlannerConfig(robot_front_extent_cm=30.0,
                               robot_rear_extent_cm=6.0, robot_width_cm=10.0,
                               safety_margin_cm=2.0)
        self.assertEqual(local_bounds(config), (-8.0, 32.0, -7.0, 7.0))


class ArenaBoundary(unittest.TestCase):
    # (heading, axis, limit, inside_direction) per edge of the margin-inflated
    # footprint: the pose is valid on the inside of `limit` along `axis`.
    CASES = {
        "east": (0.0, [("front", "x", 173.0, -1), ("rear", "x", 3.0, +1),
                       ("left", "y", 187.5, -1), ("right", "y", 12.5, +1)]),
        "north": (math.pi / 2, [("front", "y", 173.0, -1), ("rear", "y", 3.0, +1),
                                ("left", "x", 12.5, +1), ("right", "x", 187.5, -1)]),
        "west": (math.pi, [("front", "x", 27.0, +1), ("rear", "x", 197.0, -1),
                           ("left", "y", 12.5, +1), ("right", "y", 187.5, -1)]),
        "south": (-math.pi / 2, [("front", "y", 27.0, +1), ("rear", "y", 197.0, -1),
                                 ("left", "x", 187.5, -1), ("right", "x", 12.5, +1)]),
    }

    def test_each_extent_against_each_wall(self):
        for name, (heading, edges) in self.CASES.items():
            for edge, axis, limit, inside in edges:
                for step, expected in ((0.01, True), (-0.01, False)):
                    value = limit + inside * step
                    x, y = (value, 100.0) if axis == "x" else (100.0, value)
                    with self.subTest(heading=name, edge=edge, value=value):
                        self.assertIs(footprint_in_arena(
                            Pose(x, y, heading), CONFIG), expected)


class ObstacleCollision(unittest.TestCase):
    pose = Pose(50.0, 50.0, 0.0)   # raw footprint x 50..74, y 40.5..59.5

    def hit(self, obstacle_x, obstacle_y=45.0, pose=None):
        return footprint_hits_obstacle(pose or self.pose, Obstacle(1, obstacle_x, obstacle_y, "N"), CONFIG)

    def test_long_front_side(self):
        self.assertFalse(self.hit(74.0 + 3.5))   # 3.5 cm gap: clear of 3 cm margin
        self.assertTrue(self.hit(74.0 + 2.5))    # 2.5 cm gap: inside the margin
        self.assertTrue(self.hit(60.0))          # inside the long front body

    def test_zero_rear_extent_side(self):
        self.assertFalse(self.hit(50.0 - 3.5 - 10.0))   # right edge 3.5 cm behind
        self.assertTrue(self.hit(50.0 - 2.5 - 10.0))    # right edge 2.5 cm behind
        # 5 cm behind the reference is clear: a centred 28 cm robot would hit it.
        self.assertFalse(self.hit(50.0 - 5.0 - 10.0))

    def test_side(self):
        self.assertFalse(self.hit(60.0, obstacle_y=59.5 + 3.5))
        self.assertTrue(self.hit(60.0, obstacle_y=59.5 + 2.5))

    def test_front_after_rotation(self):
        north = Pose(50.0, 50.0, math.pi / 2)    # raw footprint y 50..74
        self.assertFalse(self.hit(45.0, obstacle_y=74.0 + 3.5, pose=north))
        self.assertTrue(self.hit(45.0, obstacle_y=74.0 + 2.5, pose=north))
        self.assertFalse(self.hit(45.0, obstacle_y=50.0 - 3.5 - 10.0, pose=north))

    def test_pose_is_safe_combines_arena_and_obstacles(self):
        obstacle = Obstacle(1, 80.0, 45.0, "N")
        self.assertTrue(pose_is_safe(self.pose, [obstacle], CONFIG))
        self.assertFalse(pose_is_safe(Pose(70.0, 50.0, 0.0), [obstacle], CONFIG))
        self.assertFalse(pose_is_safe(Pose(180.0, 50.0, 0.0), [], CONFIG))


class ViewingTargets(unittest.TestCase):
    obstacle_xy = (100.0, 100.0)   # centre (105, 105), faces at 100/110

    def check_face(self, face, config):
        obstacle = Obstacle(1, *self.obstacle_xy, face)
        target = viewing_target(obstacle, config)
        out_x, out_y = FACE_OUTWARD[face]
        half = config.obstacle_cm / 2
        face_x, face_y = 105.0 + out_x * half, 105.0 + out_y * half
        lens_x, lens_y = camera_position(target, config)
        # Lens sits on the face's centre line, standoff away from the face.
        self.assertAlmostEqual((lens_x - face_x) * out_x + (lens_y - face_y) * out_y,
                               config.camera_standoff_cm)
        self.assertAlmostEqual((lens_x - face_x) * out_y - (lens_y - face_y) * out_x, 0.0)
        # Robot faces the image.
        self.assertAlmostEqual(math.cos(target.heading), -out_x)
        self.assertAlmostEqual(math.sin(target.heading), -out_y)
        return target, (lens_x, lens_y)

    def test_all_faces_default_standoff(self):
        expected_rear = {"N": (105.0, 150.25), "S": (105.0, 59.75),
                         "E": (150.25, 105.0), "W": (59.75, 105.0)}
        for face, (ex, ey) in expected_rear.items():
            with self.subTest(face=face):
                target, _ = self.check_face(face, CONFIG)
                self.assertAlmostEqual(target.x, ex)
                self.assertAlmostEqual(target.y, ey)
                centre_distance = math.hypot(target.x - 105.0, target.y - 105.0)
                self.assertAlmostEqual(centre_distance, 5.0 + 15.25 + 25.0)

    def test_standoff_is_configurable_without_changing_robot(self):
        config = PlannerConfig(camera_standoff_cm=30.0)
        self.assertEqual(local_bounds(config), local_bounds(CONFIG))
        for face in FACE_OUTWARD:
            with self.subTest(face=face):
                default, _ = self.check_face(face, CONFIG)
                moved, _ = self.check_face(face, config)
                out_x, out_y = FACE_OUTWARD[face]
                shift = 30.0 - CONFIG.camera_standoff_cm   # 30 cm vs the 25 cm default
                self.assertAlmostEqual(moved.x - default.x, shift * out_x)
                self.assertAlmostEqual(moved.y - default.y, shift * out_y)

    def test_forward_offset_moves_pose_along_view_axis_only(self):
        config = PlannerConfig(camera_forward_offset_cm=10.0)
        target, _ = self.check_face("N", config)
        self.assertAlmostEqual(target.x, 105.0)
        self.assertAlmostEqual(target.y, 110.0 + 25.0 + 10.0)

    def test_lateral_offset_shifts_pose_sideways(self):
        config = PlannerConfig(camera_lateral_offset_cm=3.0)
        # Facing south (N face): robot's left is +x, so the pose moves to -x.
        target, lens = self.check_face("N", config)
        self.assertAlmostEqual(target.x, 102.0)
        self.assertAlmostEqual(target.y, 150.25)
        self.assertAlmostEqual(lens[0], 105.0)
        # Facing west (E face): robot's left is -y, so the pose moves to +y.
        target, lens = self.check_face("E", config)
        self.assertAlmostEqual(target.x, 150.25)
        self.assertAlmostEqual(target.y, 108.0)
        self.assertAlmostEqual(lens[1], 105.0)

    def test_face_availability_uses_full_footprint(self):
        # N face at y=145: face at 155, rear reference at 195.25, inflated rear edge 198.25.
        self.assertTrue(face_is_available(Obstacle(1, 100.0, 145.0, ""), "N", CONFIG))
        # N face at y=150: face at 160, rear reference at 200.25, outside the arena
        # (with the 25 cm standoff; this layout was available at 20 cm).
        self.assertFalse(face_is_available(Obstacle(1, 100.0, 150.0, ""), "N", CONFIG))
        # N face at y=160: rear reference at 210.25, outside the arena.
        self.assertFalse(face_is_available(Obstacle(1, 100.0, 160.0, ""), "N", CONFIG))
        # W face near the left wall: face at x=5, rear reference at 5 - 40.25 < 0.
        self.assertFalse(face_is_available(Obstacle(1, 5.0, 100.0, ""), "W", CONFIG))


def load_fake_map() -> dict:
    path = Path(__file__).resolve().parents[1] / "fake_map.json"
    return json.loads(path.read_text(encoding="utf-8"))


class DefaultStartPose(unittest.TestCase):
    def test_default_start_footprint_inside_start_zone(self):
        from config import START_ZONE_CM
        from main import DEFAULT_START_POSE
        from planning_service import parse_plan_request

        self.assertEqual((DEFAULT_START_POSE.x, DEFAULT_START_POSE.y), (20, 10))
        _, fake_map_start, _ = parse_plan_request(load_fake_map())
        self.assertEqual(fake_map_start, DEFAULT_START_POSE)
        min_x, max_x, min_y, max_y = bounds(footprint_corners(DEFAULT_START_POSE, CONFIG))
        self.assertAlmostEqual(min_x, 7.5)
        self.assertAlmostEqual(max_x, 32.5)
        self.assertAlmostEqual(min_y, 7.0)
        self.assertAlmostEqual(max_y, 37.0)
        self.assertTrue(0 <= min_x and max_x <= START_ZONE_CM)
        self.assertTrue(0 <= min_y and max_y <= START_ZONE_CM)


class SamplePlanning(unittest.TestCase):
    def test_fake_map_b3_plans_valid_complete_collision_free_route(self):
        from planning_service import parse_plan_request, plan_request

        payload = load_fake_map()
        payload["mode"] = "B3"
        _, _, obstacles = parse_plan_request(payload)
        response = plan_request(payload)
        self.assertEqual(response["type"], "PLAN_RESULT")
        self.assertIs(response["valid"], True, response.get("error"))
        self.assertIs(response["complete"], True, response.get("error"))
        self.assertTrue(response["waypoints"])
        for waypoint in response["waypoints"]:
            pose = Pose(waypoint["x_cm"], waypoint["y_cm"], waypoint["heading_rad"])
            self.assertTrue(pose_is_safe(pose, obstacles, CONFIG), waypoint)


if __name__ == "__main__":
    unittest.main()
