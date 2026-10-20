"""Protocol-v1 regression tests (first-ever automated coverage of the live
`planning_service.py`/`b3_optimizer.py`/`server.py` request path).

Purely additive: this file only ever imports and calls the existing,
unmodified v1 modules, and asserts their exact current behaviour so that
Original Roadmap Step 5's v2 backend integration can be proven not to have
changed protocol v1 in any way. Most tests inject a hand-built
`B3RouteResult` via `plan_request()`'s own existing `b2_planner`/
`b3_planner` injection points (the same pattern `planning_service.py`
already exposes), so the exact v1 JSON schema is pinned down without
depending on the legacy Dubins search's numeric output. One small
real-pipeline test exercises the actual `find_b2_route()`/`plan_car_path()`
path end to end.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import planning_service as ps  # noqa: E402
from b3_optimizer import B3RouteResult, find_b2_route  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
from models import MotionCommand, Obstacle, PathResult, Pose  # noqa: E402

DEFAULT_CONFIG = DEFAULT_PLANNER_CONFIG


def _valid_payload(**overrides):
    payload = {
        "type": "PLAN_REQUEST",
        "protocol_version": 1,
        "mode": "B2",
        "coordinate_units": "cm",
        "start": {"x": 0, "y": 0, "heading": "E"},
        "obstacles": [{"id": 1, "x": 50, "y": 50, "face": "N"}],
    }
    payload.update(overrides)
    return payload


def _fake_b2(result):
    def planner(start_pose, obstacles, config, allow_alternative_viewing=False):
        return result
    return planner


class ParseRequestUnchanged(unittest.TestCase):
    def test_valid_grid_request_parses(self):
        payload = _valid_payload(
            coordinate_units="grid", start={"x": 2, "y": 1, "heading": "N"},
            obstacles=[{"id": 1, "x": 6, "y": 5, "face": "N"}],
        )
        mode, start, obstacles = ps.parse_plan_request(payload)
        self.assertEqual(mode, "B2")
        self.assertEqual(start, Pose(20.0, 10.0, math.pi / 2))
        self.assertEqual(len(obstacles), 1)

    def test_default_mode_is_b3(self):
        payload = _valid_payload()
        del payload["mode"]
        mode, _, _ = ps.parse_plan_request(payload)
        self.assertEqual(mode, "B3")

    def test_numeric_heading_accepted_as_radians(self):
        payload = _valid_payload(start={"x": 0, "y": 0, "heading": 1.5})
        _, start, _ = ps.parse_plan_request(payload)
        self.assertEqual(start.heading, 1.5)

    def test_wrong_type_rejected(self):
        with self.assertRaises(ps.RequestError):
            ps.parse_plan_request(_valid_payload(type="NOT_A_REQUEST"))

    def test_wrong_protocol_version_rejected(self):
        with self.assertRaises(ps.RequestError):
            ps.parse_plan_request(_valid_payload(protocol_version=2))

    def test_six_obstacles_rejected(self):
        payload = _valid_payload(obstacles=[
            {"id": i, "x": 10, "y": 10, "face": "N"} for i in range(1, 7)
        ])
        with self.assertRaises(ps.RequestError):
            ps.parse_plan_request(payload)

    def test_duplicate_obstacle_id_rejected(self):
        payload = _valid_payload(obstacles=[
            {"id": 1, "x": 10, "y": 10, "face": "N"},
            {"id": 1, "x": 50, "y": 50, "face": "S"},
        ])
        with self.assertRaises(ps.RequestError):
            ps.parse_plan_request(payload)


class ExactResultSchema(unittest.TestCase):
    """Pins the exact `_result_payload()` JSON shape via an injected
    B3RouteResult, independent of the legacy Dubins search's own numerics."""

    def test_successful_result_shape(self):
        start = Pose(0.0, 0.0, 0.0)
        target = Pose(50.0, 0.0, 0.0)
        leg = PathResult(
            poses=[start, target],
            commands=[MotionCommand("F", 50.0)],
            distance_cm=50.0,
            estimated_time_s=2.5,
            is_valid=True,
        )
        result = B3RouteResult(
            obstacle_order=[1], target_poses=[target], leg_results=[leg],
            estimated_time_s=2.5, is_valid=True, orders_considered=1,
            valid_orders=1, rejected_orders=0, is_complete=True,
            skipped_obstacle_ids=[],
        )
        response = ps.plan_request(_valid_payload(), b2_planner=_fake_b2(result))

        self.assertEqual(response["type"], "PLAN_RESULT")
        self.assertEqual(response["protocol_version"], 1)
        self.assertEqual(response["mode"], "B2")
        self.assertTrue(response["valid"])
        self.assertTrue(response["complete"])
        self.assertEqual(response["obstacle_order"], [1])
        self.assertEqual(response["visited_count"], 1)
        self.assertEqual(response["skipped_obstacle_ids"], [])
        self.assertEqual(response["target_poses"], [
            {"x_cm": 50.0, "y_cm": 0.0, "heading_rad": 0.0}
        ])
        self.assertEqual(response["waypoints"], [
            {"x_cm": 0.0, "y_cm": 0.0, "heading_rad": 0.0},
            {"x_cm": 50.0, "y_cm": 0.0, "heading_rad": 0.0},
        ])
        self.assertEqual(response["commands"], [
            {"type": "FW", "value": 50.0, "value_unit": "cm_path_length"},
            {"type": "SNAP", "obstacle_id": 1},
        ])
        self.assertEqual(response["estimated_time_s"], 2.5)
        self.assertEqual(response["minimum_turning_radius_cm"], DEFAULT_CONFIG.minimum_turning_radius_cm)
        self.assertEqual(
            response["command_contract"], "turn values are arc path lengths, not turn angles"
        )
        # v1's own historical field names -- must never be replaced by v2's.
        self.assertNotIn("mission_id", response)
        self.assertNotIn("heading_deg", str(response))

    def test_no_route_result_shape(self):
        result = B3RouteResult(is_valid=False, failure_reason="No route.")
        response = ps.plan_request(_valid_payload(), b2_planner=_fake_b2(result))
        self.assertEqual(response, {
            "type": "PLAN_RESULT",
            "protocol_version": 1,
            "mode": "B2",
            "valid": False,
            "error": "No route.",
        })

    def test_malformed_request_shape(self):
        response = ps.plan_request(_valid_payload(type="BAD"))
        self.assertEqual(response, {
            "type": "PLAN_ERROR",
            "protocol_version": 1,
            "valid": False,
            "error": "type must be PLAN_REQUEST",
        })

    def test_command_kind_bw_maps_correctly(self):
        start = Pose(0.0, 0.0, 0.0)
        target = Pose(-50.0, 0.0, 0.0)
        leg = PathResult(
            poses=[start, target], commands=[MotionCommand("B", 50.0)],
            distance_cm=50.0, estimated_time_s=3.0, is_valid=True,
        )
        result = B3RouteResult(
            obstacle_order=[1], target_poses=[target], leg_results=[leg],
            estimated_time_s=3.0, is_valid=True, orders_considered=1,
            valid_orders=1, rejected_orders=0, is_complete=True,
        )
        response = ps.plan_request(_valid_payload(), b2_planner=_fake_b2(result))
        self.assertEqual(response["commands"][0]["type"], "BW")

    def test_turn_command_kind_passes_through_unchanged(self):
        start = Pose(0.0, 0.0, 0.0)
        target = Pose(10.0, 10.0, math.pi / 2)
        leg = PathResult(
            poses=[start, target], commands=[MotionCommand("FL", 15.7)],
            distance_cm=15.7, estimated_time_s=1.3, is_valid=True,
        )
        result = B3RouteResult(
            obstacle_order=[1], target_poses=[target], leg_results=[leg],
            estimated_time_s=1.3, is_valid=True, orders_considered=1,
            valid_orders=1, rejected_orders=0, is_complete=True,
        )
        response = ps.plan_request(_valid_payload(), b2_planner=_fake_b2(result))
        command = response["commands"][0]
        self.assertEqual(command["type"], "FL")  # unchanged -- not remapped like F/B
        self.assertEqual(command["value"], 15.7)
        self.assertEqual(command["value_unit"], "cm_path_length")


class RealPipelineRegression(unittest.TestCase):
    """One small, fast real end-to-end request through the unmodified legacy
    pipeline -- kept to a single nearby obstacle to stay fast."""

    def test_real_b2_single_obstacle(self):
        payload = _valid_payload(
            mode="B2", start={"x": 20, "y": 10, "heading": "N"},
            obstacles=[{"id": 1, "x": 60, "y": 100, "face": "N"}],
        )
        response = ps.plan_request(payload)
        self.assertEqual(response["type"], "PLAN_RESULT")
        self.assertEqual(response["protocol_version"], 1)
        self.assertTrue(response["valid"])
        self.assertEqual(response["obstacle_order"], [1])
        self.assertEqual(
            response["minimum_turning_radius_cm"], DEFAULT_CONFIG.minimum_turning_radius_cm
        )
        for command in response["commands"]:
            if command["type"] != "SNAP":
                self.assertIn(command["type"], ("FW", "BW", "FL", "FR", "BL", "BR"))
                self.assertIsInstance(command["value"], float)
                self.assertEqual(command["value_unit"], "cm_path_length")


if __name__ == "__main__":
    unittest.main()
