"""Original Roadmap Step 5 -- protocol-v2 HTTP planning service tests.

Most tests inject a fake B2/B3 planner returning a hand-built
`ExecutableRouteResult` (mirroring the injectable-planner pattern
`test_executable_route_optimizer.py` already uses for leg planners), so
request parsing, response serialization, heading normalisation, and the
waypoint-index mapping are all tested without running the real, sometimes
slow, executable pipeline. `RealPipelineIntegration` is the focused
exception that exercises the actual pipeline end to end, kept small
(1-2 obstacles) to stay fast.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import planning_service_v2 as psv2  # noqa: E402
import bounded_prefix_planning as bp  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG, PlannerConfig  # noqa: E402
from dataclasses import replace  # noqa: E402
from executable_route_optimizer import (  # noqa: E402
    ExecutableRouteInvariantError,
    ExecutableRouteLeg,
    ExecutableRouteResult,
)
from models import Obstacle, Pose  # noqa: E402
from route_prefix_search import PrefixRoute  # noqa: E402
from stm_command_generation import STMCommandPlan, STMMotionCommand  # noqa: E402

DEFAULT_CONFIG = DEFAULT_PLANNER_CONFIG


def _command(direction, value, unit, start_pose, end_pose, samples=None):
    return STMMotionCommand(
        direction=direction, value=value, unit=unit,
        stm_command=f"MOV,{direction},{value}",
        start_pose=start_pose, end_pose=end_pose,
        length_cm=float(value), samples=samples or (start_pose, end_pose),
    )


def _plan(commands, final_pose, total_distance=0.0, total_cost=0.0):
    return STMCommandPlan(
        is_valid=True, commands=tuple(commands), waypoints=(),
        total_distance_cm=total_distance, total_cost=total_cost,
        expansions=0, final_pose=final_pose, reason=None,
    )


def _leg(obstacle_id, target_pose, command_plan):
    return ExecutableRouteLeg(
        obstacle_id=obstacle_id, target_pose=target_pose, command_plan=command_plan
    )


def _result(legs, obstacle_order, target_poses, final_pose, **overrides):
    fields = dict(
        obstacle_order=obstacle_order, target_poses=target_poses, legs=tuple(legs),
        total_distance_cm=10.0, total_cost=10.0, is_valid=True, is_complete=True,
        orders_considered=1, valid_orders=1, rejected_orders=0, pruned_orders=0,
        skipped_obstacle_ids=(), final_pose=final_pose, failure_reason=None,
    )
    fields.update(overrides)
    return ExecutableRouteResult(**fields)


def _fake_planner(result):
    def planner(start_pose, obstacles, config=DEFAULT_CONFIG, allow_alternative_viewing=False):
        return result
    return planner


def _valid_payload(**overrides):
    payload = {
        "type": "PLAN_REQUEST",
        "protocol_version": 2,
        "mission_id": "test-mission-1",
        "mode": "B2",
        "coordinate_units": "cm",
        "start": {"x": 0, "y": 0, "heading_deg": 0},
        "obstacles": [{"id": 1, "x": 10, "y": 0, "face": "N"}],
    }
    payload.update(overrides)
    return payload


START = Pose(0.0, 0.0, 0.0)
OBSTACLE_1 = Obstacle(obstacle_id=1, x=10, y=0, image_face="N")


class HeadingNormalization(unittest.TestCase):
    def test_zero_degrees(self):
        self.assertEqual(psv2._normalized_heading_deg(0.0), 0.0)

    def test_ninety_degrees(self):
        self.assertAlmostEqual(psv2._normalized_heading_deg(math.pi / 2), 90.0, places=9)

    def test_one_eighty_degrees(self):
        self.assertAlmostEqual(psv2._normalized_heading_deg(math.pi), 180.0, places=9)

    def test_negative_ninety_becomes_two_seventy(self):
        self.assertAlmostEqual(psv2._normalized_heading_deg(-math.pi / 2), 270.0, places=9)

    def test_wrapped_heading(self):
        # 5*pi/2 radians == 450 degrees, should wrap to 90.
        self.assertAlmostEqual(psv2._normalized_heading_deg(5 * math.pi / 2), 90.0, places=9)

    def test_negative_zero_is_serialized_as_positive_zero(self):
        degrees = psv2._normalized_heading_deg(-0.0)
        self.assertEqual(degrees, 0.0)
        self.assertFalse(math.copysign(1.0, degrees) < 0)

    def test_pose_json_uses_heading_deg_not_heading_rad(self):
        payload = psv2._pose_json(Pose(1.0, 2.0, math.pi))
        self.assertIn("heading_deg", payload)
        self.assertNotIn("heading_rad", payload)
        self.assertAlmostEqual(payload["heading_deg"], 180.0, places=9)


class RequestHeadingIsDegrees(unittest.TestCase):
    def test_start_requires_heading_deg_field(self):
        payload = _valid_payload()
        payload["start"] = {"x": 0, "y": 0, "heading": 0}  # v1-style key, not heading_deg
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_heading_deg_must_be_numeric(self):
        payload = _valid_payload()
        payload["start"]["heading_deg"] = "90"
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_heading_deg_rejects_boolean(self):
        payload = _valid_payload()
        payload["start"]["heading_deg"] = True
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_heading_deg_rejects_non_finite(self):
        payload = _valid_payload()
        payload["start"]["heading_deg"] = float("nan")
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_heading_deg_is_converted_to_radians_internally(self):
        payload = _valid_payload()
        payload["start"] = {"x": 0, "y": 0, "heading_deg": 90}
        _, _, start, _, _, _ = psv2.parse_plan_request_v2(payload)
        self.assertAlmostEqual(start.heading, math.pi / 2, places=9)


class MissionIdRequired(unittest.TestCase):
    def test_missing_mission_id_raises(self):
        payload = _valid_payload()
        del payload["mission_id"]
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_empty_mission_id_raises(self):
        payload = _valid_payload(mission_id="")
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_non_string_mission_id_raises(self):
        for bad in (123, None, True, ["a"]):
            with self.subTest(bad=bad):
                payload = _valid_payload(mission_id=bad)
                with self.assertRaises(psv2.RequestErrorV2):
                    psv2.parse_plan_request_v2(payload)

    def test_missing_mission_id_via_plan_request_v2_is_bad_request(self):
        payload = _valid_payload()
        del payload["mission_id"]
        response = psv2.plan_request_v2(payload)
        self.assertEqual(response["type"], "PLAN_ERROR")
        self.assertEqual(response["protocol_version"], 2)

    def test_mission_id_echoed_on_success(self):
        final_pose = Pose(10.0, 0.0, 0.0)
        plan = _plan([], final_pose)
        result = _result([_leg(1, Pose(10.0, 0.0, 0.0), plan)], (1,), (Pose(10.0, 0.0, 0.0),), final_pose)
        payload = _valid_payload(mission_id="mission-xyz")
        response = psv2.plan_request_v2(payload, b2_planner=_fake_planner(result))
        self.assertEqual(response["mission_id"], "mission-xyz")

    def test_mission_id_echoed_on_no_route(self):
        result = ExecutableRouteResult(
            obstacle_order=(), target_poses=(), legs=(),
            total_distance_cm=0.0, total_cost=0.0, is_valid=False, is_complete=False,
            orders_considered=1, valid_orders=0, rejected_orders=1, pruned_orders=0,
            skipped_obstacle_ids=(), final_pose=None, failure_reason="No complete executable route.",
        )
        payload = _valid_payload(mission_id="mission-no-route")
        response = psv2.plan_request_v2(payload, b2_planner=_fake_planner(result))
        self.assertEqual(response["mission_id"], "mission-no-route")
        self.assertFalse(response["valid"])
        self.assertEqual(response["type"], "PLAN_RESULT")


class RequestValidation(unittest.TestCase):
    def test_valid_request_parses(self):
        (mission_id, mode, start, obstacles, allow_alt,
         motion_profile) = psv2.parse_plan_request_v2(_valid_payload())
        self.assertEqual(mission_id, "test-mission-1")
        self.assertEqual(mode, "B2")
        self.assertEqual(start, START)
        self.assertEqual(obstacles, [OBSTACLE_1])
        self.assertFalse(allow_alt)
        self.assertEqual(motion_profile, "PRECISION")

    def test_allow_alternative_viewing_defaults_false(self):
        _, _, _, _, allow_alt, _ = psv2.parse_plan_request_v2(_valid_payload())
        self.assertFalse(allow_alt)

    def test_allow_alternative_viewing_true_accepted(self):
        _, _, _, _, allow_alt, _ = psv2.parse_plan_request_v2(
            _valid_payload(allow_alternative_viewing=True)
        )
        self.assertTrue(allow_alt)

    def test_allow_alternative_viewing_rejects_non_boolean(self):
        payload = _valid_payload(allow_alternative_viewing="true")
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_wrong_type_rejected(self):
        payload = _valid_payload(type="NOT_A_PLAN_REQUEST")
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_wrong_protocol_version_rejected(self):
        payload = _valid_payload(protocol_version=1)
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_mode_is_required(self):
        payload = _valid_payload()
        del payload["mode"]
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_invalid_mode_rejected(self):
        payload = _valid_payload(mode="B4")
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_coordinate_units_is_required(self):
        payload = _valid_payload()
        del payload["coordinate_units"]
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_invalid_coordinate_units_rejected(self):
        payload = _valid_payload(coordinate_units="furlongs")
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)


class AllowAlternativeViewingStrictBoolean(unittest.TestCase):
    """`allow_alternative_viewing` is optional and defaults to `false` only
    when the key is absent from the request. If the key is present, its
    value must be an actual boolean -- an explicit JSON `null` is rejected,
    not silently treated the same as an absent key."""

    def test_missing_key_defaults_to_false(self):
        payload = _valid_payload()
        payload.pop("allow_alternative_viewing", None)
        _, _, _, _, allow_alt, _ = psv2.parse_plan_request_v2(payload)
        self.assertFalse(allow_alt)

    def test_explicit_false_accepted(self):
        _, _, _, _, allow_alt, _ = psv2.parse_plan_request_v2(
            _valid_payload(allow_alternative_viewing=False)
        )
        self.assertFalse(allow_alt)

    def test_explicit_true_accepted(self):
        _, _, _, _, allow_alt, _ = psv2.parse_plan_request_v2(
            _valid_payload(allow_alternative_viewing=True)
        )
        self.assertTrue(allow_alt)

    def test_explicit_null_is_rejected(self):
        payload = _valid_payload(allow_alternative_viewing=None)
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_string_is_rejected(self):
        payload = _valid_payload(allow_alternative_viewing="false")
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_number_is_rejected(self):
        payload = _valid_payload(allow_alternative_viewing=1)
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_list_is_rejected(self):
        payload = _valid_payload(allow_alternative_viewing=[])
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_object_is_rejected(self):
        payload = _valid_payload(allow_alternative_viewing={})
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_obstacle_count_must_be_one_to_eight(self):
        payload = _valid_payload(obstacles=[])
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)
        eight = _valid_payload(obstacles=[
            {"id": i, "x": 10 * i, "y": 50, "face": "N"} for i in range(1, 9)
        ])
        self.assertEqual(len(psv2.parse_plan_request_v2(eight)[3]), 8)
        payload = _valid_payload(obstacles=[
            {"id": i, "x": 10 * i, "y": 50, "face": "N"} for i in range(1, 10)
        ])
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_duplicate_obstacle_ids_rejected(self):
        payload = _valid_payload(obstacles=[
            {"id": 1, "x": 10, "y": 0, "face": "N"},
            {"id": 1, "x": 50, "y": 50, "face": "S"},
        ])
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_invalid_face_rejected(self):
        payload = _valid_payload(obstacles=[{"id": 1, "x": 10, "y": 0, "face": "Q"}])
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)

    def test_out_of_arena_obstacle_rejected(self):
        payload = _valid_payload(obstacles=[{"id": 1, "x": 9999, "y": 0, "face": "N"}])
        with self.assertRaises(psv2.RequestErrorV2):
            psv2.parse_plan_request_v2(payload)


class ResponseSerialization(unittest.TestCase):
    def test_mov_fields_copied_verbatim_from_stm_motion_command(self):
        p0, p1 = Pose(0.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0)
        command = _command("F", 5, "cm", p0, p1, samples=(p0, p1))
        plan = _plan([command], p1)
        result = _result([_leg(1, Pose(5.0, 0.0, 0.0), plan)], (1,), (Pose(5.0, 0.0, 0.0),), p1)
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))

        action = response["actions"][0]
        self.assertEqual(action["type"], "MOV")
        self.assertEqual(action["dir"], "F")
        self.assertEqual(action["value"], 5)
        self.assertIsInstance(action["value"], int)
        self.assertEqual(action["unit"], "cm")
        self.assertEqual(action["stm_command"], "MOV,F,5")

    def test_turn_radii_sourced_from_config_not_literal(self):
        custom_config = replace(
            DEFAULT_PLANNER_CONFIG,
            turn_radius_fl_cm=1.1, turn_radius_fr_cm=2.2,
            turn_radius_bl_cm=3.3, turn_radius_br_cm=4.4,
        )
        final_pose = Pose(10.0, 0.0, 0.0)
        plan = _plan([], final_pose)
        result = _result([_leg(1, final_pose, plan)], (1,), (final_pose,), final_pose)
        response = psv2.plan_request_v2(
            _valid_payload(), custom_config, b2_planner=_fake_planner(result)
        )
        self.assertEqual(response["turn_radii_cm"], {
            "FL": 1.1, "FR": 2.2, "BL": 3.3, "BR": 4.4,
        })

    def test_snap_uses_estimated_final_pose_not_ideal_target(self):
        ideal_target = Pose(10.0, 0.0, 0.3)
        estimated_final = Pose(9.8, 0.1, 0.29)
        plan = _plan([], estimated_final)
        result = _result([_leg(1, ideal_target, plan)], (1,), (ideal_target,), estimated_final)
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))

        snap = response["actions"][0]
        self.assertEqual(snap["type"], "SNAP")
        self.assertEqual(snap["obstacle_id"], 1)
        self.assertEqual(snap["pose"], psv2._pose_json(estimated_final))
        self.assertNotEqual(snap["pose"], psv2._pose_json(ideal_target))

    def test_snap_carries_no_stm_fields_or_waypoint_range(self):
        final_pose = Pose(10.0, 0.0, 0.0)
        plan = _plan([], final_pose)
        result = _result([_leg(1, final_pose, plan)], (1,), (final_pose,), final_pose)
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))

        snap = response["actions"][0]
        for forbidden in ("stm_command", "value", "unit", "dir",
                          "waypoint_start_index", "waypoint_end_index"):
            self.assertNotIn(forbidden, snap)

    def test_sequence_numbers_span_mov_and_snap(self):
        p0, p1, p2 = Pose(0.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0), Pose(10.0, 0.0, 0.0)
        c1 = _command("F", 5, "cm", p0, p1, samples=(p0, p1))
        c2 = _command("F", 5, "cm", p1, p2, samples=(p1, p2))
        plan = _plan([c1, c2], p2)
        result = _result([_leg(1, p2, plan)], (1,), (p2,), p2)
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))
        seqs = [action["seq"] for action in response["actions"]]
        self.assertEqual(seqs, [0, 1, 2])
        self.assertEqual([a["type"] for a in response["actions"]], ["MOV", "MOV", "SNAP"])

    def test_no_timing_fields_anywhere(self):
        p0, p1 = Pose(0.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0)
        command = _command("F", 5, "cm", p0, p1, samples=(p0, p1))
        plan = _plan([command], p1)
        result = _result([_leg(1, p1, plan)], (1,), (p1,), p1)
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))
        forbidden = {"estimated_time_s", "estimated_time_status"}
        self.assertEqual(set(response.keys()) & forbidden, set())
        for action in response["actions"]:
            self.assertEqual(set(action.keys()) & forbidden, set())

    def test_no_heading_rad_anywhere_in_response(self):
        p0, p1 = Pose(0.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0)
        command = _command("F", 5, "cm", p0, p1, samples=(p0, p1))
        plan = _plan([command], p1)
        result = _result([_leg(1, p1, plan)], (1,), (p1,), p1)
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))

        def _walk(value):
            if isinstance(value, dict):
                self.assertNotIn("heading_rad", value)
                for v in value.values():
                    _walk(v)
            elif isinstance(value, list):
                for v in value:
                    _walk(v)

        _walk(response)

    def test_complete_route_failure_reason_is_null(self):
        p0, p1 = Pose(0.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0)
        command = _command("F", 5, "cm", p0, p1, samples=(p0, p1))
        plan = _plan([command], p1)
        result = _result([_leg(1, p1, plan)], (1,), (p1,), p1)
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))
        self.assertIsNone(response["failure_reason"])

    def test_determinism(self):
        p0, p1 = Pose(0.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0)
        command = _command("F", 5, "cm", p0, p1, samples=(p0, p1))
        plan = _plan([command], p1)
        result = _result([_leg(1, p1, plan)], (1,), (p1,), p1)
        response_a = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))
        response_b = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))
        self.assertEqual(response_a, response_b)


class WaypointConstruction(unittest.TestCase):
    def test_first_mov_covers_all_samples_from_index_zero(self):
        p0, p1, p2 = Pose(0.0, 0.0, 0.0), Pose(2.5, 0.0, 0.0), Pose(5.0, 0.0, 0.0)
        command = _command("F", 5, "cm", p0, p2, samples=(p0, p1, p2))
        plan = _plan([command], p2)
        result = _result([_leg(1, p2, plan)], (1,), (p2,), p2)
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))

        self.assertEqual(len(response["waypoints"]), 3)
        action = response["actions"][0]
        self.assertEqual(action["waypoint_start_index"], 0)
        self.assertEqual(action["waypoint_end_index"], 2)

    def test_leg_boundary_shares_pose_without_duplication(self):
        p0, p1, p2 = Pose(0.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0), Pose(10.0, 0.0, 0.0)
        c1 = _command("F", 5, "cm", p0, p1, samples=(p0, p1))
        c2 = _command("F", 5, "cm", p1, p2, samples=(p1, p2))
        plan = _plan([c1, c2], p2)
        result = _result([_leg(1, p2, plan)], (1,), (p2,), p2)
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))

        # 2 samples + 2 samples sharing one boundary = 3 total, not 4.
        self.assertEqual(len(response["waypoints"]), 3)
        first, second = response["actions"][0], response["actions"][1]
        self.assertEqual(first["waypoint_start_index"], 0)
        self.assertEqual(first["waypoint_end_index"], 1)
        self.assertEqual(second["waypoint_start_index"], 1)
        self.assertEqual(second["waypoint_end_index"], 2)

    def test_zero_action_leg_alone_yields_single_waypoint_and_snap_only(self):
        final_pose = Pose(0.0, 0.0, 0.0)
        plan = _plan([], final_pose)
        result = _result([_leg(1, final_pose, plan)], (1,), (final_pose,), final_pose)
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))

        self.assertEqual(len(response["waypoints"]), 1)
        self.assertEqual(response["waypoints"][0], psv2._pose_json(final_pose))
        self.assertEqual(len(response["actions"]), 1)
        self.assertEqual(response["actions"][0]["type"], "SNAP")

    def test_zero_action_leg_followed_by_mov_leg(self):
        shared = Pose(0.0, 0.0, 0.0)
        next_pose = Pose(5.0, 0.0, 0.0)
        zero_plan = _plan([], shared)
        command = _command("F", 5, "cm", shared, next_pose, samples=(shared, next_pose))
        real_plan = _plan([command], next_pose)
        result = _result(
            [_leg(1, shared, zero_plan), _leg(2, next_pose, real_plan)],
            (1, 2), (shared, next_pose), next_pose,
        )
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))

        # zero-action leg's SNAP contributes exactly 1 waypoint; the next
        # leg's single command shares that boundary and appends 1 more.
        self.assertEqual(len(response["waypoints"]), 2)
        actions = response["actions"]
        self.assertEqual(actions[0]["type"], "SNAP")  # leg 1
        mov = actions[1]
        self.assertEqual(mov["type"], "MOV")
        self.assertEqual(mov["waypoint_start_index"], 0)
        self.assertEqual(mov["waypoint_end_index"], 1)

    def test_revisiting_a_previous_physical_pose_is_not_deduplicated_across_legs(self):
        p_start = Pose(0.0, 0.0, 0.0)
        p_mid = Pose(5.0, 0.0, 0.0)
        p_far = Pose(10.0, 0.0, 0.0)
        c1 = _command("F", 5, "cm", p_start, p_mid, samples=(p_start, p_mid))
        leg1_plan = _plan([c1], p_mid)

        c2a = _command("F", 5, "cm", p_mid, p_far, samples=(p_mid, p_far))
        c2b = _command("B", 5, "cm", p_far, p_mid, samples=(p_far, p_mid))
        leg2_plan = _plan([c2a, c2b], p_mid)  # leg2 returns to leg1's exact final pose

        result = _result(
            [_leg(1, p_mid, leg1_plan), _leg(2, p_mid, leg2_plan)],
            (1, 2), (p_mid, p_mid), p_mid,
        )
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))

        waypoints = response["waypoints"]
        # p_start, p_mid, p_far, p_mid again -- 4 entries, p_mid appears twice
        # at non-adjacent indices (1 and 3), each independently addressable.
        self.assertEqual(len(waypoints), 4)
        self.assertEqual(waypoints[1], psv2._pose_json(p_mid))
        self.assertEqual(waypoints[3], psv2._pose_json(p_mid))
        self.assertEqual(waypoints[1], waypoints[3])  # same pose, different indices
        actions = [a for a in response["actions"] if a["type"] == "MOV"]
        self.assertEqual(actions[0]["waypoint_start_index"], 0)
        self.assertEqual(actions[0]["waypoint_end_index"], 1)
        self.assertEqual(actions[1]["waypoint_start_index"], 1)
        self.assertEqual(actions[1]["waypoint_end_index"], 2)
        self.assertEqual(actions[2]["waypoint_start_index"], 2)
        self.assertEqual(actions[2]["waypoint_end_index"], 3)


class InternalInvariantErrorsPropagate(unittest.TestCase):
    def test_executable_route_invariant_error_is_not_caught(self):
        def raising_planner(start_pose, obstacles, config=DEFAULT_CONFIG, allow_alternative_viewing=False):
            raise ExecutableRouteInvariantError("simulated internal invariant failure")

        with self.assertRaises(ExecutableRouteInvariantError):
            psv2.plan_request_v2(_valid_payload(), b2_planner=raising_planner)


class NoRouteAndFailureShapes(unittest.TestCase):
    def test_no_route_result_shape(self):
        result = ExecutableRouteResult(
            obstacle_order=(), target_poses=(), legs=(),
            total_distance_cm=0.0, total_cost=0.0, is_valid=False, is_complete=False,
            orders_considered=1, valid_orders=0, rejected_orders=1, pruned_orders=0,
            skipped_obstacle_ids=(), final_pose=None,
            failure_reason="No complete executable route visits all obstacles.",
        )
        response = psv2.plan_request_v2(_valid_payload(), b2_planner=_fake_planner(result))
        self.assertEqual(response["type"], "PLAN_RESULT")
        self.assertFalse(response["valid"])
        self.assertEqual(response["error"], "No complete executable route visits all obstacles.")
        self.assertEqual(response["orders_considered"], 1)
        self.assertEqual(response["rejected_orders"], 1)

    def test_partial_route_shape(self):
        p0, p1 = Pose(0.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0)
        command = _command("F", 5, "cm", p0, p1, samples=(p0, p1))
        plan = _plan([command], p1, total_distance=5.0, total_cost=5.0)
        reason = "No complete executable route visits all 2 obstacles; returning the best 1/2-obstacle partial route."
        result = ExecutableRouteResult(
            obstacle_order=(1,), target_poses=(p1,),
            legs=(_leg(1, p1, plan),),
            total_distance_cm=5.0, total_cost=5.0, is_valid=True, is_complete=False,
            orders_considered=1, valid_orders=0, rejected_orders=1, pruned_orders=0,
            skipped_obstacle_ids=(2,), final_pose=p1,
            failure_reason=reason,
        )
        payload = _valid_payload(obstacles=[
            {"id": 1, "x": 10, "y": 0, "face": "N"},
            {"id": 2, "x": 50, "y": 50, "face": "S"},
        ])
        response = psv2.plan_request_v2(payload, b2_planner=_fake_planner(result))

        self.assertTrue(response["valid"])
        self.assertFalse(response["complete"])
        # The exact partial-route reason must be preserved verbatim, never
        # forced to None just because the overall route is otherwise valid.
        self.assertEqual(response["failure_reason"], reason)
        self.assertEqual(response["skipped_obstacle_ids"], [2])
        self.assertEqual(response["obstacle_order"], [1])
        self.assertEqual(response["target_poses"], [psv2._pose_json(p1)])
        self.assertEqual(response["final_pose"], psv2._pose_json(p1))
        mov_actions = [a for a in response["actions"] if a["type"] == "MOV"]
        snap_actions = [a for a in response["actions"] if a["type"] == "SNAP"]
        self.assertEqual(len(mov_actions), 1)
        self.assertEqual(mov_actions[0]["dir"], "F")
        self.assertEqual(len(snap_actions), 1)
        self.assertEqual(snap_actions[0]["obstacle_id"], 1)
        self.assertEqual(snap_actions[0]["pose"], psv2._pose_json(p1))


EXPECTED_STM_SETUP = {
    "PRECISION": {
        "set_command": "PROFILE,PRECISION",
        "expected_set_reply": "PROFILE,OK",
        "verify_command": "PROFILE,GET",
        "expected_verify_reply": "PROFILE,PRECISION",
    },
    "STANDARD": {
        "set_command": "PROFILE,STANDARD",
        "expected_set_reply": "PROFILE,OK",
        "verify_command": "PROFILE,GET",
        "expected_verify_reply": "PROFILE,STANDARD",
    },
}
PROFILE_KEYS = ("motion_profile", "stm_setup")
# (request value, resolved profile): None = key omitted from the request.
PROFILE_REQUESTS = ((None, "PRECISION"), ("PRECISION", "PRECISION"), ("STANDARD", "STANDARD"))


def _profile_payload(requested, **overrides):
    payload = _valid_payload(**overrides)
    if requested is not None:
        payload["motion_profile"] = requested
    return payload


def _without_profile_fields(response):
    return {k: v for k, v in response.items() if k not in PROFILE_KEYS}


def _profile_fixtures():
    """(result, payload overrides, expected valid, expected complete) for
    the three PLAN_RESULT kinds: complete, partial and no-route."""
    p0, p1 = Pose(0.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0)
    command = _command("F", 5, "cm", p0, p1, samples=(p0, p1))
    plan = _plan([command], p1, total_distance=5.0, total_cost=5.0)
    complete = _result([_leg(1, p1, plan)], (1,), (p1,), p1)
    partial = ExecutableRouteResult(
        obstacle_order=(1,), target_poses=(p1,), legs=(_leg(1, p1, plan),),
        total_distance_cm=5.0, total_cost=5.0, is_valid=True, is_complete=False,
        orders_considered=1, valid_orders=0, rejected_orders=1, pruned_orders=0,
        skipped_obstacle_ids=(2,), final_pose=p1,
        failure_reason="No complete executable route visits all 2 obstacles; "
                       "returning the best 1/2-obstacle partial route.",
    )
    no_route = ExecutableRouteResult(
        obstacle_order=(), target_poses=(), legs=(),
        total_distance_cm=0.0, total_cost=0.0, is_valid=False, is_complete=False,
        orders_considered=1, valid_orders=0, rejected_orders=1, pruned_orders=0,
        skipped_obstacle_ids=(), final_pose=None,
        failure_reason="No complete executable route visits all obstacles.",
    )
    two_obstacles = dict(obstacles=[
        {"id": 1, "x": 10, "y": 0, "face": "N"},
        {"id": 2, "x": 50, "y": 50, "face": "S"},
    ])
    return {
        "complete": (complete, {}, True, True),
        "partial": (partial, two_obstacles, True, False),
        "no_route": (no_route, {}, False, False),
    }


class MotionProfileRequest(unittest.TestCase):
    """`motion_profile` is optional: absent resolves to PRECISION (the team's
    default, also the STM's boot profile); present, it must be exactly
    "PRECISION" or "STANDARD"."""

    INVALID_VALUES = (
        "precision", "standard", "Precision", "Standard", "PRECISION ", " STANDARD",
        "STANDARD\n", "FAST", "CUSTOM", "custom", "",
        None, 0, 1, 1.0, True, False, [], ["PRECISION"], {}, {"profile": "PRECISION"},
    )

    def test_supported_values_are_exactly_the_two_stm_profiles(self):
        self.assertEqual(psv2.MOTION_PROFILES, ("PRECISION", "STANDARD"))
        self.assertEqual(psv2.DEFAULT_MOTION_PROFILE, "PRECISION")

    def test_omitted_field_resolves_to_precision(self):
        payload = _valid_payload()
        self.assertNotIn("motion_profile", payload)
        *_, motion_profile = psv2.parse_plan_request_v2(payload)
        self.assertEqual(motion_profile, "PRECISION")

    def test_explicit_precision_accepted(self):
        *_, motion_profile = psv2.parse_plan_request_v2(_valid_payload(motion_profile="PRECISION"))
        self.assertEqual(motion_profile, "PRECISION")

    def test_explicit_standard_accepted(self):
        *_, motion_profile = psv2.parse_plan_request_v2(_valid_payload(motion_profile="STANDARD"))
        self.assertEqual(motion_profile, "STANDARD")

    def test_invalid_values_rejected_by_the_parser(self):
        for value in self.INVALID_VALUES:
            with self.subTest(value=value):
                with self.assertRaises(psv2.RequestErrorV2):
                    psv2.parse_plan_request_v2(_valid_payload(motion_profile=value))

    def test_invalid_values_are_plan_error_without_profile_fields_and_never_plan(self):
        def must_not_run(*args, **kwargs):
            raise AssertionError("planner must not run for a rejected request")

        for value in self.INVALID_VALUES:
            with self.subTest(value=value):
                response = psv2.plan_request_v2(
                    _valid_payload(motion_profile=value),
                    b2_planner=must_not_run, b3_planner=must_not_run,
                )
                self.assertEqual(response["type"], "PLAN_ERROR")
                self.assertEqual(response["protocol_version"], 2)
                self.assertFalse(response["valid"])
                self.assertIn("motion_profile", response["error"])
                self.assertEqual(set(response) & set(PROFILE_KEYS), set())


class MotionProfileResponse(unittest.TestCase):
    """Every PLAN_RESULT echoes the resolved profile and carries the exact
    STM setup strings, outside `actions`; PLAN_ERROR carries neither."""

    def test_setup_and_echo_in_every_plan_result_kind(self):
        for kind, (result, overrides, valid, complete) in _profile_fixtures().items():
            for requested, resolved in PROFILE_REQUESTS:
                with self.subTest(kind=kind, requested=requested):
                    response = psv2.plan_request_v2(
                        _profile_payload(requested, **overrides), b2_planner=_fake_planner(result)
                    )
                    self.assertEqual(response["type"], "PLAN_RESULT")
                    self.assertEqual(response["valid"], valid)
                    if valid:
                        self.assertEqual(response["complete"], complete)
                    self.assertEqual(response["motion_profile"], resolved)
                    self.assertEqual(response["stm_setup"], EXPECTED_STM_SETUP[resolved])

    def test_setup_and_echo_also_in_b3_results(self):
        result, overrides, _, _ = _profile_fixtures()["complete"]
        for requested, resolved in PROFILE_REQUESTS:
            with self.subTest(requested=requested):
                response = psv2.plan_request_v2(
                    _profile_payload(requested, mode="B3", **overrides),
                    b3_planner=_fake_planner(result),
                )
                self.assertEqual(response["mode"], "B3")
                self.assertEqual(response["motion_profile"], resolved)
                self.assertEqual(response["stm_setup"], EXPECTED_STM_SETUP[resolved])

    def test_stm_setup_has_exactly_four_exact_plain_strings(self):
        for resolved in ("PRECISION", "STANDARD"):
            with self.subTest(profile=resolved):
                setup = psv2._stm_setup_v2(resolved)
                self.assertEqual(
                    list(setup),
                    ["set_command", "expected_set_reply", "verify_command", "expected_verify_reply"],
                )
                self.assertEqual(setup, EXPECTED_STM_SETUP[resolved])
                for text in setup.values():
                    self.assertIsInstance(text, str)
                    self.assertEqual(text, text.strip())
                    self.assertNotIn("\n", text)
                    self.assertNotIn("\r", text)

    def test_stm_setup_is_not_an_action_and_carries_no_path_fields(self):
        result, _, _, _ = _profile_fixtures()["complete"]
        response = psv2.plan_request_v2(
            _profile_payload("STANDARD"), b2_planner=_fake_planner(result)
        )
        path_fields = {
            "seq", "type", "pose", "start_pose", "end_pose",
            "waypoint_start_index", "waypoint_end_index", "waypoints",
        }
        self.assertEqual(set(response["stm_setup"]) & path_fields, set())
        self.assertNotIn("stm_setup", response["actions"])
        self.assertEqual(
            [a["seq"] for a in response["actions"]], list(range(len(response["actions"])))
        )
        for action in response["actions"]:
            self.assertIn(action["type"], ("MOV", "SNAP"))
            self.assertEqual(set(action) & set(PROFILE_KEYS), set())
            self.assertFalse(str(action.get("stm_command", "")).startswith("PROFILE"))

    def test_profile_changes_nothing_else_in_the_response(self):
        for kind, (result, overrides, _, _) in _profile_fixtures().items():
            with self.subTest(kind=kind):
                responses = {
                    requested: psv2.plan_request_v2(
                        _profile_payload(requested, **overrides), b2_planner=_fake_planner(result)
                    )
                    for requested, _ in PROFILE_REQUESTS
                }
                baseline = _without_profile_fields(responses[None])
                for requested in ("PRECISION", "STANDARD"):
                    other = responses[requested]
                    self.assertEqual(_without_profile_fields(other), baseline)
                    self.assertEqual(other.get("actions"), responses[None].get("actions"))
                    self.assertEqual(other.get("waypoints"), responses[None].get("waypoints"))
                    self.assertEqual(other.get("obstacle_order"), responses[None].get("obstacle_order"))

    def test_determinism_for_each_profile(self):
        for kind, (result, overrides, _, _) in _profile_fixtures().items():
            for requested, _ in PROFILE_REQUESTS:
                with self.subTest(kind=kind, requested=requested):
                    first = psv2.plan_request_v2(
                        _profile_payload(requested, **overrides), b2_planner=_fake_planner(result)
                    )
                    second = psv2.plan_request_v2(
                        _profile_payload(requested, **overrides), b2_planner=_fake_planner(result)
                    )
                    self.assertEqual(
                        json.dumps(first, sort_keys=True), json.dumps(second, sort_keys=True)
                    )

    def test_profile_is_never_passed_to_the_planner(self):
        result, overrides, _, _ = _profile_fixtures()["complete"]
        for mode, keyword in (("B2", "b2_planner"), ("B3", "b3_planner")):
            calls = []

            # Accepts only the pre-existing arguments: any extra keyword
            # (a leaked motion_profile) raises TypeError.
            def recording(start_pose, obstacles, config=DEFAULT_CONFIG, allow_alternative_viewing=False):
                calls.append((start_pose, tuple(obstacles), config, allow_alternative_viewing))
                return result

            for requested, _ in PROFILE_REQUESTS:
                planners = {keyword: recording}
                psv2.plan_request_v2(_profile_payload(requested, mode=mode, **overrides), **planners)
            with self.subTest(mode=mode):
                self.assertEqual(len(calls), 3)
                self.assertEqual(calls[0], calls[1])
                self.assertEqual(calls[0], calls[2])

class MotionProfileRealPipeline(unittest.TestCase):
    """Real 1-obstacle pipeline: the profile is metadata only, so planner
    geometry, MOV values, estimated poses and waypoints are identical."""

    PAYLOAD = {
        "type": "PLAN_REQUEST", "protocol_version": 2,
        "mission_id": "real-profile",
        "mode": "B2", "coordinate_units": "cm",
        "allow_alternative_viewing": False,
        "start": {"x": 100, "y": 100, "heading_deg": 0},
        "obstacles": [{"id": 1, "x": 150, "y": 100, "face": "W"}],
    }

    def _plan(self, requested):
        payload = dict(self.PAYLOAD)
        if requested is not None:
            payload["motion_profile"] = requested
        return psv2.plan_request_v2(payload)

    def test_responses_differ_only_in_the_two_profile_fields(self):
        responses = {requested: self._plan(requested) for requested, _ in PROFILE_REQUESTS}
        for requested, resolved in PROFILE_REQUESTS:
            with self.subTest(requested=requested):
                response = responses[requested]
                self.assertTrue(response["valid"])
                self.assertTrue(response["complete"])
                self.assertEqual(response["motion_profile"], resolved)
                self.assertEqual(response["stm_setup"], EXPECTED_STM_SETUP[resolved])
        baseline = _without_profile_fields(responses[None])
        self.assertEqual(_without_profile_fields(responses["PRECISION"]), baseline)
        self.assertEqual(_without_profile_fields(responses["STANDARD"]), baseline)
        movs = [a for a in responses["STANDARD"]["actions"] if a["type"] == "MOV"]
        self.assertGreater(len(movs), 0)
        self.assertEqual(movs, [a for a in responses["PRECISION"]["actions"] if a["type"] == "MOV"])
        self.assertEqual(
            responses["STANDARD"]["turn_radii_cm"], responses["PRECISION"]["turn_radii_cm"]
        )

    def test_real_pipeline_is_deterministic_for_each_profile(self):
        for requested in ("PRECISION", "STANDARD"):
            with self.subTest(requested=requested):
                self.assertEqual(
                    json.dumps(self._plan(requested), sort_keys=True),
                    json.dumps(self._plan(requested), sort_keys=True),
                )


class RealPipelineIntegration(unittest.TestCase):
    """Small, fast cases using the actual executable pipeline (kept to
    1-2 obstacles, alternatives disabled -- see the module docstring's
    experimental-scope note)."""

    def test_b2_real_pipeline_two_obstacles(self):
        payload = {
            "type": "PLAN_REQUEST", "protocol_version": 2,
            "mission_id": "real-b2",
            "mode": "B2", "coordinate_units": "cm",
            "allow_alternative_viewing": False,
            "start": {"x": 100, "y": 100, "heading_deg": 0},
            "obstacles": [
                {"id": 1, "x": 150, "y": 100, "face": "W"},
                {"id": 2, "x": 100, "y": 150, "face": "S"},
            ],
        }
        response = psv2.plan_request_v2(payload)
        self.assertTrue(response["valid"])
        self.assertTrue(response["complete"])
        self.assertEqual(response["mission_id"], "real-b2")
        self.assertGreater(len(response["actions"]), 0)
        mov_actions = [a for a in response["actions"] if a["type"] == "MOV"]
        self.assertTrue(all(a["stm_command"].startswith("MOV,") for a in mov_actions))
        self.assertTrue(all(isinstance(a["value"], int) for a in mov_actions))

    def test_b3_real_pipeline_single_obstacle(self):
        payload = {
            "type": "PLAN_REQUEST", "protocol_version": 2,
            "mission_id": "real-b3",
            "mode": "B3", "coordinate_units": "cm",
            "allow_alternative_viewing": False,
            "start": {"x": 100, "y": 100, "heading_deg": 0},
            "obstacles": [{"id": 1, "x": 150, "y": 100, "face": "W"}],
        }
        response = psv2.plan_request_v2(payload)
        self.assertTrue(response["valid"])
        self.assertTrue(response["complete"])
        self.assertEqual(response["obstacle_order"], [1])


def _bounded_route():
    start = Pose(0.0, 0.0, 0.0)
    final = Pose(5.0, 0.0, 0.0)
    command = _command("F", 5, "cm", start, final, samples=(start, final))
    plan = _plan([command], final, total_distance=5.0, total_cost=5.0)
    leg = _leg(1, final, plan)
    return PrefixRoute(
        obstacle_order=(1,), candidate_indices=(0,), target_poses=(final,),
        legs=(leg,), total_cost=5.0, total_distance_cm=5.0,
        final_pose=final, skipped_obstacle_ids=(),
    )


def _bounded_outcome(route=None, *, diagnostic_route=None, message=None):
    deliverable = route is not None
    return bp.BoundedPlanOutcome(
        mode="B2", termination=bp.TERM_SEARCH_FINISHED,
        route_status="complete" if deliverable else "none",
        route=route, route_source=bp.SOURCE_FINAL if deliverable else None,
        b2_route=None, diagnostic_route=diagnostic_route,
        optimality_proven=False, complete_route_proven_impossible=False,
        search_space_exhausted=False,
        planner_metadata=None,
        execution_message=message or (
            "Best complete verified route found. Optimality not proven."
            if deliverable else "No complete route found; non-existence is not proven."
        ),
        failure_detail=None, incumbents_received=1 if (route or diagnostic_route) else 0,
        verification_failures=(), deliverable=deliverable,
        delivery=(bp.DELIVERY_ON_TIME if diagnostic_route is None
                  else bp.DELIVERY_CEILING_EXCEEDED),
        cleanup=bp.Cleanup(False, None, None, None, None, None, None),
        timing=bp.Timing(returned_s=1.0),
    )


class _FakeBoundedPlanner:
    def __init__(self, outcome):
        self.outcome = outcome
        self.calls = []

    def plan(self, inp, *, accepted_at, cancel=None):
        self.calls.append((inp, accepted_at, cancel))
        return self.outcome


class BoundedEndpointIntegration(unittest.TestCase):
    def test_complete_verified_route_keeps_the_existing_stm_action_schema(self):
        fake = _FakeBoundedPlanner(_bounded_outcome(_bounded_route()))
        response = psv2.plan_request_v2(
            _valid_payload(), bounded_planner=fake, accepted_at=123.5,
        )

        self.assertTrue(response["valid"])
        self.assertTrue(response["complete"])
        self.assertEqual(response["obstacle_order"], [1])
        self.assertEqual(response["actions"][0]["stm_command"], "MOV,F,5")
        self.assertEqual(response["actions"][-1]["type"], "SNAP")
        self.assertNotIn("stm_command", response["actions"][-1])
        self.assertEqual(
            [a["type"] for a in response["actions"] if "stm_command" in a], ["MOV"],
        )
        self.assertEqual(response["termination"], bp.TERM_SEARCH_FINISHED)
        self.assertEqual(fake.calls[0][1], 123.5)
        self.assertEqual(fake.calls[0][0].mode, "B2")

    def test_no_route_is_a_clear_non_executable_plan_result(self):
        message = "No complete route found within budget; non-existence is not proven."
        fake = _FakeBoundedPlanner(_bounded_outcome(message=message))
        response = psv2.plan_request_v2(_valid_payload(), bounded_planner=fake)

        self.assertEqual(response["type"], "PLAN_RESULT")
        self.assertFalse(response["valid"])
        self.assertFalse(response["complete"])
        self.assertEqual(response["actions"], [])
        self.assertEqual(response["waypoints"], [])
        self.assertEqual(response["error"], message)
        self.assertEqual(response["failure_reason"], message)

    def test_late_verified_route_is_not_exposed_for_execution(self):
        route = _bounded_route()
        message = "The verified route was ready after the response deadline and is not delivered."
        fake = _FakeBoundedPlanner(
            _bounded_outcome(diagnostic_route=route, message=message)
        )
        response = psv2.plan_request_v2(_valid_payload(), bounded_planner=fake)

        self.assertFalse(response["valid"])
        self.assertEqual(response["delivery"], bp.DELIVERY_CEILING_EXCEEDED)
        self.assertEqual(response["actions"], [])
        self.assertEqual(response["target_poses"], [])
        self.assertNotIn("diagnostic_route", response)


if __name__ == "__main__":
    unittest.main()
