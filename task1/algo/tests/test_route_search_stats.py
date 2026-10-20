"""Profiling instrumentation of the executable route optimizer.

`RouteSearchStats` is observational only: passing it must not change any
returned value, exception, ordering, cost or counter, and its own counters
must be internally consistent. Most tests inject a deterministic,
start-dependent fake leg planner (fast, and it proves nothing substitutes an
ideal target for an estimated final pose); the real pipeline is used only for
one small obstacle.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import dataclasses
import inspect
import json
import math
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import executable_route_optimizer as ero  # noqa: E402
import planning_service_v2 as psv2  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
from models import Obstacle, Pose  # noqa: E402
from state_lattice import _heuristic_cm  # noqa: E402
from stm_command_generation import STMCommandPlan  # noqa: E402

START = Pose(20.0, 10.0, math.pi / 2)
OBSTACLES = (
    Obstacle(1, 60.0, 100.0, "N"),
    Obstacle(2, 130.0, 50.0, "W"),
    Obstacle(3, 150.0, 140.0, "S"),
)
EXPANSIONS_PER_LEG = 7
TIME_FIELDS = ("wall_s", "elapsed_s", "leg_planner_s", "lattice_s", "postprocess_s")


class StartDependentFake:
    """Deterministic fake: the estimated final pose depends on the start
    pose (never equal to the ideal target) and the cost respects the real
    admissible lower bound. `blocked(start, target)` marks unreachable legs."""

    def __init__(self, blocked=None):
        self.calls = []
        self.blocked = blocked or (lambda start, target: False)

    def __call__(self, start, target, obstacles, config=DEFAULT_PLANNER_CONFIG):
        self.calls.append((start, target))
        if self.blocked(start, target):
            return STMCommandPlan(False, (), (), 0.0, 0.0, EXPANSIONS_PER_LEG, None, "no_path")
        cost = _heuristic_cm(start, target, config) + 5.0
        final = Pose(target.x + 0.3 * math.sin(start.x), target.y - 0.2, target.heading + 0.01)
        return STMCommandPlan(True, (), (), cost, cost, EXPANSIONS_PER_LEG, final, None)


def scrub_times(value):
    """A copy of a stats dict without any wall-clock field."""
    if isinstance(value, dict):
        return {k: scrub_times(v) for k, v in value.items() if k not in TIME_FIELDS}
    if isinstance(value, list):
        return [scrub_times(v) for v in value]
    return value


class SearchFunctions(unittest.TestCase):
    """Run every test for both algorithms and both alternative settings."""

    CASES = (
        ("b2", ero.find_executable_b2_route, False),
        ("b2", ero.find_executable_b2_route, True),
        ("b3", ero.optimise_executable_b3_route, False),
        ("b3", ero.optimise_executable_b3_route, True),
    )

    def test_stats_do_not_change_the_returned_result(self):
        for name, function, alternatives in self.CASES:
            with self.subTest(algorithm=name, alternatives=alternatives):
                plain = function(START, OBSTACLES, leg_planner=StartDependentFake(),
                                 allow_alternative_viewing=alternatives)
                measured = function(START, OBSTACLES, leg_planner=StartDependentFake(),
                                    allow_alternative_viewing=alternatives, stats=ero.RouteSearchStats())
                self.assertEqual(plain, measured)
                self.assertTrue(plain.is_complete)

    def test_stats_do_not_change_the_leg_planner_call_sequence(self):
        for name, function, alternatives in self.CASES:
            with self.subTest(algorithm=name, alternatives=alternatives):
                plain_fake, measured_fake = StartDependentFake(), StartDependentFake()
                function(START, OBSTACLES, leg_planner=plain_fake, allow_alternative_viewing=alternatives)
                function(START, OBSTACLES, leg_planner=measured_fake, allow_alternative_viewing=alternatives,
                         stats=ero.RouteSearchStats())
                self.assertEqual(plain_fake.calls, measured_fake.calls)

    def test_unreachable_legs_and_partial_routes_are_unchanged(self):
        blocked = lambda start, target: target.x > 120   # noqa: E731  (obstacle 3 side unreachable)
        for name, function, alternatives in self.CASES:
            with self.subTest(algorithm=name, alternatives=alternatives):
                plain = function(START, OBSTACLES, leg_planner=StartDependentFake(blocked),
                                 allow_alternative_viewing=alternatives)
                stats = ero.RouteSearchStats()
                measured = function(START, OBSTACLES, leg_planner=StartDependentFake(blocked),
                                    allow_alternative_viewing=alternatives, stats=stats)
                self.assertEqual(plain, measured)
                # invalid_leg_results counts every candidate evaluation that saw
                # an invalid leg (cache hits included); outcomes counts only
                # legs actually planned, so it can be smaller.
                self.assertGreater(stats.invalid_leg_results, 0)
                self.assertGreater(stats.outcomes["no_path"][0], 0)
                self.assertLessEqual(stats.outcomes["no_path"][0], stats.invalid_leg_results)

    def test_exceptions_are_unchanged_and_stats_are_finished(self):
        def bad_leg(start, target, obstacles, config=DEFAULT_PLANNER_CONFIG):
            return STMCommandPlan(True, (), (), -1.0, -1.0, 0, target, None)   # negative cost: invalid

        for name, function, alternatives in self.CASES:
            with self.subTest(algorithm=name, alternatives=alternatives):
                with self.assertRaises(ero.ExecutableRouteInvariantError):
                    function(START, OBSTACLES, leg_planner=bad_leg, allow_alternative_viewing=alternatives)
                stats = ero.RouteSearchStats()
                with self.assertRaises(ero.ExecutableRouteInvariantError):
                    function(START, OBSTACLES, leg_planner=bad_leg, allow_alternative_viewing=alternatives,
                             stats=stats)
                self.assertTrue(stats.finished)
                duplicates = (Obstacle(1, 60.0, 100.0, "N"), Obstacle(1, 130.0, 50.0, "W"))
                stats = ero.RouteSearchStats()
                with self.assertRaises(ero.InvalidExecutableRouteInputError):
                    function(START, duplicates, leg_planner=StartDependentFake(), stats=stats)
                self.assertTrue(stats.finished)

    def test_empty_obstacles_are_unchanged(self):
        for name, function, _ in self.CASES:
            with self.subTest(algorithm=name):
                stats = ero.RouteSearchStats()
                self.assertEqual(function(START, (), leg_planner=StartDependentFake()),
                                 function(START, (), leg_planner=StartDependentFake(), stats=stats))
                self.assertEqual(stats.leg_requests, 0)

    def test_leg_request_accounting_is_consistent(self):
        for name, function, alternatives in self.CASES:
            with self.subTest(algorithm=name, alternatives=alternatives):
                fake, stats = StartDependentFake(), ero.RouteSearchStats()
                result = function(START, OBSTACLES, leg_planner=fake,
                                  allow_alternative_viewing=alternatives, stats=stats)
                self.assertEqual(stats.leg_requests, stats.cache_hits + stats.cache_misses)
                # Every miss is exactly one leg-planner call, so unique legs
                # equal the calls the fake independently recorded.
                self.assertEqual(stats.unique_legs, stats.cache_misses)
                self.assertEqual(stats.cache_misses, len(fake.calls))
                self.assertEqual(len(fake.calls), len(set(fake.calls)))
                self.assertEqual(sum(row[0] for row in stats.legs_by_depth.values()), stats.leg_requests)
                self.assertEqual(sum(row[1] for row in stats.legs_by_depth.values()), stats.cache_hits)
                self.assertEqual(sum(row[0] for row in stats.outcomes.values()), stats.cache_misses)
                self.assertEqual(len(stats.leg_records), stats.cache_misses)
                self.assertEqual(stats.lattice_expansions, EXPANSIONS_PER_LEG * stats.cache_misses)
                self.assertEqual(stats.orders_searched, result.orders_considered)
                self.assertLessEqual(max(stats.legs_by_depth), len(OBSTACLES) - 1)
                self.assertGreater(stats.candidates_attempted, 0)

    def test_b3_reuses_shared_prefix_legs(self):
        stats = ero.RouteSearchStats()
        ero.optimise_executable_b3_route(START, OBSTACLES, leg_planner=StartDependentFake(),
                                         allow_alternative_viewing=False, stats=stats)
        self.assertGreater(stats.cache_hits, 0)
        self.assertGreaterEqual(stats.incumbent_updates, 1)
        self.assertEqual(stats.algorithm, "b3")

    def test_b2_stops_at_the_first_complete_order(self):
        stats = ero.RouteSearchStats()
        result = ero.find_executable_b2_route(START, OBSTACLES, leg_planner=StartDependentFake(),
                                              allow_alternative_viewing=False, stats=stats)
        self.assertEqual(stats.orders_searched, 1)
        self.assertEqual(stats.orders_completed, 1)
        self.assertEqual(result.obstacle_order, (1, 2, 3))
        self.assertEqual(stats.incumbent_updates, 0)
        self.assertEqual(stats.algorithm, "b2")

    def test_counters_are_deterministic(self):
        for name, function, alternatives in self.CASES:
            with self.subTest(algorithm=name, alternatives=alternatives):
                runs = []
                for _ in range(2):
                    stats = ero.RouteSearchStats()
                    function(START, OBSTACLES, leg_planner=StartDependentFake(),
                             allow_alternative_viewing=alternatives, stats=stats)
                    runs.append(scrub_times(stats.to_dict(include_legs=True)))
                self.assertEqual(runs[0], runs[1])

    def test_injected_planner_has_no_pipeline_split(self):
        stats = ero.RouteSearchStats()
        ero.find_executable_b2_route(START, OBSTACLES, leg_planner=StartDependentFake(), stats=stats)
        self.assertEqual(stats.separable_legs, 0)
        self.assertEqual((stats.lattice_s, stats.postprocess_s), (0.0, 0.0))
        self.assertGreater(stats.leg_planner_s, 0.0)
        self.assertTrue(all(r.lattice_s is None and r.postprocess_s is None for r in stats.leg_records))

    def test_to_dict_is_json_serialisable_and_complete(self):
        stats = ero.RouteSearchStats()
        ero.optimise_executable_b3_route(START, OBSTACLES, leg_planner=StartDependentFake(), stats=stats)
        summary = stats.to_dict()
        self.assertNotIn("leg_records", summary)
        self.assertIn("leg_records", stats.to_dict(include_legs=True))
        text = json.dumps(stats.to_dict(include_legs=True))
        self.assertEqual(json.loads(text)["leg_requests"], stats.leg_requests)
        self.assertEqual(summary["cache_hit_rate"], stats.cache_hits / stats.leg_requests)
        self.assertTrue(summary["finished"])

    def test_record_legs_can_be_disabled_without_changing_counters(self):
        full, light = ero.RouteSearchStats(), ero.RouteSearchStats(record_legs=False)
        for stats in (full, light):
            ero.optimise_executable_b3_route(START, OBSTACLES, leg_planner=StartDependentFake(), stats=stats)
        self.assertEqual(light.leg_records, [])
        self.assertEqual(scrub_times(full.to_dict()), scrub_times(light.to_dict()))


class RealPipeline(unittest.TestCase):
    """One small obstacle through the actual lattice -> compression ->
    command-generation pipeline (fast case, alternatives off)."""

    START = Pose(100.0, 100.0, 0.0)
    OBSTACLE = (Obstacle(1, 150.0, 100.0, "W"),)

    def test_results_identical_with_and_without_stats(self):
        for function in (ero.find_executable_b2_route, ero.optimise_executable_b3_route):
            with self.subTest(function=function.__name__):
                plain = function(self.START, self.OBSTACLE, allow_alternative_viewing=False)
                stats = ero.RouteSearchStats()
                measured = function(self.START, self.OBSTACLE, allow_alternative_viewing=False, stats=stats)
                self.assertEqual(plain, measured)
                self.assertTrue(plain.is_complete)
                self.assertEqual(stats.cache_misses, 1)
                self.assertEqual(stats.separable_legs, 1)
                self.assertGreater(stats.lattice_s, 0.0)
                self.assertGreaterEqual(stats.postprocess_s, 0.0)
                self.assertAlmostEqual(stats.lattice_s + stats.postprocess_s, stats.leg_planner_s, places=6)
                self.assertEqual(stats.lattice_expansions, plain.legs[0].command_plan.expansions)
                self.assertEqual(list(stats.outcomes), ["success"])

    def test_the_same_two_pipeline_functions_are_called_once_per_leg(self):
        with mock.patch.object(ero, "plan_lattice_path", wraps=ero.plan_lattice_path) as lattice, \
                mock.patch.object(ero, "generate_stm_command_plan", wraps=ero.generate_stm_command_plan) as generate:
            stats = ero.RouteSearchStats()
            ero.find_executable_b2_route(self.START, self.OBSTACLE, allow_alternative_viewing=False, stats=stats)
        self.assertEqual(lattice.call_count, stats.cache_misses)
        self.assertEqual(generate.call_count, stats.cache_misses)


class NoPublicSurfaceChange(unittest.TestCase):
    """Profiling instrumentation must not leak into public planner surfaces."""

    EXPECTED_RESULT_FIELDS = (
        "obstacle_order", "target_poses", "legs", "total_distance_cm", "total_cost", "is_valid",
        "is_complete", "orders_considered", "valid_orders", "rejected_orders", "pruned_orders",
        "skipped_obstacle_ids", "final_pose", "failure_reason",
    )
    EXPECTED_RESPONSE_KEYS = {
        "type", "protocol_version", "mission_id", "motion_model", "turn_radii_cm", "mode",
        "motion_profile", "stm_setup", "valid", "complete", "final_pose", "obstacle_order",
        "skipped_obstacle_ids", "target_poses", "actions", "waypoints", "total_distance_cm",
        "total_cost", "failure_reason", "termination", "delivery", "route_source",
        "incumbents_received", "optimality_proven", "complete_route_proven_impossible",
        "search_space_exhausted", "message",
    }

    def payload(self, **extra):
        payload = {
            "type": "PLAN_REQUEST", "protocol_version": 2, "mission_id": "profiling-unchanged",
            "mode": "B2", "coordinate_units": "cm", "allow_alternative_viewing": False,
            "start": {"x": 100, "y": 100, "heading_deg": 0},
            "obstacles": [{"id": 1, "x": 150, "y": 100, "face": "W"}],
        }
        payload.update(extra)
        return payload

    def test_result_dataclass_fields_are_unchanged(self):
        self.assertEqual(tuple(f.name for f in dataclasses.fields(ero.ExecutableRouteResult)),
                         self.EXPECTED_RESULT_FIELDS)

    def test_stats_parameter_defaults_to_disabled(self):
        for function in (ero.find_executable_b2_route, ero.optimise_executable_b3_route):
            self.assertIsNone(inspect.signature(function).parameters["stats"].default)

    def test_service_response_has_exactly_the_documented_keys(self):
        response = psv2.plan_request_v2(self.payload())
        self.assertEqual(response["type"], "PLAN_RESULT")
        self.assertEqual(set(response), self.EXPECTED_RESPONSE_KEYS)

    def test_service_signature_and_obstacle_maximum_match_bounded_v2(self):
        self.assertEqual(list(inspect.signature(psv2.plan_request_v2).parameters),
                         ["payload", "config", "b2_planner", "b3_planner",
                          "bounded_planner", "accepted_at"])
        self.assertEqual(psv2.MAX_OBSTACLES, 8)

    def test_service_never_passes_stats_to_the_planner(self):
        def strict_planner(start_pose, obstacles, config=DEFAULT_PLANNER_CONFIG, allow_alternative_viewing=False):
            return ero.find_executable_b2_route(start_pose, obstacles, config,
                                                allow_alternative_viewing=allow_alternative_viewing)

        response = psv2.plan_request_v2(self.payload(), b2_planner=strict_planner)
        self.assertTrue(response["valid"])

    def test_requests_have_no_budget_deadline_or_stats_fields(self):
        baseline = psv2.parse_plan_request_v2(self.payload())
        extras = {"time_budget_s": 1, "budget": 5, "deadline_s": 1, "timeout_s": 1, "stats": True,
                  "planning_budget_ms": 100, "max_planning_time_s": 1}
        self.assertEqual(psv2.parse_plan_request_v2(self.payload(**extras)), baseline)
        self.assertEqual(psv2.plan_request_v2(self.payload(**extras)), psv2.plan_request_v2(self.payload()))


if __name__ == "__main__":
    unittest.main()
