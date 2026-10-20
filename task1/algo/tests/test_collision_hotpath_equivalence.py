"""Differential tests: the prepared swept-path collision check vs the reference.

The reference oracle is the UNCHANGED footprint.pose_is_safe() (and its
local_bounds()/footprint_corners()/footprint_hits_obstacle()/
footprint_in_arena()) under the temporary guarded config that
swept_collision always built before the optimisation. The oracle side never
calls footprint.prepare_pose_check() or footprint.pose_is_safe_prepared().
Every randomised case uses a fixed seed.

Run from algo/:  python -m unittest tests.test_collision_hotpath_equivalence -v
"""

import dataclasses
import json
import math
import random
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import footprint  # noqa: E402
import state_lattice  # noqa: E402
import swept_collision as sc  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG, PlannerConfig  # noqa: E402
from models import Obstacle, Pose  # noqa: E402

CONFIG = DEFAULT_PLANNER_CONFIG
ACTIONS = ("F", "B", "FL", "FR", "BL", "BR")
SUMMARY = Path(__file__).resolve().parents[1] / "benchmarks" / "results" / "prefix_validation_summary.json"

CUSTOM_CONFIGS = (
    CONFIG,
    PlannerConfig(safety_margin_cm=0.0),
    PlannerConfig(robot_rear_extent_cm=3.5, robot_front_extent_cm=21.0),
    PlannerConfig(robot_width_cm=15.0, obstacle_cm=5),
    PlannerConfig(robot_width_cm=25.0, obstacle_cm=100.0),
    PlannerConfig(arena_cm=150.0, safety_margin_cm=2.5),
)


def reference_pose_is_safe(pose, obstacles, config, guard_cm):
    """Exactly the pre-change swept check for one sample: replace() + pose_is_safe()."""
    guarded = dataclasses.replace(config, safety_margin_cm=config.safety_margin_cm + guard_cm)
    return footprint.pose_is_safe(pose, obstacles, guarded)


def reference_safe_primitive_samples(start, action, value, obstacles, config=CONFIG):
    """The pre-change body of swept_collision.safe_primitive_samples(), verbatim."""
    obstacles = tuple(obstacles)
    samples, guard_cm = sc._sample_and_guard_cm(start, action, value, config)
    guarded_config = dataclasses.replace(
        config, safety_margin_cm=config.safety_margin_cm + guard_cm
    )
    if all(footprint.pose_is_safe(pose, obstacles, guarded_config) for pose in samples):
        return samples
    return None


def prepared_pose_is_safe(pose, obstacles, config, guard_cm):
    prepared = footprint.prepare_pose_check(config, guard_cm, tuple(obstacles))
    assert prepared is not None, "expected the prepared path for this input"
    return footprint.pose_is_safe_prepared(pose, *prepared)


def reference_aabb_overlap(pose, obstacle, config, guard_cm):
    """Reference-corner AABB overlap (strict), for anti-vacuity counting only."""
    guarded = dataclasses.replace(config, safety_margin_cm=config.safety_margin_cm + guard_cm)
    corners = footprint.footprint_corners(pose, guarded)
    xs = [x for x, _ in corners]
    ys = [y for _, y in corners]
    left, right = obstacle.x, obstacle.x + config.obstacle_cm
    bottom, top = obstacle.y, obstacle.y + config.obstacle_cm
    return max(xs) > min(left, right) and max(left, right) > min(xs) and \
        max(ys) > min(bottom, top) and max(bottom, top) > min(ys)


def bits(poses):
    return tuple((p.x.hex(), p.y.hex(), p.heading.hex()) for p in poses)


def random_obstacles(rng, count, arena):
    obstacles = []
    for index in range(count):
        style = rng.random()
        if style < 0.15:  # against a wall
            x = rng.choice((0.0, arena - 10.0, rng.uniform(0, arena - 10)))
            y = rng.choice((0.0, arena - 10.0)) if x not in (0.0, arena - 10.0) else rng.uniform(0, arena - 10)
        elif style < 0.25:  # integer grid position, as JSON maps give
            x, y = rng.randrange(0, int(arena) - 10, 5), rng.randrange(0, int(arena) - 10, 5)
        else:
            x, y = rng.uniform(-5, arena - 5), rng.uniform(-5, arena - 5)
        obstacles.append(Obstacle(index + 1, x, y, rng.choice("NSEW")))
    return tuple(obstacles)


class PreparedBoundsMatchReferenceBounds(unittest.TestCase):
    def test_bounds_are_bit_identical_to_local_bounds_under_the_guarded_config(self):
        rng = random.Random(20260930)
        for config in CUSTOM_CONFIGS:
            for guard in (0.0, 0.5, 0.25, 0.3793, rng.uniform(0, 3), rng.uniform(0, 3)):
                bounds, _, arena = footprint.prepare_pose_check(config, guard, ())
                guarded = dataclasses.replace(config, safety_margin_cm=config.safety_margin_cm + guard)
                reference = footprint.local_bounds(guarded)
                self.assertEqual([float(v).hex() for v in bounds], [float(v).hex() for v in reference])
                self.assertEqual([type(v) for v in bounds], [type(v) for v in reference])
                self.assertIs(arena, config.arena_cm)

    def test_obstacle_rectangles_use_the_reference_expressions(self):
        obstacles = (Obstacle(1, 50, 60, "N"), Obstacle(2, 12.25, 0.1, "S"))
        _, rects, _ = footprint.prepare_pose_check(CONFIG, 0.5, obstacles)
        for (left, right, bottom, top, *_), obstacle in zip(rects, obstacles):
            self.assertEqual((left, right, bottom, top), (
                obstacle.x, obstacle.x + CONFIG.obstacle_cm,
                obstacle.y, obstacle.y + CONFIG.obstacle_cm,
            ))


class RandomisedPoseEquivalence(unittest.TestCase):
    def test_pose_decisions_match_the_reference(self):
        counts = {"safe": 0, "hit": 0, "arena": 0, "aabb_but_separated": 0, "cases": 0}
        for seed in range(5):
            rng = random.Random(20260930 + seed)
            for config in CUSTOM_CONFIGS:
                arena = float(config.arena_cm)
                for _ in range(1700):
                    heading = rng.choice((
                        rng.uniform(-4 * math.pi, 4 * math.pi),
                        rng.randrange(-8, 9) * math.pi / 4,
                        0.0, math.pi / 2, math.pi, 3 * math.pi / 2,
                    ))
                    pose = Pose(rng.uniform(-30, arena + 30), rng.uniform(-30, arena + 30), heading)
                    obstacles = random_obstacles(rng, rng.randrange(0, 9), arena)
                    guard = rng.choice((0.5, 0.0, rng.uniform(0, 3)))
                    expected = reference_pose_is_safe(pose, obstacles, config, guard)
                    actual = prepared_pose_is_safe(pose, obstacles, config, guard)
                    self.assertIs(actual, expected, (seed, config, pose, obstacles, guard))
                    counts["cases"] += 1
                    guarded = dataclasses.replace(config, safety_margin_cm=config.safety_margin_cm + guard)
                    if not footprint.footprint_in_arena(pose, guarded):
                        counts["arena"] += 1
                    elif expected:
                        counts["safe"] += 1
                        counts["aabb_but_separated"] += sum(
                            reference_aabb_overlap(pose, o, config, guard) for o in obstacles)
                    else:
                        counts["hit"] += 1
        # Anti-vacuity: every branch of the prepared check was exercised.
        self.assertGreater(counts["cases"], 50000)
        for key in ("safe", "hit", "arena"):
            self.assertGreater(counts[key], counts["cases"] // 20, counts)
        self.assertGreater(counts["aabb_but_separated"], 100, counts)  # dense cover: near-contact test

    def test_near_contact_rotated_poses_match_the_reference(self):
        """Obstacles placed around each reference corner: AABB overlaps, rotated axes decide."""
        rng = random.Random(4242)
        outcomes = {True: 0, False: 0}
        aabb_but_separated = 0
        for _ in range(6000):
            heading = rng.uniform(-math.pi, math.pi)
            pose = Pose(rng.uniform(60, 140), rng.uniform(60, 140), heading)
            guard = rng.choice((0.5, rng.uniform(0, 1)))
            guarded = dataclasses.replace(CONFIG, safety_margin_cm=CONFIG.safety_margin_cm + guard)
            cx, cy = rng.choice(footprint.footprint_corners(pose, guarded))
            obstacle = Obstacle(1, cx + rng.uniform(-16.0, 6.0), cy + rng.uniform(-16.0, 6.0), "N")
            expected = reference_pose_is_safe(pose, (obstacle,), CONFIG, guard)
            self.assertIs(prepared_pose_is_safe(pose, (obstacle,), CONFIG, guard), expected,
                          (pose, obstacle, guard))
            outcomes[expected] += 1
            if expected and reference_aabb_overlap(pose, obstacle, CONFIG, guard):
                aabb_but_separated += 1
        self.assertGreater(min(outcomes.values()), 1000, outcomes)
        self.assertGreater(aabb_but_separated, 300, aabb_but_separated)


class ExactBoundaryEquivalence(unittest.TestCase):
    """Exact touching (strict SAT / inclusive arena) and one-ulp nudges either way."""

    def _both(self, pose, obstacles, guard=0.5, config=CONFIG):
        expected = reference_pose_is_safe(pose, obstacles, config, guard)
        self.assertIs(prepared_pose_is_safe(pose, obstacles, config, guard), expected,
                      (pose, obstacles, guard))
        return expected

    def test_axis_aligned_edge_touch_and_ulp_nudges(self):
        guard = 0.5
        guarded = dataclasses.replace(CONFIG, safety_margin_cm=CONFIG.safety_margin_cm + guard)
        pose = Pose(80.0, 90.0, 0.0)
        xs = [x for x, _ in footprint.footprint_corners(pose, guarded)]
        ys = [y for _, y in footprint.footprint_corners(pose, guarded)]
        front, top_y = max(xs), max(ys)
        outcomes = set()
        for edge in (front, math.nextafter(front, -math.inf), math.nextafter(front, math.inf)):
            outcomes.add(self._both(pose, (Obstacle(1, edge, 92.0, "W"),), guard))
        for edge in (top_y, math.nextafter(top_y, -math.inf), math.nextafter(top_y, math.inf)):
            outcomes.add(self._both(pose, (Obstacle(1, 85.0, edge, "S"),), guard))
        # Touching obstacle's far side against the footprint's rear/right faces.
        rear, right_y = min(xs), min(ys)
        for edge in (rear, math.nextafter(rear, math.inf), math.nextafter(rear, -math.inf)):
            outcomes.add(self._both(pose, (Obstacle(1, edge - CONFIG.obstacle_cm, 92.0, "E"),), guard))
        for edge in (right_y, math.nextafter(right_y, math.inf)):
            outcomes.add(self._both(pose, (Obstacle(1, 85.0, edge - CONFIG.obstacle_cm, "N"),), guard))
        self.assertEqual(outcomes, {True, False})

    def test_rotated_corner_touch_and_ulp_nudges(self):
        rng = random.Random(77)
        outcomes = set()
        for heading in (math.pi / 2, math.pi, 3 * math.pi / 2, math.pi / 4, 1.0, -2.2,
                        *(rng.uniform(-math.pi, math.pi) for _ in range(40))):
            pose = Pose(100.0, 100.0, heading)
            guarded = dataclasses.replace(CONFIG, safety_margin_cm=CONFIG.safety_margin_cm + 0.5)
            corners = footprint.footprint_corners(pose, guarded)
            top_y = max(y for _, y in corners)
            top_x = next(x for x, y in corners if y == top_y)
            for edge in (top_y, math.nextafter(top_y, -math.inf), math.nextafter(top_y, math.inf)):
                outcomes.add(self._both(pose, (Obstacle(1, top_x - 5.0, edge, "S"),)))
            max_x = max(x for x, _ in corners)
            for edge in (max_x, math.nextafter(max_x, -math.inf)):
                outcomes.add(self._both(pose, (Obstacle(1, edge, 95.0, "W"),)))
        self.assertEqual(outcomes, {True, False})

    def test_arena_wall_touch_and_ulp_nudges(self):
        guarded = dataclasses.replace(CONFIG, safety_margin_cm=CONFIG.safety_margin_cm + 0.5)
        rear, front, right, left = footprint.local_bounds(guarded)
        outcomes = set()
        for x in (-rear, math.nextafter(-rear, -math.inf), math.nextafter(-rear, math.inf)):
            outcomes.add(self._both(Pose(x, 100.0, 0.0), ()))
        for x in (CONFIG.arena_cm - front, math.nextafter(CONFIG.arena_cm - front, math.inf)):
            outcomes.add(self._both(Pose(x, 100.0, 0.0), ()))
        for y in (-right, math.nextafter(-right, -math.inf)):
            outcomes.add(self._both(Pose(100.0, y, 0.0), ()))
        for y in (CONFIG.arena_cm - left, math.nextafter(CONFIG.arena_cm - left, math.inf)):
            outcomes.add(self._both(Pose(100.0, y, 0.0), ()))
        self.assertEqual(outcomes, {True, False})

    def test_containment_signed_zero_and_empty_cases(self):
        self._both(Pose(100.0, 100.0, 0.3), (Obstacle(1, 105.0, 97.0, "N"),))  # obstacle inside footprint
        big = PlannerConfig(obstacle_cm=100.0)
        self._both(Pose(80.0, 80.0, 0.7), (Obstacle(1, 50.0, 50.0, "N"),), config=big)  # footprint inside obstacle
        self._both(Pose(-0.0, 10.5, -0.0), ())
        self._both(Pose(0.0, 100.0, 0.0), (Obstacle(1, -0.0, -0.0, "N"),))
        self._both(Pose(100.0, 100.0, 1.0), ())

    def test_non_finite_pose_values_match_the_reference(self):
        for pose in (Pose(math.nan, 100.0, 0.0), Pose(100.0, math.nan, 0.0),
                     Pose(100.0, 100.0, math.nan), Pose(math.inf, 100.0, 0.0)):
            self.assertFalse(self._both(pose, (Obstacle(1, 50.0, 50.0, "N"),)))
        pose = Pose(100.0, 100.0, math.inf)
        with self.assertRaises(ValueError) as reference:
            reference_pose_is_safe(pose, (), CONFIG, 0.5)
        with self.assertRaises(ValueError) as prepared:
            prepared_pose_is_safe(pose, (), CONFIG, 0.5)
        self.assertEqual(str(prepared.exception), str(reference.exception))


class ReferenceFallback(unittest.TestCase):
    """Inputs outside the proven domain must use the unchanged reference path."""

    def _counting(self):
        return mock.patch.object(footprint, "pose_is_safe", wraps=footprint.pose_is_safe)

    def test_default_config_takes_the_prepared_path(self):
        with self._counting() as spy:
            sc.safe_primitive_samples(Pose(100.0, 100.0, 0.0), "FL", 30, [Obstacle(1, 40, 40, "N")], CONFIG)
        spy.assert_not_called()

    def test_prepare_rejects_inputs_outside_the_proven_domain(self):
        bad_obstacles = (
            (Obstacle(1, math.nan, 50.0, "N"),), (Obstacle(1, 50.0, math.inf, "N"),),
            (Obstacle(1, True, 50.0, "N"),), (Obstacle(1, 2 ** 60, 50.0, "N"),),
        )
        for obstacles in bad_obstacles:
            self.assertIsNone(footprint.prepare_pose_check(CONFIG, 0.5, obstacles), obstacles)
        # Finite x, but x + size overflows to inf.
        self.assertIsNone(footprint.prepare_pose_check(
            PlannerConfig(obstacle_cm=1e308), 0.5, (Obstacle(1, 1.7e308, 50.0, "N"),)))
        self.assertIsNone(footprint.prepare_pose_check(CONFIG, math.nan, ()))
        for config in (PlannerConfig(arena_cm=math.inf), PlannerConfig(obstacle_cm=math.nan),
                       PlannerConfig(safety_margin_cm=True), PlannerConfig(robot_width_cm="19")):
            self.assertIsNone(footprint.prepare_pose_check(config, 0.5, ()), config)

    def test_fallback_inputs_still_give_reference_results(self):
        start = Pose(60.0, 60.0, 0.4)
        cases = (
            ((Obstacle(1, math.nan, 50.0, "N"),), CONFIG),
            ((Obstacle(1, 70.0, math.inf, "N"),), CONFIG),
            ((Obstacle(1, 75.0, 55.0, "N"),), PlannerConfig(arena_cm=math.inf)),
            ((Obstacle(1, 2 ** 60, 50.0, "N"),), CONFIG),
        )
        for obstacles, config in cases:
            for action in ACTIONS:
                with self._counting() as spy:
                    actual = sc.safe_primitive_samples(start, action, 20, obstacles, config)
                self.assertTrue(spy.called)
                expected = reference_safe_primitive_samples(start, action, 20, obstacles, config)
                self.assertEqual(actual is None, expected is None)
                if expected is not None:
                    self.assertEqual(bits(actual), bits(expected))

    def test_planner_config_subclass_uses_the_reference_path(self):
        @dataclasses.dataclass(frozen=True)
        class TracedConfig(PlannerConfig):
            pass

        config = TracedConfig()
        obstacles = (Obstacle(1, 70.0, 55.0, "N"),)
        for action in ACTIONS:
            with self._counting() as spy:
                actual = sc.safe_primitive_samples(Pose(60.0, 60.0, 0.4), action, 30, obstacles, config)
            self.assertTrue(spy.called)
            expected = reference_safe_primitive_samples(Pose(60.0, 60.0, 0.4), action, 30, obstacles, config)
            self.assertEqual(actual is None, expected is None)

    def test_non_dataclass_config_still_raises_the_replace_type_error(self):
        """Regression: the pre-change code raised dataclasses.replace()'s TypeError."""
        config = types.SimpleNamespace(**dataclasses.asdict(CONFIG))
        with self.assertRaises(TypeError) as expected:
            dataclasses.replace(config, safety_margin_cm=1.5)
        for action in ACTIONS:
            with self.assertRaises(TypeError) as raised:
                sc.safe_primitive_samples(Pose(100.0, 100.0, 0.0), action, 10, [], config)
            self.assertEqual(str(raised.exception), str(expected.exception))
            with self.assertRaises(TypeError):
                reference_safe_primitive_samples(Pose(100.0, 100.0, 0.0), action, 10, [], config)


class PrimitiveEquivalence(unittest.TestCase):
    def _compare(self, start, action, value, obstacles, config=CONFIG):
        expected = reference_safe_primitive_samples(start, action, value, obstacles, config)
        actual = sc.safe_primitive_samples(start, action, value, obstacles, config)
        self.assertEqual(actual is None, expected is None, (start, action, value, obstacles))
        if expected is not None:
            self.assertEqual(bits(actual), bits(expected))
        return expected is not None

    def test_randomised_primitives_match_the_reference(self):
        rng = random.Random(9001)
        outcomes = {True: 0, False: 0}
        values = {"F": (1, 10, 17, 55, 999), "B": (1, 10, 17, 55, 999)}
        for _ in range(900):
            start = Pose(rng.uniform(15, 185), rng.uniform(15, 185), rng.uniform(-math.pi, math.pi))
            obstacles = random_obstacles(rng, rng.randrange(0, 9), 200.0)
            config = rng.choice(CUSTOM_CONFIGS[:4])
            action = rng.choice(ACTIONS)
            value = rng.choice(values.get(action, (1, 10, 17, 90, 180, 360)))
            outcomes[self._compare(start, action, value, obstacles, config)] += 1
        self.assertGreater(min(outcomes.values()), 150, outcomes)

    def test_lattice_successors_near_contact_match_the_reference(self):
        """Every lattice successor from poses at graded distances from an obstacle."""
        obstacle = Obstacle(1, 100.0, 100.0, "N")
        outcomes = {True: 0, False: 0}
        primitives = state_lattice._successor_primitives(CONFIG)
        for gap in (0.0, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 6.0, 12.0):
            for heading_deg in range(0, 360, 15):
                heading = math.radians(heading_deg)
                # Rear reference placed so the front of the footprint is `gap` from the obstacle.
                reach = CONFIG.robot_front_extent_cm + CONFIG.safety_margin_cm + gap
                start = Pose(105.0 - (reach + 5.0) * math.cos(heading),
                             105.0 - (reach + 5.0) * math.sin(heading), heading)
                for action, value in primitives:
                    outcomes[self._compare(start, action, value, (obstacle,))] += 1
        self.assertGreater(min(outcomes.values()), 200, outcomes)

    def test_fixture_poses_match_the_reference(self):
        from benchmarks.fixtures import load_fixture

        primitives = state_lattice._successor_primitives(CONFIG)
        for fixture_id in ("A_spread_04", "B_clutter_04", "A_spread_08", "D_seeded_04_s1"):
            fixture = load_fixture(fixture_id)
            for x in range(10, 200, 20):
                for y in range(10, 200, 20):
                    for heading_deg in (0, 50, 135, 270):
                        start = Pose(float(x), float(y), math.radians(heading_deg))
                        for action, value in primitives:
                            self._compare(start, action, value, fixture.obstacles)

    def test_one_shot_obstacle_iterator_is_consumed_once(self):
        obstacles = [Obstacle(1, 60.0, 60.0, "N"), Obstacle(2, 120.0, 40.0, "N")]
        expected = reference_safe_primitive_samples(Pose(30.0, 60.0, 0.0), "F", 30, iter(obstacles))
        actual = sc.safe_primitive_samples(Pose(30.0, 60.0, 0.0), "F", 30, iter(obstacles))
        self.assertEqual(actual is None, expected is None)


class LatticeLegEquivalence(unittest.TestCase):
    def test_lattice_plans_are_identical_with_the_reference_checker(self):
        cases = (
            (Pose(20.0, 10.0, math.pi / 2), Pose(40.0, 70.0, 0.0), (Obstacle(1, 70.0, 60.0, "W"),)),
            (Pose(20.0, 10.0, math.pi / 2), Pose(60.0, 40.0, math.pi / 2),
             (Obstacle(1, 60.0, 90.0, "S"), Obstacle(2, 120.0, 20.0, "N"))),
        )
        for start, goal, obstacles in cases:
            optimised = state_lattice.plan_lattice_path(start, goal, obstacles, CONFIG)
            # plan_lattice_path() reaches the checker through the per-leg entry point, so
            # patch it too; otherwise this would compare the optimised path with itself.
            def reference_for_leg(start_pose, action, value, leg):
                return reference_safe_primitive_samples(start_pose, action, value, leg.obstacles, leg.config)

            with mock.patch.object(state_lattice.swept_collision, "safe_primitive_samples",
                                   reference_safe_primitive_samples), \
                    mock.patch.object(state_lattice.swept_collision, "_safe_primitive_samples_for_leg",
                                      side_effect=reference_for_leg) as reference_calls:
                reference = state_lattice.plan_lattice_path(start, goal, obstacles, CONFIG)
            self.assertGreater(reference_calls.call_count, 0)
            self.assertTrue(reference.expansions > 0)
            self.assertEqual(repr(optimised), repr(reference))
            self.assertEqual(optimised.final_pose.x.hex(), reference.final_pose.x.hex())


class GoldenRouteHashes(unittest.TestCase):
    """Real-pipeline prefix B2 routes reproduce the full committed Phase A hashes.

    The committed hashes were recorded at the historical fixture geometry (1 cm
    margin, 20 cm standoff, 5 cm goal tolerance;
    `benchmarks.fixtures.FIXTURE_GEOMETRY`), not at the
    live defaults, so the search runs with that geometry to reproduce them."""

    def test_small_fixture_routes_match_committed_hashes(self):
        import route_prefix_search as rps
        from benchmarks import normalized
        from benchmarks.fixtures import FIXTURE_GEOMETRY, load_fixture

        cases = json.loads(SUMMARY.read_text(encoding="utf-8"))["cases"]
        for case_id in ("R1", "R4"):
            case = cases[case_id]
            self.assertEqual(case["mode"], "pb2")
            self.assertEqual(case["alternatives"], "off")
            fixture = load_fixture(case["fixture"])
            result = rps.find_prefix_b2_route(fixture.start, fixture.obstacles, config=FIXTURE_GEOMETRY,
                                              allow_alternative_viewing=False)
            record = normalized.normalize_prefix_result(result, search="prefix_b2",
                                                        start_pose=fixture.start, alternatives=False)
            self.assertEqual(record["hash"], case["result_hash"], case_id)
            self.assertEqual(repr(record["total_cost"]), repr(case["total_cost"]), case_id)
            self.assertEqual(repr(record["total_distance_cm"]), repr(case["total_distance_cm"]), case_id)


if __name__ == "__main__":
    unittest.main()
