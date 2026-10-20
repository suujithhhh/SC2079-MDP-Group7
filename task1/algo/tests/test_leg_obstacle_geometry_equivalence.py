"""Differential tests: per-leg prepared obstacle geometry vs the per-primitive path.

The oracles are the UNCHANGED footprint.prepare_pose_check(), footprint.
pose_is_safe() (through the guarded temporary config), the public
swept_collision.safe_primitive_samples() and a verbatim copy of the pre-change
body of safe_primitive_samples(). The optimised side is
footprint.prepare_obstacle_rects()/prepare_primitive_check() and
swept_collision._prepare_leg_geometry()/_safe_primitive_samples_for_leg().
Every randomised case uses a fixed seed.

The mutation tests run each checker under a deliberately broken
implementation and require it to raise AssertionError, i.e. they prove the
checker can detect that class of error.

Run from algo/:  python -m unittest tests.test_leg_obstacle_geometry_equivalence -v
"""

import dataclasses
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

CUSTOM_CONFIGS = (
    CONFIG,
    PlannerConfig(safety_margin_cm=0.0),
    PlannerConfig(robot_rear_extent_cm=3.5, robot_front_extent_cm=21.0),
    PlannerConfig(robot_width_cm=15.0, obstacle_cm=5),
    PlannerConfig(robot_width_cm=25.0, obstacle_cm=100.0),
    PlannerConfig(arena_cm=150.0, safety_margin_cm=2.5),
)

LATTICE_CASES = (
    (Pose(20.0, 10.0, math.pi / 2), Pose(40.0, 70.0, 0.0), (Obstacle(1, 70.0, 60.0, "W"),)),
    (Pose(20.0, 10.0, math.pi / 2), Pose(60.0, 40.0, math.pi / 2),
     (Obstacle(1, 60.0, 90.0, "S"), Obstacle(2, 120.0, 20.0, "N"))),
    (Pose(20.0, 10.0, math.pi / 2), Pose(40.0, 70.0, 0.0),
     (Obstacle(1, 70.0, 60.0, "W"), Obstacle(2, 70.0, 60.0, "W"), Obstacle(3, 120.0, 20.0, "N"))),
)


def reference_safe_primitive_samples(start, action, value, obstacles, config=CONFIG):
    """The pre-PR-#19 body of swept_collision.safe_primitive_samples(), verbatim."""
    obstacles = tuple(obstacles)
    samples, guard_cm = sc._sample_and_guard_cm(start, action, value, config)
    guarded_config = dataclasses.replace(
        config, safety_margin_cm=config.safety_margin_cm + guard_cm
    )
    if all(footprint.pose_is_safe(pose, obstacles, guarded_config) for pose in samples):
        return samples
    return None


def reference_pose_is_safe(pose, obstacles, config, guard_cm):
    guarded = dataclasses.replace(config, safety_margin_cm=config.safety_margin_cm + guard_cm)
    return footprint.pose_is_safe(pose, obstacles, guarded)


def bits(poses):
    return tuple((p.x.hex(), p.y.hex(), p.heading.hex()) for p in poses)


def canonical(value):
    """Type-and-bits form of a number or nested tuple of numbers."""
    if isinstance(value, tuple):
        return tuple(canonical(item) for item in value)
    return (type(value).__name__, float(value).hex())


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
    if obstacles and rng.random() < 0.3:  # duplicate an obstacle (kept, in order)
        obstacles.insert(rng.randrange(len(obstacles) + 1), obstacles[0])
    return tuple(obstacles)


def split_prepared(config, guard_cm, obstacles):
    rects = footprint.prepare_obstacle_rects(config, obstacles)
    if rects is None:
        return None
    return footprint.prepare_primitive_check(config, guard_cm, rects)


def leg_samples(start, action, value, obstacles, config=CONFIG):
    leg = sc._prepare_leg_geometry(tuple(obstacles), config)
    return sc._safe_primitive_samples_for_leg(start, action, value, leg)


def reference_leg_planner():
    """Patches that route plan_lattice_path() through the pre-change per-primitive call."""
    def reference_for_leg(start, action, value, leg):
        return reference_safe_primitive_samples(start, action, value, leg.obstacles, leg.config)
    return mock.patch.object(sc, "_safe_primitive_samples_for_leg", side_effect=reference_for_leg)


# --- Checkers (also run under deliberate mutations) ---------------------------

def check_split_equals_oracle(test):
    rng = random.Random(20260930)
    compared = 0
    for config in CUSTOM_CONFIGS:
        for guard in (0.0, 0.5, 0.25, 0.3793, rng.uniform(0, 3), rng.uniform(0, 3)):
            for count in (0, 1, 2, 3, 5, 8):
                obstacles = random_obstacles(rng, count, config.arena_cm)
                oracle = footprint.prepare_pose_check(config, guard, obstacles)
                split = split_prepared(config, guard, obstacles)
                test.assertIsNotNone(oracle)
                test.assertIsNotNone(split, (config, guard, obstacles))
                test.assertEqual(canonical(split[0]), canonical(oracle[0]), "split bounds differ from oracle")
                test.assertEqual(canonical(split[1]), canonical(oracle[1]), "split rects differ from oracle")
                test.assertEqual(len(split[1]), len(obstacles))
                test.assertIs(split[2], config.arena_cm)
                test.assertEqual(type(split[2]), type(oracle[2]))
                compared += 1
    test.assertGreater(compared, 200)
    # Out-of-domain guards: both split halves and the oracle return None.
    for guard in (math.nan, math.inf, -math.inf, True, "1", None):
        test.assertIsNone(footprint.prepare_pose_check(CONFIG, guard, ()))
        test.assertIsNone(split_prepared(CONFIG, guard, ()), guard)


def _boundary_mismatches(test):
    """Pose decisions at exact contact and +-ulp around it, split path vs reference."""
    mismatches = 0
    outcomes = {True: 0, False: 0}
    scenarios = []
    for heading_deg in (0.0, 30.0, 45.0, 90.0, 137.0, 200.0, 270.0, 333.0):
        heading = math.radians(heading_deg)
        for guard in (0.0, 0.37):
            for config, obstacle in (
                (CONFIG, Obstacle(1, 100.0, 100.0, "N")),
                (CONFIG, Obstacle(1, 63.3, 71.7, "N")),
                (PlannerConfig(obstacle_cm=100.0), Obstacle(1, 40.0, 40.0, "N")),
            ):
                scenarios.append((config, (obstacle,), heading, guard))
            scenarios.append((CONFIG, (), heading, guard))  # arena wall
    for config, obstacles, heading, guard in scenarios:
        direction = (math.cos(heading), math.sin(heading))
        # Approach the obstacle (or the wall) from a safe pose along the heading.
        if obstacles:
            origin = (obstacles[0].x + config.obstacle_cm / 2 - direction[0] * 80.0,
                      obstacles[0].y + config.obstacle_cm / 2 - direction[1] * 80.0)
        else:
            origin = (100.0, 100.0)

        def decide(t):
            pose = Pose(origin[0] + t * direction[0], origin[1] + t * direction[1], heading)
            return reference_pose_is_safe(pose, obstacles, config, guard)

        lo, hi = 0.0, 400.0
        if decide(lo) is not True or decide(hi) is not False:
            continue
        for _ in range(200):
            mid = (lo + hi) / 2
            if mid in (lo, hi):
                break
            lo, hi = (mid, hi) if decide(mid) else (lo, mid)
        probes = [lo]
        for direction_sign in (-math.inf, math.inf):
            t = lo
            for _ in range(6):
                t = math.nextafter(t, direction_sign)
                probes.append(t)
        prepared = split_prepared(config, guard, obstacles)
        test.assertIsNotNone(prepared)
        for t in probes:
            pose = Pose(origin[0] + t * direction[0], origin[1] + t * direction[1], heading)
            expected = reference_pose_is_safe(pose, obstacles, config, guard)
            actual = footprint.pose_is_safe_prepared(pose, *prepared)
            outcomes[expected] += 1
            mismatches += actual != expected
    test.assertGreater(min(outcomes.values()), 50, outcomes)
    return mismatches


def check_boundaries(test):
    test.assertEqual(_boundary_mismatches(test), 0, "boundary decisions differ from the reference")


def check_near_contact_leg_vs_reference(test):
    obstacle = Obstacle(1, 100.0, 100.0, "N")
    primitives = state_lattice._successor_primitives(CONFIG)
    outcomes = {True: 0, False: 0}
    for gap in (0.0, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 6.0, 12.0):
        for heading_deg in range(0, 360, 15):
            heading = math.radians(heading_deg)
            reach = CONFIG.robot_front_extent_cm + CONFIG.safety_margin_cm + gap
            start = Pose(105.0 - (reach + 5.0) * math.cos(heading),
                         105.0 - (reach + 5.0) * math.sin(heading), heading)
            leg = sc._prepare_leg_geometry((obstacle,), CONFIG)
            for action, value in primitives:
                expected = reference_safe_primitive_samples(start, action, value, (obstacle,))
                actual = sc._safe_primitive_samples_for_leg(start, action, value, leg)
                test.assertEqual(actual is None, expected is None, (start, action, value))
                if expected is not None:
                    test.assertEqual(bits(actual), bits(expected))
                outcomes[expected is not None] += 1
    test.assertGreater(min(outcomes.values()), 200, outcomes)


class FallbackCases:
    """Inputs outside the hoist domain, each paired with the config to use."""

    @staticmethod
    def build():
        @dataclasses.dataclass(frozen=True)
        class SubObstacle(Obstacle):
            pass

        @dataclasses.dataclass(frozen=True)
        class SubConfig(PlannerConfig):
            pass

        def obstacle(x, y):
            return Obstacle(1, x, y, "N")

        return {
            "obstacle_subclass": ((SubObstacle(1, 75.0, 55.0, "N"),), CONFIG),
            "duck_typed_obstacle": ((types.SimpleNamespace(x=75.0, y=55.0),), CONFIG),
            "nan_x": ((obstacle(math.nan, 50.0),), CONFIG),
            "inf_y": ((obstacle(70.0, math.inf),), CONFIG),
            "bool_x": ((obstacle(True, 50.0),), CONFIG),
            "huge_int": ((obstacle(2 ** 60, 50.0),), CONFIG),
            "sum_overflow": ((obstacle(1.7e308, 50.0),), PlannerConfig(obstacle_cm=1e308)),
            "nonfinite_config": ((obstacle(75.0, 55.0),), PlannerConfig(arena_cm=math.inf)),
            "bool_config": ((obstacle(75.0, 55.0),), PlannerConfig(safety_margin_cm=True)),
            "config_subclass": ((obstacle(75.0, 55.0),), SubConfig()),
            "mixed_good_and_bad": ((obstacle(75.0, 55.0), obstacle(math.nan, 5.0)), CONFIG),
        }


def check_fallback_domain(test):
    start = Pose(60.0, 60.0, 0.4)
    for name, (obstacles, config) in FallbackCases.build().items():
        leg = sc._prepare_leg_geometry(obstacles, config)
        test.assertIsNone(leg.rects, f"rects must be None for {name}")
        test.assertIs(leg.obstacles, obstacles)
        test.assertIs(leg.config, config)
        for action in ACTIONS:
            with mock.patch.object(footprint, "prepare_primitive_check",
                                   wraps=footprint.prepare_primitive_check) as split:
                actual = sc._safe_primitive_samples_for_leg(start, action, 20, leg)
            split.assert_not_called()
            expected = sc.safe_primitive_samples(start, action, 20, obstacles, config)
            test.assertEqual(actual is None, expected is None, (name, action))
            if expected is not None:
                test.assertEqual(bits(actual), bits(expected))


class _Flaky:
    """Duck-typed obstacle whose x changes on every read."""

    def __init__(self):
        self.reads = []

    @property
    def x(self):
        self.reads.append("x")
        return 75.0 + 0.5 * len(self.reads)

    @property
    def y(self):
        self.reads.append("y")
        return 55.0


@dataclasses.dataclass(frozen=True)
class _LoggedObstacle(Obstacle):
    """Obstacle subclass that logs coordinate reads (values stay constant)."""

    def __post_init__(self):
        object.__setattr__(self, "reads", [])

    def __getattribute__(self, name):
        if name in ("x", "y"):
            object.__getattribute__(self, "reads").append(name)
        return object.__getattribute__(self, name)


def check_property_backed_reads(test):
    """Out-of-domain obstacles are not read at leg start, then are read exactly as before."""
    start = Pose(60.0, 60.0, 0.4)
    for make in (_Flaky, lambda: _LoggedObstacle(1, 75.0, 55.0, "N")):
        for action in ACTIONS:
            via_public, via_leg = make(), make()
            leg = sc._prepare_leg_geometry((via_leg,), CONFIG)
            test.assertEqual(via_leg.reads, [], "obstacle read at leg start")
            test.assertIsNone(leg.rects)
            public = sc.safe_primitive_samples(start, action, 20, (via_public,), CONFIG)
            hoisted = sc._safe_primitive_samples_for_leg(start, action, 20, leg)
            test.assertEqual(hoisted is None, public is None, action)
            if public is not None:
                test.assertEqual(bits(hoisted), bits(public))
            test.assertEqual(via_leg.reads, via_public.reads, "per-primitive reads differ")
            test.assertGreater(len(via_leg.reads), 0)


def check_prepared_once_per_leg(test):
    """One preparation per plan_lattice_path() call, none per primitive."""
    for start, goal, obstacles in LATTICE_CASES:
        with mock.patch.object(sc, "_prepare_leg_geometry", wraps=sc._prepare_leg_geometry) as prepare, \
                mock.patch.object(sc, "_safe_primitive_samples_for_leg",
                                  wraps=sc._safe_primitive_samples_for_leg) as per_primitive, \
                mock.patch.object(sc, "safe_primitive_samples") as public, \
                mock.patch.object(footprint, "prepare_pose_check") as per_primitive_prepare, \
                mock.patch.object(footprint, "prepare_obstacle_rects",
                                  wraps=footprint.prepare_obstacle_rects) as rects:
            public.side_effect = AssertionError("public path used in the hoist domain")
            per_primitive_prepare.side_effect = AssertionError("per-primitive preparation")
            plan = state_lattice.plan_lattice_path(start, goal, iter(obstacles), CONFIG)
        test.assertEqual(prepare.call_count, 1, "leg geometry must be prepared exactly once")
        test.assertEqual(rects.call_count, 1, "leg geometry must be prepared exactly once")
        test.assertTrue(plan.is_valid)
        # Every expansion but the goal test's own tries all six primitives.
        test.assertEqual(per_primitive.call_count, 6 * (plan.expansions - 1))
        test.assertGreater(per_primitive.call_count, 0)
        for call in per_primitive.call_args_list:
            test.assertIs(call.args[3].obstacles, prepare.call_args.args[0])
    # Legs that return before the search loop never prepare.
    unsafe_start = Pose(1.0, 1.0, 0.0)
    for start, goal in ((unsafe_start, Pose(100.0, 100.0, 0.0)),
                        (Pose(100.0, 100.0, 0.0), Pose(-50.0, 100.0, 0.0)),
                        (Pose(100.0, 100.0, 0.0), Pose(101.0, 100.0, 0.0))):
        with mock.patch.object(sc, "_prepare_leg_geometry", wraps=sc._prepare_leg_geometry) as prepare:
            state_lattice.plan_lattice_path(start, goal, (), CONFIG)
        test.assertEqual(prepare.call_count, 0, (start, goal))


# --- Tests ---------------------------------------------------------------------

class SplitEqualsOracle(unittest.TestCase):
    def test_split_reproduces_prepare_pose_check_bitwise(self):
        check_split_equals_oracle(self)

    def test_rectangles_use_the_reference_expressions_in_input_order(self):
        obstacles = (Obstacle(1, 50, 60, "N"), Obstacle(2, 12.25, 0.1, "S"), Obstacle(1, 50, 60, "N"))
        rects = footprint.prepare_obstacle_rects(CONFIG, obstacles)
        self.assertEqual(len(rects), 3)
        self.assertEqual(rects[0], rects[2])
        for rect, obstacle in zip(rects, obstacles):
            left, right, bottom, top, *_ = rect
            self.assertEqual((left, right, bottom, top), (
                obstacle.x, obstacle.x + CONFIG.obstacle_cm,
                obstacle.y, obstacle.y + CONFIG.obstacle_cm,
            ))
        self.assertEqual(footprint.prepare_obstacle_rects(CONFIG, ()), ())

    def test_bounds_are_bit_identical_to_local_bounds_under_the_guarded_config(self):
        rng = random.Random(7)
        for config in CUSTOM_CONFIGS:
            rects = footprint.prepare_obstacle_rects(config, ())
            for guard in (0.0, 0.5, 0.3793, rng.uniform(0, 3)):
                bounds, out_rects, arena = footprint.prepare_primitive_check(config, guard, rects)
                guarded = dataclasses.replace(config, safety_margin_cm=config.safety_margin_cm + guard)
                self.assertEqual(canonical(bounds), canonical(footprint.local_bounds(guarded)))
                self.assertIs(out_rects, rects)
                self.assertIs(arena, config.arena_cm)

    def test_bounds_follow_each_primitives_own_guard(self):
        rects = footprint.prepare_obstacle_rects(CONFIG, (Obstacle(1, 50.0, 50.0, "N"),))
        first = footprint.prepare_primitive_check(CONFIG, 0.25, rects)
        second = footprint.prepare_primitive_check(CONFIG, 0.5, rects)
        self.assertNotEqual(canonical(first[0]), canonical(second[0]))
        self.assertIs(first[1], second[1])


class PoseBoundaries(unittest.TestCase):
    def test_contact_and_ulp_neighbours_match_the_reference(self):
        check_boundaries(self)

    def test_exact_axis_aligned_touch(self):
        obstacle = Obstacle(1, 100.0, 60.0, "N")
        for guard in (0.0, 0.5):
            front = CONFIG.robot_front_extent_cm + CONFIG.safety_margin_cm + guard
            prepared = split_prepared(CONFIG, guard, (obstacle,))
            x = 100.0 - front
            for offset, expected in ((-1e-9, True), (0.0, True), (1e-9, False)):
                pose = Pose(x + offset, 65.0, 0.0)
                self.assertEqual(reference_pose_is_safe(pose, (obstacle,), CONFIG, guard), expected)
                self.assertEqual(footprint.pose_is_safe_prepared(pose, *prepared), expected)

    def test_obstacle_inside_footprint_and_footprint_inside_obstacle(self):
        small = (Obstacle(1, 100.0, 100.0, "N"),)
        pose = Pose(90.0, 105.0, 0.0)
        self.assertFalse(reference_pose_is_safe(pose, small, CONFIG, 0.5))
        self.assertFalse(footprint.pose_is_safe_prepared(pose, *split_prepared(CONFIG, 0.5, small)))
        big_config = PlannerConfig(obstacle_cm=100.0)
        big = (Obstacle(1, 40.0, 40.0, "N"),)
        pose = Pose(70.0, 90.0, 0.3)
        self.assertFalse(reference_pose_is_safe(pose, big, big_config, 0.5))
        self.assertFalse(footprint.pose_is_safe_prepared(pose, *split_prepared(big_config, 0.5, big)))

    def test_empty_obstacles_keep_the_arena_only_decision(self):
        prepared = split_prepared(CONFIG, 0.5, ())
        for pose in (Pose(100.0, 100.0, 0.0), Pose(1.0, 100.0, 0.0), Pose(199.0, 100.0, 0.0)):
            self.assertEqual(footprint.pose_is_safe_prepared(pose, *prepared),
                             reference_pose_is_safe(pose, (), CONFIG, 0.5))


class LegPathVersusPublicPath(unittest.TestCase):
    def _compare(self, start, action, value, obstacles, config=CONFIG):
        expected = reference_safe_primitive_samples(start, action, value, obstacles, config)
        public = sc.safe_primitive_samples(start, action, value, obstacles, config)
        actual = leg_samples(start, action, value, obstacles, config)
        self.assertEqual(actual is None, expected is None, (start, action, value, obstacles))
        self.assertEqual(public is None, expected is None)
        if expected is not None:
            self.assertEqual(bits(actual), bits(expected))
            self.assertEqual(bits(public), bits(expected))
        return expected is not None

    def test_random_primitives_all_actions(self):
        rng = random.Random(4242)
        outcomes = {True: 0, False: 0}
        values = {"F": (1, 10, 17, 55, 999), "B": (1, 10, 17, 55, 999)}
        for _ in range(900):
            start = Pose(rng.uniform(15, 185), rng.uniform(15, 185), rng.uniform(-math.pi, math.pi))
            config = rng.choice(CUSTOM_CONFIGS[:4])
            obstacles = random_obstacles(rng, rng.randrange(0, 9), 200.0)
            action = rng.choice(ACTIONS)
            value = rng.choice(values.get(action, (1, 10, 17, 90, 180, 360)))
            outcomes[self._compare(start, action, value, obstacles, config)] += 1
        self.assertGreater(min(outcomes.values()), 150, outcomes)

    def test_one_leg_serves_every_primitive_and_start(self):
        """One prepared leg reused across many starts/actions, as plan_lattice_path() does."""
        from benchmarks.fixtures import load_fixture

        primitives = state_lattice._successor_primitives(CONFIG)
        for fixture_id in ("A_spread_04", "B_clutter_04", "A_spread_08"):
            obstacles = tuple(load_fixture(fixture_id).obstacles)
            leg = sc._prepare_leg_geometry(obstacles, CONFIG)
            self.assertIsNotNone(leg.rects)
            for x in range(10, 200, 30):
                for y in range(10, 200, 30):
                    for heading_deg in (0, 50, 135, 270):
                        start = Pose(float(x), float(y), math.radians(heading_deg))
                        for action, value in primitives:
                            expected = reference_safe_primitive_samples(start, action, value, obstacles)
                            actual = sc._safe_primitive_samples_for_leg(start, action, value, leg)
                            self.assertEqual(actual is None, expected is None)
                            if expected is not None:
                                self.assertEqual(bits(actual), bits(expected))

    def test_near_contact_lattice_successors(self):
        check_near_contact_leg_vs_reference(self)

    def test_empty_single_multiple_and_duplicate_obstacles(self):
        start = Pose(60.0, 60.0, 0.4)
        one = Obstacle(1, 75.0, 55.0, "N")
        for obstacles in ((), (one,), (one, Obstacle(2, 20.0, 20.0, "N")), (one, one, one)):
            for action in ACTIONS:
                self._compare(start, action, 20, obstacles)

    def test_wall_adjacent_starts(self):
        for start in (Pose(13.0, 100.0, math.pi), Pose(100.0, 187.0, math.pi / 2),
                      Pose(10.5, 10.5, 0.0)):
            for action in ACTIONS:
                for value in (1, 5, 30):
                    self._compare(start, action, value, (Obstacle(1, 0.0, 0.0, "N"),))

    def test_generators_and_one_shot_iterators(self):
        obstacles = [Obstacle(1, 60.0, 60.0, "N"), Obstacle(2, 120.0, 40.0, "N")]
        expected = reference_safe_primitive_samples(Pose(30.0, 60.0, 0.0), "F", 30, iter(obstacles))
        actual = sc.safe_primitive_samples(Pose(30.0, 60.0, 0.0), "F", 30, iter(obstacles))
        self.assertEqual(actual is None, expected is None)
        for start, goal, obstacle_set in LATTICE_CASES:
            from_list = state_lattice.plan_lattice_path(start, goal, list(obstacle_set), CONFIG)
            from_generator = state_lattice.plan_lattice_path(
                start, goal, (o for o in obstacle_set), CONFIG)
            self.assertEqual(repr(from_list), repr(from_generator))


class FallbackDomain(unittest.TestCase):
    def test_out_of_domain_inputs_take_the_public_path(self):
        check_fallback_domain(self)

    def test_non_dataclass_config_raises_the_same_type_error(self):
        config = types.SimpleNamespace(**dataclasses.asdict(CONFIG))
        leg = sc._prepare_leg_geometry((), config)
        self.assertIsNone(leg.rects)
        with self.assertRaises(TypeError) as expected:
            dataclasses.replace(config, safety_margin_cm=1.5)
        for action in ACTIONS:
            with self.assertRaises(TypeError) as raised:
                sc._safe_primitive_samples_for_leg(Pose(100.0, 100.0, 0.0), action, 10, leg)
            self.assertEqual(str(raised.exception), str(expected.exception))
            with self.assertRaises(TypeError) as public:
                sc.safe_primitive_samples(Pose(100.0, 100.0, 0.0), action, 10, (), config)
            self.assertEqual(str(public.exception), str(expected.exception))

    def test_invalid_action_and_value_errors_are_unchanged(self):
        leg = sc._prepare_leg_geometry((Obstacle(1, 75.0, 55.0, "N"),), CONFIG)
        self.assertIsNotNone(leg.rects)
        start = Pose(60.0, 60.0, 0.4)
        for action, value in (("X", 10), ("F", 0), ("F", True), ("FL", 361), ("F", 1000), ("F", 2.5)):
            with self.assertRaises(Exception) as public:
                sc.safe_primitive_samples(start, action, value, leg.obstacles, CONFIG)
            with self.assertRaises(Exception) as hoisted:
                sc._safe_primitive_samples_for_leg(start, action, value, leg)
            self.assertIs(type(hoisted.exception), type(public.exception), (action, value))
            self.assertEqual(str(hoisted.exception), str(public.exception))

    def test_invalid_sweep_step_error_is_unchanged(self):
        config = PlannerConfig(sweep_max_point_step_cm=0.0)
        leg = sc._prepare_leg_geometry((Obstacle(1, 75.0, 55.0, "N"),), config)
        with self.assertRaises(sc.InvalidSweepStepError) as public:
            sc.safe_primitive_samples(Pose(60.0, 60.0, 0.4), "F", 10, leg.obstacles, config)
        with self.assertRaises(sc.InvalidSweepStepError) as hoisted:
            sc._safe_primitive_samples_for_leg(Pose(60.0, 60.0, 0.4), "F", 10, leg)
        self.assertEqual(str(hoisted.exception), str(public.exception))

    def test_property_backed_and_logging_obstacles_are_read_exactly_as_before(self):
        check_property_backed_reads(self)


class LegLifetimeAndImmutability(unittest.TestCase):
    def test_leg_carries_its_own_tuple_and_config_and_is_immutable(self):
        obstacles = (Obstacle(1, 75.0, 55.0, "N"),)
        leg = sc._prepare_leg_geometry(obstacles, CONFIG)
        self.assertIs(leg.obstacles, obstacles)
        self.assertIs(leg.config, CONFIG)
        self.assertIsInstance(leg.rects, tuple)
        for field in ("obstacles", "config", "rects"):
            with self.assertRaises(dataclasses.FrozenInstanceError):
                setattr(leg, field, None)
        self.assertFalse(hasattr(leg, "__dict__"))  # slots: no per-instance dict

    def test_no_module_level_state_survives_a_leg(self):
        before = {name: id(value) for name, value in vars(sc).items()}
        state_lattice.plan_lattice_path(*LATTICE_CASES[0], CONFIG)
        after = {name: id(value) for name, value in vars(sc).items()}
        self.assertEqual(before, after)

    def test_preparation_happens_once_per_leg(self):
        check_prepared_once_per_leg(self)

    def test_each_leg_gets_its_own_geometry(self):
        seen = []
        original = sc._prepare_leg_geometry

        def spy(obstacles, config):
            leg = original(obstacles, config)
            seen.append(leg)
            return leg

        with mock.patch.object(sc, "_prepare_leg_geometry", side_effect=spy):
            state_lattice.plan_lattice_path(*LATTICE_CASES[0], CONFIG)
            state_lattice.plan_lattice_path(*LATTICE_CASES[1], CONFIG)
        self.assertEqual(len(seen), 2)
        self.assertIsNot(seen[0], seen[1])
        self.assertIsNot(seen[0].obstacles, seen[1].obstacles)


class LatticeLegEquivalence(unittest.TestCase):
    def test_plans_are_repr_identical_with_the_reference_call(self):
        for start, goal, obstacles in LATTICE_CASES:
            optimised = state_lattice.plan_lattice_path(start, goal, obstacles, CONFIG)
            with reference_leg_planner() as reference_calls:
                reference = state_lattice.plan_lattice_path(start, goal, obstacles, CONFIG)
            self.assertGreater(reference_calls.call_count, 0)
            self.assertGreater(reference.expansions, 0)
            self.assertEqual(repr(optimised), repr(reference))

    def test_failure_plans_are_identical(self):
        limited = PlannerConfig(lattice_max_expansions=40)
        blocked = (Obstacle(1, 55.0, 20.0, "N"), Obstacle(2, 55.0, 40.0, "N"), Obstacle(3, 55.0, 60.0, "N"))
        cases = (
            (Pose(20.0, 10.0, math.pi / 2), Pose(150.0, 150.0, 0.0), (), limited),
            (Pose(20.0, 30.0, 0.0), Pose(90.0, 30.0, 0.0), blocked, limited),
            (Pose(1.0, 1.0, 0.0), Pose(100.0, 100.0, 0.0), (), CONFIG),
            (Pose(100.0, 100.0, 0.0), Pose(-50.0, 100.0, 0.0), (), CONFIG),
        )
        reasons = set()
        for start, goal, obstacles, config in cases:
            optimised = state_lattice.plan_lattice_path(start, goal, obstacles, config)
            with reference_leg_planner():
                reference = state_lattice.plan_lattice_path(start, goal, obstacles, config)
            self.assertFalse(optimised.is_valid)
            self.assertEqual(repr(optimised), repr(reference))
            reasons.add(optimised.reason)
        self.assertIn(state_lattice.REASON_EXPANSION_LIMIT, reasons)

    def test_fallback_domain_plans_are_identical(self):
        start, goal = Pose(20.0, 10.0, math.pi / 2), Pose(40.0, 70.0, 0.0)
        for name, (obstacles, config) in FallbackCases.build().items():
            config = dataclasses.replace(config, lattice_max_expansions=40)  # keep the reference run short
            outcomes = []
            public_calls = None
            for use_reference in (False, True):
                try:
                    if use_reference:
                        with reference_leg_planner():
                            plan = state_lattice.plan_lattice_path(start, goal, obstacles, config)
                    else:
                        with mock.patch.object(sc, "safe_primitive_samples",
                                               wraps=sc.safe_primitive_samples) as public:
                            plan = state_lattice.plan_lattice_path(start, goal, obstacles, config)
                        public_calls = public.call_count
                    outcomes.append(repr(plan))
                except Exception as error:  # noqa: BLE001 - the type and text must agree
                    outcomes.append((type(error), str(error)))
            self.assertEqual(outcomes[0], outcomes[1], name)
            if public_calls is not None:
                self.assertGreater(public_calls, 0, name)  # the fallback really ran


class MutationsAreDetected(unittest.TestCase):
    """Each checker must fail under the error it exists to catch."""

    def test_one_ulp_rectangle_nudge_is_detected(self):
        original = footprint.prepare_obstacle_rects

        def nudged(config, obstacles):
            rects = original(config, obstacles)
            if not rects:
                return rects
            left, right, bottom, top, min_x, max_x, min_y, max_y = rects[0]
            down, up = -math.inf, math.inf
            return ((math.nextafter(left, down), math.nextafter(right, up),
                     math.nextafter(bottom, down), math.nextafter(top, up),
                     math.nextafter(min_x, down), math.nextafter(max_x, up),
                     math.nextafter(min_y, down), math.nextafter(max_y, up)),) + rects[1:]

        self.assertEqual(_boundary_mismatches(self), 0)
        with mock.patch.object(footprint, "prepare_obstacle_rects", nudged):
            self.assertGreater(_boundary_mismatches(self), 0)
            with self.assertRaisesRegex(AssertionError, "boundary decisions differ"):
                check_boundaries(self)
            with self.assertRaisesRegex(AssertionError, "split rects differ"):
                check_split_equals_oracle(self)

    def test_preparation_per_primitive_is_detected(self):
        original = sc._safe_primitive_samples_for_leg

        def repreparing(start, action, value, leg):
            fresh = sc._prepare_leg_geometry(leg.obstacles, leg.config)
            return original(start, action, value, fresh)

        check_prepared_once_per_leg(self)
        with mock.patch.object(sc, "_safe_primitive_samples_for_leg", repreparing):
            with self.assertRaisesRegex(AssertionError, "prepared exactly once"):
                check_prepared_once_per_leg(self)

    def test_calling_prepare_pose_check_per_primitive_is_detected(self):
        def falls_back_always(start, action, value, leg):
            return sc.safe_primitive_samples(start, action, value, leg.obstacles, leg.config)

        check_prepared_once_per_leg(self)
        with mock.patch.object(sc, "_safe_primitive_samples_for_leg", falls_back_always):
            with self.assertRaisesRegex(AssertionError, "public path used in the hoist domain"):
                check_prepared_once_per_leg(self)

    def test_ignoring_rects_none_for_out_of_domain_obstacles_is_detected(self):
        def ignores_domain(obstacles, config):
            rects = []
            for obstacle in obstacles:
                left, bottom = obstacle.x, obstacle.y
                right, top = left + config.obstacle_cm, bottom + config.obstacle_cm
                rects.append((left, right, bottom, top,
                              min(left, right), max(left, right), min(bottom, top), max(bottom, top)))
            return sc._LegCollisionGeometry(obstacles, config, tuple(rects))

        check_fallback_domain(self)
        check_property_backed_reads(self)
        with mock.patch.object(sc, "_prepare_leg_geometry", ignores_domain):
            with self.assertRaisesRegex(AssertionError, "rects must be None"):
                check_fallback_domain(self)
            # Behavioural, not only structural: values would be frozen at leg start.
            with self.assertRaisesRegex(AssertionError, "obstacle read at leg start"):
                check_property_backed_reads(self)


class HotspotWiring(unittest.TestCase):
    def test_new_functions_are_wrapped_and_restored(self):
        from benchmarks.hotspots import WRAPPED, Hotspots

        names = (("_prepare_leg_geometry", sc), ("_safe_primitive_samples_for_leg", sc),
                 ("prepare_primitive_check", footprint))
        for entry in names:
            self.assertIn(entry, WRAPPED)
        originals = [getattr(module, name) for name, module in names]
        hotspots = Hotspots()
        hotspots.install()
        try:
            for (name, module), original in zip(names, originals):
                self.assertIsNot(getattr(module, name), original)
            state_lattice.plan_lattice_path(*LATTICE_CASES[0], CONFIG)
        finally:
            hotspots.uninstall()
        for (name, module), original in zip(names, originals):
            self.assertIs(getattr(module, name), original)
        functions = hotspots.report()["functions"]
        self.assertEqual(functions["_prepare_leg_geometry"]["calls"], 1)
        self.assertGreater(functions["_safe_primitive_samples_for_leg"]["calls"], 0)
        self.assertEqual(functions["prepare_primitive_check"]["calls"],
                         functions["_safe_primitive_samples_for_leg"]["calls"])


if __name__ == "__main__":
    unittest.main()
