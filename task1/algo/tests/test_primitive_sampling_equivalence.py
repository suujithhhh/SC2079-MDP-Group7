"""Differential tests: prepared uniform sampling vs the unchanged pose_at_fraction().

Oracles (never the optimized helper):
* `oracle_samples`: tuple(pose_at_fraction(pose, action, value, i / n, config)) with
  the UNCHANGED public motion_primitives.pose_at_fraction();
* `reference_sample_and_guard`: the pre-change swept_collision._sample_and_guard_cm
  body, verbatim, built only from unchanged functions.
Floats are compared bit-for-bit (struct), including NaN payload and sign.
Every randomised case uses a fixed seed.

Run from algo/:  python -m unittest tests.test_primitive_sampling_equivalence -v
"""

import dataclasses
import math
import random
import struct
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import footprint  # noqa: E402
import motion_primitives as mp  # noqa: E402
import state_lattice  # noqa: E402
import swept_collision as sc  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG, PlannerConfig  # noqa: E402
from models import Obstacle, Pose  # noqa: E402

CONFIG = DEFAULT_PLANNER_CONFIG
ACTIONS = ("F", "B", "FL", "FR", "BL", "BR")
STRAIGHT_VALUES = (1, 2, 10, 17, 55, 998, 999)
TURN_VALUES = (1, 2, 9, 10, 11, 45, 89, 90, 91, 179, 180, 181, 359, 360)
INTERVALS = (1, 2, 3, 7, 10, 11, 64, 1000)
SPECIAL_HEADINGS = (
    0.0, -0.0, math.pi, -math.pi, math.pi / 2, -math.pi / 2, 3 * math.pi, -5 * math.pi / 4,
    math.nextafter(math.pi, 0.0), math.nextafter(-math.pi, 0.0),
    math.nextafter(math.pi, math.inf), math.nextafter(-math.pi, -math.inf), 1e6, -1e-300,
)
CONFIGS = (
    CONFIG,
    PlannerConfig(turn_radius_fl_cm=0.1, turn_radius_fr_cm=1e4, turn_radius_bl_cm=25, turn_radius_br_cm=25),
    PlannerConfig(turn_radius_fl_cm=30.0, turn_radius_fr_cm=30.0, turn_radius_bl_cm=30.0, turn_radius_br_cm=30.0),
)


def oracle_samples(pose, action, value, n, config=CONFIG):
    return tuple(mp.pose_at_fraction(pose, action, value, i / n, config) for i in range(n + 1))


def reference_sample_and_guard(start, action, value, config=CONFIG):
    """Pre-change swept_collision._sample_and_guard_cm body, verbatim."""
    mp.pose_at_fraction(start, action, value, 0.0, config)
    sc._validate_sweep_step(config.sweep_max_point_step_cm)
    step_cm = config.sweep_max_point_step_cm

    if action in mp.STRAIGHT_ACTIONS:
        distance_cm = abs(float(value))
        interval_count = max(1, math.ceil(distance_cm / step_cm))
        interval_distance_cm = distance_cm / interval_count
        guard_cm = interval_distance_cm / 2.0
    else:
        theta_rad = math.radians(value)
        max_radius_cm = sc._max_margin_corner_radius(start, action, config)
        arc_bound_cm = max_radius_cm * theta_rad
        interval_count = max(1, math.ceil(arc_bound_cm / step_cm))
        interval_angle_rad = theta_rad / interval_count
        guard_cm = max_radius_cm * (interval_angle_rad / 2.0)

    fractions = [i / interval_count for i in range(interval_count + 1)]
    samples = tuple(
        mp.pose_at_fraction(start, action, value, f, config)
        for f in fractions
    )
    return samples, guard_cm


def reference_safe_primitive_samples(start, action, value, obstacles, config=CONFIG):
    """Reference sampling + reference (unprepared) collision check, pre-PR-19 style."""
    obstacles = tuple(obstacles)
    samples, guard_cm = reference_sample_and_guard(start, action, value, config)
    guarded = dataclasses.replace(config, safety_margin_cm=config.safety_margin_cm + guard_cm)
    if all(footprint.pose_is_safe(pose, obstacles, guarded) for pose in samples):
        return samples
    return None


def field_key(value):
    if type(value) is float:
        return ("float", struct.pack("<d", value))
    return (type(value).__name__, repr(value))


def bits(poses):
    return tuple((type(p), field_key(p.x), field_key(p.y), field_key(p.heading)) for p in poses)


def values_for(action):
    return STRAIGHT_VALUES if action in mp.STRAIGHT_ACTIONS else TURN_VALUES


class HelperMatchesPoseAtFraction(unittest.TestCase):
    def _check(self, pose, action, value, n, config=CONFIG):
        self.assertTrue(mp._uniform_sampling_supported(pose, action, value, config))
        expected = oracle_samples(pose, action, value, n, config)
        actual = mp._uniform_primitive_samples(pose, action, value, n, config)
        self.assertIs(type(actual), tuple)
        self.assertEqual(len(actual), n + 1)
        self.assertEqual(bits(actual), bits(expected), (pose, action, value, n))
        self.assertIsNot(actual[0], pose)  # the reference returns a new start Pose too

    def test_randomised_poses_actions_values_intervals_and_radii(self):
        rng = random.Random(20261001)
        for _ in range(1500):
            action = rng.choice(ACTIONS)
            pose = Pose(rng.uniform(-50, 250), rng.uniform(-50, 250),
                        rng.choice((rng.uniform(-4 * math.pi, 4 * math.pi), rng.choice(SPECIAL_HEADINGS))))
            self._check(pose, action, rng.choice(values_for(action)), rng.choice(INTERVALS[:7]),
                        rng.choice(CONFIGS))

    def test_every_action_value_and_boundary_heading(self):
        for heading in SPECIAL_HEADINGS:
            for action in ACTIONS:
                for value in values_for(action):
                    self._check(Pose(80.0, 90.0, heading), action, value, 11)

    def test_many_intervals_and_integer_pose_fields(self):
        for action in ACTIONS:
            self._check(Pose(50.0, 60.0, 0.3), action, 360 if action not in mp.STRAIGHT_ACTIONS else 999, 1000)
            self._check(Pose(50, 60, 0), action, 90 if action not in mp.STRAIGHT_ACTIONS else 20, 10)
            self._check(Pose(-3, 7, 2), action, 45 if action not in mp.STRAIGHT_ACTIONS else 7, 3)

    def test_last_sample_is_propagate_bitwise(self):
        rng = random.Random(7)
        for _ in range(300):
            action = rng.choice(ACTIONS)
            value = rng.choice(values_for(action))
            pose = Pose(rng.uniform(0, 200), rng.uniform(0, 200), rng.uniform(-math.pi, math.pi))
            samples = mp._uniform_primitive_samples(pose, action, value, rng.choice(INTERVALS[:7]), CONFIG)
            self.assertEqual(bits(samples[-1:]), bits((mp.propagate(pose, action, value, CONFIG),)))

    def test_nan_poses_match_bitwise(self):
        for pose in (Pose(math.nan, 5.0, 0.2), Pose(5.0, 5.0, math.nan), Pose(math.inf, 5.0, 0.2)):
            for action in ACTIONS:
                self._check(pose, action, 10, 10)

    def test_infinite_heading_raises_the_same_error(self):
        for action in ACTIONS:
            pose = Pose(5.0, 5.0, math.inf)
            with self.assertRaises(ValueError) as expected:
                oracle_samples(pose, action, 10, 10)
            with self.assertRaises(ValueError) as actual:
                mp._uniform_primitive_samples(pose, action, 10, 10, CONFIG)
            self.assertEqual(str(actual.exception), str(expected.exception))

    def test_interval_count_is_validated(self):
        for bad in (0, -1, True, 2.0, "3", None):
            with self.assertRaises(ValueError):
                mp._uniform_primitive_samples(Pose(1.0, 2.0, 0.0), "F", 10, bad, CONFIG)


class SampleAndGuardMatchesReference(unittest.TestCase):
    def _compare(self, start, action, value, config=CONFIG):
        expected = reference_sample_and_guard(start, action, value, config)
        actual = sc._sample_and_guard_cm(start, action, value, config)
        self.assertEqual(bits(actual[0]), bits(expected[0]), (start, action, value))
        self.assertEqual(field_key(actual[1]), field_key(expected[1]))

    def test_randomised_primitives_and_sweep_steps(self):
        rng = random.Random(4711)
        configs = CONFIGS + tuple(PlannerConfig(sweep_max_point_step_cm=s) for s in (0.01, 0.3, 7.5, 1e6))
        for _ in range(1200):
            action = rng.choice(ACTIONS)
            config = rng.choice(configs)
            value = rng.choice(values_for(action))
            if config.sweep_max_point_step_cm == 0.01 and value > 90:
                value = 90  # keep the tiny-step cases fast
            start = Pose(rng.uniform(0, 200), rng.uniform(0, 200),
                         rng.choice((rng.uniform(-math.pi, math.pi), rng.choice(SPECIAL_HEADINGS))))
            self._compare(start, action, value, config)

    def test_public_sample_primitive_matches(self):
        for action in ACTIONS:
            start = Pose(100.0, 100.0, 0.7)
            value = 90 if action not in mp.STRAIGHT_ACTIONS else 30
            self.assertEqual(bits(sc.sample_primitive(start, action, value, CONFIG)),
                             bits(reference_sample_and_guard(start, action, value)[0]))

    def test_invalid_inputs_raise_the_same_errors_in_the_same_order(self):
        bad_step = PlannerConfig(sweep_max_point_step_cm=0)
        cases = (
            (Pose(1.0, 1.0, 0.0), "FW", 10, CONFIG),
            (Pose(1.0, 1.0, 0.0), "F", 0, CONFIG),
            (Pose(1.0, 1.0, 0.0), "FL", 361, CONFIG),
            (Pose(1.0, 1.0, 0.0), "F", True, CONFIG),
            (Pose(1.0, 1.0, 0.0), "F", 10, bad_step),
            (Pose(1.0, 1.0, 0.0), "FW", 0, bad_step),   # action error before sweep-step error
            (Pose(1.0, 1.0, math.inf), "F", 10, CONFIG),
            (Pose(1.0, 1.0, math.inf), "FR", 10, CONFIG),
        )
        for start, action, value, config in cases:
            with self.assertRaises(Exception) as expected:
                reference_sample_and_guard(start, action, value, config)
            with self.assertRaises(Exception) as actual:
                sc._sample_and_guard_cm(start, action, value, config)
            self.assertIs(type(actual.exception), type(expected.exception), (action, value))
            self.assertEqual(str(actual.exception), str(expected.exception))


class HotPathCallCounts(unittest.TestCase):
    def test_one_validation_call_and_one_helper_call_per_primitive(self):
        for action, value in (("F", 10), ("B", 999), ("FL", 10), ("BR", 360)):
            with mock.patch.object(mp, "pose_at_fraction", wraps=mp.pose_at_fraction) as paf, \
                    mock.patch.object(mp, "_uniform_primitive_samples",
                                      wraps=mp._uniform_primitive_samples) as helper, \
                    mock.patch.object(mp, "_validate_fraction", wraps=mp._validate_fraction) as vf, \
                    mock.patch.object(mp, "_validate_value", wraps=mp._validate_value) as vv:
                samples = sc.sample_primitive(Pose(100.0, 100.0, 0.4), action, value, CONFIG)
            self.assertGreater(len(samples), 2)
            self.assertEqual(paf.call_count, 1)
            self.assertEqual(paf.call_args.args[3], 0.0)
            self.assertEqual(helper.call_count, 1)
            self.assertEqual(vf.call_count, 1)
            self.assertEqual(vv.call_count, 1)

    def test_action_validation_count_does_not_grow_with_sample_count(self):
        counts = []
        for value in (10, 360):
            with mock.patch.object(mp, "_validate_action", wraps=mp._validate_action) as va:
                samples = sc.sample_primitive(Pose(100.0, 100.0, 0.4), "FL", value, CONFIG)
            counts.append((len(samples), va.call_count))
        self.assertNotEqual(counts[0][0], counts[1][0])
        self.assertEqual(counts[0][1], counts[1][1])


# --- Fallback domain: original fraction-list + pose_at_fraction path ------------

ACCESS_LOG = []


@dataclasses.dataclass(frozen=True)
class TracedPose(Pose):
    def __getattribute__(self, name):
        if name in ("x", "y", "heading"):
            ACCESS_LOG.append(("pose", name))
        return object.__getattribute__(self, name)


@dataclasses.dataclass(frozen=True)
class TracedConfig(PlannerConfig):
    def __getattribute__(self, name):
        if not name.startswith("__"):
            ACCESS_LOG.append(("config", name))
        return object.__getattribute__(self, name)


class CountingFloat(float):
    def _log(self, op, result):
        ACCESS_LOG.append(("float", op))
        return result

    def __add__(self, other): return self._log("add", float(self) + other)
    def __radd__(self, other): return self._log("radd", other + float(self))
    def __sub__(self, other): return self._log("sub", float(self) - other)
    def __rsub__(self, other): return self._log("rsub", other - float(self))
    def __mul__(self, other): return self._log("mul", float(self) * other)
    def __rmul__(self, other): return self._log("rmul", other * float(self))


class ActionStr(str):
    pass


class ValueInt(int):
    pass


class FallbackPreservesOriginalBehaviour(unittest.TestCase):
    def _run_both(self, start, action, value, config):
        """Run reference then optimized; return (results, logs) or exceptions."""
        outcomes = []
        for function in (reference_sample_and_guard, sc._sample_and_guard_cm):
            ACCESS_LOG.clear()
            try:
                result = function(start, action, value, config)
                outcomes.append(("ok", bits(result[0]), field_key(result[1]), list(ACCESS_LOG)))
            except Exception as error:  # compared, never swallowed
                outcomes.append(("error", type(error), str(error), list(ACCESS_LOG)))
        ACCESS_LOG.clear()
        return outcomes

    def _assert_fallback(self, start, action, value, config):
        self.assertFalse(mp._uniform_sampling_supported(start, action, value, config))
        with mock.patch.object(mp, "_uniform_primitive_samples",
                               side_effect=AssertionError("prepared path used")):
            reference, optimized = self._run_both(start, action, value, config)
        self.assertEqual(optimized, reference)
        return reference

    def test_pose_subclass(self):
        for action in ACTIONS:
            ref = self._assert_fallback(TracedPose(60.0, 70.0, 0.9), action, 20, CONFIG)
            self.assertEqual(ref[0], "ok")
            self.assertTrue(ref[3])  # attribute reads were observed and matched

    def test_planner_config_subclass_and_duck_typed_config(self):
        traced = TracedConfig()
        duck = types.SimpleNamespace(**dataclasses.asdict(CONFIG))
        for config in (traced, duck):
            for action in ACTIONS:
                ref = self._assert_fallback(Pose(60.0, 70.0, 0.9), action, 20, config)
                self.assertEqual(ref[0], "ok")

    def test_str_and_int_subclasses(self):
        for action in ACTIONS:
            self._assert_fallback(Pose(60.0, 70.0, 0.9), ActionStr(action), 20, CONFIG)
            self._assert_fallback(Pose(60.0, 70.0, 0.9), action, ValueInt(20), CONFIG)

    def test_non_plain_pose_fields_and_turn_radius(self):
        for action in ACTIONS:
            ref = self._assert_fallback(Pose(CountingFloat(60.0), 70.0, 0.9), action, 20, CONFIG)
            self.assertIn(("float", "add"), ref[3])
            self._assert_fallback(Pose(60.0, 70.0, CountingFloat(0.9)), action, 20, CONFIG)
            self._assert_fallback(Pose(60.0, True, 0.9), action, 20, CONFIG)
        radius_config = PlannerConfig(turn_radius_fl_cm=CountingFloat(24.8))
        ref = self._assert_fallback(Pose(60.0, 70.0, 0.9), "FL", 30, radius_config)
        self.assertIn(("float", "rmul"), ref[3])
        # A straight action never reads the radius, so it stays on the prepared path.
        self.assertTrue(mp._uniform_sampling_supported(Pose(60.0, 70.0, 0.9), "F", 20, radius_config))

    def test_fallback_errors_match(self):
        for action in ACTIONS:
            ref = self._assert_fallback(TracedPose(60.0, 70.0, math.inf), action, 20, CONFIG)
            self.assertEqual(ref[:2], ("error", ValueError))
            ref = self._assert_fallback(Pose(60.0, 70.0, "north"), action, 20, CONFIG)
            self.assertEqual(ref[:2], ("error", TypeError))

    def test_non_dataclass_config_type_error_is_preserved(self):
        duck = types.SimpleNamespace(**dataclasses.asdict(CONFIG))
        with self.assertRaises(TypeError) as expected:
            dataclasses.replace(duck, safety_margin_cm=1.5)
        with self.assertRaises(TypeError) as actual:
            sc.safe_primitive_samples(Pose(100.0, 100.0, 0.0), "FL", 20, [], duck)
        self.assertEqual(str(actual.exception), str(expected.exception))


class SweptAndLatticeEquivalence(unittest.TestCase):
    def test_safe_primitive_samples_matches_full_reference(self):
        rng = random.Random(90210)
        outcomes = {True: 0, False: 0}
        for _ in range(700):
            action = rng.choice(ACTIONS)
            start = Pose(rng.uniform(15, 185), rng.uniform(15, 185), rng.uniform(-math.pi, math.pi))
            obstacles = tuple(Obstacle(i, rng.uniform(0, 190), rng.uniform(0, 190), "N")
                              for i in range(rng.randrange(0, 9)))
            value = rng.choice((10, 20, 45, 90) if action not in mp.STRAIGHT_ACTIONS else (10, 20, 60))
            expected = reference_safe_primitive_samples(start, action, value, obstacles)
            actual = sc.safe_primitive_samples(start, action, value, obstacles, CONFIG)
            self.assertEqual(actual is None, expected is None)
            if expected is not None:
                self.assertEqual(bits(actual), bits(expected))
            outcomes[expected is not None] += 1
        self.assertGreater(min(outcomes.values()), 100, outcomes)

    def test_lattice_plans_are_identical_with_the_reference_sampler(self):
        cases = (
            (Pose(20.0, 10.0, math.pi / 2), Pose(40.0, 70.0, 0.0), (Obstacle(1, 70.0, 60.0, "W"),)),
            (Pose(20.0, 10.0, math.pi / 2), Pose(60.0, 40.0, math.pi / 2),
             (Obstacle(1, 60.0, 90.0, "S"), Obstacle(2, 120.0, 20.0, "N"))),
        )
        for start, goal, obstacles in cases:
            optimised = state_lattice.plan_lattice_path(start, goal, obstacles, CONFIG)
            with mock.patch.object(sc, "_sample_and_guard_cm",
                                   wraps=reference_sample_and_guard) as reference_calls:
                reference = state_lattice.plan_lattice_path(start, goal, obstacles, CONFIG)
            self.assertGreater(reference_calls.call_count, 0)  # the patched reference really ran
            self.assertGreater(reference.expansions, 0)
            self.assertEqual(repr(optimised), repr(reference))


class HotspotWiring(unittest.TestCase):
    def test_hotspots_wrap_and_restore_the_prepared_sampler(self):
        from benchmarks.hotspots import WRAPPED, Hotspots

        self.assertIn(("_uniform_primitive_samples", mp), WRAPPED)
        original = mp._uniform_primitive_samples
        hotspots = Hotspots()
        hotspots.install()
        try:
            self.assertIsNot(mp._uniform_primitive_samples, original)
            sc.sample_primitive(Pose(100.0, 100.0, 0.0), "FL", 20, CONFIG)
        finally:
            hotspots.uninstall()
        self.assertIs(mp._uniform_primitive_samples, original)
        self.assertEqual(hotspots.report()["functions"]["_uniform_primitive_samples"]["calls"], 1)


if __name__ == "__main__":
    unittest.main()
