"""Standalone admissible remaining-route lower bound + Held-Karp table.

`route_lower_bound.py` is not connected to the live optimizer, the
protocol-v2 service or the server; these tests exercise it directly. They
are deterministic (no RNG) and make no wall-clock assertion: performance is
checked through closed-form entry/transition counts only. Exhaustive
brute-force comparisons are kept to n <= 5 groups (n <= 4 for real
geometry); 7- and 8-group tables are checked by construction, counts and
hand-computed queries.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import ast
import copy
import dataclasses
import inspect
import itertools
import math
import sys
import unittest
from pathlib import Path

ALGO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO_DIR))

import footprint  # noqa: E402
import motion_primitives as mp  # noqa: E402
import route_lower_bound as rlb  # noqa: E402
import targets  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
from models import Obstacle, Pose  # noqa: E402

EPS = rlb.LOWER_BOUND_EPSILON
INF = math.inf
DEFAULT = DEFAULT_PLANNER_CONFIG
ZERO_TOL = dataclasses.replace(
    DEFAULT, lattice_goal_position_tolerance_cm=0.0, lattice_goal_heading_tolerance_deg=0.0
)
RP_HALF = dataclasses.replace(DEFAULT, reverse_penalty=0.5)
RP_TWO = dataclasses.replace(DEFAULT, reverse_penalty=2.0)
REVERSE = frozenset({"B", "BL", "BR"})

# The mandated 12-action sweep set: 12 + 12**2 + 12**3 = 1,884 sequences.
SWEEP_ACTIONS = (
    ("F", 10), ("F", 25), ("B", 10), ("B", 25),
    ("FL", 45), ("FL", 90), ("FR", 45), ("FR", 90),
    ("BL", 45), ("BL", 90), ("BR", 45), ("BR", 90),
)


def _cost(action: str, value: int, config) -> float:
    length = mp.primitive_length_cm(action, value, config)
    return length * config.reverse_penalty if action in REVERSE else length


def _run(start: Pose, sequence, config):
    pose, cost = start, 0.0
    for action, value in sequence:
        pose = mp.propagate(pose, action, value, config)
        cost += _cost(action, value, config)
    return pose, cost


def _table(node_groups, group_count, edge):
    return rlb.build_held_karp_table(node_groups, group_count, edge)


def _brute_force(node_groups, group_count, edge, first):
    """{(mask, first_node): exact minimum route cost} over every permutation
    of every non-empty group subset and every candidate assignment."""
    group_nodes = [[v for v, g in enumerate(node_groups) if g == h] for h in range(group_count)]
    best = {}
    for size in range(1, group_count + 1):
        for subset in itertools.combinations(range(group_count), size):
            mask = sum(1 << g for g in subset)
            for order in itertools.permutations(subset):
                for choice in itertools.product(*(group_nodes[g] for g in order)):
                    cost = first[choice[0]]
                    for a, b in zip(choice, choice[1:]):
                        cost += edge[a][b]
                    key = (mask, choice[0])
                    if cost < best.get(key, INF):
                        best[key] = cost
    return best


def _fake_graph(n, k, salt=0):
    """Deterministic integer, asymmetric, non-negative fake graph."""
    node_groups = tuple(g for g in range(n) for _ in range(k))
    coords = [((g * 37 + i * 11 + salt) % 23, (g * 17 + i * 29 + 3 * salt) % 19)
              for g in range(n) for i in range(k)]
    size = len(node_groups)
    edge = tuple(
        tuple(abs(coords[v][0] - coords[w][0]) + abs(coords[v][1] - coords[w][1])
              + (v * 7 + w * 3 + salt) % 4 for w in range(size))
        for v in range(size)
    )
    firsts = (
        tuple(coords[v][0] + coords[v][1] for v in range(size)),
        tuple((v * 13 + salt) % 17 + 1 for v in range(size)),
    )
    return node_groups, edge, firsts


def _mask_groups(mask):
    return [g for g in range(8) if mask & (1 << g)]


class ModelFromConfig(unittest.TestCase):
    def test_default_constants(self):
        model = rlb.lower_bound_model_from_config(DEFAULT)
        self.assertEqual(model.straight_cost_per_cm, 1.0)
        self.assertEqual(model.ccw_cost_per_rad, 24.8)                    # FL beats BL*1.08
        self.assertAlmostEqual(model.cw_cost_per_rad, 24.5 * 1.08, places=12)  # BR*1.08 beats FR
        self.assertLess(model.cw_cost_per_rad, DEFAULT.turn_radius_fr_cm)
        self.assertEqual(model.position_tolerance_cm, 3.0 + rlb.POSE_SLACK_CM)
        self.assertEqual(model.heading_tolerance_rad, math.radians(5.0) + rlb.HEADING_SLACK_RAD)

    def test_reverse_penalty_variants(self):
        half = rlb.lower_bound_model_from_config(RP_HALF)
        self.assertEqual(half.straight_cost_per_cm, 0.5)
        self.assertAlmostEqual(half.ccw_cost_per_rad, 33.6 * 0.5)
        self.assertAlmostEqual(half.cw_cost_per_rad, 24.5 * 0.5)
        two = rlb.lower_bound_model_from_config(RP_TWO)
        self.assertEqual(two.straight_cost_per_cm, 1.0)
        self.assertEqual(two.ccw_cost_per_rad, 24.8)
        self.assertEqual(two.cw_cost_per_rad, 33.7)

    def test_zero_tolerance_keeps_slacks(self):
        model = rlb.lower_bound_model_from_config(ZERO_TOL)
        self.assertEqual(model.position_tolerance_cm, rlb.POSE_SLACK_CM)
        self.assertEqual(model.heading_tolerance_rad, rlb.HEADING_SLACK_RAD)

    def test_invalid_configs_raise(self):
        bad = [
            dataclasses.replace(DEFAULT, reverse_penalty=0.0),
            dataclasses.replace(DEFAULT, reverse_penalty=-1.0),
            dataclasses.replace(DEFAULT, reverse_penalty=True),
            dataclasses.replace(DEFAULT, reverse_penalty=math.inf),
            dataclasses.replace(DEFAULT, turn_radius_br_cm=0.0),
            dataclasses.replace(DEFAULT, turn_radius_fl_cm=math.nan),
            dataclasses.replace(DEFAULT, lattice_goal_position_tolerance_cm=-1.0),
            dataclasses.replace(DEFAULT, lattice_goal_heading_tolerance_deg=math.nan),
        ]
        for config in bad:
            with self.subTest(config=config), self.assertRaises(rlb.InvalidLowerBoundInputError):
                rlb.lower_bound_model_from_config(config)
        with self.assertRaises(rlb.InvalidLowerBoundInputError):
            rlb.lower_bound_model_from_config("not a config")
        with self.assertRaises(rlb.InvalidLowerBoundInputError):
            rlb.LowerBoundModel(1.0, 0.1, 0.0, 24.8, 26.46)


class EdgeComponents(unittest.TestCase):
    MODEL = rlb.lower_bound_model_from_config(DEFAULT)
    ZERO = rlb.lower_bound_model_from_config(ZERO_TOL)

    def test_region_to_region_subtracts_two_tolerance_ends(self):
        # Default goal tolerance T: a real F leg of 20 - 2T cm starts inside
        # the source region (T cm from it) and ends inside the target region.
        tol = DEFAULT.lattice_goal_position_tolerance_cm
        length = round(20.0 - 2 * tol)
        source, target = Pose(0.0, 0.0, 0.0), Pose(20.0, 0.0, 0.0)
        bound = rlb.region_to_region_lower_bound(source, target, self.MODEL)
        self.assertAlmostEqual(bound, 20.0 - 2 * tol - 2 * rlb.POSE_SLACK_CM, places=12)
        leg_start = Pose(tol, 0.0, 0.0)
        leg_end = mp.propagate(leg_start, "F", length, DEFAULT)
        self.assertLessEqual(math.hypot(leg_end.x - target.x, leg_end.y - target.y), tol)
        self.assertLessEqual(bound, _cost("F", length, DEFAULT))
        # One tolerance end would exceed that real leg's cost: inadmissible.
        one_end = rlb.positional_lower_bound(source, target, self.MODEL, 1)
        self.assertGreater(one_end, _cost("F", length, DEFAULT))

    def test_heading_wrap_uses_cheapest_direction(self):
        start, target = Pose(0.0, 0.0, 0.0), Pose(0.0, 0.0, math.radians(-10.0))
        heading = rlb.heading_lower_bound(start, target, self.ZERO, 1)
        expected = (24.5 * 1.08) * (math.radians(10.0) - rlb.HEADING_SLACK_RAD)
        self.assertAlmostEqual(heading, expected, places=9)
        self.assertLess(heading, 5.0)  # the CCW 350 deg way would be ~151
        for action in ("FR", "BR"):
            end, cost = _run(start, [(action, 10)], ZERO_TOL)
            self.assertLessEqual(rlb.pose_to_region_lower_bound(start, end, self.ZERO), cost)

    def test_heading_tolerance_of_pi_or_more_is_zero(self):
        model = rlb.lower_bound_model_from_config(
            dataclasses.replace(DEFAULT, lattice_goal_heading_tolerance_deg=180.0))
        for degrees in range(0, 360, 15):
            b = Pose(0.0, 0.0, math.radians(degrees))
            self.assertEqual(rlb.heading_lower_bound(Pose(0, 0, 0.0), b, model, 1), 0.0)

    def test_start_inside_goal_tolerance_is_zero(self):
        start = Pose(2.0, 2.0, math.radians(3.0))
        self.assertEqual(rlb.pose_to_region_lower_bound(start, Pose(0.0, 0.0, 0.0), self.MODEL), 0.0)

    def test_identical_poses_are_zero(self):
        pose = Pose(50.0, 60.0, 1.0)
        self.assertEqual(rlb.pose_to_region_lower_bound(pose, pose, self.ZERO), 0.0)
        self.assertEqual(rlb.region_to_region_lower_bound(pose, pose, self.ZERO), 0.0)


class RealPrimitiveGuards(unittest.TestCase):
    """Zero goal tolerance (slacks still apply): tight cases that fail if the
    bound is summed, uses the wrong radius or ignores reverse_penalty < 1."""

    def test_fl_90_bound_is_tight_and_sum_would_overestimate(self):
        model = rlb.lower_bound_model_from_config(ZERO_TOL)
        start = Pose(0.0, 0.0, 0.0)
        end, cost = _run(start, [("FL", 90)], ZERO_TOL)
        self.assertAlmostEqual(cost, 24.8 * math.pi / 2, places=9)
        bound = rlb.pose_to_region_lower_bound(start, end, model)
        self.assertLessEqual(bound, cost)
        self.assertLess(cost - bound, 1e-3)
        p = rlb.positional_lower_bound(start, end, model, 1)
        h = rlb.heading_lower_bound(start, end, model, 1)
        self.assertEqual(bound, max(p, h))
        self.assertGreater(p + h, cost)

    def test_br_90_uses_br_radius_with_reverse_penalty(self):
        model = rlb.lower_bound_model_from_config(ZERO_TOL)
        start = Pose(0.0, 0.0, 0.0)
        end, cost = _run(start, [("BR", 90)], ZERO_TOL)
        self.assertAlmostEqual(cost, 24.5 * 1.08 * math.pi / 2, places=9)
        bound = rlb.pose_to_region_lower_bound(start, end, model)
        self.assertLessEqual(bound, cost)
        self.assertLess(cost - bound, 1e-3)
        fr_rate = ZERO_TOL.turn_radius_fr_cm * (math.pi / 2 - rlb.HEADING_SLACK_RAD)
        self.assertGreater(fr_rate, cost)  # an FR-only CW rate would be inadmissible

    def test_reverse_penalty_below_one_scales_straights(self):
        config = dataclasses.replace(ZERO_TOL, reverse_penalty=0.5)
        model = rlb.lower_bound_model_from_config(config)
        start = Pose(100.0, 100.0, 0.0)
        end, cost = _run(start, [("B", 50)], config)
        self.assertEqual(cost, 25.0)
        bound = rlb.pose_to_region_lower_bound(start, end, model)
        self.assertLessEqual(bound, cost)
        self.assertGreater((50.0 - rlb.POSE_SLACK_CM) * 1.0, cost)  # m = 1 would be inadmissible


class PrimitiveSweepAdmissibility(unittest.TestCase):
    STARTS = (Pose(100.0, 100.0, 0.0), Pose(37.5, 80.25, math.radians(123.0)))
    CONFIGS = (("default", DEFAULT), ("rp_0.5", RP_HALF), ("rp_2.0", RP_TWO), ("zero_tol", ZERO_TOL))

    def test_every_sequence_of_one_to_three_primitives(self):
        sequences = [seq for length in (1, 2, 3)
                     for seq in itertools.product(SWEEP_ACTIONS, repeat=length)]
        self.assertEqual(len(sequences), 1884)
        directions = [(math.cos(math.radians(45 * i)), math.sin(math.radians(45 * i)))
                      for i in range(8)]
        for start in self.STARTS:
            ends = [_run(start, seq, DEFAULT)[0] for seq in sequences]  # geometry is config-independent here
            for name, config in self.CONFIGS:
                model = rlb.lower_bound_model_from_config(config)
                tol = config.lattice_goal_position_tolerance_cm
                tol_h = math.radians(config.lattice_goal_heading_tolerance_deg)
                costs = [sum(_cost(a, v, config) for a, v in seq) for seq in sequences]
                worst = -INF
                for seq, end, cost in zip(sequences, ends, costs):
                    values = [rlb.pose_to_region_lower_bound(start, end, model)]
                    for i, (ux, uy) in enumerate(directions):
                        sign = 1.0 if i % 2 == 0 else -1.0
                        target = Pose(end.x + tol * ux, end.y + tol * uy, end.heading + sign * tol_h)
                        source = Pose(start.x - tol * uy, start.y + tol * ux, start.heading - sign * tol_h)
                        values.append(rlb.pose_to_region_lower_bound(start, target, model))
                        values.append(rlb.region_to_region_lower_bound(source, target, model))
                    for value in values:
                        self.assertGreaterEqual(value, 0.0)
                    worst = max(worst, max(values) - cost)
                with self.subTest(start=start, config=name):
                    self.assertLessEqual(worst, 0.0)


class HeldKarpHandMatrices(unittest.TestCase):
    def test_three_groups_beats_greedy(self):
        # A=0, B=1, C=2. Order B, A, C costs 2 + 1 + 5 = 8; greedy (A, B, C) is 102.
        edge = ((0, 1, 5), (1, 0, 100), (5, 100, 0))
        table = _table((0, 1, 2), 3, edge)
        self.assertEqual(rlb.held_karp_completion_bound(table, [0, 1, 2], (1, 2, 10)), 8.0 - EPS)
        greedy = 1 + edge[0][1] + edge[1][2]
        self.assertEqual(greedy, 102)

    def test_alternative_candidate_is_used(self):
        # Group X: x1 (node 0, primary), x2 (node 1). Group Y: node 2.
        edge = ((0, 0, 50), (0, 0, 2), (50, 2, 0))
        table = _table((0, 0, 1), 2, edge)
        first = (1, 4, 30)
        self.assertEqual(rlb.held_karp_completion_bound(table, [0, 1], first), 6.0 - EPS)
        # Fixing the first leg to one candidate (the core's child equivalent):
        self.assertEqual(rlb.held_karp_completion_bound(table, [0, 1], (1, INF, INF)), 51.0 - EPS)
        self.assertEqual(rlb.held_karp_completion_bound(table, [0, 1], (INF, 4, INF)), 6.0 - EPS)
        self.assertNotEqual(6.0 - EPS, 6.0 - 2 * EPS)


class HeldKarpBruteForce(unittest.TestCase):
    """Exhaustive permutation x candidate enumeration, n <= 5 groups only."""

    def test_exact_edges_equal_brute_force_optimum(self):
        for n in range(1, 6):
            for k in (1, 2, 3):
                node_groups, edge, firsts = _fake_graph(n, k, salt=n + k)
                table = _table(node_groups, n, edge)
                for first in firsts:
                    brute = _brute_force(node_groups, n, edge, first)
                    for mask in range(1, 1 << n):
                        groups = _mask_groups(mask)
                        optimum = min(v for (m, _), v in brute.items() if m == mask)
                        with self.subTest(n=n, k=k, mask=mask):
                            self.assertEqual(
                                rlb.held_karp_completion_bound(table, groups, first),
                                max(0.0, optimum - EPS))
                        for w in range(len(node_groups)):
                            if not mask & (1 << node_groups[w]):
                                continue
                            only_w = tuple(first[w] if v == w else INF for v in range(len(first)))
                            self.assertEqual(
                                rlb.held_karp_completion_bound(table, groups, only_w),
                                max(0.0, brute[(mask, w)] - EPS))

    def test_lower_edges_never_exceed_exact_optimum(self):
        for n in range(1, 6):
            for k in (1, 2, 3):
                node_groups, edge, firsts = _fake_graph(n, k, salt=2 * n + k)
                lower_edge = tuple(tuple(max(0, e - 1) * 0.5 for e in row) for row in edge)
                table = _table(node_groups, n, lower_edge)
                for first in firsts:
                    lower_first = tuple(f * 0.75 for f in first)
                    brute = _brute_force(node_groups, n, edge, first)
                    for mask in range(1, 1 << n):
                        optimum = min(v for (m, _), v in brute.items() if m == mask)
                        bound = rlb.held_karp_completion_bound(table, _mask_groups(mask), lower_first)
                        self.assertLessEqual(bound, optimum)
                        self.assertGreaterEqual(bound, 0.0)


class RealGeometryBruteForce(unittest.TestCase):
    OBSTACLES = (
        Obstacle(1, 50, 50, "N"), Obstacle(2, 150, 60, "W"),
        Obstacle(3, 100, 150, "S"), Obstacle(4, 40, 140, "E"),
    )
    START = Pose(20.0, 10.0, math.pi / 2)

    def _candidates(self, obstacles):
        return {
            o.obstacle_id: [p for p in targets.viewing_targets(o, DEFAULT)
                            if footprint.pose_is_safe(p, obstacles, DEFAULT)]
            for o in obstacles
        }

    def test_held_karp_matches_brute_force_over_same_edges(self):
        for n in (3, 4):
            obstacles = self.OBSTACLES[:n]
            candidates = self._candidates(obstacles)
            self.assertTrue(all(candidates.values()))
            bound = rlb.build_remaining_route_bound(candidates, DEFAULT)
            model = bound.model
            ids = sorted(candidates)
            best_by_first = {}
            for order in itertools.permutations(ids):
                for choice in itertools.product(*(range(len(candidates[i])) for i in order)):
                    poses = [candidates[i][c] for i, c in zip(order, choice)]
                    cost = rlb.pose_to_region_lower_bound(self.START, poses[0], model)
                    for a, b in zip(poses, poses[1:]):
                        cost += rlb.region_to_region_lower_bound(a, b, model)
                    key = (order[0], choice[0])
                    best_by_first[key] = min(best_by_first.get(key, INF), cost)
            optimum = min(best_by_first.values())
            self.assertAlmostEqual(bound.lower_bound(self.START, ids), optimum - EPS, delta=1e-9)
            for (obstacle_id, index), value in best_by_first.items():
                child = bound.child_lower_bound(self.START, obstacle_id, index, ids)
                self.assertAlmostEqual(child, value - EPS, delta=1e-9)
                self.assertLessEqual(bound.lower_bound(self.START, ids), child)

    def test_lower_bound_is_min_of_children_and_epsilon_applied_once(self):
        candidates = self._candidates(self.OBSTACLES[:3])
        bound = rlb.build_remaining_route_bound(candidates, DEFAULT)
        remaining = [1, 2, 3]
        children = []
        for obstacle_id in remaining:
            group = bound.obstacle_ids.index(obstacle_id)
            mask_after = sum(1 << bound.obstacle_ids.index(i) for i in remaining if i != obstacle_id)
            for index, pose in enumerate(bound.candidates[group]):
                child = bound.child_lower_bound(self.START, obstacle_id, index, remaining)
                raw = (rlb.pose_to_region_lower_bound(self.START, pose, bound.model)
                       + bound.table.suffix[mask_after][bound.table.group_nodes[group][index]])
                self.assertEqual(child, max(0.0, raw - EPS))
                children.append(child)
        self.assertEqual(bound.lower_bound(self.START, remaining), min(children))


class SevenAndEightGroups(unittest.TestCase):
    def test_closed_form_entry_and_transition_counts(self):
        for n in range(0, 9):
            for k in (1, 4):
                node_groups = tuple(g for g in range(n) for _ in range(k))
                size = len(node_groups)
                table = _table(node_groups, n, [[0.0] * size for _ in range(size)])
                entries = n * k * 2 ** (n - 1) if n else 0
                transitions = n * k * k * (n - 1) * 2 ** (n - 2) if n >= 2 else 0
                with self.subTest(n=n, k=k):
                    self.assertEqual(table.entry_count, entries)
                    self.assertEqual(table.transition_count, transitions)
                    self.assertEqual(len(table.suffix), 1 << n)
        self.assertEqual(transitions, 57344)  # n = 8, K = 4

    def _line(self, n, alternative=None):
        # Group g sits at position g + 1 on a line; the start is at 0.
        positions = [g + 1 for g in range(n)]
        node_groups, points = [], []
        for g in range(n):
            node_groups.append(g)
            points.append(positions[g])
            if alternative is not None:
                node_groups.append(g)
                points.append(alternative(g, positions[g]))
        edge = tuple(tuple(abs(a - b) for b in points) for a in points)
        first = tuple(points)
        return _table(tuple(node_groups), n, edge), first

    def test_line_graph_hand_values(self):
        for n in (7, 8):
            table, first = self._line(n)
            self.assertEqual(rlb.held_karp_completion_bound(table, range(n), first), n - EPS)
        table, first = self._line(8)
        self.assertEqual(rlb.held_karp_completion_bound(table, [2, 5], first), 6 - EPS)
        self.assertEqual(rlb.held_karp_completion_bound(table, [0], first), 1 - EPS)
        self.assertEqual(rlb.held_karp_completion_bound(table, [], first), 0.0)

    def test_decoy_and_cheaper_alternative(self):
        table, first = self._line(8, alternative=lambda g, p: p + 100)
        self.assertEqual(rlb.held_karp_completion_bound(table, range(8), first), 8 - EPS)
        # Group 7's alternative sits at 2: every group is then covered by [1, 7].
        table, first = self._line(8, alternative=lambda g, p: 2 if g == 7 else p + 100)
        self.assertEqual(rlb.held_karp_completion_bound(table, range(8), first), 7 - EPS)

    def test_rebuild_and_repeat_are_identical_and_frozen(self):
        a, first = self._line(8, alternative=lambda g, p: p + 3)
        b, _ = self._line(8, alternative=lambda g, p: p + 3)
        self.assertEqual(a, b)
        values = {rlb.held_karp_completion_bound(a, range(8), first).hex() for _ in range(5)}
        self.assertEqual(len(values), 1)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            a.transition_count = 0


class EmptyAndSingle(unittest.TestCase):
    def test_empty_remaining_is_positive_zero(self):
        bound = rlb.build_remaining_route_bound({1: [Pose(50.0, 50.0, 0.0)]})
        value = bound.lower_bound(Pose(0.0, 0.0, 0.0), [])
        self.assertEqual(value, 0.0)
        self.assertEqual(math.copysign(1.0, value), 1.0)

    def test_zero_groups(self):
        bound = rlb.build_remaining_route_bound({})
        self.assertEqual(bound.table.suffix, ((),))
        self.assertEqual(bound.lower_bound(Pose(0.0, 0.0, 0.0), ()), 0.0)

    def test_one_group(self):
        target = Pose(100.0, 20.0, 0.0)
        bound = rlb.build_remaining_route_bound({7: (target,)}, ZERO_TOL)
        start = Pose(20.0, 20.0, 0.0)
        expected = 80.0 - rlb.POSE_SLACK_CM - EPS
        self.assertAlmostEqual(bound.lower_bound(start, [7]), expected, places=9)
        self.assertEqual(bound.child_lower_bound(start, 7, 0, [7]), bound.lower_bound(start, [7]))


class Infinity(unittest.TestCase):
    P = Pose(20.0, 20.0, 0.0)

    def test_empty_candidate_group(self):
        bound = rlb.build_remaining_route_bound({1: [Pose(60.0, 20.0, 0.0)], 2: []})
        self.assertEqual(bound.lower_bound(self.P, [1, 2]), INF)
        self.assertEqual(bound.lower_bound(self.P, [2]), INF)
        self.assertTrue(math.isfinite(bound.lower_bound(self.P, [1])))
        self.assertEqual(bound.child_lower_bound(self.P, 1, 0, [1, 2]), INF)

    def test_disconnected_groups(self):
        table = _table((0, 1), 2, ((0, INF), (INF, 0)))
        self.assertEqual(rlb.held_karp_completion_bound(table, [0, 1], (1, 1)), INF)
        self.assertEqual(rlb.held_karp_completion_bound(table, [0], (1, 1)), 1 - EPS)

    def test_single_infinite_edge_with_finite_alternative(self):
        table = _table((0, 1), 2, ((0, INF), (3, 0)))
        self.assertEqual(rlb.held_karp_completion_bound(table, [0, 1], (1, 2)), 5 - EPS)

    def test_nan_negative_and_negative_infinity_raise(self):
        for bad in (math.nan, -1.0, -INF, True, "1"):
            with self.subTest(bad=bad):
                with self.assertRaises(rlb.InvalidLowerBoundInputError):
                    _table((0, 1), 2, ((0, bad), (1, 0)))
                table = _table((0, 1), 2, ((0, 1), (1, 0)))
                with self.assertRaises(rlb.InvalidLowerBoundInputError):
                    rlb.held_karp_completion_bound(table, [0, 1], (bad, 1))

    def test_no_api_accepts_a_leg_outcome(self):
        forbidden = {"reason", "outcome", "status", "expansions", "leg", "leg_result", "plan"}
        callables = [
            rlb.build_remaining_route_bound, rlb.build_held_karp_table,
            rlb.held_karp_completion_bound, rlb.RemainingRouteBound.lower_bound,
            rlb.RemainingRouteBound.child_lower_bound, rlb.pose_to_region_lower_bound,
            rlb.region_to_region_lower_bound,
        ]
        for fn in callables:
            self.assertFalse(set(inspect.signature(fn).parameters) & forbidden, fn.__name__)


class EpsilonAndClamp(unittest.TestCase):
    TABLE = _table((0,), 1, ((0,),))

    def test_values(self):
        self.assertEqual(rlb.held_karp_completion_bound(self.TABLE, [0], (6.0,)), 6.0 - EPS)
        self.assertEqual(rlb.held_karp_completion_bound(self.TABLE, [0], (5e-7,)), 0.0)
        zero = rlb.held_karp_completion_bound(self.TABLE, [0], (0.0,))
        self.assertEqual(zero, 0.0)
        self.assertEqual(math.copysign(1.0, zero), 1.0)
        self.assertEqual(rlb.held_karp_completion_bound(self.TABLE, [0], (INF,)), INF)


class Determinism(unittest.TestCase):
    CANDIDATES = {
        3: [Pose(100.0, 120.0, -math.pi / 2), Pose(95.0, 120.0, -math.pi / 2)],
        1: [Pose(55.0, 80.0, -math.pi / 2)],
        2: [Pose(120.0, 65.0, 0.0), Pose(120.0, 60.0, 0.0), Pose(115.0, 65.0, 0.0)],
    }
    START = Pose(20.0, 10.0, math.pi / 2)

    def test_mapping_order_and_iterable_type_do_not_change_a_bit(self):
        items = list(self.CANDIDATES.items())
        orders = [dict(items), dict(reversed(items)), dict(items[1:] + items[:1])]
        bounds = [rlb.build_remaining_route_bound(c) for c in orders]
        self.assertEqual(bounds[0].obstacle_ids, (1, 2, 3))
        for other in bounds[1:]:
            self.assertEqual(other.table, bounds[0].table)
            self.assertEqual(other.candidates, bounds[0].candidates)
        reference = bounds[0].lower_bound(self.START, [1, 2, 3]).hex()
        for bound in bounds:
            for remaining in ([3, 1, 2], (2, 3, 1), {1, 2, 3}, frozenset({3, 2, 1}),
                              (i for i in (2, 1, 3))):
                self.assertEqual(bound.lower_bound(self.START, remaining).hex(), reference)
            self.assertEqual(bound.child_lower_bound(self.START, 2, 1, {3, 2, 1}),
                             bounds[0].child_lower_bound(self.START, 2, 1, [1, 2, 3]))

    def test_candidate_order_preserved(self):
        bound = rlb.build_remaining_route_bound(self.CANDIDATES)
        self.assertEqual(bound.candidates[1], tuple(self.CANDIDATES[2]))
        self.assertEqual(bound.candidates[2], tuple(self.CANDIDATES[3]))


class ValidationAndImmutability(unittest.TestCase):
    CANDIDATES = Determinism.CANDIDATES
    P = Pose(20.0, 10.0, 0.0)

    def test_build_rejects_bad_input(self):
        bad_inputs = [
            [(1, [self.P])],
            {True: [self.P]},
            {"1": [self.P]},
            {i: [self.P] for i in range(9)},
            {1: [self.P, "pose"]},
            {1: [Pose(math.nan, 0.0, 0.0)]},
            {1: [Pose(0.0, 0.0, math.inf)]},
            {1: {self.P}},
            {1: self.P},
        ]
        for candidates in bad_inputs:
            with self.subTest(candidates=candidates), self.assertRaises(rlb.InvalidLowerBoundInputError):
                rlb.build_remaining_route_bound(candidates)
        with self.assertRaises(rlb.InvalidLowerBoundInputError):
            rlb.build_remaining_route_bound({1: [self.P]}, dataclasses.replace(DEFAULT, reverse_penalty=0))

    def test_query_rejections(self):
        bound = rlb.build_remaining_route_bound(self.CANDIDATES)
        bad_lower = [
            (self.P, [9]), (self.P, [1, 1]), (self.P, ["1"]), (self.P, [True]), (self.P, 5),
            ("pose", [1]), (Pose(math.inf, 0.0, 0.0), [1]),
        ]
        for pose, remaining in bad_lower:
            with self.subTest(remaining=remaining), self.assertRaises(rlb.InvalidLowerBoundInputError):
                bound.lower_bound(pose, remaining)
        bad_child = [
            (9, 0, [1, 2]),          # unknown obstacle
            (1, 0, [1, 1, 2]),       # duplicate remaining id
            (1, 0, [1, 9]),          # unknown remaining id
            (3, 0, [1, 2]),          # chosen obstacle not in remaining
            (2, -1, [1, 2]),         # index out of range
            (2, 3, [1, 2]),
            (2, True, [1, 2]),       # bool index
            (2, 1.0, [1, 2]),        # non-int index
        ]
        for obstacle_id, index, remaining in bad_child:
            with self.subTest(child=(obstacle_id, index, remaining)):
                with self.assertRaises(rlb.InvalidLowerBoundInputError):
                    bound.child_lower_bound(self.P, obstacle_id, index, remaining)
        with self.assertRaises(rlb.InvalidLowerBoundInputError):
            bound.child_lower_bound(Pose(0.0, math.nan, 0.0), 1, 0, [1])

    def test_core_and_edge_rejections(self):
        square = ((0, 1), (1, 0))
        bad_tables = [
            ((0, 1), 9, square), ((0, 1), -1, square), ((0, 1), True, square),
            ((1, 0), 2, square), ((0, 2), 2, square), ((0, 1), 2, ((0, 1),)),
            ((0, 1), 2, ((0, 1), (1,))), ((0, 1), 2, 5),
        ]
        for args in bad_tables:
            with self.subTest(args=args), self.assertRaises(rlb.InvalidLowerBoundInputError):
                rlb.build_held_karp_table(*args)
        table = _table((0, 1), 2, square)
        for remaining, first in (([0, 0], (1, 1)), ([2], (1, 1)), ([0], (1,)), ([True], (1, 1))):
            with self.assertRaises(rlb.InvalidLowerBoundInputError):
                rlb.held_karp_completion_bound(table, remaining, first)
        with self.assertRaises(rlb.InvalidLowerBoundInputError):
            rlb.held_karp_completion_bound("table", [0], (1, 1))
        model = rlb.lower_bound_model_from_config(DEFAULT)
        for ends in (0, 3, True, 1.0):
            with self.assertRaises(rlb.InvalidLowerBoundInputError):
                rlb.positional_lower_bound(self.P, self.P, model, ends)
        with self.assertRaises(rlb.InvalidLowerBoundInputError):
            rlb.pose_to_region_lower_bound(self.P, self.P, "model")

    def test_inputs_not_mutated_and_later_mutation_ignored(self):
        source = {k: list(v) for k, v in self.CANDIDATES.items()}
        snapshot = copy.deepcopy(source)
        bound = rlb.build_remaining_route_bound(source)
        self.assertEqual(source, snapshot)
        before = bound.lower_bound(self.P, [1, 2, 3])
        source[1].clear()
        source[2].append(Pose(0.0, 0.0, 0.0))
        self.assertEqual(bound.lower_bound(self.P, [1, 2, 3]), before)
        for obj, field in ((bound, "table"), (bound.table, "suffix"), (bound.model, "cw_cost_per_rad")):
            with self.assertRaises(dataclasses.FrozenInstanceError):
                setattr(obj, field, None)


class Isolation(unittest.TestCase):
    ALLOWED = {"math", "collections", "dataclasses", "typing", "config", "models"}

    @staticmethod
    def _imports(path: Path) -> set:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                names.add(node.module.split(".")[0])
        return names

    def test_module_imports_only_allowed(self):
        imports = self._imports(ALGO_DIR / "route_lower_bound.py")
        self.assertLessEqual(imports, self.ALLOWED)
        self.assertNotIn("time", imports)

    def test_live_modules_do_not_import_it(self):
        for name in ("executable_route_optimizer.py", "planning_service_v2.py", "server.py"):
            self.assertNotIn("route_lower_bound", self._imports(ALGO_DIR / name), name)


if __name__ == "__main__":
    unittest.main()
