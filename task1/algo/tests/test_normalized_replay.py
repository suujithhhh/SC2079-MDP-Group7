"""Normalized route records and independent replay (`algo/benchmarks/normalized.py`,
`algo/benchmarks/route_replay.py`).

Real planning is limited to A_spread_01 (well under a second) and one real
A_spread_02 route (a few seconds) so a genuine two-leg pose chain is replayed.
Every corruption test starts from a clean real route, changes one thing, and
asserts the replay reports exactly the intended check.

Run from algo/:
    python -m unittest tests.test_normalized_replay -v
"""

import copy
import sys
import unittest
from pathlib import Path

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

import executable_route_optimizer as ero  # noqa: E402
import route_prefix_search as rps  # noqa: E402
from benchmarks import bench_executable_planner as bench  # noqa: E402
from benchmarks import fixtures as bfx  # noqa: E402
from benchmarks import normalized as nz  # noqa: E402
from benchmarks import route_replay as rr  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG as CFG  # noqa: E402
from models import Pose  # noqa: E402
from motion_primitives import primitive_length_cm, propagate  # noqa: E402
from stm_command_generation import STMCommandPlan  # noqa: E402

REVERSE = ("B", "BL", "BR")


def ero_norm(fixture, result, search="ero_b2", alternatives=False):
    return nz.normalize_ero_result(result, search=search, start_pose=fixture.start,
                                   obstacles=fixture.obstacles, alternatives=alternatives)


def prefix_norm(fixture, result, search="prefix_b2", alternatives=False):
    return nz.normalize_prefix_result(result, search=search, start_pose=fixture.start,
                                      alternatives=alternatives)


_REAL = {}


def real_routes() -> dict:
    """Plan the real routes once per process (the two-leg one takes a few seconds)."""
    if not _REAL:
        f1, f2 = bfx.load_fixture("A_spread_01"), bfx.load_fixture("A_spread_02")
        kw = dict(allow_alternative_viewing=False)
        two_leg = rps.find_prefix_b2_route(f2.start, f2.obstacles, **kw)
        _REAL.update(
            f1=f1, f2=f2,
            ero_b2=ero.find_executable_b2_route(f1.start, f1.obstacles, **kw),
            ero_b3=ero.optimise_executable_b3_route(f1.start, f1.obstacles, **kw),
            pb2=rps.find_prefix_b2_route(f1.start, f1.obstacles, **kw),
            pb3=rps.optimise_prefix_b3_route(f1.start, f1.obstacles, **kw),
            two_leg=two_leg, clean2=prefix_norm(f2, two_leg),
        )
    return _REAL


class RealRoutes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        for name, value in real_routes().items():
            setattr(cls, name, value)

    def corrupted(self):
        return copy.deepcopy(self.clean2)


class Mappings(RealRoutes):
    def test_hashes_match_the_runner_route_hash_and_each_other(self):
        n = ero_norm(self.f1, self.ero_b2)
        self.assertEqual(n["hash"], bench.route_hash(self.ero_b2))
        self.assertEqual(ero_norm(self.f1, self.ero_b3, "ero_b3")["hash"], bench.route_hash(self.ero_b3))
        for prefix in (self.pb2, self.pb3):
            self.assertEqual(prefix_norm(self.f1, prefix)["hash"], n["hash"])

    def test_ero_candidate_indices_are_derived_and_prefix_ones_copied(self):
        self.assertEqual(ero_norm(self.f1, self.ero_b2)["candidate_indices"], [0])
        self.assertEqual(prefix_norm(self.f1, self.pb2)["candidate_indices"],
                         list(self.pb2.complete_route.candidate_indices))
        # A target that is not a safety-filtered candidate is derived as -1.
        result = ero.find_executable_b2_route(self.f1.start, self.f1.obstacles,
                                              allow_alternative_viewing=False)
        n = nz.normalize_ero_result(result, search="ero_b2", start_pose=self.f1.start,
                                    obstacles=self.f1.obstacles, alternatives=False)
        self.assertEqual(n["candidate_indices"], [0])
        alt = nz.normalize_ero_result(result, search="ero_b2", start_pose=self.f1.start,
                                      obstacles=(), alternatives=False)
        self.assertEqual(alt["candidate_indices"], [-1])

    def test_record_shape_and_json_round_trip(self):
        import json
        n = ero_norm(self.f1, self.ero_b2)
        self.assertEqual(set(n), {"search", "alternatives", "start_pose", "route_kind", "obstacle_order",
                                  "skipped_obstacle_ids", "candidate_indices", "legs", "total_cost",
                                  "total_distance_cm", "final_pose", "hash"})
        self.assertEqual(json.loads(json.dumps(n)), n)      # exact float round trip
        self.assertEqual(n["route_kind"], "complete")
        self.assertIn(n["legs"][0]["commands"][0]["stm_command"][:4], ("MOV,",))

    def test_partial_and_none_results_map_the_same_way_for_both_result_types(self):
        obstacles = self.f2.obstacles
        def fake(start, target, obs, config=CFG):
            if target in fake.fail:
                return STMCommandPlan(False, (), (), 0.0, 0.0, 0, None, "no_path")
            return STMCommandPlan(True, (), (), 10.0, 10.0, 0, Pose(target.x + 0.1, target.y, target.heading), None)

        second = obstacles[1]
        fake.fail = set(ero._candidate_targets(second, obstacles, CFG, True))
        old = ero.find_executable_b2_route(self.f2.start, obstacles, leg_planner=fake,
                                           allow_alternative_viewing=True)
        new = rps.find_prefix_b2_route(self.f2.start, obstacles, leg_planner=fake,
                                       allow_alternative_viewing=True)
        a = nz.normalize_ero_result(old, search="ero_b2", start_pose=self.f2.start,
                                    obstacles=obstacles, alternatives=True)
        b = nz.normalize_prefix_result(new, search="prefix_b2", start_pose=self.f2.start, alternatives=True)
        self.assertEqual((a["route_kind"], b["route_kind"]), ("partial", "partial"))
        self.assertEqual(a["skipped_obstacle_ids"], b["skipped_obstacle_ids"])
        self.assertEqual(a["hash"], b["hash"])
        none_old = ero.find_executable_b2_route(self.f2.start, obstacles, leg_planner=lambda *a, **k: STMCommandPlan(
            False, (), (), 0.0, 0.0, 0, None, "no_path"), allow_alternative_viewing=False)
        none_new = rps.find_prefix_b2_route(self.f2.start, obstacles, leg_planner=lambda *a, **k: STMCommandPlan(
            False, (), (), 0.0, 0.0, 0, None, "no_path"), allow_alternative_viewing=False)
        na = nz.normalize_ero_result(none_old, search="ero_b2", start_pose=self.f2.start,
                                     obstacles=obstacles, alternatives=False)
        nb = nz.normalize_prefix_result(none_new, search="prefix_b2", start_pose=self.f2.start, alternatives=False)
        self.assertEqual((na["route_kind"], nb["route_kind"]), ("none", "none"))
        self.assertEqual(na["hash"], nb["hash"])

    def test_raw_metadata_is_preserved_alongside(self):
        raw = nz.prefix_raw(self.pb3)
        self.assertEqual(raw["stop_reason"], self.pb3.stop_reason)
        self.assertEqual(raw["work"]["leg_calls"], self.pb3.work.leg_calls)
        self.assertIsNotNone(raw["b2_work"])
        self.assertEqual(nz.ero_raw(self.ero_b3)["valid_orders"], self.ero_b3.valid_orders)

    def test_wrong_search_name_is_rejected(self):
        with self.assertRaises(ValueError):
            nz.normalize_ero_result(self.ero_b2, search="prefix_b2", start_pose=self.f1.start,
                                    obstacles=self.f1.obstacles, alternatives=False)
        with self.assertRaises(ValueError):
            nz.normalize_prefix_result(self.pb2, search="ero_b2", start_pose=self.f1.start, alternatives=False)


class CleanReplay(RealRoutes):
    def test_real_routes_from_both_searches_replay_cleanly(self):
        for norm in (ero_norm(self.f1, self.ero_b2), ero_norm(self.f1, self.ero_b3, "ero_b3"),
                     prefix_norm(self.f1, self.pb2), prefix_norm(self.f1, self.pb3, "prefix_b3")):
            with self.subTest(search=norm["search"]):
                self.assertEqual(rr.replay_violations(norm, self.f1), [])

    def test_a_real_two_leg_route_replays_cleanly_across_its_pose_chain(self):
        self.assertEqual(len(self.clean2["legs"]), 2)
        self.assertTrue(all(leg["commands"] for leg in self.clean2["legs"]))
        self.assertEqual(rr.replay_violations(self.clean2, self.f2), [])

    def test_a_none_route_is_clean_and_a_malformed_none_route_is_not(self):
        none = dict(prefix_norm(self.f1, self.pb2), route_kind="none", legs=[], obstacle_order=[],
                    skipped_obstacle_ids=[], candidate_indices=[], final_pose=None, total_cost=0.0,
                    total_distance_cm=0.0)
        self.assertEqual(rr.replay_violations(none, self.f1), [])
        self.assertIn("structure", rr.codes(rr.replay_violations(dict(none, obstacle_order=[1]), self.f1)))

    def test_replay_accepts_only_normalized_dicts(self):
        for not_a_dict in (self.pb2, self.ero_b2, [1], None):
            with self.assertRaises(TypeError):
                rr.replay_violations(not_a_dict, self.f1)


class Corruption(RealRoutes):
    def only(self, codes, *expected):
        self.assertTrue(set(expected) <= codes, (expected, codes))

    def test_changed_value_and_alias_wire_string(self):
        n = self.corrupted()
        command = n["legs"][0]["commands"][0]
        command["value"] += 1
        codes = rr.codes(rr.replay_violations(n, self.f2))
        self.only(codes, "command.wire", "command.length", "chain.propagate")
        n = self.corrupted()
        n["legs"][0]["commands"][0]["stm_command"] = "MOVE," + n["legs"][0]["commands"][0]["stm_command"][4:]
        self.assertEqual(rr.codes(rr.replay_violations(n, self.f2)), {"command.wire"})

    def test_illegal_direction_value_and_unit(self):
        n = self.corrupted()
        n["legs"][0]["commands"][0]["direction"] = "FW"
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "command.legality")
        n = self.corrupted()
        n["legs"][0]["commands"][0]["value"] = 0
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "command.legality")
        n = self.corrupted()
        n["legs"][0]["commands"][0]["value"] = True
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "command.legality")
        n = self.corrupted()
        n["legs"][0]["commands"][0]["unit"] = "deg" if n["legs"][0]["commands"][0]["unit"] == "cm" else "cm"
        self.assertEqual(rr.codes(rr.replay_violations(n, self.f2)), {"command.unit"})

    def test_broken_pose_chains_are_exact(self):
        n = self.corrupted()       # between legs
        n["legs"][1]["commands"][0]["start_pose"][0] += 1e-9
        self.assertEqual(rr.codes(rr.replay_violations(n, self.f2)) & {"chain.leg_start"}, {"chain.leg_start"})
        n = self.corrupted()       # within a leg
        leg = next(l for l in n["legs"] if len(l["commands"]) > 1)
        leg["commands"][1]["start_pose"][1] += 1e-9
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "chain.command")

    def test_epsilon_end_pose_change(self):
        n = self.corrupted()
        n["legs"][0]["commands"][-1]["end_pose"][0] += 1e-3
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "chain.propagate", "chain.leg_final")
        n = self.corrupted()
        n["legs"][-1]["final_pose"][2] += 1e-3
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "chain.leg_final", "chain.route_final")
        n = self.corrupted()
        n["final_pose"][0] += 1e-9
        self.assertEqual(rr.codes(rr.replay_violations(n, self.f2)), {"chain.route_final"})

    def test_an_unsafe_swept_command_is_reported_as_a_collision(self):
        n = self.corrupted()
        command = n["legs"][0]["commands"][0]
        start = Pose(*command["start_pose"])
        end = propagate(start, "F", 400, CFG)          # a consistent but unsafe 4 m drive
        command.update(direction="F", value=400, unit="cm", stm_command="MOV,F,400",
                       length_cm=400.0, end_pose=[end.x, end.y, end.heading])
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "collision")

    def test_wrong_or_missing_candidate_and_far_target(self):
        n = self.corrupted()
        n["candidate_indices"][0] = -1
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "target.candidate")
        n = self.corrupted()
        n["candidate_indices"][0] = 3
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "target.candidate")
        n = self.corrupted()
        n["legs"][0]["target_pose"][0] += 50.0
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "target.tolerance", "target.candidate")

    def test_order_problems(self):
        n = self.corrupted()
        n["obstacle_order"] = [n["obstacle_order"][0]] * 2
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "order")
        n = self.corrupted()
        n["skipped_obstacle_ids"] = [1]
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "order")
        n = self.corrupted()
        n["legs"][0]["obstacle_id"] = 99
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "order")
        n = self.corrupted()
        del n["legs"]
        self.only(rr.codes(rr.replay_violations(n, self.f2)), "structure")

    def test_distance_and_cost_are_checked_independently(self):
        n = self.corrupted()                     # distance only
        n["legs"][0]["total_distance_cm"] += 1.0
        codes = rr.codes(rr.replay_violations(n, self.f2))
        self.assertIn("accounting.distance", codes)
        self.assertNotIn("accounting.cost", codes)
        n = self.corrupted()                     # cost only
        n["legs"][0]["total_cost"] += 1.0
        codes = rr.codes(rr.replay_violations(n, self.f2))
        self.assertIn("accounting.cost", codes)
        self.assertNotIn("accounting.distance", codes)
        n = self.corrupted()                     # route totals only
        n["total_cost"] += 1e-9
        self.assertEqual(rr.codes(rr.replay_violations(n, self.f2)), {"accounting.cost"})
        n = self.corrupted()
        n["total_distance_cm"] += 1e-9
        self.assertEqual(rr.codes(rr.replay_violations(n, self.f2)), {"accounting.distance"})


class ReverseAccounting(unittest.TestCase):
    """A synthetic leg with reverse commands: cost = distance x reverse_penalty on
    B/BL/BR only, so neither total can be validated with the other's formula."""

    COMMANDS = (("F", 10), ("B", 20), ("BL", 30), ("FR", 15), ("BR", 10))

    @classmethod
    def setUpClass(cls):
        cls.fixture = bfx.load_fixture("A_spread_01")

    def record(self, distance=None, cost=None):
        pose = self.fixture.start
        commands, dist, total = [], 0.0, 0.0
        for direction, value in self.COMMANDS:
            end = propagate(pose, direction, value, CFG)
            length = primitive_length_cm(direction, value, CFG)
            dist += length
            total += length * CFG.reverse_penalty if direction in REVERSE else length
            commands.append({
                "direction": direction, "value": value, "unit": "cm" if direction in ("F", "B") else "deg",
                "stm_command": f"MOV,{direction},{value}", "start_pose": nz.pose_list(pose),
                "end_pose": nz.pose_list(end), "length_cm": length})
            pose = end
        leg = {"obstacle_id": 1, "target_pose": nz.pose_list(pose), "final_pose": nz.pose_list(pose),
               "total_cost": total if cost is None else cost,
               "total_distance_cm": dist if distance is None else distance, "commands": commands}
        route_cost = 0.0 + leg["total_cost"]
        route_distance = 0.0 + leg["total_distance_cm"]
        return {"search": "prefix_b2", "alternatives": False, "start_pose": nz.pose_list(self.fixture.start),
                "route_kind": "complete", "obstacle_order": [1], "skipped_obstacle_ids": [],
                "candidate_indices": [0], "legs": [leg], "total_cost": route_cost,
                "total_distance_cm": route_distance, "final_pose": nz.pose_list(pose), "hash": ""}, dist, total

    def accounting(self, record):
        return {c for c in rr.codes(rr.replay_violations(record, self.fixture)) if c.startswith("accounting")}

    def test_distance_and_cost_differ_and_the_correct_record_passes(self):
        record, dist, total = self.record()
        self.assertGreater(total, 0)
        self.assertNotAlmostEqual(dist, total)
        self.assertEqual(self.accounting(record), set())

    def test_swapping_the_two_totals_is_caught_by_each_own_check(self):
        _, dist, total = self.record()
        swapped_distance, _, _ = self.record(distance=total)     # distance holding the COST value
        self.assertEqual(self.accounting(swapped_distance), {"accounting.distance"})
        swapped_cost, _, _ = self.record(cost=dist)              # cost holding the DISTANCE value
        self.assertEqual(self.accounting(swapped_cost), {"accounting.cost"})

    def test_only_reverse_commands_are_penalised(self):
        record, dist, total = self.record()
        forward_only = [c for c in self.COMMANDS if c[0] not in REVERSE]
        self.assertLess(len(forward_only), len(self.COMMANDS))
        self.assertAlmostEqual(total - dist,
                               sum(primitive_length_cm(d, v, CFG) * (CFG.reverse_penalty - 1.0)
                                   for d, v in self.COMMANDS if d in REVERSE))


class ReplayMutantsAreDetected(unittest.TestCase):
    """Each mutant disables one replay check by patching the module it uses; the
    corruption test that guards that check must then fail."""

    def survivors(self, case_class, test_names, **patches):
        import io
        from unittest import mock
        loader = unittest.defaultTestLoader
        suite = unittest.TestSuite(t for t in loader.loadTestsFromTestCase(case_class)
                                   if t._testMethodName in test_names)
        self.assertEqual(suite.countTestCases(), len(test_names))
        runner = unittest.TextTestRunner(stream=io.StringIO(), verbosity=0)
        if not patches:
            return runner.run(suite)
        with mock.patch.multiple(rr, **patches):
            return runner.run(suite)

    def test_collision_check_is_exercised(self):
        result = self.survivors(Corruption, {"test_an_unsafe_swept_command_is_reported_as_a_collision"},
                                safe_primitive_samples=lambda *a, **k: ("safe",))
        self.assertFalse(result.wasSuccessful())

    def test_reproduction_of_the_end_pose_is_exercised(self):
        result = self.survivors(Corruption, {"test_changed_value_and_alias_wire_string",
                                             "test_epsilon_end_pose_change"},
                                _poses_match_strict=lambda a, b: True)
        self.assertFalse(result.wasSuccessful())

    def test_goal_tolerance_check_is_exercised(self):
        result = self.survivors(Corruption, {"test_wrong_or_missing_candidate_and_far_target"},
                                _within_goal_tolerance=lambda pose, goal, config: True)
        self.assertFalse(result.wasSuccessful())

    def test_reverse_penalty_is_exercised(self):
        result = self.survivors(ReverseAccounting, {"test_distance_and_cost_differ_and_the_correct_record_passes"},
                                REVERSE=())
        self.assertFalse(result.wasSuccessful())

    def test_unmutated_guards_pass(self):
        result = self.survivors(Corruption, {"test_an_unsafe_swept_command_is_reported_as_a_collision",
                                             "test_epsilon_end_pose_change",
                                             "test_wrong_or_missing_candidate_and_far_target"})
        self.assertTrue(result.wasSuccessful())


if __name__ == "__main__":
    unittest.main()
