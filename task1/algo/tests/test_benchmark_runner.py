"""Benchmark runner (`algo/benchmarks/bench_executable_planner.py`).

Proves the JSON-lines schema is stable, repeated runs are deterministic,
timeouts are recorded outcomes rather than failures, the old permutation
search is refused where it is exponential, and the profiling modes never
change a planning result. The runner is invoked as a real subprocess. Only
fast fixtures are measured (A_spread_01 completes in well under a second);
timeout and refusal cases are bounded to a few seconds.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import json
import math
import subprocess
import sys
import unittest
from pathlib import Path

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

from benchmarks import bench_executable_planner as bench  # noqa: E402
from benchmarks.hotspots import Hotspots  # noqa: E402
import footprint  # noqa: E402

SCRIPT = ALGO / "benchmarks" / "bench_executable_planner.py"
TIME_FIELDS = {"wall_s", "elapsed_s", "leg_planner_s", "lattice_s", "postprocess_s"}
STATS_KEYS = {
    "algorithm", "finished", "wall_s", "orders_searched", "orders_completed", "candidates_attempted",
    "bound_prunes_pre_call", "bound_prunes_post_call", "invalid_leg_results", "incumbent_updates",
    "leg_requests", "cache_hits", "cache_misses", "unique_legs", "cache_hit_rate", "legs_by_depth",
    "leg_planner_s", "lattice_s", "postprocess_s", "separable_legs", "lattice_expansions", "outcomes",
}
ANALYSIS_KEYS = {
    "leg_planner_share_of_planning", "outcome_time_shares", "failed_or_limited_time_share",
    "lattice_time_share", "postprocess_time_share", "seconds_per_expansion", "heading_bins",
}


def run_cli(*args, timeout=120):
    completed = subprocess.run([sys.executable, str(SCRIPT), *args], cwd=ALGO,
                               capture_output=True, text=True, timeout=timeout)
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    return completed.returncode, [json.loads(line) for line in lines], completed.stdout, completed.stderr


def scrub(value):
    if isinstance(value, dict):
        return {k: scrub(v) for k, v in value.items() if k not in TIME_FIELDS}
    if isinstance(value, list):
        return [scrub(v) for v in value]
    return value


class OutputSchema(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.code, cls.records, cls.stdout, cls.stderr = run_cli(
            "run", "--fixture", "A_spread_01", "--mode", "b2", "b3", "legs", "--timeout", "60")

    def test_runs_succeed_and_stdout_is_json_lines_only(self):
        self.assertEqual(self.code, 0)
        self.assertEqual(len(self.records), 3)
        self.assertEqual(len(self.stdout.strip().splitlines()), 3)
        self.assertIn("[1/3]", self.stderr)     # progress goes to stderr, never stdout

    def test_every_record_has_exactly_the_documented_keys_in_order(self):
        for record in self.records:
            self.assertEqual(tuple(record), bench.RESULT_KEYS)
            self.assertEqual(record["schema_version"], bench.SCHEMA_VERSION)
            self.assertEqual(record["kind"], bench.KIND)
            self.assertIn(record["status"], bench.STATUSES)

    def test_record_contents(self):
        modes = {r["mode"]: r for r in self.records}
        self.assertEqual(set(modes), {"b2", "b3", "legs"})
        for mode, record in modes.items():
            with self.subTest(mode=mode):
                self.assertEqual(record["status"], "complete")
                self.assertEqual(record["fixture_id"], "A_spread_01")
                self.assertEqual(record["n_obstacles"], 1)
                self.assertEqual(set(record["stats"]), STATS_KEYS)
                self.assertEqual(set(record["analysis"]), ANALYSIS_KEYS)
                self.assertEqual(len(record["result_hash"]), 64)
                self.assertGreater(record["wall_time_s"], record["planning_time_s"])
                self.assertTrue(record["git"].keys() >= {"sha", "dirty"})
                self.assertTrue(record["machine"].keys() >= {"system", "machine", "cpu_count"})
                self.assertTrue(record["python"].keys() >= {"version", "implementation"})
                peak = record["peak_memory"]["peak_rss_bytes"]
                self.assertTrue(peak is None or peak > 0)
                self.assertIsNone(record["partial_stats"])
                self.assertIsNone(record["hotspots"])
        self.assertEqual(modes["b2"]["alternatives"], False)
        self.assertIsNone(modes["legs"]["alternatives"])
        self.assertEqual(modes["legs"]["result"]["chained_from"], "ideal_target_poses")
        self.assertEqual(modes["legs"]["result"]["legs_planned"], 1)
        self.assertTrue(modes["b2"]["result"]["is_complete"])

    def test_no_machine_specific_paths_or_hostnames_are_recorded(self):
        text = self.stdout
        self.assertNotIn(str(Path.home()), text)
        self.assertNotIn(str(ALGO), text)
        self.assertNotIn(str(ALGO).replace("\\", "/"), text)


class Determinism(unittest.TestCase):
    def test_repeated_runs_give_identical_results_hashes_and_counters(self):
        runs = []
        for _ in range(2):
            _, records, _, _ = run_cli("run", "--fixture", "A_spread_01", "--mode", "b3", "--timeout", "60")
            runs.append(records[0])
        first, second = runs
        self.assertEqual(first["result_hash"], second["result_hash"])
        self.assertEqual(first["result"], second["result"])
        self.assertEqual(scrub(first["stats"]), scrub(second["stats"]))


class Timeouts(unittest.TestCase):
    def test_timeout_is_a_recorded_outcome_not_a_failure(self):
        code, records, _, _ = run_cli("run", "--fixture", "A_spread_04", "--mode", "b3",
                                      "--timeout", "3", "--progress-interval", "0.5")
        self.assertEqual(code, 0)
        record = records[0]
        self.assertEqual(record["status"], "timeout")
        self.assertEqual(record["timeout_s"], 3)
        self.assertGreaterEqual(record["wall_time_s"], 3)
        self.assertLess(record["wall_time_s"], 15)       # the child was actually killed
        self.assertIsNone(record["result"])
        self.assertIsNone(record["planning_time_s"])
        partial = record["partial_stats"]
        self.assertIsInstance(partial, dict)
        self.assertEqual(partial["finished"], False)
        self.assertGreater(partial["wall_s"], 0)
        self.assertEqual(tuple(record), bench.RESULT_KEYS)


class Refusals(unittest.TestCase):
    def refused(self, *args):
        code, records, stdout, _ = run_cli("run", *args, timeout=30)
        self.assertEqual(code, 2)
        self.assertEqual(len(records), 1)
        record = records[0]
        self.assertEqual(record["status"], "refused")
        self.assertTrue(record["status_detail"])
        self.assertIsNone(record["planning_time_s"])
        self.assertIsNone(record["stats"])
        self.assertIsNone(record["wall_time_s"])         # nothing was run
        return record

    def test_old_search_at_six_or_more_obstacles_is_refused_without_the_flag(self):
        for fixture in ("A_spread_06", "A_spread_08", "B_clutter_08"):
            for mode in ("b2", "b3"):
                with self.subTest(fixture=fixture, mode=mode):
                    self.assertIn("exponential", self.refused("--fixture", fixture, "--mode", mode)["status_detail"])

    def test_flag_alone_or_a_long_timeout_is_still_refused(self):
        self.refused("--fixture", "A_spread_07", "--mode", "b3", "--allow-large-baseline")
        self.refused("--fixture", "A_spread_07", "--mode", "b3", "--allow-large-baseline", "--timeout", "300")
        self.refused("--fixture", "A_spread_07", "--mode", "b3", "--timeout", "60")

    def test_four_obstacle_runs_are_bounded(self):
        self.refused("--fixture", "A_spread_04", "--mode", "b3", "--timeout", "300")

    def test_flagged_large_baseline_runs_only_under_a_hard_timeout(self):
        code, records, _, _ = run_cli("run", "--fixture", "A_spread_06", "--mode", "b3",
                                      "--allow-large-baseline", "--timeout", "3")
        self.assertEqual(code, 0)
        self.assertEqual(records[0]["status"], "timeout")
        self.assertLess(records[0]["wall_time_s"], 15)

    def test_unknown_fixture_is_an_error_and_prints_no_records(self):
        completed = subprocess.run([sys.executable, str(SCRIPT), "run", "--fixture", "nope"], cwd=ALGO,
                                   capture_output=True, text=True, timeout=30)
        self.assertNotEqual(completed.returncode, 0)
        self.assertEqual(completed.stdout.strip(), "")
        self.assertIn("unknown fixture", completed.stderr)


class TimeoutRules(unittest.TestCase):
    def test_rule_table(self):
        cases = [
            # (mode, n, timeout, allow_large) -> effective timeout, or None when refused
            (("legs", 8, None, False), 120), (("legs", 8, 600, False), 600), (("legs", 1, 601, False), None),
            (("b3", 1, None, False), 180), (("b2", 3, 400, False), 400), (("b3", 3, 601, False), None),
            (("b3", 4, None, False), 120), (("b3", 4, 120, False), 120), (("b3", 4, 121, False), None),
            (("b2", 5, None, False), 120), (("b2", 5, 300, False), None),
            (("b3", 6, None, False), None), (("b3", 6, 60, False), None), (("b3", 6, None, True), None),
            (("b3", 6, 121, True), None), (("b3", 6, 120, True), 120), (("b2", 8, 30, True), 30),
            (("b3", 2, 0, False), None), (("b3", 2, -5, False), None),
            (("b3", 2, math.inf, False), None), (("b3", 2, math.nan, False), None),
        ]
        for args, expected in cases:
            with self.subTest(args=args):
                timeout, refusal = bench.resolve_timeout(*args)
                if expected is None:
                    self.assertIsNone(timeout)
                    self.assertTrue(refusal)
                else:
                    self.assertEqual(timeout, expected)
                    self.assertIsNone(refusal)


class ProfilingModesDoNotChangeResults(unittest.TestCase):
    def test_hotspots_and_cprofile_leave_the_result_unchanged(self):
        records = {}
        for profile in ("none", "hotspots", "cprofile"):
            _, out, _, _ = run_cli("run", "--fixture", "A_spread_01", "--mode", "b2",
                                   "--profile", profile, "--timeout", "60")
            records[profile] = out[0]
        hashes = {p: r["result_hash"] for p, r in records.items()}
        self.assertEqual(len(set(hashes.values())), 1, hashes)
        self.assertEqual(records["none"]["result"], records["hotspots"]["result"])
        self.assertIsNone(records["none"]["hotspots"])
        self.assertIsNone(records["none"]["cprofile_top"])
        functions = records["hotspots"]["hotspots"]["functions"]
        self.assertTrue({"pose_is_safe", "safe_primitive_samples", "footprint_hits_obstacle"} <= set(functions))
        self.assertGreater(functions["pose_is_safe"]["calls"], 0)
        goal = records["hotspots"]["hotspots"]["goal_tests"]
        self.assertGreater(goal["goal_tests"], 0)
        self.assertLessEqual(goal["heading_blocked"], goal["position_in_tolerance"])
        self.assertLessEqual(goal["position_in_tolerance"], goal["goal_tests"])
        top = records["cprofile"]["cprofile_top"]
        self.assertTrue(top["by_tottime"] and top["by_cumtime"])
        self.assertIn("hotspot_time_shares_of_leg_planner", records["hotspots"]["analysis"])

    def test_hotspots_install_and_uninstall_restore_the_original_functions(self):
        original = footprint.pose_is_safe
        hotspots = Hotspots()
        hotspots.install()
        try:
            self.assertIsNot(footprint.pose_is_safe, original)
        finally:
            hotspots.uninstall()
        self.assertIs(footprint.pose_is_safe, original)


if __name__ == "__main__":
    unittest.main()
