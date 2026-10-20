"""Prefix-search benchmark modes and runner integration
(`algo/benchmarks/bench_executable_planner.py`, `prefix_modes.py`).

The runner is invoked as a real subprocess. Only fast fixtures are measured
(A_spread_01 in well under a second; A_spread_02 for the budget and cancellation
cases, a few seconds each); timeout and refusal cases are bounded to seconds.
No campaign is run.

Run from algo/:
    python -m unittest tests.test_benchmark_prefix_modes -v
"""

import contextlib
import io
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

from benchmarks import bench_executable_planner as bench  # noqa: E402
from benchmarks import fixtures as bfx  # noqa: E402
from benchmarks import process_control as pc  # noqa: E402

SCRIPT = ALGO / "benchmarks" / "bench_executable_planner.py"
PREFIX_LIMIT_KEYS = ("max_leg_calls", "max_lattice_expansions", "max_nodes", "wall_clock_checkpoint_limit_s",
                     "cancel_after_legs")


def run_cli(*args, timeout=120):
    completed = subprocess.run([sys.executable, str(SCRIPT), *args], cwd=ALGO,
                               capture_output=True, text=True, timeout=timeout)
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    return completed.returncode, [json.loads(line) for line in lines], completed.stdout, completed.stderr


class Schema(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.code, cls.records, cls.stdout, cls.stderr = run_cli(
            "run", "--fixture", "A_spread_01", "--mode", "b2", "b3", "pb2", "pb3", "--timeout", "60")
        cls.by_mode = {r["mode"]: r for r in cls.records}

    def test_schema_v2_keys_are_exact_and_appended(self):
        self.assertEqual(self.code, 0, self.stderr)
        self.assertEqual(bench.SCHEMA_VERSION, 3)
        self.assertEqual(bench.RESULT_KEYS[-5:], ("limits", "validation", "cleanup", "case", "bounded"))
        self.assertEqual(len(self.records), 4)
        for record in self.records:
            self.assertEqual(tuple(record), bench.RESULT_KEYS)
            self.assertEqual(record["schema_version"], 3)
            self.assertIsNone(record["bounded"])          # schema 3 adds a key only bpb2/bpb3 fill
            self.assertEqual(record["status"], "complete")

    def test_identical_route_hash_across_old_and_prefix_searches(self):
        hashes = {r["result_hash"] for r in self.records}
        self.assertEqual(len(hashes), 1, hashes)
        for mode in ("b2", "b3", "pb2", "pb3"):
            self.assertEqual(self.by_mode[mode]["validation"]["normalized"]["hash"],
                             self.by_mode[mode]["result_hash"])

    def test_correctness_runs_carry_no_planner_limits_or_checkpoint(self):
        for mode in ("pb2", "pb3"):
            limits = self.by_mode[mode]["limits"]
            self.assertEqual(set(limits), {"max_leg_calls", "max_lattice_expansions", "max_nodes",
                                           "max_frontier_entries", "wall_clock_checkpoint_limit_s",
                                           "cancel_after_legs"})
            for key in PREFIX_LIMIT_KEYS:
                self.assertIsNone(limits[key], key)
            self.assertEqual(limits["max_frontier_entries"], 200_000)     # provisional memory guard only
        for mode in ("b2", "b3"):
            self.assertIsNone(self.by_mode[mode]["limits"])

    def test_validation_block_replay_and_metadata_are_clean(self):
        for mode in ("pb2", "pb3"):
            validation = self.by_mode[mode]["validation"]
            self.assertEqual(validation["replay_violations"], [])
            self.assertEqual(validation["invariant_violations"], [])
            self.assertEqual(validation["b2_replay_violations"], [] if mode == "pb3" else None)
            self.assertIsNone(validation["overshoot"])
            self.assertEqual(validation["raw"]["stop_reason"],
                             "first_complete_route" if mode == "pb2" else "exhausted")
        self.assertTrue(self.by_mode["pb3"]["validation"]["b3_not_worse_than_b2"])
        self.assertTrue(self.by_mode["pb3"]["validation"]["raw"]["optimality_proven"])
        self.assertFalse(self.by_mode["pb2"]["validation"]["raw"]["optimality_proven"])
        self.assertIsNone(self.by_mode["b2"]["validation"]["invariant_violations"])
        self.assertEqual(self.by_mode["b2"]["validation"]["replay_violations"], [])

    def test_timing_is_reported_separately(self):
        for record in self.records:
            timing = record["validation"]["timing"]
            self.assertGreaterEqual(timing["normalize_s"], 0)
            self.assertGreaterEqual(timing["replay_s"], 0)
            self.assertGreater(record["wall_time_s"], record["planning_time_s"])
            self.assertLess(timing["normalize_s"], record["planning_time_s"] + 1)

    def test_cleanup_block_is_verified_and_complete(self):
        for record in self.records:
            cleanup = record["cleanup"]
            self.assertTrue(cleanup["death_verified"])
            self.assertFalse(cleanup["killed"])
            self.assertIsNone(cleanup["timeout_kind"])
            self.assertEqual(cleanup["exit_code"], 0)
            self.assertTrue(cleanup["tempdir_removed"])
            self.assertIn(cleanup["containment"], ("job_object", "process_kill_fallback", "process_group"))
            self.assertIsNone(record["case"])

    def test_stats_and_analysis_reuse_the_existing_shapes(self):
        pb = self.by_mode["pb2"]
        self.assertEqual(pb["stats"]["algorithm"], "pb2")
        self.assertEqual(pb["stats"]["cache_misses"], pb["validation"]["raw"]["work"]["leg_calls"])
        self.assertIn("leg_planner_share_of_planning", pb["analysis"])
        self.assertIs(pb["alternatives"], False)


class Determinism(unittest.TestCase):
    def test_separate_invocations_give_identical_hash_and_work(self):
        runs = [run_cli("run", "--fixture", "A_spread_01", "--mode", "pb3", "--timeout", "60")[1][0]
                for _ in range(2)]
        self.assertEqual(runs[0]["result_hash"], runs[1]["result_hash"])
        self.assertEqual(runs[0]["validation"]["raw"]["work"], runs[1]["validation"]["raw"]["work"])
        self.assertEqual(runs[0]["result"], runs[1]["result"])

    def test_repeat_runs_separate_workers(self):
        _, records, _, stderr = run_cli("run", "--fixture", "A_spread_01", "--mode", "pb2",
                                        "--repeat", "2", "--timeout", "60")
        self.assertEqual(len(records), 2)
        self.assertIn("[2/2]", stderr)
        self.assertEqual(records[0]["result_hash"], records[1]["result_hash"])


class BudgetAndCancellation(unittest.TestCase):
    def test_cancellation_after_one_leg_is_honest(self):
        code, records, _, _ = run_cli("run", "--fixture", "A_spread_02", "--mode", "pb2",
                                      "--cancel-after-legs", "1", "--timeout", "60")
        self.assertEqual(code, 0)
        record = records[0]
        raw = record["validation"]["raw"]
        self.assertEqual(record["limits"]["cancel_after_legs"], 1)
        self.assertEqual(raw["stop_reason"], "cancelled")
        self.assertEqual(raw["work"]["leg_calls"], 1)
        self.assertNotIn("exists", raw["message"])
        self.assertFalse(raw["optimality_proven"])
        self.assertFalse(raw["complete_route_proven_impossible"])
        self.assertEqual(record["status"], "partial")
        self.assertEqual(record["validation"]["invariant_violations"], [])
        self.assertEqual(record["validation"]["replay_violations"], [])

    def test_leg_call_budget_stops_b3_but_keeps_the_b2_incumbent(self):
        code, records, _, _ = run_cli("run", "--fixture", "A_spread_02", "--mode", "pb3",
                                      "--max-leg-calls", "2", "--timeout", "90")
        self.assertEqual(code, 0)
        record = records[0]
        raw = record["validation"]["raw"]
        self.assertEqual(record["status"], "complete")
        self.assertEqual((raw["stop_reason"], raw["work_limit_hit"]), ("work_limit", "max_leg_calls"))
        self.assertFalse(raw["optimality_proven"])
        self.assertTrue(record["validation"]["b3_not_worse_than_b2"])
        self.assertEqual(record["validation"]["normalized"]["hash"], record["validation"]["b2_normalized"]["hash"])
        self.assertEqual(record["validation"]["invariant_violations"], [])
        self.assertEqual(record["validation"]["replay_violations"], [])
        self.assertEqual(record["validation"]["b2_replay_violations"], [])

    def test_no_safe_pose_is_proven_impossible_even_with_a_tiny_budget(self):
        code, records, _, _ = run_cli("run", "--fixture", "C_no_safe_pose_02", "--mode", "pb2",
                                      "--max-leg-calls", "1", "--timeout", "60")
        record = records[0]
        raw = record["validation"]["raw"]
        self.assertEqual(code, 0)
        self.assertTrue(raw["complete_route_proven_impossible"])
        self.assertEqual(raw["empty_candidate_obstacle_ids"], [2])
        self.assertIn(record["status"], ("partial", "no_route"))
        self.assertEqual(record["validation"]["invariant_violations"], [])


class B2SeedNormalization(unittest.TestCase):
    """B3's B2 seed is labelled from its own metadata, never assumed complete
    (regression: Quick invocation Q7, C_no_safe_pose_02, where the seed skips the
    obstacle that has no safe viewing candidate)."""

    def validation(self, fixture_id):
        import executable_route_optimizer as ero
        import route_prefix_search as rps
        from benchmarks import prefix_modes
        fixture = bfx.load_fixture(fixture_id)
        result = rps.optimise_prefix_b3_route(fixture.start, fixture.obstacles, allow_alternative_viewing=False)
        _status, _summary, _hash, validation = prefix_modes.prefix_validation(
            result, fixture, "pb3", False, {}, ero.RouteSearchStats())
        return fixture, result, validation

    def test_partial_seed_is_labelled_partial_and_replays_cleanly(self):
        from benchmarks import route_replay
        fixture, result, v = self.validation("C_no_safe_pose_02")
        self.assertEqual(v["normalized"]["route_kind"], "partial")
        self.assertEqual(v["b2_normalized"]["route_kind"], "partial")
        self.assertEqual(v["b2_normalized"]["skipped_obstacle_ids"], [2])
        self.assertEqual(v["b2_normalized"]["obstacle_order"], [1])
        self.assertEqual(route_replay.replay_violations(v["b2_normalized"], fixture), [])
        self.assertEqual(route_replay.replay_violations(v["normalized"], fixture), [])
        self.assertEqual(v["invariant_violations"], [])
        raw = v["raw"]
        self.assertEqual(raw["stop_reason"], "exhausted")
        self.assertTrue(raw["complete_route_proven_impossible"])
        self.assertFalse(raw["optimality_proven"])
        self.assertIn("No complete route exists", raw["message"])
        self.assertIsNone(result.complete_route)
        self.assertIsNone(v["b3_not_worse_than_b2"])                 # there is no complete route to compare

    def test_a_genuinely_complete_seed_stays_complete(self):
        from benchmarks import route_replay
        fixture, _result, v = self.validation("A_spread_01")
        self.assertEqual(v["b2_normalized"]["route_kind"], "complete")
        self.assertEqual(v["b2_normalized"]["skipped_obstacle_ids"], [])
        self.assertEqual(route_replay.replay_violations(v["b2_normalized"], fixture), [])

    def test_seed_kind_derivation(self):
        from benchmarks import prefix_modes
        from types import SimpleNamespace as NS
        fixture = NS(obstacles=(1, 2))
        self.assertEqual(prefix_modes.seed_route_kind(NS(skipped_obstacle_ids=(), obstacle_order=(1, 2)), fixture), "complete")
        self.assertEqual(prefix_modes.seed_route_kind(NS(skipped_obstacle_ids=(2,), obstacle_order=(1,)), fixture), "partial")
        self.assertEqual(prefix_modes.seed_route_kind(NS(skipped_obstacle_ids=(), obstacle_order=(1,)), fixture), "partial")

    def test_restoring_the_unconditional_complete_label_is_detected(self):
        import io
        from unittest import mock
        from benchmarks import prefix_modes
        def suite():
            return unittest.TestSuite([B2SeedNormalization("test_partial_seed_is_labelled_partial_and_replays_cleanly")])

        runner = unittest.TextTestRunner(stream=io.StringIO())
        with mock.patch.object(prefix_modes, "seed_route_kind", lambda route, fixture: "complete"):
            self.assertFalse(runner.run(suite()).wasSuccessful())
        self.assertTrue(runner.run(suite()).wasSuccessful())


class HarnessTimeoutIsNotPlannerMetadata(unittest.TestCase):
    def test_a_killed_worker_leaves_every_planner_and_validation_field_null(self):
        code, records, _, _ = run_cli("run", "--fixture", "A_spread_04", "--mode", "pb2", "--timeout", "3",
                                      "--progress-interval", "0.5")
        self.assertEqual(code, 0)
        record = records[0]
        self.assertEqual(record["status"], "timeout")
        self.assertNotEqual(record["status"], "no_route")
        self.assertEqual(record["cleanup"]["timeout_kind"], "harness_hard_timeout")
        self.assertTrue(record["cleanup"]["killed"])
        self.assertTrue(record["cleanup"]["death_verified"])
        self.assertTrue(record["cleanup"]["tempdir_removed"])
        for key in ("result", "result_hash", "planning_time_s", "validation", "limits", "stats"):
            self.assertIsNone(record[key], key)
        self.assertGreaterEqual(record["wall_time_s"], 3)
        self.assertLess(record["wall_time_s"], 15)
        self.assertIn("harness hard timeout", record["status_detail"])


class TimeoutRules(unittest.TestCase):
    def test_prefix_rule_table(self):
        cases = [
            ((1, None), 60), ((3, None), 60), ((4, None), None), ((8, None), None),
            ((4, 300), 300), ((8, 600), 600), ((8, 601), None), ((2, 0), None), ((2, -1), None),
            ((2, math.inf), None), ((2, math.nan), None),
        ]
        for args, expected in cases:
            with self.subTest(args=args):
                timeout, refusal = bench.resolve_prefix_timeout(*args)
                if expected is None:
                    self.assertIsNone(timeout)
                    self.assertTrue(refusal)
                else:
                    self.assertEqual(timeout, expected)
                    self.assertIsNone(refusal)

    def test_old_search_rules_are_unchanged(self):
        self.assertEqual(bench.resolve_timeout("b3", 4, None, False), (120.0, None))
        self.assertIsNone(bench.resolve_timeout("b3", 4, 121, False)[0])
        self.assertIsNone(bench.resolve_timeout("b3", 6, None, False)[0])
        self.assertEqual(bench.resolve_timeout("b3", 6, 120, True), (120, None))

    def test_prefix_run_at_four_obstacles_requires_an_explicit_timeout(self):
        code, records, _, _ = run_cli("run", "--fixture", "A_spread_04", "--mode", "pb2")
        self.assertEqual(code, 2)
        self.assertEqual(records[0]["status"], "refused")
        self.assertIn("explicit --timeout", records[0]["status_detail"])
        self.assertIsNone(records[0]["cleanup"])

    def test_prefix_flags_are_refused_for_the_old_search(self):
        code, records, _, _ = run_cli("run", "--fixture", "A_spread_01", "--mode", "b2",
                                      "--max-leg-calls", "3", "--timeout", "60")
        self.assertEqual(code, 2)
        self.assertEqual(records[0]["status"], "refused")
        self.assertIn("--max-leg-calls", records[0]["status_detail"])
        self.assertIsNone(records[0]["wall_time_s"])            # nothing was run


class CleanupFailureAborts(unittest.TestCase):
    def test_run_isolated_records_an_unverified_death_as_an_error(self):
        fixture = bfx.load_fixture("A_spread_01")
        options = {"timeout_s": 60.0, "allow_large_baseline": False, "profile": "none", "tracemalloc": False,
                   "leg_records": False, "profile_out": None, "progress_interval_s": 1.0}
        for key in bench.prefix_modes.LIMIT_KEYS:
            options[key] = None
        failure = pc.CleanupError("worker could not be verified dead",
                                  {"containment": "job_object", "timeout_kind": None, "killed": False,
                                   "death_verified": False, "exit_code": None, "job_closed": True,
                                   "tempdir_removed": None})
        with mock.patch.object(pc, "run_worker", side_effect=failure):
            record = bench.run_isolated(fixture, "pb2", False, options)
        self.assertEqual(record["status"], "error")
        self.assertIs(record["cleanup"]["death_verified"], False)
        self.assertTrue(record["cleanup"]["tempdir_removed"])
        self.assertIn("cleanup failed", record["status_detail"])

    def test_execute_plan_stops_at_once_on_an_unverified_death(self):
        fixture = bfx.load_fixture("A_spread_01")
        bad = {"status": "error", "cleanup": {"death_verified": False}}
        emitted = []
        with mock.patch.object(bench, "run_isolated", return_value=bad) as run:
            code = bench.execute_plan([(fixture, "pb2", False, {})] * 3, {}, emitted.append)
        self.assertEqual(code, bench.EXIT_ABORT)
        self.assertEqual(run.call_count, 1)
        self.assertEqual(len(emitted), 1)

    def test_exit_codes_for_errors_refusals_and_violations(self):
        fixture = bfx.load_fixture("A_spread_01")
        cases = [({"status": "error", "cleanup": {"death_verified": True}}, bench.EXIT_ERROR),
                 ({"status": "refused"}, bench.EXIT_REFUSED),
                 ({"status": "complete", "validation": {"replay_violations": ["chain.command: x"]}},
                  bench.EXIT_VIOLATIONS),
                 ({"status": "complete", "validation": {"invariant_violations": ["x"]}}, bench.EXIT_VIOLATIONS),
                 ({"status": "complete", "validation": {"raw": {"bound_violation_count": 1}}},
                  bench.EXIT_VIOLATIONS),
                 ({"status": "complete", "validation": {"replay_violations": [], "raw": {}}}, 0),
                 ({"status": "timeout"}, 0)]
        for record, expected in cases:
            with self.subTest(record=record):
                with mock.patch.object(bench, "run_isolated", return_value=record):
                    self.assertEqual(bench.execute_plan([(fixture, "pb2", False, {})], {}, lambda r: None), expected)


class CampaignCli(unittest.TestCase):
    def test_dry_run_prints_the_arithmetic_and_runs_nothing(self):
        code, records, _, stderr = run_cli("campaign", "quick", "--dry-run")
        self.assertEqual(code, 0)
        self.assertEqual(len(records), 1)
        self.assertEqual((records[0]["timeouts_s"], records[0]["invocations"], records[0]["allowance_s"],
                          records[0]["worst_case_s"], records[0]["cap_s"], records[0]["fits"]),
                         (470, 11, 165, 635, 720, True))
        self.assertNotIn("[1/", stderr)

    def test_every_standard_and_extended_session_dry_run_fits_its_cap(self):
        for name, expected in (("standard", (3255, 3450, 1845)), ("extended", (3075, 3105, 2850))):
            code, records, _, _ = run_cli("campaign", name, "--session", "all", "--dry-run")
            self.assertEqual(code, 0)
            self.assertEqual(tuple(r["worst_case_s"] for r in records), expected)
            self.assertTrue(all(r["fits"] and r["worst_case_s"] <= r["cap_s"] for r in records))

    def test_a_multi_session_campaign_needs_a_session(self):
        completed = subprocess.run([sys.executable, str(SCRIPT), "campaign", "standard", "--dry-run"], cwd=ALGO,
                                   capture_output=True, text=True, timeout=30)
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("several sessions", completed.stderr)

    def test_a_case_that_does_not_fit_the_budget_is_reported_missing_and_not_started(self):
        # In-process with a clean Git state: the start guard would otherwise (rightly) refuse a
        # developer's dirty working tree, and this test is about the budget, not about Git.
        clean = {"sha": "0123456789abcdef0123456789abcdef01234567", "dirty": False}
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "raw.jsonl"
            stdout, stderr = io.StringIO(), io.StringIO()
            with mock.patch.object(bench, "git_info", return_value=clean), mock.patch.object(bench, "_GIT", None),                     contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                code = bench.main(["campaign", "standard", "--session", "S3", "--out", str(out), "--budget-s", "0"])
            self.assertEqual(code, bench.EXIT_INCOMPLETE)
            self.assertEqual(stdout.getvalue().strip(), "")
            self.assertEqual(stderr.getvalue().count("MISSING"), 3)
            self.assertFalse(out.exists())

    def test_load_done_reads_recorded_invocations(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "raw.jsonl"
            out.write_text(json.dumps({"case": {"invocation": "Q1"}}) + "\n" + json.dumps({"case": None}) + "\n",
                           encoding="utf-8")
            self.assertEqual(bench._load_done(str(out)), frozenset({"Q1"}))
            self.assertEqual(bench._load_done(str(out) + ".missing"), frozenset())


if __name__ == "__main__":
    unittest.main()
