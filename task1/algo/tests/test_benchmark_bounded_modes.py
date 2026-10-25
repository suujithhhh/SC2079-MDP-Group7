"""Bounded-executor benchmark modes `bpb2`/`bpb3` (`algo/benchmarks/bounded_modes.py`),
their record schema (3) and their checks and gates in `algo/benchmarks/report.py`.

The executor, search and pipeline are used unchanged. Process tests use short,
test-only policies and the existing test worker (`tests/_bounded_test_worker.py`)
through the executor's `worker_command` seam; they are tooling tests, not evidence.
Gate tests use SYNTHETIC records built here.

Run from algo/:
    python -m unittest tests.test_benchmark_bounded_modes -v
"""

import copy
import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

import bounded_prefix_planning as bpp  # noqa: E402
import worker_containment as wc  # noqa: E402
from benchmarks import bench_executable_planner as bench  # noqa: E402
from benchmarks import bounded_modes as bm  # noqa: E402
from benchmarks import campaigns as cmp  # noqa: E402
from benchmarks import fixtures as bfx  # noqa: E402
from benchmarks import report as rep  # noqa: E402
from tests.test_prefix_validation_summary import SHA, rec, standard_records  # noqa: E402

GIB = 1024 ** 3
WINDOWS = sys.platform == "win32"
# Short test-only policies (never a campaign policy).
SCRIPT_POLICY = bpp.ExecutionPolicy(planning_target_s=2.5, hard_stop_s=3.0, verification_deadline_s=8.0,
                                    return_target_s=8.5, return_deadline_s=9.0, request_ceiling_s=9.5,
                                    min_search_budget_s=0.5)
REAL_POLICY = bpp.ExecutionPolicy(planning_target_s=4.0, hard_stop_s=6.0, verification_deadline_s=11.0,
                                  return_target_s=11.5, return_deadline_s=12.0, request_ceiling_s=12.5)


def _options(**kw):
    base = {"timeout_s": None, "allow_large_baseline": False, "profile": "none", "tracemalloc": False,
            "leg_records": False, "profile_out": None, "progress_interval_s": 1.0}
    for key in bench.prefix_modes.LIMIT_KEYS + bm.OPTION_KEYS:
        base[key] = None
    base.update(kw)
    return base


_PRODUCTION = bpp.PRODUCTION_POLICY
TARGET = _PRODUCTION.planning_target_s
HARD_STOP = _PRODUCTION.hard_stop_s
RETURN_DEADLINE = _PRODUCTION.return_deadline_s


class Options(unittest.TestCase):
    def test_policies(self):
        self.assertIs(bm.policy_for("production"), bpp.PRODUCTION_POLICY)
        stress = bm.policy_for("stress")
        self.assertEqual((stress.planning_target_s, stress.hard_stop_s, stress.verification_deadline_s,
                          stress.return_target_s, stress.return_deadline_s, stress.request_ceiling_s),
                         (5.0, 10.0, 15.0, 17.0, 18.0, 20.0))
        self.assertEqual(bm.policy_record("production")["hard_stop_s"], 66.0)
        with self.assertRaises(ValueError):
            bm.policy_for("fast")

    def test_refusals(self):
        self.assertIsNone(bm.option_refusal("bpb3", {}))
        self.assertIsNone(bm.option_refusal("pb3", {}))
        self.assertIn("bpb2/bpb3", bm.option_refusal("pb3", {"policy": "stress"}))
        self.assertIn("profiling", bm.option_refusal("bpb3", {"profile": "hotspots"}))
        self.assertIn("profiling", bm.option_refusal("bpb3", {"tracemalloc": True}))
        self.assertIn("unknown policy", bm.option_refusal("bpb3", {"policy": "fast"}))
        self.assertIn("unknown stall", bm.option_refusal("bpb3", {"stall": "forever"}))
        for bad in (0, 21, True, 2.0):
            self.assertIn("soak_plans", bm.option_refusal("bpb3", {"soak_plans": bad}))

    def test_timeouts(self):
        self.assertEqual(bm.resolve_timeout(None, "production", None), (90.0, None))
        self.assertIn("explicit", bm.resolve_timeout(None, "stress", 8)[1])
        self.assertEqual(bm.resolve_timeout(420, "stress", 8), (420.0, None))
        self.assertIn("request ceiling", bm.resolve_timeout(62, "production", None)[1])
        self.assertIn("request ceiling", bm.resolve_timeout(150, "stress", 8)[1])
        self.assertIn("maximum", bm.resolve_timeout(601, "production", None)[1])
        self.assertIn("finite", bm.resolve_timeout(float("inf"), "production", None)[1])

    def test_the_runner_refuses_before_starting_anything(self):
        fixture = bfx.load_fixture("A_spread_01")
        for mode, kw in (("bpb3", {"profile": "hotspots"}), ("bpb3", {"max_leg_calls": 3}),
                         ("pb3", {"policy": "stress"}), ("bpb3", {"soak_plans": 3}),
                         ("bpb3", {"timeout_s": 30.0})):
            record = bench.run_isolated(fixture, mode, False, _options(**kw))
            self.assertEqual(record["status"], "refused", (mode, kw))
            self.assertEqual(tuple(record), bench.RESULT_KEYS)
            self.assertIsNone(record["cleanup"])

    def test_campaign_bounded_cases_resolve(self):
        cases = [c for s in cmp.SESSIONS["bounded"] for c in s.cases]
        self.assertEqual([c.case_id for c in cases],
                         ["BQ1", "BQ2"] + [f"BR{i}" for i in range(1, 11)] + [f"BX{i}" for i in range(1, 18)]
                         + ["BS1", "BS2", "BS5"])
        for case in cases:
            with self.subTest(case=case.case_id):
                options = case.option_dict()
                self.assertIn(options["policy"], bm.POLICY_NAMES)
                self.assertIsNone(bm.option_refusal(case.mode, options))
                timeout, refusal = bm.resolve_timeout(float(case.timeout_s), options["policy"],
                                                      options.get("soak_plans"))
                self.assertIsNone(refusal)
                self.assertNotIn("T", case.accept)
        representatives = {c.case_id: c.fixture for c in cases if c.case_id in rep.BOUNDED_AVAILABILITY}
        self.assertEqual(sorted(set(representatives.values())), [f"A_spread_0{n}" for n in range(4, 9)])
        self.assertEqual({c.case_id: c.repeats for c in cases if c.repeats > 1},
                         {"BR1": 3, "BR3": 3, "BR5": 3, "BS1": 2})
        bs1, bs5 = cmp.all_cases()["BS1"].option_dict(), cmp.all_cases()["BS5"].option_dict()
        self.assertEqual(bs1, {"policy": "production", "stall": "before_final"})
        self.assertEqual(bs5, {"policy": "stress", "stall": "at_call_1", "soak_plans": 8})


class Capture(unittest.TestCase):
    def test_the_wrapper_passes_the_same_object_through_and_is_removed(self):
        original = wc.start_worker
        capture = bm.WorkerCapture()
        sentinel = object()
        wc.start_worker = lambda *a, **k: sentinel
        try:
            inner = wc.start_worker
            capture.install()
            self.assertIsNot(wc.start_worker, inner)
            self.assertIs(wc.start_worker(1, cwd=None, stderr=None), sentinel)
            self.assertEqual(capture.workers, [sentinel])
            capture.uninstall()
            self.assertIs(wc.start_worker, inner)
        finally:
            wc.start_worker = original


def _run_in_process(test, fixture_id, mode, config, policy, *, plans=None, stall_key=None):
    """`bounded_modes.run_worker` in this process with a short test policy and a
    test-worker command, through the `planner_factory` seam."""
    folder = tempfile.mkdtemp(prefix="bounded_test_")
    test.addCleanup(lambda: __import__("shutil").rmtree(folder, ignore_errors=True))
    config_path = Path(folder) / "worker.json"
    config_path.write_text(json.dumps(config), encoding="utf-8")
    kind = "script" if "steps" in config else "real"
    command = [sys.executable, str(bm.TEST_WORKER), kind, str(config_path)]
    spec = {"fixture_id": fixture_id, "mode": mode, "alternatives": False, "policy": "stress",
            "stall": stall_key, "soak_plans": plans, "private_tmp": str(Path(folder) / "tmp")}
    saved = bm.tempfile.tempdir
    test.addCleanup(setattr, bm.tempfile, "tempdir", saved)
    return bm.run_worker(spec, peak_rss=bench.peak_rss_bytes, redact=bench.redact,
                         planner_factory=lambda _p, _c: bpp.BoundedPrefixPlanner(policy, worker_command=command))


class InProcess(unittest.TestCase):
    STALL = {"steps": [{"op": "hello"}, {"op": "read_spec"}, {"op": "stall"}]}

    def test_a_stalled_worker_is_hard_stopped_and_every_fact_is_recorded(self):
        record = _run_in_process(self, "A_spread_01", "bpb3", self.STALL, SCRIPT_POLICY)
        self.assertEqual(record["status"], "no_route")
        view = record["bounded"]["plan"]
        self.assertEqual(view["termination"], "hard_backstop")
        self.assertEqual((view["route_status"], view["incumbents_received"]), ("none", 0))
        self.assertFalse(any(view[k] for k in ("optimality_proven", "complete_route_proven_impossible",
                                               "search_space_exhausted")))
        self.assertIn("within budget", view["execution_message"])
        cleanup = view["executor_cleanup"]
        for key in ("death_verified", "tree_dead", "reader_stopped", "tempdir_removed", "job_released"):
            self.assertIs(cleanup[key], True, key)
        tree = view["tree"]
        self.assertEqual((tree["workers_started"], tree["worker_exit_verified"], tree["private_tempdir_leftovers"]),
                         (1, True, 0))
        self.assertTrue(view["healthy_after"])
        self.assertLessEqual(view["timing"]["decided_s"], SCRIPT_POLICY.hard_stop_s + rep.DECISION_MARGIN_S)
        self.assertAlmostEqual(view["overshoot_s"], view["timing"]["decided_s"] - SCRIPT_POLICY.planning_target_s)
        if WINDOWS:
            self.assertEqual(view["memory"]["worker_peak_rss_source"], "after_exit")
            self.assertGreater(view["memory"]["worker_peak_rss_bytes"], 0)
        self.assertEqual(record["bounded"]["r0_anchor"], bm.R0_ANCHOR)
        self.assertEqual(record["bounded"]["policy"]["name"], "stress")
        json.dumps(record)                                            # strictly serialisable

    def test_retention_through_a_hard_stop_with_a_real_search(self):
        record = _run_in_process(self, "A_spread_01", "bpb3", {"stall_before_final": True}, REAL_POLICY,
                                 stall_key="before_final")
        view = record["bounded"]["plan"]
        self.assertEqual(view["termination"], "hard_backstop")
        self.assertGreaterEqual(view["incumbents_received"], 1)
        self.assertTrue(view["deliverable"])
        self.assertEqual(record["status"], "complete")
        routes = view["routes"]
        self.assertEqual(record["result_hash"], routes["route"]["hash"])
        self.assertIsNotNone(routes["b2_route"])
        self.assertTrue(view["b3_not_worse_than_b2"])
        self.assertFalse(view["optimality_proven"])
        bm.replay_routes(record["bounded"], bfx.load_fixture("A_spread_01"))
        self.assertEqual(routes["route"]["replay_violations"], [])
        self.assertEqual(routes["b2_route"]["replay_violations"], [])

    def test_soak_rows(self):
        record = _run_in_process(self, "A_spread_01", "bpb3", self.STALL, SCRIPT_POLICY, plans=3)
        soak = record["bounded"]["soak"]
        self.assertIsNone(record["bounded"]["plan"])
        self.assertEqual([r["index"] for r in soak["rows"]], [1, 2, 3])
        self.assertEqual({r["termination"] for r in soak["rows"]}, {"hard_backstop"})
        self.assertTrue(all(r["healthy_after"] for r in soak["rows"]))
        self.assertEqual({r["thread_count"] for r in soak["rows"]}, {soak["thread_count_before"]})
        self.assertEqual(record["status"], "no_route")
        if WINDOWS:
            self.assertTrue(all(isinstance(r["handle_count"], int) for r in soak["rows"]))
        self.assertEqual(threading.active_count(), soak["thread_count_before"])


@unittest.skipUnless(WINDOWS, "nested Job Object accounting is Windows-only")
class HarnessPath(unittest.TestCase):
    """The full parent path (sentinel, harness job report, parent replay) on one
    stalled stress-policy plan. Tooling test, not evidence: not a campaign case."""

    @classmethod
    def setUpClass(cls):
        fixture = bfx.load_fixture("A_spread_01")
        cls.record = bench.run_isolated(fixture, "bpb3", False,
                                        _options(timeout_s=40.0, policy="stress", stall="at_call_1"))

    def test_record_shape_and_facts(self):
        r = self.record
        self.assertEqual(tuple(r), bench.RESULT_KEYS)
        self.assertEqual(r["schema_version"], 3)
        self.assertEqual(r["status"], "no_route", r["status_detail"])
        for key in ("validation", "limits", "stats", "planning_time_s", "leg_records"):
            self.assertIsNone(r[key], key)
        self.assertFalse(r["leg_records_included"])
        harness = r["bounded"]["harness"]
        self.assertEqual(harness["job_active_processes_before_close"], 0)
        self.assertTrue(harness["sentinel_alive_after"] and harness["sentinel_termination_verified"])
        self.assertIsInstance(harness["job_peak_commit_bytes"], int)
        tree = r["bounded"]["plan"]["tree"]
        self.assertGreaterEqual(tree["job_active_processes_max_during"], 2)     # the nested worker counted
        self.assertEqual(tree["job_active_processes_after"], 1)
        self.assertTrue(r["cleanup"]["death_verified"] and r["cleanup"]["tempdir_removed"])

    def test_no_bounded_problems_and_no_violation_exit(self):
        self.assertEqual(rep.bounded_problems(self.record), [])
        self.assertFalse(bench.record_has_violations(self.record))
        self.assertEqual(rep.bounded_checks(self.record)["memory"], "ok")


# --------------------------------------------------------------------------
# Synthetic records for the checks and gates
# --------------------------------------------------------------------------

ROUTE_COST = 100.0


def _route(tag):
    return {"hash": tag, "normalized": {"route_kind": "complete", "total_cost": ROUTE_COST},
            "replay_violations": []}


def view(case_id, **kw):
    case = cmp.all_cases()[case_id]
    policy = bm.policy_for(case.option_dict()["policy"])
    stopped = case_id in ("BQ2", "BS1", "BS5")
    termination = kw.get("termination", "hard_backstop" if stopped else
                         "planner_budget" if case_id == "BS2" else "search_finished")
    has_route = kw.get("route", case_id not in ("BQ2", "BS2", "BS5", "BX6", "BX16"))
    decided = kw.get("decided", policy.hard_stop_s + 0.1 if termination == "hard_backstop"
                     else policy.planning_target_s - 5.0)
    if has_route:
        message = "Best complete verified route found" + (
            " within budget (hard backstop)" if termination == "hard_backstop" else "") + ". Optimality not proven."
    elif termination == "search_finished":
        message = "No complete route exists under the supplied viewing candidates (search exhausted)."
    else:
        message = "No complete route found within budget (hard backstop); non-existence is not proven."
    impossible = kw.get("impossible", not has_route and termination == "search_finished")
    metadata = {"stop_reason": "exhausted" if impossible else "first_complete_route",
                "optimality_proven": kw.get("proven", False), "complete_route_proven_impossible": impossible,
                "unresolved_branch_count": 0, "unresolved_bound_known": True, "min_unresolved_lower_bound": None,
                "bound_violation_count": 0, "lower_bound_certified": True, "work_limit_hit": None,
                "empty_candidate_obstacle_ids": [5] if impossible else []}
    metadata.update(kw.get("metadata", {}))
    return {
        "termination": termination, "route_status": "complete" if has_route else "none",
        "route_source": "final" if has_route else None, "deliverable": has_route, "delivery": "on_time",
        "optimality_proven": kw.get("proven", False), "complete_route_proven_impossible": impossible,
        "search_space_exhausted": kw.get("exhausted", impossible),
        "incumbents_received": kw.get("incumbents", 1 if has_route else 0),
        "verification_failures": kw.get("verification_failures", []), "failure_detail": None,
        "execution_message": kw.get("message", message),
        "planner_metadata": metadata,
        "timing": {"decided_s": decided, "returned_s": kw.get("returned", decided + 1.0)},
        "overshoot_s": decided - policy.planning_target_s,
        "executor_cleanup": {"worker_started": True, "containment": "job_object", "death_verified": True,
                             "tree_dead": True, "job_released": True, "tempdir_removed": True, "exit_code": 1,
                             "terminate_error": False, "reader_stopped": True},
        "routes": {"route": _route(kw.get("route_hash", "R-" + case.fixture)) if has_route else None,
                   "b2_route": _route(kw.get("b2_hash", "S-" + case.fixture)) if has_route else None,
                   "diagnostic_route": None},
        "b3_not_worse_than_b2": kw.get("not_worse", True if has_route else None),
        "tree": {"workers_started": 1, "worker_pid": 4242, "worker_exit_verified": True,
                 "job_active_processes_max_during": kw.get("max_during", 2),
                 "job_active_processes_after": kw.get("after", 1),
                 "private_tempdir_leftovers": kw.get("leftovers", 0)},
        "memory": {"worker_peak_rss_bytes": kw.get("worker_rss", 200 * 1024 ** 2),
                   "worker_peak_rss_source": kw.get("source", "after_exit")},
        "healthy_after": True,
    }


def brec(invocation, **kw):
    case_id = invocation.split("#")[0]
    case = cmp.all_cases()[case_id]
    session = next(s for s in cmp.all_sessions() if case in s.cases)
    options = case.option_dict()
    plan = soak = None
    if options.get("soak_plans"):
        rows = []
        for index in range(1, options["soak_plans"] + 1):
            row = view(case_id, **kw.get("row", {}))
            row.update(index=index, handle_count=500 + (kw.get("handle_growth", 0) if index == 8 else 0),
                       thread_count=kw.get("threads", {}).get(index, 3), rss_bytes=100)
            rows.append(row)
        soak = {"handle_count_before": 480, "thread_count_before": 3, "rows": rows}
    else:
        plan = view(case_id, **kw)
    has_route = bool(plan and plan["routes"]["route"])
    return {
        "schema_version": 3, "mode": case.mode, "status": kw.get("status", "complete" if has_route else "no_route"),
        "case": {"campaign": session.campaign, "session": session.name, "case_id": case_id,
                 "invocation": invocation},
        "result_hash": plan["routes"]["route"]["hash"] if has_route else None,
        "wall_time_s": 30.0, "planning_time_s": None, "validation": None,
        "cleanup": {"death_verified": True, "tempdir_removed": True, "timeout_kind": None},
        "peak_memory": {"peak_rss_bytes": kw.get("host_rss", 150 * 1024 ** 2), "tracemalloc_peak_bytes": None},
        "machine": {"system": "Test"}, "python": {"version": "3"}, "git": {"sha": SHA, "dirty": False},
        "bounded": {"policy": bm.policy_record(options["policy"]), "stall": options.get("stall"),
                    "soak_plans": options.get("soak_plans"), "r0_anchor": bm.R0_ANCHOR,
                    "planner_mode": "B3" if case.mode == "bpb3" else "B2", "plan": plan, "soak": soak,
                    "harness": {"job_active_processes_before_close": kw.get("before_close", 0),
                                "job_peak_process_commit_bytes": 5 * GIB, "job_peak_commit_bytes": 9 * GIB,
                                "sentinel_pid": 1, "sentinel_alive_after": kw.get("sentinel", True),
                                "sentinel_termination_verified": True},
                    "replay_s": 0.01},
    }


def bounded_records(**overrides):
    out = {}
    for session in cmp.SESSIONS["bounded"]:
        for case, invocation in cmp.session_invocations(session):
            kw = {"termination": "planner_budget", "decided": TARGET + 1.0} if case.case_id == "BR3" else {}
            out[invocation] = brec(invocation, **kw)
    out.update(overrides)
    return out


def gates(records):
    return {g: v["status"] for g, v in rep.build_summary(list(records.values()))["gates"].items()}


def problems(record):
    return rep.bounded_problems(record)


class _Proc:
    def __init__(self, exited):
        self.exited = exited

    def poll(self):
        return 0 if self.exited else None


class _Worker:
    def __init__(self, exited):
        self.proc = _Proc(exited)


class _Sampler:
    def __init__(self, last):
        self.last_worker_peak = last


class WorkerPeakSource(unittest.TestCase):
    """`after_exit` only when the worker's own handle confirms it exited."""

    def read(self, worker, sampler, handle_read):
        from unittest import mock
        with mock.patch.object(bm, "process_peak_rss", return_value=handle_read):
            return bm.worker_peak_rss(worker, sampler)

    def test_an_exited_worker_gives_after_exit(self):
        self.assertEqual(self.read(_Worker(True), _Sampler(100), 300), (300, "after_exit"))

    def test_a_still_running_worker_is_diagnostic_only(self):
        value, source = self.read(_Worker(False), _Sampler(100), 300)
        self.assertEqual((value, source), (300, "last_sample"))
        self.assertEqual(self.read(_Worker(False), _Sampler(500), 300), (500, "last_sample"))

    def test_an_unreadable_handle_falls_back_to_the_last_sample_or_nothing(self):
        self.assertEqual(self.read(_Worker(True), _Sampler(100), None), (100, "last_sample"))
        self.assertEqual(self.read(_Worker(True), _Sampler(None), None), (None, None))
        self.assertEqual(self.read(None, _Sampler(None), None), (None, None))

    def test_a_live_worker_reading_does_not_satisfy_mem_and_clean_still_fails(self):
        record = brec("BR2", source="last_sample")
        record["bounded"]["plan"]["executor_cleanup"]["death_verified"] = False
        record["bounded"]["plan"]["tree"]["worker_exit_verified"] = False
        checks = rep.bounded_checks(record)
        self.assertEqual(checks["memory"], "not_measured")
        self.assertTrue(checks["clean"])
        self.assertEqual(rep.build_summary(list(bounded_records(BR2=record).values()))["gates"]["G1"]["status"],
                         "fail")

    @unittest.skipUnless(WINDOWS, "peak working set handles are Windows-only")
    def test_with_real_test_owned_processes(self):
        import subprocess
        sleeper = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                                   stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            value, source = bm.worker_peak_rss(_RealWorker(sleeper), _Sampler(None))
            self.assertEqual(source, "last_sample")
            self.assertGreater(value, 0)
        finally:
            sleeper.kill()
            sleeper.wait(timeout=10)
        value, source = bm.worker_peak_rss(_RealWorker(sleeper), _Sampler(None))
        self.assertEqual(source, "after_exit")
        self.assertGreater(value, 0)


class _RealWorker:
    def __init__(self, proc):
        self.proc = proc


class StressDecisionBound(unittest.TestCase):
    """The stress-policy decision limit (hard stop + 0.5 s = 10.5 s) is a G1 check for BQ2 and BS2.
    BS5 is a soak/leak/health test: its rows' decision times are reported, not gated."""

    def test_a_late_bq2_and_bs2_still_fail(self):
        for case_id in ("BQ2", "BS2"):
            with self.subTest(case=case_id):
                late = brec(case_id, decided=10.6)
                self.assertTrue(rep.bounded_checks(late)["decide"])
                self.assertTrue(any("decided_s" in p for p in problems(late)))
                on_time = brec(case_id, decided=10.5)
                self.assertEqual(rep.bounded_checks(on_time)["decide"], [])

    def test_a_bs5_row_at_10_6_does_not_fail_on_decision_time(self):
        record = brec("BS5", row={"decided": 10.6})
        checks = rep.bounded_checks(record)
        self.assertEqual(checks["decide"], [])
        self.assertEqual(len(checks["decide_reported"]), 8)
        self.assertEqual(problems(record), [])
        records = bounded_records(BS5=record)
        self.assertEqual(rep.build_summary(list(records.values()))["cases"]["BS5"]["bounded"]["verdict"], "pass")

    def test_every_other_bs5_requirement_still_applies_with_a_late_row(self):
        for kw, key in (({"row": {"decided": 10.6, "leftovers": 1}}, "clean"),
                        ({"row": {"decided": 10.6, "not_worse": False}}, "retain"),
                        ({"row": {"decided": 10.6, "verification_failures": ["x"]}}, "safe"),
                        ({"row": {"decided": 10.6, "termination": "planner_budget"}}, "expect"),
                        ({"row": {"decided": 10.6, "source": "last_sample"}}, "memory_problems"),
                        ({"row": {"decided": 10.6}, "handle_growth": 21}, "soak"),
                        ({"row": {"decided": 10.6}, "threads": {5: 4}}, "soak"),
                        ({"row": {"decided": 10.6, "worker_rss": GIB + 1}}, "memory_problems")):
            with self.subTest(kw=kw):
                self.assertTrue(rep.bounded_checks(brec("BS5", **kw))[key], kw)
        honest = brec("BS5", row={"decided": 10.6, "proven": True})
        self.assertTrue(rep.bounded_checks(honest)["honest"])

    def test_a_production_policy_soak_would_not_be_exempt(self):
        record = brec("BS5", row={"decided": HARD_STOP + 1.0})
        record["bounded"]["policy"] = bm.policy_record("production")
        self.assertTrue(rep.bounded_checks(record)["decide"])


class Checks(unittest.TestCase):
    def test_every_clean_synthetic_record_has_no_problems(self):
        for invocation, record in bounded_records().items():
            with self.subTest(invocation=invocation):
                self.assertEqual(problems(record), [])
                self.assertEqual(rep.bounded_checks(record)["memory"], "ok")

    def test_safe_and_retain(self):
        bad = brec("BR2")
        bad["bounded"]["plan"]["routes"]["route"]["replay_violations"] = ["collision: x"]
        self.assertTrue(any("replay violation" in p for p in problems(bad)))
        unreplayed = brec("BR2")
        unreplayed["bounded"]["plan"]["routes"]["b2_route"]["replay_violations"] = None
        self.assertTrue(any("not replayed" in p for p in problems(unreplayed)))
        self.assertTrue(problems(brec("BR2", verification_failures=["b3: bad"])))
        self.assertTrue(any("B2 seed" in p for p in problems(brec("BR2", not_worse=False))))
        lost = brec("BX1", route=False, incumbents=1, termination="hard_backstop")
        self.assertTrue(any("no route was retained" in p for p in problems(lost)))

    def test_clean(self):
        for kw, text in (({"after": 2}, "job active processes after"), ({"leftovers": 1}, "temporary"),
                         ({"before_close": 1}, "before close"), ({"sentinel": False}, "sentinel")):
            with self.subTest(kw=kw):
                self.assertTrue(any(text in p for p in problems(brec("BX1", **kw))), kw)
        record = brec("BX1")
        record["bounded"]["plan"]["executor_cleanup"]["reader_stopped"] = False
        self.assertTrue(any("reader_stopped" in p for p in problems(record)))
        record = brec("BX1")
        record["bounded"]["plan"]["tree"]["worker_exit_verified"] = None
        self.assertTrue(any("worker exit" in p for p in problems(record)))

    def test_honest(self):
        self.assertTrue(rep.bounded_checks(brec("BR2", termination="planner_budget", proven=True))["honest"])
        self.assertTrue(rep.bounded_checks(brec("BS2", message="No complete route found."))["honest"])
        bad = brec("BR2", proven=True)
        bad["bounded"]["plan"]["planner_metadata"]["unresolved_branch_count"] = 2
        self.assertTrue(rep.bounded_checks(bad)["honest"])
        self.assertTrue(rep.bounded_checks(brec("BX1", route=False, impossible=True,
                                                termination="hard_backstop"))["honest"])
        self.assertFalse(rep.bounded_checks(brec("BX6"))["honest"])     # a finished proof is allowed

    def test_memory_host_and_worker_each_gate_committed_memory_never_does(self):
        self.assertEqual(rep.bounded_checks(brec("BR2", host_rss=GIB + 1))["memory"], "fail")
        self.assertEqual(rep.bounded_checks(brec("BR2", worker_rss=GIB + 1))["memory"], "fail")
        self.assertEqual(rep.bounded_checks(brec("BR2", worker_rss=GIB + 1, source="last_sample"))["memory"],
                         "fail")
        self.assertEqual(rep.bounded_checks(brec("BR2", source="last_sample"))["memory"], "not_measured")
        self.assertEqual(rep.bounded_checks(brec("BR2", worker_rss=None, source=None))["memory"], "not_measured")
        self.assertEqual(rep.bounded_checks(brec("BR2"))["memory"], "ok")     # job commit is 9 GiB here

    def test_case_expectations(self):
        self.assertTrue(rep.bounded_checks(brec("BQ1", termination="hard_backstop"))["expect"])
        self.assertTrue(rep.bounded_checks(brec("BQ1", source="last_sample"))["expect"])
        self.assertTrue(rep.bounded_checks(brec("BQ2", incumbents=1))["expect"])
        self.assertTrue(rep.bounded_checks(brec("BS1", incumbents=0))["expect"])
        self.assertTrue(rep.bounded_checks(brec("BS1", route=False))["expect"])
        self.assertTrue(rep.bounded_checks(brec("BX6", route=True, impossible=False))["expect"])
        self.assertTrue(rep.bounded_checks(brec("BS2", termination="cancelled"))["expect"])
        self.assertTrue(rep.bounded_checks(brec("BQ2", max_during=1))["nested"])
        self.assertTrue(rep.bounded_checks(brec("BQ1", before_close=1))["nested"])

    def test_stress_decision_is_a_g1_check_and_production_decision_is_g6(self):
        self.assertTrue(problems(brec("BS2", decided=10.6)))
        self.assertTrue(rep.bounded_checks(brec("BR2", decided=HARD_STOP + 0.6))["decide"])
        self.assertFalse(rep.bounded_checks(brec("BR2", decided=HARD_STOP + 0.5))["decide"])
        self.assertTrue(rep.bounded_checks(brec("BR2", returned=RETURN_DEADLINE + 0.01))["return"])
        self.assertFalse(rep.bounded_checks(brec("BR2", returned=RETURN_DEADLINE))["return"])

    def test_soak(self):
        self.assertEqual(rep.bounded_checks(brec("BS5"))["soak"], [])
        self.assertTrue(rep.bounded_checks(brec("BS5", handle_growth=21))["soak"])
        self.assertEqual(rep.bounded_checks(brec("BS5", handle_growth=20))["soak"], [])
        self.assertTrue(rep.bounded_checks(brec("BS5", threads={4: 4}))["soak"])
        self.assertTrue(rep.bounded_checks(brec("BS5", row={"termination": "planner_budget"}))["expect"])
        unhealthy = brec("BS5")
        unhealthy["bounded"]["soak"]["rows"][6]["healthy_after"] = False
        self.assertTrue(rep.bounded_checks(unhealthy)["soak"])
        short = brec("BS5")
        short["bounded"]["soak"]["rows"].pop()
        self.assertTrue(rep.bounded_checks(short)["soak"])

    def test_harness_timeout_and_error_fail_safe(self):
        for status in ("timeout", "error"):
            record = brec("BR2", status=status)
            record["bounded"]["plan"] = None
            self.assertTrue(any("status" in p for p in problems(record)))


class ReturnDeadline(unittest.TestCase):
    """The return deadline applies to every production plan outcome, not only deliverable ones."""

    def ret(self, record):
        return rep.bounded_checks(record)["return"]

    def test_on_time_no_route_passes(self):
        self.assertEqual(self.ret(brec("BX1", route=False, termination="hard_backstop", decided=HARD_STOP + 0.1,
                                       returned=50.0)), [])

    def test_late_no_route_fails(self):
        self.assertTrue(self.ret(brec("BX1", route=False, termination="hard_backstop", decided=HARD_STOP + 0.1,
                                      returned=RETURN_DEADLINE + 0.5)))

    def test_late_withheld_complete_route_fails(self):
        record = brec("BR2", termination="hard_backstop", decided=HARD_STOP + 0.1, returned=RETURN_DEADLINE + 0.3)
        plan = record["bounded"]["plan"]
        plan["routes"]["diagnostic_route"], plan["routes"]["route"] = plan["routes"]["route"], None
        plan.update(deliverable=False, delivery="request_ceiling_exceeded", route_status="none",
                    route_source=None,
                    execution_message="The verified route was ready after the response deadline and is "
                                      "not delivered.")
        record["status"] = "no_route"
        self.assertTrue(self.ret(record))
        self.assertTrue(problems(record))

    def test_on_time_deliverable_route_passes(self):
        self.assertEqual(self.ret(brec("BR2", returned=RETURN_DEADLINE - 0.1)), [])
        self.assertEqual(self.ret(brec("BR2", returned=RETURN_DEADLINE)), [])

    def test_missing_or_non_numeric_returned_fails(self):
        for value in (None, "58", True, float("nan")):
            with self.subTest(value=value):
                self.assertTrue(self.ret(brec("BX1", route=False, termination="hard_backstop", decided=HARD_STOP + 0.1,
                                              returned=value)))

    def test_a_late_production_no_route_fails_g5(self):
        late = brec("BX1", route=False, termination="hard_backstop", decided=HARD_STOP + 0.1, returned=RETURN_DEADLINE + 0.5)
        self.assertEqual(gates(bounded_records(BX1=late))["G5"], "fail")


class NaturalOvershoot(unittest.TestCase):
    """G6 counts a production BR/BX plan as natural overshoot from its RECORDED decision time."""

    def without_overrun(self, **overrides):
        records = bounded_records(**{f"BR3#{i}": brec(f"BR3#{i}") for i in (1, 2, 3)})
        records.update(overrides)
        return records

    def test_a_search_finished_plan_that_decides_after_the_target_counts(self):
        records = self.without_overrun(BR2=brec("BR2", termination="search_finished", decided=TARGET + 1.0))
        self.assertEqual(gates(records)["G6"], "pass")

    def test_a_termination_label_alone_does_not_count(self):
        records = self.without_overrun(**{f"BR3#{i}": brec(f"BR3#{i}", termination="planner_budget",
                                                           decided=TARGET - 0.1) for i in (1, 2, 3)})
        self.assertEqual(gates(records)["G6"], "not_measured")
        exactly = self.without_overrun(BR2=brec("BR2", termination="hard_backstop", decided=TARGET))
        self.assertEqual(gates(exactly)["G6"], "not_measured")

    def test_stall_stress_and_soak_records_never_count(self):
        records = self.without_overrun()
        self.assertEqual(gates(records)["G6"], "not_measured")     # BS1 (stall), BS2/BQ2/BS5 (stress)
        for invocation in ("BS1#1", "BS1#2"):
            self.assertGreater(records[invocation]["bounded"]["plan"]["timing"]["decided_s"], TARGET)

    def test_the_decision_bound_still_applies(self):
        records = self.without_overrun(BR2=brec("BR2", decided=HARD_STOP + 0.6), BX2=brec("BX2", decided=TARGET + 1.0))
        self.assertEqual(gates(records)["G6"], "fail")
        entry = rep.build_summary(list(self.without_overrun(BX2=brec("BX2", decided=TARGET + 1.0)).values()))
        self.assertEqual(entry["gates"]["G6"]["status"], "pass")


class HonestProofRules(unittest.TestCase):
    """HONEST re-applies the executor's own proof rules to the recorded outcome."""

    def honest(self, record):
        return rep.bounded_checks(record)["honest"]

    def proof(self, **metadata):
        base = {"stop_reason": "bound_closed", "optimality_proven": True, "unresolved_branch_count": 3,
                "unresolved_bound_known": True, "min_unresolved_lower_bound": ROUTE_COST + 5.0}
        base.update(metadata)
        return brec("BR2", proven=True, metadata=base)

    def test_closed_proof_with_zero_unresolved(self):
        self.assertEqual(self.honest(self.proof(unresolved_branch_count=0, min_unresolved_lower_bound=None)), [])

    def test_bound_closed_proof_with_unresolved_branches_covered_by_their_bound(self):
        self.assertEqual(self.honest(self.proof()), [])
        self.assertEqual(self.honest(self.proof(min_unresolved_lower_bound=ROUTE_COST)), [])   # equal

    def test_uncovered_unresolved_branches_are_rejected(self):
        for metadata in ({"min_unresolved_lower_bound": ROUTE_COST - 0.001},
                         {"unresolved_bound_known": False},
                         {"min_unresolved_lower_bound": None},
                         {"min_unresolved_lower_bound": "inf"}):
            with self.subTest(metadata=metadata):
                self.assertTrue(self.honest(self.proof(**metadata)))
        missing = self.proof()
        del missing["bounded"]["plan"]["planner_metadata"]["min_unresolved_lower_bound"]
        self.assertTrue(self.honest(missing))
        no_cost = self.proof()
        del no_cost["bounded"]["plan"]["routes"]["route"]["normalized"]["total_cost"]
        self.assertTrue(self.honest(no_cost))

    def test_other_proof_conditions_are_still_required(self):
        for metadata in ({"stop_reason": "first_complete_route"}, {"lower_bound_certified": False},
                         {"bound_violation_count": 1}, {"optimality_proven": False}):
            with self.subTest(metadata=metadata):
                self.assertTrue(self.honest(self.proof(**metadata)))
        b2 = self.proof()
        b2["bounded"]["planner_mode"] = "B2"
        self.assertTrue(self.honest(b2))
        incumbent = self.proof()
        incumbent["bounded"]["plan"]["route_source"] = "b3_incumbent"
        self.assertTrue(self.honest(incumbent))
        withheld = self.proof()
        withheld["bounded"]["plan"]["deliverable"] = False
        self.assertTrue(self.honest(withheld))
        unclean = self.proof()
        unclean["bounded"]["plan"]["executor_cleanup"]["tree_dead"] = False
        self.assertTrue(self.honest(unclean))

    def test_search_space_exhausted(self):
        valid = brec("BR2", exhausted=True, metadata={"stop_reason": "exhausted"})
        self.assertEqual(self.honest(valid), [])
        closed = brec("BR2", exhausted=True, metadata={"stop_reason": "bound_closed"})
        self.assertEqual(self.honest(closed), [])
        for metadata in ({"stop_reason": "exhausted", "unresolved_branch_count": 2},
                         {"stop_reason": "first_complete_route"},
                         {"stop_reason": "exhausted", "unresolved_branch_count": None}):
            with self.subTest(metadata=metadata):
                self.assertTrue(self.honest(brec("BR2", exhausted=True, metadata=metadata)))

    def test_impossibility(self):
        self.assertEqual(self.honest(brec("BX6")), [])
        exhaustive = brec("BX6", metadata={"empty_candidate_obstacle_ids": [], "stop_reason": "exhausted"})
        self.assertEqual(self.honest(exhaustive), [])
        for metadata in ({"empty_candidate_obstacle_ids": [], "unresolved_branch_count": 1},
                         {"complete_route_proven_impossible": False}):
            with self.subTest(metadata=metadata):
                self.assertTrue(self.honest(brec("BX6", metadata=metadata)))
        retained = brec("BX6")
        retained["bounded"]["plan"]["routes"]["diagnostic_route"] = _route("D")
        self.assertTrue(self.honest(retained))

    def test_bounded_stops_force_every_flag_false(self):
        for termination in ("planner_budget", "hard_backstop", "cancelled", "planner_work_limit",
                            "startup_overran_budget", "worker_failed", "executor_failed"):
            for flag in ({"proven": True}, {"exhausted": True}, {"impossible": True, "route": False}):
                with self.subTest(termination=termination, flag=flag):
                    record = brec("BX1", termination=termination, decided=TARGET + 1.0, **flag)
                    self.assertTrue(any("claimed after" in p for p in self.honest(record)))


class NoRouteWording(unittest.TestCase):
    """The "within budget" wording rule covers an ordinary clean bounded no-route reply only.
    Messages come from the executor itself, so a wording change there cannot hide behind a copy."""

    NO_ROUTE = dict(route=False, termination="hard_backstop", decided=HARD_STOP + 0.1)

    def msg(self, cleanup_ok, on_time, had_route):
        return bpp._message("hard_backstop", None, False, False, cleanup_ok, on_time, had_route)

    def test_a_cleanup_failure_is_owned_by_clean_and_g1_not_honest(self):
        message = self.msg(False, True, False)
        self.assertNotIn("within budget", message)
        record = brec("BX1", message=message, **self.NO_ROUTE)
        record["bounded"]["plan"]["executor_cleanup"]["tree_dead"] = False
        checks = rep.bounded_checks(record)
        self.assertEqual(checks["honest"], [])
        self.assertTrue(checks["clean"])
        records = bounded_records(BX1=record)
        summary = rep.build_summary(list(records.values()))
        self.assertEqual(summary["gates"]["G1"]["status"], "fail")
        self.assertIn("BX1", summary["gates"]["G1"]["evidence"])
        self.assertNotEqual(summary["gates"]["G8"]["status"], "fail")

    def test_a_late_withheld_route_is_owned_by_return_and_g5_not_honest(self):
        message = self.msg(True, False, True)
        self.assertNotIn("within budget", message)
        record = brec("BR2", termination="hard_backstop", decided=HARD_STOP + 0.1, returned=RETURN_DEADLINE + 0.3, message=message)
        plan = record["bounded"]["plan"]
        plan["routes"]["diagnostic_route"], plan["routes"]["route"] = plan["routes"]["route"], None
        plan.update(deliverable=False, delivery="request_ceiling_exceeded", route_status="none",
                    route_source=None)
        record["status"] = "no_route"
        checks = rep.bounded_checks(record, "BR2")
        self.assertEqual(checks["honest"], [])
        self.assertTrue(checks["return"])
        summary = rep.build_summary(list(bounded_records(BR2=record).values()))
        self.assertEqual(summary["gates"]["G5"]["status"], "fail")
        self.assertNotEqual(summary["gates"]["G8"]["status"], "fail")

    def test_an_ordinary_clean_no_route_reply_without_within_budget_still_fails_honest(self):
        record = brec("BX1", message="No complete route found.", **self.NO_ROUTE)
        self.assertTrue(rep.bounded_checks(record)["honest"])
        summary = rep.build_summary(list(bounded_records(BX1=record).values()))
        self.assertEqual(summary["gates"]["G8"]["status"], "fail")

    def test_the_proper_ordinary_no_route_message_passes(self):
        message = self.msg(True, True, False)
        self.assertIn("within budget", message)
        self.assertEqual(rep.bounded_checks(brec("BX1", message=message, **self.NO_ROUTE))["honest"], [])
        self.assertEqual(problems(brec("BX1", message=message, **self.NO_ROUTE)), [])


class Gates(unittest.TestCase):
    def test_clean_bounded_evidence_passes_g5_and_g6_only(self):
        g = gates(bounded_records())
        self.assertEqual((g["G5"], g["G6"]), ("pass", "pass"))
        self.assertEqual((g["G1"], g["G3"], g["G7"], g["G8"]),
                         ("not_measured", "not_measured", "not_measured", "not_measured"))

    def test_g5(self):
        records = bounded_records()
        del records["BR4"]
        self.assertEqual(gates(records)["G5"], "not_measured")
        self.assertEqual(gates(bounded_records(**{"BR1#2": brec("BR1#2", route=False, termination="planner_budget")}))
                         ["G5"], "fail")
        self.assertEqual(gates(bounded_records(**{"BR7": brec("BR7", returned=RETURN_DEADLINE + 0.2)}))["G5"], "fail")
        self.assertEqual(gates(bounded_records(**{"BX3": brec("BX3", returned=RETURN_DEADLINE + 0.2)}))["G5"], "fail")
        timeout = brec("BX3", status="timeout")
        timeout["bounded"]["plan"] = None
        self.assertEqual(gates(bounded_records(BX3=timeout))["G5"], "fail")

    def test_repeat(self):
        seed = bounded_records(**{"BR5#2": brec("BR5#2", b2_hash="other")})
        self.assertEqual(gates(seed)["G5"], "fail")
        finished = bounded_records(**{"BR5#3": brec("BR5#3", route_hash="other")})
        self.assertEqual(gates(finished)["G5"], "fail")
        varied = bounded_records(**{f"BR5#{i}": brec(f"BR5#{i}", termination="planner_budget", decided=TARGET + 1.0,
                                                     route_hash=f"B3-{i}") for i in (1, 2, 3)})
        summary = rep.build_summary(list(varied.values()))
        self.assertEqual(summary["gates"]["G5"]["status"], "pass")      # reported, not failed
        self.assertTrue(summary["bounded"]["repeat"]["BR5"]["wall_clock_variation"])
        self.assertEqual(summary["bounded"]["repeat"]["BR1"]["state"], "ok")

    def test_g6(self):
        self.assertEqual(gates(bounded_records(BR2=brec("BR2", decided=HARD_STOP + 0.6)))["G6"], "fail")
        bs1 = bounded_records(**{"BS1#1": brec("BS1#1", decided=HARD_STOP + 0.6)})
        self.assertEqual(gates(bs1)["G6"], "fail")          # a stall-injected production stop still counts
        stress = bounded_records(BS2=brec("BS2", decided=10.6))
        self.assertEqual(gates(stress)["G6"], "pass")
        self.assertIn("BS2", rep.build_summary(list(stress.values()))["gates"]["G1"]["evidence"])
        no_overrun = bounded_records(**{f"BR3#{i}": brec(f"BR3#{i}") for i in (1, 2, 3)})
        self.assertEqual(gates(no_overrun)["G6"], "not_measured")

    def test_g1_g7_g8_bounded_clauses(self):
        self.assertEqual(gates(bounded_records(BX2=brec("BX2", sentinel=False)))["G1"], "fail")
        self.assertEqual(gates(bounded_records(BR2=brec("BR2", worker_rss=GIB + 1)))["G7"], "fail")
        self.assertEqual(gates(bounded_records(BR2=brec("BR2", termination="hard_backstop", proven=True,
                                                        decided=HARD_STOP + 0.1)))["G8"], "fail")

    def test_legacy_gates_are_unchanged_by_clean_bounded_evidence(self):
        legacy = standard_records(X13=rec("X13", rss=GIB // 2))
        before = gates(legacy)
        after = gates({**legacy, **bounded_records()})
        for gate in ("G1", "G2", "G3", "G4", "G7", "G8"):
            self.assertEqual(before[gate], after[gate], gate)
        self.assertEqual((before["G5"], before["G6"]), ("not_measured", "not_measured"))

    def test_g7_is_not_passed_on_an_unread_worker_peak(self):
        legacy = standard_records(X13=rec("X13", rss=GIB // 2))
        self.assertEqual(gates(legacy)["G7"], "pass")
        self.assertEqual(gates({**legacy, "BR2": brec("BR2", source="last_sample")})["G7"], "not_measured")


class SchemaCompatibility(unittest.TestCase):
    def test_a_schema_2_record_is_judged_exactly_as_before(self):
        legacy = rec("R1")
        explicit = dict(copy.deepcopy(legacy), schema_version=3, bounded=None)
        a = rep.build_summary([legacy])["cases"]["R1"]
        b = rep.build_summary([explicit])["cases"]["R1"]
        self.assertEqual(a, b)
        self.assertNotIn("bounded", a)
        self.assertFalse(bench.record_has_violations(legacy))

    def test_bounded_entries_only_for_bounded_cases(self):
        s = rep.build_summary(list(bounded_records().values()))
        self.assertTrue(all("bounded" in s["cases"][i] for i in bounded_records()))
        self.assertTrue(all("bounded" not in e for i, e in s["cases"].items()
                            if not cmp.all_cases()[i.split("#")[0]].mode.startswith("bpb")))
        self.assertEqual(set(s["bounded"]), {"repeat", "overshoot", "memory", "verdicts"})
        self.assertEqual(set(s["bounded"]["verdicts"].values()), {"pass"})
        text = rep.render_text(s)
        self.assertIn("bounded invocations:", text)
        self.assertIn("BQ1: pass", text)


class RunnerExitCodes(unittest.TestCase):
    def test_a_job_left_non_empty_aborts_the_campaign(self):
        fixture = bfx.load_fixture("A_spread_01")
        left = brec("BX1", before_close=2)
        left["status"] = "no_route"
        from unittest import mock
        with mock.patch.object(bench, "run_isolated", return_value=left):
            code = bench.execute_plan([(fixture, "bpb3", False, {}), (fixture, "bpb3", False, {})], {},
                                      lambda r: None)
        self.assertEqual(code, bench.EXIT_ABORT)

    def test_bounded_problems_give_the_violation_exit(self):
        fixture = bfx.load_fixture("A_spread_01")
        from unittest import mock
        with mock.patch.object(bench, "run_isolated", return_value=brec("BX1", leftovers=1)):
            code = bench.execute_plan([(fixture, "bpb3", False, {})], {}, lambda r: None)
        self.assertEqual(code, bench.EXIT_VIOLATIONS)


if __name__ == "__main__":
    unittest.main()
