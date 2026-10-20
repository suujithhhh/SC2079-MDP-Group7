"""Bounded prefix-search execution with real worker processes.

Real-route tests run the real worker and pipeline on the existing `A_spread_02`
fixture (two obstacles, about 2 s, and a genuine B3 improvement over B2) and
inject only delays, stalls and crashes around the real leg planner
(tests/_bounded_test_worker.py). Byte-level and edge cases use a scripted worker
and a stub verifier, so they are deterministic.

Everything killed here is a worker the planner under test started, its
grandchild, or a sentinel this test owns and kills through its own handle.
Nothing enumerates or signals any other process.

Run from algo/:
    python -m unittest tests.test_bounded_prefix_process -v
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

import bounded_prefix_planning as bp  # noqa: E402
import prefix_worker as pw  # noqa: E402
import route_prefix_search as rps  # noqa: E402
import worker_containment as wc  # noqa: E402
from benchmarks import fixtures as bfx  # noqa: E402  (test-only)
from benchmarks import normalized as nz  # noqa: E402  (test-only independent oracle)
from benchmarks import route_replay  # noqa: E402  (test-only independent oracle)
from benchmarks.process_control import process_alive  # noqa: E402  (test-only liveness probe)
from tests.test_bounded_prefix_planning import INPUT, SCALED, fake_final, fake_record  # noqa: E402

TEST_WORKER = ALGO / "tests" / "_bounded_test_worker.py"
FAST = bp.ExecutionPolicy(planning_target_s=1.5, hard_stop_s=2.0, verification_deadline_s=6.0,
                          return_target_s=7.0, return_deadline_s=8.0, request_ceiling_s=10.0,
                          min_search_budget_s=0.5, death_wait_s=3.0, cancel_grace_s=1.0)
COOPERATIVE = bp.ExecutionPolicy(
    planning_target_s=4.0, hard_stop_s=9.0, verification_deadline_s=12.0,
    return_target_s=13.0, return_deadline_s=14.0, request_ceiling_s=16.0,
    min_search_budget_s=0.5, death_wait_s=3.0, cancel_grace_s=1.0)
SLEEPER = [sys.executable, "-c", "import time; time.sleep(120)"]
PRELUDE = [{"op": "hello"}, {"op": "read_spec"}]

_ORACLE = {}


def oracle():
    """Direct (in-process) search on the fixture: the reference for every real test."""
    if not _ORACLE:
        fixture = bfx.load_fixture("A_spread_02")
        seen = []
        result = rps.optimise_prefix_b3_route(
            fixture.start, fixture.obstacles, allow_alternative_viewing=False,
            on_incumbent=lambda route, phase: seen.append((phase, route)))
        _ORACLE.update(fixture=fixture, result=result, seen=seen,
                       input=bp.PlanInput(fixture.start, tuple(fixture.obstacles), "B3", False))
    return _ORACLE


def stub_verifier(record, inp, config=None, *, deadline=None, clock=None):
    return SimpleNamespace(total_cost=record["total_cost"], obstacle_order=tuple(record["obstacle_order"]))


class ProcessCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="bounded_test_")
        self.addCleanup(self._tmp.cleanup)
        self.created = []
        real_mkdtemp = tempfile.mkdtemp

        def recording_mkdtemp(*args, **kwargs):
            path = real_mkdtemp(*args, **kwargs)
            self.created.append(path)
            return path

        patcher = mock.patch.object(bp.tempfile, "mkdtemp", side_effect=recording_mkdtemp)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.addCleanup(self._remove_leftovers)

        # Record every worker the planner starts, so the test can release what it
        # owns even when a fail-closed test leaves it deliberately unverified.
        self.workers = []
        real_start = wc.start_worker

        def recording_start(*args, **kwargs):
            worker = real_start(*args, **kwargs)
            self.workers.append(worker)
            return worker

        patcher = mock.patch.object(bp.wc, "start_worker", side_effect=recording_start)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.addCleanup(self._release_workers)      # runs before _remove_leftovers

    def _release_workers(self):
        """Test hygiene for the workers this test's planner started, and only
        those: reap each through its own handle, let its reader thread reach end
        of stream, then close every pipe and job handle. Production fail-closed
        behaviour is not involved: this runs after the outcome was checked."""
        for worker in self.workers:
            if worker.proc.poll() is None:
                try:
                    worker.terminate_tree()
                except OSError:
                    worker.proc.kill()
            worker.proc.wait(timeout=15)
        for thread in threading.enumerate():
            if thread.name == "prefix-frame-reader":
                thread.join(timeout=15)
        for worker in self.workers:
            worker.close_pipes()
            worker.close()

    def _remove_leftovers(self):
        for path in self.created:
            shutil.rmtree(path, ignore_errors=True)

    def planner(self, mode, config, policy=SCALED, **kwargs):
        path = Path(self._tmp.name) / f"config{len(list(Path(self._tmp.name).iterdir()))}.json"
        path.write_text(json.dumps(config), encoding="utf-8")
        command = [sys.executable, str(TEST_WORKER), mode, str(path)]
        return bp.BoundedPrefixPlanner(policy, worker_command=command, **kwargs)

    def scripted(self, steps, policy=FAST, verifier=stub_verifier, **kwargs):
        return self.planner("script", {"steps": steps}, policy, verifier=verifier, **kwargs)

    @staticmethod
    def run_plan(planner, inp=INPUT, elapsed=0.0, cancel=None):
        return planner.plan(inp, accepted_at=time.monotonic() - elapsed, cancel=cancel)

    def assertTempdirsGone(self):
        self.assertTrue(self.created)
        for path in self.created:
            self.assertFalse(os.path.exists(path), f"temporary directory left behind: {path}")

    def assertCleanTeardown(self, outcome):
        cleanup = outcome.cleanup
        self.assertTrue(cleanup.worker_started)
        self.assertTrue(cleanup.death_verified)
        self.assertTrue(cleanup.tree_dead)
        self.assertTrue(cleanup.job_released)
        self.assertTrue(cleanup.tempdir_removed)
        self.assertTrue(cleanup.reader_stopped)
        self.assertTrue(cleanup.ok)
        self.assertTempdirsGone()
        for worker in self.workers:
            self.assertIsNotNone(worker.proc.returncode, "the worker was not reaped")
            self.assertTrue(worker.proc.stdin.closed and worker.proc.stdout.closed, "a worker pipe was left open")

    def assertNothingProven(self, outcome):
        self.assertFalse(outcome.optimality_proven)
        self.assertFalse(outcome.complete_route_proven_impossible)
        self.assertFalse(outcome.search_space_exhausted)


class RealRoutes(ProcessCase):
    def assertSameRoute(self, route, direct):
        self.assertEqual(route.stm_commands, direct.stm_commands)
        self.assertEqual(route.total_cost, direct.total_cost)
        self.assertEqual(route.final_pose, direct.final_pose)
        self.assertEqual(route.obstacle_order, direct.obstacle_order)

    def assertReplayValid(self, route, kind="prefix_b3"):
        data = oracle()
        norm = nz.normalize_prefix_route(route, "complete", search=kind, start_pose=data["fixture"].start,
                                         alternatives=False)
        self.assertEqual(route_replay.replay_violations(norm, data["fixture"]), [])

    def test_the_reference_fixture_really_improves_on_b2(self):
        seen = oracle()["seen"]
        self.assertEqual([phase for phase, _ in seen], ["b2", "b3"], "test premise: B3 improves on B2")
        self.assertLess(seen[1][1].total_cost, seen[0][1].total_cost)

    def test_normal_completion(self):
        data = oracle()
        outcome = self.run_plan(self.planner("real", {}), data["input"])
        direct, b2 = data["result"].complete_route, data["seen"][0][1]
        self.assertEqual(outcome.termination, bp.TERM_SEARCH_FINISHED)
        self.assertTrue(outcome.deliverable)
        self.assertEqual(outcome.delivery, bp.DELIVERY_ON_TIME)
        self.assertEqual(outcome.route_status, "complete")
        self.assertEqual(outcome.route_source, bp.SOURCE_FINAL)
        self.assertSameRoute(outcome.route, direct)
        self.assertSameRoute(outcome.b2_route, b2)
        self.assertEqual(outcome.incumbents_received, 2)
        self.assertLessEqual(outcome.route.total_cost, outcome.b2_route.total_cost)
        self.assertReplayValid(outcome.route)
        self.assertTrue(outcome.optimality_proven)          # an honestly exhausted, certified B3 search
        self.assertEqual(outcome.planner_metadata["stop_reason"], data["result"].stop_reason)
        self.assertEqual(outcome.verification_failures, ())
        self.assertIsNone(outcome.diagnostic_route)
        self.assertEqual(outcome.cleanup.exit_code, 0)
        self.assertCleanTeardown(outcome)
        self.assertTrue(self.planner("real", {}).healthy)

    def test_b2_mode_never_claims_optimality(self):
        data = oracle()
        inp = bp.PlanInput(data["input"].start, data["input"].obstacles, "B2", False)
        outcome = self.run_plan(self.planner("real", {}), inp)
        self.assertEqual(outcome.termination, bp.TERM_SEARCH_FINISHED)
        self.assertSameRoute(outcome.route, data["seen"][0][1])
        self.assertEqual(outcome.incumbents_received, 1)
        self.assertNothingProven(outcome)
        self.assertCleanTeardown(outcome)

    def test_cooperative_budget_keeps_the_b2_route_and_is_not_a_kill(self):
        data = oracle()
        outcome = self.run_plan(
            self.planner("real", {"sleep_per_call_s": 1.8}, policy=COOPERATIVE), data["input"])
        self.assertEqual(outcome.termination, bp.TERM_PLANNER_BUDGET)
        self.assertEqual(outcome.planner_metadata["stop_reason"], rps.STOP_WALL_CLOCK_CHECKPOINT)
        self.assertSameRoute(outcome.route, data["seen"][0][1])      # complete B2 route survived
        self.assertEqual(outcome.route_source, bp.SOURCE_FINAL)
        self.assertReplayValid(outcome.route, "prefix_b2")
        self.assertNothingProven(outcome)
        self.assertLess(outcome.timing.decided_s, COOPERATIVE.hard_stop_s)  # worker stopped itself
        self.assertEqual(outcome.cleanup.exit_code, 0)
        self.assertIn("within budget (planning budget)", outcome.execution_message)
        self.assertIn("Optimality not proven", outcome.execution_message)
        self.assertCleanTeardown(outcome)

    def test_budget_stop_without_a_complete_route_is_an_honest_bounded_failure(self):
        outcome = self.run_plan(self.planner("real", {"sleep_per_call_s": 4.5}), oracle()["input"])
        self.assertEqual(outcome.termination, bp.TERM_PLANNER_BUDGET)
        self.assertIsNone(outcome.route)
        self.assertEqual(outcome.route_status, "none")
        self.assertFalse(outcome.deliverable)
        self.assertEqual(outcome.planner_metadata["route_status"], "partial_only")
        self.assertNothingProven(outcome)
        self.assertIn("No complete route found within budget", outcome.execution_message)
        self.assertIn("non-existence is not proven", outcome.execution_message)
        self.assertNotIn("impossible", outcome.execution_message.lower())
        self.assertCleanTeardown(outcome)

    def test_b2_survives_a_hard_stop_during_b3(self):
        data = oracle()
        started = time.monotonic()
        outcome = self.run_plan(self.planner("real", {"stall_at_call": 3}), data["input"])
        self.assertEqual(outcome.termination, bp.TERM_HARD_BACKSTOP)
        self.assertGreaterEqual(outcome.timing.decided_s, SCALED.hard_stop_s)
        self.assertLess(outcome.timing.decided_s, SCALED.hard_stop_s + 1.0)
        self.assertLess(outcome.timing.returned_s, SCALED.return_target_s)
        self.assertLess(time.monotonic() - started, SCALED.return_deadline_s)
        self.assertTrue(outcome.deliverable)
        self.assertEqual(outcome.route_source, bp.SOURCE_B2)
        self.assertSameRoute(outcome.route, data["seen"][0][1])
        self.assertEqual(outcome.incumbents_received, 1)
        self.assertReplayValid(outcome.route, "prefix_b2")
        self.assertLessEqual(outcome.route.total_cost, outcome.b2_route.total_cost)
        self.assertNothingProven(outcome)
        self.assertNotEqual(outcome.cleanup.exit_code, 0)             # it was killed, not finished
        self.assertCleanTeardown(outcome)

    def test_an_improved_b3_incumbent_survives_a_hard_stop_and_nothing_is_proven(self):
        data = oracle()
        # The search finishes (exhausted) but the worker stalls before FINAL: only the
        # incumbents reached the parent, and a bounded stop proves nothing.
        outcome = self.run_plan(self.planner("real", {"stall_before_final": True}), data["input"])
        b2, b3 = data["seen"][0][1], data["seen"][1][1]
        self.assertEqual(outcome.termination, bp.TERM_HARD_BACKSTOP)
        self.assertEqual(outcome.route_source, bp.SOURCE_B3)
        self.assertSameRoute(outcome.route, b3)
        self.assertSameRoute(outcome.b2_route, b2)
        self.assertLess(outcome.route.total_cost, outcome.b2_route.total_cost)
        self.assertEqual(outcome.incumbents_received, 2)
        self.assertReplayValid(outcome.route)
        self.assertReplayValid(outcome.b2_route, "prefix_b2")
        self.assertTrue(outcome.deliverable)
        self.assertNothingProven(outcome)
        self.assertIsNone(outcome.planner_metadata)
        self.assertCleanTeardown(outcome)

    def test_no_incumbent_at_the_hard_stop_is_an_honest_bounded_failure(self):
        outcome = self.run_plan(self.planner("real", {"stall_at_call": 1}), oracle()["input"])
        self.assertEqual(outcome.termination, bp.TERM_HARD_BACKSTOP)
        self.assertIsNone(outcome.route)
        self.assertIsNone(outcome.diagnostic_route)
        self.assertEqual((outcome.route_status, outcome.deliverable, outcome.incumbents_received),
                         ("none", False, 0))
        self.assertNothingProven(outcome)
        self.assertIn("No complete route found within budget (hard backstop)", outcome.execution_message)
        self.assertIn("non-existence is not proven", outcome.execution_message)
        self.assertNotIn("impossible", outcome.execution_message.lower())
        self.assertCleanTeardown(outcome)

    def test_a_crash_during_b3_keeps_the_b2_route(self):
        data = oracle()
        outcome = self.run_plan(self.planner("real", {"crash_at_call": 3}), data["input"])
        self.assertEqual(outcome.termination, bp.TERM_WORKER_FAILED)
        self.assertIn("without a FINAL", outcome.failure_detail)
        self.assertTrue(outcome.deliverable)
        self.assertEqual(outcome.route_source, bp.SOURCE_B2)
        self.assertSameRoute(outcome.route, data["seen"][0][1])
        self.assertReplayValid(outcome.route, "prefix_b2")
        self.assertNothingProven(outcome)
        self.assertEqual(outcome.cleanup.exit_code, 7)
        self.assertCleanTeardown(outcome)


class ScriptedProtocol(ProcessCase):
    def test_truncated_frame_keeps_the_earlier_incumbent(self):
        partial = pw.encode_frame(2, pw.KIND_INCUMBENT, {"phase": "b3", "route": fake_record(40.0)})[:-30]
        steps = PRELUDE + [
            {"op": "incumbent", "phase": "b2", "record": fake_record(50.0)},
            {"op": "raw", "hex": partial.hex()}, {"op": "crash", "code": 5}]
        outcome = self.run_plan(self.scripted(steps))
        self.assertEqual(outcome.termination, bp.TERM_WORKER_FAILED)
        self.assertIn("truncated", outcome.failure_detail)
        self.assertEqual((outcome.route.total_cost, outcome.route_source), (50.0, bp.SOURCE_B2))
        self.assertEqual(outcome.incumbents_received, 1)
        self.assertCleanTeardown(outcome)

    def test_protocol_violations_fail_the_worker_but_keep_what_was_verified(self):
        good = PRELUDE + [{"op": "incumbent", "phase": "b2", "record": fake_record(50.0)}]
        violations = {
            "oversized": [{"op": "oversized"}],
            "seq gap": [{"op": "frame", "seq": 9, "kind": "INCUMBENT",
                         "payload": {"phase": "b3", "route": fake_record(40.0)}}],
            "bad json": [{"op": "raw", "hex": b'{"seq":2,"kind":\n'.hex()}],
            "schema": [{"op": "incumbent", "phase": "b3", "record": {"nope": 1}}],
            "not cheaper": [{"op": "incumbent", "phase": "b3", "record": fake_record(50.0)}],
            "frame after final": [{"op": "final", "payload": fake_final("exhausted")},
                                  {"op": "incumbent", "phase": "b3", "record": fake_record(1.0)}],
        }
        for name, extra in violations.items():
            outcome = self.run_plan(self.scripted(good + extra + [{"op": "stall"}]))
            # A frame after a valid FINAL is simply never read: the FINAL decided the run.
            expected = bp.TERM_SEARCH_FINISHED if name == "frame after final" else bp.TERM_WORKER_FAILED
            self.assertEqual(outcome.termination, expected, name)
            self.assertEqual(outcome.route.total_cost, 50.0, name)
            self.assertLess(outcome.timing.decided_s, FAST.hard_stop_s, f"{name}: must not wait for the hard stop")
            if expected == bp.TERM_WORKER_FAILED:
                self.assertNothingProven(outcome)
            self.assertCleanTeardown(outcome)

    def test_improved_incumbent_retention_is_deterministic(self):
        steps = PRELUDE + [
            {"op": "incumbent", "phase": "b2", "record": fake_record(50.0)},
            {"op": "incumbent", "phase": "b3", "record": fake_record(40.0)},
            {"op": "stall"}]
        outcome = self.run_plan(self.scripted(steps))
        self.assertEqual(outcome.termination, bp.TERM_HARD_BACKSTOP)
        self.assertEqual((outcome.route.total_cost, outcome.route_source), (40.0, bp.SOURCE_B3))
        self.assertEqual(outcome.b2_route.total_cost, 50.0)
        self.assertNothingProven(outcome)
        self.assertCleanTeardown(outcome)
        crash = PRELUDE + steps[2:4] + [{"op": "crash", "code": 4}]
        outcome = self.run_plan(self.scripted(crash))
        self.assertEqual((outcome.termination, outcome.route.total_cost), (bp.TERM_WORKER_FAILED, 40.0))

    def test_a_stray_print_cannot_corrupt_the_frame_channel(self):
        steps = PRELUDE + [
            {"op": "print", "text": "noise on stdout"},
            {"op": "incumbent", "phase": "b2", "record": fake_record(50.0)},
            {"op": "print", "text": "more noise"},
            {"op": "final", "payload": fake_final("first_complete_route", fake_record(50.0))}]
        outcome = self.run_plan(self.scripted(steps))
        self.assertEqual(outcome.termination, bp.TERM_SEARCH_FINISHED)
        self.assertEqual(outcome.route_source, bp.SOURCE_FINAL)
        self.assertTrue(outcome.deliverable)
        self.assertCleanTeardown(outcome)

    def test_a_route_that_fails_verification_is_never_delivered(self):
        steps = PRELUDE + [{"op": "incumbent", "phase": "b2", "record": fake_record(5.0)},
                           {"op": "stall"}]
        outcome = self.run_plan(self.scripted(steps, verifier=bp.verify_route_record))
        self.assertEqual(outcome.termination, bp.TERM_WORKER_FAILED)
        self.assertEqual(outcome.failure_detail, "route_verification_failed")
        self.assertIsNone(outcome.route)
        self.assertIsNone(outcome.diagnostic_route)
        self.assertFalse(outcome.deliverable)
        self.assertTrue(outcome.verification_failures)
        self.assertNothingProven(outcome)
        self.assertCleanTeardown(outcome)

    def test_a_final_claiming_more_than_the_incumbents_support_is_rejected(self):
        steps = PRELUDE + [
            {"op": "incumbent", "phase": "b2", "record": fake_record(50.0)},
            {"op": "final", "payload": fake_final("exhausted", fake_record(70.0), optimality_proven=True)}]
        outcome = self.run_plan(self.scripted(steps))
        # the FINAL route is costlier than an incumbent the worker itself reported
        self.assertEqual(outcome.termination, bp.TERM_WORKER_FAILED)
        self.assertIsNone(outcome.route)
        self.assertFalse(outcome.optimality_proven)
        self.assertIn("costlier", " ".join(outcome.verification_failures))

    def test_a_partial_result_is_diagnostic_metadata_only(self):
        steps = PRELUDE + [{"op": "final", "payload": fake_final(
            "wall_clock_checkpoint_limit", route_status="partial_only", message="partial")}]
        outcome = self.run_plan(self.scripted(steps, verifier=bp.verify_route_record))
        self.assertEqual(outcome.termination, bp.TERM_PLANNER_BUDGET)
        self.assertEqual(outcome.planner_metadata["route_status"], "partial_only")
        self.assertEqual((outcome.route, outcome.diagnostic_route, outcome.route_status, outcome.deliverable),
                         (None, None, "none", False))
        self.assertNothingProven(outcome)
        self.assertCleanTeardown(outcome)

    def test_a_partial_route_claimed_as_complete_is_never_returned(self):
        partial = {**fake_record(5.0, order=()), "skipped_obstacle_ids": [1]}
        for steps in (
            PRELUDE + [{"op": "final", "payload": fake_final("exhausted", partial)}],
            PRELUDE + [{"op": "incumbent", "phase": "b2", "record": partial}, {"op": "stall"}],
        ):
            outcome = self.run_plan(self.scripted(steps, verifier=bp.verify_route_record))
            self.assertEqual(outcome.termination, bp.TERM_WORKER_FAILED)
            self.assertEqual((outcome.route, outcome.diagnostic_route, outcome.route_status, outcome.deliverable),
                             (None, None, "none", False))
            self.assertNothingProven(outcome)
            self.assertCleanTeardown(outcome)

    def test_an_error_frame_is_reported_by_class_name_only(self):
        steps = PRELUDE + [{"op": "incumbent", "phase": "b2", "record": fake_record(50.0)},
                           {"op": "error", "name": "ValueError"}]
        outcome = self.run_plan(self.scripted(steps))
        self.assertEqual((outcome.termination, outcome.failure_detail),
                         (bp.TERM_WORKER_FAILED, "worker_error: ValueError"))
        self.assertEqual(outcome.route.total_cost, 50.0)


class Timing(ProcessCase):
    def test_the_search_budget_is_what_is_left_of_the_r0_anchored_target(self):
        spec_path = Path(self._tmp.name) / "spec.json"
        steps = [{"op": "hello"}, {"op": "read_spec", "save": str(spec_path)},
                 {"op": "final", "payload": fake_final("exhausted", optimality_proven=False)}]
        outcome = self.run_plan(self.scripted(steps, SCALED), elapsed=1.5)
        sent = outcome.timing.search_budget_s
        self.assertEqual(json.loads(spec_path.read_text(encoding="utf-8"))["search_budget_s"], sent)
        # R0 was 1.5 s ago, so at most target - 1.5 - margin remains; start-up time is deducted too.
        self.assertLessEqual(sent, SCALED.planning_target_s - 1.5 - SCALED.startup_margin_s)
        self.assertAlmostEqual(sent, SCALED.planning_target_s - outcome.timing.spec_sent_s
                               - SCALED.startup_margin_s, delta=0.25)
        self.assertGreater(outcome.timing.spec_sent_s, 1.5)
        self.assertEqual(outcome.termination, bp.TERM_SEARCH_FINISHED)
        self.assertCleanTeardown(outcome)

    def test_no_time_left_to_start_runs_nothing_and_spawns_nothing(self):
        with mock.patch.object(bp.wc, "start_worker") as start:
            outcome = self.run_plan(self.scripted(PRELUDE, SCALED), elapsed=3.5)
        start.assert_not_called()
        self.assertEqual(outcome.termination, bp.TERM_STARTUP_OVERRAN)
        self.assertFalse(outcome.cleanup.worker_started)
        self.assertIsNone(outcome.timing.spec_sent_s)
        self.assertEqual((outcome.route, outcome.deliverable, outcome.route_status), (None, False, "none"))
        self.assertNothingProven(outcome)
        self.assertTrue(outcome.cleanup.ok)
        self.assertEqual(self.created, [])

    def test_a_slow_worker_start_is_stopped_without_ever_receiving_a_spec(self):
        steps = [{"op": "sleep", "s": 30.0}] + PRELUDE
        started = time.monotonic()
        outcome = self.run_plan(self.scripted(steps, SCALED))
        self.assertEqual(outcome.termination, bp.TERM_STARTUP_OVERRAN)
        self.assertIsNone(outcome.timing.spec_sent_s)
        self.assertIsNone(outcome.timing.hello_s)
        latest = SCALED.planning_target_s - SCALED.startup_margin_s - SCALED.min_search_budget_s
        self.assertGreaterEqual(outcome.timing.decided_s, latest)
        self.assertLess(time.monotonic() - started, latest + 3.0)
        self.assertFalse(outcome.deliverable)
        self.assertNothingProven(outcome)
        self.assertCleanTeardown(outcome)

    def test_the_hard_stop_is_anchored_to_r0_not_to_the_worker_start(self):
        steps = PRELUDE + [{"op": "incumbent", "phase": "b2", "record": fake_record(50.0)}, {"op": "stall"}]
        outcome = self.run_plan(self.scripted(steps, SCALED), elapsed=3.0)   # R0 was 3 s ago
        self.assertEqual(outcome.termination, bp.TERM_HARD_BACKSTOP)
        self.assertGreaterEqual(outcome.timing.decided_s, SCALED.hard_stop_s)
        self.assertLess(outcome.timing.decided_s, SCALED.hard_stop_s + 1.0)
        self.assertEqual(outcome.route.total_cost, 50.0)
        self.assertCleanTeardown(outcome)

    def test_a_plan_that_returns_after_the_deadline_is_not_delivered(self):
        steps = PRELUDE + [{"op": "incumbent", "phase": "b2", "record": fake_record(50.0)},
                           {"op": "final", "payload": fake_final("first_complete_route", fake_record(50.0))}]
        tight = bp.ExecutionPolicy(planning_target_s=1.5, hard_stop_s=2.0, verification_deadline_s=3.0,
                                   return_target_s=3.5, return_deadline_s=4.0, request_ceiling_s=5.0,
                                   min_search_budget_s=0.5, death_wait_s=0.5)

        def slow_verifier(record, inp, config=None, *, deadline=None, clock=None):
            time.sleep(4.3)                     # verification alone overruns the return deadline
            return stub_verifier(record, inp)

        planner = self.scripted(steps, tight, verifier=slow_verifier)
        outcome = self.run_plan(planner)
        self.assertGreater(outcome.timing.returned_s, tight.return_deadline_s)
        self.assertEqual(outcome.delivery, bp.DELIVERY_CEILING_EXCEEDED)
        self.assertFalse(outcome.deliverable)
        self.assertIsNone(outcome.route)
        self.assertEqual(outcome.route_status, "none")
        self.assertEqual(outcome.diagnostic_route.total_cost, 50.0)
        self.assertFalse(outcome.optimality_proven)
        self.assertIn("not delivered", outcome.execution_message)
        self.assertTrue(outcome.cleanup.ok)
        self.assertCleanTeardown(outcome)
        # Everything was verified dead and cleaned, so a late return withholds the
        # route but does NOT make the executor unhealthy: the next plan runs.
        self.assertTrue(planner.healthy)
        planner.verifier = stub_verifier
        again = self.run_plan(planner)
        self.assertEqual((again.termination, again.deliverable), (bp.TERM_SEARCH_FINISHED, True))


class TreeTerminationAndIsolation(ProcessCase):
    def test_the_worker_tree_dies_and_an_unrelated_process_survives(self):
        pidfile = Path(self._tmp.name) / "grandchild.pid"
        sentinel = subprocess.Popen(SLEEPER)                    # owned by this test, never contained
        try:
            steps = PRELUDE + [{"op": "grandchild", "pidfile": str(pidfile)},
                               {"op": "incumbent", "phase": "b2", "record": fake_record(50.0)},
                               {"op": "stall"}]
            outcome = self.run_plan(self.scripted(steps))
            grandchild = int(pidfile.read_text(encoding="utf-8"))
            self.assertEqual(outcome.termination, bp.TERM_HARD_BACKSTOP)
            self.assertTrue(outcome.cleanup.tree_dead)
            deadline = time.monotonic() + 10
            while process_alive(grandchild) and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertFalse(process_alive(grandchild), "a descendant of the worker survived the hard stop")
            self.assertTrue(process_alive(sentinel.pid), "an unrelated process was terminated")
            self.assertIsNone(sentinel.poll())
            self.assertEqual(outcome.route.total_cost, 50.0)
            self.assertCleanTeardown(outcome)
        finally:
            sentinel.kill()                                     # through its own handle
            sentinel.wait(timeout=10)


class GracefulFinishSweepsTheTree(ProcessCase):
    def test_a_descendant_left_behind_by_a_finished_worker_is_killed(self):
        pidfile = Path(self._tmp.name) / "grandchild.pid"
        steps = PRELUDE + [{"op": "grandchild", "pidfile": str(pidfile)},
                           {"op": "incumbent", "phase": "b2", "record": fake_record(50.0)},
                           {"op": "final", "payload": fake_final("first_complete_route", fake_record(50.0))},
                           {"op": "exit", "code": 0}]
        outcome = self.run_plan(self.scripted(steps))
        grandchild = int(pidfile.read_text(encoding="utf-8"))
        self.assertEqual((outcome.termination, outcome.route_source), (bp.TERM_SEARCH_FINISHED, bp.SOURCE_FINAL))
        self.assertEqual(outcome.cleanup.exit_code, 0)          # the worker itself finished normally
        self.assertTrue(outcome.deliverable)
        deadline = time.monotonic() + 10
        while process_alive(grandchild) and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertFalse(process_alive(grandchild), "a descendant outlived a graceful finish")
        self.assertCleanTeardown(outcome)


class Cancellation(ProcessCase):
    def test_a_cancel_the_worker_acknowledges_is_a_planner_stop(self):
        steps = PRELUDE + [{"op": "incumbent", "phase": "b2", "record": fake_record(50.0)},
                           {"op": "wait_cancel", "timeout": 20.0},
                           {"op": "final", "payload": fake_final("cancelled")}]
        cancel = threading.Event()
        threading.Timer(0.6, cancel.set).start()
        outcome = self.run_plan(self.scripted(steps, SCALED), cancel=cancel)
        self.assertEqual(outcome.termination, bp.TERM_CANCELLED)
        self.assertEqual(outcome.planner_metadata["stop_reason"], rps.STOP_CANCELLED)
        self.assertEqual(outcome.route.total_cost, 50.0)
        self.assertLess(outcome.timing.decided_s, 2.5)           # acknowledged, not escalated
        self.assertEqual(outcome.cleanup.exit_code, 0)
        self.assertNothingProven(outcome)
        self.assertCleanTeardown(outcome)

    def test_an_ignored_cancel_is_escalated_before_the_hard_stop(self):
        steps = PRELUDE + [{"op": "incumbent", "phase": "b2", "record": fake_record(50.0)}, {"op": "stall"}]
        cancel = threading.Event()
        threading.Timer(0.6, cancel.set).start()
        outcome = self.run_plan(self.scripted(steps, SCALED), cancel=cancel)
        self.assertEqual((outcome.termination, outcome.failure_detail), (bp.TERM_CANCELLED, "cancel_escalated"))
        self.assertLess(outcome.timing.decided_s, SCALED.hard_stop_s)
        self.assertEqual(outcome.route.total_cost, 50.0)
        self.assertCleanTeardown(outcome)


class FailClosed(ProcessCase):
    STEPS = PRELUDE + [{"op": "incumbent", "phase": "b2", "record": fake_record(50.0)}, {"op": "stall"}]

    def assertFailedClosed(self, planner, outcome):
        self.assertFalse(outcome.deliverable)
        self.assertIsNone(outcome.route)
        self.assertEqual(outcome.route_status, "none")
        self.assertEqual(outcome.diagnostic_route.total_cost, 50.0)      # diagnostics only
        self.assertFalse(outcome.cleanup.ok)
        self.assertFalse(planner.healthy)
        self.assertNothingProven(outcome)
        self.assertIn("nothing is delivered", outcome.execution_message)
        # Later requests fail until restart, and never start a worker.
        with mock.patch.object(bp.wc, "start_worker") as start:
            again = self.run_plan(planner)
        start.assert_not_called()
        self.assertEqual((again.termination, again.deliverable, again.route), (bp.TERM_EXECUTOR_FAILED, False, None))
        self.assertEqual(again.failure_detail, "executor_unhealthy")

    def assertTempdirKept(self, outcome):
        self.assertFalse(outcome.cleanup.tempdir_removed)
        self.assertTrue(all(os.path.isdir(path) for path in self.created),
                        "the temporary directory was deleted before the worker was verified dead")

    def test_unverifiable_worker_death_fails_closed(self):
        planner = self.scripted(self.STEPS)
        with mock.patch.object(wc.ContainedWorker, "wait_dead", return_value=False):
            outcome = self.run_plan(planner)
        self.assertFalse(outcome.cleanup.death_verified)
        self.assertFalse(outcome.cleanup.tree_dead)
        self.assertFalse(outcome.cleanup.reader_stopped)
        self.assertTempdirKept(outcome)
        self.assertFailedClosed(planner, outcome)

    def test_a_tree_that_cannot_be_verified_empty_fails_closed(self):
        planner = self.scripted(self.STEPS)
        with mock.patch.object(wc.ContainedWorker, "tree_dead", return_value=False):
            outcome = self.run_plan(planner)
        self.assertTrue(outcome.cleanup.death_verified)
        self.assertFalse(outcome.cleanup.tree_dead)
        self.assertTempdirKept(outcome)
        self.assertFailedClosed(planner, outcome)

    def test_a_reader_that_does_not_stop_fails_closed(self):
        planner = self.scripted(self.STEPS)
        with mock.patch.object(bp._FrameReader, "is_alive", return_value=True):
            outcome = self.run_plan(planner)
        self.assertTrue(outcome.cleanup.tree_dead)
        self.assertFalse(outcome.cleanup.reader_stopped)
        self.assertFailedClosed(planner, outcome)

    def test_a_temporary_directory_that_cannot_be_removed_fails_closed(self):
        planner = self.scripted(self.STEPS)
        with mock.patch.object(bp.shutil, "rmtree"):
            outcome = self.run_plan(planner)
        self.assertFalse(outcome.cleanup.tempdir_removed)
        self.assertTrue(outcome.cleanup.death_verified)
        self.assertFailedClosed(planner, outcome)

    @unittest.skipUnless(sys.platform == "win32", "the job handle close is what ends a worker the OS would not terminate")
    def test_a_refused_tree_termination_fails_closed(self):
        planner = self.scripted(self.STEPS)
        with mock.patch.object(wc.ContainedWorker, "terminate_tree", side_effect=PermissionError("refused")):
            outcome = self.run_plan(planner)
        self.assertTrue(outcome.cleanup.terminate_error)
        self.assertTempdirKept(outcome)
        self.assertFailedClosed(planner, outcome)

    def test_a_containment_failure_fails_closed_and_never_sends_work(self):
        planner = self.scripted(PRELUDE)
        with mock.patch.object(bp.wc, "start_worker", side_effect=wc.ContainmentError("no job")):
            outcome = self.run_plan(planner)
        self.assertEqual(outcome.termination, bp.TERM_EXECUTOR_FAILED)
        self.assertFalse(outcome.deliverable)
        self.assertFalse(planner.healthy)
        self.assertFalse(outcome.cleanup.worker_started)
        self.assertTempdirsGone()

    def test_an_unreaped_gated_worker_keeps_its_directory(self):
        planner = self.scripted(PRELUDE)
        error = wc.ContainmentError("no job, and the gated worker would not die", worker_reaped=False)
        with mock.patch.object(bp.wc, "start_worker", side_effect=error):
            outcome = self.run_plan(planner)
        self.assertEqual(outcome.termination, bp.TERM_EXECUTOR_FAILED)
        self.assertFalse(planner.healthy)
        self.assertFalse(outcome.cleanup.tempdir_removed)
        self.assertTrue(all(os.path.isdir(path) for path in self.created))


class Busy(ProcessCase):
    def test_concurrent_plans_are_refused_not_queued(self):
        steps = PRELUDE + [{"op": "sleep", "s": 1.0},
                           {"op": "final", "payload": fake_final("exhausted")}]
        planner = self.scripted(steps)
        holder = {}
        thread = threading.Thread(target=lambda: holder.update(outcome=self.run_plan(planner)))
        thread.start()
        try:
            time.sleep(0.4)
            with self.assertRaises(bp.ExecutorBusyError):
                self.run_plan(planner)
        finally:
            thread.join(timeout=30)
        self.assertEqual(holder["outcome"].termination, bp.TERM_SEARCH_FINISHED)
        self.assertTrue(planner.healthy)


if __name__ == "__main__":
    unittest.main()
