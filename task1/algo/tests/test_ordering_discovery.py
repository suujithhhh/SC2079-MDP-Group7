"""Four-obstacle fixture discovery evaluator (`algo/benchmarks/ordering_discovery.py`).

Every record here is SYNTHETIC test data built in this file: no planner runs. The tests pin
the fixed stage, criteria, tier, confirmation and stop rules, and that the evaluator refuses
input that could hide an attempt.

Run from algo/:
    python -m unittest tests.test_ordering_discovery -v
"""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

from benchmarks import fixtures as bfx  # noqa: E402
from benchmarks import ordering_discovery as od  # noqa: E402
from benchmarks import prefix_modes  # noqa: E402
from benchmarks import report as rep  # noqa: E402

SHA = "0123456789abcdef0123456789abcdef01234567"
OTHER_SHA = "fedcba9876543210fedcba9876543210fedcba98"
IDS = od.CANDIDATE_IDS
WORK = {"leg_requests": 40, "cache_hits": 10, "leg_calls": 30, "lattice_expansions": 9000, "nodes_expanded": 50,
        "children_generated": 90, "bound_evaluations": 120, "bound_prunes": 7, "dominance_prunes": 2,
        "frontier_peak": 12, "unique_prefixes_reached": 30, "unique_complete_orders": 6,
        "complete_routes_found": 6, "complete_leaf_discoveries": 8, "incumbent_updates": 1,
        "leg_outcome_counts": [["success", 30]]}
# What the runner records when no operator flag is given: `SearchLimits` has its own default
# memory cap (`max_frontier_entries`), which is not a limit somebody set.
NO_LIMITS = prefix_modes.limits_record({})


def legs(turning=3):
    return [{"commands": [{"direction": "FL" if i < turning else "F", "value": 10}]} for i in range(4)]


def rec(fixture_id, mode, status="complete", **kw):
    """One synthetic runner record. Overrides: wall, cost, seed, unique, turning, stop, hit, proven,
    unresolved, bviol, replay, inv, death, sha, dirty, limits, timeout, alternatives, hash, notworse,
    work, orders."""
    finished = status in ("complete", "partial", "no_route")
    normalized = {"route_kind": {"complete": "complete", "partial": "partial", "no_route": "none"}.get(status),
                  "total_cost": kw.get("cost", 400.0), "total_distance_cm": 300.0,
                  "obstacle_order": [1, 2, 3, 4], "legs": legs(kw.get("turning", 3))}
    if mode == "pb3":
        raw = {"stop_reason": kw.get("stop", "exhausted"), "work_limit_hit": kw.get("hit"),
               "optimality_proven": kw.get("proven", True), "unresolved_branch_count": kw.get("unresolved", 0),
               "bound_violation_count": kw.get("bviol", 0), "lower_bound_certified": True,
               "work": kw.get("work", {**WORK, "unique_complete_orders": kw.get("unique", 6)})}
    else:
        raw = {"is_valid": True, "is_complete": True, "orders_considered": 24,
               "valid_orders": kw.get("orders", 6), "rejected_orders": 12, "pruned_orders": 6,
               "failure_reason": None}
    validation = {
        "normalized": normalized, "raw": raw,
        "b2_normalized": ({"route_kind": "complete", "total_cost": kw.get("seed", 400.0)}
                          if mode == "pb3" else None),
        "replay_violations": list(kw.get("replay", [])), "b2_replay_violations": [],
        "invariant_violations": list(kw.get("inv", [])),
        "b3_not_worse_than_b2": kw.get("notworse", True) if mode == "pb3" else None,
    } if finished else None
    return {
        "kind": "benchmark_result", "started_at_utc": "2026-01-01T00:00:00Z", "fixture_id": fixture_id,
        "mode": mode, "alternatives": kw.get("alternatives", False), "profile": "none", "tracemalloc": False,
        "timeout_s": kw.get("timeout", od.TIMEOUT_S.get(mode, 60)), "status": status,
        "limits": kw.get("limits", NO_LIMITS if mode == "pb3" else None),
        "wall_time_s": kw.get("wall", 20.0), "planning_time_s": 19.0,
        "result_hash": kw.get("hash", f"hash-{mode}") if finished else None,
        "cleanup": {"death_verified": kw.get("death", True), "tempdir_removed": True,
                    "timeout_kind": "harness_hard_timeout" if status == "timeout" else None},
        "machine": {"system": "Test"}, "git": {"sha": kw.get("sha", SHA), "dirty": kw.get("dirty", False)},
        "validation": validation,
    }


def tier1(fixture_id, **over):
    """pb3 + b3 + b2 records of a candidate that reaches tier 1 (B2 seed costs more than B3)."""
    return [rec(fixture_id, "pb3", seed=410.0, **over), rec(fixture_id, "b3"), rec(fixture_id, "b2", cost=400.0)]


def tier2(fixture_id):
    return [rec(fixture_id, "pb3"), rec(fixture_id, "b3"), rec(fixture_id, "b2")]


def s1_fail(fixture_id, **over):
    return [rec(fixture_id, "pb3", **over)]


def confirm_pair(fixture_id, **over):
    """The confirmation runs that were due: old B3 is only run after prefix B3 confirmed."""
    runs = [rec(fixture_id, "pb3", seed=410.0, **over.get("pb3", {}))]
    if not over.get("pb3"):
        runs.append(rec(fixture_id, "b3", **over.get("b3", {})))
    return runs


def by_id(ledger):
    return {c["fixture_id"]: c for c in ledger["candidates"]}


class Constants(unittest.TestCase):
    def test_rules_are_the_approved_ones(self):
        self.assertEqual(od.TIMEOUT_S, {"pb3": 300, "b3": 120, "b2": 60})
        self.assertEqual(od.WALL_LIMIT_S, {"pb3": 150.0, "b3": 90.0})
        self.assertEqual((od.MIN_UNIQUE_COMPLETE_ORDERS, od.MIN_TURNING_LEGS), (2, 2))
        self.assertEqual(od.TURNING_COMMANDS, frozenset({"FL", "FR", "BL", "BR"}))
        self.assertLessEqual(max(od.TIMEOUT_S.values()), 600)
        self.assertLessEqual(od.TOTAL_CAP_S, 3600)
        self.assertEqual(od.MAX_SITTINGS, 2)
        self.assertEqual(od.CANDIDATE_IDS, tuple(f"E_ordering_04_c{k}" for k in range(1, 7)))

    def test_the_unset_default_is_what_the_runner_records(self):
        self.assertEqual(NO_LIMITS, {"max_leg_calls": None, "max_lattice_expansions": None, "max_nodes": None,
                                     "max_frontier_entries": 200000, "wall_clock_checkpoint_limit_s": None,
                                     "cancel_after_legs": None})
        self.assertEqual(od.record_problems(rec(IDS[0], "pb3")), [])

    def test_the_plan_lists_each_candidate_with_unlimited_off_commands(self):
        plan = od.first_run_plan()
        self.assertEqual([p["fixture_id"] for p in plan], list(IDS))
        for candidate in plan:
            commands = [s["command"] for s in candidate["stages"]]
            self.assertEqual([c[c.index("--mode") + 1] for c in commands], ["pb3", "b3", "b2"])
            for command in commands:
                self.assertEqual(command[command.index("--alternatives") + 1], "off")
                for flag in ("--max-leg-calls", "--max-expansions", "--max-nodes", "--max-frontier",
                             "--checkpoint-s", "--cancel-after-legs", "--profile", "--tracemalloc",
                             "--allow-large-baseline"):
                    self.assertNotIn(flag, command)
        self.assertEqual([int(c[c.index("--timeout") + 1]) for c in (s["command"] for s in plan[0]["stages"])],
                         [300, 120, 60])

    def test_stop_reasons_match_the_report_and_the_search(self):
        self.assertEqual(rep.CLOSED_STOPS, frozenset({"exhausted", "bound_closed"}))


class NextAction(unittest.TestCase):
    def test_nothing_recorded_is_refused_not_started_silently(self):
        with self.assertRaises(od.DiscoveryInputError):
            od.evaluate([])

    def test_first_action_is_c1_pb3_after_a_record_exists(self):
        ledger = od.evaluate(s1_fail(IDS[0], unique=1))
        self.assertEqual(ledger["status"], "incomplete")
        self.assertEqual(ledger["next_action"]["fixture_id"], IDS[1])
        self.assertEqual(ledger["next_action"]["mode"], "pb3")

    def test_a_passing_s1_asks_for_the_old_b3_then_the_old_b2(self):
        first = od.next_action([rec(IDS[0], "pb3", seed=410.0)])
        self.assertEqual((first["fixture_id"], first["mode"], first["file"]), (IDS[0], "b3", "primary"))
        second = od.next_action([rec(IDS[0], "pb3", seed=410.0), rec(IDS[0], "b3")])
        self.assertEqual((second["fixture_id"], second["mode"]), (IDS[0], "b2"))

    def test_a_finished_search_reports_done(self):
        done = od.next_action([r for f in IDS for r in s1_fail(f, unique=1)])
        self.assertEqual((done["action"], done["status"]), ("done", "no_go"))


class Stage1(unittest.TestCase):
    def s1(self, **kw):
        ledger = od.evaluate(s1_fail(IDS[0], **kw))
        candidate = by_id(ledger)[IDS[0]]
        return candidate["stages"]["S1"], candidate

    def test_a_certified_complete_proven_run_passes(self):
        stage, _ = self.s1(seed=410.0)
        self.assertEqual((stage["status"], stage["reason"]), ("pass", None))

    def test_each_check_can_reject(self):
        for label, kw, reason in (
            ("not proven", {"proven": False}, "certified"),
            ("work limit stop", {"stop": "work_limit", "hit": "max_leg_calls", "proven": False}, "certified"),
            ("first-complete stop", {"stop": "first_complete_route"}, "certified"),
            ("unresolved branches", {"unresolved": 1}, "certified"),
            ("unproven with unresolved", {"proven": False, "unresolved": 8}, "certified"),
            ("slow", {"wall": 150.5}, "wall_within_limit"),
            ("one order only", {"unique": 1}, "O1"),
            ("straight legs", {"turning": 1}, "O2"),
            ("b3 worse than b2", {"notworse": False}, "b3_not_worse_than_b2"),
        ):
            with self.subTest(label):
                stage, candidate = self.s1(**kw)
                self.assertEqual((stage["status"], stage["reason"]), ("fail", reason))
                self.assertEqual(candidate["outcome"], "rejected_S1")
                self.assertEqual(candidate["stages"]["S2"]["status"], "not_attempted")
                self.assertEqual(candidate["stages"]["S3"]["status"], "not_attempted")

    def test_the_wall_limit_boundary_is_inclusive(self):
        self.assertEqual(self.s1(wall=150.0)[0]["status"], "pass")

    def test_the_certification_is_the_reports_own_definition(self):
        for kw in ({}, {"proven": False}, {"unresolved": 1}, {"stop": "work_limit", "hit": "max_leg_calls"},
                   {"stop": "bound_closed"}, {"stop": "wall_clock_checkpoint_limit"}):
            record = rec(IDS[0], "pb3", seed=410.0, **kw)
            entry = od._entry(IDS[0], "pb3", record)
            stage, _ = self.s1(seed=410.0, **kw)
            self.assertEqual(stage["checks"]["certified"], rep._certified(entry, "b3"), kw)

    def test_a_harness_timeout_rejects_and_keeps_the_record(self):
        candidate = by_id(od.evaluate([rec(IDS[0], "pb3", "timeout")]))[IDS[0]]
        self.assertEqual((candidate["stages"]["S1"]["status"], candidate["stages"]["S1"]["reason"]),
                         ("fail", "harness_timeout"))
        self.assertEqual(candidate["outcome"], "rejected_S1")

    def test_a_partial_route_is_not_complete(self):
        candidate = by_id(od.evaluate([rec(IDS[0], "pb3", "partial")]))[IDS[0]]
        self.assertEqual(candidate["stages"]["S1"]["reason"], "complete")


class Stage2And3(unittest.TestCase):
    def evaluate(self, *records):
        return by_id(od.evaluate(list(records)))[IDS[0]]

    def test_old_b3_with_equal_cost_passes_and_b2_is_a_witness_only(self):
        candidate = self.evaluate(rec(IDS[0], "pb3", seed=410.0), rec(IDS[0], "b3"),
                                  rec(IDS[0], "b2", "timeout"))
        self.assertEqual(candidate["stages"]["S2"]["status"], "pass")
        self.assertEqual(candidate["stages"]["S3"]["status"], "pass")
        self.assertEqual(candidate["tier"], 1)      # the seed route already costs more than B3's

    def test_a_cost_mismatch_is_a_hard_stop_not_a_rejection(self):
        ledger = od.evaluate([rec(IDS[0], "pb3"), rec(IDS[0], "b3", cost=400.0000000001)])
        self.assertEqual(ledger["status"], "stopped")
        self.assertIn({"fixture_id": IDS[0], "reason": "cost_mismatch"}, ledger["stops"])
        self.assertEqual(by_id(ledger)[IDS[0]]["outcome"], "stopped")
        self.assertIsNone(ledger["selected"])

    def test_one_ulp_is_a_mismatch(self):
        import math
        ledger = od.evaluate([rec(IDS[0], "pb3"), rec(IDS[0], "b3", cost=math.nextafter(400.0, 500.0))])
        self.assertEqual(ledger["status"], "stopped")

    def test_old_b3_timeout_and_slow_runs_reject(self):
        for label, record, reason in (("timeout", rec(IDS[0], "b3", "timeout"), "harness_timeout"),
                                      ("slow", rec(IDS[0], "b3", wall=90.5), "wall_within_limit")):
            with self.subTest(label):
                candidate = self.evaluate(rec(IDS[0], "pb3"), record)
                self.assertEqual((candidate["stages"]["S2"]["reason"], candidate["outcome"]),
                                 (reason, "rejected_S2"))
                self.assertEqual(candidate["stages"]["S3"]["status"], "not_attempted")

    def test_old_b3_that_is_not_complete_rejects(self):
        candidate = self.evaluate(rec(IDS[0], "pb3"), rec(IDS[0], "b3", "partial"))
        self.assertEqual(candidate["stages"]["S2"]["reason"], "complete")

    def test_o3_needs_a_strict_difference_from_either_witness(self):
        self.assertEqual(self.evaluate(*tier2(IDS[0]))["tier"], 2)
        self.assertEqual(self.evaluate(rec(IDS[0], "pb3", seed=400.0), rec(IDS[0], "b3"),
                                       rec(IDS[0], "b2", cost=400.5))["tier"], 1)
        self.assertEqual(self.evaluate(rec(IDS[0], "pb3", seed=399.0), rec(IDS[0], "b3"),
                                       rec(IDS[0], "b2", cost=400.0))["tier"], 2)
        detail = self.evaluate(rec(IDS[0], "pb3", seed=400.0), rec(IDS[0], "b3"),
                               rec(IDS[0], "b2", cost=400.5))["criteria"]["O3_detail"]
        self.assertEqual((detail["b2_seed_cost_above_b3"], detail["old_b2_cost_above_old_b3"]), (False, True))

    def test_a_partial_old_b2_is_no_o3_witness(self):
        candidate = self.evaluate(rec(IDS[0], "pb3"), rec(IDS[0], "b3"), rec(IDS[0], "b2", "partial", cost=999.0))
        self.assertEqual(candidate["tier"], 2)


class Selection(unittest.TestCase):
    def test_the_first_tier_one_candidate_ends_discovery_and_later_ones_are_not_attempted(self):
        records = s1_fail(IDS[0], unique=1) + tier1(IDS[1])
        ledger = od.evaluate(records)
        self.assertEqual(ledger["status"], "incomplete")
        self.assertEqual({k: ledger["next_action"][k] for k in ("fixture_id", "mode", "file")},
                         {"fixture_id": IDS[1], "mode": "pb3", "file": "confirm"})
        outcomes = {f: c["outcome"] for f, c in by_id(ledger).items()}
        self.assertEqual(outcomes[IDS[0]], "rejected_S1")
        self.assertEqual(outcomes[IDS[1]], "qualified_pending")
        for later in IDS[2:]:
            self.assertEqual(outcomes[later], "not_attempted")

    def test_a_confirmed_tier_one_candidate_is_selected(self):
        ledger = od.evaluate(tier1(IDS[0]), confirm_records=confirm_pair(IDS[0]))
        self.assertEqual(ledger["status"], "selected")
        self.assertEqual(ledger["selected"], {"fixture_id": IDS[0], "tier": 1, "candidate_number": 1,
                                              "confirmation_status": "passed"})
        self.assertIsNone(ledger["next_action"])
        self.assertEqual(by_id(ledger)[IDS[0]]["outcome"], "selected")
        self.assertEqual(by_id(ledger)[IDS[0]]["confirmation"]["runs"]["pb3"]["differences"], [])
        self.assertEqual(ledger["discovery_commit"], SHA)

    def test_the_confirmation_runs_pb3_then_b3(self):
        first = od.next_action(tier1(IDS[0]))
        self.assertEqual((first["mode"], first["file"]), ("pb3", "confirm"))
        second = od.next_action(tier1(IDS[0]), [rec(IDS[0], "pb3", seed=410.0)])
        self.assertEqual((second["mode"], second["file"]), ("b3", "confirm"))

    def test_a_tier_two_candidate_does_not_end_discovery(self):
        ledger = od.evaluate(tier2(IDS[0]))
        self.assertEqual((ledger["status"], ledger["next_action"]["fixture_id"]), ("incomplete", IDS[1]))
        self.assertEqual(by_id(ledger)[IDS[0]]["outcome"], "qualified_pending")

    def test_a_later_tier_one_beats_an_earlier_tier_two(self):
        records = tier2(IDS[0]) + tier1(IDS[1])
        ledger = od.evaluate(records, confirm_records=confirm_pair(IDS[1]))
        self.assertEqual((ledger["status"], ledger["selected"]["fixture_id"], ledger["selected"]["tier"]),
                         ("selected", IDS[1], 1))
        self.assertEqual(by_id(ledger)[IDS[0]]["outcome"], "qualified_not_selected")

    def test_tier_two_is_the_fallback_after_all_six_and_needs_confirmation(self):
        records = tier2(IDS[2]) + [r for f in IDS if f != IDS[2] for r in s1_fail(f, unique=1)]
        records.sort(key=lambda r: IDS.index(r["fixture_id"]))
        ledger = od.evaluate(records)
        self.assertEqual({k: ledger["next_action"][k] for k in ("fixture_id", "mode", "file")},
                         {"fixture_id": IDS[2], "mode": "pb3", "file": "confirm"})
        ledger = od.evaluate(records, confirm_records=confirm_pair(IDS[2]))
        self.assertEqual((ledger["status"], ledger["selected"]["tier"], ledger["selected"]["candidate_number"]),
                         ("selected", 2, 3))

    def test_all_six_failing_is_no_go_with_every_candidate_recorded(self):
        ledger = od.evaluate([r for f in IDS for r in s1_fail(f, unique=1)])
        self.assertEqual((ledger["status"], ledger["selected"], ledger["next_action"]), ("no_go", None, None))
        self.assertEqual([c["outcome"] for c in ledger["candidates"]], ["rejected_S1"] * 6)
        self.assertEqual([c["fixture_id"] for c in ledger["candidates"]], list(IDS))

    def test_a_timing_failure_on_confirmation_resumes_with_the_next_candidate(self):
        records = tier1(IDS[0])
        slow = confirm_pair(IDS[0], pb3={"wall": 151.0})
        ledger = od.evaluate(records, confirm_records=slow)
        self.assertEqual(ledger["status"], "incomplete")
        self.assertEqual(by_id(ledger)[IDS[0]]["outcome"], "rejected_on_confirmation")
        self.assertEqual(ledger["next_action"]["fixture_id"], IDS[1])
        self.assertEqual(ledger["next_action"]["file"], "primary")
        self.assertEqual(by_id(ledger)[IDS[0]]["confirmation"]["reason"], "pb3_ineligible_or_too_slow")

    def test_a_confirmation_timeout_is_a_timing_failure_not_a_stop(self):
        ledger = od.evaluate(tier1(IDS[0]), confirm_records=[rec(IDS[0], "pb3", "timeout")])
        self.assertEqual(by_id(ledger)[IDS[0]]["outcome"], "rejected_on_confirmation")
        self.assertEqual(ledger["stops"], [])

    def test_a_failed_tier_one_is_never_reused_as_the_fallback(self):
        records = tier1(IDS[0]) + [r for f in IDS[1:] for r in s1_fail(f, unique=1)]
        records.sort(key=lambda r: IDS.index(r["fixture_id"]))
        ledger = od.evaluate(records, confirm_records=confirm_pair(IDS[0], b3={"wall": 91.0}))
        self.assertEqual((ledger["status"], by_id(ledger)[IDS[0]]["outcome"]), ("no_go", "rejected_on_confirmation"))

    def test_a_differing_hash_cost_or_work_is_a_determinism_stop(self):
        for label, over in (("pb3 hash", {"pb3": {"hash": "other"}}), ("pb3 cost", {"pb3": {"cost": 401.0}}),
                            ("pb3 work", {"pb3": {"work": {**WORK, "leg_calls": 31}}}),
                            ("b3 hash", {"b3": {"hash": "other"}}), ("b3 orders", {"b3": {"orders": 7}})):
            with self.subTest(label):
                ledger = od.evaluate(tier1(IDS[0]), confirm_records=confirm_pair(IDS[0], **over))
                self.assertEqual(ledger["status"], "stopped")
                self.assertIn("determinism_mismatch", [s["reason"] for s in ledger["stops"]])
                self.assertIsNone(ledger["selected"])


class HardStops(unittest.TestCase):
    def stopped(self, *records, confirm=()):
        ledger = od.evaluate(list(records), confirm_records=list(confirm))
        self.assertEqual(ledger["status"], "stopped")
        self.assertIsNone(ledger["selected"])
        return [s["reason"] for s in ledger["stops"]]

    def test_replay_and_invariant_violations_stop(self):
        self.assertIn("replay_or_invariant_violation", self.stopped(rec(IDS[0], "pb3", replay=["chain: x"])))
        self.assertIn("replay_or_invariant_violation", self.stopped(rec(IDS[0], "pb3", inv=["proof: x"])))
        self.assertIn("replay_or_invariant_violation",
                      self.stopped(rec(IDS[0], "pb3"), rec(IDS[0], "b3", replay=["chain: x"])))

    def test_a_bound_violation_stops(self):
        self.assertIn("bound_violation", self.stopped(rec(IDS[0], "pb3", bviol=1)))

    def test_an_error_record_stops(self):
        self.assertIn("error_record", self.stopped(rec(IDS[0], "pb3", "error", death=False)))

    def test_an_unverified_worker_death_stops_even_for_a_timeout(self):
        self.assertIn("unverified_worker_death", self.stopped(rec(IDS[0], "pb3", "timeout", death=False)))
        self.assertIn("unverified_worker_death", self.stopped(rec(IDS[0], "pb3", death=False)))

    def test_a_stop_in_the_confirmation_run_stops(self):
        self.assertIn("replay_or_invariant_violation", self.stopped(
            *tier1(IDS[0]), confirm=[rec(IDS[0], "pb3", seed=410.0, replay=["chain: x"])]))

    def test_a_stop_never_offers_another_run(self):
        self.assertIsNone(od.evaluate([rec(IDS[0], "pb3", bviol=1)])["next_action"])


class Refusals(unittest.TestCase):
    def refused(self, records, confirm=(), fragment=""):
        with self.assertRaises(od.DiscoveryInputError) as caught:
            od.evaluate(list(records), confirm_records=list(confirm))
        self.assertIn(fragment, str(caught.exception))

    def test_dirty_or_mixed_provenance_is_refused(self):
        self.refused([rec(IDS[0], "pb3", dirty=True)], fragment="dirty")
        self.refused([rec(IDS[0], "pb3", dirty=None)], fragment="dirty")
        self.refused([rec(IDS[0], "pb3", sha="nothex")], fragment="git.sha")
        self.refused([rec(IDS[0], "pb3"), rec(IDS[0], "b3", sha=OTHER_SHA)], fragment="several commits")
        self.refused(tier1(IDS[0]), confirm=[rec(IDS[0], "pb3", seed=410.0, sha=OTHER_SHA)],
                     fragment="several commits")

    def test_an_undeclared_fixture_or_mode_is_refused(self):
        self.refused([rec("A_spread_04", "pb3")], fragment="not a declared candidate")
        self.refused([rec(IDS[0], "pb2")], fragment="mode must be one of")

    def test_a_duplicate_is_refused_so_the_better_run_cannot_be_chosen(self):
        self.refused([rec(IDS[0], "pb3", unique=1), rec(IDS[0], "pb3", seed=410.0)], fragment="more than once")
        self.refused(tier1(IDS[0]), confirm=confirm_pair(IDS[0]) + [rec(IDS[0], "pb3", seed=410.0)],
                     fragment="more than once")

    def test_limits_checkpoint_alternatives_profile_and_timeouts_are_refused(self):
        self.refused([rec(IDS[0], "pb3", limits={**NO_LIMITS, "max_leg_calls": 30})], fragment="work limit")
        self.refused([rec(IDS[0], "pb3", limits={**NO_LIMITS, "wall_clock_checkpoint_limit_s": 30.0})],
                     fragment="work limit")
        self.refused([rec(IDS[0], "pb3", limits={**NO_LIMITS, "max_frontier_entries": 50})],
                     fragment="work limit")
        self.refused([rec(IDS[0], "pb3", limits={**NO_LIMITS, "max_frontier_entries": None})],
                     fragment="work limit")
        self.refused([rec(IDS[0], "pb3", limits=None)], fragment="work limit")
        self.refused([rec(IDS[0], "pb3", alternatives=True)], fragment="alternatives must be off")
        self.refused([rec(IDS[0], "pb3", timeout=600)], fragment="declared 300")
        self.refused([rec(IDS[0], "pb3"), rec(IDS[0], "b3", timeout=60)], fragment="declared 120")
        self.refused([rec(IDS[0], "pb3"), rec(IDS[0], "b3", limits=NO_LIMITS)], fragment="carries limits")
        profiled = rec(IDS[0], "pb3")
        profiled["profile"] = "hotspots"
        self.refused([profiled], fragment="profile")
        refused = rec(IDS[0], "pb3", "complete")
        refused["status"] = "refused"
        self.refused([refused], fragment="not an executed outcome")

    def test_a_later_candidate_before_an_earlier_one_finished_is_refused(self):
        self.refused([rec(IDS[0], "pb3", seed=410.0), *s1_fail(IDS[1], unique=1)], fragment="is unfinished")
        self.refused([rec(IDS[2], "pb3")], fragment="is unfinished")

    def test_a_stage_without_its_predecessor_is_refused(self):
        self.refused([rec(IDS[0], "b3")], fragment="without its pb3 run")
        self.refused([rec(IDS[0], "pb3"), rec(IDS[0], "b2")], fragment="without its b3 run")

    def test_a_confirmation_that_is_not_due_is_refused(self):
        self.refused(tier1(IDS[0]), confirm=confirm_pair(IDS[1]), fragment="was not due")
        self.refused(s1_fail(IDS[0], unique=1), confirm=[rec(IDS[0], "pb3")], fragment="was not due")
        self.refused(tier1(IDS[0]), confirm=[rec(IDS[0], "b3")], fragment="was not due")
        self.refused(tier1(IDS[0]), confirm=[rec(IDS[0], "pb3", seed=410.0, wall=151.0), rec(IDS[0], "b3")],
                     fragment="was not due")
        self.refused(tier1(IDS[0]), confirm=[rec(IDS[0], "b2")], fragment="not a confirmation mode")

    def test_an_executed_record_needs_its_validation(self):
        broken = rec(IDS[0], "pb3")
        broken["validation"] = None
        self.refused([broken], fragment="normalized validation")


class LedgerShape(unittest.TestCase):
    def test_the_ledger_is_deterministic_serialisable_and_complete(self):
        records = s1_fail(IDS[0], unique=1) + tier1(IDS[1])
        first, second = od.evaluate(records), od.evaluate(records)
        self.assertEqual(od.dumps_ledger(first), od.dumps_ledger(second))
        self.assertEqual(json.loads(od.dumps_ledger(first)), first)
        self.assertEqual([c["fixture_id"] for c in first["candidates"]], list(IDS))
        self.assertEqual(first["ledger_schema_version"], od.LEDGER_SCHEMA_VERSION)
        self.assertEqual(first["rules"], od.RULES)
        self.assertEqual(first["records_total"], 4)
        for candidate in first["candidates"]:
            self.assertEqual(set(candidate["stages"]), {"S1", "S2", "S3"})
            self.assertEqual(len(candidate["fixture_sha256"]), 64)
            self.assertEqual(candidate["seed"], bfx.ordering_seed(IDS.index(candidate["fixture_id"]) + 1))

    def test_existing_four_obstacle_fixtures_are_listed_as_not_run_and_ineligible(self):
        ledger = od.evaluate(s1_fail(IDS[0], unique=1))
        listed = ledger["existing_four_obstacle_fixtures"]
        self.assertEqual([e["fixture_id"] for e in listed], list(od.EXISTING_FOUR_OBSTACLE_FIXTURES))
        for entry in listed:
            self.assertFalse(entry["eligible_under_heading_rule"], entry["fixture_id"])
            self.assertFalse(entry["run_in_discovery"])

    def test_metrics_keep_the_certification_fields_and_costs_as_repr(self):
        ledger = od.evaluate(tier1(IDS[0]))
        metrics = by_id(ledger)[IDS[0]]["stages"]["S1"]["metrics"]
        for key in ("stop_reason", "optimality_proven", "unresolved_branch_count", "bound_violation_count",
                    "unique_complete_orders", "leg_outcome_counts", "b2_seed_total_cost", "turning_legs",
                    "wall_time_s", "planning_time_s", "result_hash"):
            self.assertIn(key, metrics)
        self.assertEqual((metrics["total_cost"], metrics["b2_seed_total_cost"]), ("400.0", "410.0"))
        old = by_id(ledger)[IDS[0]]["stages"]["S2"]["metrics"]
        self.assertEqual((old["orders_considered"], old["valid_orders"], old["pruned_orders"]), (24, 6, 6))

    def test_the_ledger_has_no_personal_paths(self):
        text = od.dumps_ledger(od.evaluate(tier1(IDS[0])))
        for needle in ("ksuji", "AppData", "C:\\\\", "C:/", "Users"):
            self.assertNotIn(needle, text)


class Cli(unittest.TestCase):
    SCRIPT = ALGO / "benchmarks" / "ordering_discovery.py"

    def run_cli(self, *args):
        return subprocess.run([sys.executable, str(self.SCRIPT), *args], capture_output=True, text=True)

    @staticmethod
    def write(path, records):
        path.write_text("".join(json.dumps(r) + "\n" for r in records), encoding="utf-8")

    def test_plan_prints_the_declared_candidates(self):
        done = self.run_cli("plan")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertEqual([c["fixture_id"] for c in json.loads(done.stdout)["candidates"]], list(IDS))

    def test_next_and_evaluate_read_jsonl_and_hash_the_inputs(self):
        with tempfile.TemporaryDirectory() as folder:
            primary = Path(folder) / "primary.jsonl"
            self.write(primary, tier1(IDS[0]))
            done = self.run_cli("next", "--primary", str(primary))
            self.assertEqual(done.returncode, 0, done.stderr)
            self.assertEqual(json.loads(done.stdout)["mode"], "pb3")
            out = Path(folder) / "ledger.json"
            done = self.run_cli("evaluate", "--primary", str(primary), "--out", str(out))
            self.assertEqual(done.returncode, 0, done.stderr)
            ledger = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(ledger["inputs"], [{"name": "primary.jsonl", "sha256": rep.sha256_file(primary),
                                                 "records": 3}])

    def test_voided_runs_are_disclosed_with_their_hash_and_reason_and_never_evaluated(self):
        with tempfile.TemporaryDirectory() as folder:
            primary, voided = Path(folder) / "primary.jsonl", Path(folder) / "voided.jsonl"
            self.write(primary, tier1(IDS[0]))
            self.write(voided, [rec(IDS[0], "pb3", replay=["chain: x"], sha=OTHER_SHA)])
            out = Path(folder) / "ledger.json"
            done = self.run_cli("evaluate", "--primary", str(primary), "--voided", str(voided),
                                "--voided-reason", "evaluator fault", "--out", str(out))
            self.assertEqual(done.returncode, 0, done.stderr)
            ledger = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(ledger["voided_runs"], [{"name": "voided.jsonl", "sha256": rep.sha256_file(voided),
                                                      "records": 1, "reason": "evaluator fault"}])
            self.assertEqual(ledger["status"], "incomplete")
            self.assertEqual(od.evaluate(tier1(IDS[0]))["voided_runs"], [])
            done = self.run_cli("evaluate", "--primary", str(primary), "--voided", str(voided))
            self.assertNotEqual(done.returncode, 0)

    def test_a_refusal_exits_six_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as folder:
            primary, out = Path(folder) / "primary.jsonl", Path(folder) / "ledger.json"
            self.write(primary, [rec(IDS[0], "pb3", dirty=True)])
            done = self.run_cli("evaluate", "--primary", str(primary), "--out", str(out))
            self.assertEqual(done.returncode, 6)
            self.assertIn("refused:", done.stderr)
            self.assertFalse(out.exists())

    def test_a_stopped_search_exits_four(self):
        with tempfile.TemporaryDirectory() as folder:
            primary = Path(folder) / "primary.jsonl"
            self.write(primary, [rec(IDS[0], "pb3", bviol=1)])
            self.assertEqual(self.run_cli("evaluate", "--primary", str(primary)).returncode, 4)


if __name__ == "__main__":
    unittest.main()
