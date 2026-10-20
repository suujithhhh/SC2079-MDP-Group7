"""Evidence summary, coverage and gate logic (`algo/benchmarks/report.py`).

Most records here are SYNTHETIC test data built in this file. The one exception
is `TrackedSummary`, which pins the committed, measured summary
(`benchmarks/results/prefix_validation_summary.json`, generated from four accepted
raw files that stay outside the repository): its provenance, shape and gate
verdicts, and that the README quotes it consistently.

Run from algo/:
    python -m unittest tests.test_prefix_validation_summary -v
"""

import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

from benchmarks import campaigns as cmp  # noqa: E402
from benchmarks import report as rep  # noqa: E402

SCRIPT = ALGO / "benchmarks" / "bench_executable_planner.py"
GIB = 1024 ** 3
SHA = "0123456789abcdef0123456789abcdef01234567"          # the one clean commit of synthetic evidence
OTHER_SHA = "fedcba9876543210fedcba9876543210fedcba98"
WORK = {"leg_requests": 4, "cache_hits": 2, "leg_calls": 2, "lattice_expansions": 100, "nodes_expanded": 3,
        "children_generated": 6, "bound_evaluations": 9, "bound_prunes": 0, "dominance_prunes": 0,
        "frontier_peak": 0, "unique_prefixes_reached": 2, "unique_complete_orders": 1,
        "complete_routes_found": 1, "complete_leaf_discoveries": 1, "incumbent_updates": 0,
        "leg_outcome_counts": [["success", 2]]}


def case_of(invocation):
    return cmp.all_cases()[invocation.split("#")[0]]


def rec(invocation, status="complete", **kw):
    """A synthetic campaign record. Keyword overrides: cost, hash, stop, hit, proven, impossible,
    inv, replay, rss, planning, death, kind, work, unresolved, bound_violations, sha, dirty.
    A pb3 record defaults to a closed (`exhausted`), proven, limit-free search."""
    case = case_of(invocation)
    session = next(s for s in cmp.all_sessions() if case in s.cases)
    finished = status in ("complete", "partial", "no_route")
    kind = kw.get("kind", {"complete": "complete", "partial": "partial", "no_route": "none"}.get(status))
    raw = {"stop_reason": kw.get("stop", "exhausted" if case.mode == "pb3" else "first_complete_route"),
           "work_limit_hit": kw.get("hit"),
           "optimality_proven": kw.get("proven", case.mode == "pb3"),
           "complete_route_proven_impossible": kw.get("impossible", False),
           "unresolved_branch_count": kw.get("unresolved", 0),
           "bound_violation_count": kw.get("bound_violations", 0), "work": kw.get("work", dict(WORK))}
    validation = {
        "normalized": {"route_kind": kind, "total_cost": kw.get("cost", 10.0),
                       "total_distance_cm": kw["distance"] if "distance" in kw else 20.0, "legs": []},
        "raw": raw,
        "replay_violations": list(kw.get("replay", [])), "b2_replay_violations": [],
        "invariant_violations": list(kw.get("inv", [])),
        "timing": {"normalize_s": 0.001, "replay_s": 0.002}, "b3_not_worse_than_b2": True,
    } if finished else None
    return {
        "status": status, "case": {"campaign": session.campaign, "session": session.name,
                                   "case_id": case.case_id, "invocation": invocation},
        "result_hash": kw.get("hash", f"{case.fixture}-{case.mode[-2:]}-{case.alternatives}") if finished else None,
        "planning_time_s": kw.get("planning", 1.0) if finished else None, "wall_time_s": 2.0,
        "cleanup": {"death_verified": kw.get("death", True), "tempdir_removed": kw.get("tempdir", True),
                    "timeout_kind": "harness_hard_timeout" if status == "timeout" else None},
        "peak_memory": {"peak_rss_bytes": kw.get("rss", 100 * 1024 ** 2)},
        "machine": {"system": "Test"}, "python": {"version": "3"},
        "git": {"sha": kw.get("sha", SHA), "dirty": kw.get("dirty", False)},
        "validation": validation,
    }


def standard_records(**overrides):
    """A clean synthetic record for every standard invocation. R21 (old search) times out, the
    way it likely does for real; R22/R23 are proven impossible; R26-R29 are the budget,
    cancellation, frontier and checkpoint stops."""
    special = {
        "R21": dict(status="timeout"),
        "R22": dict(status="no_route", impossible=True), "R23": dict(status="partial", impossible=True,
                                                                     stop="work_limit", hit="max_leg_calls"),
        "R26": dict(status="complete", stop="work_limit", hit="max_leg_calls", proven=False),
        "R27": dict(status="partial", stop="cancelled", proven=False),
        "R28": dict(status="complete", stop="work_limit", hit="max_frontier_entries", proven=False),
        "R29": dict(status="partial", stop="wall_clock_checkpoint_limit", proven=False),
    }
    out = {}
    for session in cmp.SESSIONS["standard"]:
        for case, invocation in cmp.session_invocations(session):
            spec = dict(special.get(invocation, {}))
            status = spec.pop("status", "complete")
            out[invocation] = rec(invocation, status, **spec)
    for key, value in overrides.items():
        out[key] = value
    return out


def summary_of(records):
    return rep.build_summary(list(records.values()) if isinstance(records, dict) else list(records))


class EmptySummary(unittest.TestCase):
    def test_structure(self):
        s = rep.build_summary([], [])
        self.assertEqual(s["summary_schema_version"], rep.SUMMARY_SCHEMA_VERSION)
        self.assertEqual(s["manifest_version"], cmp.MANIFEST_VERSION)
        self.assertEqual((s["records_total"], s["unassigned_records"], s["inputs"], s["environment"]),
                         (0, 0, [], None))
        self.assertEqual(len(s["cases"]), 102)
        self.assertEqual({e["item_status"] for e in s["cases"].values()}, {"missing"})
        self.assertEqual(set(s["gates"]), {f"G{i}" for i in range(1, 9)})
        self.assertEqual({g["status"] for g in s["gates"].values()}, {"not_measured"})
        self.assertEqual(set(s["coverage"]["sessions"].values()), {"not_started"})
        self.assertEqual(s["coverage"]["minimum_evidence_set"], "not_started")
        self.assertEqual(set(s["coverage"]["requirements"].values()), {"not_started"})
        self.assertEqual(s["calibration"], {"status": "insufficient_data", "records": 0})
        self.assertEqual(s["overshoot"], [])
        self.assertTrue(all(c["state"] == "missing" for c in s["comparisons"]))

    def test_no_measured_values_are_present(self):
        for entry in rep.build_summary([], [])["cases"].values():
            self.assertNotIn("planning_time_s", entry)
            self.assertNotIn("result_hash", entry)
            self.assertNotIn("peak_rss_bytes", entry)

    def test_summary_is_deterministic(self):
        self.assertEqual(rep.dumps_summary(rep.build_summary([], [])), rep.dumps_summary(rep.build_summary([], [])))
        records = standard_records()
        self.assertEqual(rep.dumps_summary(summary_of(records)), rep.dumps_summary(summary_of(records)))


ACCEPTED_INPUTS = [   # the four accepted raw files, in the order they were given to `report`
    ("quick-fixed.jsonl", 11, "12bc36008c35a9bf28dfca8cb23edd54f21d7a85ff172344b8177a0dcca9a24c"),
    ("standard-s1.jsonl", 21, "6a1c5d889cbfd8b64ccbfb41152dc0e15bf6775a59802d98d8d001e9348264e4"),
    ("standard-s2.jsonl", 10, "b4b39af1f0251a92229fa6f48c8f4ae85f7d6370d688af7d82f038acf0f571ed"),
    ("standard-s3.jsonl", 3, "a3bce21b690e611713d12d25c8ecac0a057762302c5052526727b266d674e2f3"),
]
FAILED_QUICK_SHA = "c26ecaa8074b2aff6c6086495dd8a555f927ba36538561115aa4346eb65d9742"
EXPECTED_GATES = {"G1": "pass", "G2": "pass", "G3": "not_measured", "G4": "not_measured",
                  "G5": "not_measured", "G6": "not_measured", "G7": "not_measured", "G8": "pass"}
TRACKED_TEXT = rep.SUMMARY_PATH.read_text(encoding="utf-8")
TRACKED = json.loads(TRACKED_TEXT)
README_TEXT = (ALGO / "benchmarks" / "README.md").read_text(encoding="utf-8")


class TrackedSummary(unittest.TestCase):
    """The committed, measured evidence summary. Its raw inputs live outside the repository, so these
    tests pin its provenance, shape and verdicts and check that nothing else was invented."""

    def measured(self):
        return {k: v for k, v in TRACKED["cases"].items() if v["item_status"] != "missing"}

    def test_versions_and_provenance(self):
        # The tracked summary is historical: it predates the bounded campaign (manifest 2) and
        # the bounded summary fields (summary schema 2), and is replaced only by an evidence PR.
        self.assertEqual((TRACKED["summary_schema_version"], TRACKED["manifest_version"]), (1, 1))
        self.assertEqual((rep.SUMMARY_SCHEMA_VERSION, cmp.MANIFEST_VERSION), (2, 2))
        self.assertEqual([(i["name"], i["records"], i["sha256"]) for i in TRACKED["inputs"]], ACCEPTED_INPUTS)
        self.assertEqual(TRACKED["records_total"], 45)
        self.assertEqual(sum(i["records"] for i in TRACKED["inputs"]), 45)
        self.assertEqual(TRACKED["unassigned_records"], 0)

    def test_the_failed_first_quick_run_is_not_an_input(self):
        self.assertNotIn("quick.jsonl", [i["name"] for i in TRACKED["inputs"]])
        self.assertNotIn(FAILED_QUICK_SHA, [i["sha256"] for i in TRACKED["inputs"]])
        self.assertNotIn(FAILED_QUICK_SHA, TRACKED_TEXT)
        self.assertEqual(TRACKED["environment"]["git_shas"],
                         ["0c4e328669eb4c6e57a58efdddf7601eecae02d8", "3324d97aaa035b00ad7897b09dc2e06e0cead1c9"])
        self.assertFalse(TRACKED["environment"]["git_dirty_any"])

    def test_coverage_is_complete_for_the_standard_set_only(self):
        c = TRACKED["coverage"]
        self.assertEqual(c["minimum_evidence_set"], "complete")
        self.assertEqual(c["sessions"], {"quick/Q": "complete", "standard/S1": "complete",
                                         "standard/S2": "complete", "standard/S3": "complete",
                                         "extended/E1": "not_started", "extended/E2": "not_started",
                                         "extended/E3": "not_started"})
        self.assertEqual(c["campaigns"], {"quick": "complete", "standard": "complete", "extended": "not_started"})
        self.assertEqual(set(c["requirements"].values()), {"complete"})

    def test_exact_gate_states(self):
        self.assertEqual({g: v["status"] for g, v in TRACKED["gates"].items()}, EXPECTED_GATES)
        self.assertIn("X1", TRACKED["gates"]["G3"]["evidence"])
        self.assertIn("n5=missing", TRACKED["gates"]["G4"]["reason"])
        self.assertIn("n7=missing", TRACKED["gates"]["G4"]["reason"])
        for gate in ("G5", "G6"):
            self.assertIn("backstop", TRACKED["gates"][gate]["reason"])
        self.assertEqual(TRACKED["gates"]["G7"]["evidence"], [])

    def test_gate_evidence_ids(self):
        gates = TRACKED["gates"]
        self.assertEqual(gates["G1"]["evidence"], sorted(self.measured()))
        self.assertEqual(gates["G2"]["evidence"], sorted(["R2", "R3", "R5", "R6", "R8", "R9", "R12", "R13", "R20", "R21",
                                                          "R24", "R25", "R33", "R34"]))
        self.assertEqual(gates["G3"]["evidence"], sorted(["R8", "R10", "R14", "X1", "R16", "R17"]))
        self.assertEqual(gates["G4"]["evidence"], sorted(["R11", "X12", "R14", "X18", "R16", "R22", "R23"]))
        self.assertEqual(gates["G8"]["evidence"], sorted(["Q9", "Q10", "R15", "R18", "R23", "R26", "R27", "R28", "R29"]))
        self.assertEqual({c["old"]: c["state"] for c in TRACKED["comparisons"]},
                         {"R3": "ok", "R6": "ok", "R9": "ok", "R21": "ok", "R25": "ok", "R34": "ok", "R13": "excluded"})

    def test_only_the_extended_cases_are_missing(self):
        missing = {k for k, v in TRACKED["cases"].items() if v["item_status"] == "missing"}
        self.assertEqual(missing, {f"X{i}" for i in range(1, 19)})
        self.assertEqual(len(self.measured()), 45)
        self.assertEqual({v["item_status"] for v in self.measured().values()}, {"satisfied", "timeout"})

    def test_no_invented_measurements(self):
        for case_id, entry in TRACKED["cases"].items():
            if entry["item_status"] == "missing":
                for key in ("planning_time_s", "result_hash", "peak_rss_bytes", "work", "outcome"):
                    self.assertNotIn(key, entry, case_id)
        timeouts = [k for k, v in self.measured().items() if v["outcome"] == "T"]
        self.assertEqual(timeouts, ["R13"])
        r13 = TRACKED["cases"]["R13"]
        self.assertEqual(r13["timeout_kind"], "harness_hard_timeout")
        for key in ("result_hash", "planning_time_s", "optimality_proven", "stop_reason", "route_kind", "violations", "work"):
            self.assertIsNone(r13[key], key)
        self.assertTrue(r13["cleanup_death_verified"] and r13["cleanup_tempdir_removed"])
        for case_id, entry in self.measured().items():
            self.assertNotEqual(entry["outcome"], "E", case_id)
            self.assertTrue(entry["cleanup_death_verified"] and entry["cleanup_tempdir_removed"], case_id)
            if entry["outcome"] != "T":
                self.assertEqual(entry["violations"], {"replay": 0, "b2_replay": 0, "invariant": 0}, case_id)
                self.assertFalse(entry["bound_violation_count"], case_id)
                self.assertIsNotNone(entry["result_hash"], case_id)
        # A proof is only ever claimed by a prefix B3 run.
        for case_id, entry in self.measured().items():
            if entry.get("optimality_proven"):
                self.assertEqual(entry["mode"], "pb3", case_id)

    def test_baseline_values_quoted_in_the_readme(self):
        cases = TRACKED["cases"]
        self.assertEqual([round(cases[c]["planning_time_s"], 1) for c in ("R11", "R14", "R16", "R17", "R12", "R18")],
                         [172.7, 38.1, 55.5, 48.0, 536.6, 140.8])
        self.assertEqual(cases["R16"]["result_hash"], cases["R17"]["result_hash"])
        self.assertEqual(cases["R16"]["work"], cases["R17"]["work"])
        self.assertEqual(cases["R12"]["result_hash"], cases["R11"]["result_hash"])          # no B3 improvement
        self.assertFalse(cases["R12"]["optimality_proven"])
        overshoot = TRACKED["overshoot"]
        self.assertEqual([o["invocation"] for o in overshoot], ["R29"])
        self.assertEqual((round(overshoot[0]["overshoot_s"], 1), round(overshoot[0]["last_planned_leg_s"], 1)), (33.3, 34.7))
        for text in ("172.7 s", "38.1 s", "55.5 s / 48.0 s", "536.6 s", "33.3 s", "34.7 s", "about 98%", "11.1 million"):
            self.assertIn(text, README_TEXT)

    def test_no_personal_paths(self):
        for needle in ("ksuji", "AppData", "C:\\\\", "C:/", "Users"):
            self.assertNotIn(needle, TRACKED_TEXT)

    def test_format_is_valid_and_deterministic(self):
        self.assertEqual(TRACKED_TEXT, rep.dumps_summary(TRACKED))
        self.assertTrue(TRACKED_TEXT.endswith("}\n"))

    def test_the_readme_gate_table_matches_the_tracked_summary(self):
        rows = {}
        for line in README_TEXT.splitlines():
            for gate in TRACKED["gates"]:
                if line.startswith(f"| {gate} "):
                    rows[gate] = line.rstrip().rsplit("| ", 1)[1].strip(" |`")
        self.assertEqual(rows, {g: v["status"] for g, v in TRACKED["gates"].items()})
        for _, _, sha in ACCEPTED_INPUTS:
            self.assertIn(sha, README_TEXT)
        for name, records, _ in ACCEPTED_INPUTS:
            self.assertIn(f"`{name}`", README_TEXT)
        self.assertIn("45 accepted records", README_TEXT)
        for claim in ("G1 pass, G8 pass", "G3 not measured", "G4 not measured", "G7 not measured"):
            self.assertIn(claim, README_TEXT)
        self.assertNotIn("G3 pass", README_TEXT)


class Inputs(unittest.TestCase):
    def test_each_raw_input_is_hashed(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "raw.jsonl"
            path.write_text(json.dumps(rec("R1")) + "\n\n" + json.dumps(rec("R2")) + "\n", encoding="utf-8")
            records, inputs = rep.load_records([path])
            self.assertEqual(len(records), 2)
            self.assertEqual(inputs, [{"name": "raw.jsonl", "records": 2,
                                       "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}])
            s = rep.build_summary(records, inputs)
            self.assertEqual(s["inputs"], inputs)
            self.assertEqual(s["cases"]["R1"]["item_status"], "satisfied")
            self.assertEqual(s["cases"]["R3"]["item_status"], "missing")

    def test_records_without_a_case_are_counted_not_assigned(self):
        s = rep.build_summary([{"status": "complete", "git": {"sha": SHA, "dirty": False}}], [])
        self.assertEqual((s["records_total"], s["unassigned_records"]), (1, 1))


class Coverage(unittest.TestCase):
    def test_full_standard_set_is_complete_and_other_campaigns_untouched(self):
        s = summary_of(standard_records())
        self.assertEqual(s["coverage"]["minimum_evidence_set"], "complete")
        self.assertEqual({v for k, v in s["coverage"]["sessions"].items() if k.startswith("standard")}, {"complete"})
        self.assertEqual(s["coverage"]["sessions"]["quick/Q"], "not_started")
        self.assertEqual(s["coverage"]["campaigns"]["extended"], "not_started")
        self.assertEqual(set(s["coverage"]["requirements"].values()), {"complete"})

    def test_a_missing_or_unsatisfied_item_makes_it_partial(self):
        records = standard_records()
        del records["R7"]
        s = summary_of(records)
        self.assertEqual(s["coverage"]["minimum_evidence_set"], "partial")
        self.assertEqual(s["coverage"]["requirements"]["n3"], "partial")
        self.assertEqual(s["cases"]["R7"]["item_status"], "missing")
        records = standard_records(R1=rec("R1", "timeout"))          # R1 accepts C only
        s = summary_of(records)
        self.assertEqual(s["cases"]["R1"]["item_status"], "unsatisfied")
        self.assertEqual(s["coverage"]["minimum_evidence_set"], "partial")

    def test_accepted_timeouts_count_as_timeout_evidence(self):
        s = summary_of(standard_records())
        self.assertEqual(s["cases"]["R21"]["item_status"], "timeout")
        self.assertEqual(s["cases"]["R21"]["outcome"], "T")
        self.assertIsNone(s["cases"]["R21"]["result_hash"])          # no planner data on a harness timeout


class Gates(unittest.TestCase):
    def status(self, records, gate):
        return summary_of(records)["gates"][gate]["status"]

    def test_g1(self):
        self.assertEqual(self.status({"R1": rec("R1")}, "G1"), "not_measured")         # clean but incomplete
        self.assertEqual(self.status(standard_records(), "G1"), "pass")
        for bad in (rec("R1", inv=["x"]), rec("R1", replay=["chain.command: x"]), rec("R1", "error"),
                    rec("R1", death=False)):
            with self.subTest(bad=bad["status"]):
                s = summary_of(standard_records(R1=bad))
                self.assertEqual(s["gates"]["G1"]["status"], "fail")
                self.assertIn("R1", s["gates"]["G1"]["evidence"])
        violated = rec("R1")
        violated["validation"]["raw"]["bound_violation_count"] = 1
        self.assertEqual(self.status(standard_records(R1=violated), "G1"), "fail")

    def test_g2_exactness_excludes_old_timeouts_and_fails_on_disagreement(self):
        self.assertEqual(self.status(standard_records(), "G2"), "pass")              # R21 old timeout: excluded
        self.assertEqual(self.status(standard_records(R2=rec("R2", cost=10.5)), "G2"), "fail")
        self.assertEqual(self.status(standard_records(R2=rec("R2", proven=False)), "G2"), "not_measured")
        self.assertEqual(self.status(standard_records(R2=rec("R2", "partial", kind="partial")), "G2"), "fail")
        self.assertEqual(self.status(standard_records(R2=rec("R2", "timeout")), "G2"), "not_measured")
        self.assertEqual(self.status(standard_records(R34=rec("R34", "no_route", kind="none"),
                                                      R33=rec("R33", "complete")), "G2"), "fail")
        all_old_timed_out = {k: rec(k, "timeout") for k in ("R3", "R6", "R9", "R13", "R21", "R25", "R34")}
        self.assertEqual(self.status(standard_records(**all_old_timed_out), "G2"), "not_measured")

    def test_four_obstacle_pair_is_part_of_the_exactness_inventory(self):
        self.assertIn(("R13", "R12", "b3"), rep.EXACT_PAIRS)
        pairs = {c["old"]: (c["prefix"], c["kind"]) for c in summary_of(standard_records())["comparisons"]}
        self.assertEqual(pairs["R13"], ("R12", "b3"))

    def g2_r13(self, old, new, **more):
        s = summary_of(standard_records(R13=old, R12=new, **more))
        return s["gates"]["G2"]["status"], {c["old"]: c["state"] for c in s["comparisons"]}["R13"]

    def test_g2_stays_not_measured_while_the_four_obstacle_pair_is_missing(self):
        for missing in (("R12",), ("R13",), ("R12", "R13")):
            records = standard_records()
            for case in missing:
                del records[case]
            s = summary_of(records)
            self.assertEqual(s["gates"]["G2"]["status"], "not_measured", missing)
            self.assertEqual({c["old"]: c["state"] for c in s["comparisons"]}["R13"], "missing")
        # Every S1 pair recorded and matching, but no S2 evidence yet: still not_measured.
        s1_only = {k: v for k, v in standard_records().items() if cmp.all_cases()[k.split("#")[0]].case_id
                   not in ("R12", "R13")}
        self.assertEqual(self.status(s1_only, "G2"), "not_measured")

    def test_an_r13_timeout_is_excluded_not_agreement(self):
        state = self.g2_r13(rec("R13", "timeout"), rec("R12", "complete", proven=False, stop="work_limit",
                                                       hit="max_leg_calls"))
        self.assertEqual(state, ("pass", "excluded"))              # the other pairs still count
        only_excluded = {k: rec(k, "timeout") for k in ("R3", "R6", "R9", "R13", "R21", "R25", "R34")}
        self.assertEqual(self.status(standard_records(**only_excluded), "G2"), "not_measured")

    def test_completed_proven_equal_cost_r12_r13_passes(self):
        self.assertEqual(self.g2_r13(rec("R13", cost=12.5), rec("R12", cost=12.5)), ("pass", "ok"))

    def test_r12_r13_cost_disagreement_fails(self):
        for old, new in ((12.5, 12.500000000000002), (12.500000000000002, 12.5), (12.5, 11.0)):
            with self.subTest(old=old, new=new):
                self.assertEqual(self.g2_r13(rec("R13", cost=old), rec("R12", cost=new)), ("fail", "fail"))

    def test_unproven_or_incomplete_r12_cannot_count_as_agreement(self):
        unproven = rec("R12", cost=10.0, proven=False, stop="work_limit", hit="max_leg_calls")
        self.assertEqual(self.g2_r13(rec("R13"), unproven), ("not_measured", "not_certified"))
        self.assertEqual(self.g2_r13(rec("R13"), rec("R12", proven=None)), ("not_measured", "not_certified"))
        self.assertEqual(self.g2_r13(rec("R13"), rec("R12", "partial", kind="partial")), ("fail", "fail"))
        self.assertEqual(self.g2_r13(rec("R13"), rec("R12", "timeout")), ("not_measured", "unmeasured"))

    def test_g3_determinism(self):
        extra = {"X1": rec("X1")}
        self.assertEqual(self.status(standard_records(**extra), "G3"), "pass")
        self.assertEqual(self.status(standard_records(), "G3"), "not_measured")       # X1 (n=6 pair) missing
        self.assertEqual(self.status(standard_records(X1=rec("X1", hash="other")), "G3"), "fail")
        different = dict(WORK, leg_calls=99)
        self.assertEqual(self.status(standard_records(X1=rec("X1"), R10=rec("R10", work=different)), "G3"), "fail")
        self.assertEqual(self.status(standard_records(X1=rec("X1"), R17=rec("R17", "timeout")), "G3"), "not_measured")

    def test_g4_coverage_needs_a_representative_at_every_count_from_four_to_eight(self):
        full = standard_records(X12=rec("X12"), X18=rec("X18"))
        s = summary_of(full)
        self.assertEqual(s["gates"]["G4"]["status"], "pass")
        self.assertEqual(s["gates"]["G4"]["evidence"], sorted(["R11", "X12", "R14", "X18", "R16", "R22", "R23"]))
        self.assertIn("from 4 through 8", s["gates"]["G4"]["reason"])
        self.assertNotIn("every", s["gates"]["G4"]["reason"])
        # Standard evidence alone (no five- or seven-obstacle result) can never pass.
        alone = summary_of(standard_records())["gates"]["G4"]
        self.assertEqual(alone["status"], "not_measured")
        self.assertIn("n5=missing", alone["reason"])
        self.assertIn("n7=missing", alone["reason"])

    def test_g4_cannot_pass_without_the_five_or_the_seven_obstacle_result(self):
        self.assertEqual(self.status(standard_records(X18=rec("X18")), "G4"), "not_measured")     # n=5 missing
        self.assertEqual(self.status(standard_records(X12=rec("X12")), "G4"), "not_measured")     # n=7 missing
        for missing in ("R11", "R14", "R16"):
            records = standard_records(X12=rec("X12"), X18=rec("X18"))
            del records[missing]
            self.assertEqual(self.status(records, "G4"), "not_measured", missing)

    def test_g4_counter_evidence_and_timeouts(self):
        base = dict(X12=rec("X12"), X18=rec("X18"))
        self.assertEqual(self.status(standard_records(**base, R11=rec("R11", "partial", kind="partial")), "G4"), "fail")
        self.assertEqual(self.status(standard_records(**dict(base, X12=rec("X12", "partial", kind="partial"))), "G4"), "fail")
        self.assertEqual(self.status(standard_records(**dict(base, X18=rec("X18", "no_route", kind="none"))), "G4"), "fail")
        self.assertEqual(self.status(standard_records(**dict(base, X18=rec("X18", "timeout"))), "G4"), "not_measured")
        self.assertEqual(self.status(standard_records(**base, R11=rec("R11", "timeout")), "G4"), "not_measured")
        self.assertEqual(self.status(standard_records(**base, R22=rec("R22", "no_route", impossible=False)), "G4"), "fail")

    def test_g4_representatives_are_unlimited_alternatives_off_spread_cases(self):
        self.assertEqual(sorted(rep.G4_REPRESENTATIVE), [4, 5, 6, 7, 8])
        cases = cmp.all_cases()
        for count, case_id in rep.G4_REPRESENTATIVE.items():
            case = cases[case_id]
            with self.subTest(count=count):
                self.assertEqual(case.fixture, f"A_spread_{count:02d}")
                self.assertEqual((case.mode, case.alternatives, case.option_dict()), ("pb2", "off", {}))
                self.assertIn("C", case.accept)
                self.assertTrue(0 < case.timeout_s <= 600)

    def test_g1_cleanup_failures_block_and_ordinary_timeouts_do_not(self):
        # tempdir_removed == false fails G1 and is exposed in the entry.
        s = summary_of(standard_records(R1=rec("R1", tempdir=False)))
        self.assertEqual(s["gates"]["G1"]["status"], "fail")
        self.assertIn("R1", s["gates"]["G1"]["evidence"])
        self.assertIs(s["cases"]["R1"]["cleanup_tempdir_removed"], False)
        # tempdir_removed == true does not.
        s = summary_of(standard_records(R1=rec("R1", tempdir=True)))
        self.assertEqual(s["gates"]["G1"]["status"], "pass")
        self.assertIs(s["cases"]["R1"]["cleanup_tempdir_removed"], True)
        # job_closed == false alone is not a failure (fallback and POSIX paths use no Job Object).
        no_job = rec("R1")
        no_job["cleanup"]["job_closed"] = False
        self.assertEqual(self.status(standard_records(R1=no_job), "G1"), "pass")
        # An ordinary harness timeout (verified death, temp dir removed) follows its case's outcomes.
        accepted = summary_of(standard_records(R11=rec("R11", "timeout")))            # R11 accepts C and T
        self.assertEqual(accepted["cases"]["R11"]["item_status"], "timeout")
        self.assertEqual(accepted["gates"]["G1"]["status"], "pass")
        refused = summary_of(standard_records(R1=rec("R1", "timeout")))               # R1 accepts C only
        self.assertEqual(refused["cases"]["R1"]["item_status"], "unsatisfied")
        self.assertEqual(refused["gates"]["G1"]["status"], "not_measured")            # incomplete, not a violation
        # A timeout whose temporary directory was not removed is still a cleanup failure.
        self.assertEqual(self.status(standard_records(R11=rec("R11", "timeout", tempdir=False)), "G1"), "fail")
        self.assertEqual(self.status(standard_records(R11=rec("R11", "timeout", death=False)), "G1"), "fail")

    def test_partial_route_exactness_needs_distance_and_cost_to_agree(self):
        def partial(case, **kw):
            kw.setdefault("cost", 10.0)
            return rec(case, "partial", kind="partial", **kw)

        def g2(old, new):
            s = summary_of(standard_records(R34=old, R33=new))
            return s["gates"]["G2"]["status"], {c["old"]: c["state"] for c in s["comparisons"]}["R34"]

        self.assertEqual(g2(partial("R34"), partial("R33")), ("pass", "ok"))                      # matching
        self.assertEqual(g2(partial("R34"), partial("R33", distance=20.000000000000004)), ("fail", "fail"))
        self.assertEqual(g2(partial("R34", distance=19.5), partial("R33")), ("fail", "fail"))     # distance
        self.assertEqual(g2(partial("R34"), partial("R33", cost=10.000000000000002)), ("fail", "fail"))
        self.assertEqual(g2(partial("R34", cost=11.0), partial("R33")), ("fail", "fail"))         # cost
        # Distance equal but cost different, and the reverse: each is checked on its own.
        self.assertEqual(g2(partial("R34", cost=10.0, distance=20.0), partial("R33", cost=10.0, distance=21.0))[1], "fail")
        self.assertEqual(g2(partial("R34", cost=10.0, distance=20.0), partial("R33", cost=11.0, distance=20.0))[1], "fail")
        # Missing or unusable partial accounting can never pass.
        for bad in (None, float("nan"), float("inf"), True, "20.0"):
            with self.subTest(bad=bad):
                self.assertEqual(g2(partial("R34", distance=bad), partial("R33", distance=bad))[0], "fail")
                self.assertEqual(g2(partial("R34"), partial("R33", distance=bad))[0], "fail")
                self.assertEqual(g2(partial("R34", cost=bad), partial("R33", cost=bad))[0], "fail")
        # One side found a partial route and the other found none.
        self.assertEqual(g2(partial("R34"), rec("R33", "no_route", kind="none"))[0], "fail")
        self.assertEqual(g2(rec("R34", "no_route", kind="none"), rec("R33", "no_route", kind="none"))[1], "ok")

    def test_existing_g2_rules_still_hold(self):
        self.assertEqual(self.status(standard_records(), "G2"), "pass")
        self.assertEqual(self.status(standard_records(R3=rec("R3", "timeout")), "G2"), "pass")        # excluded
        self.assertEqual(self.status(standard_records(R2=rec("R2", "timeout")), "G2"), "not_measured")
        self.assertEqual(self.status(standard_records(R2=rec("R2", cost=10.5)), "G2"), "fail")
        entry = summary_of(standard_records(R2=rec("R2", distance=33.5)))["cases"]["R2"]
        self.assertEqual((entry["total_cost"], entry["total_distance_cm"]), (10.0, 33.5))

    def test_timing_and_overshoot_gates_never_pass_without_the_hard_backstop(self):
        s = summary_of(standard_records(X1=rec("X1")))
        for gate in ("G5", "G6"):
            self.assertEqual(s["gates"][gate]["status"], "not_measured")
            self.assertIn("hard backstop", s["gates"][gate]["reason"])

    def test_g7_memory(self):
        self.assertEqual(self.status(standard_records(), "G7"), "not_measured")       # X13 is extended
        ok = standard_records(X13=rec("X13", rss=GIB // 2))
        self.assertEqual(self.status(ok, "G7"), "pass")
        self.assertEqual(self.status(standard_records(X13=rec("X13", rss=2 * GIB)), "G7"), "fail")
        self.assertEqual(self.status(standard_records(X13=rec("X13", "timeout")), "G7"), "not_measured")

    def test_g8_metadata(self):
        self.assertEqual(self.status(standard_records(), "G8"), "pass")
        s = summary_of(standard_records(R27=rec("R27", "partial", stop="cancelled", inv=["bad wording"])))
        self.assertEqual(s["gates"]["G8"]["status"], "fail")
        records = standard_records()
        records["R29"] = rec("R29", "complete")            # no checkpoint stop observed any more
        s = summary_of(records)
        self.assertEqual(s["gates"]["G8"]["status"], "not_measured")
        self.assertIn("checkpoint", s["gates"]["G8"]["reason"])

    def test_nothing_is_marked_pass_without_evidence(self):
        for gate, info in summary_of([]).get("gates", {}).items():
            self.assertEqual(info["status"], "not_measured", gate)


class ExactnessEligibility(unittest.TestCase):
    """G2 only certifies comparisons whose prefix route is eligible for one; anything else is
    `not_certified` (G2 not_measured), never a false pass or a false failure."""

    OLD_COST = 400.2433292025247            # A_spread_04, as recorded for R11/R12

    @staticmethod
    def r12(cost=OLD_COST, **kw):
        """R12 as recorded: complete, closed search (`exhausted`) but not proven, 8 unresolved."""
        kw = {"proven": False, "stop": "exhausted", "unresolved": 8, **kw}
        return rec("R12", cost=cost, **kw)

    def pair(self, old_id, old, new_id, new, **more):
        s = summary_of(standard_records(**{old_id: old, new_id: new}, **more))
        return s["gates"]["G2"]["status"], {c["old"]: c["state"] for c in s["comparisons"]}[old_id], s

    def r13(self, old, new, **more):
        return self.pair("R13", old, "R12", new, **more)[:2]

    def test_every_pair_compares_the_search_modes_its_kind_names(self):
        cases = cmp.all_cases()
        kinds = {}
        for old_id, new_id, kind in rep.EXACT_PAIRS:
            kinds[(old_id, new_id)] = kind
            with self.subTest(pair=f"{old_id}/{new_id}"):
                self.assertEqual((cases[old_id].mode, cases[new_id].mode), (kind, "pb" + kind[1]))
                self.assertEqual(cases[old_id].alternatives, cases[new_id].alternatives)
        self.assertEqual([k for k, v in kinds.items() if v == "b2"], [("R34", "R33")])   # the only B2 pair
        self.assertEqual(len(kinds), 7)

    def test_stop_vocabulary_matches_the_search(self):
        import route_prefix_search as rps
        self.assertEqual(rep.BUDGET_STOPS, {rps.STOP_WORK_LIMIT, rps.STOP_CANCELLED, rps.STOP_WALL_CLOCK_CHECKPOINT})
        self.assertEqual(rep.CLOSED_STOPS, {rps.STOP_EXHAUSTED, rps.STOP_BOUND_CLOSED})

    def test_a_faster_r13_and_the_real_shaped_r12_are_not_a_false_failure(self):
        # The old search now finishes A_spread_04 while R12 is the recorded, unproven search.
        status, state, s = self.pair("R13", rec("R13", cost=self.OLD_COST), "R12", self.r12())
        self.assertEqual((status, state), ("not_measured", "not_certified"))
        self.assertIn("R13/R12", s["gates"]["G2"]["reason"])
        self.assertIn("never a pass", s["gates"]["G2"]["reason"])
        self.assertEqual(s["gates"]["G1"]["status"], "pass")              # an unproven route is no violation
        self.assertIs(s["cases"]["R12"]["optimality_proven"], False)
        # The same run with the old search timing out stays excluded, and the other pairs still pass.
        self.assertEqual(self.r13(rec("R13", "timeout"), self.r12()), ("pass", "excluded"))

    def test_equal_higher_and_lower_costs_against_an_uncertified_route(self):
        old = rec("R13", cost=self.OLD_COST)
        for label, cost in (("equal", self.OLD_COST), ("higher", self.OLD_COST + 10.0),
                            ("slightly higher", self.OLD_COST + 1e-9)):
            with self.subTest(label):
                self.assertEqual(self.r13(old, self.r12(cost=cost)), ("not_measured", "not_certified"))
        # A replay-valid complete route CHEAPER than the old optimum is counter-evidence.
        for label, cost in (("lower", self.OLD_COST - 1.0), ("slightly lower", self.OLD_COST - 1e-9)):
            with self.subTest(label):
                self.assertEqual(self.r13(old, self.r12(cost=cost)), ("fail", "fail"))
        # ...but a route that failed replay is not trustworthy counter-evidence; G1 carries that failure.
        status, state, s = self.pair("R13", old, "R12", self.r12(cost=self.OLD_COST - 1.0, replay=["chain.command: x"]))
        self.assertEqual((status, state), ("not_measured", "not_certified"))
        self.assertEqual(s["gates"]["G1"]["status"], "fail")
        # Unusable costs cannot be counter-evidence either.
        for bad in (None, float("nan"), True):
            with self.subTest(bad=bad):
                self.assertEqual(self.r13(old, self.r12(cost=bad))[1], "not_certified")
        # A certified route keeps the exact comparison: bit-equal passes, anything else fails.
        self.assertEqual(self.r13(old, rec("R12", cost=self.OLD_COST)), ("pass", "ok"))
        self.assertEqual(self.r13(old, rec("R12", cost=self.OLD_COST - 1.0)), ("fail", "fail"))
        self.assertEqual(self.r13(old, rec("R12", cost=self.OLD_COST + 1.0)), ("fail", "fail"))

    def test_a_proof_claim_beside_a_budget_stop_is_never_certified(self):
        old = rec("R13", cost=10.0)
        for stop, hit in (("work_limit", "max_leg_calls"), ("work_limit", "max_frontier_entries"),
                          ("cancelled", None), ("wall_clock_checkpoint_limit", None)):
            with self.subTest(stop=stop, hit=hit):
                claimed = rec("R12", cost=10.0, proven=True, stop=stop, hit=hit)
                self.assertEqual(self.r13(old, claimed), ("not_measured", "not_certified"))
        for label, kw in (("limit hit beside a closed stop", dict(hit="max_leg_calls")),
                          ("stop not closed", dict(stop="first_complete_route")),
                          ("unresolved branches", dict(unresolved=1)),
                          ("unresolved unknown", dict(unresolved=None)),
                          ("unresolved is a bool", dict(unresolved=False)),
                          ("bound violation", dict(bound_violations=1)),
                          ("bound violation unknown", dict(bound_violations=None)),
                          ("proof not claimed", dict(proven=False)),
                          ("proof unknown", dict(proven=None))):
            with self.subTest(label):
                self.assertEqual(self.r13(old, rec("R12", cost=10.0, **kw)), ("not_measured", "not_certified"))
        for stop in ("exhausted", "bound_closed"):                     # both closed stops certify
            with self.subTest(stop=stop):
                self.assertEqual(self.r13(old, rec("R12", cost=10.0, stop=stop)), ("pass", "ok"))

    def test_partial_and_missing_outcomes(self):
        for stop, hit in (("work_limit", "max_leg_calls"), ("cancelled", None), ("wall_clock_checkpoint_limit", None)):
            with self.subTest(stop=stop):
                partial = rec("R2", "partial", kind="partial", stop=stop, hit=hit)
                self.assertEqual(self.pair("R3", rec("R3"), "R2", partial)[:2], ("not_measured", "not_certified"))
                nothing = rec("R2", "no_route", kind="none", stop=stop, hit=hit)
                self.assertEqual(self.pair("R3", rec("R3"), "R2", nothing)[:2], ("not_measured", "not_certified"))
        # Without a budget stop, an incomplete answer to a completed old search is still a failure.
        self.assertEqual(self.pair("R3", rec("R3"), "R2", rec("R2", "partial", kind="partial"))[:2], ("fail", "fail"))
        self.assertEqual(self.pair("R3", rec("R3"), "R2", rec("R2", "no_route", kind="none"))[:2], ("fail", "fail"))
        # An old partial result against a complete prefix route is a mismatch whatever the stop.
        old_partial = rec("R34", "partial", kind="partial")
        self.assertEqual(self.pair("R34", old_partial, "R33", rec("R33"))[:2], ("fail", "fail"))
        self.assertEqual(self.pair("R34", old_partial, "R33", rec("R33", stop="work_limit", hit="max_leg_calls"))[:2],
                         ("fail", "fail"))
        # Two partial results: a budget-stopped prefix run is not certified even when the numbers match.
        matching = rec("R33", "partial", kind="partial", stop="work_limit", hit="max_leg_calls")
        self.assertEqual(self.pair("R34", old_partial, "R33", matching)[:2], ("not_measured", "not_certified"))
        self.assertEqual(self.pair("R34", old_partial, "R33", rec("R33", "partial", kind="partial"))[:2], ("pass", "ok"))
        # Missing and timed-out evidence keep their own states.
        records = standard_records()
        del records["R2"]
        self.assertEqual({c["old"]: c["state"] for c in summary_of(records)["comparisons"]}["R3"], "missing")
        self.assertEqual(self.pair("R3", rec("R3", "timeout"), "R2", rec("R2", proven=False, unresolved=8))[:2],
                         ("pass", "excluded"))
        self.assertEqual(self.pair("R3", rec("R3"), "R2", rec("R2", "timeout"))[:2], ("not_measured", "unmeasured"))

    def test_the_b2_pair_is_not_held_to_the_b3_proof_rules(self):
        # R34/R33 is old b2 versus pb2: completeness must agree; costs and proof are not asked.
        self.assertEqual(self.pair("R34", rec("R34", cost=10.0), "R33", rec("R33", cost=99.0))[:2], ("pass", "ok"))
        self.assertIs(summary_of(standard_records())["cases"]["R33"]["optimality_proven"], False)
        # ...but a budget-stopped prefix run is never certified, B2 or not.
        stopped = rec("R33", stop="work_limit", hit="max_leg_calls")
        self.assertEqual(self.pair("R34", rec("R34"), "R33", stopped)[:2], ("not_measured", "not_certified"))

    def test_not_certified_never_passes_and_names_every_pair(self):
        _, _, s = self.pair("R13", rec("R13"), "R12", self.r12(), R3=rec("R3"), R2=rec("R2", proven=False))
        reason = s["gates"]["G2"]["reason"]
        self.assertEqual(s["gates"]["G2"]["status"], "not_measured")
        for pair in ("R13/R12", "R3/R2"):
            self.assertIn(pair, reason)
        self.assertIn("R13/R12=not_certified", reason)
        self.assertIn("R6/R5=ok", reason)
        # A failure elsewhere still wins over a not-certified pair.
        _, _, failing = self.pair("R13", rec("R13"), "R12", self.r12(), R5=rec("R5", cost=1.0))
        self.assertEqual(failing["gates"]["G2"]["status"], "fail")
        # With every pair certified or excluded, G2 passes again.
        self.assertEqual(summary_of(standard_records())["gates"]["G2"]["status"], "pass")


class Provenance(unittest.TestCase):
    """Only records from ONE exact clean commit reach the gates."""

    def refused(self, records):
        with self.assertRaises(rep.ProvenanceError) as caught:
            summary_of(records)
        return caught.exception

    def test_one_clean_commit_is_accepted_and_reported(self):
        s = summary_of(standard_records())
        self.assertEqual(s["environment"]["git_shas"], [SHA])
        self.assertIs(s["environment"]["git_dirty_any"], False)
        self.assertEqual(rep.provenance_problems(list(standard_records().values())), [])
        self.assertEqual(rep.provenance_problems([]), [])

    def test_mixed_commits_are_refused(self):
        error = self.refused(standard_records(R1=rec("R1", sha=OTHER_SHA)))
        self.assertIn("several commits", str(error))
        self.assertIn(SHA[:12], str(error))
        self.assertIn(OTHER_SHA[:12], str(error))

    def test_anything_but_an_exactly_false_dirty_flag_is_refused(self):
        for dirty in (True, None, "false", "False", 0, "", [], {}):
            with self.subTest(dirty=dirty):
                error = self.refused(standard_records(R1=rec("R1", dirty=dirty)))
                self.assertIn("git.dirty", str(error))
                self.assertIn("R1", str(error))
        for git in ({"sha": SHA}, None, "clean", []):                    # no usable dirty flag at all
            record = rec("R1")
            record["git"] = git
            with self.subTest(git=git):
                self.refused(standard_records(R1=record))

    def test_a_missing_or_malformed_sha_is_refused(self):
        bad = (None, "", "abc", SHA[:39], SHA + "0", SHA.upper(), "g" * 40, " " + SHA[1:], SHA + "\n", 123, [SHA])
        for sha in bad:
            with self.subTest(sha=sha):
                error = self.refused(standard_records(R1=rec("R1", sha=sha)))
                self.assertIn("40 lowercase hex", str(error))
        for change in ({"git": {"dirty": False}}, {"git": None}, {"git": "abc"}):
            with self.subTest(change=change):
                base = rec("R1")
                base.update(change)
                self.refused(standard_records(R1=base))
        base = rec("R1")
        del base["git"]
        self.refused(standard_records(R1=base))

    def test_a_genuine_duplicate_invocation_is_refused(self):
        records = list(standard_records().values())
        error = self.refused(records + [rec("R1")])                       # even an identical copy
        self.assertIn("R1", str(error))
        self.assertIn("more than once", str(error))
        # A different outcome for the same invocation is not a way to choose a favourable run.
        self.refused(records + [rec("R1", "timeout")])

    def test_intentional_repeats_with_distinct_invocation_ids_are_accepted(self):
        s = summary_of([rec("Q5#1"), rec("Q5#2"), rec("Q4")])
        self.assertEqual((s["cases"]["Q5#1"]["item_status"], s["cases"]["Q5#2"]["item_status"]),
                         ("satisfied", "satisfied"))
        self.assertEqual(s["cases"]["Q5#1"]["case_id"], s["cases"]["Q5#2"]["case_id"])
        self.assertEqual(rep.provenance_problems([rec("Q5#1"), rec("Q5#2")]), [])
        self.assertTrue(rep.provenance_problems([rec("Q5#1"), rec("Q5#1")]))

    def test_records_without_a_case_need_provenance_too(self):
        self.refused([{"status": "complete"}])
        self.refused([{"status": "complete", "git": {"sha": OTHER_SHA, "dirty": False}}, rec("R1")])
        self.refused([{"status": "complete", "git": {"sha": SHA, "dirty": True}}])

    def test_every_problem_is_listed_and_long_lists_are_capped(self):
        error = self.refused([rec("R1", dirty=True), rec("R2", sha="bad"), rec("R3", sha=OTHER_SHA)])
        self.assertEqual(len(error.problems), 3)                          # dirty, malformed sha, several commits
        many = [rec(i, dirty=True) for i in ("R1", "R2", "R3", "R4", "R5", "R6", "R7", "R8", "R9", "R10", "R11", "R12")]
        error = self.refused(many)
        self.assertEqual(len(error.problems), 12)
        self.assertIn("and 2 more", str(error))

    def test_refusal_comes_before_any_coverage_or_gate_evaluation(self):
        from unittest import mock
        with mock.patch.object(rep, "_gates", side_effect=AssertionError("gates evaluated")), \
                mock.patch.object(rep, "_index", side_effect=AssertionError("records indexed")):
            with self.assertRaises(rep.ProvenanceError):
                rep.build_summary([rec("R1", dirty=True)])
            with self.assertRaises(rep.ProvenanceError):
                rep.build_summary([rec("R1"), rec("R2", sha=OTHER_SHA)])
            with self.assertRaises(rep.ProvenanceError):
                rep.build_summary([rec("R1"), rec("R1")])

    def test_expected_sha_flags_another_commit_even_if_every_record_agrees(self):
        records = [rec("R1"), rec("R2")]
        self.assertEqual(rep.provenance_problems(records, expected_sha=SHA), [])
        problems = rep.provenance_problems(records, expected_sha=OTHER_SHA)
        self.assertEqual(len(problems), 1)
        self.assertIn(SHA[:12], problems[0])
        self.assertIn(OTHER_SHA[:12], problems[0])

    def test_empty_input_still_gives_the_empty_structure(self):
        self.assertIsNone(rep.check_provenance([]))
        s = rep.build_summary([], [])
        self.assertIsNone(s["environment"])
        self.assertEqual({e["item_status"] for e in s["cases"].values()}, {"missing"})


class Calibration(unittest.TestCase):
    def test_a_known_linear_model_is_recovered(self):
        records = []
        cases = [c for c in cmp.all_cases().values() if c.mode.startswith("pb")][:10]
        for i, case in enumerate(cases):
            work = dict(WORK, lattice_expansions=1000 * (i + 1), leg_calls=2 + (i % 3), nodes_expanded=5 + i * i)
            seconds = 0.002 * work["lattice_expansions"] + 0.5 * work["leg_calls"] + 0.01 * work["nodes_expanded"] + 1.0
            records.append(rec(case.case_id, work=work, planning=seconds))
        fit = rep.build_summary(records)["calibration"]
        self.assertEqual(fit["status"], "fitted")
        self.assertAlmostEqual(fit["coefficients"]["a"], 0.002, places=6)
        self.assertAlmostEqual(fit["coefficients"]["b"], 0.5, places=4)
        self.assertGreater(fit["r_squared"], 0.999999)

    def test_too_few_records_report_insufficient_data(self):
        self.assertEqual(rep.build_summary([rec("R4")])["calibration"]["status"], "insufficient_data")

    def test_least_squares_detects_a_singular_system(self):
        self.assertIsNone(rep.least_squares([[1.0, 2.0], [2.0, 4.0], [3.0, 6.0]], [1.0, 2.0, 3.0]))


class ReportCli(unittest.TestCase):
    def test_report_with_no_input_writes_the_empty_structure_and_says_not_started(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "nested" / "summary.json"
            done = subprocess.run([sys.executable, str(SCRIPT), "report", "--summary-out", str(out)], cwd=ALGO,
                                  capture_output=True, text=True, timeout=60)
            self.assertEqual(done.returncode, 0, done.stderr)
            self.assertIn("minimum evidence set: not_started", done.stdout)
            self.assertEqual(json.loads(out.read_text(encoding="utf-8")), json.loads(rep.dumps_summary(rep.build_summary([], []))))
            self.assertIn("report_s", done.stderr)

    def test_report_hashes_its_input(self):
        with tempfile.TemporaryDirectory() as folder:
            raw = Path(folder) / "raw.jsonl"
            raw.write_text(json.dumps(rec("R1")) + "\n", encoding="utf-8")
            done = subprocess.run([sys.executable, str(SCRIPT), "report", "--input", str(raw), "--format", "json"],
                                  cwd=ALGO, capture_output=True, text=True, timeout=60)
            self.assertEqual(done.returncode, 0, done.stderr)
            summary = json.loads(done.stdout)
            self.assertEqual(summary["inputs"][0]["sha256"], hashlib.sha256(raw.read_bytes()).hexdigest())
            self.assertEqual(summary["cases"]["R1"]["item_status"], "satisfied")

    def run_report(self, folder, *records):
        raw = Path(folder) / "raw.jsonl"
        raw.write_text("".join(json.dumps(r) + "\n" for r in records), encoding="utf-8")
        out = Path(folder) / "nested" / "summary.json"
        done = subprocess.run([sys.executable, str(SCRIPT), "report", "--input", str(raw), "--summary-out", str(out)],
                              cwd=ALGO, capture_output=True, text=True, timeout=60)
        return done, out

    def test_report_refuses_unclean_evidence_with_a_dedicated_exit_code_and_writes_nothing(self):
        cases = {"mixed commits": [rec("R1"), rec("R2", sha=OTHER_SHA)], "dirty": [rec("R1", dirty=True)],
                 "dirty unknown": [rec("R1", dirty=None)], "malformed sha": [rec("R1", sha="abc")],
                 "duplicate": [rec("R1"), rec("R1")]}
        for label, records in cases.items():
            with self.subTest(label), tempfile.TemporaryDirectory() as folder:
                done, out = self.run_report(folder, *records)
                self.assertEqual(done.returncode, 6, done.stderr)
                self.assertEqual(done.stdout, "")                      # no summary text either
                self.assertIn("report refused", done.stderr)
                self.assertFalse(out.exists())
                self.assertFalse(out.parent.exists())                  # not even the output directory

    def test_a_refused_report_leaves_an_existing_summary_untouched(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "nested" / "summary.json"
            out.parent.mkdir()
            out.write_text("previous summary\n", encoding="utf-8")
            done, _ = self.run_report(folder, rec("R1", dirty=True))
            self.assertEqual(done.returncode, 6, done.stderr)
            self.assertEqual(out.read_text(encoding="utf-8"), "previous summary\n")

    def test_a_clean_single_commit_report_still_writes_the_summary(self):
        with tempfile.TemporaryDirectory() as folder:
            done, out = self.run_report(folder, rec("R1"), rec("Q5#1"), rec("Q5#2"))
            self.assertEqual(done.returncode, 0, done.stderr)
            written = json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual(written["environment"]["git_shas"], [SHA])
            self.assertEqual(written["records_total"], 3)

    def test_exit_code_six_is_new_and_distinct(self):
        from benchmarks import bench_executable_planner as bench
        codes = (0, bench.EXIT_ERROR, bench.EXIT_REFUSED, bench.EXIT_ABORT, bench.EXIT_VIOLATIONS,
                 bench.EXIT_INCOMPLETE, bench.EXIT_PROVENANCE)
        self.assertEqual(bench.EXIT_PROVENANCE, 6)
        self.assertEqual(len(set(codes)), len(codes))


if __name__ == "__main__":
    unittest.main()
