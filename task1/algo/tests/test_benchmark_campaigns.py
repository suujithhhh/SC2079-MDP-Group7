"""Benchmark campaign tables, dry-run arithmetic, coverage manifest and the
session budget logic (`algo/benchmarks/campaigns.py`).

Nothing here runs a planner. The tables are pinned to their recalculated
worst-case totals so a change to a timeout, a repeat count or the per-invocation
allowance cannot silently push a session past its cap.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ALGO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ALGO))

from benchmarks import bench_executable_planner as bench  # noqa: E402
from benchmarks import campaigns as cmp  # noqa: E402

EXPECTED = {
    # session: (sum of timeout x repeats, invocations, worst case with the 15 s allowance, cap)
    "Q": (470, 11, 635, 720),
    "S1": (2940, 21, 3255, 3600),
    "S2": (3300, 10, 3450, 3600),
    "S3": (1800, 3, 1845, 3600),
    "E1": (3000, 5, 3075, 3600),
    "E2": (3000, 7, 3105, 3600),
    "E3": (2760, 6, 2850, 3600),
    "BQ": (180, 2, 210, 720),
    "B1": (1440, 16, 1680, 3600),
    "B2": (1530, 17, 1785, 3600),
    "B3": (690, 4, 750, 3600),
}


def record(status):
    return {"status": status}


class Arithmetic(unittest.TestCase):
    def test_allowance_is_fifteen_seconds_per_invocation(self):
        self.assertEqual(cmp.ALLOWANCE_S, 15)

    def test_every_session_matches_the_recalculated_totals_and_fits_its_cap(self):
        seen = set()
        for session in cmp.all_sessions():
            with self.subTest(session=session.name):
                a = cmp.session_arithmetic(session)
                timeouts, invocations, worst, cap = EXPECTED[session.name]
                self.assertEqual((a["timeouts_s"], a["invocations"], a["worst_case_s"], a["cap_s"]),
                                 (timeouts, invocations, worst, cap))
                self.assertEqual(a["allowance_s"], 15 * invocations)
                self.assertEqual(a["worst_case_s"], a["timeouts_s"] + a["allowance_s"])
                self.assertTrue(a["fits"])
                self.assertLessEqual(a["worst_case_s"], session.cap_s)
                seen.add(session.name)
        self.assertEqual(seen, set(EXPECTED))

    def test_repeats_are_counted(self):
        q5 = next(c for c in cmp.QUICK if c.case_id == "Q5")
        self.assertEqual(q5.repeats, 2)
        self.assertEqual(q5.invocations(), ("Q5#1", "Q5#2"))
        rows = {r["case_id"]: r for r in cmp.session_arithmetic(cmp.find_session("quick"))["rows"]}
        self.assertEqual(rows["Q5"]["subtotal_s"], 120)

    def test_quick_cap_and_standard_minimum_evidence_total(self):
        self.assertEqual(cmp.find_session("quick").cap_s, 720)
        self.assertEqual(sum(cmp.session_arithmetic(s)["worst_case_s"] for s in cmp.SESSIONS["standard"]), 8550)
        self.assertTrue(all(s.cap_s == 3600 for s in cmp.SESSIONS["standard"] + cmp.SESSIONS["extended"]))


class Rules(unittest.TestCase):
    def test_case_ids_are_unique_and_timeouts_are_explicit_and_bounded(self):
        ids = [c.case_id for s in cmp.all_sessions() for c in s.cases]
        self.assertEqual(len(ids), len(set(ids)))
        for case in cmp.all_cases().values():
            with self.subTest(case=case.case_id):
                self.assertIsInstance(case.timeout_s, int)
                self.assertTrue(0 < case.timeout_s <= cmp.MAX_TIMEOUT_S)
                self.assertTrue(case.repeats >= 1)
                self.assertTrue(set(case.accept) <= set("CPT"))
                self.assertIn(case.mode, ("b2", "b3", "pb2", "pb3", "bpb2", "bpb3"))
                self.assertIn(case.alternatives, ("off", "on"))
                self.assertTrue(set(case.option_dict()) <= cmp.OPTION_KEYS)

    def test_old_search_cases_stay_within_the_old_search_cap(self):
        old = [c for c in cmp.all_cases().values() if c.mode in ("b2", "b3")]
        self.assertGreaterEqual(len(old), 6)
        for case in old:
            with self.subTest(case=case.case_id):
                self.assertLessEqual(case.timeout_s, cmp.OLD_SEARCH_CAP_S)
                self.assertEqual(cmp.OLD_SEARCH_CAP_S, 120)
                from benchmarks import fixtures as bfx
                timeout, refusal = bench.resolve_timeout(
                    case.mode, bfx.load_fixture(case.fixture).n_obstacles, case.timeout_s, False)
                self.assertIsNone(refusal, case.fixture)
                self.assertEqual(timeout, case.timeout_s)

    def test_r21_and_r34_use_the_old_search_cap(self):
        cases = cmp.all_cases()
        self.assertEqual(cases["R21"].timeout_s, 120)
        self.assertEqual(cases["R34"].timeout_s, 120)

    def test_planner_checkpoint_is_never_implicit(self):
        # Only the overshoot experiment sets one; every other case, including all
        # exactness, determinism and proof cases, relies on the hard timeout alone.
        with_checkpoint = [c.case_id for c in cmp.all_cases().values() if "checkpoint_s" in c.option_dict()]
        self.assertEqual(with_checkpoint, ["R29"])
        for case_id in ("R2", "R5", "R8", "R10", "R16", "R17", "X1"):
            self.assertEqual(cmp.all_cases()[case_id].option_dict(), {})

    def test_determinism_pairs_reference_real_invocations(self):
        invocations = {inv for s in cmp.all_sessions() for _c, inv in cmp.session_invocations(s)}
        for pair in cmp.DETERMINISM_PAIRS:
            self.assertTrue(set(pair) <= invocations, pair)


class MinimumEvidence(unittest.TestCase):
    def test_representative_obstacle_counts_and_edge_cases_are_covered(self):
        standard = {c.case_id: c for s in cmp.SESSIONS["standard"] for c in s.cases}
        from benchmarks import fixtures as bfx
        counts = {bfx.load_fixture(c.fixture).n_obstacles for c in standard.values()}
        self.assertTrue({1, 2, 3, 4, 6, 8} <= counts)
        for name in ("n1", "n2", "n3", "n4", "n6", "n8", "unsafe_primary", "no_safe_pose", "reversal",
                     "budget", "cancellation", "frontier", "checkpoint_overshoot", "alternatives_on",
                     "hotspots_3_4", "exactness"):
            self.assertIn(name, cmp.REQUIREMENTS)
            self.assertTrue(cmp.REQUIREMENTS[name])
            for case_id in cmp.REQUIREMENTS[name]:
                self.assertIn(case_id, standard, (name, case_id))

    def test_budget_cancellation_and_frontier_cases_carry_their_options(self):
        cases = cmp.all_cases()
        self.assertIn("cancel_after_legs", cases["R27"].option_dict())
        self.assertIn("max_frontier", cases["R28"].option_dict())
        for case_id in cmp.REQUIREMENTS["budget"]:
            self.assertIn("max_leg_calls", cases[case_id].option_dict())

    def test_cprofile_is_only_asked_at_three_or_four_obstacles(self):
        profiled = [c for c in cmp.all_cases().values() if c.option_dict().get("profile")]
        self.assertTrue(profiled)
        for case in profiled:
            self.assertTrue(case.fixture in ("A_spread_03", "B_clutter_04"), case.case_id)


class SessionBudget(unittest.TestCase):
    def test_fresh_session_runs_everything(self):
        s = cmp.find_session("quick")
        plan = cmp.plan_session_run(s, frozenset())
        self.assertEqual(len(plan["to_run"]), 11)
        self.assertEqual(plan["refused"], [])
        self.assertEqual(plan["remaining_s"], 720 - 635)

    def test_resume_skips_recorded_invocations(self):
        s = cmp.find_session("quick")
        plan = cmp.plan_session_run(s, frozenset({"Q1", "Q2", "Q5#1"}))
        run = [inv for _c, inv in plan["to_run"]]
        self.assertNotIn("Q1", run)
        self.assertNotIn("Q5#1", run)
        self.assertIn("Q5#2", run)
        self.assertEqual(len(run), 8)

    def test_a_case_that_does_not_fit_is_refused_and_everything_after_it_is_too(self):
        s = cmp.find_session("standard", "S1")
        plan = cmp.plan_session_run(s, frozenset(), budget_s=500)
        run = [inv for _c, inv in plan["to_run"]]
        refused = [inv for _c, inv in plan["refused"]]
        self.assertEqual(run + refused, [inv for _c, inv in cmp.session_invocations(s)])
        self.assertTrue(refused)
        cost = sum(c.timeout_s + cmp.ALLOWANCE_S for c, _ in plan["to_run"])
        self.assertLessEqual(cost, 500)
        # A later, smaller case is never started past a refused one.
        first_refused = plan["refused"][0][0]
        self.assertGreater(first_refused.timeout_s + cmp.ALLOWANCE_S, plan["remaining_s"])

    def test_zero_budget_refuses_every_required_invocation(self):
        s = cmp.find_session("standard", "S3")
        plan = cmp.plan_session_run(s, frozenset(), budget_s=0)
        self.assertEqual(plan["to_run"], [])
        self.assertEqual(len(plan["refused"]), 3)

    def test_unknown_session_and_ambiguous_campaign(self):
        with self.assertRaises(KeyError):
            cmp.find_session("standard", "S9")
        with self.assertRaises(KeyError):
            cmp.find_session("standard")


class Coverage(unittest.TestCase):
    def test_outcome_letters(self):
        for status, letter in (("complete", "C"), ("partial", "P"), ("no_route", "P"),
                               ("timeout", "T"), ("error", "E"), ("refused", "R")):
            self.assertEqual(cmp.outcome_letter(record(status)), letter)

    def test_item_status(self):
        c_only = cmp.all_cases()["R1"]          # accepts C
        ct = cmp.all_cases()["R11"]             # accepts C, T
        p = cmp.all_cases()["R22"]              # accepts P
        self.assertEqual(cmp.item_status(c_only, None), "missing")
        self.assertEqual(cmp.item_status(c_only, record("complete")), "satisfied")
        self.assertEqual(cmp.item_status(c_only, record("timeout")), "unsatisfied")
        self.assertEqual(cmp.item_status(c_only, record("error")), "unsatisfied")
        self.assertEqual(cmp.item_status(ct, record("timeout")), "timeout")
        self.assertEqual(cmp.item_status(ct, record("complete")), "satisfied")
        self.assertEqual(cmp.item_status(p, record("no_route")), "satisfied")
        self.assertEqual(cmp.item_status(p, record("complete")), "unsatisfied")

    def test_coverage_status(self):
        self.assertEqual(cmp.coverage_status([]), "not_started")
        self.assertEqual(cmp.coverage_status(["missing", "missing"]), "not_started")
        self.assertEqual(cmp.coverage_status(["satisfied", "timeout"]), "complete")
        self.assertEqual(cmp.coverage_status(["satisfied", "missing"]), "partial")
        self.assertEqual(cmp.coverage_status(["satisfied", "unsatisfied"]), "partial")
        self.assertEqual(cmp.coverage_status(["unsatisfied", "unsatisfied"]), "partial")


class SevenObstacleRepresentative(unittest.TestCase):
    def test_x18_is_a_finite_unlimited_alternatives_off_pb2_case_in_an_extended_session(self):
        case = cmp.all_cases()["X18"]
        self.assertEqual((case.fixture, case.mode, case.alternatives, case.accept),
                         ("A_spread_07", "pb2", "off", "CT"))
        self.assertEqual(case.option_dict(), {})                      # no planner limit or checkpoint
        self.assertEqual((case.timeout_s, case.repeats), (600, 1))
        owner = next(s for s in cmp.SESSIONS["extended"] if case in s.cases)
        self.assertEqual(owner.name, "E3")
        a = cmp.session_arithmetic(owner)
        self.assertEqual((a["timeouts_s"], a["invocations"], a["allowance_s"], a["worst_case_s"]),
                         (2760, 6, 90, 2850))
        self.assertLessEqual(a["worst_case_s"], 3600)

    def test_every_extended_session_still_fits_its_cap(self):
        for session in cmp.SESSIONS["extended"]:
            with self.subTest(session=session.name):
                self.assertLessEqual(cmp.session_arithmetic(session)["worst_case_s"], 3600)

    def test_case_and_invocation_counts(self):
        invocations = [inv for s in cmp.all_sessions() for _c, inv in cmp.session_invocations(s)]
        self.assertEqual(len(invocations), 102)
        self.assertEqual(len(set(invocations)), 102)
        self.assertEqual(len(cmp.all_cases()), 94)    # Q5, BS1 two invocations; BR1, BR3, BR5 three
        self.assertEqual(len(cmp.manifest()["campaigns"]["extended"][2]["cases"]), 6)


class StartGuard(unittest.TestCase):
    """`campaign` starts only from a known, clean Git state and only resumes clean evidence of
    the same commit. No planner runs here: `--budget-s 0` leaves every case unstarted, and a
    refusal returns before any case is planned."""

    SHA = "0123456789abcdef0123456789abcdef01234567"
    OTHER = "fedcba9876543210fedcba9876543210fedcba98"
    CLEAN = {"sha": SHA, "dirty": False}

    def campaign(self, folder, state, *extra, name="run.jsonl"):
        out = Path(folder) / name
        argv = ["campaign", "standard", "--session", "S3", "--out", str(out), "--budget-s", "0", *extra]
        stderr = io.StringIO()
        with mock.patch.object(bench, "git_info", return_value=state), mock.patch.object(bench, "_GIT", None), \
                contextlib.redirect_stderr(stderr):
            code = bench.main(argv)
            recorded = bench._GIT
        return code, stderr.getvalue(), out, recorded

    @staticmethod
    def write(path, *records):
        path.write_text("".join(json.dumps(r) + "\n" for r in records), encoding="utf-8")

    def record(self, invocation, sha=None, dirty=False, **more):
        git = {"sha": self.SHA if sha is None else sha, "dirty": dirty}
        return {"case": {"invocation": invocation}, "git": git, **more}

    def test_a_dirty_or_unknown_state_is_refused_before_anything_runs(self):
        states = {"dirty": {"sha": self.SHA, "dirty": True}, "dirty unknown": {"sha": self.SHA, "dirty": None},
                  "dirty not a bool": {"sha": self.SHA, "dirty": "clean"}, "no sha": {"sha": None, "dirty": False},
                  "short sha": {"sha": "abc", "dirty": False}, "upper-case sha": {"sha": self.SHA.upper(), "dirty": False},
                  "empty": {}}
        for label, state in states.items():
            with self.subTest(label), tempfile.TemporaryDirectory() as folder:
                code, err, out, _ = self.campaign(folder, state)
                self.assertEqual(code, bench.EXIT_PROVENANCE)
                self.assertIn("campaign refused", err)
                self.assertFalse(out.exists())

    def test_a_clean_known_state_passes_the_guard_and_is_what_records_will_carry(self):
        with tempfile.TemporaryDirectory() as folder:
            code, err, out, recorded = self.campaign(folder, self.CLEAN)
            self.assertEqual(code, bench.EXIT_INCOMPLETE, err)          # zero budget: nothing fits, nothing runs
            self.assertNotIn("refused", err)
            self.assertEqual(recorded, self.CLEAN)
            self.assertFalse(out.exists())

    def test_a_dry_run_is_not_guarded_and_prints_the_unchanged_arithmetic(self):
        stdout = io.StringIO()
        dirty = {"sha": None, "dirty": True}
        with mock.patch.object(bench, "git_info", return_value=dirty), contextlib.redirect_stdout(stdout):
            code = bench.main(["campaign", "standard", "--dry-run", "--session", "S3"])
        self.assertEqual(code, 0)
        row = json.loads(stdout.getvalue())
        self.assertEqual((row["session"], row["timeouts_s"], row["invocations"], row["worst_case_s"], row["cap_s"]),
                         ("S3", 1800, 3, 1845, 3600))

    def test_resume_accepts_clean_evidence_of_the_current_commit(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "run.jsonl"
            self.write(out, self.record("R16"), self.record("R17"))
            before = out.read_bytes()
            code, err, _, _ = self.campaign(folder, self.CLEAN, "--resume")
            self.assertEqual(code, bench.EXIT_INCOMPLETE, err)
            self.assertEqual(out.read_bytes(), before)
            out.write_text("", encoding="utf-8")                        # an empty file is nothing to refuse
            self.assertEqual(self.campaign(folder, self.CLEAN, "--resume")[0], bench.EXIT_INCOMPLETE)
        with tempfile.TemporaryDirectory() as folder:                   # nothing to resume yet
            self.assertEqual(self.campaign(folder, self.CLEAN, "--resume")[0], bench.EXIT_INCOMPLETE)

    def test_resume_refuses_a_file_from_another_commit(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "run.jsonl"
            self.write(out, self.record("R16", sha=self.OTHER))
            before = out.read_bytes()
            code, err, _, _ = self.campaign(folder, self.CLEAN, "--resume")
            self.assertEqual(code, bench.EXIT_PROVENANCE)
            self.assertIn("campaign refused", err)
            self.assertIn(self.OTHER[:12], err)
            self.assertIn(self.SHA[:12], err)
            self.assertEqual(out.read_bytes(), before)                  # never modified

    def test_resume_refuses_any_file_the_report_would_refuse(self):
        files = {
            "mixed commits": [self.record("R16"), self.record("R17", sha=self.OTHER)],
            "dirty record": [self.record("R16"), self.record("R17", dirty=True)],
            "dirty unknown": [self.record("R16", dirty=None)],
            "no git block": [{"case": {"invocation": "R16"}}],
            "malformed sha": [self.record("R16", sha="abc")],
            "duplicate invocation": [self.record("R16"), self.record("R16")],
        }
        for label, records in files.items():
            with self.subTest(label), tempfile.TemporaryDirectory() as folder:
                out = Path(folder) / "run.jsonl"
                self.write(out, *records)
                code, err, _, _ = self.campaign(folder, self.CLEAN, "--resume")
                self.assertEqual(code, bench.EXIT_PROVENANCE, err)
                self.assertIn("cannot resume", err)

    def test_a_dirty_state_is_refused_even_when_resuming_a_matching_file(self):
        with tempfile.TemporaryDirectory() as folder:
            self.write(Path(folder) / "run.jsonl", self.record("R16"))
            code, err, _, _ = self.campaign(folder, {"sha": self.SHA, "dirty": True}, "--resume")
            self.assertEqual(code, bench.EXIT_PROVENANCE)
            self.assertIn("working tree is dirty", err)

    def test_starting_over_an_existing_file_is_still_refused_without_resume(self):
        with tempfile.TemporaryDirectory() as folder:
            self.write(Path(folder) / "run.jsonl", self.record("R16"))
            with self.assertRaises(SystemExit) as caught, contextlib.redirect_stderr(io.StringIO()):
                self.campaign(folder, self.CLEAN)
            self.assertIn("--resume", str(caught.exception))


class Documentation(unittest.TestCase):
    def test_markdown_links_resolve(self):
        import re
        pattern = re.compile(r"\]\(([^)\s]+)\)")
        for path in (ALGO / "benchmarks" / "README.md",):
            text = path.read_text(encoding="utf-8")
            for target in pattern.findall(text):
                if target.startswith(("http://", "https://", "#", "mailto:")):
                    continue
                relative = re.sub(r"(#.*|:\d+)$", "", target)
                with self.subTest(file=path.name, link=target):
                    self.assertTrue((path.parent / relative).exists(), f"{path.name}: broken link {target}")

    def test_readme_arithmetic_table_matches_the_campaign_tables(self):
        readme = (ALGO / "benchmarks" / "README.md").read_text(encoding="utf-8")
        for session in cmp.all_sessions():
            a = cmp.session_arithmetic(session)
            row = (f"| {a['timeouts_s']:,} s | {a['invocations']} | {a['allowance_s']} s | "
                   f"{a['worst_case_s']:,} s | {a['cap_s']:,} s |")
            with self.subTest(session=session.name):
                self.assertIn(row, readme)

    def test_readme_has_one_status_row_per_gate(self):
        # The statuses themselves are checked against the tracked summary in
        # test_prefix_validation_summary.TrackedSummary.
        readme = (ALGO / "benchmarks" / "README.md").read_text(encoding="utf-8")
        gate_rows = [line for line in readme.splitlines() if line[:4] in {f"| G{i}" for i in range(1, 9)}]
        self.assertEqual(len(gate_rows), 8)
        for row in gate_rows:
            self.assertRegex(row.rstrip(), r"\| `(pass|fail|not_measured)` \|$")

    def test_future_timing_wording(self):
        readme = (ALGO / "benchmarks" / "README.md").read_text(encoding="utf-8")
        self.assertIn("ENTIRE `optimise_prefix_b3_route`", readme)
        self.assertIn("including its B2 seed phase", readme)
        self.assertIn("separate 45-second production hard backstop", readme)
        module = (ALGO / "route_prefix_search.py").read_text(encoding="utf-8")
        self.assertNotIn("backstop", module)          # the module never calls its checkpoint a backstop


class ManifestShape(unittest.TestCase):
    def test_manifest_is_deterministic_and_lists_every_case(self):
        first, second = cmp.manifest(), cmp.manifest()
        self.assertEqual(first, second)
        self.assertEqual(first["manifest_version"], cmp.MANIFEST_VERSION)
        listed = {c["case_id"] for sessions in first["campaigns"].values() for s in sessions for c in s["cases"]}
        self.assertEqual(listed, set(cmp.all_cases()))


if __name__ == "__main__":
    unittest.main()
