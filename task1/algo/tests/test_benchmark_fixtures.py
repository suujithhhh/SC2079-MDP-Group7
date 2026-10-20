"""Benchmark fixture validity (`algo/benchmarks/fixtures/*.json`).

Static, geometry-level checks only: fixtures are measurement inputs, so
these tests prove ids are unique, layouts lie inside the arena, feasible
layouts have safe primary viewing poses, infeasible ones are labelled and
really have an obstacle with no safe viewing pose, and the seeded layouts
still equal what the committed generator produces.

Run from algo/:
    python -m unittest discover -s tests -v
"""

import json
import math
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import planning_service_v2 as psv2  # noqa: E402
from benchmarks import fixtures as bfx  # noqa: E402

REQUIRED_IDS = (
    [f"A_spread_{n:02d}" for n in (1, 2, 3, 4, 5, 6, 7, 8)]
    + [f"B_clutter_{n:02d}" for n in (4, 6, 8)]
    + ["C_unsafe_primary_02", "C_unsafe_primary_05", "C_unsafe_primary_08",
       "C_no_safe_pose_02", "C_no_safe_pose_05", "C_no_safe_pose_08", "C_reversal_02"]
    + [f"D_seeded_{n:02d}_s{v}" for n in (4, 6, 8) for v in (1, 2, 3)]
    + [f"E_ordering_04_c{k}" for k in (1, 2, 3, 4, 5, 6)]
)


class FixtureSet(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ids = bfx.list_fixture_ids()
        cls.fixtures = {fixture_id: bfx.load_fixture(fixture_id) for fixture_id in cls.ids}

    def test_required_fixtures_exist(self):
        for fixture_id in REQUIRED_IDS:
            self.assertIn(fixture_id, self.fixtures)

    def test_fixture_ids_are_unique_and_match_file_names(self):
        self.assertEqual(len(self.ids), len(set(self.ids)))
        files = sorted(p.stem for p in bfx.FIXTURE_DIR.glob("*.json"))
        self.assertEqual(sorted(self.ids), files)
        for fixture_id, fixture in self.fixtures.items():
            self.assertEqual(fixture.fixture_id, fixture_id)

    def test_every_fixture_passes_structural_validation(self):
        for path in bfx.FIXTURE_DIR.glob("*.json"):
            with self.subTest(fixture=path.stem):
                data = json.loads(path.read_text(encoding="utf-8"))
                self.assertEqual(bfx.validate_fixture_data(data), [])

    def test_obstacle_ids_are_unique_and_positive_within_each_fixture(self):
        for fixture_id, fixture in self.fixtures.items():
            with self.subTest(fixture=fixture_id):
                ids = [o.obstacle_id for o in fixture.obstacles]
                self.assertEqual(len(ids), len(set(ids)))
                self.assertTrue(all(i > 0 for i in ids))

    def test_obstacles_lie_inside_the_arena(self):
        limit = bfx.DEFAULT_PLANNER_CONFIG.arena_cm - bfx.DEFAULT_PLANNER_CONFIG.obstacle_cm
        for fixture_id, fixture in self.fixtures.items():
            for o in fixture.obstacles:
                with self.subTest(fixture=fixture_id, obstacle=o.obstacle_id):
                    self.assertTrue(0 <= o.x <= limit and 0 <= o.y <= limit)

    def test_start_pose_is_safe_in_every_fixture(self):
        for fixture_id, fixture in self.fixtures.items():
            with self.subTest(fixture=fixture_id):
                self.assertTrue(bfx.viewing_report(fixture)["start_safe"])

    def test_expect_labels_are_valid(self):
        for fixture_id, fixture in self.fixtures.items():
            with self.subTest(fixture=fixture_id):
                self.assertIn(fixture.expect, bfx.EXPECTATIONS)
                self.assertTrue(fixture.verification)

    def test_feasible_fixtures_have_safe_viewing_poses(self):
        for fixture_id, fixture in self.fixtures.items():
            if fixture.expect != "feasible":
                continue
            with self.subTest(fixture=fixture_id):
                report = bfx.viewing_report(fixture)
                per_obstacle = [report[o.obstacle_id] for o in fixture.obstacles]
                self.assertTrue(all(row["safe_candidates"] >= 1 for row in per_obstacle))
                if fixture.requires_alternatives:
                    self.assertTrue(any(not row["primary_safe"] for row in per_obstacle))
                else:
                    self.assertTrue(all(row["primary_safe"] for row in per_obstacle))

    def test_infeasible_fixtures_are_labelled_and_really_have_no_viewing_pose(self):
        infeasible = [f for f in self.fixtures.values() if f.expect == "infeasible"]
        self.assertGreaterEqual(len(infeasible), 2)
        for fixture in infeasible:
            with self.subTest(fixture=fixture.fixture_id):
                report = bfx.viewing_report(fixture)
                self.assertTrue(any(report[o.obstacle_id]["safe_candidates"] == 0 for o in fixture.obstacles))
                self.assertTrue({"unreachable", "no_safe_viewing_pose"} <= set(fixture.tags))
        for fixture in self.fixtures.values():
            if fixture.expect != "infeasible":
                report = bfx.viewing_report(fixture)
                with self.subTest(fixture=fixture.fixture_id):
                    self.assertTrue(all(report[o.obstacle_id]["safe_candidates"] >= 1 for o in fixture.obstacles))


class FixtureFamilies(unittest.TestCase):
    def test_spread_layouts_are_nested_prefixes_of_the_eight_obstacle_layout(self):
        full = bfx.load_fixture("A_spread_08").data["obstacles"]
        for n in (1, 2, 3, 4, 5, 6, 7, 8):
            with self.subTest(n=n):
                fixture = bfx.load_fixture(f"A_spread_{n:02d}")
                self.assertEqual(fixture.data["obstacles"], full[:n])
                self.assertEqual(fixture.n_obstacles, n)

    def test_clutter_and_seeded_sizes(self):
        for n in (4, 6, 8):
            self.assertEqual(bfx.load_fixture(f"B_clutter_{n:02d}").n_obstacles, n)
            for variant in (1, 2, 3):
                self.assertEqual(bfx.load_fixture(f"D_seeded_{n:02d}_s{variant}").n_obstacles, n)

    def test_all_validation_fixtures_fit_the_bounded_service_maximum(self):
        # The bounded /plan/v2 integration and validation fixtures share the
        # same eight-obstacle ceiling.
        large = [f for f in bfx.list_fixture_ids() if bfx.load_fixture(f).n_obstacles > psv2.MAX_OBSTACLES]
        self.assertEqual(large, [])
        self.assertEqual(psv2.MAX_OBSTACLES, 8)

    def test_edge_case_tags(self):
        self.assertTrue(bfx.load_fixture("C_unsafe_primary_02").requires_alternatives)
        report = bfx.viewing_report(bfx.load_fixture("C_unsafe_primary_02"))
        self.assertFalse(report[1]["primary_safe"])
        self.assertGreaterEqual(report[1]["safe_candidates"], 1)
        self.assertIn("reversal_observed", bfx.load_fixture("C_reversal_02").tags)

    def test_phase1_validation_fixtures(self):
        # Added for the prefix-search validation campaigns. Each unsafe-primary
        # layout has exactly one obstacle whose physical primary is unsafe (with
        # one safe alternative) and every other primary safe; the no-safe-pose
        # layout has an obstacle with no safe viewing pose at all.
        for fixture_id, size in (("C_unsafe_primary_05", 5), ("C_unsafe_primary_08", 8)):
            with self.subTest(fixture=fixture_id):
                fixture = bfx.load_fixture(fixture_id)
                report = bfx.viewing_report(fixture)
                self.assertEqual(fixture.n_obstacles, size)
                self.assertTrue(fixture.requires_alternatives)
                unsafe = [o.obstacle_id for o in fixture.obstacles if not report[o.obstacle_id]["primary_safe"]]
                self.assertEqual(unsafe, [1])
                self.assertEqual(report[1]["safe_candidates"], 1)
        fixture = bfx.load_fixture("C_no_safe_pose_05")
        self.assertEqual(fixture.expect, "infeasible")
        self.assertEqual(bfx.viewing_report(fixture)[5]["safe_candidates"], 0)
        self.assertEqual(bfx.load_fixture("A_spread_05").expect, "feasible")

    def test_every_campaign_fixture_exists(self):
        from benchmarks import campaigns
        known = set(bfx.list_fixture_ids())
        for case in campaigns.all_cases().values():
            with self.subTest(case=case.case_id):
                self.assertIn(case.fixture, known)


class OrderingGenerator(unittest.TestCase):
    """`E_ordering_04_c1..c6`: four-obstacle candidates for a certifiable exactness pair. These
    tests are static: they say nothing about planner behaviour (only the discovery ledger does)."""

    IDS = [bfx.ordering_fixture_id(k) for k in bfx.ORDERING_CANDIDATES]

    def test_the_declared_candidates_are_exactly_c1_to_c6(self):
        self.assertEqual(self.IDS, [f"E_ordering_04_c{k}" for k in (1, 2, 3, 4, 5, 6)])
        self.assertEqual([bfx.ordering_seed(k) for k in bfx.ORDERING_CANDIDATES], list(range(41001, 41007)))

    def test_committed_files_equal_the_generator_output(self):
        for k in bfx.ORDERING_CANDIDATES:
            with self.subTest(candidate=k):
                committed = json.loads((bfx.FIXTURE_DIR / f"{bfx.ordering_fixture_id(k)}.json")
                                       .read_text(encoding="utf-8"))
                self.assertEqual(committed, bfx.ordering_fixture_data(k))

    def test_generator_is_deterministic_and_seed_dependent(self):
        self.assertEqual(bfx.generate_ordering_candidate(3), bfx.generate_ordering_candidate(3))
        layouts = [json.dumps(bfx.generate_ordering_candidate(k)) for k in bfx.ORDERING_CANDIDATES]
        self.assertEqual(len(set(layouts)), len(layouts))

    def test_layout_rules(self):
        for fixture_id in self.IDS:
            with self.subTest(fixture=fixture_id):
                fixture = bfx.load_fixture(fixture_id)
                self.assertEqual(fixture.family, "E_ordering")
                self.assertEqual((fixture.n_obstacles, fixture.expect, fixture.requires_alternatives),
                                 (4, "feasible", False))
                self.assertEqual(fixture.data["start"], {"x": 20, "y": 10, "heading_deg": 90})
                faces = {o.image_face for o in fixture.obstacles}
                self.assertIn(faces, ({"S", "W"}, {"S", "E"}))
                for first in fixture.obstacles:
                    self.assertFalse(first.x < 50 and first.y < 50)
                    for second in fixture.obstacles:
                        if first is not second:
                            self.assertGreaterEqual(max(abs(first.x - second.x), abs(first.y - second.y)), 50)
                report = bfx.viewing_report(fixture)
                self.assertTrue(report["start_safe"])
                self.assertTrue(all(report[o.obstacle_id]["primary_safe"] for o in fixture.obstacles))

    def test_every_viewing_heading_is_within_ninety_degrees(self):
        for fixture_id in self.IDS:
            with self.subTest(fixture=fixture_id):
                fixture = bfx.load_fixture(fixture_id)
                spread = bfx.viewing_heading_spread_deg(fixture.start, fixture.obstacles)
                self.assertLessEqual(spread, bfx.ORDERING_MAX_HEADING_SPREAD_DEG + 1e-6)
                self.assertGreater(spread, 1.0)                # two distinct headings, not all parallel

    def test_the_existing_four_obstacle_fixtures_break_the_heading_rule(self):
        for fixture_id in ("A_spread_04", "B_clutter_04", "D_seeded_04_s1", "D_seeded_04_s2", "D_seeded_04_s3"):
            with self.subTest(fixture=fixture_id):
                fixture = bfx.load_fixture(fixture_id)
                self.assertGreater(bfx.viewing_heading_spread_deg(fixture.start, fixture.obstacles),
                                   bfx.ORDERING_MAX_HEADING_SPREAD_DEG + 1e-6)

    def test_the_heading_spread_is_circular(self):
        from models import Obstacle, Pose
        start = Pose(20.0, 10.0, math.radians(350.0))
        north_facing = Obstacle(1, 100.0, 100.0, "S")          # viewed heading 90 deg
        self.assertAlmostEqual(bfx.viewing_heading_spread_deg(start, (north_facing,)), 100.0, places=6)

    def test_writer_script_check_passes(self):
        script = Path(__file__).resolve().parents[1] / "benchmarks" / "make_ordering_fixtures.py"
        done = subprocess.run([sys.executable, str(script), "--check"], capture_output=True, text=True)
        self.assertEqual((done.returncode, done.stderr), (0, ""))

    def test_an_unplaceable_layout_raises_instead_of_returning_fewer_obstacles(self):
        with self.assertRaises(bfx.FixtureError):
            bfx.generate_ordering_candidate(1, max_attempts=3)


class SeededGenerator(unittest.TestCase):
    def test_committed_seeded_fixtures_equal_the_generator_output(self):
        for n in bfx.SEEDED_SIZES:
            for variant in bfx.SEEDED_VARIANTS:
                fixture_id = bfx.seeded_fixture_id(n, variant)
                with self.subTest(fixture=fixture_id):
                    committed = json.loads((bfx.FIXTURE_DIR / f"{fixture_id}.json").read_text(encoding="utf-8"))
                    self.assertEqual(committed, bfx.seeded_fixture_data(n, variant))

    def test_generator_is_deterministic_and_seed_dependent(self):
        self.assertEqual(bfx.generate_seeded(6, 123), bfx.generate_seeded(6, 123))
        self.assertNotEqual(bfx.generate_seeded(6, 123), bfx.generate_seeded(6, 124))

    def test_generated_layouts_keep_the_start_clear_and_obstacles_apart(self):
        for n in bfx.SEEDED_SIZES:
            for variant in bfx.SEEDED_VARIANTS:
                obstacles = bfx.generate_seeded(n, bfx.seeded_seed(n, variant))
                with self.subTest(n=n, variant=variant):
                    self.assertFalse(any(o["x"] < 50 and o["y"] < 50 for o in obstacles))
                    for i, a in enumerate(obstacles):
                        for b in obstacles[i + 1:]:
                            self.assertGreaterEqual(max(abs(a["x"] - b["x"]), abs(a["y"] - b["y"])), 40)


class Validation(unittest.TestCase):
    def valid(self):
        return json.loads(json.dumps(bfx.load_fixture("A_spread_02").data))

    def test_valid_data_has_no_errors(self):
        self.assertEqual(bfx.validate_fixture_data(self.valid()), [])

    def test_problems_are_reported(self):
        cases = {
            "duplicate id": lambda d: d["obstacles"][1].update(id=1),
            "outside arena": lambda d: d["obstacles"][0].update(x=195),
            "negative": lambda d: d["obstacles"][0].update(y=-1),
            "bad face": lambda d: d["obstacles"][0].update(face="Q"),
            "bad expect": lambda d: d.update(expect="maybe"),
            "bad family": lambda d: d.update(family="Z"),
            "count mismatch": lambda d: d.update(n_obstacles=5),
            "no start": lambda d: d.pop("start"),
            "bool coordinate": lambda d: d["obstacles"][0].update(x=True),
            "zero id": lambda d: d["obstacles"][0].update(id=0),
        }
        for label, mutate in cases.items():
            with self.subTest(case=label):
                data = self.valid()
                mutate(data)
                self.assertNotEqual(bfx.validate_fixture_data(data), [])
                with self.assertRaises(bfx.FixtureError):
                    bfx.fixture_from_data(data)

    def test_unknown_fixture_is_an_error(self):
        with self.assertRaises(bfx.FixtureError):
            bfx.load_fixture("does_not_exist")


if __name__ == "__main__":
    unittest.main()
