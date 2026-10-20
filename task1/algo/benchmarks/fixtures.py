"""Deterministic benchmark fixtures for the executable route optimizer.

A fixture is one JSON file in `fixtures/` describing a start pose and an
obstacle layout in centimetres, in the same shape as a protocol-v2 request
(`start.heading_deg`, `obstacles[].id/x/y/face`) plus benchmark metadata. It
is deliberately NOT a full `PLAN_REQUEST`: 6-8 obstacle fixtures exist to
measure future implementations, while the service maximum stays 5, so they
are only ever fed to the optimizer directly, never to `/plan/v2`.

Metadata:
  fixture_id            equals the file name without `.json`
  family                A_spread | B_clutter | C_edge | D_seeded | E_ordering
  expect                feasible | infeasible | unverified (see below)
  tags                  free-form labels (reversal, unsafe_primary, ...)
  requires_alternatives true when the layout needs alternative viewing poses
  verification          what has actually been checked, in words

`expect` is a static, geometry-level label, never a promise about planner
runtime: `feasible` = every obstacle has a safe primary viewing pose;
`infeasible` = at least one obstacle has NO safe viewing pose at all, so no
complete route can exist; `unverified` = a layout kept for future
validation whose planner behaviour has not been established.
"""

import json
import math
import random
from dataclasses import dataclass, replace
from pathlib import Path

import footprint
from config import (
    BENCHMARK_FIXTURE_CAMERA_STANDOFF_CM,
    BENCHMARK_FIXTURE_GOAL_POSITION_TOLERANCE_CM,
    BENCHMARK_FIXTURE_SAFETY_MARGIN_CM,
    DEFAULT_PLANNER_CONFIG,
    PlannerConfig,
)
from models import Obstacle, Pose
from motion_primitives import normalize_heading
from targets import viewing_target, viewing_targets

# The committed fixtures (and the generators that must reproduce them byte for
# byte) are defined at the geometry they were built for: a 1 cm safety margin
# and a 20 cm camera standoff. The live planner defaults moved to 3 cm / 25 cm,
# under which nine committed fixtures have no safe viewing pose, so fixture
# generation, validation and `viewing_report` default to this historical
# geometry instead. Benchmark *runs* (replay, campaigns) keep using the live
# config and therefore measure the live planner on these fixtures.
FIXTURE_GEOMETRY = replace(
    DEFAULT_PLANNER_CONFIG,
    safety_margin_cm=BENCHMARK_FIXTURE_SAFETY_MARGIN_CM,
    camera_standoff_cm=BENCHMARK_FIXTURE_CAMERA_STANDOFF_CM,
    lattice_goal_position_tolerance_cm=BENCHMARK_FIXTURE_GOAL_POSITION_TOLERANCE_CM,
)

FIXTURE_DIR = Path(__file__).resolve().parent / "fixtures"
SCHEMA_VERSION = 1
FAMILIES = ("A_spread", "B_clutter", "C_edge", "D_seeded", "E_ordering")
EXPECTATIONS = ("feasible", "infeasible", "unverified")
FACES = ("N", "E", "S", "W")
SEEDED_SIZES = (4, 6, 8)
SEEDED_VARIANTS = (1, 2, 3)
GENERATOR_VERSION = 1
# E_ordering: four-obstacle layouts built so the exactness comparison can be certified
# (see `generate_ordering_candidate`). The rules and the candidate list are fixed here
# before any planner run; `benchmarks/ordering_discovery.py` judges the candidates.
ORDERING_OBSTACLES = 4
ORDERING_CANDIDATES = (1, 2, 3, 4, 5, 6)
ORDERING_GENERATOR_VERSION = 1
ORDERING_SEED_BASE = 41000
ORDERING_FACE_SETS = (("S", "W"), ("S", "E"))
ORDERING_MIN_SPACING_CM = 50
ORDERING_MAX_HEADING_SPREAD_DEG = 90.0
ORDERING_START = {"x": 20, "y": 10, "heading_deg": 90}     # the start of every other fixture


class FixtureError(ValueError):
    """A fixture file is malformed or fails validation."""


@dataclass(frozen=True)
class Fixture:
    fixture_id: str
    family: str
    n_obstacles: int
    expect: str
    tags: tuple
    requires_alternatives: bool
    description: str
    verification: str
    start: Pose
    obstacles: tuple
    data: dict


def seeded_fixture_id(n_obstacles: int, variant: int) -> str:
    return f"D_seeded_{n_obstacles:02d}_s{variant}"


def seeded_seed(n_obstacles: int, variant: int) -> int:
    return n_obstacles * 1000 + variant


def validate_fixture_data(data: dict, config: PlannerConfig = FIXTURE_GEOMETRY) -> list:
    """Structural problems of one fixture dict (empty list = valid)."""
    errors = []
    if not isinstance(data, dict):
        return ["fixture must be a JSON object"]
    if data.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"schema_version must be {SCHEMA_VERSION}")
    if not isinstance(data.get("fixture_id"), str) or not data.get("fixture_id"):
        errors.append("fixture_id must be a non-empty string")
    if data.get("family") not in FAMILIES:
        errors.append(f"family must be one of {FAMILIES}")
    if data.get("expect") not in EXPECTATIONS:
        errors.append(f"expect must be one of {EXPECTATIONS}")
    tags = data.get("tags")
    if not isinstance(tags, list) or not all(isinstance(tag, str) for tag in tags):
        errors.append("tags must be a list of strings")
    if not isinstance(data.get("requires_alternatives"), bool):
        errors.append("requires_alternatives must be a boolean")
    for key in ("description", "verification"):
        if not isinstance(data.get(key), str) or not data.get(key):
            errors.append(f"{key} must be a non-empty string")
    start = data.get("start")
    if not isinstance(start, dict) or not all(
        isinstance(start.get(k), (int, float)) and not isinstance(start.get(k), bool)
        for k in ("x", "y", "heading_deg")
    ):
        errors.append("start must have numeric x, y and heading_deg")
    obstacles = data.get("obstacles")
    if not isinstance(obstacles, list) or not obstacles:
        errors.append("obstacles must be a non-empty list")
        return errors
    if data.get("n_obstacles") != len(obstacles):
        errors.append("n_obstacles must equal len(obstacles)")
    seen = set()
    limit = config.arena_cm - config.obstacle_cm
    for index, item in enumerate(obstacles):
        if not isinstance(item, dict):
            errors.append(f"obstacles[{index}] must be an object")
            continue
        ident = item.get("id")
        if isinstance(ident, bool) or not isinstance(ident, int) or ident <= 0:
            errors.append(f"obstacles[{index}].id must be a positive integer")
        elif ident in seen:
            errors.append(f"duplicate obstacle id {ident}")
        else:
            seen.add(ident)
        if item.get("face") not in FACES:
            errors.append(f"obstacles[{index}].face must be one of {FACES}")
        for axis in ("x", "y"):
            value = item.get(axis)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                errors.append(f"obstacles[{index}].{axis} must be a finite number")
            elif not 0 <= value <= limit:
                errors.append(f"obstacles[{index}].{axis}={value} lies outside the arena (0..{limit})")
    return errors


def fixture_from_data(data: dict, config: PlannerConfig = FIXTURE_GEOMETRY) -> Fixture:
    errors = validate_fixture_data(data, config)
    if errors:
        raise FixtureError("; ".join(errors))
    start = Pose(
        float(data["start"]["x"]), float(data["start"]["y"]),
        normalize_heading(math.radians(float(data["start"]["heading_deg"]))),
    )
    obstacles = tuple(
        Obstacle(item["id"], float(item["x"]), float(item["y"]), item["face"])
        for item in data["obstacles"]
    )
    return Fixture(
        fixture_id=data["fixture_id"], family=data["family"], n_obstacles=len(obstacles),
        expect=data["expect"], tags=tuple(data["tags"]),
        requires_alternatives=data["requires_alternatives"],
        description=data["description"], verification=data["verification"],
        start=start, obstacles=obstacles, data=data,
    )


def list_fixture_ids() -> list:
    """Fixture ids ordered by family, obstacle count, then id."""
    entries = []
    for path in FIXTURE_DIR.glob("*.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        entries.append((FAMILIES.index(data["family"]), len(data["obstacles"]), path.stem))
    return [entry[2] for entry in sorted(entries)]


def load_fixture(fixture_id: str, config: PlannerConfig = FIXTURE_GEOMETRY) -> Fixture:
    path = FIXTURE_DIR / f"{fixture_id}.json"
    if not path.is_file():
        raise FixtureError(f"unknown fixture {fixture_id!r}")
    fixture = fixture_from_data(json.loads(path.read_text(encoding="utf-8")), config)
    if fixture.fixture_id != fixture_id:
        raise FixtureError(f"fixture_id {fixture.fixture_id!r} does not match file name {fixture_id!r}")
    return fixture


def viewing_report(fixture: Fixture, config: PlannerConfig = FIXTURE_GEOMETRY) -> dict:
    """Static viewing-pose facts, computed with the planner own geometry
    (`targets.viewing_target(s)` and `footprint.pose_is_safe`); nothing is
    re-derived here. Keys: `start_safe` and, per obstacle id, `primary_safe`,
    `safe_candidates` and `candidates`."""
    report = {"start_safe": footprint.pose_is_safe(fixture.start, fixture.obstacles, config)}
    for obstacle in fixture.obstacles:
        primary = viewing_target(obstacle, config)
        candidates = viewing_targets(obstacle, config)
        report[obstacle.obstacle_id] = {
            "primary_safe": footprint.pose_is_safe(primary, fixture.obstacles, config),
            "safe_candidates": sum(
                1 for pose in candidates if footprint.pose_is_safe(pose, fixture.obstacles, config)
            ),
            "candidates": len(candidates),
        }
    return report


def generate_seeded(n_obstacles: int, seed: int, config: PlannerConfig = FIXTURE_GEOMETRY,
                    max_attempts: int = 100000) -> list:
    """Deterministic random layout: obstacle lower-left corners on the 10 cm
    grid, random faces, at least 30 cm between obstacles, the start area
    kept clear, and every primary viewing pose safe against all obstacles.
    Uses only `random.Random(seed).randrange`, whose stream is stable across
    Python 3 versions. Returns the `obstacles` list of a fixture."""
    rng = random.Random(seed)
    cells = int(config.arena_cm // config.obstacle_cm)
    chosen: list = []
    for _ in range(max_attempts):
        if len(chosen) == n_obstacles:
            break
        x = rng.randrange(cells) * int(config.obstacle_cm)
        y = rng.randrange(cells) * int(config.obstacle_cm)
        face = FACES[rng.randrange(len(FACES))]
        if x < 50 and y < 50:
            continue
        if any(max(abs(x - o.x), abs(y - o.y)) < 40 for o in chosen):
            continue
        trial = chosen + [Obstacle(len(chosen) + 1, float(x), float(y), face)]
        if all(footprint.pose_is_safe(viewing_target(o, config), trial, config) for o in trial):
            chosen = trial
    if len(chosen) != n_obstacles:
        raise FixtureError(f"could not place {n_obstacles} obstacles for seed {seed}")
    return [{"id": o.obstacle_id, "x": int(o.x), "y": int(o.y), "face": o.image_face} for o in chosen]


def seeded_fixture_data(n_obstacles: int, variant: int, config: PlannerConfig = FIXTURE_GEOMETRY) -> dict:
    seed = seeded_seed(n_obstacles, variant)
    return {
        "schema_version": SCHEMA_VERSION,
        "fixture_id": seeded_fixture_id(n_obstacles, variant),
        "family": "D_seeded",
        "n_obstacles": n_obstacles,
        "expect": "feasible",
        "tags": ["seeded", f"seed_{seed}"],
        "requires_alternatives": False,
        "description": f"Seeded random {n_obstacles}-obstacle layout (seed {seed}).",
        "verification": "Generated by benchmarks.fixtures.generate_seeded; every primary viewing pose "
                        "is statically safe. Planner behaviour is not established.",
        "generator": {"name": "generate_seeded", "version": GENERATOR_VERSION, "seed": seed},
        "coordinate_units": "cm",
        "start": {"x": 20, "y": 10, "heading_deg": 90},
        "obstacles": generate_seeded(n_obstacles, seed, config),
    }


def dump_fixture(data: dict) -> str:
    return json.dumps(data, indent=2) + "\n"


def ordering_fixture_id(candidate: int) -> str:
    return f"E_ordering_{ORDERING_OBSTACLES:02d}_c{candidate}"


def ordering_seed(candidate: int) -> int:
    return ORDERING_SEED_BASE + candidate


def viewing_heading_spread_deg(start: Pose, obstacles, config: PlannerConfig = FIXTURE_GEOMETRY) -> float:
    """Largest circular difference, in degrees, between any two of the start heading and
    the primary viewing-pose headings. The ordering rule keeps it at most 90: a heading
    reversal is what makes state-lattice legs hit their expansion cap (benchmark README,
    H4), and an expansion-limited leg leaves unresolved branches that block a proof."""
    headings = [start.heading] + [viewing_target(o, config).heading for o in obstacles]
    widest = 0.0
    for i, first in enumerate(headings):
        for second in headings[i + 1:]:
            gap = abs(math.degrees(first - second)) % 360.0
            widest = max(widest, min(gap, 360.0 - gap))
    return widest


def generate_ordering_candidate(candidate: int, config: PlannerConfig = FIXTURE_GEOMETRY,
                                max_attempts: int = 100000) -> list:
    """Deterministic four-obstacle layout for candidate `candidate` (seed 41000 + candidate).

    The first random draw fixes the allowed faces: `S`/`W` or `S`/`E`, so every viewing
    heading is within 90 degrees of the start heading and of each other. Obstacle
    lower-left corners lie on the 10 cm grid, are at least 50 cm apart (Chebyshev), the
    start area stays clear, and the start and every primary viewing pose are safe against
    all obstacles. A layout that uses only one of the two faces is discarded and placement
    restarts on the same random stream. Uses only `random.Random(seed).randrange`.
    Returns the `obstacles` list of a fixture."""
    rng = random.Random(ordering_seed(candidate))
    faces = ORDERING_FACE_SETS[rng.randrange(len(ORDERING_FACE_SETS))]
    cells = int(config.arena_cm // config.obstacle_cm)
    start = Pose(ORDERING_START["x"], ORDERING_START["y"], math.radians(ORDERING_START["heading_deg"]))
    chosen: list = []
    for _ in range(max_attempts):
        if len(chosen) == ORDERING_OBSTACLES:
            if len({o.image_face for o in chosen}) == len(faces):
                break
            chosen = []
        x = rng.randrange(cells) * int(config.obstacle_cm)
        y = rng.randrange(cells) * int(config.obstacle_cm)
        face = faces[rng.randrange(len(faces))]
        if x < 50 and y < 50:
            continue
        if any(max(abs(x - o.x), abs(y - o.y)) < ORDERING_MIN_SPACING_CM for o in chosen):
            continue
        trial = chosen + [Obstacle(len(chosen) + 1, float(x), float(y), face)]
        if footprint.pose_is_safe(start, trial, config) and all(
                footprint.pose_is_safe(viewing_target(o, config), trial, config) for o in trial):
            chosen = trial
    if len(chosen) != ORDERING_OBSTACLES or len({o.image_face for o in chosen}) != len(faces):
        raise FixtureError(f"could not place {ORDERING_OBSTACLES} obstacles for ordering candidate {candidate}")
    return [{"id": o.obstacle_id, "x": int(o.x), "y": int(o.y), "face": o.image_face} for o in chosen]


def ordering_fixture_data(candidate: int, config: PlannerConfig = FIXTURE_GEOMETRY) -> dict:
    seed = ordering_seed(candidate)
    return {
        "schema_version": SCHEMA_VERSION,
        "fixture_id": ordering_fixture_id(candidate),
        "family": "E_ordering",
        "n_obstacles": ORDERING_OBSTACLES,
        "expect": "feasible",
        "tags": ["ordering", "candidate", f"seed_{seed}", f"candidate_{candidate}"],
        "requires_alternatives": False,
        "description": f"Four-obstacle ordering candidate {candidate} of {len(ORDERING_CANDIDATES)} "
                       f"(seed {seed}): two viewing faces, headings within 90 degrees of the start.",
        "verification": "Generated by benchmarks.fixtures.generate_ordering_candidate; the start and every "
                        "primary viewing pose are statically safe. Static checks only: planner behaviour is "
                        "established solely by the discovery ledger (benchmarks/ordering_discovery.py).",
        "generator": {"name": "generate_ordering_candidate", "version": ORDERING_GENERATOR_VERSION,
                      "seed": seed},
        "coordinate_units": "cm",
        "start": dict(ORDERING_START),
        "obstacles": generate_ordering_candidate(candidate, config),
    }
