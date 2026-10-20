"""Bounded discovery of a certifiable four-obstacle exactness fixture (measurement only).

The G2 comparison of old B3 against prefix B3 at four obstacles (R13/R12) cannot be
certified on `A_spread_04`: its prefix B3 run has unresolved branches from
expansion-limited legs. This tool decides, by rules fixed here before any planner run,
which of the six declared candidates `E_ordering_04_c1`..`c6` (see
`benchmarks.fixtures.generate_ordering_candidate`) supplies a certifiable pair, and
writes every attempt to one ledger. It never runs a planner: the runner
(`bench_executable_planner.py run ...`) does, from a clean commit, into JSONL files kept
outside the repository, and this tool only reads those files.

    python benchmarks/ordering_discovery.py plan
    python benchmarks/ordering_discovery.py next --primary P.jsonl [--confirm C.jsonl]
    python benchmarks/ordering_discovery.py evaluate --primary P.jsonl [--confirm C.jsonl] [--out LEDGER]

Stages per candidate, in order, stopping at the first failed stage:

  S1  prefix B3 (`pb3`, 300 s hard timeout): complete, certified by `report._certified`
      (proven, `exhausted`/`bound_closed`, no limit, 0 unresolved, 0 bound violations),
      `b3_not_worse_than_b2`, wall time at most 150 s, and ordering criteria O1 and O2.
  S2  old B3 (`b3`, 120 s): complete, replay clean, wall time at most 90 s, and a total
      cost whose `repr` equals S1's.
  S3  old B2 (`b2`, 60 s): an ordering witness only; it never rejects a candidate.

Ordering criteria: O1 at least two distinct complete obstacle orders were found by the
real pipeline; O2 at least two legs of the final route contain a turning command (FL, FR,
BL, BR), so it is not four straight legs; O3 (tier 1) a strict cost difference between two
real-pipeline feasible routes: prefix B3's B2 seed route costs more than its B3 route, or
old B2 (input order first) costs more than old B3. Tier 2 passes S1 and S2 without O3.

Discovery stops at the first tier-1 candidate not rejected on confirmation, or after c6.
The selection is the first such tier-1 candidate, else the first tier-2 candidate that
confirms, else NO-GO. Confirmation reruns prefix B3 and old B3 once, separately: hash,
cost and work must be identical (else STOP: a determinism mismatch) and the time limits must
hold (else `rejected_on_confirmation`). A cost mismatch between two eligible results, a replay,
invariant or bound-audit violation, an error record and an unverified worker death are
hard stops: nothing is discarded and discovery halts.

The evaluator refuses input that could hide an attempt: a dirty tree, several commits, an
undeclared fixture, a duplicated (fixture, mode), a limit/checkpoint/profile/alternatives
setting, an unexpected timeout, or a later candidate run before an earlier one finished.
"""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

if __name__ == "__main__":      # run as a script: make `benchmarks` importable
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmarks import campaigns as cmp  # noqa: E402
from benchmarks import fixtures as bfx  # noqa: E402
from benchmarks import prefix_modes  # noqa: E402
from benchmarks import report as rep  # noqa: E402

LEDGER_SCHEMA_VERSION = 1
EVALUATOR_VERSION = 1
LEDGER_PATH = Path(__file__).resolve().parent / "results" / "ordering_fixture_discovery.json"
RUNNER = "benchmarks/bench_executable_planner.py"

CANDIDATE_IDS = tuple(bfx.ordering_fixture_id(k) for k in bfx.ORDERING_CANDIDATES)
EXISTING_FOUR_OBSTACLE_FIXTURES = ("A_spread_04", "B_clutter_04", "D_seeded_04_s1", "D_seeded_04_s2",
                                   "D_seeded_04_s3")
STAGE_MODES = (("S1", "pb3"), ("S2", "b3"), ("S3", "b2"))
MODES = ("pb3", "b3", "b2")
TIMEOUT_S = {"pb3": 300, "b3": 120, "b2": 60}
WALL_LIMIT_S = {"pb3": 150.0, "b3": 90.0}
MIN_UNIQUE_COMPLETE_ORDERS = 2
MIN_TURNING_LEGS = 2
TURNING_COMMANDS = frozenset({"FL", "FR", "BL", "BR"})
TOTAL_CAP_S = 3600
MAX_SITTINGS = 2
RULES = {
    "stages": {"S1": "pb3", "S2": "b3", "S3": "b2"},
    "timeout_s": dict(TIMEOUT_S), "wall_limit_s": dict(WALL_LIMIT_S),
    "alternatives": "off", "limits": "runner default, no operator limit", "profile": "none",
    "min_unique_complete_orders": MIN_UNIQUE_COMPLETE_ORDERS, "min_turning_legs": MIN_TURNING_LEGS,
    "turning_commands": sorted(TURNING_COMMANDS),
    "max_viewing_heading_spread_deg": bfx.ORDERING_MAX_HEADING_SPREAD_DEG,
    "hard_stops": ["cost_mismatch", "replay_or_invariant_violation", "bound_violation", "error_record",
                   "unverified_worker_death", "determinism_mismatch"],
    "total_cap_s": TOTAL_CAP_S, "max_sittings": MAX_SITTINGS,
}
_SESSION = cmp.Session("discovery", "D", TOTAL_CAP_S, ())


class DiscoveryInputError(ValueError):
    """The records cannot be evaluated honestly. `problems` lists why."""

    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__("; ".join(self.problems))


# ---------------------------------------------------------------- record helpers

def _get(record, *path):
    return rep._get(record, *path)


def _num(value) -> bool:
    return rep._usable_number(value)


def _zero(value) -> bool:
    return rep._zero(value)


def _entry(fixture_id: str, mode: str, record: dict) -> dict:
    """The report's own entry for a record, so certification uses ONE definition."""
    case = cmp.Case(f"{fixture_id}:{mode}", fixture_id, mode, "off", TIMEOUT_S[mode], "CT", "discovery")
    return rep._entry(case, f"{fixture_id}:{mode}", _SESSION, record)


def record_problems(record) -> list:
    """Why a runner record is not an acceptable discovery record (empty when it is)."""
    if not isinstance(record, dict):
        return ["record is not an object"]
    fixture_id, mode = record.get("fixture_id"), record.get("mode")
    label = f"{fixture_id}/{mode}"
    if fixture_id not in CANDIDATE_IDS:
        return [f"{label}: fixture is not a declared candidate"]
    if mode not in MODES:
        return [f"{label}: mode must be one of {MODES}"]
    problems = []
    if record.get("kind") != "benchmark_result":
        problems.append(f"{label}: not a benchmark_result record")
    if record.get("status") not in ("complete", "partial", "no_route", "timeout", "error"):
        problems.append(f"{label}: status {record.get('status')!r} is not an executed outcome")
    if record.get("alternatives") is not False:
        problems.append(f"{label}: alternatives must be off")
    if record.get("profile") != "none" or record.get("tracemalloc") is not False:
        problems.append(f"{label}: profile must be none and tracemalloc off")
    if record.get("timeout_s") != TIMEOUT_S[mode]:
        problems.append(f"{label}: timeout_s {record.get('timeout_s')!r} is not the declared {TIMEOUT_S[mode]}")
    limits = record.get("limits")
    if mode == "pb3":
        # `SearchLimits` ships a memory cap of its own (`max_frontier_entries`); "no limit" means
        # exactly the runner's record when no operator flag is given, never a stricter one.
        if limits != prefix_modes.limits_record({}):
            problems.append(f"{label}: a work limit, checkpoint or cancellation differs from the "
                            f"unset default")
    elif limits is not None:
        problems.append(f"{label}: an old-search record carries limits")
    if record.get("status") in ("complete", "partial", "no_route"):
        validation = record.get("validation")
        if not isinstance(validation, dict) or not isinstance(validation.get("normalized"), dict):
            problems.append(f"{label}: an executed record lacks its normalized validation")
    return problems


def hard_stops(record) -> list:
    """Stop conditions a record carries on its own."""
    stops = []
    if record.get("status") == "error":
        stops.append("error_record")
    validation = record.get("validation") if isinstance(record.get("validation"), dict) else {}
    for key in ("replay_violations", "b2_replay_violations", "invariant_violations"):
        if validation.get(key):
            stops.append("replay_or_invariant_violation")
            break
    raw = validation.get("raw") if isinstance(validation.get("raw"), dict) else {}
    if raw.get("bound_violation_count"):
        stops.append("bound_violation")
    if _get(record, "cleanup", "death_verified") is not True:
        stops.append("unverified_worker_death")
    return stops


def turning_legs(normalized) -> int:
    return sum(1 for leg in (normalized or {}).get("legs", [])
               if any(c.get("direction") in TURNING_COMMANDS for c in leg.get("commands", [])))


def _metrics(entry, record, normalized) -> dict:
    validation = record.get("validation") if isinstance(record.get("validation"), dict) else {}
    raw = validation.get("raw") if isinstance(validation.get("raw"), dict) else {}
    work = raw.get("work") if isinstance(raw.get("work"), dict) else None
    out = {
        "record_status": record.get("status"), "wall_time_s": record.get("wall_time_s"),
        "planning_time_s": record.get("planning_time_s"), "result_hash": record.get("result_hash"),
        "started_at_utc": record.get("started_at_utc"),
    }
    if normalized:
        out.update({
            "route_kind": normalized.get("route_kind"), "obstacle_order": normalized.get("obstacle_order"),
            "total_cost": None if normalized.get("total_cost") is None else repr(normalized["total_cost"]),
            "total_distance_cm": None if normalized.get("total_distance_cm") is None
            else repr(normalized["total_distance_cm"]),
            "legs": len(normalized.get("legs", [])), "turning_legs": turning_legs(normalized),
        })
    if record.get("mode") == "pb3":
        out.update({
            "stop_reason": raw.get("stop_reason"), "work_limit_hit": raw.get("work_limit_hit"),
            "optimality_proven": raw.get("optimality_proven"),
            "unresolved_branch_count": raw.get("unresolved_branch_count"),
            "bound_violation_count": raw.get("bound_violation_count"),
            "lower_bound_certified": raw.get("lower_bound_certified"),
            "b3_not_worse_than_b2": validation.get("b3_not_worse_than_b2"),
            "b2_seed_total_cost": None,
        })
        seed = validation.get("b2_normalized")
        if isinstance(seed, dict) and seed.get("route_kind") == "complete" and seed.get("total_cost") is not None:
            out["b2_seed_total_cost"] = repr(seed["total_cost"])
        if work:
            for key in ("leg_calls", "lattice_expansions", "nodes_expanded", "unique_complete_orders",
                        "complete_routes_found", "bound_prunes", "dominance_prunes", "incumbent_updates",
                        "leg_outcome_counts"):
                out[key] = work.get(key)
    else:
        for key in ("orders_considered", "valid_orders", "rejected_orders", "pruned_orders"):
            out[key] = raw.get(key)
    return out


def _normalized(record):
    validation = record.get("validation") if isinstance(record.get("validation"), dict) else {}
    return validation.get("normalized") if isinstance(validation.get("normalized"), dict) else None


def _violation_free(record) -> bool:
    validation = record.get("validation") if isinstance(record.get("validation"), dict) else {}
    return (validation.get("replay_violations") == [] and not validation.get("b2_replay_violations")
            and not validation.get("invariant_violations"))


def _cost(record):
    normalized = _normalized(record)
    return None if normalized is None else normalized.get("total_cost")


# ---------------------------------------------------------------- stages

def _stage(status, reason=None, checks=None, record=None, fixture_id=None, mode=None) -> dict:
    stage = {"status": status, "reason": reason, "checks": checks or {}}
    if record is not None:
        stage["metrics"] = _metrics(_entry(fixture_id, mode, record), record, _normalized(record))
    return stage


def _timed_out(record) -> bool:
    return record.get("status") == "timeout"


def evaluate_s1(fixture_id, record) -> tuple:
    """`(stage, criteria)` for the prefix B3 record."""
    criteria = {"O1": None, "O2": None, "O3": None}
    if _timed_out(record):
        return _stage("fail", "harness_timeout", record=record, fixture_id=fixture_id, mode="pb3"), criteria
    entry = _entry(fixture_id, "pb3", record)
    normalized = _normalized(record)
    wall = record.get("wall_time_s")
    certified = entry["outcome"] == "C" and rep._certified(entry, "b3")
    work = entry.get("work") or {}
    criteria["O1"] = isinstance(work.get("unique_complete_orders"), int) \
        and work["unique_complete_orders"] >= MIN_UNIQUE_COMPLETE_ORDERS
    criteria["O2"] = entry["outcome"] == "C" and turning_legs(normalized) >= MIN_TURNING_LEGS
    checks = {
        "complete": entry["outcome"] == "C", "certified": certified,
        "b3_not_worse_than_b2": entry.get("b3_not_worse_than_b2") is True,
        "wall_within_limit": _num(wall) and wall <= WALL_LIMIT_S["pb3"],
        "O1": criteria["O1"], "O2": criteria["O2"],
    }
    failed = next((name for name, ok in checks.items() if not ok), None)
    stage = _stage("fail" if failed else "pass", failed, checks, record, fixture_id, "pb3")
    return stage, criteria


def evaluate_s2(fixture_id, record, s1_record) -> dict:
    if _timed_out(record):
        return _stage("fail", "harness_timeout", record=record, fixture_id=fixture_id, mode="b3")
    entry = _entry(fixture_id, "b3", record)
    wall = record.get("wall_time_s")
    checks = {
        "complete": entry["outcome"] == "C", "replay_clean": _violation_free(record),
        "wall_within_limit": _num(wall) and wall <= WALL_LIMIT_S["b3"],
        "cost_bit_equal": entry["outcome"] == "C" and repr(_cost(record)) == repr(_cost(s1_record)),
    }
    failed = next((name for name, ok in checks.items() if not ok), None)
    return _stage("fail" if failed else "pass", failed, checks, record, fixture_id, "b3")


def evaluate_s3(fixture_id, record) -> dict:
    if _timed_out(record):
        return _stage("pass", "harness_timeout_witness_only", {"complete": False}, record, fixture_id, "b2")
    complete = _entry(fixture_id, "b2", record)["outcome"] == "C"
    return _stage("pass", None if complete else "not_complete_witness_only", {"complete": complete},
                  record, fixture_id, "b2")


def evaluate_o3(pb3, b3, b2) -> dict:
    """Whether two real-pipeline routes of different cost exist (strictly)."""
    seed = _get(pb3, "validation", "b2_normalized")
    seed_cost = seed.get("total_cost") if isinstance(seed, dict) and seed.get("route_kind") == "complete" else None
    final = _cost(pb3)
    via_seed = _num(seed_cost) and _num(final) and seed_cost > final
    old_b2 = _cost(b2) if b2 is not None and b2.get("status") == "complete" else None
    old_b3 = _cost(b3)
    via_old = _num(old_b2) and _num(old_b3) and old_b2 > old_b3
    return {"met": bool(via_seed or via_old), "b2_seed_cost_above_b3": bool(via_seed),
            "old_b2_cost_above_old_b3": bool(via_old)}


# ---------------------------------------------------------------- candidate state

def candidate_state(fixture_id, by_mode) -> dict:
    """Stages, criteria and tier of one candidate from its records; `pending` names the
    next mode to run (None once the candidate is resolved)."""
    state = {"stages": {"S1": _stage("not_attempted"), "S2": _stage("not_attempted"),
                        "S3": _stage("not_attempted")},
             "criteria": {"O1": None, "O2": None, "O3": None}, "tier": None, "resolution": "pending",
             "pending": "pb3", "stops": []}
    pb3, b3, b2 = (by_mode.get(m) for m in MODES)
    for record in (pb3, b3, b2):
        if record is not None:
            state["stops"] += hard_stops(record)
    if pb3 is None:
        return state
    state["stages"]["S1"], criteria = evaluate_s1(fixture_id, pb3)
    state["criteria"].update(criteria)
    if state["stages"]["S1"]["status"] == "fail":
        state.update(resolution="rejected_S1", pending=None)
        return state
    state["pending"] = "b3"
    if b3 is None:
        return state
    state["stages"]["S2"] = evaluate_s2(fixture_id, b3, pb3)
    checks = state["stages"]["S2"]["checks"]
    if checks.get("complete") and checks.get("replay_clean") and not checks.get("cost_bit_equal"):
        state["stops"].append("cost_mismatch")
    if state["stages"]["S2"]["status"] == "fail":
        state.update(resolution="rejected_S2", pending=None)
        return state
    state["pending"] = "b2"
    if b2 is None:
        return state
    state["stages"]["S3"] = evaluate_s3(fixture_id, b2)
    o3 = evaluate_o3(pb3, b3, b2)
    state["criteria"]["O3"] = o3["met"]
    state["criteria"]["O3_detail"] = o3
    state["tier"] = 1 if o3["met"] else 2
    state.update(resolution=f"tier{state['tier']}", pending=None)
    return state


# ---------------------------------------------------------------- confirmation

def _work(record):
    return _get(record, "validation", "raw", "work")


def _same_determinism(first, second, mode) -> list:
    """What differs between two runs of the same invocation (empty = identical)."""
    diffs = []
    if first.get("result_hash") != second.get("result_hash"):
        diffs.append("result_hash")
    if repr(_cost(first)) != repr(_cost(second)):
        diffs.append("total_cost")
    if mode == "pb3":
        if _work(first) != _work(second):
            diffs.append("work")
        for key in ("stop_reason", "optimality_proven", "unresolved_branch_count"):
            if _get(first, "validation", "raw", key) != _get(second, "validation", "raw", key):
                diffs.append(key)
    else:
        for key in ("orders_considered", "valid_orders", "rejected_orders", "pruned_orders"):
            if _get(first, "validation", "raw", key) != _get(second, "validation", "raw", key):
                diffs.append(key)
    return diffs


def confirmation_state(fixture_id, primary, confirm) -> dict:
    """`{status: pending|passed|failed|stop, pending, reason, details}` for one candidate."""
    result = {"status": "pending", "pending": "pb3", "reason": None, "stops": [], "runs": {}}
    for mode in ("pb3", "b3"):
        record = confirm.get(mode)
        if record is None:
            result["pending"] = mode
            return result
        result["stops"] += hard_stops(record)
        result["pending"] = None
        original = primary[mode]
        if _timed_out(record):
            result.update(status="failed", reason=f"{mode}_harness_timeout")
            result["runs"][mode] = {"record_status": "timeout", "wall_time_s": record.get("wall_time_s")}
            return result
        diffs = _same_determinism(original, record, mode)
        wall = record.get("wall_time_s")
        if mode == "pb3":
            entry = _entry(fixture_id, "pb3", record)
            eligible = entry["outcome"] == "C" and rep._certified(entry, "b3")
        else:
            eligible = _entry(fixture_id, "b3", record)["outcome"] == "C" and _violation_free(record)
        result["runs"][mode] = {
            "wall_time_s": wall, "planning_time_s": record.get("planning_time_s"),
            "result_hash": record.get("result_hash"), "total_cost": repr(_cost(record)),
            "record_status": record.get("status"), "eligible": eligible, "differences": diffs,
            **({"work": _work(record)} if mode == "pb3" else
               {k: _get(record, "validation", "raw", k)
                for k in ("orders_considered", "valid_orders", "rejected_orders", "pruned_orders")}),
        }
        if diffs:
            result["stops"].append("determinism_mismatch")
            result["reason"] = f"{mode}_differs:" + ",".join(diffs)
            return result
        if not eligible or not (_num(wall) and wall <= WALL_LIMIT_S[mode]):
            result.update(status="failed", reason=f"{mode}_ineligible_or_too_slow")
            return result
    result["status"] = "passed"
    return result


# ---------------------------------------------------------------- whole evaluation

def _group(records, where, problems) -> dict:
    grouped = {}
    for record in records:
        found = record_problems(record)
        if found:
            problems += [f"{where}: {p}" for p in found]
            continue
        key = (record["fixture_id"], record["mode"])
        if key in grouped:
            problems.append(f"{where}: {key[0]}/{key[1]} is recorded more than once")
        grouped[key] = record
    return grouped


def _fixture_info(fixture_id) -> dict:
    path = bfx.FIXTURE_DIR / f"{fixture_id}.json"
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    fixture = bfx.load_fixture(fixture_id)
    return {
        "fixture_id": fixture_id, "seed": (fixture.data.get("generator") or {}).get("seed"),
        "fixture_sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(),
        "obstacles": fixture.data["obstacles"],
        "faces": sorted({o.image_face for o in fixture.obstacles}),
        "viewing_heading_spread_deg": bfx.viewing_heading_spread_deg(fixture.start, fixture.obstacles),
    }


def evaluate(primary_records, confirm_records=(), inputs=(), voided=()) -> dict:
    """The ledger for the given runner records. Raises `DiscoveryInputError` when the
    records could hide an attempt. `status`: `incomplete` (more runs are needed, see
    `next_action`), `selected`, `no_go` or `stopped`. `voided` lists runs that were discarded
    before any result was judged (a tooling fault); they are disclosed, never evaluated."""
    primary_records, confirm_records = list(primary_records), list(confirm_records)
    problems = []
    every = primary_records + confirm_records
    if not every:
        problems.append("no records")
    problems += rep.provenance_problems(every)
    primary = _group(primary_records, "primary", problems)
    confirm = _group(confirm_records, "confirm", problems)
    if problems:
        raise DiscoveryInputError(problems)
    for (fixture_id, mode) in confirm:
        if mode == "b2":
            problems.append(f"confirm: {fixture_id}/b2 is not a confirmation mode")

    states, earlier_open = {}, None
    for fixture_id in CANDIDATE_IDS:
        by_mode = {m: primary.get((fixture_id, m)) for m in MODES}
        states[fixture_id] = candidate_state(fixture_id, by_mode)
        touched = any(record is not None for record in by_mode.values())
        if touched and earlier_open is not None:
            problems.append(f"primary: {fixture_id} has records while {earlier_open} is unfinished")
        if states[fixture_id]["pending"] is not None and earlier_open is None:
            earlier_open = fixture_id
    for fixture_id in CANDIDATE_IDS:      # a stage never runs before the one it depends on
        for later, earlier in (("b3", "pb3"), ("b2", "b3")):
            if (fixture_id, later) in primary and (fixture_id, earlier) not in primary:
                problems.append(f"primary: {fixture_id}/{later} is recorded without its {earlier} run")
    if problems:
        raise DiscoveryInputError(sorted(set(problems)))

    stops = [{"fixture_id": f, "reason": r} for f, s in states.items() for r in s["stops"]]
    confirmations, selected, action, status = {}, None, None, "incomplete"

    def consider(fixture_id):
        """Drive confirmation for one qualified candidate; return passed/failed/pending."""
        if fixture_id not in confirmations:
            by_mode = {m: confirm.get((fixture_id, m)) for m in ("pb3", "b3")}
            confirmations[fixture_id] = confirmation_state(fixture_id, {m: primary[(fixture_id, m)]
                                                                       for m in ("pb3", "b3")}, by_mode)
            stops.extend({"fixture_id": fixture_id, "reason": r} for r in confirmations[fixture_id]["stops"])
        return confirmations[fixture_id]

    if not stops:
        frontier = None
        for fixture_id in CANDIDATE_IDS:
            state = states[fixture_id]
            if state["pending"] is not None:
                frontier = ("run", fixture_id, state["pending"], "primary")
                break
            if state["tier"] == 1:
                conf = consider(fixture_id)
                if conf["status"] == "passed":
                    selected = fixture_id
                    break
                if conf["status"] == "pending":
                    frontier = ("run", fixture_id, conf["pending"], "confirm")
                    break
        if selected is None and frontier is None:          # all six resolved, no tier-1 confirmed
            for fixture_id in CANDIDATE_IDS:
                if states[fixture_id]["tier"] == 2:
                    conf = consider(fixture_id)
                    if conf["status"] == "passed":
                        selected = fixture_id
                        break
                    if conf["status"] == "pending":
                        frontier = ("run", fixture_id, conf["pending"], "confirm")
                        break
        if stops:
            status = "stopped"
        elif selected is not None:
            status = "selected"
        elif frontier is not None:
            action = frontier
        else:
            status = "no_go"
    else:
        status = "stopped"
    if stops:
        status = "stopped"

    for fixture_id in {f for f, _m in confirm}:     # a confirmation run is only ever due for one candidate
        due = confirmations.get(fixture_id)
        if due is None or ("b3" in {m for f, m in confirm if f == fixture_id} and "b3" not in due["runs"]):
            problems.append(f"confirm: {fixture_id} has a confirmation run that was not due")
    if problems:
        raise DiscoveryInputError(sorted(set(problems)))
    infos = {f: _fixture_info(f) for f in CANDIDATE_IDS + EXISTING_FOUR_OBSTACLE_FIXTURES}
    candidates = []
    for fixture_id in CANDIDATE_IDS:
        state, info = states[fixture_id], infos[fixture_id]
        conf = confirmations.get(fixture_id)
        if any(s["fixture_id"] == fixture_id for s in stops):
            outcome = "stopped"
        elif fixture_id == selected:
            outcome = "selected"
        elif conf is not None and conf["status"] == "failed":
            outcome = "rejected_on_confirmation"
        elif state["resolution"] in ("rejected_S1", "rejected_S2"):
            outcome = state["resolution"]
        elif state["tier"] is not None:
            outcome = "qualified_not_selected" if status in ("selected", "no_go") else "qualified_pending"
        elif state["pending"] is not None and any(state["stages"][s]["status"] != "not_attempted"
                                                  for s in ("S1", "S2", "S3")):
            outcome = "pending"
        else:
            outcome = "not_attempted"
        candidates.append({**info, "stages": state["stages"], "criteria": state["criteria"],
                           "tier": state["tier"], "confirmation": conf, "outcome": outcome})

    all_records = every
    shas = sorted({_get(r, "git", "sha") for r in all_records})
    started = sorted(r["started_at_utc"] for r in all_records if r.get("started_at_utc"))
    ledger = {
        "ledger_schema_version": LEDGER_SCHEMA_VERSION, "evaluator_version": EVALUATOR_VERSION,
        "policy": ("Generated by benchmarks/ordering_discovery.py from runner JSONL kept outside the "
                   "repository. Every declared candidate appears once, in order; a failed or unattempted "
                   "candidate is never removed. Nothing here is hand-written."),
        "status": status, "rules": RULES,
        "discovery_commit": shas[0] if len(shas) == 1 else None,
        "inputs": list(inputs), "voided_runs": [dict(v) for v in voided], "records_total": len(all_records),
        "first_record_utc": started[0] if started else None, "last_record_utc": started[-1] if started else None,
        "recorded_wall_time_s": sum(r["wall_time_s"] for r in all_records if _num(r.get("wall_time_s"))),
        "selected": None if selected is None else {
            "fixture_id": selected, "tier": states[selected]["tier"],
            "candidate_number": CANDIDATE_IDS.index(selected) + 1,
            "confirmation_status": confirmations[selected]["status"],
        },
        "stops": stops,
        "existing_four_obstacle_fixtures": [{
            "fixture_id": f, "faces": infos[f]["faces"],
            "viewing_heading_spread_deg": infos[f]["viewing_heading_spread_deg"],
            "eligible_under_heading_rule": infos[f]["viewing_heading_spread_deg"]
            <= bfx.ORDERING_MAX_HEADING_SPREAD_DEG + 1e-6,
            "run_in_discovery": False} for f in EXISTING_FOUR_OBSTACLE_FIXTURES],
        "candidates": candidates,
    }
    ledger["next_action"] = None if action is None else {
        "action": action[0], "fixture_id": action[1], "mode": action[2], "file": action[3],
        "command": run_command(action[1], action[2])}
    return ledger


def next_action(primary_records, confirm_records=()) -> dict:
    ledger = evaluate(primary_records, confirm_records)
    if ledger["status"] != "incomplete":
        return {"action": "done", "status": ledger["status"], "selected": ledger["selected"],
                "stops": ledger["stops"]}
    return ledger["next_action"]


def run_command(fixture_id, mode) -> list:
    return ["python", RUNNER, "run", "--fixture", fixture_id, "--mode", mode, "--alternatives", "off",
            "--timeout", str(TIMEOUT_S[mode])]


def first_run_plan() -> list:
    return [{"fixture_id": f, "stages": [{"stage": s, "mode": m, "command": run_command(f, m)}
                                         for s, m in STAGE_MODES]} for f in CANDIDATE_IDS]


def dumps_ledger(ledger) -> str:
    return json.dumps(ledger, indent=2) + "\n"


def load_jsonl(path) -> tuple:
    records, count = [], 0
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if line.strip():
            records.append(json.loads(line))
            count += 1
    return records, {"name": Path(path).name, "sha256": rep.sha256_file(path), "records": count}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("plan", help="print the declared candidates and their commands")
    for name in ("next", "evaluate"):
        sub = commands.add_parser(name)
        sub.add_argument("--primary", required=True, help="JSONL of the S1/S2/S3 runs")
        sub.add_argument("--confirm", default=None, help="JSONL of the confirmation runs")
    commands.choices["evaluate"].add_argument("--out", default=None, help="write the ledger JSON here")
    commands.choices["evaluate"].add_argument(
        "--voided", action="append", default=[], metavar="JSONL",
        help="a run discarded before its result was judged (a tooling fault); disclosed in the ledger")
    commands.choices["evaluate"].add_argument("--voided-reason", default=None,
                                              help="why the --voided runs were discarded (required with them)")
    args = parser.parse_args(argv)
    if args.command == "plan":
        print(json.dumps({"rules": RULES, "candidates": first_run_plan()}, indent=2))
        return 0
    try:
        primary, primary_input = load_jsonl(args.primary)
        confirm, confirm_input = load_jsonl(args.confirm) if args.confirm else ([], None)
        inputs = [primary_input] + ([confirm_input] if confirm_input else [])
        if args.command == "next":
            print(json.dumps(next_action(primary, confirm), indent=2))
            return 0
        voided = []
        if args.voided and not args.voided_reason:
            parser.error("--voided needs --voided-reason")
        for path in args.voided:
            _records, described = load_jsonl(path)
            voided.append({**described, "reason": args.voided_reason})
        ledger = evaluate(primary, confirm, inputs, voided)
    except DiscoveryInputError as error:
        for problem in error.problems:
            print(f"refused: {problem}", file=sys.stderr)
        return 6
    text = dumps_ledger(ledger)
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8", newline="")
    else:
        print(text, end="")
    return {"selected": 0, "incomplete": 0, "no_go": 0, "stopped": 4}[ledger["status"]]


if __name__ == "__main__":
    sys.exit(main())
