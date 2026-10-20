"""Evidence summary, coverage manifest status and adoption gates (measurement only).

`build_summary(records, inputs)` turns campaign JSONL records (kept OUTSIDE the
repository) into the compact, deterministic evidence summary that is tracked at
`benchmarks/results/prefix_validation_summary.json`. With no records it yields
the empty initial structure: every case `missing`, every coverage `not_started`,
every gate `not_measured`. Nothing measured is ever invented; every gate stays
`not_measured` until recorded evidence supports a verdict, and a `fail` is only
ever produced by counter-evidence in a record.

Gate verdicts are `pass`, `fail` or `not_measured`:

  G1 correctness   `fail` on any replay/metadata/bound-audit violation, error record,
                   unverified worker death or a temporary directory that was not
                   removed; `pass` only with zero of those AND the standard
                   minimum-evidence set complete. (`job_closed == false` is not a
                   failure: the fallback and POSIX paths use no Job Object.)
  G2 exactness     old `b3`/`b2` versus the prefix search where the old search completed
                   (R3/R2, R6/R5, R9/R8, R21/R20, R25/R24, R34/R33, R13/R12; a missing pair
                   keeps G2 not_measured)
                   (old timeouts are excluded, never counted as agreement). Complete
                   routes: B3 must be proven and bit-equal in cost. Partial routes:
                   `total_distance_cm` AND `total_cost` must agree bit-exactly
                   (`repr`), and missing or unusable partial accounting fails.
                   A comparison is only CERTIFIED when the prefix route is eligible for it:
                   for a B3 pair the route is complete, `optimality_proven` is true, the stop
                   is `exhausted`/`bound_closed`, no work limit was hit, and unresolved and
                   bound-violation counts are zero; for the B2 pair it is complete and not
                   budget-stopped (no proof is asked of B2). Anything else is
                   `not_certified`: never a pass, never a failure, and it keeps G2
                   not_measured. A prefix run stopped by a budget (`work_limit`,
                   `cancelled`, `wall_clock_checkpoint_limit`) is never certified, so a
                   faster old search cannot turn a work-limited run into a false failure. Two
                   cases stay failures: a complete, replay-valid prefix route CHEAPER than
                   the old route (counter-evidence to the old optimum), and a mismatch
                   that no budget stop explains (complete versus partial or none).
  G3 determinism   identical result hash and work counters across separate invocations.
  G4 coverage      the representative feasible spread fixture (unlimited pb2, alternatives
                   off) completes at each obstacle count from 4 through 8, and the
                   designated infeasible fixtures are proven impossible. It does not
                   claim that every possible 4-8 obstacle layout was validated.
  G5 timing       bounded executor (`bpb2`/`bpb3`, production policy): every 4-8 `A_spread`
                   representative invocation (BR1-BR10) delivers a complete route on time
                   (AVAIL), BR1/BR3/BR5 repeat (REPEAT: identical B2 seed hash; identical
                   final hash among `search_finished` repetitions; a differing final hash
                   after a budget or hard stop is REPORTED, not failed), and every production
                   bounded plan outcome (a route, no route or a failure) returns by R0 + 58 s
                   with no harness timeout. `not_measured` until every BR invocation is
                   recorded.
  G6 overshoot     every production bounded record decides by R0 + 45.5 s; `pass` needs at
                   least one production BR/BX record (no stall, no soak) whose RECORDED
                   `decided_s` exceeds its planning target (the termination label alone never
                   counts). Stress-policy records never count toward G5/G6; the planner
                   checkpoint alone is not the hard backstop.
  G7 memory        peak RSS <= 1 GiB on the 8-obstacle alternatives-on run, frontier cap
                   honest.
  G8 metadata      budget, cancellation, frontier and checkpoint stops each observed with
                   clean metadata.

Bounded records add checks without changing how any other record is judged
(`bounded_checks`): SAFE, RETAIN, CLEAN, NESTED (BQ), SOAK (BS5) and per-case
expectations feed G1's bad-record list; HONEST fails G8; MEM (host AND worker
peak RSS each <= 1 GiB) fails G7, and a worker peak that was not read after exit
keeps G7 from passing. Job committed memory is recorded as a diagnostic only.
A record without a `bounded` block (schema 2, or any non-bounded mode) is
evaluated exactly as before.

Provenance: every record given to `build_summary` must come from ONE clean commit.
It raises `ProvenanceError`, before any coverage or gate is evaluated, when a record
has a missing or malformed `git.sha` (not 40 lowercase hex digits), a `git.dirty`
that is not exactly `False`, when the records carry more than one SHA, or when an
invocation appears twice. Distinct invocations of one case (`Q5#1`, `Q5#2`) are not
duplicates. No records is valid and gives the empty initial structure.

Future operational timing (NOT implemented here): the provisional 30 s planner
checkpoint applies to the entire `optimise_prefix_b3_route` call, including its
B2 seed phase; if a complete B2 incumbent exists, a budget stop returns the
best complete incumbent. The production hard backstop is a separate 45 s limit.
"""

import hashlib
import json
import math
import re
from pathlib import Path

from benchmarks import campaigns as cmp

SUMMARY_SCHEMA_VERSION = 2          # 2: bounded entries, gates G5/G6 and the `bounded` section
SUMMARY_PATH = Path(__file__).resolve().parent / "results" / "prefix_validation_summary.json"
GATE_TITLES = {
    "G1": "correctness", "G2": "exactness", "G3": "determinism", "G4": "coverage",
    "G5": "timing", "G6": "overshoot", "G7": "memory", "G8": "metadata",
}
EXACT_PAIRS = (   # (old case, prefix case, kind)
    ("R3", "R2", "b3"), ("R6", "R5", "b3"), ("R9", "R8", "b3"), ("R21", "R20", "b3"),
    ("R25", "R24", "b3"), ("R34", "R33", "b2"),
    ("R13", "R12", "b3"),   # four obstacles: counts only if the old search completes
)
DETERMINISM_GATE_PAIRS = (("R8", "R10"), ("R14", "X1"), ("R16", "R17"))
# Representative feasible spread fixture per obstacle count (unlimited pb2, alternatives off).
G4_REPRESENTATIVE = {4: "R11", 5: "X12", 6: "R14", 7: "X18", 8: "R16"}
G4_COMPLETE = tuple(G4_REPRESENTATIVE.values())
G4_IMPOSSIBLE = ("R22", "R23")
MEMORY_LIMIT_BYTES = 1024 ** 3
BOUNDED_MODES = ("bpb2", "bpb3")
BOUNDED_AVAILABILITY = tuple(f"BR{i}" for i in range(1, 11))
BOUNDED_REPEAT = ("BR1", "BR3", "BR5")
BOUNDED_NESTED = ("BQ1", "BQ2")
BOUNDED_SOAK = ("BS5",)
DECISION_MARGIN_S = 0.5                  # hard-stop decision allowance over the policy's hard stop
SOAK_HANDLE_GROWTH = 20                  # row 8 handle count may exceed row 1 by at most this
BOUNDED_STOPS = frozenset({"planner_budget", "hard_backstop", "cancelled", "planner_work_limit",
                           "startup_overran_budget"})
# Case-specific expectations (all feed G1). Keys: status, termination, route_status,
# incumbents (exact), min_incumbents, deliverable, route_present, rss_after_exit, row_termination.
BOUNDED_EXPECT = {
    "BQ1": {"status": ("complete",), "termination": ("search_finished",), "rss_after_exit": True},
    "BQ2": {"termination": ("hard_backstop",), "route_status": ("none",), "incumbents": 0,
            "rss_after_exit": True},
    "BX6": {"status": ("no_route",)},
    "BX16": {"status": ("no_route",)},
    "BS1": {"termination": ("hard_backstop",), "min_incumbents": 1, "deliverable": True,
            "route_present": True},
    "BS2": {"termination": ("search_finished", "planner_budget", "hard_backstop")},
    "BS5": {"row_termination": ("hard_backstop",)},
}
# Stop reasons of `route_prefix_search` (a test pins these against its constants).
BUDGET_STOPS = frozenset({"work_limit", "cancelled", "wall_clock_checkpoint_limit"})
CLOSED_STOPS = frozenset({"exhausted", "bound_closed"})
SHA_PATTERN = re.compile(r"[0-9a-f]{40}")
MAX_PROVENANCE_PROBLEMS = 10


class ProvenanceError(ValueError):
    """The records are not evidence from one exact clean commit. `problems` lists why."""

    def __init__(self, problems):
        self.problems = list(problems)
        shown = self.problems[:MAX_PROVENANCE_PROBLEMS]
        extra = len(self.problems) - len(shown)
        super().__init__("; ".join(shown) + (f"; and {extra} more" if extra > 0 else ""))


def is_commit_sha(value) -> bool:
    return isinstance(value, str) and SHA_PATTERN.fullmatch(value) is not None


def provenance_problems(records, expected_sha=None) -> list:
    """Why `records` are not evidence from ONE exact clean commit (empty when they are).
    Each record needs a 40-hex `git.sha` and `git.dirty` exactly `False`; all records
    share one SHA (`expected_sha` when given); no invocation appears twice."""
    problems, shas, invocations = [], {}, {}
    for index, record in enumerate(records, 1):
        invocation = _get(record, "case", "invocation")
        label = f"record {index}" + (f" ({invocation})" if isinstance(invocation, str) else "")
        git = record.get("git") if isinstance(record, dict) else None
        sha = git.get("sha") if isinstance(git, dict) else None
        if not is_commit_sha(sha):
            problems.append(f"{label}: git.sha is missing or not 40 lowercase hex digits")
        else:
            shas.setdefault(sha, []).append(label)
        dirty = git.get("dirty") if isinstance(git, dict) else None
        if dirty is not False:
            problems.append(f"{label}: git.dirty is {dirty!r}, not false")
        if isinstance(invocation, str) and invocation:
            if invocation in invocations:
                problems.append(f"{label}: invocation {invocation} is recorded more than once")
            invocations.setdefault(invocation, index)
    if expected_sha is not None:
        for sha in sorted(shas):
            if sha != expected_sha:
                problems.append(f"{len(shas[sha])} record(s) are from commit {sha[:12]}, "
                                f"not the current commit {expected_sha[:12]}")
    elif len(shas) > 1:
        problems.append("records come from several commits: " + ", ".join(sorted(s[:12] for s in shas)))
    return problems


def check_provenance(records) -> None:
    """Raise `ProvenanceError` unless every record is from one exact clean commit."""
    problems = provenance_problems(records)
    if problems:
        raise ProvenanceError(problems)


def sha256_file(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_records(paths) -> tuple:
    """`(records, inputs)` from JSONL files. `inputs` lists each file's basename,
    SHA-256 of its raw bytes and record count."""
    records, inputs = [], []
    for path in paths:
        path = Path(path)
        count = 0
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                records.append(json.loads(line))
                count += 1
        inputs.append({"name": path.name, "sha256": sha256_file(path), "records": count})
    return records, inputs


def _get(record, *path):
    for key in path:
        if not isinstance(record, dict):
            return None
        record = record.get(key)
    return record


def _index(records) -> tuple:
    """`(by_invocation, unassigned)`; the last record for an invocation wins."""
    by_invocation, unassigned = {}, 0
    for record in records:
        case = record.get("case")
        if isinstance(case, dict) and case.get("invocation"):
            by_invocation[case["invocation"]] = record
        else:
            unassigned += 1
    return by_invocation, unassigned


def _entry(case, invocation, session, record) -> dict:
    entry = {
        "case_id": case.case_id, "invocation": invocation, "campaign": session.campaign,
        "session": session.name, "fixture": case.fixture, "mode": case.mode,
        "alternatives": case.alternatives, "timeout_s": case.timeout_s, "accept": case.accept,
        "item_status": cmp.item_status(case, record),
    }
    if record is None:
        return entry
    raw = _get(record, "validation", "raw") or {}
    norm = _get(record, "validation", "normalized") or {}
    work = raw.get("work") if isinstance(raw, dict) else None
    violations = None
    if isinstance(record.get("validation"), dict):
        v = record["validation"]
        violations = {
            "replay": len(v.get("replay_violations") or []),
            "b2_replay": len(v.get("b2_replay_violations") or []),
            "invariant": len(v.get("invariant_violations") or []),
        }
    entry.update({
        "record_status": record.get("status"), "outcome": cmp.outcome_letter(record),
        "timeout_kind": _get(record, "cleanup", "timeout_kind"),
        "result_hash": record.get("result_hash"), "route_kind": norm.get("route_kind"),
        "total_cost": norm.get("total_cost"), "total_distance_cm": norm.get("total_distance_cm"),
        "n_commands": sum(len(leg.get("commands", [])) for leg in norm.get("legs", [])) if norm else None,
        "stop_reason": raw.get("stop_reason"), "work_limit_hit": raw.get("work_limit_hit"),
        "optimality_proven": raw.get("optimality_proven"),
        "complete_route_proven_impossible": raw.get("complete_route_proven_impossible"),
        "unresolved_branch_count": raw.get("unresolved_branch_count"),
        "bound_violation_count": raw.get("bound_violation_count"),
        "planning_time_s": record.get("planning_time_s"), "wall_time_s": record.get("wall_time_s"),
        "replay_s": _get(record, "validation", "timing", "replay_s"),
        "normalize_s": _get(record, "validation", "timing", "normalize_s"),
        "peak_rss_bytes": _get(record, "peak_memory", "peak_rss_bytes"),
        "work": work, "violations": violations,
        "b3_not_worse_than_b2": _get(record, "validation", "b3_not_worse_than_b2"),
        "cleanup_death_verified": _get(record, "cleanup", "death_verified"),
        "cleanup_tempdir_removed": _get(record, "cleanup", "tempdir_removed"),
    })
    if case.mode in BOUNDED_MODES:
        entry["bounded"] = _bounded_entry(case, record)
    return entry


# --------------------------------------------------------------------------
# Bounded-executor records (`bpb2`/`bpb3`, record schema 3)
# --------------------------------------------------------------------------

def _views(bounded) -> list:
    if not isinstance(bounded, dict):
        return []
    soak = bounded.get("soak")
    if isinstance(soak, dict):
        return [r for r in soak.get("rows") or [] if isinstance(r, dict)]
    plan = bounded.get("plan")
    return [plan] if isinstance(plan, dict) else []


def _policy(bounded) -> dict:
    policy = bounded.get("policy") if isinstance(bounded, dict) else None
    return policy if isinstance(policy, dict) else {}


def _view_checks(view, policy, tag) -> dict:
    """SAFE, RETAIN, CLEAN, HONEST, DECIDE and RETURN problems of one plan."""
    safe, retain, clean, honest, decide, ret = [], [], [], [], [], []
    routes = view.get("routes") or {}
    for key in ("route", "b2_route", "diagnostic_route"):
        entry = routes.get(key)
        if entry is None:
            continue
        if not isinstance(entry, dict) or not isinstance(entry.get("replay_violations"), list):
            safe.append(f"{tag}{key}: not replayed")
        elif entry["replay_violations"]:
            safe.append(f"{tag}{key}: {len(entry['replay_violations'])} replay violation(s)")
    if view.get("verification_failures"):
        safe.append(f"{tag}verification failures: {len(view['verification_failures'])}")
    # RETAIN
    if view.get("b3_not_worse_than_b2") is False:
        retain.append(f"{tag}retained route costs more than its B2 seed")
    if (view.get("incumbents_received") or 0) >= 1 and routes.get("route") is None \
            and routes.get("diagnostic_route") is None:
        retain.append(f"{tag}incumbents were received but no route was retained")
    # CLEAN (plan level)
    cleanup = view.get("executor_cleanup") or {}
    tree = view.get("tree") or {}
    if cleanup.get("worker_started"):
        for name in ("death_verified", "tree_dead", "reader_stopped", "tempdir_removed", "job_released"):
            if cleanup.get(name) is not True:
                clean.append(f"{tag}executor {name} is {cleanup.get(name)!r}")
        if cleanup.get("terminate_error"):
            clean.append(f"{tag}executor terminate_error")
        if tree.get("workers_started") != 1:
            clean.append(f"{tag}{tree.get('workers_started')!r} workers captured, expected 1")
        if tree.get("worker_exit_verified") is not True:
            clean.append(f"{tag}worker exit not verified through its handle")
    elif cleanup.get("tempdir_removed") is False:
        clean.append(f"{tag}executor tempdir not removed")
    if tree.get("job_active_processes_after") != 1:
        clean.append(f"{tag}job active processes after the plan: {tree.get('job_active_processes_after')!r}")
    if tree.get("private_tempdir_leftovers") != 0:
        clean.append(f"{tag}temporary directories left: {tree.get('private_tempdir_leftovers')!r}")
    # HONEST
    honest += _honest_problems(view, policy, tag)
    # DECIDE / RETURN against the record's own policy, for every plan outcome
    timing = view.get("timing") or {}
    decided, returned = timing.get("decided_s"), timing.get("returned_s")
    if policy.get("hard_stop_s") is not None:
        limit = policy["hard_stop_s"] + DECISION_MARGIN_S
        if not _usable_number(decided) or decided > limit:
            decide.append(f"{tag}decided_s {decided!r} > {limit:g}")
    if policy.get("return_deadline_s") is not None:
        if not _usable_number(returned) or returned > policy["return_deadline_s"]:
            ret.append(f"{tag}returned_s {returned!r} > {policy['return_deadline_s']:g}")
    return {"safe": safe, "retain": retain, "clean": clean, "honest": honest, "decide": decide, "return": ret}


def _route_cost(view, key="route"):
    entry = (view.get("routes") or {}).get(key)
    normalized = entry.get("normalized") if isinstance(entry, dict) else None
    cost = normalized.get("total_cost") if isinstance(normalized, dict) else None
    return cost if _usable_number(cost) else None


def _honest_problems(view, policy, tag) -> list:
    """The executor's own proof rules (`bounded_prefix_planning._honest_flags` and
    `_assemble`), re-checked from the recorded outcome, planner metadata and route."""
    problems = []
    termination = view.get("termination")
    flags = [f for f in ("optimality_proven", "complete_route_proven_impossible", "search_space_exhausted")
             if view.get(f)]
    message = view.get("execution_message") or ""
    metadata = view.get("planner_metadata") or {}
    if termination != "search_finished":
        if flags:
            problems.append(f"{tag}{', '.join(flags)} claimed after a {termination} stop")
        if "Optimality proven." in message or "No complete route exists" in message:
            problems.append(f"{tag}message claims a proof after a {termination} stop")
        routes = view.get("routes") or {}
        # The wording rule covers an ordinary bounded no-route reply only. A failed cleanup and a
        # verified route withheld for returning late have their own executor wording; CLEAN/G1 and
        # RETURN/G5 own those two failures.
        if (termination in BOUNDED_STOPS and view.get("route_status") == "none"
                and _cleanup_ok(view.get("executor_cleanup") or {}) and routes.get("diagnostic_route") is None
                and "within budget" not in message):
            problems.append(f"{tag}no-route message does not say 'within budget'")
        return problems
    if flags and not _cleanup_ok(view.get("executor_cleanup") or {}):
        problems.append(f"{tag}{', '.join(flags)} claimed although cleanup was not verified")
    closed = metadata.get("stop_reason") in CLOSED_STOPS
    unresolved = metadata.get("unresolved_branch_count")
    if view.get("search_space_exhausted") and not (closed and _zero(unresolved)):
        problems.append(f"{tag}search_space_exhausted without a closed search with zero unresolved branches")
    if view.get("optimality_proven"):
        cost = _route_cost(view)
        bound = metadata.get("min_unresolved_lower_bound")
        covered = _zero(unresolved) or (
            metadata.get("unresolved_bound_known") is True and _usable_number(bound) and cost is not None
            and bound >= cost)
        if not (metadata.get("optimality_proven") is True and policy.get("planner_mode", "B3") == "B3"
                and view.get("deliverable") is True and view.get("route_status") == "complete"
                and view.get("route_source") == "final" and cost is not None and closed
                and metadata.get("lower_bound_certified") is True
                and _zero(metadata.get("bound_violation_count")) and covered):
            problems.append(f"{tag}optimality_proven without its conditions")
    if view.get("complete_route_proven_impossible"):
        routes = view.get("routes") or {}
        if not (metadata.get("complete_route_proven_impossible") is True
                and routes.get("route") is None and routes.get("diagnostic_route") is None
                and (metadata.get("empty_candidate_obstacle_ids")
                     or (metadata.get("stop_reason") == "exhausted" and _zero(unresolved)))):
            problems.append(f"{tag}complete_route_proven_impossible without its conditions")
    return problems


def _cleanup_ok(cleanup) -> bool:
    """`bounded_prefix_planning.Cleanup.ok`: nothing unverified, no terminate error."""
    return not cleanup.get("terminate_error") and all(
        cleanup.get(k) is not False for k in ("death_verified", "tree_dead", "job_released", "tempdir_removed",
                                               "reader_stopped"))


def bounded_checks(record, case_id=None) -> dict:
    """Every bounded check for one record, as lists of problems (empty = holds),
    plus `memory` (`ok`/`fail`/`not_measured`). See the module docstring."""
    bounded = record.get("bounded") if isinstance(record, dict) else None
    out = {k: [] for k in ("safe", "retain", "clean", "honest", "decide", "return", "avail", "nested",
                           "soak", "expect", "decide_reported")}
    out["memory"], out["memory_problems"] = "ok", []
    if not isinstance(bounded, dict):
        out["safe"].append("no bounded block")
        return out
    policy = dict(_policy(bounded), planner_mode=bounded.get("planner_mode", "B3"))
    status = record.get("status")
    if status in ("error", "timeout", "refused") or status is None:
        out["safe"].append(f"status {status!r}")
    views = _views(bounded)
    if not views and status not in ("timeout", "refused"):
        out["safe"].append("no plan recorded")
    soak = bounded.get("soak") if isinstance(bounded.get("soak"), dict) else None
    for index, view in enumerate(views, 1):
        tag = f"plan {index}: " if soak else ""
        for key, problems in _view_checks(view, policy, tag).items():
            out[key] += problems
    if soak is not None and policy.get("name") != "production":
        # A stress-policy soak (BS5) is a leak/health test: its per-row decision times are
        # reported, never a G1 failure. Every other soak requirement stays in force.
        out["decide_reported"] = out["decide"]
        out["decide"] = []
    # CLEAN (record level: the harness worker, its job and the sentinel)
    cleanup = record.get("cleanup") or {}
    harness = bounded.get("harness") or {}
    if cleanup.get("death_verified") is not True:
        out["clean"].append(f"harness worker death_verified {cleanup.get('death_verified')!r}")
    if cleanup.get("tempdir_removed") is not True:
        out["clean"].append(f"harness tempdir_removed {cleanup.get('tempdir_removed')!r}")
    if harness.get("job_active_processes_before_close") != 0:
        out["clean"].append(f"harness job active processes before close: "
                            f"{harness.get('job_active_processes_before_close')!r}")
    if harness.get("sentinel_alive_after") is not True:
        out["clean"].append("the unrelated sentinel did not survive")
    if harness.get("sentinel_termination_verified") is not True:
        out["clean"].append("the sentinel's termination was not verified")
    # MEM: host and worker peak RSS, each <= 1 GiB; committed memory is diagnostic only
    host = (record.get("peak_memory") or {}).get("peak_rss_bytes")
    readings = [("host", host, "after_exit")] + [
        (f"worker{' plan ' + str(i) if soak else ''}", (v.get("memory") or {}).get("worker_peak_rss_bytes"),
         (v.get("memory") or {}).get("worker_peak_rss_source")) for i, v in enumerate(views, 1)]
    for label, value, source in readings:
        if isinstance(value, (int, float)) and value > MEMORY_LIMIT_BYTES:
            out["memory"] = "fail"
            out["memory_problems"].append(f"{label} peak RSS {value} > {MEMORY_LIMIT_BYTES}")
        elif value is None or source != "after_exit":
            if out["memory"] == "ok":
                out["memory"] = "not_measured"
            out["memory_problems"].append(f"{label} peak RSS not read after exit ({source!r})")
    if not views and out["memory"] == "ok":
        out["memory"] = "not_measured"
    # AVAIL (G5 representatives)
    case = case_id or ((record.get("case") or {}).get("case_id"))
    if case in BOUNDED_AVAILABILITY:
        view = views[0] if views else {}
        if not (status == "complete" and view.get("deliverable") is True
                and view.get("route_status") == "complete" and view.get("delivery") == "on_time"):
            out["avail"].append(f"no complete deliverable on-time route (status {status!r}, "
                                f"termination {view.get('termination')!r})")
    # NESTED (BQ)
    if case in BOUNDED_NESTED:
        for index, view in enumerate(views, 1):
            cleanup_v, tree = view.get("executor_cleanup") or {}, view.get("tree") or {}
            if cleanup_v.get("containment") != "job_object":
                out["nested"].append(f"executor containment {cleanup_v.get('containment')!r}")
            if cleanup_v.get("job_released") is not True:
                out["nested"].append("executor job not released")
            if not isinstance(tree.get("job_active_processes_max_during"), int) \
                    or tree["job_active_processes_max_during"] < 2:
                out["nested"].append(f"nested worker never counted in the harness job "
                                     f"(max {tree.get('job_active_processes_max_during')!r})")
            if tree.get("job_active_processes_after") != 1:
                out["nested"].append("harness job not back to 1 process after the plan")
        if harness.get("job_active_processes_before_close") != 0:
            out["nested"].append("harness job not empty before close")
        if not views:
            out["nested"].append("no plan recorded")
    # SOAK (BS5)
    if case in BOUNDED_SOAK:
        if soak is None or len(views) != (bounded.get("soak_plans") or 0):
            out["soak"].append("soak rows missing")
        else:
            before = soak.get("thread_count_before")
            for index, view in enumerate(views, 1):
                if view.get("healthy_after") is not True:
                    out["soak"].append(f"plan {index}: executor unhealthy")
                if view.get("thread_count") != before:
                    out["soak"].append(f"plan {index}: {view.get('thread_count')!r} threads, before {before!r}")
            first, last = views[0].get("handle_count"), views[-1].get("handle_count")
            if not isinstance(first, int) or not isinstance(last, int) or last - first > SOAK_HANDLE_GROWTH:
                out["soak"].append(f"handle count {first!r} -> {last!r} (limit +{SOAK_HANDLE_GROWTH})")
    # Case expectations
    expect = BOUNDED_EXPECT.get(case, {})
    view = views[0] if views and not soak else {}
    if "status" in expect and status not in expect["status"]:
        out["expect"].append(f"status {status!r}, expected {expect['status']}")
    if "termination" in expect and view.get("termination") not in expect["termination"]:
        out["expect"].append(f"termination {view.get('termination')!r}, expected {expect['termination']}")
    if "route_status" in expect and view.get("route_status") not in expect["route_status"]:
        out["expect"].append(f"route_status {view.get('route_status')!r}")
    if "incumbents" in expect and view.get("incumbents_received") != expect["incumbents"]:
        out["expect"].append(f"incumbents_received {view.get('incumbents_received')!r}")
    if "min_incumbents" in expect and (view.get("incumbents_received") or 0) < expect["min_incumbents"]:
        out["expect"].append(f"incumbents_received {view.get('incumbents_received')!r}: retention not exercised")
    if expect.get("deliverable") and view.get("deliverable") is not True:
        out["expect"].append("route not deliverable")
    if expect.get("route_present") and (view.get("routes") or {}).get("route") is None:
        out["expect"].append("no delivered route")
    if expect.get("rss_after_exit") and any(
            (v.get("memory") or {}).get("worker_peak_rss_source") != "after_exit" for v in views):
        out["expect"].append("worker peak RSS was not read after exit")
    if "row_termination" in expect:
        for index, row in enumerate(views, 1):
            if row.get("termination") not in expect["row_termination"]:
                out["expect"].append(f"plan {index}: termination {row.get('termination')!r}")
    return out


def _g1_problems(checks, policy_name) -> list:
    problems = checks["safe"] + checks["retain"] + checks["clean"] + checks["nested"] + checks["soak"] \
        + checks["expect"]
    if policy_name != "production":
        problems = problems + checks["decide"]      # a stress stop must still be decided on time
    return problems


def bounded_problems(record) -> list:
    """Every failing bounded check of a record (runner exit code 4 when non-empty)."""
    bounded = record.get("bounded") if isinstance(record, dict) else None
    checks = bounded_checks(record)
    name = _policy(bounded).get("name")
    problems = _g1_problems(checks, name) + checks["honest"] + checks["avail"]
    if name == "production":
        problems += checks["decide"] + checks["return"]
    if checks["memory"] == "fail":
        problems += checks["memory_problems"]
    return problems


def _bounded_entry(case, record) -> dict:
    bounded = record.get("bounded") or {}
    checks = bounded_checks(record, case.case_id)
    views = _views(bounded)
    first = views[0] if views and not isinstance(bounded.get("soak"), dict) else {}
    routes = first.get("routes") or {}
    policy_name = _policy(bounded).get("name")
    harness = bounded.get("harness") or {}
    g1 = _g1_problems(checks, policy_name)
    gating = g1 + checks["honest"] + checks["avail"] + (
        checks["decide"] + checks["return"] if policy_name == "production" else [])
    verdict = "fail" if gating or checks["memory"] == "fail" else (
        "not_measured" if checks["memory"] == "not_measured" else "pass")
    return {
        "policy": policy_name, "stall": bounded.get("stall"), "soak_plans": bounded.get("soak_plans"),
        "planning_target_s": _policy(bounded).get("planning_target_s"),
        "termination": first.get("termination"), "route_status": first.get("route_status"),
        "deliverable": first.get("deliverable"), "delivery": first.get("delivery"),
        "incumbents_received": first.get("incumbents_received"),
        "decided_s": (first.get("timing") or {}).get("decided_s"),
        "returned_s": (first.get("timing") or {}).get("returned_s"),
        "overshoot_s": first.get("overshoot_s"),
        "route_hash": (routes.get("route") or {}).get("hash"),
        "b2_route_hash": (routes.get("b2_route") or {}).get("hash"),
        "host_peak_rss_bytes": (record.get("peak_memory") or {}).get("peak_rss_bytes"),
        "worker_peak_rss_bytes": [(v.get("memory") or {}).get("worker_peak_rss_bytes") for v in views],
        "worker_peak_rss_source": [(v.get("memory") or {}).get("worker_peak_rss_source") for v in views],
        "diagnostic_job_peak_process_commit_bytes": harness.get("job_peak_process_commit_bytes"),
        "diagnostic_job_peak_commit_bytes": harness.get("job_peak_commit_bytes"),
        "row_terminations": [v.get("termination") for v in views] if bounded.get("soak") else None,
        "checks": checks, "g1_problems": g1, "verdict": verdict,
    }


def _repeat_state(entries, case_id) -> dict:
    """REPEAT for one case: identical B2 seed hash in every repetition, identical
    final hash among `search_finished` repetitions; other differences reported."""
    items = [e for e in entries.values() if e["case_id"] == case_id]
    if not items or any(e["item_status"] == "missing" or "bounded" not in e for e in items):
        return {"state": "missing"}
    seeds = [e["bounded"]["b2_route_hash"] for e in items]
    finished = {e["bounded"]["route_hash"] for e in items if e["bounded"]["termination"] == "search_finished"}
    others = [e["invocation"] for e in items if e["bounded"]["termination"] != "search_finished"]
    finals = {e["bounded"]["route_hash"] for e in items}
    state = {"b2_seed_hashes": seeds, "finished_route_hashes": sorted(h for h in finished if h),
             "wall_clock_variation": len(finals) > 1 and bool(others), "budget_or_hard_stopped": others}
    if any(h is None for h in seeds) or len(set(seeds)) != 1:
        return {"state": "fail", "reason": "B2 seed hash missing or not identical", **state}
    if len(finished) > 1:
        return {"state": "fail", "reason": "search_finished repetitions differ", **state}
    return {"state": "ok", **state}


def _bounded_present(entries):
    return {i: e for i, e in entries.items() if "bounded" in e and e["item_status"] != "missing"}


def _gate_g5(entries, set_gate) -> None:
    present = _bounded_present(entries)
    production = {i: e for i, e in present.items() if e["bounded"]["policy"] == "production"}
    representatives = {i: e for i, e in entries.items() if e["case_id"] in BOUNDED_AVAILABILITY}
    fails = [i for i, e in representatives.items() if i in present and e["bounded"]["checks"]["avail"]]
    fails += [i for i, e in production.items() if e["bounded"]["checks"]["return"] or e["outcome"] == "T"]
    repeats = {c: _repeat_state(entries, c) for c in BOUNDED_REPEAT}
    fails += [c for c, r in repeats.items() if r["state"] == "fail"]
    evidence = sorted(set(representatives) | set(production))
    if fails:
        set_gate("G5", "fail", "bounded availability, repeat or return-by-deadline failed: "
                 + ", ".join(sorted(set(fails))), evidence)
    elif all(i in present for i in representatives) and all(r["state"] == "ok" for r in repeats.values()):
        set_gate("G5", "pass", "every 4-8 representative delivered a complete route on time under the "
                 "production policy and repeated", evidence)
    elif present:
        missing = sorted(i for i in representatives if i not in present)
        set_gate("G5", "not_measured", "bounded representatives not yet recorded: " + ", ".join(missing), evidence)
    else:
        set_gate("G5", "not_measured", "no bounded-executor (hard backstop) evidence recorded")


def _natural_overshoot(entry) -> bool:
    """A production BR/BX plan (no stall, no soak) whose RECORDED decision came after
    its planning target: natural evidence that the target was overrun and bounded.
    The termination label alone never counts."""
    b = entry["bounded"]
    return (entry["case_id"].startswith(("BR", "BX")) and b["policy"] == "production"
            and b["stall"] is None and b["soak_plans"] is None
            and _usable_number(b["decided_s"]) and _usable_number(b["planning_target_s"])
            and b["decided_s"] > b["planning_target_s"])


def _gate_g6(entries, set_gate) -> None:
    present = _bounded_present(entries)
    production = {i: e for i, e in present.items() if e["bounded"]["policy"] == "production"}
    fails = [i for i, e in production.items() if e["bounded"]["checks"]["decide"]]
    overran = [i for i, e in production.items() if _natural_overshoot(e)]
    if fails:
        set_gate("G6", "fail", f"hard-stop decision later than hard stop + {DECISION_MARGIN_S:g} s: "
                 + ", ".join(sorted(fails)), sorted(production))
    elif overran:
        set_gate("G6", "pass", "every production decision within the hard stop; the 30 s target was "
                 "overrun and bounded", sorted(production))
    elif production:
        set_gate("G6", "not_measured", "no production BR/BX run went past the planning target yet",
                 sorted(production))
    else:
        set_gate("G6", "not_measured", "no bounded-executor (hard backstop) evidence recorded")


def _bounded_summary(entries) -> dict:
    present = _bounded_present(entries)
    return {
        "repeat": {c: _repeat_state(entries, c) for c in BOUNDED_REPEAT},
        "overshoot": [{"invocation": i, "termination": e["bounded"]["termination"],
                       "decided_s": e["bounded"]["decided_s"], "overshoot_s": e["bounded"]["overshoot_s"]}
                      for i, e in sorted(present.items()) if e["bounded"]["policy"] == "production"],
        "memory": [{"invocation": i, "host_peak_rss_bytes": e["bounded"]["host_peak_rss_bytes"],
                    "worker_peak_rss_bytes": e["bounded"]["worker_peak_rss_bytes"],
                    "worker_peak_rss_source": e["bounded"]["worker_peak_rss_source"],
                    "diagnostic_job_peak_process_commit_bytes":
                        e["bounded"]["diagnostic_job_peak_process_commit_bytes"],
                    "diagnostic_job_peak_commit_bytes": e["bounded"]["diagnostic_job_peak_commit_bytes"]}
                   for i, e in sorted(present.items())],
        "verdicts": {i: e["bounded"]["verdict"] for i, e in sorted(present.items())},
    }


def least_squares(rows, ys):
    """Solve min |A x - y| by the normal equations (pure Python). Returns the
    coefficient list, or None if the system is singular."""
    n = len(rows[0])
    a = [[sum(r[i] * r[j] for r in rows) for j in range(n)] + [sum(r[i] * y for r, y in zip(rows, ys))]
         for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-12:
            return None
        a[col], a[pivot] = a[pivot], a[col]
        for r in range(n):
            if r != col:
                factor = a[r][col] / a[col][col]
                a[r] = [x - factor * p for x, p in zip(a[r], a[col])]
    return [a[i][n] / a[i][i] for i in range(n)]


def calibration(entries) -> dict:
    rows, ys = [], []
    for e in entries.values():
        work = e.get("work")
        if e.get("mode", "").startswith("pb") and work and e.get("planning_time_s") is not None \
                and e.get("record_status") in ("complete", "partial", "no_route"):
            rows.append([float(_lat(e)), float(work["leg_calls"]), float(work["nodes_expanded"]), 1.0])
            ys.append(float(e["planning_time_s"]))
    if len(rows) < 6:
        return {"status": "insufficient_data", "records": len(rows)}
    x = least_squares(rows, ys)
    if x is None:
        return {"status": "singular", "records": len(rows)}
    predicted = [sum(c * v for c, v in zip(x, r)) for r in rows]
    mean = sum(ys) / len(ys)
    ss_tot = sum((y - mean) ** 2 for y in ys)
    ss_res = sum((y - p) ** 2 for y, p in zip(ys, predicted))
    return {"status": "fitted", "records": len(rows), "model": "planning_time_s = a*expansions + b*leg_calls + c*nodes + d",
            "coefficients": {"a": x[0], "b": x[1], "c": x[2], "d": x[3]},
            "r_squared": None if ss_tot == 0 else 1 - ss_res / ss_tot,
            "max_abs_residual_s": max(abs(y - p) for y, p in zip(ys, predicted))}


def _lat(entry) -> float:
    return (entry["work"].get("lattice_expansions") or 0)


def _zero(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value == 0


def _budget_stopped(entry) -> bool:
    """A prefix run that a budget, cancellation or checkpoint stopped: its result says
    only what was found within that budget."""
    return entry.get("work_limit_hit") is not None or entry.get("stop_reason") in BUDGET_STOPS


def _certified(new, kind) -> bool:
    """Whether a COMPLETE prefix route is eligible for an exactness comparison. B3 must
    prove optimality (stop `exhausted`/`bound_closed`, no limit hit, no unresolved
    branch, no bound violation); B2 asks for no proof, only a route no budget stopped."""
    if new["outcome"] != "C" or _budget_stopped(new):
        return False
    if kind != "b3":
        return True
    return (new.get("optimality_proven") is True and new.get("stop_reason") in CLOSED_STOPS
            and _zero(new.get("unresolved_branch_count")) and _zero(new.get("bound_violation_count")))


def _cheaper_replay_valid(old, new) -> bool:
    """A complete prefix route that passed replay and costs strictly less than the old
    route's cost: counter-evidence to the old search's claimed optimum."""
    v = new.get("violations")
    return (isinstance(v, dict) and _zero(v.get("replay"))
            and _usable_number(old.get("total_cost")) and _usable_number(new.get("total_cost"))
            and new["total_cost"] < old["total_cost"])


def _pair_state(old, new, kind):
    """Comparison state for an old-search/prefix-search pair of entries: `ok`, `fail`,
    `not_certified` (the prefix result is not eligible for an exactness claim: never a
    pass and never a failure), `missing`, `excluded` (old timeout) or `unmeasured`."""
    if old["item_status"] == "missing" or new["item_status"] == "missing":
        return "missing"
    if old["outcome"] == "T":
        return "excluded"          # the old search did not finish: no exactness claim
    if old["outcome"] not in ("C", "P") or new["outcome"] not in ("C", "P", "T"):
        return "unmeasured"
    if new["outcome"] == "T":
        return "unmeasured"
    if old["outcome"] == "C" and new["outcome"] != "C":
        return "not_certified" if _budget_stopped(new) else "fail"
    if old["outcome"] == "P" and new["outcome"] == "C":
        return "fail"
    if old["outcome"] == "C":
        if not _certified(new, kind):
            if kind == "b3" and _cheaper_replay_valid(old, new):
                return "fail"
            return "not_certified"
        if kind == "b3" and repr(old["total_cost"]) != repr(new["total_cost"]):
            return "fail"
        return "ok"
    # Both are partial or no-route results.
    if _budget_stopped(new):
        return "not_certified"
    kinds = (old.get("route_kind"), new.get("route_kind"))
    if kinds == ("none", "none"):
        return "ok"
    if kinds != ("partial", "partial"):
        return "fail"                   # one found a partial route, the other did not (or unusable)
    for key in ("total_distance_cm", "total_cost"):
        if not _usable_number(old.get(key)) or not _usable_number(new.get(key)) \
                or repr(old[key]) != repr(new[key]):
            return "fail"               # partial accounting must be present and bit-equal
    return "ok"


def _usable_number(value) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _gates(entries, mes_complete, tallies) -> dict:
    gates = {g: {"title": t, "status": "not_measured", "reason": "no evidence recorded", "evidence": []}
             for g, t in GATE_TITLES.items()}

    def set_gate(name, status, reason, evidence=()):
        gates[name].update(status=status, reason=reason, evidence=sorted(set(evidence)))

    # G1
    bad = tallies["bad_records"]
    if bad:
        set_gate("G1", "fail", "violations, errors or unverified cleanup recorded", bad)
    elif tallies["records"] and mes_complete:
        set_gate("G1", "pass", "no violations across recorded evidence; minimum evidence set complete",
                 [i for i, e in entries.items() if e["item_status"] != "missing"])
    elif tallies["records"]:
        set_gate("G1", "not_measured", "no violations so far, but the minimum evidence set is incomplete")
    # G2
    states = {}
    for old_id, new_id, kind in EXACT_PAIRS:
        if old_id in entries and new_id in entries:
            states[(old_id, new_id)] = _pair_state(entries[old_id], entries[new_id], kind)
    evidence = [i for pair in states for i in pair]
    if not states or all(s == "missing" for s in states.values()):
        pass                                            # nothing recorded: the gate stays not_measured
    elif any(s == "fail" for s in states.values()):
        set_gate("G2", "fail", "old and prefix search disagree", evidence)
    elif states and all(s in ("ok", "excluded") for s in states.values()) \
            and any(s == "ok" for s in states.values()) and len(states) == len(EXACT_PAIRS):
        set_gate("G2", "pass", "agreement wherever the old search completed", evidence)
    elif states:
        listing = ", ".join(f"{o}/{n}={s}" for (o, n), s in states.items())
        uncertified = [f"{o}/{n}" for (o, n), s in states.items() if s == "not_certified"]
        reason = "some pairs missing, unmeasured or not certified: " + listing
        if uncertified:
            reason = ("not certified, so never a pass: " + ", ".join(uncertified) + "; " + reason)
        set_gate("G2", "not_measured", reason, evidence)
    # G3
    verdicts = []
    for first, second in DETERMINISM_GATE_PAIRS:
        a, b = entries.get(first), entries.get(second)
        if not a or not b or a["item_status"] == "missing" or b["item_status"] == "missing":
            verdicts.append("missing")
        elif a["outcome"] not in ("C", "P") or b["outcome"] not in ("C", "P"):
            verdicts.append("unmeasured")
        elif a["result_hash"] == b["result_hash"] and a["work"] == b["work"]:
            verdicts.append("ok")
        else:
            verdicts.append("fail")
    ev = [i for pair in DETERMINISM_GATE_PAIRS for i in pair]
    if "fail" in verdicts:
        set_gate("G3", "fail", "separate invocations differ", ev)
    elif verdicts and all(v == "ok" for v in verdicts):
        set_gate("G3", "pass", "identical hash and work at 3, 6 and 8 obstacles", ev)
    elif any(v != "missing" for v in verdicts):
        set_gate("G3", "not_measured", "pairs " + ",".join(verdicts), ev)
    # G4
    checks = []
    for case_id in G4_COMPLETE:
        e = entries.get(case_id)
        checks.append("missing" if not e or e["item_status"] == "missing" else
                      "ok" if e["outcome"] == "C" else "unmeasured" if e["outcome"] == "T" else "fail")
    for case_id in G4_IMPOSSIBLE:
        e = entries.get(case_id)
        checks.append("missing" if not e or e["item_status"] == "missing" else
                      "ok" if e.get("complete_route_proven_impossible") is True
                      else "unmeasured" if e["outcome"] == "T" else "fail")
    ev = list(G4_COMPLETE) + list(G4_IMPOSSIBLE)
    labels = [f"n{n}={c}" for n, c in zip(G4_REPRESENTATIVE, checks)] + \
             [f"{i}={c}" for i, c in zip(G4_IMPOSSIBLE, checks[len(G4_COMPLETE):])]
    if "fail" in checks:
        set_gate("G4", "fail", "a representative case did not complete or an infeasible case was not proven: "
                 + ", ".join(labels), ev)
    elif checks and all(c == "ok" for c in checks):
        set_gate("G4", "pass", "the representative spread fixture completes at each obstacle count from 4 "
                 "through 8, and the designated infeasible fixtures are proven impossible", ev)
    elif any(c != "missing" for c in checks):
        set_gate("G4", "not_measured", "checks " + ", ".join(labels), ev)
    # G5, G6 (bounded executor only)
    _gate_g5(entries, set_gate)
    _gate_g6(entries, set_gate)
    # G7
    mem, frontier = entries.get("X13"), entries.get("R28")
    if mem and mem["item_status"] != "missing":
        rss = mem.get("peak_rss_bytes")
        if mem["outcome"] == "T" or rss is None:
            set_gate("G7", "not_measured", "8-obstacle memory run timed out or has no RSS", ["X13"])
        elif rss > MEMORY_LIMIT_BYTES:
            set_gate("G7", "fail", f"peak RSS {rss} exceeds {MEMORY_LIMIT_BYTES}", ["X13"])
        elif frontier and frontier["item_status"] != "missing" and frontier["outcome"] in ("C", "P"):
            set_gate("G7", "pass", "peak RSS within limit; frontier-cap run recorded", ["X13", "R28"])
        else:
            set_gate("G7", "not_measured", "frontier-cap run missing", ["X13"])
    bounded = _bounded_present(entries)
    memory_fail = sorted(i for i, e in bounded.items() if e["bounded"]["checks"]["memory"] == "fail")
    memory_unread = sorted(i for i, e in bounded.items() if e["bounded"]["checks"]["memory"] == "not_measured")
    if memory_fail:
        set_gate("G7", "fail", "bounded peak RSS (host or worker) exceeds 1 GiB", memory_fail)
    elif memory_unread and gates["G7"]["status"] == "pass":
        set_gate("G7", "not_measured", "a bounded worker peak RSS was not read after exit", memory_unread)
    # G8
    kinds = {"leg_call_budget": ("max_leg_calls",), "cancelled": None, "frontier": ("max_frontier_entries",),
             "checkpoint": None}
    observed, dirty = {k: [] for k in kinds}, []
    for i, e in entries.items():
        if e["item_status"] == "missing" or not e.get("mode", "").startswith("pb"):
            continue
        stop, hit = e.get("stop_reason"), e.get("work_limit_hit")
        kind = ("cancelled" if stop == "cancelled" else "checkpoint" if stop == "wall_clock_checkpoint_limit"
                else "frontier" if hit == "max_frontier_entries" else "leg_call_budget" if hit == "max_leg_calls"
                else None)
        if kind:
            observed[kind].append(i)
            if (e.get("violations") or {}).get("invariant"):
                dirty.append(i)
    if dirty:
        set_gate("G8", "fail", "a limit or cancellation stop has metadata violations", dirty)
    elif all(observed.values()):
        set_gate("G8", "pass", "each stop kind observed with clean metadata",
                 [i for v in observed.values() for i in v])
    elif any(observed.values()):
        set_gate("G8", "not_measured", "stop kinds not yet observed: "
                 + ", ".join(k for k, v in observed.items() if not v),
                 [i for v in observed.values() for i in v])
    dishonest = sorted(i for i, e in _bounded_present(entries).items() if e["bounded"]["checks"]["honest"])
    if dishonest:
        set_gate("G8", "fail", "a bounded result claims a proof it does not have", dishonest)
    return gates


def build_summary(records, inputs=()) -> dict:
    check_provenance(records)      # before any coverage or gate is evaluated
    by_invocation, unassigned = _index(records)
    entries = {}
    session_status, campaign_items = {}, {}
    for session in cmp.all_sessions():
        statuses = []
        for case, invocation in cmp.session_invocations(session):
            entry = _entry(case, invocation, session, by_invocation.get(invocation))
            entries[invocation] = entry
            statuses.append(entry["item_status"])
            campaign_items.setdefault(session.campaign, []).append(entry["item_status"])
        session_status[f"{session.campaign}/{session.name}"] = cmp.coverage_status(statuses)
    requirements = {}
    for name, case_ids in cmp.REQUIREMENTS.items():
        items = [e["item_status"] for i, e in entries.items() if e["case_id"] in case_ids]
        requirements[name] = cmp.coverage_status(items)
    standard_status = cmp.coverage_status(campaign_items.get("standard", []))
    bad = []
    for i, e in entries.items():
        v = e.get("violations") or {}
        if e["item_status"] != "missing" and (
                v.get("replay") or v.get("b2_replay") or v.get("invariant")
                or e.get("outcome") in ("E",) or e.get("cleanup_death_verified") is False
                or e.get("cleanup_tempdir_removed") is False
                or e.get("bound_violation_count") or e.get("b3_not_worse_than_b2") is False
                or (e.get("bounded") or {}).get("g1_problems")):
            bad.append(i)
    tallies = {"records": len(by_invocation), "bad_records": bad}
    machines = sorted({json.dumps(r.get("machine"), sort_keys=True) for r in records if r.get("machine")})
    environment = None
    if records:
        first = next((r for r in records if r.get("machine")), records[0])
        environment = {
            "machine": first.get("machine"), "python": first.get("python"),
            "git_shas": sorted({_get(r, "git", "sha") for r in records if _get(r, "git", "sha")}),
            "git_dirty_any": any(bool(_get(r, "git", "dirty")) for r in records),
            "machine_variants": len(machines),
        }
    exact = [{"old": o, "prefix": n, "kind": k,
              "state": _pair_state(entries[o], entries[n], k)} for o, n, k in EXACT_PAIRS]
    overshoot = [{"invocation": i, **(_get(r, "validation", "overshoot") or {})}
                 for i, r in by_invocation.items() if _get(r, "validation", "overshoot")]
    return {
        "summary_schema_version": SUMMARY_SCHEMA_VERSION, "manifest_version": cmp.MANIFEST_VERSION,
        "evidence_policy": ("Generated by `report` from campaign JSONL kept outside the repository. "
                            "Nothing here is hand-written; every gate is not_measured until a record "
                            "supports a verdict."),
        "environment": environment, "inputs": list(inputs), "records_total": len(records),
        "unassigned_records": unassigned, "cases": entries,
        "coverage": {"sessions": session_status,
                     "campaigns": {c: cmp.coverage_status(v) for c, v in campaign_items.items()},
                     "minimum_evidence_set": standard_status, "requirements": requirements},
        "gates": _gates(entries, standard_status == "complete", tallies),
        "comparisons": exact, "overshoot": overshoot, "calibration": calibration(entries),
        "bounded": _bounded_summary(entries),
    }


def dumps_summary(summary) -> str:
    return json.dumps(summary, indent=2) + "\n"


def render_text(summary) -> str:
    lines = [f"records: {summary['records_total']} (unassigned {summary['unassigned_records']})",
             f"minimum evidence set: {summary['coverage']['minimum_evidence_set']}"]
    for name, status in summary["coverage"]["sessions"].items():
        lines.append(f"  session {name}: {status}")
    lines.append("gates:")
    for gate, info in summary["gates"].items():
        lines.append(f"  {gate} {info['title']}: {info['status']} - {info['reason']}")
    missing = [i for i, e in summary["cases"].items() if e["item_status"] == "missing"]
    lines.append(f"missing invocations: {len(missing)}")
    verdicts = (summary.get("bounded") or {}).get("verdicts") or {}
    if verdicts:
        lines.append("bounded invocations:")
        for invocation, verdict in verdicts.items():
            b = summary["cases"][invocation]["bounded"]
            problems = b["g1_problems"] + b["checks"]["honest"] + b["checks"]["avail"] + (
                b["checks"]["decide"] + b["checks"]["return"] if b["policy"] == "production" else []) + (
                b["checks"]["memory_problems"] if b["checks"]["memory"] != "ok" else [])
            lines.append(f"  {invocation}: {verdict} ({b['policy']}, {b['termination'] or b['row_terminations']})"
                         + ("" if not problems else " - " + "; ".join(problems[:4])))
    return "\n".join(lines)
