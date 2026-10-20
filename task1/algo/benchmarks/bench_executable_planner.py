"""Benchmark runner for the executable route optimizer (measurement only).

Run from `algo/`:

    python benchmarks/bench_executable_planner.py list
    python benchmarks/bench_executable_planner.py run --fixture A_spread_02 --mode b2 b3
    python benchmarks/bench_executable_planner.py run --fixture A_spread_04 --mode b3 --timeout 60

Every measurement runs in its own subprocess with a hard timeout, so a slow
search can never hang the harness; a timeout is a recorded outcome
(`status: "timeout"`), not a failure. One JSON object per line is written to
stdout (schema: README.md, `RESULT_KEYS` below); progress goes to stderr.

Safety: the current permutation search is exponential. The runner refuses
old-search (b2/b3) runs at 6 or more obstacles unless
`--allow-large-baseline` AND an explicit `--timeout` of at most 120 s are
given, bounds 4-5 obstacle runs to 120 s, and caps every timeout at 600 s.
The `legs` mode plans a few individual legs and is how per-leg cost is
measured at 6-8 obstacles without running the permutation search.

The planner is measured, never modified: this module only calls the public
optimizer functions with `stats=`, and (with `--profile hotspots`) installs
timing wrappers from `benchmarks/hotspots.py`.

Modes `pb2`/`pb3` measure the standalone prefix-tree search (README, "Prefix-
search validation"); `campaign` runs capped, resumable case sessions and
`report` summarises their JSONL into the evidence summary. Every case runs in a
job-contained worker whose death is verified (`process_control.py`). A harness
hard timeout is never planner metadata, and no planner checkpoint or work limit
is applied unless a flag asks for one.

Modes `bpb2`/`bpb3` measure the standalone bounded executor
(`bounded_prefix_planning.py`, worker side in `benchmarks/bounded_modes.py`)
under a named server-side policy. Schema 3 appends one key, `bounded`, which is
null for every other mode; every schema-2 key keeps its meaning.
"""

import argparse
import cProfile
import dataclasses
import hashlib
import json
import math
import os
import platform
import pstats
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmarks import bounded_modes  # noqa: E402
from benchmarks import campaigns as bcampaigns  # noqa: E402
from benchmarks import fixtures as bfx  # noqa: E402
from benchmarks import prefix_modes  # noqa: E402
from benchmarks import process_control  # noqa: E402
from benchmarks import report as breport  # noqa: E402
from benchmarks import route_replay  # noqa: E402
from config import DEFAULT_PLANNER_CONFIG  # noqa: E402
import executable_route_optimizer as ero  # noqa: E402
from targets import viewing_target  # noqa: E402

SCHEMA_VERSION = 3
KIND = "benchmark_result"
MODES = ("b2", "b3", "legs", "pb2", "pb3", "bpb2", "bpb3")
PROFILES = ("none", "hotspots", "cprofile")
STATUSES = ("complete", "partial", "no_route", "timeout", "error", "refused")
MAX_TIMEOUT_S = 600.0
BOUNDED_TIMEOUT_S = 120.0     # required for old-search runs at 4+ obstacles
DEFAULT_SMALL_TIMEOUT_S = 180.0   # old search, 1-3 obstacles
LARGE_OLD_SEARCH_N = 6        # old-search runs at or above this need the explicit flag
BOUNDED_OLD_SEARCH_N = 4      # old-search runs at or above this are always bounded
PREFIX_DEFAULT_TIMEOUT_S = 60.0   # prefix search, 1-3 obstacles; 4+ needs an explicit --timeout
PREFIX_DEFAULT_MAX_N = 3
EXIT_ERROR, EXIT_REFUSED, EXIT_ABORT, EXIT_VIOLATIONS, EXIT_INCOMPLETE = 1, 2, 3, 4, 5
EXIT_PROVENANCE = 6   # dirty/unknown Git state, another commit's resume file, or unusable `report` input
RESULT_KEYS = (
    "schema_version", "kind", "started_at_utc", "fixture_id", "family", "n_obstacles", "expect",
    "mode", "alternatives", "profile", "tracemalloc", "leg_records_included", "timeout_s",
    "status", "status_detail", "wall_time_s", "planning_time_s", "result", "result_hash",
    "stats", "partial_stats", "analysis", "hotspots", "cprofile_top", "leg_records",
    "machine", "python", "git", "peak_memory",
    "limits", "validation", "cleanup", "case",   # schema v2, appended
    "bounded",                                    # schema v3, appended: bpb2/bpb3 only, else null
)


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def machine_info() -> dict:
    return {
        "system": platform.system(), "release": platform.release(), "machine": platform.machine(),
        "processor": platform.processor(), "cpu_count": os.cpu_count(),
    }


def python_info() -> dict:
    return {"version": platform.python_version(), "implementation": platform.python_implementation()}


def git_info() -> dict:
    root = Path(__file__).resolve().parents[2]
    try:
        sha = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=10).stdout.strip() or None
        dirty = bool(subprocess.run(["git", "-C", str(root), "status", "--porcelain"],
                                    capture_output=True, text=True, timeout=10).stdout.strip())
    except Exception:
        sha, dirty = None, None
    return {"sha": sha, "dirty": dirty}


def peak_rss_bytes():
    """Peak resident set size of this process, or None if unavailable."""
    try:
        if sys.platform == "win32":
            import ctypes
            from ctypes import wintypes

            class Counters(ctypes.Structure):
                _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                            ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                            ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                            ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                            ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                            ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]

            counters = Counters()
            counters.cb = ctypes.sizeof(Counters)
            kernel32 = ctypes.WinDLL("kernel32")
            kernel32.GetCurrentProcess.restype = wintypes.HANDLE
            psapi = ctypes.WinDLL("psapi")
            psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(Counters), wintypes.DWORD]
            psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
            if psapi.GetProcessMemoryInfo(kernel32.GetCurrentProcess(), ctypes.byref(counters), counters.cb):
                return int(counters.PeakWorkingSetSize)
            return None
        import resource
        peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        return int(peak) * (1 if sys.platform == "darwin" else 1024)
    except Exception:
        return None


def redact(text: str) -> str:
    """Remove machine-specific absolute paths from diagnostic text."""
    for real, label in ((str(Path(__file__).resolve().parents[2]), "<repo>"), (str(Path.home()), "<home>")):
        text = text.replace(real, label).replace(real.replace("\\", "/"), label)
    return text


def resolve_timeout(mode: str, n_obstacles: int, timeout_s, allow_large: bool):
    """Apply the baseline safety rules. Returns `(timeout_s, refusal_reason)`;
    `refusal_reason` is None when the run is allowed."""
    if timeout_s is not None:
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            return None, "timeout must be a positive finite number of seconds"
        if timeout_s > MAX_TIMEOUT_S:
            return None, f"timeout {timeout_s:g}s exceeds the {MAX_TIMEOUT_S:g}s maximum"
    if mode == "legs":
        return (BOUNDED_TIMEOUT_S if timeout_s is None else timeout_s), None
    if n_obstacles >= LARGE_OLD_SEARCH_N:
        if not allow_large:
            return None, (f"the old permutation search is exponential ({n_obstacles}! orders); "
                          f"{mode} at {n_obstacles} obstacles is refused. Pass --allow-large-baseline "
                          f"with an explicit --timeout of at most {BOUNDED_TIMEOUT_S:g}s to record a bounded attempt")
        if timeout_s is None:
            return None, f"--allow-large-baseline requires an explicit --timeout of at most {BOUNDED_TIMEOUT_S:g}s"
        if timeout_s > BOUNDED_TIMEOUT_S:
            return None, f"large-baseline runs are limited to {BOUNDED_TIMEOUT_S:g}s, got {timeout_s:g}s"
        return timeout_s, None
    if n_obstacles >= BOUNDED_OLD_SEARCH_N:
        bound = BOUNDED_TIMEOUT_S if timeout_s is None else timeout_s
        if bound > BOUNDED_TIMEOUT_S:
            return None, (f"old-search runs at {n_obstacles} obstacles are limited to "
                          f"{BOUNDED_TIMEOUT_S:g}s, got {bound:g}s")
        return bound, None
    return (DEFAULT_SMALL_TIMEOUT_S if timeout_s is None else timeout_s), None


def resolve_prefix_timeout(n_obstacles: int, timeout_s):
    """Hard-timeout rules for `pb2`/`pb3`. Returns `(timeout_s, refusal_reason)`.
    The timeout is always finite and at most 600 s; up to 3 obstacles it
    defaults to 60 s, and 4+ obstacles require an explicit `--timeout`. The
    old-search 120 s cap does not apply, and no planner limit is implied."""
    if timeout_s is not None:
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            return None, "timeout must be a positive finite number of seconds"
        if timeout_s > MAX_TIMEOUT_S:
            return None, f"timeout {timeout_s:g}s exceeds the {MAX_TIMEOUT_S:g}s maximum"
        return timeout_s, None
    if n_obstacles <= PREFIX_DEFAULT_MAX_N:
        return PREFIX_DEFAULT_TIMEOUT_S, None
    return None, (f"prefix-search runs at {n_obstacles} obstacles require an explicit --timeout "
                  f"(at most {MAX_TIMEOUT_S:g}s)")


def prefix_option_refusal(mode: str, options: dict):
    """Prefix limits only make sense for pb2/pb3; anything else refuses."""
    if prefix_modes.is_prefix_mode(mode):
        return None
    given = [k for k in prefix_modes.LIMIT_KEYS if options.get(k) is not None]
    if given:
        return (f"{', '.join('--' + k.replace('_', '-') for k in given)} apply only to the pb2/pb3 "
                f"modes, not {mode}")
    return None


def _digest(payload) -> str:
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def route_summary(result) -> dict:
    commands = [c for leg in result.legs for c in leg.command_plan.commands]
    return {
        "is_valid": result.is_valid, "is_complete": result.is_complete,
        "obstacle_order": list(result.obstacle_order),
        "skipped_obstacle_ids": list(result.skipped_obstacle_ids),
        "total_cost": result.total_cost, "total_distance_cm": result.total_distance_cm,
        "n_legs": len(result.legs), "n_commands": len(commands),
        "reverse_commands": sum(1 for c in commands if c.direction in ("B", "BL", "BR")),
        "orders_considered": result.orders_considered, "valid_orders": result.valid_orders,
        "rejected_orders": result.rejected_orders, "pruned_orders": result.pruned_orders,
        "failure_reason": result.failure_reason,
    }


def route_hash(result) -> str:
    pose = result.final_pose
    return _digest({
        "valid": result.is_valid, "complete": result.is_complete,
        "order": list(result.obstacle_order), "skipped": list(result.skipped_obstacle_ids),
        "cost": repr(result.total_cost),
        "commands": [[c.stm_command for c in leg.command_plan.commands] for leg in result.legs],
        "final_pose": None if pose is None else [repr(pose.x), repr(pose.y), repr(pose.heading)],
    })


def route_status(result) -> str:
    if not result.is_valid:
        return "no_route"
    return "complete" if result.is_complete else "partial"


def _share(part, whole):
    return part / whole if whole else None


def analyze(stats, planning_s: float, hotspot_report) -> dict:
    """Derived measurements behind hypotheses H1-H6 (see README.md)."""
    planned = stats.leg_planner_s
    separable = stats.cache_misses > 0 and stats.separable_legs == stats.cache_misses
    bins = []
    for low, high in ((0, 30), (30, 90), (90, 150), (150, 181)):
        legs = [r for r in stats.leg_records if low <= r.heading_diff_deg < high]
        bins.append({
            "heading_diff_deg": [low, min(high, 180)], "legs": len(legs),
            "failed": sum(1 for r in legs if r.outcome != "success"),
            "mean_expansions": _share(sum(r.expansions for r in legs), len(legs)),
            "mean_elapsed_s": _share(sum(r.elapsed_s for r in legs), len(legs)),
        })
    analysis = {
        "leg_planner_share_of_planning": _share(planned, planning_s),
        "outcome_time_shares": {name: _share(row[1], planned) for name, row in sorted(stats.outcomes.items())},
        "failed_or_limited_time_share": _share(
            sum(row[1] for name, row in stats.outcomes.items() if name != "success"), planned),
        "lattice_time_share": _share(stats.lattice_s, planned) if separable else None,
        "postprocess_time_share": _share(stats.postprocess_s, planned) if separable else None,
        "seconds_per_expansion": _share(stats.lattice_s, stats.lattice_expansions) if separable else None,
        "heading_bins": bins,
    }
    if hotspot_report:
        analysis["hotspot_time_shares_of_leg_planner"] = {
            label: _share(row["seconds"], planned) for label, row in hotspot_report["functions"].items()
        }
    return analysis


def cprofile_rows(profile, top: int = 25) -> dict:
    rows = []
    for (filename, line, name), (_cc, calls, tottime, cumtime, _callers) in pstats.Stats(profile).stats.items():
        rows.append({"function": f"{Path(filename).name}:{line}({name})", "calls": calls,
                     "tottime_s": tottime, "cumtime_s": cumtime})
    return {
        "by_tottime": sorted(rows, key=lambda r: r["tottime_s"], reverse=True)[:top],
        "by_cumtime": sorted(rows, key=lambda r: r["cumtime_s"], reverse=True)[:top],
    }


class ProgressWriter(threading.Thread):
    """Writes a statistics snapshot to a file every `interval` seconds so a
    run killed by its timeout still leaves partial statistics behind."""

    def __init__(self, path, stats, interval: float):
        super().__init__(daemon=True)
        self.path, self.stats, self.interval = path, stats, interval
        self._halt = threading.Event()

    def write(self) -> None:
        if not self.path:
            return
        try:
            temporary = self.path + ".tmp"
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(self.stats.to_dict(), handle)
            os.replace(temporary, self.path)
        except Exception:
            pass    # a torn read of live counters is simply skipped

    def run(self) -> None:
        while not self._halt.wait(self.interval):
            self.write()

    def stop(self) -> None:
        self._halt.set()


def run_search_mode(fixture, mode: str, alternatives: bool, stats):
    function = ero.find_executable_b2_route if mode == "b2" else ero.optimise_executable_b3_route
    result = function(fixture.start, fixture.obstacles, DEFAULT_PLANNER_CONFIG,
                      allow_alternative_viewing=alternatives, stats=stats)
    return route_status(result), route_summary(result), route_hash(result), result


def run_legs_mode(fixture, stats):
    """Plan a fixed, deterministic set of individual legs with the real
    pipeline: start -> primary target i for every obstacle (depth 0), then
    primary i -> primary i+1 in input order (depth 1). Every leg starts from
    an IDEAL pose, not a previous leg estimated final pose, so this measures
    per-leg cost, not a route."""
    config = DEFAULT_PLANNER_CONFIG
    targets = [viewing_target(o, config) for o in fixture.obstacles]
    legs = [(fixture.start, target, 0) for target in targets]
    if len(targets) > 1:
        legs += [(targets[i], targets[(i + 1) % len(targets)], 1) for i in range(len(targets))]
    stats.begin("legs")
    digest, outcomes, valid = [], {}, 0
    try:
        for start, target, depth in legs:
            stats.note_request(depth, False)
            plan = stats.plan_leg(ero.plan_executable_leg, start, target, fixture.obstacles, config, depth)
            outcome = "success" if plan.is_valid else str(plan.reason)
            outcomes[outcome] = outcomes.get(outcome, 0) + 1
            valid += plan.is_valid
            digest.append([outcome, repr(plan.total_cost), [c.stm_command for c in plan.commands]])
    finally:
        stats.end()
    summary = {"chained_from": "ideal_target_poses", "legs_planned": len(legs), "legs_valid": valid,
               "outcome_counts": dict(sorted(outcomes.items()))}
    return "complete", summary, _digest(digest)


def run_worker(spec: dict) -> dict:
    """Run one measurement in this process and return its raw record."""
    import tracemalloc
    if bounded_modes.is_bounded_mode(spec["mode"]):
        return bounded_modes.run_worker(spec, peak_rss=peak_rss_bytes, redact=redact)
    fixture = bfx.load_fixture(spec["fixture_id"])
    stats = ero.RouteSearchStats(record_legs=True)
    writer = ProgressWriter(spec.get("progress_file"), stats, spec.get("progress_interval_s", 1.0))
    hotspots = None
    profile = None
    if spec["profile"] == "hotspots":
        from benchmarks.hotspots import Hotspots
        hotspots = Hotspots()
    elif spec["profile"] == "cprofile":
        profile = cProfile.Profile()
    if spec["tracemalloc"]:
        tracemalloc.start()
    writer.write()
    writer.start()
    record = {"status_detail": None, "hotspots": None, "cprofile_top": None}
    if hotspots:
        hotspots.install()
    mode = spec["mode"]
    alternatives = bool(spec["alternatives"])
    status = summary = digest = result_obj = None
    started = time.perf_counter()
    try:
        if profile:
            profile.enable()
        try:
            if mode == "legs":
                status, summary, digest = run_legs_mode(fixture, stats)
            elif prefix_modes.is_prefix_mode(mode):
                result_obj = prefix_modes.run_prefix_search(fixture, mode, alternatives, spec, stats)
            else:
                status, summary, digest, result_obj = run_search_mode(fixture, mode, alternatives, stats)
        finally:
            if profile:
                profile.disable()
    except Exception as error:   # a recorded outcome, not a crash of the harness
        status, summary, digest, result_obj = "error", None, None, None
        record["status_detail"] = redact(f"{type(error).__name__}: {error}")[:500]
    finally:
        planning_s = time.perf_counter() - started
        if hotspots:
            hotspots.uninstall()
        writer.stop()
    # Normalisation and metadata checks run AFTER the timed search and outside
    # any profiler, so planning_time_s never includes them.
    validation = None
    if result_obj is not None:
        normalize_started = time.perf_counter()
        try:
            if prefix_modes.is_prefix_mode(mode):
                status, summary, digest, validation = prefix_modes.prefix_validation(
                    result_obj, fixture, mode, alternatives, spec, stats)
            else:
                validation = prefix_modes.ero_validation(result_obj, fixture, mode, alternatives)
            validation["timing"]["normalize_s"] = time.perf_counter() - normalize_started
        except Exception as error:
            status, summary, digest, validation = "error", None, None, None
            record["status_detail"] = redact(f"normalisation failed: {type(error).__name__}: {error}")[:500]
    if hotspots:
        record["hotspots"] = hotspots.report()
    if profile:
        record["cprofile_top"] = cprofile_rows(profile)
        if spec.get("profile_out"):
            directory = Path(spec["profile_out"])
            directory.mkdir(parents=True, exist_ok=True)
            label = f"{spec['fixture_id']}_{spec['mode']}_{'alt' if spec['alternatives'] else 'prim'}"
            profile.dump_stats(str(directory / f"{label}.pstats"))
    tm_peak = None
    if spec["tracemalloc"]:
        tm_peak = tracemalloc.get_traced_memory()[1]
        tracemalloc.stop()
    record.update({
        "status": status, "planning_time_s": planning_s, "result": summary, "result_hash": digest,
        "stats": stats.to_dict(), "analysis": analyze(stats, planning_s, record["hotspots"]),
        "leg_records": [dataclasses.asdict(r) for r in stats.leg_records]
        if spec.get("leg_records") else None,
        "peak_memory": {"peak_rss_bytes": peak_rss_bytes(), "tracemalloc_peak_bytes": tm_peak},
        "limits": prefix_modes.limits_record(spec) if prefix_modes.is_prefix_mode(mode) else None,
        "validation": validation,
    })
    return record


_GIT = None


def blank_record(fixture, mode: str, alternatives, options: dict) -> dict:
    global _GIT
    if _GIT is None:
        _GIT = git_info()
    record = dict.fromkeys(RESULT_KEYS)
    record.update(
        schema_version=SCHEMA_VERSION, kind=KIND, started_at_utc=utc_now(),
        fixture_id=fixture.fixture_id, family=fixture.family, n_obstacles=fixture.n_obstacles,
        expect=fixture.expect, mode=mode, alternatives=alternatives, profile=options["profile"],
        tracemalloc=options["tracemalloc"], leg_records_included=options["leg_records"],
        machine=machine_info(), python=python_info(), git=_GIT, case=options.get("case"),
    )
    return record


def read_progress(path: str):
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except Exception:
        return None


def last_json_line(text: str):
    for line in reversed(text.strip().splitlines()):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return None


def run_isolated(fixture, mode: str, alternatives, options: dict) -> dict:
    """One measurement in a subprocess with a hard timeout. Always returns a
    full record; refusals, timeouts and crashes are recorded outcomes."""
    record = blank_record(fixture, mode, alternatives, options)
    bounded = bounded_modes.is_bounded_mode(mode)
    if bounded:
        record["leg_records_included"] = False     # the search runs in the executor's worker
    refusal = prefix_option_refusal(mode, options) or bounded_modes.option_refusal(mode, options)
    timeout_s = None
    if refusal is None:
        if bounded:
            timeout_s, refusal = bounded_modes.resolve_timeout(
                options["timeout_s"], options.get("policy"), options.get("soak_plans"))
        elif prefix_modes.is_prefix_mode(mode):
            timeout_s, refusal = resolve_prefix_timeout(fixture.n_obstacles, options["timeout_s"])
        else:
            timeout_s, refusal = resolve_timeout(
                mode, fixture.n_obstacles, options["timeout_s"], options["allow_large_baseline"])
    if refusal:
        record.update(status="refused", status_detail=refusal, timeout_s=options["timeout_s"])
        return record
    record["timeout_s"] = timeout_s
    scratch = tempfile.mkdtemp(prefix="bench_")
    sentinel = None
    try:
        progress = os.path.join(scratch, "progress.json")
        spec = {
            "fixture_id": fixture.fixture_id, "mode": mode, "alternatives": bool(alternatives),
            "profile": options["profile"], "tracemalloc": options["tracemalloc"],
            "leg_records": options["leg_records"], "profile_out": options["profile_out"],
            "progress_file": progress, "progress_interval_s": options["progress_interval_s"],
            "case_id": (options.get("case") or {}).get("invocation"),
        }
        for key in prefix_modes.LIMIT_KEYS:
            spec[key] = options.get(key)
        if bounded:
            for key in bounded_modes.OPTION_KEYS:
                spec[key] = options.get(key)
            spec["policy"] = options.get("policy") or "production"
            spec["private_tmp"] = os.path.join(scratch, "tmp")
            record["bounded"] = {"policy": bounded_modes.policy_record(spec["policy"]),
                                 "stall": spec["stall"], "soak_plans": spec["soak_plans"], "harness": None}
            sentinel = bounded_modes.start_sentinel()
        command = [sys.executable, str(Path(__file__).resolve()), "worker", "--spec", json.dumps(spec)]
        started = time.perf_counter()
        try:
            outcome = process_control.run_worker(
                command, timeout_s,
                memory_cap_bytes=options.get("memory_cap_bytes", process_control.DEFAULT_MEMORY_CAP_BYTES))
        except process_control.CleanupError as error:
            record.update(
                status="error", status_detail=redact(f"worker cleanup failed: {error}"),
                wall_time_s=time.perf_counter() - started, partial_stats=read_progress(progress),
                cleanup=error.cleanup)
            return record
        record["wall_time_s"] = time.perf_counter() - started
        record["cleanup"] = outcome.cleanup
        harness = None
        if bounded:
            report = outcome.job_report or {}
            harness = {"job_active_processes_before_close": report.get("active_processes"),
                       "job_peak_process_commit_bytes": report.get("peak_process_commit_bytes"),
                       "job_peak_commit_bytes": report.get("peak_job_commit_bytes")}
            harness.update(bounded_modes.stop_sentinel(sentinel))
            sentinel = None
            record["bounded"]["harness"] = harness
        if outcome.timed_out:
            # A HARNESS hard timeout: the worker was killed. This says nothing about the
            # planner's result, so every planner and validation field stays null.
            record.update(status="timeout", partial_stats=read_progress(progress),
                          status_detail=f"killed after the {timeout_s:g}s harness hard timeout")
            return record
        payload = last_json_line(outcome.stdout)
        if outcome.returncode != 0 or payload is None:
            tail = redact(outcome.stderr.strip()[-500:])
            record.update(status="error", status_detail=f"worker exited {outcome.returncode}: {tail}",
                          partial_stats=read_progress(progress))
            return record
        for key, value in payload.items():
            if key in RESULT_KEYS:
                record[key] = value
        if bounded and isinstance(record.get("bounded"), dict):
            record["bounded"]["harness"] = harness
            bounded_modes.replay_routes(record["bounded"], fixture)     # parent side, after the worker
        validation = record.get("validation")
        if isinstance(validation, dict) and validation.get("normalized"):
            # Replay runs here, in the parent, so it never inflates planning time or
            # eats into the worker's hard timeout.
            replay_started = time.perf_counter()
            validation["replay_violations"] = route_replay.replay_violations(validation["normalized"], fixture)
            if validation.get("b2_normalized"):
                validation["b2_replay_violations"] = route_replay.replay_violations(
                    validation["b2_normalized"], fixture)
            validation["timing"]["replay_s"] = time.perf_counter() - replay_started
        return record
    finally:
        if sentinel is not None:     # an early return: still stop the harness-owned sentinel
            stopped = bounded_modes.stop_sentinel(sentinel)
            if isinstance(record.get("bounded"), dict):
                record["bounded"]["harness"] = {"job_active_processes_before_close": None, **stopped}
        shutil.rmtree(scratch, ignore_errors=True)
        if isinstance(record.get("cleanup"), dict):
            record["cleanup"]["tempdir_removed"] = not os.path.exists(scratch)


def selected_fixture_ids(args) -> list:
    known = bfx.list_fixture_ids()
    chosen = []
    for fixture_id in args.fixture or []:
        if fixture_id not in known:
            raise SystemExit(f"unknown fixture {fixture_id!r}; use the list command")
        chosen.append(fixture_id)
    for family in args.family or []:
        chosen += [f for f in known if f.startswith(family)]
    if args.all:
        chosen = known
    if not chosen:
        raise SystemExit("select fixtures with --fixture ID, --family PREFIX or --all")
    return [f for f in known if f in set(chosen)]


def cmd_list(_args) -> int:
    for fixture_id in bfx.list_fixture_ids():
        fixture = bfx.load_fixture(fixture_id)
        print(json.dumps({
            "kind": "fixture", "fixture_id": fixture_id, "family": fixture.family,
            "n_obstacles": fixture.n_obstacles, "expect": fixture.expect,
            "tags": list(fixture.tags), "requires_alternatives": fixture.requires_alternatives,
        }))
    return 0


def record_has_violations(record: dict) -> bool:
    if isinstance(record.get("bounded"), dict):
        return bool(breport.bounded_problems(record))
    validation = record.get("validation")
    if not isinstance(validation, dict):
        return False
    raw = validation.get("raw") or {}
    return bool(validation.get("replay_violations") or validation.get("b2_replay_violations")
                or validation.get("invariant_violations") or raw.get("bound_violation_count")
                or validation.get("b3_not_worse_than_b2") is False)


def execute_plan(plan, base_options: dict, emit) -> int:
    """Run `(fixture, mode, alternatives, extra_options)` entries one at a time,
    each in its own contained worker. Exit codes: 0 ok, 1 an error record, 2 a
    refusal, 4 replay/metadata/bound-audit violations, 3 abort (a worker's
    death could not be verified: the plan stops at once, never continuing with
    a possibly live worker)."""
    exit_code = 0
    for index, (fixture, mode, alternatives, extra) in enumerate(plan, 1):
        print(f"[{index}/{len(plan)}] {fixture.fixture_id} {mode} alternatives={alternatives}",
              file=sys.stderr, flush=True)
        record = run_isolated(fixture, mode, alternatives, {**base_options, **extra})
        emit(record)
        if record["status"] == "error" and (record.get("cleanup") or {}).get("death_verified") is False:
            print("aborting: a worker's death could not be verified", file=sys.stderr, flush=True)
            return EXIT_ABORT
        left = ((record.get("bounded") or {}).get("harness") or {}).get("job_active_processes_before_close")
        if left:
            print(f"aborting: {left} process(es) were still active in the worker's job before it was closed",
                  file=sys.stderr, flush=True)
            return EXIT_ABORT
        if record["status"] == "refused":
            exit_code = max(exit_code, EXIT_REFUSED)
        elif record["status"] == "error":
            exit_code = max(exit_code, EXIT_ERROR)
        if record_has_violations(record):
            exit_code = max(exit_code, EXIT_VIOLATIONS)
    return exit_code


def base_options(args) -> dict:
    return {
        "timeout_s": args.timeout, "allow_large_baseline": args.allow_large_baseline,
        "profile": args.profile, "tracemalloc": args.tracemalloc, "leg_records": args.leg_records,
        "profile_out": args.profile_out, "progress_interval_s": args.progress_interval,
        "max_leg_calls": args.max_leg_calls, "max_expansions": args.max_expansions,
        "max_nodes": args.max_nodes, "max_frontier": args.max_frontier,
        "checkpoint_s": args.checkpoint_s, "cancel_after_legs": args.cancel_after_legs,
        "policy": args.policy, "stall": args.stall, "soak_plans": args.soak_plans,
    }


def cmd_run(args) -> int:
    options = base_options(args)
    plan = []
    for fixture_id in selected_fixture_ids(args):
        fixture = bfx.load_fixture(fixture_id)
        for mode in args.mode:
            if mode == "legs":
                entries = [(fixture, mode, None, {})]
            else:
                entries = [(fixture, mode, flag == "on", {}) for flag in args.alternatives]
            plan += entries * args.repeat     # each repeat is a separate worker invocation
    return execute_plan(plan, options, lambda record: print(json.dumps(record), flush=True))


def _campaign_sessions(args):
    if args.session == "all":
        return list(bcampaigns.SESSIONS[args.name])
    try:
        return [bcampaigns.find_session(args.name, args.session)]
    except KeyError as error:
        raise SystemExit(str(error.args[0]))


def _load_records(path) -> list:
    if not path or not os.path.exists(path):
        return []
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def _load_done(path) -> frozenset:
    done = set()
    for record in _load_records(path):
        case = record.get("case")
        if isinstance(case, dict) and case.get("invocation"):
            done.add(case["invocation"])
    return frozenset(done)


def start_refusal(state: dict):
    """Why a campaign may not start from this Git state, or None. The commit must be
    known and the working tree (untracked files included) known to be clean, because
    every record carries that state and `report` accepts evidence of one clean commit only."""
    if not breport.is_commit_sha(state.get("sha")):
        return "the current Git commit is unknown"
    if state.get("dirty") is not False:
        return ("the working tree is dirty or its state is unknown (commit, stash or remove every "
                "change, untracked files included, before collecting evidence)")
    return None


def cmd_campaign(args) -> int:
    sessions = _campaign_sessions(args)
    if args.dry_run:
        fits = True
        for session in sessions:
            arithmetic = bcampaigns.session_arithmetic(session)
            fits &= arithmetic["fits"]
            print(json.dumps({"kind": "campaign_dry_run", **arithmetic}), flush=True)
        return 0 if fits else EXIT_ERROR
    if len(sessions) != 1:
        raise SystemExit("choose one session with --session (the 'all' choice is for --dry-run)")
    if not args.out:
        raise SystemExit("--out FILE.jsonl is required (keep it outside the repository)")
    session = sessions[0]
    if args.budget_s is not None and not 0 <= args.budget_s <= session.cap_s:
        raise SystemExit(f"--budget-s must be between 0 and the session cap {session.cap_s}")
    state = git_info()
    refusal = start_refusal(state)
    if refusal:
        print(f"campaign refused: {refusal}", file=sys.stderr)
        return EXIT_PROVENANCE
    global _GIT
    _GIT = state    # every record of this session carries exactly the state that was verified
    if args.resume and os.path.exists(args.out):
        problems = breport.provenance_problems(_load_records(args.out), expected_sha=state["sha"])
        if problems:
            print("campaign refused: cannot resume a file that is not clean evidence of the current "
                  "commit: " + "; ".join(problems[:breport.MAX_PROVENANCE_PROBLEMS]), file=sys.stderr)
            return EXIT_PROVENANCE
    done = _load_done(args.out) if args.resume else frozenset()
    if not args.resume and os.path.exists(args.out) and _load_done(args.out):
        raise SystemExit(f"{args.out} already holds records; pass --resume to continue it")
    plan = bcampaigns.plan_session_run(session, done, args.budget_s)
    entries = []
    for case, invocation in plan["to_run"]:
        extra = case.option_dict()
        extra.update(timeout_s=float(case.timeout_s), leg_records=True, allow_large_baseline=False,
                     case={"campaign": session.campaign, "session": session.name,
                           "case_id": case.case_id, "invocation": invocation})
        extra.setdefault("profile", "none")
        extra.setdefault("tracemalloc", False)
        entries.append((bfx.load_fixture(case.fixture), case.mode, case.alternatives == "on", extra))
    options = {"timeout_s": None, "allow_large_baseline": False, "profile": "none", "tracemalloc": False,
               "leg_records": True, "profile_out": None, "progress_interval_s": 1.0}
    for key in prefix_modes.LIMIT_KEYS + bounded_modes.OPTION_KEYS:
        options[key] = None

    def emit(record):
        with open(args.out, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")

    exit_code = execute_plan(entries, options, emit)
    if plan["refused"]:
        for case, invocation in plan["refused"]:
            print(f"MISSING (does not fit the remaining session budget): {invocation} "
                  f"[{case.timeout_s} s + {bcampaigns.ALLOWANCE_S} s allowance]", file=sys.stderr)
        exit_code = exit_code if exit_code == EXIT_ABORT else max(exit_code, EXIT_INCOMPLETE)
    return exit_code


def cmd_report(args) -> int:
    started = time.perf_counter()
    records, inputs = breport.load_records(args.input or [])
    try:
        summary = breport.build_summary(records, inputs)
    except breport.ProvenanceError as error:
        print(f"report refused: {error}", file=sys.stderr)   # nothing is printed or written
        return EXIT_PROVENANCE
    text = breport.dumps_summary(summary)
    if args.summary_out:
        Path(args.summary_out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.summary_out).write_text(text, encoding="utf-8", newline="\n")
    print(text if args.format == "json" else breport.render_text(summary))
    print(json.dumps({"report_s": time.perf_counter() - started}), file=sys.stderr)
    return 0


def cmd_worker(args) -> int:
    print(json.dumps(run_worker(json.loads(args.spec))), flush=True)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="print the fixtures as JSON lines").set_defaults(func=cmd_list)
    run = sub.add_parser("run", help="measure fixtures; one JSON line per measurement")
    run.add_argument("--fixture", action="append", help="fixture id (repeatable)")
    run.add_argument("--family", action="append", help="fixture id prefix, e.g. A_spread (repeatable)")
    run.add_argument("--all", action="store_true", help="every fixture")
    run.add_argument("--mode", nargs="+", choices=MODES, default=["b3"])
    run.add_argument("--alternatives", nargs="+", choices=("off", "on"), default=["off"],
                     help="allow_alternative_viewing for b2/b3 (ignored by legs)")
    run.add_argument("--timeout", type=float, default=None,
                     help="hard timeout in seconds (safety rules: module docstring)")
    run.add_argument("--allow-large-baseline", action="store_true",
                     help="permit old-search b2/b3 at 6+ obstacles (requires --timeout <= 120)")
    run.add_argument("--profile", choices=PROFILES, default="none")
    run.add_argument("--profile-out", default=None, help="directory for cProfile .pstats dumps")
    run.add_argument("--tracemalloc", action="store_true", help="also record peak Python heap (slower)")
    run.add_argument("--leg-records", action="store_true", help="include every planned leg in the output")
    run.add_argument("--progress-interval", type=float, default=1.0)
    run.add_argument("--repeat", type=int, default=1, help="repeat each run as separate worker invocations")
    run.add_argument("--max-leg-calls", type=int, default=None, help="pb2/pb3 work limit: leg-planner calls")
    run.add_argument("--max-expansions", type=int, default=None, help="pb2/pb3 work limit: lattice expansions")
    run.add_argument("--max-nodes", type=int, default=None, help="pb2/pb3 work limit: node expansions")
    run.add_argument("--max-frontier", type=int, default=None, help="pb2/pb3 work limit: B3 frontier entries")
    run.add_argument("--checkpoint-s", type=float, default=None,
                     help="pb2/pb3 planner checkpoint (between legs); default NONE: correctness runs "
                          "rely on the hard timeout alone")
    run.add_argument("--cancel-after-legs", type=int, default=None,
                     help="pb2/pb3: deterministic cancellation after N leg-planner calls")
    run.add_argument("--policy", choices=bounded_modes.POLICY_NAMES, default=None,
                     help="bpb2/bpb3: server-side timing policy (default production)")
    run.add_argument("--stall", choices=tuple(bounded_modes.STALLS), default=None,
                     help="bpb2/bpb3: deterministic stall of the test worker (tests/_bounded_test_worker.py)")
    run.add_argument("--soak-plans", type=int, default=None,
                     help="bpb2/bpb3: sequential plans on one executor instance (needs --timeout)")
    run.set_defaults(func=cmd_run)
    campaign = sub.add_parser("campaign", help="run or dry-run a capped, resumable campaign session")
    campaign.add_argument("name", choices=tuple(bcampaigns.SESSIONS))
    campaign.add_argument("--session", default=None,
                          help="Q | S1..S3 | E1..E3 | BQ | B1..B3 | all (dry-run only)")
    campaign.add_argument("--out", default=None, help="JSONL output file, outside the repository")
    campaign.add_argument("--resume", action="store_true", help="skip invocations already in --out")
    campaign.add_argument("--dry-run", action="store_true", help="print the arithmetic; run nothing")
    campaign.add_argument("--budget-s", type=int, default=None, help="lower the session budget (<= cap)")
    campaign.set_defaults(func=cmd_campaign)
    report = sub.add_parser("report", help="summarise campaign JSONL; runs no planner")
    report.add_argument("--input", nargs="*", default=[], help="campaign JSONL files")
    report.add_argument("--summary-out", default=None, help="write the evidence summary JSON here")
    report.add_argument("--format", choices=("text", "json"), default="text")
    report.set_defaults(func=cmd_report)
    worker = sub.add_parser("worker", help=argparse.SUPPRESS)
    worker.add_argument("--spec", required=True)
    worker.set_defaults(func=cmd_worker)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
