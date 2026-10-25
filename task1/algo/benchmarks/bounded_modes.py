"""Worker side of the bounded-executor benchmark modes (measurement only).

`bpb2`/`bpb3` run the standalone `bounded_prefix_planning.BoundedPrefixPlanner`
exactly as a future server would: one `plan()` call with `R0` taken on this
process's `time.monotonic()` immediately before the call (so worker start-up
counts toward the 30 s target, but this benchmark worker's own interpreter
start-up does not). The executor, the prefix search and the canonical pipeline
are used unchanged; nothing here changes planner behaviour.

Timing is chosen by a server-side policy name, never by a request field:

    production   `bounded_prefix_planning.PRODUCTION_POLICY` (63/66/67/67.5/68/70 s)
    stress       `ExecutionPolicy(5, 10, 15, 17, 18, 20)`: forces early stops to
                 exercise retention and cleanup; never timing evidence (G5/G6)

`stall` swaps the worker command, through the executor's server-side
`worker_command` seam, for the TEST worker `tests/_bounded_test_worker.py real`
(the real protocol, search and leg planner) with a deterministic stall:
`at_call_1` sleeps at the first leg-planner call (a guaranteed hard-stop kill
with no incumbent), `before_final` lets the search finish and send its
incumbents, then withholds FINAL (a guaranteed hard stop that must retain
them). `soak_plans` runs that many sequential plans on ONE planner instance.

Observation (read-only, nothing is signalled here):

* `worker_containment.start_worker` is wrapped for the duration of each plan so
  the exact worker `Popen` the executor started is recorded; the wrapper returns
  the same object (the `hotspots.py` pattern: installed only by this runner).
  That handle gives the worker's exit (`poll()`) and its peak working set
  (`GetProcessMemoryInfo`, readable after exit while the handle is open).
* This benchmark worker runs inside the harness Job Object, and the executor
  nests its own job for the prefix worker, so the harness job's active-process
  count (queried with a NULL handle: the job of this process) must reach 2 during
  a plan and return to exactly 1 (this process) afterwards.
* `tempfile.tempdir` is pointed at a private folder, so every `prefix_plan_*`
  directory the executor creates lands there and leftovers can be counted.

The record's `bounded` block is described in `benchmarks/README.md`. Checks and
gate verdicts are computed later, by `report.py`, from these recorded facts.
"""

import dataclasses
import json
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

BOUNDED_MODES = ("bpb2", "bpb3")
OPTION_KEYS = ("policy", "stall", "soak_plans")
POLICY_NAMES = ("production", "stress")
STALLS = {"at_call_1": {"stall_at_call": 1}, "before_final": {"stall_before_final": True}}
DEFAULT_TIMEOUT_S = 90.0
MAX_TIMEOUT_S = 600.0
MAX_SOAK_PLANS = 20
SAMPLE_INTERVAL_S = 0.05
TEST_WORKER = Path(__file__).resolve().parents[1] / "tests" / "_bounded_test_worker.py"
TEMPDIR_PREFIX = "prefix_plan_"
R0_ANCHOR = "bench worker time.monotonic() immediately before plan()"
ROUTE_KEYS = ("route", "b2_route", "diagnostic_route")
SENTINEL_MARKER = "bench_executable_planner-bounded-sentinel"

_WIN = sys.platform == "win32"


def is_bounded_mode(mode) -> bool:
    return mode in BOUNDED_MODES


def policy_for(name: str):
    """The server-side `ExecutionPolicy` for a policy name."""
    import bounded_prefix_planning as bpp
    if name == "production":
        return bpp.PRODUCTION_POLICY
    if name == "stress":
        return bpp.ExecutionPolicy(planning_target_s=5.0, hard_stop_s=10.0, verification_deadline_s=15.0,
                                   return_target_s=17.0, return_deadline_s=18.0, request_ceiling_s=20.0)
    raise ValueError(f"unknown policy {name!r}")


def policy_record(name: str) -> dict:
    return {"name": name, **dataclasses.asdict(policy_for(name))}


def option_refusal(mode: str, options: dict):
    """Why these options cannot run in `mode`, or None."""
    given = [k for k in OPTION_KEYS if options.get(k) is not None]
    if not is_bounded_mode(mode):
        if given:
            return (f"{', '.join('--' + k.replace('_', '-') for k in given)} apply only to the bpb2/bpb3 "
                    f"modes, not {mode}")
        return None
    if options.get("profile", "none") != "none" or options.get("tracemalloc"):
        return "profiling and tracemalloc are not supported by the bounded modes (the search runs in a worker)"
    policy = options.get("policy") or "production"
    if policy not in POLICY_NAMES:
        return f"unknown policy {policy!r}; choose one of {', '.join(POLICY_NAMES)}"
    stall = options.get("stall")
    if stall is not None and stall not in STALLS:
        return f"unknown stall {stall!r}; choose one of {', '.join(STALLS)}"
    plans = options.get("soak_plans")
    if plans is not None and (isinstance(plans, bool) or not isinstance(plans, int)
                              or not 1 <= plans <= MAX_SOAK_PLANS):
        return f"soak_plans must be an int from 1 to {MAX_SOAK_PLANS}"
    return None


def resolve_timeout(timeout_s, policy: str, soak_plans):
    """Harness hard timeout for a bounded invocation: `(timeout_s, refusal)`.
    Default 90 s for one plan; a soak needs an explicit timeout. Always finite,
    at most 600 s, and never below the policy's request ceiling times the number
    of plans (the harness must not kill a run the executor would finish)."""
    plans = soak_plans or 1
    if timeout_s is None:
        if plans > 1:
            return None, "a soak run requires an explicit --timeout"
        timeout_s = DEFAULT_TIMEOUT_S
    if not isinstance(timeout_s, (int, float)) or isinstance(timeout_s, bool) \
            or timeout_s != timeout_s or timeout_s <= 0 or timeout_s == float("inf"):
        return None, "timeout must be a positive finite number of seconds"
    if timeout_s > MAX_TIMEOUT_S:
        return None, f"timeout {timeout_s:g}s exceeds the {MAX_TIMEOUT_S:g}s maximum"
    floor = policy_for(policy or "production").request_ceiling_s * plans
    if timeout_s < floor:
        return None, (f"timeout {timeout_s:g}s is below the {policy or 'production'} request ceiling "
                      f"x {plans} plan(s) = {floor:g}s")
    return float(timeout_s), None


# --------------------------------------------------------------------------
# Windows observation helpers (read-only; None where unavailable)
# --------------------------------------------------------------------------

if _WIN:
    import ctypes
    from ctypes import wintypes

    class _MemoryCounters(ctypes.Structure):
        _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                    ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                    ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                    ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]

    class _BasicAccounting(ctypes.Structure):
        _fields_ = [
            ("TotalUserTime", ctypes.c_int64), ("TotalKernelTime", ctypes.c_int64),
            ("ThisPeriodTotalUserTime", ctypes.c_int64), ("ThisPeriodTotalKernelTime", ctypes.c_int64),
            ("TotalPageFaultCount", wintypes.DWORD), ("TotalProcesses", wintypes.DWORD),
            ("ActiveProcesses", wintypes.DWORD), ("TotalTerminatedProcesses", wintypes.DWORD),
        ]

    _k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _psapi = ctypes.WinDLL("psapi", use_last_error=True)
    _k32.GetCurrentProcess.restype = wintypes.HANDLE
    _k32.GetProcessHandleCount.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
    _k32.GetProcessHandleCount.restype = wintypes.BOOL
    _k32.QueryInformationJobObject.argtypes = [
        wintypes.HANDLE, ctypes.c_int, wintypes.LPVOID, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD)]
    _k32.QueryInformationJobObject.restype = wintypes.BOOL
    _psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(_MemoryCounters), wintypes.DWORD]
    _psapi.GetProcessMemoryInfo.restype = wintypes.BOOL


def _memory_counters(handle):
    if not _WIN:
        return None
    counters = _MemoryCounters()
    counters.cb = ctypes.sizeof(_MemoryCounters)
    if not _psapi.GetProcessMemoryInfo(handle, ctypes.byref(counters), counters.cb):
        return None
    return counters


def process_peak_rss(proc):
    """Peak working set of a `Popen` we own, through its own handle (valid after
    exit while the handle is open), or None."""
    handle = getattr(proc, "_handle", None)
    if handle is None:
        return None
    counters = _memory_counters(int(handle))
    return None if counters is None else int(counters.PeakWorkingSetSize)


def current_rss():
    counters = _memory_counters(_k32.GetCurrentProcess()) if _WIN else None
    return None if counters is None else int(counters.WorkingSetSize)


def handle_count():
    if not _WIN:
        return None
    count = wintypes.DWORD()
    if not _k32.GetProcessHandleCount(_k32.GetCurrentProcess(), ctypes.byref(count)):
        return None
    return int(count.value)


def own_job_active_processes():
    """Active processes in the job this process belongs to (NULL handle), or
    None when this process is not in a job or the query fails."""
    if not _WIN:
        return None
    info = _BasicAccounting()
    if not _k32.QueryInformationJobObject(None, 1, ctypes.byref(info), ctypes.sizeof(info), None):
        return None
    return int(info.ActiveProcesses)


# --------------------------------------------------------------------------
# Per-plan observation
# --------------------------------------------------------------------------

class WorkerCapture:
    """Records every worker `start_worker` returns while installed. The wrapped
    function is called unchanged and its return value is passed through."""

    def __init__(self):
        self.workers = []
        self._original = None

    def install(self):
        import worker_containment as wc
        original = self._original = wc.start_worker

        def observed(*args, **kwargs):
            worker = original(*args, **kwargs)
            self.workers.append(worker)
            return worker

        wc.start_worker = observed

    def uninstall(self):
        import worker_containment as wc
        if self._original is not None:
            wc.start_worker = self._original
            self._original = None


class Sampler(threading.Thread):
    """Samples the harness job's active-process count and the captured worker's
    peak working set while a plan runs. Reads only."""

    def __init__(self, capture: WorkerCapture, interval_s: float = SAMPLE_INTERVAL_S):
        super().__init__(name="bounded-bench-sampler", daemon=True)
        self.capture = capture
        self.interval_s = interval_s
        self.max_active = None
        self.last_worker_peak = None
        self._halt = threading.Event()

    def sample(self):
        active = own_job_active_processes()
        if active is not None:
            self.max_active = active if self.max_active is None else max(self.max_active, active)
        for worker in list(self.capture.workers):
            if worker.proc.returncode is None:
                peak = process_peak_rss(worker.proc)
                if peak is not None:
                    self.last_worker_peak = peak if self.last_worker_peak is None else max(self.last_worker_peak, peak)

    def run(self):
        self.sample()
        while not self._halt.wait(self.interval_s):
            self.sample()

    def stop(self):
        self._halt.set()
        self.join(timeout=5.0)


def _leftover_tempdirs(private_tmp) -> int:
    return sum(1 for entry in Path(private_tmp).iterdir() if entry.name.startswith(TEMPDIR_PREFIX))


def _route_entry(route, search, fixture, alternatives):
    if route is None:
        return None
    from benchmarks import normalized as nz
    norm = nz.normalize_prefix_route(route, "complete", search=search, start_pose=fixture.start,
                                     alternatives=alternatives)
    return {"hash": norm["hash"], "normalized": norm, "replay_violations": None}


def worker_peak_rss(worker, sampler):
    """`(peak_bytes, source)` for a captured worker. `after_exit` only when the worker's own
    handle confirms it exited (`poll()`), so the reading is its final peak. A reading of a
    worker that is still alive, or the sampler's last reading, is kept as diagnostic data with
    source `last_sample` and never satisfies MEM's after-exit requirement."""
    exited = worker is not None and worker.proc.poll() is not None
    read = process_peak_rss(worker.proc) if worker is not None else None
    if read is not None and exited:
        return read, "after_exit"
    candidates = [v for v in (read, sampler.last_worker_peak) if v is not None]
    if candidates:
        return max(candidates), "last_sample"
    return None, None


def plan_view(outcome, planner, capture, sampler, private_tmp, fixture, alternatives, planner_mode) -> dict:
    """The recorded facts of one `plan()` call."""
    from benchmarks import normalized as nz
    search = "prefix_b3" if planner_mode == "B3" else "prefix_b2"
    retained = outcome.route if outcome.route is not None else outcome.diagnostic_route
    b2 = outcome.b2_route
    worker = capture.workers[-1] if capture.workers else None
    worker_peak, source = worker_peak_rss(worker, sampler)
    timing = dataclasses.asdict(outcome.timing)
    decided = timing.get("decided_s")
    return nz.json_safe({
        "termination": outcome.termination, "route_status": outcome.route_status,
        "route_source": outcome.route_source, "deliverable": outcome.deliverable,
        "delivery": outcome.delivery, "optimality_proven": outcome.optimality_proven,
        "complete_route_proven_impossible": outcome.complete_route_proven_impossible,
        "search_space_exhausted": outcome.search_space_exhausted,
        "incumbents_received": outcome.incumbents_received,
        "verification_failures": list(outcome.verification_failures),
        "failure_detail": outcome.failure_detail, "execution_message": outcome.execution_message,
        "planner_metadata": outcome.planner_metadata, "timing": timing,
        "overshoot_s": None if decided is None else decided - planner.policy.planning_target_s,
        "executor_cleanup": dataclasses.asdict(outcome.cleanup),
        "routes": {
            "route": _route_entry(outcome.route, search, fixture, alternatives),
            "b2_route": _route_entry(b2, "prefix_b2", fixture, alternatives),
            "diagnostic_route": _route_entry(outcome.diagnostic_route, search, fixture, alternatives),
        },
        "b3_not_worse_than_b2": (retained.total_cost <= b2.total_cost)
        if retained is not None and b2 is not None else None,
        "tree": {
            "workers_started": len(capture.workers),
            "worker_pid": worker.proc.pid if worker is not None else None,
            "worker_exit_verified": (worker.proc.poll() is not None) if worker is not None else None,
            "job_active_processes_max_during": sampler.max_active,
            "job_active_processes_after": own_job_active_processes(),
            "private_tempdir_leftovers": _leftover_tempdirs(private_tmp),
        },
        "memory": {"worker_peak_rss_bytes": worker_peak, "worker_peak_rss_source": source},
        "healthy_after": planner.healthy,
    })


def _one_plan(planner, inp, private_tmp, fixture, alternatives):
    capture = WorkerCapture()
    sampler = Sampler(capture)
    capture.install()
    sampler.start()
    try:
        r0 = time.monotonic()
        outcome = planner.plan(inp, accepted_at=r0)
    finally:
        sampler.stop()
        capture.uninstall()
    return plan_view(outcome, planner, capture, sampler, private_tmp, fixture, alternatives, inp.mode), capture


def _status(view) -> str:
    if view["termination"] == "executor_failed":
        return "error"
    return "complete" if view["deliverable"] and view["route_status"] == "complete" else "no_route"


def _result(view) -> dict:
    entry = view["routes"]["route"]
    norm = entry["normalized"] if entry else None
    return {
        "termination": view["termination"], "route_status": view["route_status"],
        "deliverable": view["deliverable"], "delivery": view["delivery"],
        "obstacle_order": norm["obstacle_order"] if norm else None,
        "total_cost": norm["total_cost"] if norm else None,
        "total_distance_cm": norm["total_distance_cm"] if norm else None,
        "n_commands": sum(len(leg["commands"]) for leg in norm["legs"]) if norm else None,
    }


def run_worker(spec: dict, *, peak_rss, redact, planner_factory=None) -> dict:
    """Run one bounded invocation in THIS process and return its raw record
    fields. `planner_factory(policy, worker_command)` is a test seam; by default
    the real `BoundedPrefixPlanner` is built."""
    import bounded_prefix_planning as bpp
    from benchmarks import fixtures as bfx

    fixture = bfx.load_fixture(spec["fixture_id"])
    alternatives = bool(spec["alternatives"])
    policy_name = spec.get("policy") or "production"
    stall = spec.get("stall")
    plans = spec.get("soak_plans") or 1
    private_tmp = spec["private_tmp"]
    os.makedirs(private_tmp, exist_ok=True)
    tempfile.tempdir = private_tmp          # every executor tempdir lands here, and only here
    worker_command = None
    if stall is not None:
        config_path = Path(private_tmp).parent / "stall_config.json"
        config_path.write_text(json.dumps(STALLS[stall]), encoding="utf-8")
        worker_command = [sys.executable, str(TEST_WORKER), "real", str(config_path)]
    policy = policy_for(policy_name)
    factory = planner_factory or (lambda p, cmd: bpp.BoundedPrefixPlanner(p, worker_command=cmd))
    planner = factory(policy, worker_command)
    planner_mode = "B3" if spec["mode"] == "bpb3" else "B2"
    inp = bpp.PlanInput(fixture.start, fixture.obstacles, planner_mode, alternatives)

    bounded = {"policy": policy_record(policy_name), "stall": stall, "soak_plans": spec.get("soak_plans"),
               "r0_anchor": R0_ANCHOR, "planner_mode": planner_mode, "plan": None, "soak": None,
               "harness": None, "replay_s": None}
    record = {"status": None, "status_detail": None, "result": None, "result_hash": None,
              "planning_time_s": None, "stats": None, "analysis": None, "leg_records": None,
              "limits": None, "validation": None, "hotspots": None, "cprofile_top": None}
    try:
        if spec.get("soak_plans") is None:
            view, _capture = _one_plan(planner, inp, private_tmp, fixture, alternatives)
            bounded["plan"] = view
            route = view["routes"]["route"]
            record.update(status=_status(view), status_detail=view["failure_detail"], result=_result(view),
                          result_hash=route["hash"] if route else None)
        else:
            soak = {"handle_count_before": handle_count(), "thread_count_before": threading.active_count(),
                    "rows": []}
            for index in range(1, plans + 1):
                view, _capture = _one_plan(planner, inp, private_tmp, fixture, alternatives)
                view.update(index=index, handle_count=handle_count(),
                            thread_count=threading.active_count(), rss_bytes=current_rss())
                soak["rows"].append(view)
            bounded["soak"] = soak
            statuses = [_status(row) for row in soak["rows"]]
            record.update(status="error" if "error" in statuses else "no_route",
                          result={"plans": plans, "terminations": [r["termination"] for r in soak["rows"]]})
    except Exception as error:   # a recorded outcome, not a crash of the harness
        record.update(status="error", status_detail=redact(f"{type(error).__name__}: {error}")[:500])
    record["peak_memory"] = {"peak_rss_bytes": peak_rss(), "tracemalloc_peak_bytes": None}
    record["bounded"] = bounded
    return record


def plan_views(bounded) -> list:
    """Every plan recorded in a `bounded` block: the single plan or the soak rows."""
    if not isinstance(bounded, dict):
        return []
    if isinstance(bounded.get("soak"), dict):
        return list(bounded["soak"].get("rows") or [])
    return [bounded["plan"]] if isinstance(bounded.get("plan"), dict) else []


def replay_routes(bounded, fixture) -> None:
    """Parent side: replay every recorded route independently (in place)."""
    from benchmarks import route_replay
    started = time.perf_counter()
    for view in plan_views(bounded):
        for key in ROUTE_KEYS:
            entry = (view.get("routes") or {}).get(key)
            if isinstance(entry, dict) and entry.get("normalized") is not None:
                entry["replay_violations"] = route_replay.replay_violations(entry["normalized"], fixture)
    bounded["replay_s"] = time.perf_counter() - started


def start_sentinel(executable=sys.executable):
    """An unrelated, harness-owned process that must survive the invocation."""
    import subprocess
    return subprocess.Popen([executable, "-c", "import time; time.sleep(900)", SENTINEL_MARKER],
                            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_sentinel(sentinel) -> dict:
    """Check the sentinel survived, then terminate THAT process through its own
    handle and verify its death."""
    import subprocess
    alive = sentinel.poll() is None
    if alive:
        sentinel.terminate()
    try:
        sentinel.wait(timeout=5.0)
        verified = sentinel.returncode is not None
    except subprocess.TimeoutExpired:
        verified = False
    return {"sentinel_pid": sentinel.pid, "sentinel_alive_after": alive,
            "sentinel_termination_verified": verified}
