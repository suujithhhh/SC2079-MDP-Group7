"""Bounded execution of the prefix-tree B2/B3 search.

`BoundedPrefixPlanner.plan()` runs `route_prefix_search` in one contained
worker process (`prefix_worker.py`) and enforces three deadlines, all measured
from `R0`, the instant the PC accepted the request (`accepted_at`, the parent's
`time.monotonic()`):

    R0 + 63 s   planning target. Worker start-up counts. The parent sends the
                worker what is left of it (`search_budget_s`) so the search
                stops starting new legs; a leg already running may continue.
    R0 + 66 s   hard stop. The whole worker process tree is terminated.
    R0 + 70 s   request ceiling for the future HTTP adapter. `plan()` targets
                R0 + 67.5 s; an outcome assembled after R0 + 68 s is withheld.

`planning_service_v2.py` is the live experimental adapter: it captures `R0`,
accepts 1-8 obstacles, calls `plan()` and serializes only a deliverable complete
route. All timing is server-owned (`ExecutionPolicy`); no request field can
reach it. The adapter's response-write margin is the gap from the 68 s delivery
deadline to the 70 s request ceiling.

Safe incumbents
---------------
The worker reports the first complete B2 route and every strictly cheaper B3
route as it finds them (`on_incumbent`), so a hard stop, a crash or a budget
stop never loses a route the parent already holds. The parent never trusts a
route: `verify_route_record()` rebuilds each leg from the claimed `(direction,
value)` commands and runs the canonical pipeline on it (command legality,
swept collision, compression, command generation, cost and goal tolerance),
and only a route that passes is retained. The returned route is complete,
verified, and no worse than every incumbent the worker reported.

Stop taxonomy (one `termination`, plus separate cleanup and delivery facts)
---------------------------------------------------------------------------
    search_finished        the planner stopped itself: exhausted / bound closed /
                           first complete route (B2)
    planner_work_limit     a deterministic planner limit
    planner_budget         the planner's between-leg checkpoint (R0 + 63 s)
    cancelled              external cancel, acknowledged or escalated to a kill
    hard_backstop          the parent terminated the worker at R0 + 66 s
    startup_overran_budget the worker was not ready in time: no search ran
    worker_failed          crash, protocol violation, or no verifiable route
    executor_failed        containment failed or the executor is unhealthy
Precedence at the decision point: a valid FINAL already received; else a dead
worker (`worker_failed`); else an external cancel; else the hard stop. A frame
counts only if the parent's reader received it before the stop instant; none
is accepted after the decision. A request-ceiling miss is a delivery fact,
never a planning outcome.

Honesty: `optimality_proven`, `complete_route_proven_impossible` and
`search_space_exhausted` can be true only for a `search_finished` termination
whose FINAL passes the parent's own consistency check. After any bounded stop
they are false, and a missing route is reported as "no complete route found
within budget", never as impossible.

Fail closed: if containment fails, or worker-tree death, job release, reader
shutdown or temporary-directory removal cannot be verified, nothing is
delivered (`deliverable=False`; a verified route is kept only as
`diagnostic_route`) and the executor becomes unhealthy: every later `plan()`
returns `executor_failed` until the process is restarted. The temporary
directory is deleted only after the tree is verified dead.

A late return is different: if `plan()` finishes after `R0 + 68 s` with
containment, death and cleanup all verified, the route is withheld
(`deliverable=False`, `delivery="request_ceiling_exceeded"`, route only in
`diagnostic_route`) but the executor stays healthy, because nothing is left
running.

Partial routes are diagnostic metadata only in this layer: the worker never
transfers partial-route geometry (`planner_metadata["route_status"]` and the
planner's message say one existed), every retained route must visit every
obstacle and pass `verify_route_record()`, and `route_status` is only ever
`"complete"` or `"none"`.

The module never enumerates or signals any process other than the one worker
it started (see `worker_containment.py`).
"""

import math
import queue
import shutil
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence

import prefix_worker as pw
import worker_containment as wc
from command_compression import CompressionInvariantError, InvalidLatticePlanError
from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from executable_route_optimizer import (
    ExecutableRouteInvariantError,
    ExecutableRouteLeg,
    _candidate_targets,
    _validate_leg_result,
    _validate_no_duplicate_obstacle_ids,
)
from models import Obstacle, Pose
from motion_primitives import (
    STRAIGHT_ACTIONS,
    InvalidCommandValueError,
    UnsupportedActionError,
    primitive_length_cm,
)
from route_lower_bound import MAX_BOUND_OBSTACLES
from route_prefix_search import (
    MODE_B2,
    MODE_B3,
    STOP_BOUND_CLOSED,
    STOP_CANCELLED,
    STOP_EXHAUSTED,
    STOP_FIRST_COMPLETE,
    STOP_WALL_CLOCK_CHECKPOINT,
    STOP_WORK_LIMIT,
    PrefixRoute,
)
from state_lattice import LatticeAction, LatticePlan, _within_goal_tolerance
from stm_command_generation import (
    STMCommandGenerationError,
    _poses_match_strict,
    _values_match_strict,
    generate_stm_command_plan,
)
from swept_collision import safe_primitive_samples

TERM_SEARCH_FINISHED = "search_finished"
TERM_WORK_LIMIT = "planner_work_limit"
TERM_PLANNER_BUDGET = "planner_budget"
TERM_CANCELLED = "cancelled"
TERM_HARD_BACKSTOP = "hard_backstop"
TERM_STARTUP_OVERRAN = "startup_overran_budget"
TERM_WORKER_FAILED = "worker_failed"
TERM_EXECUTOR_FAILED = "executor_failed"

DELIVERY_ON_TIME = "on_time"
DELIVERY_CEILING_EXCEEDED = "request_ceiling_exceeded"

SOURCE_FINAL = "final"
SOURCE_B3 = "b3_incumbent"
SOURCE_B2 = "b2_incumbent"

_STOP_TO_TERMINATION = {
    STOP_FIRST_COMPLETE: TERM_SEARCH_FINISHED,
    STOP_EXHAUSTED: TERM_SEARCH_FINISHED,
    STOP_BOUND_CLOSED: TERM_SEARCH_FINISHED,
    STOP_WORK_LIMIT: TERM_WORK_LIMIT,
    STOP_WALL_CLOCK_CHECKPOINT: TERM_PLANNER_BUDGET,
    STOP_CANCELLED: TERM_CANCELLED,
}
_CLOSED_STOPS = (STOP_EXHAUSTED, STOP_BOUND_CLOSED)

WORKER_SCRIPT = Path(pw.__file__).resolve()
POLL_S = 0.05
MAX_COMMANDS_PER_LEG = 5000
_ROUTE_STATUSES = ("complete", "partial_only", "none")


class ExecutorBusyError(RuntimeError):
    """`plan()` was called while another run is in progress. The future HTTP
    adapter's single-flight guard must prevent this."""


class RouteVerificationError(ValueError):
    """A route record failed independent verification."""


# --------------------------------------------------------------------------
# Policy and public types
# --------------------------------------------------------------------------

def _positive_finite(value) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value) and value > 0)


@dataclass(frozen=True)
class ExecutionPolicy:
    """Server-side timing. Every offset is seconds after `R0`. Immutable and
    validated; nothing in a request can construct or modify one."""

    planning_target_s: float = 63.0
    hard_stop_s: float = 66.0
    verification_deadline_s: float = 67.0
    return_target_s: float = 67.5
    return_deadline_s: float = 68.0
    request_ceiling_s: float = 70.0
    startup_margin_s: float = 0.1          # pipe transit allowance when computing the SPEC budget
    min_search_budget_s: float = 1.0       # less than this left at SPEC: run no search
    death_wait_s: float = 1.0              # verifying worker-tree death after the kill (measured ~0.01 s)
    finish_grace_s: float = 2.0            # worker's own exit after a valid FINAL
    cancel_grace_s: float = 2.0            # worker's time to acknowledge CANCEL
    tempdir_attempts: int = 3

    def __post_init__(self):
        names = ("planning_target_s", "hard_stop_s", "verification_deadline_s", "return_target_s",
                 "return_deadline_s", "request_ceiling_s", "startup_margin_s", "min_search_budget_s",
                 "death_wait_s", "finish_grace_s", "cancel_grace_s")
        for name in names:
            if not _positive_finite(getattr(self, name)):
                raise ValueError(f"{name} must be a finite positive number")
        if isinstance(self.tempdir_attempts, bool) or not isinstance(self.tempdir_attempts, int) \
                or self.tempdir_attempts < 1:
            raise ValueError("tempdir_attempts must be a positive int")
        chain = (self.planning_target_s, self.hard_stop_s, self.verification_deadline_s,
                 self.return_target_s)
        if not all(a < b for a, b in zip(chain, chain[1:])):
            raise ValueError("need planning_target < hard_stop < verification_deadline < return_target")
        if not self.return_target_s <= self.return_deadline_s < self.request_ceiling_s:
            raise ValueError("need return_target <= return_deadline < request_ceiling")
        if self.hard_stop_s + self.death_wait_s > self.verification_deadline_s:
            raise ValueError("hard_stop + death_wait must not exceed verification_deadline")
        if self.startup_margin_s + self.min_search_budget_s >= self.planning_target_s:
            raise ValueError("startup_margin + min_search_budget must be below planning_target")


PRODUCTION_POLICY = ExecutionPolicy()


@dataclass(frozen=True)
class PlanInput:
    """What a plan needs and nothing more: no timing field exists here."""

    start: Pose
    obstacles: tuple
    mode: str
    allow_alternative_viewing: bool = False

    def __post_init__(self):
        if not isinstance(self.start, Pose):
            raise TypeError("start must be a Pose")
        obstacles = tuple(self.obstacles)
        object.__setattr__(self, "obstacles", obstacles)
        if not obstacles or len(obstacles) > MAX_BOUND_OBSTACLES \
                or not all(isinstance(o, Obstacle) for o in obstacles):
            raise ValueError(f"obstacles must be 1..{MAX_BOUND_OBSTACLES} Obstacle objects")
        _validate_no_duplicate_obstacle_ids(obstacles)
        if self.mode not in (MODE_B2, MODE_B3):
            raise ValueError(f"mode must be {MODE_B2} or {MODE_B3}")
        if not isinstance(self.allow_alternative_viewing, bool):
            raise TypeError("allow_alternative_viewing must be a bool")


@dataclass(frozen=True)
class Cleanup:
    """Facts about the worker's teardown; `None` means not applicable."""

    worker_started: bool
    containment: Optional[str]
    death_verified: Optional[bool]     # direct worker exited, seen through its Popen handle
    tree_dead: Optional[bool]          # every descendant gone (job / process group)
    job_released: Optional[bool]
    tempdir_removed: Optional[bool]
    exit_code: Optional[int]
    terminate_error: bool = False
    reader_stopped: Optional[bool] = None   # frame reader ended and stdout closed

    @property
    def ok(self) -> bool:
        checked = (self.death_verified, self.tree_dead, self.job_released, self.tempdir_removed,
                   self.reader_stopped)
        return not self.terminate_error and all(v is not False for v in checked)


@dataclass(frozen=True)
class Timing:
    """Offsets in seconds after `R0`; `None` if that point was never reached."""

    hello_s: Optional[float] = None
    spec_sent_s: Optional[float] = None
    search_budget_s: Optional[float] = None       # the value sent to the worker
    first_incumbent_s: Optional[float] = None
    decided_s: Optional[float] = None
    cleaned_s: Optional[float] = None
    verified_s: Optional[float] = None
    returned_s: Optional[float] = None


@dataclass(frozen=True)
class BoundedPlanOutcome:
    """Result of one bounded run. Immutable.

    `route` is a complete, independently verified `PrefixRoute` and is set only
    when `deliverable` is true; otherwise a verified route (if any) is only in
    `diagnostic_route`. There is never a partial route: `route_status` is
    `"complete"` or `"none"`. `planner_metadata` is the worker's own FINAL
    summary (its `message` is never edited); parent wording is in
    `execution_message`.
    """

    mode: str
    termination: str
    route_status: str                       # "complete" | "none"
    route: Optional[PrefixRoute]
    route_source: Optional[str]
    b2_route: Optional[PrefixRoute]
    diagnostic_route: Optional[PrefixRoute]
    optimality_proven: bool
    complete_route_proven_impossible: bool
    search_space_exhausted: bool
    planner_metadata: Optional[dict]
    execution_message: str
    failure_detail: Optional[str]
    incumbents_received: int
    verification_failures: tuple
    deliverable: bool
    delivery: str
    cleanup: Cleanup
    timing: Timing


# --------------------------------------------------------------------------
# Record schema validation (parent side, on receipt)
# --------------------------------------------------------------------------

def _is_int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_num(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _is_pose(value) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(_is_num(v) for v in value)


_ROUTE_KEYS = frozenset({"obstacle_order", "candidate_indices", "skipped_obstacle_ids",
                         "total_cost", "total_distance_cm", "final_pose", "legs"})
_LEG_KEYS = frozenset({"obstacle_id", "target_pose", "commands", "final_pose",
                       "total_cost", "total_distance_cm"})
_FINAL_KEYS = frozenset({
    "stop_reason", "work_limit_hit", "route_status", "optimality_proven",
    "complete_route_proven_impossible", "search_space_exhausted", "lower_bound_certified",
    "bound_violation_count", "unresolved_branch_count", "min_unresolved_lower_bound",
    "unresolved_bound_known", "empty_candidate_obstacle_ids", "message", "elapsed_s",
    "leg_calls", "lattice_expansions", "nodes_expanded", "incumbent_updates", "complete_route",
})


def check_route_record_schema(record) -> None:
    """Strict shape and type check of a route record; `FrameError` on any defect."""
    bad = pw.FrameError
    if not isinstance(record, dict) or set(record) != _ROUTE_KEYS:
        raise bad("route record keys")
    order, indices, skipped, legs = (record["obstacle_order"], record["candidate_indices"],
                                     record["skipped_obstacle_ids"], record["legs"])
    if not (isinstance(order, list) and isinstance(indices, list) and isinstance(skipped, list)
            and isinstance(legs, list)):
        raise bad("route record lists")
    if not (len(order) == len(indices) == len(legs) <= MAX_BOUND_OBSTACLES):
        raise bad("route record lengths")
    if not (all(_is_int(v) for v in order) and all(_is_int(v) and v >= 0 for v in indices)
            and all(_is_int(v) for v in skipped)):
        raise bad("route record ids")
    if not (_is_num(record["total_cost"]) and _is_num(record["total_distance_cm"])
            and _is_pose(record["final_pose"])):
        raise bad("route record totals")
    for leg in legs:
        if not isinstance(leg, dict) or set(leg) != _LEG_KEYS:
            raise bad("leg keys")
        commands = leg["commands"]
        if not (_is_int(leg["obstacle_id"]) and _is_pose(leg["target_pose"])
                and _is_pose(leg["final_pose"]) and _is_num(leg["total_cost"])
                and _is_num(leg["total_distance_cm"])
                and isinstance(commands, list) and len(commands) <= MAX_COMMANDS_PER_LEG):
            raise bad("leg fields")
        for command in commands:
            if not (isinstance(command, list) and len(command) == 2
                    and isinstance(command[0], str) and _is_int(command[1])):
                raise bad("command entry")


def check_final_schema(payload) -> None:
    bad = pw.FrameError
    if not isinstance(payload, dict) or set(payload) != _FINAL_KEYS:
        raise bad("FINAL keys")
    if payload["stop_reason"] not in _STOP_TO_TERMINATION:
        raise bad("FINAL stop_reason")
    if payload["route_status"] not in _ROUTE_STATUSES:
        raise bad("FINAL route_status")
    for name in ("optimality_proven", "complete_route_proven_impossible", "search_space_exhausted",
                 "lower_bound_certified", "unresolved_bound_known"):
        if not isinstance(payload[name], bool):
            raise bad(f"FINAL {name}")
    for name in ("bound_violation_count", "unresolved_branch_count", "leg_calls",
                 "lattice_expansions", "nodes_expanded", "incumbent_updates"):
        if not _is_int(payload[name]) or payload[name] < 0:
            raise bad(f"FINAL {name}")
    if payload["work_limit_hit"] is not None and not isinstance(payload["work_limit_hit"], str):
        raise bad("FINAL work_limit_hit")
    if payload["min_unresolved_lower_bound"] is not None and not _is_num(payload["min_unresolved_lower_bound"]):
        raise bad("FINAL min_unresolved_lower_bound")
    if not (isinstance(payload["empty_candidate_obstacle_ids"], list)
            and all(_is_int(v) for v in payload["empty_candidate_obstacle_ids"])):
        raise bad("FINAL empty_candidate_obstacle_ids")
    if not isinstance(payload["message"], str) or not _is_num(payload["elapsed_s"]):
        raise bad("FINAL message/elapsed_s")
    if payload["complete_route"] is not None:
        check_route_record_schema(payload["complete_route"])
    if (payload["route_status"] == "complete") != (payload["complete_route"] is not None):
        raise bad("FINAL route_status does not match complete_route")


# --------------------------------------------------------------------------
# Independent verification (canonical production helpers only)
# --------------------------------------------------------------------------

def _deadline_check(deadline, clock):
    if deadline is not None and clock() >= deadline:
        raise RouteVerificationError("verification deadline reached")


def verify_route_record(record, inp: PlanInput, config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
                        *, deadline: Optional[float] = None,
                        clock: Callable[[], float] = time.monotonic) -> PrefixRoute:
    """Rebuild and check a claimed COMPLETE route; return it as a `PrefixRoute`.

    Nothing in `record` is trusted beyond the `(direction, value)` commands, the
    order and the candidate indices. Per leg, from the previous leg's
    `final_pose` (the search's own chaining rule), this runs the canonical
    pipeline: `primitive_length_cm` legality, `safe_primitive_samples` swept
    collision, then `generate_stm_command_plan` (compression re-validation, exact
    wire strings, distance and cost), and requires the generated commands to
    equal the claimed ones, the leg to end within goal tolerance of its
    safety-filtered candidate, and every claimed total and pose to match.
    Raises `RouteVerificationError` (or `pw.FrameError` for a malformed record).
    """
    check_route_record_schema(record)
    obstacles = inp.obstacles
    by_id = {o.obstacle_id: o for o in obstacles}
    order = record["obstacle_order"]
    if record["skipped_obstacle_ids"] or sorted(order) != sorted(by_id):
        raise RouteVerificationError("route is not complete: every obstacle must be visited once")

    candidate_cache = {}
    pose = inp.start
    legs = []
    total_cost = 0.0
    for index, leg_record in enumerate(record["legs"]):
        _deadline_check(deadline, clock)
        obstacle_id = leg_record["obstacle_id"]
        if obstacle_id != order[index]:
            raise RouteVerificationError(f"leg {index} is for obstacle {obstacle_id}, order says {order[index]}")
        if obstacle_id not in candidate_cache:
            candidate_cache[obstacle_id] = _candidate_targets(
                by_id[obstacle_id], obstacles, config, inp.allow_alternative_viewing)
        candidates = candidate_cache[obstacle_id]
        candidate_index = record["candidate_indices"][index]
        if candidate_index >= len(candidates):
            raise RouteVerificationError(f"leg {index}: candidate index {candidate_index} does not exist")
        target = candidates[candidate_index]
        if leg_record["target_pose"] != [target.x, target.y, target.heading]:
            raise RouteVerificationError(f"leg {index}: target is not the safety-filtered candidate")

        actions = []
        current = pose
        for direction, value in leg_record["commands"]:
            try:
                length = primitive_length_cm(direction, value, config)
            except (UnsupportedActionError, InvalidCommandValueError) as error:
                raise RouteVerificationError(f"leg {index}: illegal command ({direction!r}, {value!r})") from error
            samples = safe_primitive_samples(current, direction, value, obstacles, config)
            if samples is None:
                raise RouteVerificationError(f"leg {index}: MOV,{direction},{value} sweeps an unsafe pose")
            unit = "cm" if direction in STRAIGHT_ACTIONS else "deg"
            actions.append(LatticeAction(direction, value, unit, current, samples[-1], length, samples))
            current = samples[-1]
        if not actions and (leg_record["total_cost"] != 0 or leg_record["total_distance_cm"] != 0):
            raise RouteVerificationError(f"leg {index}: a zero-command leg must cost nothing")
        waypoints = (pose,) + tuple(p for a in actions for p in a.samples[1:])
        lattice_plan = LatticePlan(True, tuple(actions), waypoints, leg_record["total_distance_cm"],
                                   leg_record["total_cost"], 0, current, None)
        try:
            plan = generate_stm_command_plan(lattice_plan, obstacles, config)
            _validate_leg_result(plan)
        except (InvalidLatticePlanError, CompressionInvariantError, STMCommandGenerationError,
                ExecutableRouteInvariantError) as error:
            raise RouteVerificationError(f"leg {index}: canonical pipeline rejected it: {error}") from error
        if not plan.is_valid or [[c.direction, c.value] for c in plan.commands] != leg_record["commands"]:
            raise RouteVerificationError(f"leg {index}: commands are not the compressed canonical form")
        claimed_final = leg_record["final_pose"]
        if not _poses_match_strict(plan.final_pose, Pose(*claimed_final)):
            raise RouteVerificationError(f"leg {index}: claimed final pose disagrees with the replay")
        if not _within_goal_tolerance(plan.final_pose, target, config):
            raise RouteVerificationError(f"leg {index}: does not end within goal tolerance of its target")
        legs.append(ExecutableRouteLeg(obstacle_id, target, plan))
        total_cost += plan.total_cost
        pose = plan.final_pose

    total_distance = sum(leg.command_plan.total_distance_cm for leg in legs)
    if not _values_match_strict(total_cost, record["total_cost"]):
        raise RouteVerificationError("route total_cost disagrees with the replay")
    if not _values_match_strict(total_distance, record["total_distance_cm"]):
        raise RouteVerificationError("route total_distance_cm disagrees with the replay")
    if not _poses_match_strict(pose, Pose(*record["final_pose"])):
        raise RouteVerificationError("route final pose disagrees with the replay")
    return PrefixRoute(
        obstacle_order=tuple(order),
        candidate_indices=tuple(record["candidate_indices"]),
        target_poses=tuple(leg.target_pose for leg in legs),
        legs=tuple(legs),
        total_cost=total_cost,
        total_distance_cm=total_distance,
        final_pose=pose,
        skipped_obstacle_ids=(),
    )


# --------------------------------------------------------------------------
# The executor
# --------------------------------------------------------------------------

class _FrameReader(threading.Thread):
    """Drains the worker's stdout into a queue of `(received_at, event)` items,
    `received_at` being the planner clock when the line was read. Events:
    `("frame", kind, payload)`, `("error", detail)` (framing, size, JSON or
    sequence violation) or `("eof",)`. It stops by itself at end of stream,
    which arrives once every holder of the pipe's write end (the worker tree)
    is gone; the owner joins it before closing the pipe."""

    def __init__(self, stream, events: queue.Queue, clock: Callable[[], float]):
        super().__init__(name="prefix-frame-reader", daemon=True)
        self._stream = stream
        self._events = events
        self._clock = clock

    def _put(self, event) -> None:
        self._events.put((self._clock(), event))

    def run(self):
        expected = 0
        try:
            while True:
                line = self._stream.readline(pw.MAX_FRAME_BYTES + 1)
                if not line:
                    self._put(("eof",))
                    return
                try:
                    kind, payload = pw.decode_frame(
                        line, expected, {pw.KIND_HELLO, pw.KIND_INCUMBENT, pw.KIND_FINAL, pw.KIND_ERROR})
                except pw.FrameError as error:
                    self._put(("error", f"protocol: {error}"))
                    return
                expected += 1
                self._put(("frame", kind, payload))
        except (OSError, ValueError) as error:
            self._put(("error", f"pipe: {type(error).__name__}"))


@dataclass
class _Decision:
    termination: str
    final: Optional[dict] = None
    detail: Optional[str] = None
    from_final: bool = False


class BoundedPrefixPlanner:
    """Runs one bounded prefix search at a time. See the module docstring.

    `worker_command` and `verifier` are server-side seams for tests; nothing
    in a request can reach them."""

    def __init__(self, policy: ExecutionPolicy = PRODUCTION_POLICY, *,
                 worker_command: Optional[Sequence[str]] = None,
                 verifier: Callable = verify_route_record,
                 config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
                 clock: Callable[[], float] = time.monotonic):
        if not isinstance(policy, ExecutionPolicy):
            raise TypeError("policy must be an ExecutionPolicy")
        self.policy = policy
        self.worker_command = list(worker_command) if worker_command is not None \
            else [sys.executable, str(WORKER_SCRIPT)]
        self.verifier = verifier
        self.config = config
        self.clock = clock
        self._healthy = True
        self._busy = threading.Lock()

    @property
    def healthy(self) -> bool:
        return self._healthy

    def plan(self, inp: PlanInput, *, accepted_at: float, cancel=None) -> BoundedPlanOutcome:
        """Plan within the deadlines above. `accepted_at` is `R0` on this
        planner's clock (`time.monotonic()` by default); `cancel` is any object
        with `is_set()` (for example a `threading.Event`)."""
        if not isinstance(inp, PlanInput):
            raise TypeError("inp must be a PlanInput")
        if isinstance(accepted_at, bool) or not isinstance(accepted_at, (int, float)) \
                or not math.isfinite(accepted_at):
            raise TypeError("accepted_at must be a finite number (R0 on the planner's clock)")
        if not self._busy.acquire(blocking=False):
            raise ExecutorBusyError("a bounded plan is already running")
        try:
            return _Run(self, inp, float(accepted_at), cancel).execute()
        finally:
            self._busy.release()

    def _mark_unhealthy(self) -> None:
        self._healthy = False


class _Run:
    def __init__(self, planner: BoundedPrefixPlanner, inp: PlanInput, r0: float, cancel):
        self.planner = planner
        self.policy = planner.policy
        self.clock = planner.clock
        self.inp = inp
        self.r0 = r0
        self.cancel = cancel
        self.events: queue.Queue = queue.Queue()   # (received_at, event), in arrival order
        self._held = None                   # a dequeued item not yet eligible (see _drain)
        self.incumbents: list = []          # [(phase, record)] in arrival order
        self.timing: dict = {}
        self.worker: Optional[wc.ContainedWorker] = None
        self.reader: Optional[_FrameReader] = None
        self.tempdir: Optional[str] = None
        self._cleanup: Optional[Cleanup] = None
        self._terminate_error = False
        self._keep_tempdir = False
        self._spec_seq = 0

    # -- helpers -----------------------------------------------------------

    def _at(self) -> float:
        return self.clock() - self.r0

    def _mark(self, name: str) -> None:
        self.timing[name] = self._at()

    def _abs(self, offset_s: float) -> float:
        return self.r0 + offset_s

    def _send(self, kind: str, payload) -> bool:
        try:
            self.worker.proc.stdin.write(pw.encode_frame(self._spec_seq, kind, payload))
            self.worker.proc.stdin.flush()
            self._spec_seq += 1
            return True
        except (OSError, ValueError, pw.FrameError):
            return False

    # -- main flow ---------------------------------------------------------

    def execute(self) -> BoundedPlanOutcome:
        planner, policy = self.planner, self.policy
        if not planner.healthy:
            return self._assemble(_Decision(TERM_EXECUTOR_FAILED, detail="executor_unhealthy"),
                                  self._no_worker_cleanup())
        latest_start = self._abs(policy.planning_target_s - policy.startup_margin_s - policy.min_search_budget_s)
        if self.clock() >= latest_start:
            self._mark("decided_s")
            return self._assemble(_Decision(TERM_STARTUP_OVERRAN, detail="no time left to start a search"),
                                  self._no_worker_cleanup())
        try:
            decision = self._start_and_converse(latest_start)
        except BaseException:
            self._teardown(kill=True)
            planner._mark_unhealthy()
            raise
        self._mark("decided_s")
        cleanup = self._teardown(kill=not decision.from_final)
        return self._assemble(decision, cleanup)

    def _start_and_converse(self, latest_start: float) -> _Decision:
        self.tempdir = tempfile.mkdtemp(prefix="prefix_plan_")
        try:
            with open(Path(self.tempdir) / "stderr.log", "wb") as log:
                self.worker = wc.start_worker(self.planner.worker_command, cwd=self.tempdir, stderr=log)
        except wc.ContainmentError as error:
            self.planner._mark_unhealthy()
            if not error.worker_reaped:
                self._keep_tempdir = True     # a process we could not verify dead may still use it
            return _Decision(TERM_EXECUTOR_FAILED, detail=f"containment: {error}")
        self.reader = _FrameReader(self.worker.proc.stdout, self.events, self.clock)
        self.reader.start()

        hello = self._await_hello(latest_start)
        if hello is not None:
            return hello
        remaining = (self._abs(self.policy.planning_target_s) - self.clock()) - self.policy.startup_margin_s
        if remaining < self.policy.min_search_budget_s:
            return _Decision(TERM_STARTUP_OVERRAN, detail="worker start-up used the planning target")
        spec = {
            "mode": self.inp.mode,
            "start": [self.inp.start.x, self.inp.start.y, self.inp.start.heading],
            "obstacles": [[o.obstacle_id, o.x, o.y, o.image_face] for o in self.inp.obstacles],
            "allow_alternative_viewing": self.inp.allow_alternative_viewing,
            "search_budget_s": remaining,
        }
        if not self._send(pw.KIND_SPEC, spec):
            return _Decision(TERM_WORKER_FAILED, detail="worker stdin closed before SPEC")
        self._mark("spec_sent_s")
        self.timing["search_budget_s"] = remaining
        return self._run_loop()

    def _await_hello(self, latest_start: float) -> Optional[_Decision]:
        """`None` once a valid HELLO arrived; else the decision that ends the run."""
        while True:
            timeout = latest_start - self.clock()
            if timeout <= 0:
                return _Decision(TERM_STARTUP_OVERRAN, detail="worker was not ready in time")
            try:
                _received_at, event = self.events.get(timeout=min(POLL_S, timeout))
            except queue.Empty:
                if self.cancel is not None and self.cancel.is_set():
                    return _Decision(TERM_CANCELLED, detail="cancelled before the search started")
                continue
            if event[0] == "frame" and event[1] == pw.KIND_HELLO:
                self._mark("hello_s")
                return None
            return _Decision(TERM_WORKER_FAILED, detail=_event_detail(event))

    def _run_loop(self) -> _Decision:
        """Accept worker frames until one decides the run, or until the stop
        instant: `R0 + hard_stop_s`, or `min(cancel + cancel_grace_s, hard stop)`
        once an external cancel has been sent.

        The boundary is defined by the reader's receive time, not by when this
        loop happens to look: every frame received BEFORE the stop instant is
        accepted, in order, even if this loop only reaches it later (so a FINAL
        received just before the hard stop wins); a frame received at or after
        the stop instant is never accepted, and nothing is accepted once this
        method has returned its decision."""
        hard_at = self._abs(self.policy.hard_stop_s)
        cancel_sent_at = None
        while True:
            now = self.clock()
            if self.cancel is not None and self.cancel.is_set() and cancel_sent_at is None:
                cancel_sent_at = now
                self._send(pw.KIND_CANCEL, {})
            stop_at = hard_at if cancel_sent_at is None else \
                min(cancel_sent_at + self.policy.cancel_grace_s, hard_at)
            decision = self._drain(before=stop_at)
            if decision is not None:
                return decision
            if now >= stop_at:
                if cancel_sent_at is not None:          # external cancel precedes the hard stop
                    return _Decision(TERM_CANCELLED, detail="cancel_escalated")
                return _Decision(TERM_HARD_BACKSTOP)
            if self._held is None:
                try:
                    self._held = self.events.get(timeout=max(0.0, min(POLL_S, stop_at - now)))
                except queue.Empty:
                    pass

    def _drain(self, before: float) -> Optional[_Decision]:
        """Accept, in arrival order, every queued event received before `before`.
        The first one received later is held back (never accepted by this run
        unless a later stop instant allows it); returns a deciding event's
        decision, else None."""
        while True:
            if self._held is not None:
                item, self._held = self._held, None
            else:
                try:
                    item = self.events.get_nowait()
                except queue.Empty:
                    return None
            received_at, event = item
            if received_at >= before:
                self._held = item
                return None
            decision = self._accept(event)
            if decision is not None:
                return decision

    def _accept(self, event) -> Optional[_Decision]:
        """Apply one worker event. `None` keeps the run going."""
        if event[0] != "frame":
            return _Decision(TERM_WORKER_FAILED, detail=_event_detail(event))
        _tag, kind, payload = event
        try:
            if kind == pw.KIND_INCUMBENT:
                self._accept_incumbent(payload)
                return None
            if kind == pw.KIND_FINAL:
                check_final_schema(payload)
                termination = _STOP_TO_TERMINATION[payload["stop_reason"]]
                return _Decision(termination, final=payload, from_final=True)
            if kind == pw.KIND_ERROR:
                name = payload.get("error") if isinstance(payload, dict) else None
                safe = name if isinstance(name, str) and name.isidentifier() and len(name) <= 64 else "?"
                return _Decision(TERM_WORKER_FAILED, detail=f"worker_error: {safe}")
        except pw.FrameError as error:
            return _Decision(TERM_WORKER_FAILED, detail=f"protocol: {error}")
        return _Decision(TERM_WORKER_FAILED, detail=f"protocol: unexpected {kind} frame")

    def _accept_incumbent(self, payload) -> None:
        if not isinstance(payload, dict) or set(payload) != {"phase", "route"}:
            raise pw.FrameError("INCUMBENT payload")
        phase, record = payload["phase"], payload["route"]
        expected = "b2" if not self.incumbents else "b3"
        if phase != expected:
            raise pw.FrameError(f"INCUMBENT phase {phase!r}, expected {expected!r}")
        check_route_record_schema(record)
        if record["skipped_obstacle_ids"]:
            raise pw.FrameError("INCUMBENT is not a complete route")
        if self.incumbents and not record["total_cost"] < self.incumbents[-1][1]["total_cost"]:
            raise pw.FrameError("INCUMBENT is not strictly cheaper than the previous one")
        if not self.incumbents:
            self._mark("first_incumbent_s")
        self.incumbents.append((phase, record))

    # -- teardown ----------------------------------------------------------

    def _no_worker_cleanup(self) -> Cleanup:
        if self.tempdir is None:
            removed = None
        elif self._keep_tempdir:
            removed = False
        else:
            removed = self._remove_tempdir()
        return Cleanup(False, None, None, None, None, removed, None)

    def _teardown(self, kill: bool) -> Cleanup:
        """Idempotent. In order:

        1. unless a valid FINAL was accepted (`kill=False`), or if the worker
           does not then exit within `finish_grace_s`, the decision stands and
           the tree is killed; after a FINAL the direct worker is only observed
           exiting (never reaped first, see `await_exit`);
        2. the whole tree is ALWAYS terminated, so a descendant the worker left
           behind cannot survive even a graceful finish;
        3. the direct worker is reaped through its handle and the tree is
           verified empty (job process count / process group);
        4. our end of stdin is closed; only after verified tree death is the
           reader joined (it ends at end-of-stream) and stdout closed, so no
           close can block on a reader still waiting for a live writer;
        5. the containment unit is released and, only after verified death, the
           temporary directory is removed and checked absent.
        Anything unverified leaves the corresponding `Cleanup` field False, and
        the run fails closed (`_assemble`)."""
        if self._cleanup is not None:
            return self._cleanup
        policy = self.policy
        worker = self.worker
        if worker is None:
            self._cleanup = self._no_worker_cleanup()
            return self._cleanup
        if not kill:
            worker.await_exit(policy.finish_grace_s)
        try:
            worker.terminate_tree()
        except OSError:
            self._terminate_error = True
        started = self.clock()
        death = worker.wait_dead(policy.death_wait_s)
        tree = worker.tree_dead(max(0.0, policy.death_wait_s - (self.clock() - started))) if death else False
        try:
            worker.proc.stdin.close()
        except (OSError, ValueError):
            pass
        reader_stopped = False
        if tree:
            if self.reader is not None:
                self.reader.join(timeout=policy.finish_grace_s)
            reader_stopped = self.reader is None or not self.reader.is_alive()
            if reader_stopped:
                worker.close_pipes()
        released = worker.close()
        removed = self._remove_tempdir() if (death and tree) else False
        self._cleanup = Cleanup(True, worker.containment, death, tree, released, removed,
                                worker.proc.returncode, self._terminate_error, reader_stopped)
        self._mark("cleaned_s")
        return self._cleanup

    def _remove_tempdir(self) -> bool:
        path = self.tempdir
        if path is None:
            return True
        for attempt in range(self.policy.tempdir_attempts):
            shutil.rmtree(path, ignore_errors=True)
            if not Path(path).exists():
                return True
            time.sleep(0.1 * (attempt + 1))
        return not Path(path).exists()

    # -- outcome -----------------------------------------------------------

    def _verify_all(self, decision: _Decision):
        """`(retained, source, b2_route, failures)`: the best independently
        verified complete route, newest incumbent first."""
        deadline = self._abs(self.policy.verification_deadline_s)
        verify = self.planner.verifier
        candidates = []
        if decision.final is not None and decision.final["complete_route"] is not None:
            candidates.append((SOURCE_FINAL, decision.final["complete_route"]))
        for phase, record in reversed(self.incumbents):
            candidates.append((SOURCE_B3 if phase == "b3" else SOURCE_B2, record))
        failures = []

        def attempt(source, record):
            try:
                return verify(record, self.inp, self.planner.config, deadline=deadline, clock=self.clock)
            except (RouteVerificationError, pw.FrameError) as error:
                failures.append(f"{source}: {error}")
                return None

        retained = source = None
        for candidate_source, record in candidates:
            if self.clock() >= deadline:
                failures.append("verification deadline reached")
                break
            retained = attempt(candidate_source, record)
            if retained is not None:
                source = candidate_source
                break
        if retained is not None and self.incumbents:
            floor = min(record["total_cost"] for _phase, record in self.incumbents)
            if retained.total_cost > floor and not _values_match_strict(retained.total_cost, floor):
                failures.append("retained route is costlier than an incumbent the worker reported")
                retained = source = None
        b2_route = None
        if retained is not None and self.incumbents:
            if source == SOURCE_B2:
                b2_route = retained
            elif self.clock() < deadline:
                b2_route = attempt(SOURCE_B2, self.incumbents[0][1])
        return retained, source, b2_route, tuple(failures)

    def _assemble(self, decision: _Decision, cleanup: Cleanup) -> BoundedPlanOutcome:
        retained, source, b2_route, failures = self._verify_all(decision)
        self._mark("verified_s")
        termination, detail = decision.termination, decision.detail
        claimed_route = bool(self.incumbents) or (
            decision.final is not None and decision.final["complete_route"] is not None)
        if claimed_route and retained is None and termination != TERM_EXECUTOR_FAILED:
            termination = TERM_WORKER_FAILED
            detail = detail or "route_verification_failed"
        proven, impossible, exhausted = _honest_flags(self.inp.mode, termination, decision.final,
                                                      retained, source)

        cleanup_ok = cleanup.ok
        if not cleanup_ok:
            self.planner._mark_unhealthy()
            proven = impossible = exhausted = False
        self._mark("returned_s")
        on_time = self.timing["returned_s"] <= self.policy.return_deadline_s
        deliverable = retained is not None and cleanup_ok and on_time
        if not deliverable:
            proven = False           # nothing is delivered, so nothing may claim a proof
        route = retained if deliverable else None
        return BoundedPlanOutcome(
            mode=self.inp.mode,
            termination=termination,
            route_status="complete" if route is not None else "none",
            route=route,
            route_source=source if route is not None else None,
            b2_route=b2_route if deliverable else None,
            diagnostic_route=retained if not deliverable else None,
            optimality_proven=proven,
            complete_route_proven_impossible=impossible,
            search_space_exhausted=exhausted,
            planner_metadata=_planner_metadata(decision.final),
            execution_message=_message(termination, route, proven, impossible,
                                       cleanup_ok, on_time, retained is not None),
            failure_detail=detail,
            incumbents_received=len(self.incumbents),
            verification_failures=failures,
            deliverable=deliverable,
            delivery=DELIVERY_ON_TIME if on_time else DELIVERY_CEILING_EXCEEDED,
            cleanup=cleanup,
            timing=Timing(**{k: self.timing.get(k) for k in Timing.__dataclass_fields__}),
        )


def _event_detail(event) -> str:
    if event[0] == "eof":
        return "worker closed its output without a FINAL frame"
    if event[0] == "error":
        return event[1]
    return f"protocol: unexpected {event[1]} frame"


def _honest_flags(mode, termination, final, route, source):
    """`(optimality_proven, complete_route_proven_impossible, search_space_exhausted)`,
    true only for a valid FINAL that ended `search_finished` and passes this
    check; every bounded stop reports all three false."""
    if final is None or termination != TERM_SEARCH_FINISHED:
        return False, False, False
    closed = final["stop_reason"] in _CLOSED_STOPS
    unresolved = final["unresolved_branch_count"]
    exhausted = closed and unresolved == 0
    optimal = bool(
        final["optimality_proven"] and mode == MODE_B3 and route is not None and source == SOURCE_FINAL
        and closed and final["lower_bound_certified"] and final["bound_violation_count"] == 0
        and (unresolved == 0 or (
            final["unresolved_bound_known"] and final["min_unresolved_lower_bound"] is not None
            and final["min_unresolved_lower_bound"] >= route.total_cost))
    )
    impossible = bool(
        final["complete_route_proven_impossible"] and route is None
        and (bool(final["empty_candidate_obstacle_ids"])
             or (final["stop_reason"] == STOP_EXHAUSTED and unresolved == 0))
    )
    return optimal, impossible, exhausted


def _planner_metadata(final) -> Optional[dict]:
    if final is None:
        return None
    return {k: v for k, v in final.items() if k != "complete_route"}


def _message(termination, route, proven, impossible, cleanup_ok, on_time, had_route) -> str:
    if not cleanup_ok:
        return ("Worker cleanup could not be verified; nothing is delivered and the executor is "
                "unhealthy until restart.")
    if had_route and not on_time:
        return "The verified route was ready after the response deadline and is not delivered."
    stops = {
        TERM_HARD_BACKSTOP: "hard backstop",
        TERM_PLANNER_BUDGET: "planning budget",
        TERM_WORK_LIMIT: "planner work limit",
        TERM_CANCELLED: "cancellation",
        TERM_STARTUP_OVERRAN: "worker start-up overran the planning budget",
    }
    if route is not None:
        text = "Best complete verified route found"
        if termination in stops:
            text += f" within budget ({stops[termination]})"
        text += ". Optimality proven." if proven else ". Optimality not proven."
        return text
    if impossible:
        return "No complete route exists under the supplied viewing candidates (search exhausted)."
    if termination in stops:
        return (f"No complete route found within budget ({stops[termination]}); "
                "non-existence is not proven.")
    if termination == TERM_WORKER_FAILED:
        return "The planner worker failed and no verified complete route is available."
    if termination == TERM_EXECUTOR_FAILED:
        return "The bounded executor could not run a search."
    return "No complete route found; non-existence is not proven."
