"""Standalone prefix-tree B2/B3 executable-route search.

A scalable replacement *candidate* for the permutation search in
`executable_route_optimizer.py` (ero). It searches obstacle-order
**prefixes** rather than complete permutations, guided (and, when the bound
is certified, pruned) by the admissible remaining-route lower bound in
`route_lower_bound.py`. It is **not** live: ero, `planning_service_v2.py`
and `server.py` do not import it, `/plan/v2` is unchanged, and the service
still accepts only 1-5 obstacles. Nothing here is a Task 1 performance
claim.

Legs
----
Every leg is planned by the injected `leg_planner` (default
`plan_executable_leg`: state lattice over all six STM actions, then
compression and command generation), from the **exact estimated final
pose** of the previous leg, never from the ideal target. Viewing-pose
candidates come from ero's `_candidate_targets` (safety-filtered by
`footprint.pose_is_safe()`); a candidate index always means the index in
that *safety-filtered* tuple, so index 0 is the preferred surviving
candidate, which is the physical primary only when the primary survived
filtering (`filtered_primary_obstacle_ids` lists the rest).

B2 (`find_prefix_b2_route`)
---------------------------
Deterministic, lower-bound-guided, lazy depth-first search. At every node
the children are ordered by `(candidate rank, g + h, input order)`: every
remaining obstacle's rank-0 candidate before any rank-1 candidate, and so
on. Each attempted child's *entire* downstream subtree is explored before
control returns to its node, so a valid rank-0 leg followed by a dead end
never rules out that obstacle's alternatives - they are tried later from
the same node. B2 stops at its first complete route and never prunes by
bound, so its non-existence proofs never depend on the bound.

B3 (`optimise_prefix_b3_route`)
-------------------------------
Runs B2 first; its route is the incumbent and is always returned as
`b2_route`. Then a fresh lazy best-first branch-and-bound from the root:
children are pushed as *deferred* entries (bound only, no leg planned) and
a leg is planned only when its entry is popped. The incumbent is replaced
only by a strictly cheaper complete route, so B3 is never worse than B2.

Lower bound and proof
---------------------
`bound_factory` (default `build_remaining_route_bound`) is called once per
call. Only the default factory returning a genuine, matching
`RemainingRouteBound` is *certified*. Pruning is `g + h >= incumbent` with
the module's own epsilon (applied once inside `route_lower_bound.py`; no
further epsilon here), and only while the bound is certified and no audit
violation has occurred. An uncertified bound only orders work. Every valid
leg is audited against the certified single-edge bound and endpoint region;
a violation disables all later pruning and turns every earlier bound-pruned
branch root into an unresolved one for proof purposes.

Leg failures never feed the bound: `no_path`/`unsafe_start`/`unsafe_goal`
close only that exact edge (exact start pose to that candidate);
`expansion_limit`, unknown reasons and unplanned legs are unresolved, never
`+inf`. `+inf` is used only for an obstacle with no safe candidate.

Limits
------
`SearchLimits` holds deterministic work limits (leg calls, lattice
expansions, node expansions, frontier size) and an optional wall-clock
limit read from an injected clock. The wall-clock limit is checked only at
checkpoints between legs: a running leg is never interrupted, so it can be
exceeded by one leg's duration. Nothing here comes from HTTP or a client. The optional `on_incumbent` observer
reports each complete incumbent as it is found; `bounded_prefix_planning.py`
uses it to keep the best route outside a terminable worker process.

When several stop conditions apply at the same checkpoint the reported stop
follows a fixed precedence: work limits first (leg calls, lattice expansions,
nodes, frontier), then cancellation, then the wall-clock checkpoint limit.
Precedence only picks the reason; it never moves the point where the search
stops.

`total_cost` is ero's summed planning cost, never time.
"""

import heapq
import math
import time
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Protocol, Sequence

from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from executable_route_optimizer import (
    ExecutableRouteInvariantError,
    ExecutableRouteLeg,
    LegPlanner,
    _candidate_targets,
    _validate_leg_result,
    _validate_no_duplicate_obstacle_ids,
    plan_executable_leg,
)
from models import Obstacle, Pose
from route_lower_bound import (
    MAX_BOUND_OBSTACLES,
    RemainingRouteBound,
    build_remaining_route_bound,
    lower_bound_model_from_config,
)
from state_lattice import REASON_NO_PATH, REASON_UNSAFE_GOAL, REASON_UNSAFE_START
from targets import viewing_target

MODE_B2 = "B2"
MODE_B3 = "B3"

ROUTE_COMPLETE = "complete"
ROUTE_PARTIAL_ONLY = "partial_only"
ROUTE_NONE = "none"

STOP_FIRST_COMPLETE = "first_complete_route"
STOP_EXHAUSTED = "exhausted"
STOP_BOUND_CLOSED = "bound_closed"
STOP_WORK_LIMIT = "work_limit"
STOP_WALL_CLOCK_CHECKPOINT = "wall_clock_checkpoint_limit"
STOP_CANCELLED = "cancelled"

LIMIT_LEG_CALLS = "max_leg_calls"
LIMIT_LATTICE_EXPANSIONS = "max_lattice_expansions"
LIMIT_NODES = "max_nodes"
LIMIT_FRONTIER = "max_frontier_entries"

DEFINITIVE_LEG_FAILURES = frozenset({REASON_NO_PATH, REASON_UNSAFE_START, REASON_UNSAFE_GOAL})

_CLOSED_STOPS = (STOP_EXHAUSTED, STOP_BOUND_CLOSED)
_KIND_NODE = 0
_KIND_DEFERRED = 1


class InvalidPrefixSearchInputError(ValueError):
    """Malformed input to this module's public API, or a malformed bound."""


class BoundLike(Protocol):
    """Minimal interface of a remaining-route bound (see `route_lower_bound`)."""

    obstacle_ids: tuple
    candidates: tuple

    def lower_bound(self, current_pose: Pose, remaining_ids: Iterable[int]) -> float: ...

    def child_lower_bound(self, current_pose: Pose, obstacle_id: int,
                          candidate_index: int, remaining_ids: Iterable[int]) -> float: ...


BoundFactory = Callable[[Mapping, PlannerConfig], BoundLike]


def _is_positive_int(value) -> bool:
    return not isinstance(value, bool) and isinstance(value, int) and value > 0


@dataclass(frozen=True)
class SearchLimits:
    """Server-side search limits. `None` means unlimited. Immutable.

    The four work limits are deterministic and machine-independent; each is
    checked before the next unit of work is issued (a leg that is already
    running always finishes, so `max_lattice_expansions` can be exceeded by
    at most one leg's own expansion count). `wall_clock_checkpoint_limit_s`
    is compared with the injected clock only at checkpoints between legs,
    so it can be exceeded by one leg's full duration. If several stop
    conditions apply together, the reported one is the first of: leg calls,
    lattice expansions, nodes, frontier, cancellation, wall-clock limit.
    """

    max_leg_calls: Optional[int] = None
    max_lattice_expansions: Optional[int] = None
    max_nodes: Optional[int] = None
    max_frontier_entries: int = 200_000      # provisional memory guard; to be profiled
    wall_clock_checkpoint_limit_s: Optional[float] = None

    def __post_init__(self):
        for name in ("max_leg_calls", "max_lattice_expansions", "max_nodes"):
            value = getattr(self, name)
            if value is not None and not _is_positive_int(value):
                raise InvalidPrefixSearchInputError(f"{name} must be None or a positive int, got {value!r}")
        if not _is_positive_int(self.max_frontier_entries):
            raise InvalidPrefixSearchInputError(
                f"max_frontier_entries must be a positive int, got {self.max_frontier_entries!r}"
            )
        value = self.wall_clock_checkpoint_limit_s
        if value is not None and (
            isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0
        ):
            raise InvalidPrefixSearchInputError(
                f"wall_clock_checkpoint_limit_s must be None or finite and > 0, got {value!r}"
            )


@dataclass(frozen=True)
class SearchWork:
    """Deterministic work counters, cumulative over the whole call (B2 + B3).

    `frontier_peak` is the B3 heap peak (0 when no B3 phase ran).
    `unique_prefixes_reached`/`unique_complete_orders` count distinct obstacle
    sequences (candidates ignored); `complete_routes_found` counts distinct
    candidate-level complete paths; `complete_leaf_discoveries` counts every
    discovery, including B3 re-walking B2's route. `leg_outcome_counts`
    covers leg-planner calls (cache misses) only.
    """

    leg_requests: int
    cache_hits: int
    leg_calls: int
    lattice_expansions: int
    nodes_expanded: int
    children_generated: int
    bound_evaluations: int
    bound_prunes: int
    dominance_prunes: int
    frontier_peak: int
    unique_prefixes_reached: int
    unique_complete_orders: int
    complete_routes_found: int
    complete_leaf_discoveries: int
    incumbent_updates: int
    leg_outcome_counts: tuple


@dataclass(frozen=True)
class PrefixRoute:
    """One (complete or partial) route. Immutable; legs share the cached
    `STMCommandPlan` objects and are never copied or mutated."""

    obstacle_order: tuple
    candidate_indices: tuple      # indices into each obstacle's safety-filtered tuple
    target_poses: tuple
    legs: tuple                   # tuple[ExecutableRouteLeg, ...]
    total_cost: float
    total_distance_cm: float
    final_pose: Pose              # estimated, never measured
    skipped_obstacle_ids: tuple

    @property
    def stm_commands(self) -> tuple:
        return tuple(
            command.stm_command for leg in self.legs for command in leg.command_plan.commands
        )


@dataclass(frozen=True)
class PrefixSearchResult:
    """Result of `find_prefix_b2_route`/`optimise_prefix_b3_route`. Immutable.

    `work` is cumulative over B2 and B3; `b2_work` is the exact snapshot at
    the B2/B3 phase boundary (B3 results only). `elapsed_s` comes from the
    injected clock and is kept apart from the deterministic `work`. The
    unresolved-branch fields describe the final phase only (B3's own
    traversal when it ran). `search_space_exhausted` refers to the searched
    obstacles (those with at least one safe candidate).
    """

    mode: str
    route_status: str
    complete_route: Optional[PrefixRoute]
    best_partial_route: Optional[PrefixRoute]
    b2_route: Optional[PrefixRoute]
    stop_reason: str
    work_limit_hit: Optional[str]
    search_space_exhausted: bool
    complete_route_proven_impossible: bool
    optimality_proven: bool
    empty_candidate_obstacle_ids: tuple
    filtered_primary_obstacle_ids: tuple
    unresolved_branch_count: int
    min_unresolved_lower_bound: Optional[float]
    unresolved_bound_known: bool
    lower_bound_certified: bool
    bound_violation_count: int
    bound_pruning_active: bool
    root_lower_bound: Optional[float]
    message: str
    work: SearchWork
    b2_work: Optional[SearchWork]
    elapsed_s: float


# --------------------------------------------------------------------------
# Small decision seams (module-level so tests can prove each guarantee is
# exercised by patching them). Not configuration.
# --------------------------------------------------------------------------

def _ordered_children(children: list) -> list:
    """B2 child order: candidate rank, then g + h, then input order."""
    return sorted(children, key=lambda child: (child[1], child[2], child[0]))


def _is_definitive_failure(reason) -> bool:
    return reason in DEFINITIVE_LEG_FAILURES


def _improves_incumbent(g: float, incumbent_cost: float) -> bool:
    return g < incumbent_cost


def _should_prune(f: float, incumbent_cost: float) -> bool:
    return f >= incumbent_cost


def _pruning_allowed(certified: bool, violation_count: int) -> bool:
    return certified and violation_count == 0


def _finalise_flags(mode: str, complete: bool, stop_reason: str, certified: bool,
                    violation_count: int, unresolved_fs: Sequence,
                    incumbent_cost: Optional[float], empty_ids: tuple) -> tuple:
    """Proof flags from the final phase state. Returns `(search_space_exhausted,
    complete_route_proven_impossible, optimality_proven,
    min_unresolved_lower_bound, unresolved_bound_known)`."""
    closed = stop_reason in _CLOSED_STOPS
    unresolved = len(unresolved_fs)
    bound_known = unresolved == 0 or (
        certified and violation_count == 0 and all(f is not None for f in unresolved_fs)
    )
    min_unresolved = min(unresolved_fs) if (unresolved and bound_known) else None
    exhausted = closed and unresolved == 0
    impossible = bool(empty_ids) or (
        not complete and stop_reason == STOP_EXHAUSTED and unresolved == 0
    )
    optimal = (
        mode == MODE_B3 and complete and closed and certified and violation_count == 0
        and (unresolved == 0 or (bound_known and min_unresolved >= incumbent_cost))
    )
    return exhausted, impossible, optimal, min_unresolved, bound_known


def _reopen_bound_prunes(phase: "_Phase") -> None:
    """After an audit violation, earlier bound prunes no longer count as closed."""
    for path_key in phase.bound_pruned:
        phase.unresolved[path_key] = None
    phase.bound_pruned.clear()


# --------------------------------------------------------------------------
# Internal state
# --------------------------------------------------------------------------

class _Node:
    __slots__ = ("pose", "g", "mask", "depth", "parent", "leg", "obstacle_id",
                 "active_index", "cand_index", "target", "order_code", "path_key")

    def __init__(self, pose, g, mask, depth, parent, leg, obstacle_id, active_index,
                 cand_index, target, order_code, path_key):
        self.pose = pose
        self.g = g
        self.mask = mask
        self.depth = depth
        self.parent = parent
        self.leg = leg
        self.obstacle_id = obstacle_id
        self.active_index = active_index
        self.cand_index = cand_index
        self.target = target
        self.order_code = order_code
        self.path_key = path_key


class _Phase:
    """Per-phase search state (B2's DFS, or B3's own fresh traversal)."""

    __slots__ = ("unresolved", "bound_pruned", "stop_reason", "work_limit_hit")

    def __init__(self):
        self.unresolved: dict = {}      # path_key -> certified f, or None
        self.bound_pruned: list = []    # path_keys closed by a certified bound
        self.stop_reason: Optional[str] = None
        self.work_limit_hit: Optional[str] = None


class _Counters:
    def __init__(self):
        self.leg_requests = 0
        self.cache_hits = 0
        self.leg_calls = 0
        self.lattice_expansions = 0
        self.nodes_expanded = 0
        self.children_generated = 0
        self.bound_evaluations = 0
        self.bound_prunes = 0
        self.dominance_prunes = 0
        self.frontier_peak = 0
        self.complete_leaf_discoveries = 0
        self.incumbent_updates = 0
        self.outcomes: dict = {}
        self.prefix_codes: set = set()
        self.complete_codes: set = set()
        self.complete_paths: set = set()

    def snapshot(self) -> SearchWork:
        return SearchWork(
            leg_requests=self.leg_requests,
            cache_hits=self.cache_hits,
            leg_calls=self.leg_calls,
            lattice_expansions=self.lattice_expansions,
            nodes_expanded=self.nodes_expanded,
            children_generated=self.children_generated,
            bound_evaluations=self.bound_evaluations,
            bound_prunes=self.bound_prunes,
            dominance_prunes=self.dominance_prunes,
            frontier_peak=self.frontier_peak,
            unique_prefixes_reached=len(self.prefix_codes),
            unique_complete_orders=len(self.complete_codes),
            complete_routes_found=len(self.complete_paths),
            complete_leaf_discoveries=self.complete_leaf_discoveries,
            incumbent_updates=self.incumbent_updates,
            leg_outcome_counts=tuple(sorted(self.outcomes.items())),
        )


def _bits(mask: int):
    index = 0
    while mask:
        if mask & 1:
            yield index
        mask >>= 1
        index += 1


def _check_bound_value(value) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or math.isnan(value) or value < 0):
        raise InvalidPrefixSearchInputError(
            f"bound returned {value!r}; expected a non-negative real or +inf"
        )
    return float(value)


class _Search:
    """State of one top-level call."""

    def __init__(self, mode, start_pose, obstacles, config, leg_planner,
                 allow_alternative_viewing, limits, clock, cancel_requested, bound_factory,
                 on_incumbent=None):
        if on_incumbent is not None and not callable(on_incumbent):
            raise InvalidPrefixSearchInputError("on_incumbent must be callable or None")
        if not isinstance(start_pose, Pose):
            raise InvalidPrefixSearchInputError(f"start_pose must be a Pose, got {start_pose!r}")
        if not isinstance(limits, SearchLimits):
            raise InvalidPrefixSearchInputError(f"limits must be a SearchLimits, got {limits!r}")
        if not callable(clock):
            raise InvalidPrefixSearchInputError("clock must be callable")
        if cancel_requested is not None and not callable(cancel_requested):
            raise InvalidPrefixSearchInputError("cancel_requested must be callable or None")
        if not callable(bound_factory):
            raise InvalidPrefixSearchInputError("bound_factory must be callable")
        if not callable(leg_planner):
            raise InvalidPrefixSearchInputError("leg_planner must be callable")

        self.mode = mode
        self.start_pose = start_pose
        self.obstacles = tuple(obstacles)
        if len(self.obstacles) > MAX_BOUND_OBSTACLES:
            raise InvalidPrefixSearchInputError(
                f"at most {MAX_BOUND_OBSTACLES} obstacles supported, got {len(self.obstacles)}"
            )
        _validate_no_duplicate_obstacle_ids(self.obstacles)
        self.config = config
        self.leg_planner = leg_planner
        self.limits = limits
        self.clock = clock
        self.cancel_requested = cancel_requested
        self.on_incumbent = on_incumbent
        self.t0 = clock()

        candidates = {
            o.obstacle_id: _candidate_targets(o, self.obstacles, config, allow_alternative_viewing)
            for o in self.obstacles
        }
        self.empty_ids = tuple(sorted(i for i, c in candidates.items() if not c))
        self.filtered_primary_ids = tuple(sorted(
            o.obstacle_id for o in self.obstacles
            if candidates[o.obstacle_id]
            and candidates[o.obstacle_id][0] != viewing_target(o, config)
        ))
        self.active = tuple(o for o in self.obstacles if candidates[o.obstacle_id])
        self.active_ids = tuple(o.obstacle_id for o in self.active)
        self.active_cands = tuple(tuple(candidates[i]) for i in self.active_ids)
        self.full_mask = (1 << len(self.active)) - 1
        self.code_base = len(self.active) + 1
        self._remaining_cache: dict = {}

        self.bound = None
        self.certified = False
        if self.active:
            self.bound = self._build_bound(bound_factory)
        else:
            # Nothing to search: no bound is built. The trivial zero bound is
            # exact, so certification follows the factory identity alone.
            self.certified = bound_factory is build_remaining_route_bound

        self.cache: dict = {}
        self.audited: dict = {}
        self.violations = 0
        self.counters = _Counters()
        self.phase: Optional[_Phase] = None
        self.best_partial: Optional[_Node] = None

    # -- bound ------------------------------------------------------------

    def _build_bound(self, bound_factory):
        supplied = {oid: poses for oid, poses in zip(self.active_ids, self.active_cands)}
        bound = bound_factory(dict(supplied), self.config)
        for name in ("lower_bound", "child_lower_bound"):
            if not callable(getattr(bound, name, None)):
                raise InvalidPrefixSearchInputError(f"bound has no callable {name}()")
        try:
            ids = tuple(bound.obstacle_ids)
            cands = tuple(tuple(group) for group in bound.candidates)
        except (AttributeError, TypeError) as exc:
            raise InvalidPrefixSearchInputError(f"bound lacks obstacle_ids/candidates: {exc}") from None
        expected_ids = tuple(sorted(self.active_ids))
        if ids != expected_ids:
            raise InvalidPrefixSearchInputError(
                f"bound obstacle_ids {ids!r} do not match the searched obstacles {expected_ids!r}"
            )
        expected_cands = tuple(supplied[oid] for oid in expected_ids)
        if cands != expected_cands:
            raise InvalidPrefixSearchInputError("bound candidates do not match the supplied candidates")
        self.certified = (
            bound_factory is build_remaining_route_bound
            and type(bound) is RemainingRouteBound
            and bound.model == lower_bound_model_from_config(self.config)
        )
        return bound

    def _remaining(self, mask: int) -> tuple:
        ids = self._remaining_cache.get(mask)
        if ids is None:
            ids = tuple(self.active_ids[i] for i in _bits(mask))
            self._remaining_cache[mask] = ids
        return ids

    def _lb(self, pose: Pose, mask: int) -> float:
        self.counters.bound_evaluations += 1
        return _check_bound_value(self.bound.lower_bound(pose, self._remaining(mask)))

    def _child_lb(self, pose: Pose, active_index: int, cand_index: int, mask: int) -> float:
        self.counters.bound_evaluations += 1
        return _check_bound_value(self.bound.child_lower_bound(
            pose, self.active_ids[active_index], cand_index, self._remaining(mask)
        ))

    def _pruning(self) -> bool:
        return _pruning_allowed(self.certified, self.violations)

    def _certified_f(self, f: float) -> Optional[float]:
        return f if self._pruning() else None

    # -- checkpoints ------------------------------------------------------

    def _stop(self, reason: str, limit: Optional[str] = None) -> str:
        self.phase.stop_reason = reason
        self.phase.work_limit_hit = limit
        return reason

    # Stop precedence when several conditions apply at the same checkpoint:
    # 1. deterministic work limits, in the order leg calls, lattice
    #    expansions, nodes, frontier;  2. cancellation;  3. the wall-clock
    #    checkpoint limit. A running leg is never interrupted, and cancellation
    #    and the wall clock are read only between legs. Precedence decides
    #    which reason is reported; it never makes the search stop earlier or
    #    later than the conditions themselves would.

    def _leg_limit_reached(self) -> Optional[str]:
        limits, counters = self.limits, self.counters
        if limits.max_leg_calls is not None and counters.leg_calls >= limits.max_leg_calls:
            return LIMIT_LEG_CALLS
        if (limits.max_lattice_expansions is not None
                and counters.lattice_expansions >= limits.max_lattice_expansions):
            return LIMIT_LATTICE_EXPANSIONS
        return None

    def _work_limit_reached(self, frontier_size: Optional[int] = None) -> Optional[str]:
        """First reached work limit in precedence order, or None. The frontier
        limit applies only when the caller supplies the B3 heap size."""
        limit = self._leg_limit_reached()
        if limit is not None:
            return limit
        if (self.limits.max_nodes is not None
                and self.counters.nodes_expanded >= self.limits.max_nodes):
            return LIMIT_NODES
        if frontier_size is not None and frontier_size >= self.limits.max_frontier_entries:
            return LIMIT_FRONTIER
        return None

    def _step_checkpoint(self, frontier_size: Optional[int] = None) -> Optional[str]:
        """Cancellation, then the wall-clock checkpoint limit. If either
        triggers while a work limit is also reached, the work limit is the
        reported stop. Work limits alone are not stopped on here; they stop the
        search where they are checked (`_leg_checkpoint`, `_node_checkpoint`,
        the frontier push)."""
        if self.cancel_requested is not None and self.cancel_requested():
            reason = STOP_CANCELLED
        else:
            limit_s = self.limits.wall_clock_checkpoint_limit_s
            if limit_s is not None and self.clock() - self.t0 >= limit_s:
                reason = STOP_WALL_CLOCK_CHECKPOINT
            else:
                return None
        limit = self._work_limit_reached(frontier_size)
        if limit is not None:
            return self._stop(STOP_WORK_LIMIT, limit)
        return self._stop(reason)

    def _leg_checkpoint(self) -> Optional[str]:
        limit = self._leg_limit_reached()
        if limit is not None:
            return self._stop(STOP_WORK_LIMIT, limit)
        return self._step_checkpoint()

    def _node_checkpoint(self) -> Optional[str]:
        """Stops only when the node limit is reached; if a higher-precedence
        work limit is also reached, that one is reported."""
        if (self.limits.max_nodes is not None
                and self.counters.nodes_expanded >= self.limits.max_nodes):
            return self._stop(STOP_WORK_LIMIT, self._work_limit_reached())
        return None

    def _frontier_stop(self, frontier_size: int) -> str:
        """Frontier cap reached (caller checked); report a higher-precedence
        work limit instead if one is also reached."""
        return self._stop(STOP_WORK_LIMIT, self._work_limit_reached(frontier_size))

    # -- legs -------------------------------------------------------------

    def _get_leg(self, start: Pose, target: Pose):
        """`(plan, None)`, or `(None, stop_reason)` if a checkpoint stopped
        the search before a needed leg-planner call."""
        key = (start, target)
        counters = self.counters
        counters.leg_requests += 1
        plan = self.cache.get(key)
        if plan is not None:
            counters.cache_hits += 1
            return plan, None
        stop = self._leg_checkpoint()
        if stop is not None:
            counters.leg_requests -= 1   # the request was never served
            return None, stop
        plan = self.leg_planner(start, target, self.obstacles, self.config)
        _validate_leg_result(plan)
        expansions = plan.expansions
        if isinstance(expansions, bool) or not isinstance(expansions, int) or expansions < 0:
            raise ExecutableRouteInvariantError(
                f"leg planner returned invalid expansions={expansions!r}"
            )
        counters.leg_calls += 1
        counters.lattice_expansions += expansions
        outcome = "success" if plan.is_valid else str(plan.reason or "invalid")
        counters.outcomes[outcome] = counters.outcomes.get(outcome, 0) + 1
        self.cache[key] = plan
        return plan, None

    def _audit_lb(self, pose: Pose, active_index: int, cand_index: int) -> float:
        """Certified single-edge bound for the audit: remaining_ids is the
        singleton `(obstacle_id,)`. Counted in `bound_evaluations` and
        validated like every other bound value, so a malformed value raises
        `InvalidPrefixSearchInputError` instead of reading as a violation."""
        return self._child_lb(pose, active_index, cand_index, 1 << active_index)

    def _audit(self, start: Pose, active_index: int, cand_index: int, target: Pose, plan) -> None:
        """Certified bounds only: the leg must cost at least its single-edge
        bound and end inside the target's tolerance region."""
        if not self.certified:
            return
        key = (start, target)
        if key in self.audited:
            return
        single = self._audit_lb(start, active_index, cand_index)
        model = self.bound.model
        end = plan.final_pose
        distance = math.hypot(end.x - target.x, end.y - target.y)
        heading = abs(math.remainder(end.heading - target.heading, math.tau))
        ok = (plan.total_cost >= single
              and distance <= model.position_tolerance_cm
              and heading <= model.heading_tolerance_rad)
        self.audited[key] = ok
        if not ok:
            self.violations += 1
            if self.phase is not None:
                _reopen_bound_prunes(self.phase)

    def _child_node(self, parent: _Node, active_index: int, cand_index: int,
                    target: Pose, plan) -> _Node:
        node = _Node(
            pose=plan.final_pose,
            g=parent.g + plan.total_cost,
            mask=parent.mask & ~(1 << active_index),
            depth=parent.depth + 1,
            parent=parent,
            leg=plan,
            obstacle_id=self.active_ids[active_index],
            active_index=active_index,
            cand_index=cand_index,
            target=target,
            order_code=parent.order_code * self.code_base + active_index + 1,
            path_key=parent.path_key + ((active_index, cand_index),),
        )
        counters = self.counters
        counters.prefix_codes.add(node.order_code)
        if node.mask == 0:
            counters.complete_codes.add(node.order_code)
            counters.complete_paths.add(node.path_key)
            counters.complete_leaf_discoveries += 1
        return node

    def _root(self) -> _Node:
        return _Node(self.start_pose, 0.0, self.full_mask, 0, None, None, None,
                     None, None, None, 0, ())

    def _children(self, node: _Node) -> list:
        """`(active_index, cand_index, f, target, path_key)` for every child."""
        children = []
        for active_index in _bits(node.mask):
            for cand_index, target in enumerate(self.active_cands[active_index]):
                f = node.g + self._child_lb(node.pose, active_index, cand_index, node.mask)
                children.append((active_index, cand_index, f, target,
                                 node.path_key + ((active_index, cand_index),)))
        self.counters.children_generated += len(children)
        return children

    def _notify_incumbent(self, phase: str, node: _Node) -> None:
        """Observation only: hand a new complete incumbent to `on_incumbent`.
        It never reads a return value and never changes search state. Not
        called for a partial route (an obstacle without a safe candidate)."""
        if self.on_incumbent is not None and not self.empty_ids:
            self.on_incumbent(self.route_from(node), phase)

    def _note_partial(self, node: _Node) -> None:
        best = self.best_partial
        if best is None or node.depth > best.depth or (node.depth == best.depth and node.g < best.g):
            self.best_partial = node

    # -- B2 ---------------------------------------------------------------

    def run_b2(self) -> Optional[_Node]:
        """Lazy DFS. Returns the first complete node, or None."""
        self.phase = phase = _Phase()
        root = self._root()
        if self.full_mask == 0:
            # Nothing to search: zero obstacles (a complete zero-leg route) or
            # every obstacle lacks a safe candidate (nothing left to explore).
            phase.stop_reason = STOP_EXHAUSTED if self.empty_ids else STOP_FIRST_COMPLETE
            return root
        if self._node_checkpoint() is not None:
            phase.unresolved[root.path_key] = self._certified_root_f()
            return None
        self.counters.nodes_expanded += 1
        stack = [[root, _ordered_children(self._children(root)), 0]]
        while stack:
            stop = self._step_checkpoint()
            if stop is not None:
                break
            frame = stack[-1]
            node, children, index = frame
            if index == len(children):
                stack.pop()
                continue
            active_index, cand_index, f, target, path_key = children[index]
            plan, stop = self._get_leg(node.pose, target)
            if stop is not None:
                break
            frame[2] = index + 1
            if not plan.is_valid:
                if not _is_definitive_failure(plan.reason):
                    phase.unresolved[path_key] = self._certified_f(f)
                continue
            self._audit(node.pose, active_index, cand_index, target, plan)
            child = self._child_node(node, active_index, cand_index, target, plan)
            self._note_partial(child)
            if child.mask == 0:
                phase.stop_reason = STOP_FIRST_COMPLETE
                self._notify_incumbent("b2", child)
                return child
            if self._node_checkpoint() is not None:
                phase.unresolved[child.path_key] = self._node_f(child)
                break
            self.counters.nodes_expanded += 1
            stack.append([child, _ordered_children(self._children(child)), 0])
        else:
            phase.stop_reason = STOP_EXHAUSTED
            return None
        # Stopped by a limit or cancellation: every untried child root is unresolved.
        for node, children, index in stack:
            for _ai, _ci, f, _t, path_key in children[index:]:
                phase.unresolved[path_key] = self._certified_f(f)
        return None

    def _node_f(self, node: _Node) -> Optional[float]:
        if not self._pruning():
            return None
        return node.g + self._lb(node.pose, node.mask)

    def _certified_root_f(self) -> Optional[float]:
        if not self._pruning() or self.full_mask == 0:
            return None
        return self._lb(self.start_pose, self.full_mask)

    # -- B3 ---------------------------------------------------------------

    def run_b3(self, incumbent: _Node) -> _Node:
        """Fresh lazy best-first branch-and-bound from the root, seeded with
        the B2 incumbent. Returns the final incumbent (never worse)."""
        self.phase = phase = _Phase()
        counters = self.counters
        cost = incumbent.g
        heap: list = []
        dominance: dict = {}
        counter = 0
        cap = self.limits.max_frontier_entries

        def push(f, depth, path_key, kind, payload):
            nonlocal counter
            heapq.heappush(heap, (f, -depth, path_key, kind, counter, payload))
            counter += 1
            if len(heap) > counters.frontier_peak:
                counters.frontier_peak = len(heap)

        root = self._root()
        root_f = self._lb(self.start_pose, self.full_mask)
        if self._pruning() and _should_prune(root_f, cost):
            counters.bound_prunes += 1
            phase.bound_pruned.append(root.path_key)
            phase.stop_reason = STOP_BOUND_CLOSED
            return incumbent
        push(root_f, 0, root.path_key, _KIND_NODE, root)

        while heap:
            if self._step_checkpoint(len(heap)) is not None:
                break
            if self._pruning() and _should_prune(heap[0][0], cost):
                for entry in heap:
                    phase.bound_pruned.append(entry[2])
                counters.bound_prunes += len(heap)
                heap.clear()
                phase.stop_reason = STOP_BOUND_CLOSED
                return incumbent
            f, _neg_depth, path_key, kind, _counter, payload = heapq.heappop(heap)

            if kind == _KIND_NODE:
                node = payload
                if self._node_checkpoint() is not None:
                    phase.unresolved[path_key] = self._certified_f(f)
                    break
                counters.nodes_expanded += 1
                children = self._children(node)
                stopped = False
                for position, (active_index, cand_index, child_f, target, child_key) in enumerate(children):
                    if self._pruning() and _should_prune(child_f, cost):
                        counters.bound_prunes += 1
                        phase.bound_pruned.append(child_key)
                        continue
                    if len(heap) >= cap:
                        self._frontier_stop(len(heap))
                        for _ai, _ci, rest_f, _t, rest_key in children[position:]:
                            phase.unresolved[rest_key] = self._certified_f(rest_f)
                        stopped = True
                        break
                    push(child_f, node.depth + 1, child_key, _KIND_DEFERRED,
                         (node, active_index, cand_index, target))
                if stopped:
                    break
                continue

            parent, active_index, cand_index, target = payload
            if self._pruning() and _should_prune(f, cost):
                counters.bound_prunes += 1
                phase.bound_pruned.append(path_key)
                continue
            plan, stop = self._get_leg(parent.pose, target)
            if stop is not None:
                phase.unresolved[path_key] = self._certified_f(f)
                break
            if not plan.is_valid:
                if not _is_definitive_failure(plan.reason):
                    phase.unresolved[path_key] = self._certified_f(f)
                continue
            self._audit(parent.pose, active_index, cand_index, target, plan)
            child = self._child_node(parent, active_index, cand_index, target, plan)
            if child.mask == 0:
                if _improves_incumbent(child.g, cost):
                    incumbent, cost = child, child.g
                    counters.incumbent_updates += 1
                    self._notify_incumbent("b3", child)
                continue
            state = (child.pose, child.mask)
            best_g = dominance.get(state)
            if best_g is not None and best_g <= child.g:
                counters.dominance_prunes += 1
                continue
            dominance[state] = child.g
            child_f = child.g + self._lb(child.pose, child.mask)
            if self._pruning() and _should_prune(child_f, cost):
                counters.bound_prunes += 1
                phase.bound_pruned.append(child.path_key)
                continue
            if len(heap) >= cap:
                self._frontier_stop(len(heap))
                phase.unresolved[child.path_key] = self._certified_f(child_f)
                break
            push(child_f, child.depth, child.path_key, _KIND_NODE, child)
        else:
            phase.stop_reason = STOP_EXHAUSTED
            return incumbent

        # Stopped by a limit or cancellation: every frontier entry is unresolved.
        for entry in heap:
            phase.unresolved[entry[2]] = self._certified_f(entry[0])
        heap.clear()
        return incumbent

    # -- results ----------------------------------------------------------

    def route_from(self, node: Optional[_Node]) -> Optional[PrefixRoute]:
        if node is None:
            return None
        chain = []
        while node is not None and node.parent is not None:
            chain.append(node)
            node = node.parent
        chain.reverse()
        legs = tuple(ExecutableRouteLeg(n.obstacle_id, n.target, n.leg) for n in chain)
        visited = {n.obstacle_id for n in chain}
        return PrefixRoute(
            obstacle_order=tuple(n.obstacle_id for n in chain),
            candidate_indices=tuple(n.cand_index for n in chain),
            target_poses=tuple(n.target for n in chain),
            legs=legs,
            total_cost=chain[-1].g if chain else 0.0,
            total_distance_cm=sum(leg.command_plan.total_distance_cm for leg in legs),
            final_pose=chain[-1].leg.final_pose if chain else self.start_pose,
            skipped_obstacle_ids=tuple(
                o.obstacle_id for o in self.obstacles if o.obstacle_id not in visited
            ),
        )

    def result(self, complete_node, b2_node, b2_work) -> PrefixSearchResult:
        phase = self.phase
        route = self.route_from(complete_node)
        complete = route is not None and not self.empty_ids
        partial_node = complete_node if (complete_node is not None and self.empty_ids) else self.best_partial
        partial = None if complete else self.route_from(partial_node)
        if partial is not None and not partial.legs:
            partial = None
        route_status = ROUTE_COMPLETE if complete else (ROUTE_PARTIAL_ONLY if partial else ROUTE_NONE)
        incumbent_cost = route.total_cost if complete else None
        unresolved_fs = list(phase.unresolved.values())
        exhausted, impossible, optimal, min_unresolved, bound_known = _finalise_flags(
            self.mode, complete, phase.stop_reason, self.certified, self.violations,
            unresolved_fs, incumbent_cost, self.empty_ids,
        )
        root_lb = None
        if self.certified and self.bound is not None:
            root_lb = _check_bound_value(
                self.bound.lower_bound(self.start_pose, self._remaining(self.full_mask))
            )
        elif self.certified and self.full_mask == 0:
            root_lb = 0.0
        return PrefixSearchResult(
            mode=self.mode,
            route_status=route_status,
            complete_route=route if complete else None,
            best_partial_route=partial,
            b2_route=self.route_from(b2_node) if self.mode == MODE_B3 else None,
            stop_reason=phase.stop_reason,
            work_limit_hit=phase.work_limit_hit,
            search_space_exhausted=exhausted,
            complete_route_proven_impossible=impossible,
            optimality_proven=optimal,
            empty_candidate_obstacle_ids=self.empty_ids,
            filtered_primary_obstacle_ids=self.filtered_primary_ids,
            unresolved_branch_count=len(unresolved_fs),
            min_unresolved_lower_bound=min_unresolved,
            unresolved_bound_known=bound_known,
            lower_bound_certified=self.certified,
            bound_violation_count=self.violations,
            bound_pruning_active=self._pruning(),
            root_lower_bound=root_lb,
            message=self._message(route_status, complete, partial, phase, impossible,
                                  optimal, len(unresolved_fs)),
            work=self.counters.snapshot(),
            b2_work=b2_work,
            elapsed_s=self.clock() - self.t0,
        )

    def _message(self, route_status, complete, partial, phase, impossible, optimal, unresolved) -> str:
        stop = phase.stop_reason
        if complete:
            text = "Complete executable route found."
            if self.mode == MODE_B3:
                if optimal:
                    text += " Optimality proven."
                else:
                    text += f" Optimality not proven ({self._not_proven_reason(stop, unresolved)})."
            return text
        if self.empty_ids:
            ids = ", ".join(str(i) for i in self.empty_ids)
            text = f"No complete route exists: obstacle(s) {ids} have no safe viewing candidate."
            if stop == STOP_WORK_LIMIT:
                text += f" Partial-route search stopped within budget ({phase.work_limit_hit})."
            elif stop == STOP_WALL_CLOCK_CHECKPOINT:
                text += (" Partial-route search stopped within budget "
                         "(wall-clock checkpoint limit reached between legs).")
            elif stop == STOP_CANCELLED:
                text += " Partial-route search cancelled."
        elif impossible:
            text = "No complete route exists under the supplied viewing candidates and leg planner."
        elif stop == STOP_EXHAUSTED:
            text = (f"No complete route found; {unresolved} branch(es) were inconclusive "
                    "(e.g. expansion_limit), so non-existence is not proven.")
        elif stop == STOP_WORK_LIMIT:
            text = f"No complete route found within budget ({phase.work_limit_hit})."
        elif stop == STOP_WALL_CLOCK_CHECKPOINT:
            text = ("No complete route found within budget "
                    "(wall-clock checkpoint limit reached between legs).")
        else:
            text = "Search cancelled before a complete route was found."
        if partial is not None:
            text += (f" Best partial route visits {len(partial.obstacle_order)}/"
                     f"{len(self.obstacles)} obstacles.")
        return text

    def _not_proven_reason(self, stop, unresolved) -> str:
        if not self.certified:
            return "lower bound not certified"
        if self.violations:
            return "lower-bound audit violation"
        if stop == STOP_WORK_LIMIT:
            return f"work limit {self.phase.work_limit_hit} reached"
        if stop == STOP_WALL_CLOCK_CHECKPOINT:
            return "wall-clock checkpoint limit reached between legs"
        if stop == STOP_CANCELLED:
            return "cancelled"
        return f"{unresolved} unresolved branch(es)"


# --------------------------------------------------------------------------
# Public API
# --------------------------------------------------------------------------

def find_prefix_b2_route(
    start_pose: Pose,
    obstacles: Iterable[Obstacle],
    *,
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
    leg_planner: LegPlanner = plan_executable_leg,
    allow_alternative_viewing: bool = True,
    limits: SearchLimits = SearchLimits(),
    clock: Callable[[], float] = time.monotonic,
    cancel_requested: Optional[Callable[[], bool]] = None,
    bound_factory: BoundFactory = build_remaining_route_bound,
    on_incumbent: Optional[Callable[[PrefixRoute, str], None]] = None,
) -> PrefixSearchResult:
    """First complete executable route by lower-bound-guided lazy DFS over
    obstacle-order prefixes (module docstring). Never claims optimality.
    Never mutates `obstacles` or `config`. `on_incumbent(route, "b2")` is
    called once, synchronously, when that complete route is found (observation
    only; see `optimise_prefix_b3_route`)."""
    search = _Search(MODE_B2, start_pose, obstacles, config, leg_planner,
                     allow_alternative_viewing, limits, clock, cancel_requested, bound_factory,
                     on_incumbent)
    complete = search.run_b2()
    return search.result(complete, None, None)


def optimise_prefix_b3_route(
    start_pose: Pose,
    obstacles: Iterable[Obstacle],
    *,
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
    leg_planner: LegPlanner = plan_executable_leg,
    allow_alternative_viewing: bool = True,
    limits: SearchLimits = SearchLimits(),
    clock: Callable[[], float] = time.monotonic,
    cancel_requested: Optional[Callable[[], bool]] = None,
    bound_factory: BoundFactory = build_remaining_route_bound,
    on_incumbent: Optional[Callable[[PrefixRoute, str], None]] = None,
) -> PrefixSearchResult:
    """B2 first (its route is the incumbent and `b2_route`), then anytime
    lazy best-first branch-and-bound. Never returns a route worse than B2's.
    Never mutates `obstacles` or `config`.

    `on_incumbent(route, phase)` is an optional observer for a caller that must
    keep the best complete route outside this call (for example a parent
    process that may terminate it). It is called synchronously, in order,
    with the first complete B2 route (`"b2"`) and then every strictly cheaper
    B3 route (`"b3"`), never for a partial route. Its return value is ignored
    and it changes no search state; an exception it raises propagates."""
    search = _Search(MODE_B3, start_pose, obstacles, config, leg_planner,
                     allow_alternative_viewing, limits, clock, cancel_requested, bound_factory,
                     on_incumbent)
    b2_node = search.run_b2()
    b2_work = search.counters.snapshot()
    if b2_node is None:
        return search.result(None, None, b2_work)
    if search.full_mask == 0:
        search.phase = _Phase()
        search.phase.stop_reason = STOP_EXHAUSTED
        return search.result(b2_node, b2_node, b2_work)
    final = search.run_b3(b2_node)
    return search.result(final, b2_node, b2_work)
