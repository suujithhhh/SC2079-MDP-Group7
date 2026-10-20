"""Original Roadmap Step 5 -- executable obstacle ordering.

Plans a full multi-obstacle route the same way the legacy `b3_optimizer.py`
does -- choose an obstacle visiting order, plan one leg per obstacle, chain
legs together -- but every leg is planned through the *executable* pipeline
built in the previous roadmap steps: `state_lattice.plan_lattice_path()`
then `stm_command_generation.generate_stm_command_plan()` (which itself
always runs `command_compression.compress_lattice_plan()` first). This
module is standalone: it does not modify, replace, or get called by the
legacy `b3_optimizer.py`, `planner_interface.py`, `planning_service.py`,
the web UI, or protocol-v1 responses. Protocol v1 and the current webpage
keep working exactly as before.

Estimated poses, not measurements
------------------------------------
Every leg's `command_plan.final_pose` is the modelled, dead-reckoned pose
the state-lattice search reached within its configured goal tolerance --
never assumed to equal the leg's ideal `target_pose` exactly, and never
described as achieved, observed, or measured (`PROTOCOL.md` §5-§7). The
*next* leg always starts from that exact estimated pose, never from the
ideal target it was aiming at.

Route search
--------------
Both `find_executable_b2_route()` (first complete order) and
`optimise_executable_b3_route()` (lowest-`total_cost` complete order)
share one candidate-chain search over `itertools.permutations(obstacles)`
-- whose first yielded permutation is always the input order, satisfying
"try the input order first" deterministically. For each obstacle, the
candidate viewing poses tried are:

- `allow_alternative_viewing=False`: just `targets.viewing_target()` (the
  primary pose).
- `allow_alternative_viewing=True`: `targets.viewing_targets()`, whose own
  first entry *is* the primary pose -- so primary is always tried before
  any alternative, and a failed primary leg simply falls through to the
  next candidate in that same fixed order.

Every candidate pose is filtered by `footprint.pose_is_safe()` -- the same
collision geometry `state_lattice.py` itself uses -- before it is ever
offered to the search, so a viewing pose that is already unsafe (out of
the arena or overlapping an obstacle) never reaches the leg planner at
all, real or injected. No collision geometry is re-derived here.

Within one obstacle order, a small forward DP tracks, for each reachable
*exact* final pose, the cheapest chain of legs that reached it (keyed by
that exact `Pose`, never a quantised or ideal one) -- this is what lets a
failed leg prune only its own prefix without discarding the rest of the
order's search, and what lets `optimise_executable_b3_route()` pick the
cheapest candidate combination once an order does complete. B3 additionally
applies branch-and-bound: once a complete route exists, every state whose
accumulated cost already reaches or exceeds it is dropped, since remaining
legs can only add non-negative cost. Before even calling the leg planner,
a candidate is also skipped when its *admissible lower bound* -- the same
Euclidean-distance-minus-goal-tolerance reasoning `state_lattice.py` uses
for its own search heuristic, `max(0, distance - goal_position_tolerance)
* min(1.0, reverse_penalty)` -- combined with the cost already spent
already reaches or exceeds the incumbent. Both prunings are sound (no leg
can ever cost less than that bound), but neither *proves* an order
unreachable -- a pruned order is tracked separately from a fully-searched,
genuinely rejected one (see `ExecutableRouteResult.pruned_orders`).

Every (start pose, target pose) leg query is cached for the duration of
one top-level call (`obstacles`/`config` are fixed for that whole call, so
they need not be part of the cache key) -- two different obstacle orders
that happen to share an identical leg never re-plan it. The cache is keyed
by the *exact* pose pair, never by the ideal target alone, so two
different actual start poses aiming at the same target are never conflated.
Every leg result returned by an injected `leg_planner` is validated before
anything downstream relies on it: a result claiming `is_valid=True` must
carry a present `final_pose`, a `None` `reason`, and a finite, non-negative,
non-boolean `total_cost`/`total_distance_cm`, or
`ExecutableRouteInvariantError` is raised. An ordinary `is_valid=False`
leg is unreachable, not malformed, and never raises this.

Partial routes require alternatives
---------------------------------------
A partial route (fewer than all obstacles visited) is only ever returned
when `allow_alternative_viewing=True`. With alternatives disabled and no
order completing, both `find_executable_b2_route()` and
`optimise_executable_b3_route()` return an invalid result -- a reachable
prefix is never silently exposed as a successful partial route unless the
caller has explicitly asked this module to fall back to alternate viewing
poses first.

No direction-switch or timing cost
-------------------------------------
`total_cost` here is exactly the sum of each leg's `STMCommandPlan.
total_cost` -- state-lattice's existing geometric-length-plus-
`reverse_penalty` model. No calibrated direction-switch or command-switch
penalty is added in this module, and `total_cost` is never described as
travel time or "fastest" -- no timing model exists yet for this pipeline.

Optional search statistics (profiling)
-----------------------------------------
Both public functions accept `stats: Optional[RouteSearchStats] = None`. When
given, the search counts and times what it already does -- leg requests,
cache hits/misses (so unique legs), per-depth cache hit rates, candidates
and bound prunes, incumbent updates, per-outcome leg time and lattice
expansions and, for the default leg planner, lattice-search time versus
compression/command-generation time. It never changes a decision, the leg
cache, candidate order, pruning or any returned value, and it adds no field
to `ExecutableRouteResult` or to any protocol-v2 response. The consumer is
`algo/benchmarks/`.

Performance: not yet validated for a full mission
-----------------------------------------------------
Real `DEFAULT_PLANNER_CONFIG` searches through this module can be slow for
some pose pairs (the measured turn radii vs. small obstacles issue already
documented for `state_lattice.py`), multiplied by however many obstacle
orders and viewing candidates are tried. An exploratory, non-benchmark
timing run in this environment measured roughly 50 seconds for a
three-obstacle `optimise_executable_b3_route()` search with alternatives
disabled (3 of its 6 orders were cut by the admissible-lower-bound
pruning above before ever reaching the leg planner; without that pruning
the same search measured roughly 68 seconds). Full five-obstacle (or
larger) mission runtime has **not** been measured, and this module must
not be described as production-ready for the Task 1 six-minute mission
until it has been. Tests keep ordering logic on an injected fake leg
planner and reserve the real pipeline for small (1-2 obstacle) focused
integration tests.
"""

import dataclasses
import itertools
import math
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

import footprint
from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from models import Obstacle, Pose
from state_lattice import _heuristic_cm, plan_lattice_path
from stm_command_generation import STMCommandPlan, generate_stm_command_plan
from targets import viewing_target, viewing_targets

LegPlanner = Callable[[Pose, Pose, Iterable[Obstacle], PlannerConfig], STMCommandPlan]


class ExecutableRouteInvariantError(RuntimeError):
    """Raised when an injected `leg_planner` returns a result claiming
    `is_valid=True` that violates an invariant route aggregation and
    branch-and-bound depend on: a present `final_pose`, `reason is None`,
    and a finite, non-negative, non-boolean `total_cost`/
    `total_distance_cm`. An ordinary `is_valid=False` result is an
    unreachable leg, not a malformed one, and never raises this."""


class InvalidExecutableRouteInputError(ValueError):
    """Raised for malformed input to this standalone module's public API.
    Currently: duplicate `obstacle_id`s, which would otherwise silently
    overwrite one obstacle's entry in the internal candidate-target map."""


def plan_executable_leg(
    start_pose: Pose,
    target_pose: Pose,
    obstacles: Iterable[Obstacle],
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
) -> STMCommandPlan:
    """Default `LegPlanner`: state-lattice search, then STM command generation
    (which always compresses first). Never bypasses either step."""
    obstacles = tuple(obstacles)
    lattice_plan = plan_lattice_path(start_pose, target_pose, obstacles, config)
    return generate_stm_command_plan(lattice_plan, obstacles, config)


@dataclass(frozen=True)
class LegRecord:
    """One *planned* leg (a leg-cache miss), recorded only when a
    `RouteSearchStats` is passed. Immutable. `outcome` is `"success"` or the
    leg's failure reason (`state_lattice.REASON_*`); `lattice_s`/
    `postprocess_s` are `None` when an injected `leg_planner` made the split
    unobservable. `depth` is the position of the target in the obstacle
    order (0 = first leg)."""

    depth: int
    outcome: str
    elapsed_s: float
    lattice_s: Optional[float]
    postprocess_s: Optional[float]
    expansions: int
    distance_cm: float
    heading_diff_deg: float
    cost: float


class RouteSearchStats:
    """Opt-in measurement of one `find_executable_b2_route()`/
    `optimise_executable_b3_route()` call.

    Purely observational: it counts and times what the search already does
    and never influences a decision, the leg cache, candidate order, pruning
    or any returned value. Passing `stats=None` (the default) runs the
    original logic with no measurement. One instance measures one call.

    `leg_requests == cache_hits + cache_misses` always, and every miss is one
    call to the leg planner, so `unique_legs == cache_misses`.
    `candidates_attempted`, `bound_prunes_*` and `invalid_leg_results` count
    candidate evaluations (cache hits included); `outcomes`, `leg_records`
    and the `*_s`/`lattice_expansions` totals cover only legs actually
    planned (misses). Times come from `time.perf_counter()`.
    """

    def __init__(self, record_legs: bool = True):
        self.record_legs = record_legs
        self.algorithm: Optional[str] = None
        self.finished = False
        self.wall_s = 0.0
        self.orders_searched = 0
        self.orders_completed = 0
        self.candidates_attempted = 0
        self.bound_prunes_pre_call = 0
        self.bound_prunes_post_call = 0
        self.invalid_leg_results = 0
        self.incumbent_updates = 0
        self.leg_requests = 0
        self.cache_hits = 0
        self.cache_misses = 0
        self.legs_by_depth: dict = {}       # depth -> [requests, hits]
        self.leg_planner_s = 0.0            # time inside cache-miss leg-planner calls
        self.lattice_s = 0.0                # ...of which state-lattice search (default planner only)
        self.postprocess_s = 0.0            # ...of which compression + command generation
        self.separable_legs = 0             # planned legs whose lattice/postprocess split is known
        self.lattice_expansions = 0
        self.outcomes: dict = {}            # outcome -> [count, elapsed_s, expansions]
        self.leg_records: list = []
        self._t0: Optional[float] = None

    @property
    def unique_legs(self) -> int:
        return self.cache_misses

    def begin(self, algorithm: str) -> None:
        self.algorithm = algorithm
        self._t0 = time.perf_counter()

    def end(self) -> None:
        self.wall_s = time.perf_counter() - self._t0
        self.finished = True

    def note_request(self, depth: int, hit: bool) -> None:
        self.leg_requests += 1
        row = self.legs_by_depth.setdefault(depth, [0, 0])
        row[0] += 1
        if hit:
            self.cache_hits += 1
            row[1] += 1
        else:
            self.cache_misses += 1

    def plan_leg(self, leg_planner: LegPlanner, start: Pose, target: Pose,
                 obstacles: tuple, config: PlannerConfig, depth: int = 0) -> STMCommandPlan:
        """Plan one leg exactly as `leg_planner` would, timing it. For the
        default `plan_executable_leg` the two pipeline stages are timed
        separately by making the very same two calls it makes."""
        t0 = time.perf_counter()
        if leg_planner is plan_executable_leg:
            obstacles = tuple(obstacles)
            lattice_plan = plan_lattice_path(start, target, obstacles, config)
            t1 = time.perf_counter()
            plan = generate_stm_command_plan(lattice_plan, obstacles, config)
            t2 = time.perf_counter()
            lattice_s, postprocess_s = t1 - t0, t2 - t1
        else:
            plan = leg_planner(start, target, obstacles, config)
            t2 = time.perf_counter()
            lattice_s = postprocess_s = None
        elapsed = t2 - t0
        outcome = "success" if plan.is_valid else str(plan.reason or "invalid")
        self.leg_planner_s += elapsed
        if lattice_s is not None:
            self.lattice_s += lattice_s
            self.postprocess_s += postprocess_s
            self.separable_legs += 1
        self.lattice_expansions += plan.expansions
        row = self.outcomes.setdefault(outcome, [0, 0.0, 0])
        row[0] += 1
        row[1] += elapsed
        row[2] += plan.expansions
        if self.record_legs:
            self.leg_records.append(LegRecord(
                depth=depth, outcome=outcome, elapsed_s=elapsed,
                lattice_s=lattice_s, postprocess_s=postprocess_s,
                expansions=plan.expansions,
                distance_cm=math.hypot(target.x - start.x, target.y - start.y),
                heading_diff_deg=abs(math.degrees(math.remainder(start.heading - target.heading, math.tau))),
                cost=plan.total_cost if plan.is_valid else 0.0,
            ))
        return plan

    def to_dict(self, include_legs: bool = False) -> dict:
        """JSON-serialisable summary. `include_legs` adds the per-leg records."""
        def rate(hits: int, requests: int):
            return hits / requests if requests else None

        elapsed = self.wall_s
        if not self.finished and self._t0 is not None:
            elapsed = time.perf_counter() - self._t0
        data = {
            "algorithm": self.algorithm,
            "finished": self.finished,
            "wall_s": elapsed,
            "orders_searched": self.orders_searched,
            "orders_completed": self.orders_completed,
            "candidates_attempted": self.candidates_attempted,
            "bound_prunes_pre_call": self.bound_prunes_pre_call,
            "bound_prunes_post_call": self.bound_prunes_post_call,
            "invalid_leg_results": self.invalid_leg_results,
            "incumbent_updates": self.incumbent_updates,
            "leg_requests": self.leg_requests,
            "cache_hits": self.cache_hits,
            "cache_misses": self.cache_misses,
            "unique_legs": self.unique_legs,
            "cache_hit_rate": rate(self.cache_hits, self.leg_requests),
            "legs_by_depth": {
                str(depth): {"requests": row[0], "hits": row[1], "misses": row[0] - row[1],
                              "hit_rate": rate(row[1], row[0])}
                for depth, row in sorted(self.legs_by_depth.items())
            },
            "leg_planner_s": self.leg_planner_s,
            "lattice_s": self.lattice_s,
            "postprocess_s": self.postprocess_s,
            "separable_legs": self.separable_legs,
            "lattice_expansions": self.lattice_expansions,
            "outcomes": {
                name: {"count": row[0], "elapsed_s": row[1], "expansions": row[2]}
                for name, row in sorted(self.outcomes.items())
            },
        }
        if include_legs:
            data["leg_records"] = [dataclasses.asdict(record) for record in list(self.leg_records)]
        return data


@dataclass(frozen=True)
class ExecutableRouteLeg:
    """One obstacle visit within an executable route. Immutable.

    `target_pose` is the requested ideal viewing pose; `command_plan.
    final_pose` is the modelled, estimated pose actually reached within
    the lattice goal tolerance -- never described as measured or observed.
    """

    obstacle_id: int
    target_pose: Pose
    command_plan: STMCommandPlan


@dataclass(frozen=True)
class ExecutableRouteResult:
    """Result of `find_executable_b2_route`/`optimise_executable_b3_route`. Immutable.

    No estimated-time field: timing is not calibrated for this pipeline.
    `total_cost` is the summed geometric/reverse-penalty planning cost.
    `orders_considered == valid_orders + rejected_orders + pruned_orders`
    always holds. `valid_orders` counts orders that produced a complete
    route (even if some of their own states were pruned elsewhere);
    `rejected_orders` counts orders fully searched to exhaustion with no
    complete route found; `pruned_orders` counts orders that did not
    complete only because a branch-and-bound cost bound cut one or more
    of their candidate paths before they could be fully explored --
    pruning never proves an order unreachable, so it is never folded into
    `rejected_orders`. A partial route (`is_complete=False`) is only ever
    returned when the search was called with `allow_alternative_viewing=
    True`.
    """

    obstacle_order: tuple                # tuple[int, ...]
    target_poses: tuple                  # tuple[Pose, ...]
    legs: tuple                          # tuple[ExecutableRouteLeg, ...]
    total_distance_cm: float
    total_cost: float
    is_valid: bool
    is_complete: bool
    orders_considered: int
    valid_orders: int
    rejected_orders: int
    pruned_orders: int
    skipped_obstacle_ids: tuple          # tuple[int, ...]
    final_pose: Optional[Pose]
    failure_reason: Optional[str]


def _validate_no_duplicate_obstacle_ids(obstacles: tuple) -> None:
    seen = set()
    for obstacle in obstacles:
        if obstacle.obstacle_id in seen:
            raise InvalidExecutableRouteInputError(
                f"duplicate obstacle_id {obstacle.obstacle_id!r} in obstacles"
            )
        seen.add(obstacle.obstacle_id)


def _validate_leg_result(plan: STMCommandPlan) -> None:
    """Guard against a malformed injected `leg_planner` result before any
    cost or pose it carries is trusted. Only a result claiming
    `is_valid=True` is checked -- an `is_valid=False` leg is an ordinary
    unreachable leg, not a malformed one, and never raises."""
    if not plan.is_valid:
        return
    if plan.final_pose is None:
        raise ExecutableRouteInvariantError(
            "leg planner returned is_valid=True with final_pose=None"
        )
    if plan.reason is not None:
        raise ExecutableRouteInvariantError(
            f"leg planner returned is_valid=True with a non-None reason: {plan.reason!r}"
        )
    for name, value in (
        ("total_cost", plan.total_cost),
        ("total_distance_cm", plan.total_distance_cm),
    ):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0
        ):
            raise ExecutableRouteInvariantError(
                f"leg planner returned is_valid=True with invalid {name}={value!r}"
            )


def _candidate_targets(
    obstacle: Obstacle,
    obstacles: tuple,
    config: PlannerConfig,
    allow_alternative_viewing: bool,
) -> tuple:
    """Viewing-pose candidates for one obstacle, safety-filtered up front.

    Reuses `footprint.pose_is_safe()` -- the same check `state_lattice.py`
    itself makes on a leg's goal pose -- so a candidate whose footprint
    already hits an obstacle or leaves the arena is dropped before it is
    ever offered to a leg planner, real or injected. No collision geometry
    is re-derived here.
    """
    if allow_alternative_viewing:
        raw = viewing_targets(obstacle, config)
    else:
        raw = (viewing_target(obstacle, config),)
    return tuple(pose for pose in raw if footprint.pose_is_safe(pose, obstacles, config))


def _best_chains_for_order(
    start_pose: Pose,
    order: tuple,
    candidates_by_id: dict,
    obstacles: tuple,
    config: PlannerConfig,
    leg_planner: LegPlanner,
    leg_cache: dict,
    cost_bound: Optional[float],
    stats: Optional["RouteSearchStats"] = None,
) -> tuple:
    """Forward DP over the viewing candidates of one obstacle order.

    Returns `(best_by_length, was_pruned)`. `best_by_length` maps
    `{chain_length: (cost, targets_tuple, legs_tuple)}`, the cheapest chain
    reaching each achieved prefix length (1..len(order), up to wherever the
    search dead-ends). States are keyed internally by the exact reached
    `Pose` (never quantised), so the goal-tolerance slack of
    `plan_lattice_path` is preserved exactly, not re-approximated here.

    If `cost_bound` is given, a candidate is skipped in two ways, both
    sound because no leg can ever cost less than its own admissible lower
    bound (the same reasoning `state_lattice.py` proves for its search
    heuristic): first, *before* the leg planner is ever called, when
    `accumulated_cost + max(0, euclidean_distance - goal_position_
    tolerance) * min(1.0, reverse_penalty) >= cost_bound`; second, after
    the leg planner has run, when its *actual* returned cost still reaches
    or exceeds `cost_bound`. `was_pruned` is True iff either kind of cut
    happened anywhere in this order search. Neither kind of pruning
    proves the order unreachable -- callers must not count a pruned order
    as rejected.

    `stats`, when given, only counts and times what happens here (leg
    requests by chain depth, cache hits, candidates, prunes); it never
    changes a decision.
    """

    def leg(start: Pose, target: Pose, depth: int) -> STMCommandPlan:
        key = (start, target)
        cached = leg_cache.get(key)
        if stats is not None:
            stats.note_request(depth, cached is not None)
        if cached is None:
            if stats is None:
                cached = leg_planner(start, target, obstacles, config)
            else:
                cached = stats.plan_leg(leg_planner, start, target, obstacles, config, depth)
            _validate_leg_result(cached)
            leg_cache[key] = cached
        return cached

    pruned = False
    first = order[0]
    states = {}  # exact Pose -> (cost, targets_tuple, legs_tuple)
    for target in candidates_by_id[first.obstacle_id]:
        if stats is not None:
            stats.candidates_attempted += 1
        if cost_bound is not None and _heuristic_cm(start_pose, target, config) >= cost_bound:
            pruned = True
            if stats is not None:
                stats.bound_prunes_pre_call += 1
            continue
        plan = leg(start_pose, target, 0)
        if not plan.is_valid:
            if stats is not None:
                stats.invalid_leg_results += 1
            continue
        cost = plan.total_cost
        if cost_bound is not None and cost >= cost_bound:
            pruned = True
            if stats is not None:
                stats.bound_prunes_post_call += 1
            continue
        pose = plan.final_pose
        existing = states.get(pose)
        if existing is None or cost < existing[0]:
            leg_obj = ExecutableRouteLeg(first.obstacle_id, target, plan)
            states[pose] = (cost, (target,), (leg_obj,))

    best_by_length = {}
    if states:
        best_by_length[1] = min(states.values(), key=lambda s: s[0])

    for depth, obstacle in enumerate(order[1:], start=1):
        next_states = {}
        for target in candidates_by_id[obstacle.obstacle_id]:
            for previous_pose, (cost, targets, legs) in states.items():
                if stats is not None:
                    stats.candidates_attempted += 1
                if cost_bound is not None:
                    lower_bound = cost + _heuristic_cm(previous_pose, target, config)
                    if lower_bound >= cost_bound:
                        pruned = True
                        if stats is not None:
                            stats.bound_prunes_pre_call += 1
                        continue
                plan = leg(previous_pose, target, depth)
                if not plan.is_valid:
                    if stats is not None:
                        stats.invalid_leg_results += 1
                    continue
                new_cost = cost + plan.total_cost
                if cost_bound is not None and new_cost >= cost_bound:
                    pruned = True
                    if stats is not None:
                        stats.bound_prunes_post_call += 1
                    continue
                new_pose = plan.final_pose
                existing = next_states.get(new_pose)
                if existing is None or new_cost < existing[0]:
                    leg_obj = ExecutableRouteLeg(obstacle.obstacle_id, target, plan)
                    next_states[new_pose] = (
                        new_cost, targets + (target,), legs + (leg_obj,)
                    )
        states = next_states
        if not states:
            break
        length = len(next(iter(states.values()))[1])
        best_by_length[length] = min(states.values(), key=lambda s: s[0])

    return best_by_length, pruned


def _better_partial(current: Optional[tuple], length: int, cost: float, visited_ids: tuple,
                     targets: tuple, legs: tuple) -> tuple:
    """Prefer more visited obstacles; break ties by lower cost; otherwise
    keep whichever was found first (deterministic traversal order)."""
    if current is None:
        return (length, cost, visited_ids, targets, legs)
    current_length, current_cost = current[0], current[1]
    if length > current_length or (length == current_length and cost < current_cost):
        return (length, cost, visited_ids, targets, legs)
    return current


def _total_distance(legs: tuple) -> float:
    return sum(leg.command_plan.total_distance_cm for leg in legs)


def _complete_result(order: tuple, targets: tuple, legs: tuple, cost: float,
                      orders_considered: int, valid_orders: int, rejected: int,
                      pruned: int) -> ExecutableRouteResult:
    return ExecutableRouteResult(
        obstacle_order=tuple(o.obstacle_id for o in order),
        target_poses=targets,
        legs=legs,
        total_distance_cm=_total_distance(legs),
        total_cost=cost,
        is_valid=True,
        is_complete=True,
        orders_considered=orders_considered,
        valid_orders=valid_orders,
        rejected_orders=rejected,
        pruned_orders=pruned,
        skipped_obstacle_ids=(),
        final_pose=legs[-1].command_plan.final_pose,
        failure_reason=None,
    )


def _partial_result(obstacles: tuple, visited_ids: tuple, targets: tuple, legs: tuple,
                     cost: float, orders_considered: int, rejected: int,
                     pruned: int) -> ExecutableRouteResult:
    visited = set(visited_ids)
    skipped = tuple(o.obstacle_id for o in obstacles if o.obstacle_id not in visited)
    return ExecutableRouteResult(
        obstacle_order=visited_ids,
        target_poses=targets,
        legs=legs,
        total_distance_cm=_total_distance(legs),
        total_cost=cost,
        is_valid=True,
        is_complete=False,
        orders_considered=orders_considered,
        valid_orders=0,
        rejected_orders=rejected,
        pruned_orders=pruned,
        skipped_obstacle_ids=skipped,
        final_pose=legs[-1].command_plan.final_pose if legs else None,
        failure_reason=(
            f"No complete executable route visits all {len(obstacles)} obstacles; "
            f"returning the best {len(visited_ids)}/{len(obstacles)}-obstacle partial route."
        ),
    )


def _no_route_result(reason: str, orders_considered: int, rejected: int,
                      pruned: int) -> ExecutableRouteResult:
    return ExecutableRouteResult(
        obstacle_order=(), target_poses=(), legs=(),
        total_distance_cm=0.0, total_cost=0.0,
        is_valid=False, is_complete=False,
        orders_considered=orders_considered, valid_orders=0, rejected_orders=rejected,
        pruned_orders=pruned, skipped_obstacle_ids=(), final_pose=None, failure_reason=reason,
    )


def _empty_result(start_pose: Pose) -> ExecutableRouteResult:
    return ExecutableRouteResult(
        obstacle_order=(), target_poses=(), legs=(),
        total_distance_cm=0.0, total_cost=0.0,
        is_valid=True, is_complete=True,
        orders_considered=1, valid_orders=1, rejected_orders=0, pruned_orders=0,
        skipped_obstacle_ids=(), final_pose=start_pose, failure_reason=None,
    )


def find_executable_b2_route(
    start_pose: Pose,
    obstacles: Iterable[Obstacle],
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
    leg_planner: LegPlanner = plan_executable_leg,
    allow_alternative_viewing: bool = True,
    stats: Optional[RouteSearchStats] = None,
) -> ExecutableRouteResult:
    """First complete executable route, trying obstacle orders deterministically
    (input order first) and, per obstacle, primary viewing target before
    alternatives. Never mutates `obstacles`/`config`. Raises
    `InvalidExecutableRouteInputError` on duplicate obstacle IDs. A
    partial route is only ever returned when `allow_alternative_viewing`
    is True. `stats` (default `None`) opts into `RouteSearchStats`
    measurement; it never changes the returned result."""
    if stats is None:
        return _find_b2(start_pose, obstacles, config, leg_planner, allow_alternative_viewing, None)
    stats.begin("b2")
    try:
        return _find_b2(start_pose, obstacles, config, leg_planner, allow_alternative_viewing, stats)
    finally:
        stats.end()


def _find_b2(start_pose, obstacles, config, leg_planner, allow_alternative_viewing, stats):
    obstacles = tuple(obstacles)
    if not obstacles:
        return _empty_result(start_pose)
    _validate_no_duplicate_obstacle_ids(obstacles)

    candidates_by_id = {
        o.obstacle_id: _candidate_targets(o, obstacles, config, allow_alternative_viewing)
        for o in obstacles
    }
    leg_cache: dict = {}
    n = len(obstacles)
    orders_considered = 0
    rejected = 0
    best_partial = None

    for order in itertools.permutations(obstacles):
        orders_considered += 1
        if stats is not None:
            stats.orders_searched += 1
        best_by_length, _pruned = _best_chains_for_order(
            start_pose, order, candidates_by_id, obstacles, config, leg_planner,
            leg_cache, cost_bound=None, stats=stats,
        )
        if n in best_by_length:
            if stats is not None:
                stats.orders_completed += 1
            cost, targets, legs = best_by_length[n]
            return _complete_result(order, targets, legs, cost, orders_considered, 1, rejected, 0)

        rejected += 1
        if allow_alternative_viewing and best_by_length:
            max_length = max(best_by_length)
            cost, targets, legs = best_by_length[max_length]
            visited_ids = tuple(o.obstacle_id for o in order[:max_length])
            best_partial = _better_partial(best_partial, max_length, cost, visited_ids, targets, legs)

    if allow_alternative_viewing and best_partial is not None:
        _, cost, visited_ids, targets, legs = best_partial
        return _partial_result(obstacles, visited_ids, targets, legs, cost, orders_considered, rejected, 0)
    return _no_route_result(
        "No complete executable route visits all obstacles.", orders_considered, rejected, 0
    )


def optimise_executable_b3_route(
    start_pose: Pose,
    obstacles: Iterable[Obstacle],
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
    leg_planner: LegPlanner = plan_executable_leg,
    allow_alternative_viewing: bool = True,
    stats: Optional[RouteSearchStats] = None,
) -> ExecutableRouteResult:
    """Lowest-`total_cost` complete executable route across every obstacle
    order and viewing candidate. Branch-and-bound once a complete route is
    found, using both the admissible-lower-bound pre-filter and the
    actual-cost check in `_best_chains_for_order`. Never mutates
    `obstacles`/`config`. Raises `InvalidExecutableRouteInputError` on
    duplicate obstacle IDs. A partial route is only ever returned when
    `allow_alternative_viewing` is True. `stats` (default `None`) opts into
    `RouteSearchStats` measurement; it never changes the returned result."""
    if stats is None:
        return _optimise_b3(start_pose, obstacles, config, leg_planner, allow_alternative_viewing, None)
    stats.begin("b3")
    try:
        return _optimise_b3(start_pose, obstacles, config, leg_planner, allow_alternative_viewing, stats)
    finally:
        stats.end()


def _optimise_b3(start_pose, obstacles, config, leg_planner, allow_alternative_viewing, stats):
    obstacles = tuple(obstacles)
    if not obstacles:
        return _empty_result(start_pose)
    _validate_no_duplicate_obstacle_ids(obstacles)

    candidates_by_id = {
        o.obstacle_id: _candidate_targets(o, obstacles, config, allow_alternative_viewing)
        for o in obstacles
    }
    leg_cache: dict = {}
    n = len(obstacles)
    orders_considered = 0
    valid_orders = 0
    rejected_orders = 0
    pruned_orders = 0
    best_complete = None  # (cost, order, targets, legs)
    best_partial = None

    for order in itertools.permutations(obstacles):
        orders_considered += 1
        if stats is not None:
            stats.orders_searched += 1
        cost_bound = best_complete[0] if best_complete is not None else None
        best_by_length, was_pruned = _best_chains_for_order(
            start_pose, order, candidates_by_id, obstacles, config, leg_planner,
            leg_cache, cost_bound, stats=stats,
        )
        if n in best_by_length:
            # A complete route was actually found for this order, even if
            # some of its other states were pruned along the way -- it is
            # valid, never pruned or rejected.
            valid_orders += 1
            if stats is not None:
                stats.orders_completed += 1
            cost, targets, legs = best_by_length[n]
            if best_complete is None or cost < best_complete[0]:
                best_complete = (cost, order, targets, legs)
                if stats is not None:
                    stats.incumbent_updates += 1
        else:
            if was_pruned:
                pruned_orders += 1
            else:
                rejected_orders += 1
            if allow_alternative_viewing and best_complete is None and best_by_length:
                max_length = max(best_by_length)
                cost, targets, legs = best_by_length[max_length]
                visited_ids = tuple(o.obstacle_id for o in order[:max_length])
                best_partial = _better_partial(best_partial, max_length, cost, visited_ids, targets, legs)

    if best_complete is not None:
        cost, order, targets, legs = best_complete
        return _complete_result(
            order, targets, legs, cost, orders_considered, valid_orders, rejected_orders, pruned_orders
        )
    if allow_alternative_viewing and best_partial is not None:
        _, cost, visited_ids, targets, legs = best_partial
        return _partial_result(
            obstacles, visited_ids, targets, legs, cost, orders_considered, rejected_orders, pruned_orders
        )
    return _no_route_result(
        "No complete executable route visits all obstacles.",
        orders_considered, rejected_orders, pruned_orders,
    )
