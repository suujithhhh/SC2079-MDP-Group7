"""Standalone admissible lower bound on the remaining executable route cost.

Foundation for a future prefix-tree B2/B3 obstacle-ordering search: given the
robot's exact current (estimated) pose and the obstacles still to visit, it
returns a value that never exceeds the cost of any executable completion. It
is **not** called by `executable_route_optimizer.py`,
`planning_service_v2.py`, `server.py` or any other live module, and it
changes no route result. It has no timing, budget or deadline logic, and
does not by itself make 4-8 obstacle planning fast.

It is pure: no HTTP, no global mutable state, no wall-clock use. All
results are deterministic and bit-identical for the same logical input.

Cost being bounded
------------------
An executable route's `total_cost` is the sum of its legs'
`STMCommandPlan.total_cost`: over every (compressed) STM primitive,
`primitive_length_cm * (reverse_penalty if B/BL/BR else 1)`. A turn's length
is `R_dir * radians(value)`, the arc travelled by the rear-axle midpoint,
which is the canonical pose reference (PROTOCOL.md §1) and the point the
four radii were measured at (§3). FL (R_FL) and BL (R_BL, reverse) increase
heading; FR (R_FR) and BR (R_BR, reverse) decrease it; F/B never change it.
A leg ends anywhere within the lattice goal tolerance of its target, and the
next leg starts from that exact end pose.

Per-edge bound
--------------
With `m = min(1, rp)`, `c+ = min(R_FL, R_BL*rp)`, `c- = min(R_FR, R_BR*rp)`,
`d` the rear-axle displacement, `t_p`/`t_h` the model tolerances (goal
tolerance plus slack) and `k` the number of tolerance ends (1 from an exact
start pose, 2 from a candidate region to a candidate region):

    P = m * max(0, d - k*t_p)
    D = (theta_b - theta_a) mod 2*pi
    H = min(c+ * max(0, D - k*t_h), c- * max(0, 2*pi - D - k*t_h))
    E = max(P, H)          # never P + H

Admissibility proof
-------------------
1. Per primitive: cost_i = w_i * len_i >= m * len_i >= m * chord_i (an arc
   is never shorter than its chord). By the triangle inequality any leg from
   s to f costs at least m * |f - s|.
2. Heading: only turns rotate. A CCW turn of angle a costs R*w*a >= c+ * a;
   a CW turn costs >= c- * a. With total CCW rotation A and CW rotation B,
   leg cost >= c+*A + c-*B, and A - B is congruent to theta_f - theta_s
   (mod 2*pi). If A - B >= 0 it is at least the smallest non-negative
   rotation into the accepted heading interval (the CCW need); otherwise B
   is at least the CW need. Either way leg cost >= H.
3. Tolerances and slacks: the replayed end of a leg lies within the goal
   tolerance plus the <= 1e-6 cm / 1e-9 rad compression/generation replay
   drift. POSE_SLACK_CM and HEADING_SLACK_RAD exceed that drift by at least
   100x / 1000x and are added at every tolerance end. A later leg also
   starts inside that inflated region of the previous candidate, hence
   k = 2 between regions. Each end reduces the needed displacement/rotation
   by at most its tolerance.
4. Leg cost >= P and leg cost >= H, so leg cost >= max(P, H). The sum is not
   a bound: one FL 90 deg arc costs 38.96 while P + H = 74.0.
5. Route: any completion from p visits the remaining set in some order via
   some candidates w_1..w_r, costing >= E_pose(p, w_1) + sum E_region(w_i,
   w_{i+1}). Held-Karp takes the minimum of that sum over every order and
   every candidate choice (the primary candidate is never assumed), so it
   cannot exceed the completion's cost. `child_lower_bound` is the same
   argument with w_1 fixed.
6. Scope: this holds for every primitive sequence, so lattice
   sub-optimality or state binning cannot break it; collisions only remove
   paths. It bounds planner cost, never time.
7. Arithmetic: magnitudes stay around 1e4 or below, so accumulated floating
   round-off is far below LOWER_BOUND_EPSILON. Each public bound subtracts it
   exactly once and clamps at 0.0; +inf is returned unchanged.
8. Guard: a non-negative switch cost keeps this valid; radii and rp come
   from `PlannerConfig`. Any change letting a primitive cost less than
   w * len, or moving the pose reference off the rear axle, requires this
   proof to be redone.

Infinity
--------
`+inf` means no completion exists under the supplied candidate set (an
obstacle with no candidates), or, in the abstract core, that the caller has
proven an edge impossible. The geometric builder never produces `+inf` from
geometry, and nothing here accepts a leg outcome: an expansion limit, a
timeout, a budget/deadline stop or an unsearched leg is never proof of
impossibility and must never be turned into `+inf` by a caller either.
`+inf` says nothing about partial routes.

Determinism
-----------
Obstacle ids are sorted ascending into canonical groups; each group keeps the
caller's candidate order (primary first). Remaining ids are converted to a
bitmask before any arithmetic, and every loop runs over integers (masks
ascending, set bits ascending, nodes ascending) with strict-`<` minimum
updates, so mapping insertion order and iterable type never change a bit.
"""

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from models import Pose

LOWER_BOUND_EPSILON = 1e-6   # cost units; subtracted once per public returned bound
POSE_SLACK_CM = 1e-4         # >= 100x the 1e-6 cm replay-drift check; per tolerance end
HEADING_SLACK_RAD = 1e-6     # >= 1000x the 1e-9 rad replay-drift check; per tolerance end
MAX_BOUND_OBSTACLES = 8      # this module only; the protocol-v2 service cap is unchanged


class InvalidLowerBoundInputError(ValueError):
    """Raised for malformed input to this module's public API."""


def _is_real(value) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float))


def _validate_pose(pose, name: str) -> None:
    if not isinstance(pose, Pose):
        raise InvalidLowerBoundInputError(f"{name} must be a Pose, got {pose!r}")
    for field in (pose.x, pose.y, pose.heading):
        if not _is_real(field) or not math.isfinite(field):
            raise InvalidLowerBoundInputError(f"{name} must have finite real fields, got {pose!r}")


def _validate_id(value, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise InvalidLowerBoundInputError(f"{name} must be an int, got {value!r}")


def _validate_cost(value, name: str) -> float:
    """A non-negative cost; +inf allowed (caller-proven impossible), never NaN/-inf."""
    if not _is_real(value) or math.isnan(value) or value < 0:
        raise InvalidLowerBoundInputError(
            f"{name} must be a non-negative real or +inf, got {value!r}"
        )
    return float(value)


def _finalise(raw: float) -> float:
    """The single place LOWER_BOUND_EPSILON is applied; +inf passes through."""
    if raw == math.inf:
        return math.inf
    return max(0.0, raw - LOWER_BOUND_EPSILON)


@dataclass(frozen=True)
class LowerBoundModel:
    """Cost-rate and tolerance constants of the per-edge bound. Immutable."""

    position_tolerance_cm: float   # goal position tolerance + POSE_SLACK_CM
    heading_tolerance_rad: float   # radians(goal heading tolerance) + HEADING_SLACK_RAD
    straight_cost_per_cm: float    # m  = min(1, reverse_penalty)
    ccw_cost_per_rad: float        # c+ = min(R_FL, R_BL * reverse_penalty)
    cw_cost_per_rad: float         # c- = min(R_FR, R_BR * reverse_penalty)

    def __post_init__(self):
        for name in ("position_tolerance_cm", "heading_tolerance_rad"):
            value = getattr(self, name)
            if not _is_real(value) or not math.isfinite(value) or value < 0:
                raise InvalidLowerBoundInputError(f"{name} must be finite and >= 0, got {value!r}")
        for name in ("straight_cost_per_cm", "ccw_cost_per_rad", "cw_cost_per_rad"):
            value = getattr(self, name)
            if not _is_real(value) or not math.isfinite(value) or value <= 0:
                raise InvalidLowerBoundInputError(f"{name} must be finite and > 0, got {value!r}")


def lower_bound_model_from_config(config: PlannerConfig = DEFAULT_PLANNER_CONFIG) -> LowerBoundModel:
    """Build the model from the four measured radii, `reverse_penalty` and the
    lattice goal tolerances. Never mutates `config`."""
    if not isinstance(config, PlannerConfig):
        raise InvalidLowerBoundInputError(f"config must be a PlannerConfig, got {config!r}")
    for name in ("turn_radius_fl_cm", "turn_radius_fr_cm", "turn_radius_bl_cm",
                 "turn_radius_br_cm", "reverse_penalty"):
        value = getattr(config, name)
        if not _is_real(value) or not math.isfinite(value) or value <= 0:
            raise InvalidLowerBoundInputError(f"config.{name} must be finite and > 0, got {value!r}")
    for name in ("lattice_goal_position_tolerance_cm", "lattice_goal_heading_tolerance_deg"):
        value = getattr(config, name)
        if not _is_real(value) or not math.isfinite(value) or value < 0:
            raise InvalidLowerBoundInputError(f"config.{name} must be finite and >= 0, got {value!r}")
    rp = config.reverse_penalty
    return LowerBoundModel(
        position_tolerance_cm=config.lattice_goal_position_tolerance_cm + POSE_SLACK_CM,
        heading_tolerance_rad=(
            math.radians(config.lattice_goal_heading_tolerance_deg) + HEADING_SLACK_RAD
        ),
        straight_cost_per_cm=min(1.0, rp),
        ccw_cost_per_rad=min(config.turn_radius_fl_cm, config.turn_radius_bl_cm * rp),
        cw_cost_per_rad=min(config.turn_radius_fr_cm, config.turn_radius_br_cm * rp),
    )


def _validate_edge_args(a, b, model, tolerance_ends) -> None:
    _validate_pose(a, "a")
    _validate_pose(b, "b")
    if not isinstance(model, LowerBoundModel):
        raise InvalidLowerBoundInputError(f"model must be a LowerBoundModel, got {model!r}")
    if (isinstance(tolerance_ends, bool) or not isinstance(tolerance_ends, int)
            or tolerance_ends not in (1, 2)):
        raise InvalidLowerBoundInputError(f"tolerance_ends must be 1 or 2, got {tolerance_ends!r}")


def _positional(a: Pose, b: Pose, model: LowerBoundModel, k: int) -> float:
    distance = math.hypot(b.x - a.x, b.y - a.y)
    return model.straight_cost_per_cm * max(0.0, distance - k * model.position_tolerance_cm)


def _heading(a: Pose, b: Pose, model: LowerBoundModel, k: int) -> float:
    delta = (b.heading - a.heading) % math.tau
    slack = k * model.heading_tolerance_rad
    ccw_need = max(0.0, delta - slack)
    cw_need = max(0.0, (math.tau - delta) - slack)
    return min(model.ccw_cost_per_rad * ccw_need, model.cw_cost_per_rad * cw_need)


def _edge(a: Pose, b: Pose, model: LowerBoundModel, k: int) -> float:
    return max(_positional(a, b, model, k), _heading(a, b, model, k))


def positional_lower_bound(a: Pose, b: Pose, model: LowerBoundModel, tolerance_ends: int) -> float:
    """Raw positional component `P` (no epsilon)."""
    _validate_edge_args(a, b, model, tolerance_ends)
    return _positional(a, b, model, tolerance_ends)


def heading_lower_bound(a: Pose, b: Pose, model: LowerBoundModel, tolerance_ends: int) -> float:
    """Raw heading component `H` (no epsilon)."""
    _validate_edge_args(a, b, model, tolerance_ends)
    return _heading(a, b, model, tolerance_ends)


def pose_to_region_lower_bound(start: Pose, target: Pose, model: LowerBoundModel) -> float:
    """Raw `max(P, H)` for a leg from an exact start pose into `target`'s
    tolerance region (one tolerance end)."""
    _validate_edge_args(start, target, model, 1)
    return _edge(start, target, model, 1)


def region_to_region_lower_bound(source: Pose, target: Pose, model: LowerBoundModel) -> float:
    """Raw `max(P, H)` for a leg starting anywhere in `source`'s tolerance
    region and ending in `target`'s (two tolerance ends)."""
    _validate_edge_args(source, target, model, 2)
    return _edge(source, target, model, 2)


@dataclass(frozen=True)
class HeldKarpTable:
    """Held-Karp suffix table over abstract nodes grouped by obstacle. Immutable.

    `suffix[mask][v]` is the RAW minimum cost, starting in node v's region, to
    visit every group in `mask` (v's own group never in `mask`; such slots are
    `None`). `+inf` means no completion. Build with `build_held_karp_table`.
    """

    group_count: int
    node_groups: tuple          # tuple[int, ...], non-decreasing
    group_nodes: tuple          # tuple[tuple[int, ...], ...], node indices of each group
    suffix: tuple               # tuple[tuple[Optional[float], ...], ...]
    entry_count: int            # live (mask, node) entries
    transition_count: int       # inner (edge + suffix) evaluations; deterministic


def _bits(mask: int):
    group = 0
    while mask:
        if mask & 1:
            yield group
        mask >>= 1
        group += 1


def build_held_karp_table(
    node_groups: Sequence[int], group_count: int, edge: Sequence[Sequence[float]]
) -> HeldKarpTable:
    """Build the raw suffix table. `edge[v][w]` must be an admissible bound on
    a leg from node v's region to node w's; `+inf` only if the caller has
    proven that transition impossible. A group may have no nodes."""
    _validate_id(group_count, "group_count")
    if not 0 <= group_count <= MAX_BOUND_OBSTACLES:
        raise InvalidLowerBoundInputError(
            f"group_count must be in [0, {MAX_BOUND_OBSTACLES}], got {group_count!r}"
        )
    try:
        groups = tuple(node_groups)
    except TypeError as exc:
        raise InvalidLowerBoundInputError(f"node_groups must be iterable: {exc}") from None
    previous = 0
    for g in groups:
        _validate_id(g, "node group")
        if not 0 <= g < group_count or g < previous:
            raise InvalidLowerBoundInputError(
                f"node_groups must be non-decreasing and within [0, {group_count}), got {groups!r}"
            )
        previous = g
    n_nodes = len(groups)
    try:
        rows = tuple(tuple(row) for row in edge)
    except TypeError as exc:
        raise InvalidLowerBoundInputError(f"edge must be a matrix: {exc}") from None
    if len(rows) != n_nodes or any(len(row) != n_nodes for row in rows):
        raise InvalidLowerBoundInputError(f"edge must be a {n_nodes}x{n_nodes} matrix")
    matrix = tuple(
        tuple(_validate_cost(value, f"edge[{v}][{w}]") for w, value in enumerate(row))
        for v, row in enumerate(rows)
    )
    group_nodes = tuple(
        tuple(v for v in range(n_nodes) if groups[v] == g) for g in range(group_count)
    )

    suffix = []
    entries = 0
    transitions = 0
    for mask in range(1 << group_count):
        row = [None] * n_nodes
        for v in range(n_nodes):
            if mask & (1 << groups[v]):
                continue
            entries += 1
            if mask == 0:
                row[v] = 0.0
                continue
            best = math.inf
            edge_row = matrix[v]
            for j in _bits(mask):
                previous_row = suffix[mask ^ (1 << j)]
                for w in group_nodes[j]:
                    transitions += 1
                    value = edge_row[w] + previous_row[w]
                    if value < best:
                        best = value
            row[v] = best
        suffix.append(tuple(row))

    return HeldKarpTable(
        group_count=group_count,
        node_groups=groups,
        group_nodes=group_nodes,
        suffix=tuple(suffix),
        entry_count=entries,
        transition_count=transitions,
    )


def _groups_to_mask(groups: Iterable[int], group_count: int) -> int:
    try:
        items = tuple(groups)
    except TypeError as exc:
        raise InvalidLowerBoundInputError(f"remaining set must be iterable: {exc}") from None
    mask = 0
    for g in items:
        _validate_id(g, "remaining group")
        if not 0 <= g < group_count:
            raise InvalidLowerBoundInputError(f"unknown group {g!r}")
        if mask & (1 << g):
            raise InvalidLowerBoundInputError(f"duplicate remaining group {g!r}")
        mask |= 1 << g
    return mask


def _raw_child(table: HeldKarpTable, first_value: float, node: int, mask_after: int) -> float:
    return first_value + table.suffix[mask_after][node]


def _raw_completion(table: HeldKarpTable, first_edge: Sequence[float], mask: int) -> float:
    if mask == 0:
        return 0.0
    best = math.inf
    for j in _bits(mask):
        mask_after = mask ^ (1 << j)
        for w in table.group_nodes[j]:
            value = _raw_child(table, first_edge[w], w, mask_after)
            if value < best:
                best = value
    return best


def held_karp_completion_bound(
    table: HeldKarpTable, remaining_groups: Iterable[int], first_edge: Sequence[float]
) -> float:
    """Lower bound on visiting every group in `remaining_groups`, where
    `first_edge[w]` bounds the first leg into node w. Epsilon applied once."""
    if not isinstance(table, HeldKarpTable):
        raise InvalidLowerBoundInputError(f"table must be a HeldKarpTable, got {table!r}")
    mask = _groups_to_mask(remaining_groups, table.group_count)
    try:
        first = tuple(first_edge)
    except TypeError as exc:
        raise InvalidLowerBoundInputError(f"first_edge must be a sequence: {exc}") from None
    if len(first) != len(table.node_groups):
        raise InvalidLowerBoundInputError(
            f"first_edge must have {len(table.node_groups)} entries, got {len(first)}"
        )
    first = tuple(_validate_cost(value, f"first_edge[{w}]") for w, value in enumerate(first))
    return _finalise(_raw_completion(table, first, mask))


@dataclass(frozen=True)
class RemainingRouteBound:
    """Geometry-backed remaining-route bound for one candidate set. Immutable.
    Build with `build_remaining_route_bound`."""

    obstacle_ids: tuple         # tuple[int, ...], ascending: canonical group order
    candidates: tuple           # tuple[tuple[Pose, ...], ...], caller's order per group
    model: LowerBoundModel
    table: HeldKarpTable

    def _group(self, obstacle_id) -> int:
        _validate_id(obstacle_id, "obstacle_id")
        for g, known in enumerate(self.obstacle_ids):
            if known == obstacle_id:
                return g
        raise InvalidLowerBoundInputError(f"unknown obstacle_id {obstacle_id!r}")

    def _mask(self, remaining_ids: Iterable[int]) -> int:
        try:
            items = tuple(remaining_ids)
        except TypeError as exc:
            raise InvalidLowerBoundInputError(f"remaining_ids must be iterable: {exc}") from None
        mask = 0
        for obstacle_id in items:
            bit = 1 << self._group(obstacle_id)
            if mask & bit:
                raise InvalidLowerBoundInputError(f"duplicate remaining id {obstacle_id!r}")
            mask |= bit
        return mask

    def lower_bound(self, current_pose: Pose, remaining_ids: Iterable[int]) -> float:
        """Lower bound on completing every obstacle in `remaining_ids` from the
        exact `current_pose`. `0.0` when nothing remains; `+inf` when a
        remaining obstacle has no candidates."""
        _validate_pose(current_pose, "current_pose")
        mask = self._mask(remaining_ids)
        if mask == 0:
            return _finalise(0.0)
        best = math.inf
        for j in _bits(mask):
            mask_after = mask ^ (1 << j)
            for index, target in enumerate(self.candidates[j]):
                node = self.table.group_nodes[j][index]
                first = _edge(current_pose, target, self.model, 1)
                value = _raw_child(self.table, first, node, mask_after)
                if value < best:
                    best = value
        return _finalise(best)

    def child_lower_bound(
        self, current_pose: Pose, obstacle_id: int, candidate_index: int,
        remaining_ids: Iterable[int],
    ) -> float:
        """Lower bound on completing `remaining_ids` when `obstacle_id` (which
        must be in `remaining_ids`) is visited next at
        `candidates[group][candidate_index]`."""
        _validate_pose(current_pose, "current_pose")
        mask = self._mask(remaining_ids)
        group = self._group(obstacle_id)
        bit = 1 << group
        if not mask & bit:
            raise InvalidLowerBoundInputError(
                f"obstacle_id {obstacle_id!r} is not in remaining_ids"
            )
        _validate_id(candidate_index, "candidate_index")
        if not 0 <= candidate_index < len(self.candidates[group]):
            raise InvalidLowerBoundInputError(
                f"candidate_index {candidate_index!r} out of range for obstacle {obstacle_id!r}"
            )
        target = self.candidates[group][candidate_index]
        node = self.table.group_nodes[group][candidate_index]
        first = _edge(current_pose, target, self.model, 1)
        return _finalise(_raw_child(self.table, first, node, mask ^ bit))


def build_remaining_route_bound(
    candidates_by_id: Mapping, config: PlannerConfig = DEFAULT_PLANNER_CONFIG
) -> RemainingRouteBound:
    """Precompute the bound for one candidate set. `candidates_by_id` maps each
    obstacle id to a list/tuple of already safety-filtered viewing poses
    (primary first); an empty list means that obstacle cannot be completed.
    Inputs are copied, never mutated."""
    if not isinstance(candidates_by_id, Mapping):
        raise InvalidLowerBoundInputError(
            f"candidates_by_id must be a mapping, got {type(candidates_by_id).__name__}"
        )
    keys = tuple(candidates_by_id.keys())
    for obstacle_id in keys:
        _validate_id(obstacle_id, "obstacle id")
    if len(keys) > MAX_BOUND_OBSTACLES:
        raise InvalidLowerBoundInputError(
            f"at most {MAX_BOUND_OBSTACLES} obstacles supported, got {len(keys)}"
        )
    obstacle_ids = tuple(sorted(keys))
    if len(set(obstacle_ids)) != len(obstacle_ids):
        raise InvalidLowerBoundInputError(f"duplicate obstacle ids {obstacle_ids!r}")
    model = lower_bound_model_from_config(config)

    candidates = []
    for obstacle_id in obstacle_ids:
        poses = candidates_by_id[obstacle_id]
        if not isinstance(poses, (list, tuple)):
            raise InvalidLowerBoundInputError(
                f"candidates for obstacle {obstacle_id!r} must be a list or tuple of Pose"
            )
        poses = tuple(poses)
        for pose in poses:
            _validate_pose(pose, f"candidate of obstacle {obstacle_id!r}")
        candidates.append(poses)
    candidates = tuple(candidates)

    nodes = tuple(pose for group in candidates for pose in group)
    node_groups = tuple(g for g, group in enumerate(candidates) for _ in group)
    edge = tuple(tuple(_edge(a, b, model, 2) for b in nodes) for a in nodes)
    table = build_held_karp_table(node_groups, len(obstacle_ids), edge)
    return RemainingRouteBound(
        obstacle_ids=obstacle_ids, candidates=candidates, model=model, table=table,
    )
