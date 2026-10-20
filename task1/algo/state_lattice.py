"""Standalone STM-executable state-lattice search for one collision-free leg.

Plans a path between two rear-axle-midpoint poses using only the six STM
primitives (`F`/`B`/`FL`/`FR`/`BL`/`BR`), each propagated exactly by
`motion_primitives.py` and validated along its whole swept path by
`swept_collision.py`. No Dubins connector, half-steer, fractional command,
pose snapping, or the legacy symmetric 25 cm radius may enter this search.

This module is fully standalone: it is not called by `planner_interface.py`,
`b3_optimizer.py`, `planning_service.py`, or the web UI, and the legacy
Dubins/lattice planner is unaffected by its existence.

Continuous pose vs. quantised search key
-----------------------------------------
Every successor pose comes from `swept_collision.safe_primitive_samples()`'s
own exact final sample — never from re-propagating a quantised pose, and
never snapped to a grid or heading bin. Position/heading binning
(`config.lattice_position_bin_cm`/`lattice_heading_bin_deg`) is used
*only* to build the search-state key for duplicate detection; the
reconstructed path, the goal comparison, every emitted action, and the
next leg's start all use the continuously propagated poses.

Search-state key
-----------------
`(position_bin_x, position_bin_y, heading_bin, previous_action)`. Binning
uses Python's built-in `round()` (banker's rounding: a value exactly
halfway between two bins rounds to the nearest *even* bin index) — this is
deterministic and stable across repeated runs, which is all this key needs
to guarantee; it does not need to match any particular rounding
convention used elsewhere. `previous_action` is the STM command that led
to this state (`"START"` for the initial state, never a real command
name) so that reaching the same quantised pose via two different prior
actions is kept as two distinct search states — this matters once a
calibrated switch-cost model is added later, even though the current cost
model has no direction-change penalty of its own.

Cost and heuristic
-------------------
Forward primitives (`F`/`FL`/`FR`) cost their own geometric length
(`motion_primitives.primitive_length_cm`). Reverse primitives (`B`/`BL`/
`BR`) cost that same length multiplied by `config.reverse_penalty`
(`reverse_penalty` already exists for the legacy planner; no new timing or
switch-cost value is invented here); `reverse_penalty` must be a finite
positive number (validated below) so a reverse step's cost is never zero
or negative and never undefined.

The goal test accepts any pose within `lattice_goal_position_tolerance_cm`
of the ideal goal position, not just the exact point — so plain Euclidean
distance to that point is *not* an admissible cost-to-go: a pose already
inside the tolerance can sit at a positive Euclidean distance while its
true remaining cost is zero. `_heuristic_cm()` corrects this in two steps:

1. Subtract the position tolerance from the Euclidean distance (floored at
   zero), so the heuristic is exactly zero for any pose that already
   satisfies the goal's position test:
   `distance_to_goal_region = max(0, position_distance - lattice_goal_position_tolerance_cm)`.
2. Scale by `min(1.0, reverse_penalty)`. Every primitive's cost is at
   least `length * min(1.0, reverse_penalty)` — forward primitives cost
   `length >= length * min(1.0, reverse_penalty)` trivially, and reverse
   primitives cost `length * reverse_penalty`, which is `>= length *
   min(1.0, reverse_penalty)` whether `reverse_penalty` is above or below
   1. Chord length is itself a lower bound on a primitive's geometric
   length (arc length >= chord), so `length * min(1.0, reverse_penalty)`
   is a lower bound on the *cost* of covering that primitive's own
   straight-line displacement, for every configured `reverse_penalty > 0`
   — including a custom configuration where `0 < reverse_penalty < 1`,
   which the old "every primitive costs at least its chord" claim did not
   cover. Summed along any path and combined with the triangle inequality,
   this keeps the heuristic a valid, non-overestimating lower bound on the
   true remaining cost to enter the goal region.
"""

import heapq
import itertools
import math
from dataclasses import dataclass
from typing import Iterable, Optional

import footprint
import motion_primitives
import swept_collision
from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from models import Obstacle, Pose

REASON_UNSAFE_START = "unsafe_start"
REASON_UNSAFE_GOAL = "unsafe_goal"
REASON_EXPANSION_LIMIT = "expansion_limit"
REASON_NO_PATH = "no_path"

_START_ACTION_KEY = "START"  # never a real command name; the initial state's "previous action"
_REVERSE_ACTIONS = frozenset({"B", "BL", "BR"})


class InvalidLatticeConfigError(ValueError):
    """Raised when a `state_lattice`-specific config value is invalid."""


@dataclass(frozen=True)
class LatticeAction:
    """One executed STM primitive within a plan. Immutable."""

    command: str          # "F" | "B" | "FL" | "FR" | "BL" | "BR"
    value: int            # integer STM command value
    unit: str             # "cm" for F/B, "deg" for turns
    start_pose: Pose
    end_pose: Pose
    length_cm: float      # geometric length: straight cm, or arc length for turns
    samples: tuple        # immutable swept samples, exactly safe_primitive_samples()'s result


@dataclass(frozen=True)
class LatticePlan:
    """Result of `plan_lattice_path`. Immutable."""

    is_valid: bool
    actions: tuple                  # tuple[LatticeAction, ...]; () on failure
    waypoints: tuple                # tuple[Pose, ...]; concatenated swept samples; () on failure
    total_distance_cm: float
    total_cost: float
    expansions: int
    final_pose: Optional[Pose]      # exact reached pose; None on failure
    reason: Optional[str]           # one of REASON_*; None on success


class _Node:
    """Internal, mutable-free-enough search bookkeeping. Not part of the public API."""

    __slots__ = ("pose", "g_cost", "parent", "action", "value", "length_cm", "samples", "key")

    def __init__(self, pose, g_cost, parent, action, value, length_cm, samples, key):
        self.pose = pose
        self.g_cost = g_cost
        self.parent = parent
        self.action = action
        self.value = value
        self.length_cm = length_cm
        self.samples = samples
        self.key = key


def _validate_lattice_config(config: PlannerConfig) -> None:
    straight = config.lattice_straight_step_cm
    if isinstance(straight, bool) or not isinstance(straight, int) or not (1 <= straight <= 999):
        raise InvalidLatticeConfigError(
            f"lattice_straight_step_cm must be an integer in [1, 999], got {straight!r}"
        )

    turn = config.lattice_turn_step_deg
    if isinstance(turn, bool) or not isinstance(turn, int) or not (1 <= turn <= 360):
        raise InvalidLatticeConfigError(
            f"lattice_turn_step_deg must be an integer in [1, 360], got {turn!r}"
        )

    pos_bin = config.lattice_position_bin_cm
    if (isinstance(pos_bin, bool) or not isinstance(pos_bin, (int, float))
            or not math.isfinite(pos_bin) or pos_bin <= 0):
        raise InvalidLatticeConfigError(
            f"lattice_position_bin_cm must be a finite positive number, got {pos_bin!r}"
        )

    heading_bin = config.lattice_heading_bin_deg
    if (isinstance(heading_bin, bool) or not isinstance(heading_bin, int) or heading_bin <= 0
            or 360 % heading_bin != 0):
        raise InvalidLatticeConfigError(
            f"lattice_heading_bin_deg must be a positive integer dividing 360 exactly, "
            f"got {heading_bin!r}"
        )

    for name in ("lattice_goal_position_tolerance_cm", "lattice_goal_heading_tolerance_deg"):
        tol = getattr(config, name)
        if (isinstance(tol, bool) or not isinstance(tol, (int, float))
                or not math.isfinite(tol) or tol < 0):
            raise InvalidLatticeConfigError(
                f"{name} must be a finite non-negative number, got {tol!r}"
            )

    max_expansions = config.lattice_max_expansions
    if (isinstance(max_expansions, bool) or not isinstance(max_expansions, int)
            or max_expansions <= 0):
        raise InvalidLatticeConfigError(
            f"lattice_max_expansions must be a positive integer, got {max_expansions!r}"
        )

    reverse_penalty = config.reverse_penalty
    if (isinstance(reverse_penalty, bool) or not isinstance(reverse_penalty, (int, float))
            or not math.isfinite(reverse_penalty) or reverse_penalty <= 0):
        raise InvalidLatticeConfigError(
            f"reverse_penalty must be a finite number greater than zero, "
            f"got {reverse_penalty!r}"
        )


def _successor_primitives(config: PlannerConfig) -> tuple:
    """Fixed, documented, stable order: F, B, FL, FR, BL, BR."""
    s, t = config.lattice_straight_step_cm, config.lattice_turn_step_deg
    return (("F", s), ("B", s), ("FL", t), ("FR", t), ("BL", t), ("BR", t))


def _position_bin(value_cm: float, bin_cm: float) -> int:
    return round(value_cm / bin_cm)


def _heading_bin(heading_rad: float, bin_deg: int) -> int:
    degrees = math.degrees(motion_primitives.normalize_heading(heading_rad)) % 360.0
    bin_count = 360 // bin_deg
    return round(degrees / bin_deg) % bin_count


def _state_key(pose: Pose, previous_action: str, config: PlannerConfig) -> tuple:
    return (
        _position_bin(pose.x, config.lattice_position_bin_cm),
        _position_bin(pose.y, config.lattice_position_bin_cm),
        _heading_bin(pose.heading, config.lattice_heading_bin_deg),
        previous_action,
    )


def _within_goal_tolerance(pose: Pose, goal: Pose, config: PlannerConfig) -> bool:
    distance_cm = math.hypot(pose.x - goal.x, pose.y - goal.y)
    if distance_cm > config.lattice_goal_position_tolerance_cm:
        return False
    heading_diff_deg = abs(
        math.degrees(motion_primitives.normalize_heading(pose.heading - goal.heading))
    )
    heading_diff_deg = min(heading_diff_deg, 360.0 - heading_diff_deg)
    return heading_diff_deg <= config.lattice_goal_heading_tolerance_deg


def _edge_cost_cm(action: str, length_cm: float, config: PlannerConfig) -> float:
    if action in _REVERSE_ACTIONS:
        return length_cm * config.reverse_penalty
    return length_cm


def _heuristic_cm(pose: Pose, goal: Pose, config: PlannerConfig) -> float:
    """Admissible lower bound on the remaining cost to enter the goal region.

    See the module docstring's "Cost and heuristic" section for the proof
    that this never overestimates, including for 0 < reverse_penalty < 1.
    """
    position_distance = math.hypot(pose.x - goal.x, pose.y - goal.y)
    distance_to_goal_region = max(
        0.0, position_distance - config.lattice_goal_position_tolerance_cm
    )
    return distance_to_goal_region * min(1.0, config.reverse_penalty)


def _reconstruct(node: _Node, start_pose: Pose) -> tuple:
    chain = []
    current = node
    while current.parent is not None:
        chain.append(current)
        current = current.parent
    chain.reverse()

    actions = []
    for n in chain:
        unit = "cm" if n.action in ("F", "B") else "deg"
        actions.append(
            LatticeAction(
                command=n.action,
                value=n.value,
                unit=unit,
                start_pose=n.parent.pose,
                end_pose=n.pose,
                length_cm=n.length_cm,
                samples=n.samples,
            )
        )

    waypoints = [start_pose]
    for action in actions:
        waypoints.extend(action.samples[1:])
    return tuple(actions), tuple(waypoints)


def _failure(reason: str, expansions: int) -> LatticePlan:
    return LatticePlan(
        is_valid=False,
        actions=(),
        waypoints=(),
        total_distance_cm=0.0,
        total_cost=0.0,
        expansions=expansions,
        final_pose=None,
        reason=reason,
    )


def plan_lattice_path(
    start: Pose,
    goal: Pose,
    obstacles: Iterable[Obstacle],
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
) -> LatticePlan:
    """Plan one collision-free STM-executable leg from `start` to `goal`.

    Neither `start`, `goal`, `obstacles` nor `config` is mutated.
    `obstacles` is consumed into a tuple up front so a one-shot iterable
    can safely be checked against every sampled pose of every primitive
    tried during the search.
    """
    _validate_lattice_config(config)
    obstacles = tuple(obstacles)

    if not footprint.pose_is_safe(start, obstacles, config):
        return _failure(REASON_UNSAFE_START, 0)
    if not footprint.pose_is_safe(goal, obstacles, config):
        return _failure(REASON_UNSAFE_GOAL, 0)

    if _within_goal_tolerance(start, goal, config):
        return LatticePlan(
            is_valid=True,
            actions=(),
            waypoints=(start,),
            total_distance_cm=0.0,
            total_cost=0.0,
            expansions=0,
            final_pose=start,
            reason=None,
        )

    primitives = _successor_primitives(config)
    leg = swept_collision._prepare_leg_geometry(obstacles, config)
    tie_breaker = itertools.count()

    start_key = _state_key(start, _START_ACTION_KEY, config)
    start_node = _Node(
        pose=start, g_cost=0.0, parent=None, action=None, value=None,
        length_cm=None, samples=None, key=start_key,
    )
    best_g = {start_key: 0.0}

    open_heap = []
    start_h = _heuristic_cm(start, goal, config)
    heapq.heappush(open_heap, (start_h, next(tie_breaker), start_node))

    expansions = 0
    while open_heap:
        if expansions >= config.lattice_max_expansions:
            return _failure(REASON_EXPANSION_LIMIT, expansions)

        f_cost, _, node = heapq.heappop(open_heap)
        if node.g_cost > best_g.get(node.key, math.inf):
            continue  # stale heap entry superseded by a cheaper route to this key

        expansions += 1

        if _within_goal_tolerance(node.pose, goal, config):
            actions, waypoints = _reconstruct(node, start)
            return LatticePlan(
                is_valid=True,
                actions=actions,
                waypoints=waypoints,
                total_distance_cm=sum(a.length_cm for a in actions),
                total_cost=node.g_cost,
                expansions=expansions,
                final_pose=node.pose,
                reason=None,
            )

        for action, value in primitives:
            samples = swept_collision._safe_primitive_samples_for_leg(
                node.pose, action, value, leg
            )
            if samples is None:
                continue
            end_pose = samples[-1]
            length_cm = motion_primitives.primitive_length_cm(action, value, config)
            edge_cost = _edge_cost_cm(action, length_cm, config)
            new_g = node.g_cost + edge_cost

            successor_key = _state_key(end_pose, action, config)
            if new_g >= best_g.get(successor_key, math.inf):
                continue
            best_g[successor_key] = new_g

            successor = _Node(
                pose=end_pose, g_cost=new_g, parent=node, action=action, value=value,
                length_cm=length_cm, samples=samples, key=successor_key,
            )
            h_cost = _heuristic_cm(end_pose, goal, config)
            heapq.heappush(open_heap, (new_g + h_cost, next(tie_breaker), successor))

    return _failure(REASON_NO_PATH, expansions)
