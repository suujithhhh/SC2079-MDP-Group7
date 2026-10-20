"""Standalone conversion of a compressed lattice plan into exact STM wire commands.

Pipeline: `LatticePlan -> command_compression.compress_lattice_plan() ->
STMCommandPlan` (a tuple of `STMMotionCommand`s, each carrying the exact
STM wire string it represents). Built entirely on `state_lattice.py`,
`command_compression.py`, `motion_primitives.py` and `swept_collision.py`
(via compression); it is not called by, and does not alter,
`planner_interface.py`, `planning_service.py`, `b3_optimizer.py`, the web
UI, protocol-v1 responses, or any STM/Android/RPi code.

Compression is mandatory, not optional
----------------------------------------
`generate_stm_command_plan()` always calls `command_compression.
compress_lattice_plan()` as its sole compression/safety-validation step —
it never accepts an uncompressed `LatticePlan`'s actions directly, and
never re-implements grouping, STM value-limit checks, turn radii, the six
supported directions, or collision rules itself. Malformed input plans
still raise `command_compression.InvalidLatticePlanError`; a merge that
cannot be safely preserved still raises `command_compression.
CompressionInvariantError`. Both propagate unchanged from this module —
they keep exactly the meanings compression already gives them.

Exact wire-string invariant
------------------------------
Every `STMMotionCommand.stm_command` is built directly from that
command's own `direction` and integer `value`, with no intermediate
formatting step that could round or reformat it:
`stm_command = f"MOV,{direction},{value}"` — e.g. `MOV,F,20`, `MOV,FL,30`,
`MOV,BR,10`. No whitespace, no decimal point, no alternate alias
(`FW`/`BW`/etc. never appear here because compression only ever emits the
six real STM directions), and no rounding, because `value` is always the
same positive integer `motion_primitives.primitive_length_cm()` already
validated for that compressed action.

Estimated poses, not measurements
------------------------------------
`start_pose`/`end_pose` on every `STMMotionCommand` are the planner's
modelled, dead-reckoned rear-axle poses — exactly what `state_lattice.py`
and `command_compression.py` already computed and validated. They are
never described as achieved, observed, or measured: nothing in this
module (or anywhere upstream of it) has access to the robot's actual
physical position. See `PROTOCOL.md` §5-§7 for why `ACK` alone can never
upgrade an estimated pose to a measured one.

Boundary with protocol v2
----------------------------
`STMCommandPlan` is a **transport-neutral, single-leg** result: an
ordered tuple of exact wire strings plus the geometry that produced them.
It is deliberately *not* the full protocol-v2 mission response
(`PROTOCOL.md` §7): it carries no `mission_id`, no per-action `seq`, no
JSON envelope, no `SNAP` entries, no obstacle IDs, and no timing estimate
(calibrated or provisional) — those are mission-composition and
RPi-integration concerns for a later stage, not something this module
invents. This module's job ends at "here is one leg's plan as the exact
STM commands it corresponds to."
"""

import math
from dataclasses import dataclass
from typing import Iterable, Optional

import command_compression
import motion_primitives
from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from models import Obstacle, Pose
from state_lattice import LatticeAction, LatticePlan

# Round-off-only tolerances, identical in spirit and magnitude to
# command_compression.py's own (private) constants: any real discrepancy
# at this scale is floating-point noise from re-deriving the same
# geometry, never a structural difference. Never the planner's 5 cm/5 deg
# goal-region tolerance, which this module never uses.
_STRICT_POSITION_TOL_CM = 1e-6
_STRICT_HEADING_TOL_RAD = 1e-9
_STRICT_VALUE_TOL_CM = 1e-6

# Mirrors state_lattice.py's and command_compression.py's own (private)
# direction classification: B/BL/BR are the STM's reverse-travel commands
# (PROTOCOL.md §3), so the same reverse_penalty cost rule applies when
# this module independently re-derives total_cost for verification.
_REVERSE_ACTIONS = frozenset({"B", "BL", "BR"})


class STMCommandGenerationError(RuntimeError):
    """Raised when a compressed `LatticePlan` cannot be turned into a
    verified `STMCommandPlan`: a compressed action's direction/value is no
    longer a valid STM primitive, its stored fields disagree with what
    independent re-verification computes (unit, wire string, replayed
    endpoint, total distance/cost, or final pose), or its action chain is
    broken. This should never happen for a genuine `command_compression.
    compress_lattice_plan()` result against the same obstacles/config; it
    exists so a bug never silently yields an incorrect wire command."""


@dataclass(frozen=True)
class STMMotionCommand:
    """One exact STM wire command, with the estimated geometry it represents.

    Immutable. `direction` is one of `F`/`B`/`FL`/`FR`/`BL`/`BR`; `value`
    is the positive integer STM command value already validated by
    `motion_primitives.primitive_length_cm()`; `unit` is `"cm"` for
    `F`/`B` and `"deg"` for the four turns; `stm_command` is the exact
    wire string `f"MOV,{direction},{value}"`. `start_pose`/`end_pose` are
    estimated, dead-reckoned rear-axle poses — never achieved, observed,
    or measured. `samples` are the exact collision-checked swept samples
    `command_compression.py` validated for this command.
    """

    direction: str
    value: int
    unit: str
    stm_command: str
    start_pose: Pose
    end_pose: Pose
    length_cm: float
    samples: tuple


@dataclass(frozen=True)
class STMCommandPlan:
    """Result of `generate_stm_command_plan`. Immutable.

    A transport-neutral, single-leg result — not the complete protocol-v2
    mission response (see module docstring). `commands` is `()` on
    failure or for a valid zero-command (already-at-goal) plan.
    """

    is_valid: bool
    commands: tuple                 # tuple[STMMotionCommand, ...]; () on failure or zero-action
    waypoints: tuple                # tuple[Pose, ...]; passed through from compression
    total_distance_cm: float
    total_cost: float
    expansions: int
    final_pose: Optional[Pose]      # estimated pose; None on failure
    reason: Optional[str]           # state_lattice.REASON_*; None on success


def _expected_unit(direction: str) -> str:
    return "cm" if direction in motion_primitives.STRAIGHT_ACTIONS else "deg"


def _wire_string(direction: str, value: int) -> str:
    return f"MOV,{direction},{value}"


def _poses_match_strict(a: Pose, b: Pose) -> bool:
    if math.hypot(a.x - b.x, a.y - b.y) > _STRICT_POSITION_TOL_CM:
        return False
    heading_diff = abs(motion_primitives.normalize_heading(a.heading - b.heading))
    return heading_diff <= _STRICT_HEADING_TOL_RAD


def _values_match_strict(a: float, b: float) -> bool:
    return (
        isinstance(a, (int, float)) and isinstance(b, (int, float))
        and not isinstance(a, bool) and not isinstance(b, bool)
        and math.isfinite(a) and math.isfinite(b)
        and abs(a - b) <= _STRICT_VALUE_TOL_CM
    )


def _edge_cost_cm(direction: str, length_cm: float, config: PlannerConfig) -> float:
    if direction in _REVERSE_ACTIONS:
        return length_cm * config.reverse_penalty
    return length_cm


def _build_and_verify_command(action: LatticeAction, config: PlannerConfig) -> STMMotionCommand:
    """Convert one compressed `LatticeAction` into a verified `STMMotionCommand`.

    Independently re-checks, rather than trusting compression: the
    direction/value are still a valid STM primitive; the unit matches the
    direction; and propagating the same integer command from `start_pose`
    with `motion_primitives.propagate()` reproduces `end_pose` within a
    strict, round-off-only tolerance.
    """
    direction = action.command
    value = action.value

    try:
        expected_length = motion_primitives.primitive_length_cm(direction, value, config)
    except (
        motion_primitives.UnsupportedActionError,
        motion_primitives.InvalidCommandValueError,
    ) as exc:
        raise STMCommandGenerationError(
            f"compressed action ({direction!r}, {value!r}) is not a valid STM "
            f"primitive: {exc}"
        ) from exc

    expected_unit = _expected_unit(direction)
    if action.unit != expected_unit:
        raise STMCommandGenerationError(
            f"compressed action {direction!r} has unit {action.unit!r}, "
            f"expected {expected_unit!r}"
        )
    if not _values_match_strict(action.length_cm, expected_length):
        raise STMCommandGenerationError(
            f"compressed action {direction!r},{value!r} has "
            f"length_cm={action.length_cm!r}, expected {expected_length!r}"
        )

    wire = _wire_string(direction, value)

    replay_end = motion_primitives.propagate(action.start_pose, direction, value, config)
    if not _poses_match_strict(replay_end, action.end_pose):
        raise STMCommandGenerationError(
            f"independently propagating {wire} from {action.start_pose!r} gives "
            f"{replay_end!r}, which disagrees with the compressed action's "
            f"end_pose {action.end_pose!r}"
        )

    return STMMotionCommand(
        direction=direction,
        value=value,
        unit=expected_unit,
        stm_command=wire,
        start_pose=action.start_pose,
        end_pose=action.end_pose,
        length_cm=action.length_cm,
        samples=action.samples,
    )


def generate_stm_command_plan(
    plan: LatticePlan,
    obstacles: Iterable[Obstacle],
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
) -> STMCommandPlan:
    """Compress `plan`, then convert every resulting action into one exact
    STM wire command.

    Never mutates `plan` (or its actions/poses), `obstacles`, or `config`.
    `obstacles` is consumed into a tuple up front so a one-shot iterable
    can safely be reused by compression's own per-command safety checks.
    `command_compression.compress_lattice_plan()` is the sole compression/
    safety-validation stage; its `InvalidLatticePlanError` (malformed
    input plan) and `CompressionInvariantError` (safety/path/cost cannot
    be preserved) propagate unchanged. After compression, this function
    raises `STMCommandGenerationError` if its own independent
    re-verification of the compressed result fails (see that class's
    docstring — should not happen for a genuine result).
    """
    obstacles = tuple(obstacles)
    compressed = command_compression.compress_lattice_plan(plan, obstacles, config)

    if not compressed.is_valid:
        return STMCommandPlan(
            is_valid=False,
            commands=(),
            waypoints=(),
            total_distance_cm=0.0,
            total_cost=0.0,
            expansions=compressed.expansions,
            final_pose=None,
            reason=compressed.reason,
        )

    if not compressed.actions:
        return STMCommandPlan(
            is_valid=True,
            commands=(),
            waypoints=compressed.waypoints,
            total_distance_cm=compressed.total_distance_cm,
            total_cost=compressed.total_cost,
            expansions=compressed.expansions,
            final_pose=compressed.final_pose,
            reason=None,
        )

    commands = []
    previous_end = compressed.actions[0].start_pose
    for action in compressed.actions:
        if action.start_pose != previous_end:
            raise STMCommandGenerationError(
                f"compressed action chain is broken: expected start_pose "
                f"{previous_end!r}, got {action.start_pose!r}"
            )
        command = _build_and_verify_command(action, config)
        commands.append(command)
        previous_end = action.end_pose

    if not _poses_match_strict(previous_end, compressed.final_pose):
        raise STMCommandGenerationError(
            f"generated final pose {previous_end!r} disagrees with the "
            f"compressed plan's final pose {compressed.final_pose!r}"
        )

    total_distance_cm = sum(command.length_cm for command in commands)
    if not _values_match_strict(total_distance_cm, compressed.total_distance_cm):
        raise STMCommandGenerationError(
            f"generated total_distance_cm={total_distance_cm!r} disagrees with "
            f"the compressed plan's total_distance_cm={compressed.total_distance_cm!r}"
        )

    total_cost = sum(
        _edge_cost_cm(command.direction, command.length_cm, config) for command in commands
    )
    if not _values_match_strict(total_cost, compressed.total_cost):
        raise STMCommandGenerationError(
            f"generated total_cost={total_cost!r} disagrees with the compressed "
            f"plan's total_cost={compressed.total_cost!r}"
        )

    return STMCommandPlan(
        is_valid=True,
        commands=tuple(commands),
        waypoints=compressed.waypoints,
        total_distance_cm=compressed.total_distance_cm,
        total_cost=compressed.total_cost,
        expansions=compressed.expansions,
        final_pose=compressed.final_pose,
        reason=None,
    )
