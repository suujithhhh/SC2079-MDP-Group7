"""Standalone, safety-preserving compression of state-lattice actions.

Merges adjacent `LatticeAction`s (from `state_lattice.plan_lattice_path`)
that share the exact same STM command into fewer, larger commands — e.g.
`F,10 + F,10 + F,10` becomes `F,30` — without changing the physical path
the plan represents. Built entirely on `state_lattice.py`,
`motion_primitives.py` and `swept_collision.py`; it is not called by, and
does not alter, `planner_interface.py`, `planning_service.py`,
`b3_optimizer.py`, the web UI, protocol-v1 responses, or any STM/Android/
RPi code. No STM wire strings, JSON, `mission_id`/`seq`, timing, or `SNAP`
belong here — this module only regroups `LatticeAction`s.

Why grouping alone is not enough
---------------------------------
A merged command's *value* can be a legal STM value (e.g. within 1-999 cm)
while its *swept path* is not: a longer combined move can sweep through
space three short moves individually would not have needed to cross in
one continuous arc/line the same way (different sampling granularity), or
simply happen to be geometrically different from what was checked before.
So every proposed merge is fully re-validated — never assumed safe from
its parts' original validation — and only accepted if:

1. `swept_collision.safe_primitive_samples()` reports the combined command
   safe from the current *exact* output pose (never a stored or quantised
   pose); and
2. replaying that combined command lands within a strict, floating-point
   round-off-only tolerance (`_STRICT_POSITION_TOL_CM`/
   `_STRICT_HEADING_TOL_RAD` — not the planner's 5 cm/5 deg goal
   tolerance) of replaying the original individual actions in that group
   from that same current pose.

If either check fails, the merge is abandoned and the original actions in
that group are instead replayed and independently revalidated one at a
time from the current output pose. Every output command's start/end poses
come from that replay, so — because each step always starts from the
previous step's *exact* output — tiny floating-point differences between
a merged and an unmerged replay can never break chaining between output
actions, even when a merge is rejected partway through a plan.

Command-limit handling
------------------------
This module never redeclares the STM's 1-999 cm / 1-360 deg limits. A
combined value is checked for legality solely via `motion_primitives.
primitive_length_cm(command, combined_value, config)`, catching its
existing `InvalidCommandValueError`. Within one same-command run, whole
original actions are folded into the current group for as long as the
combined value stays legal; the moment adding the next action's value
would become illegal, that group is closed (unsplit — no original action
is ever partially split to "fill" a maximum) and a new group starts.
"""

import math
from typing import Iterable

import motion_primitives
import swept_collision
from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from models import Obstacle, Pose
from state_lattice import LatticeAction, LatticePlan

# Round-off-only tolerance for comparing a merged command's replayed
# endpoint against the corresponding chain of original actions replayed
# from the same starting pose. Both are the same handful of trig
# operations on the same inputs, so any real discrepancy here is floating
# point noise (double precision, values in the tens-to-hundreds range),
# not a geometric difference — this is deliberately far tighter than the
# planner's 3 cm / 5 deg goal-region tolerances, which this module never
# uses.
_STRICT_POSITION_TOL_CM = 1e-6
_STRICT_HEADING_TOL_RAD = 1e-9

# Round-off-only tolerance for comparing scalar cm-scale values (lengths,
# total distance, total cost) between an input plan and what this module
# independently recomputes from it. Same rationale as the pose tolerance
# above: any real discrepancy at this scale is floating-point noise, not a
# structural difference.
_STRICT_VALUE_TOL_CM = 1e-6

# Mirrors state_lattice.py's own (private) direction classification: B/BL/BR
# are the STM's reverse-travel commands (PROTOCOL.md §3), so the same
# reverse_penalty cost rule applies when this module recomputes total_cost
# after regrouping actions.
_REVERSE_ACTIONS = frozenset({"B", "BL", "BR"})


class InvalidLatticePlanError(ValueError):
    """Raised when the input `LatticePlan` is not internally consistent
    (e.g. its action chain does not connect, or its stored final pose
    does not match its last action's end pose) — this indicates a
    malformed plan, not a compression-time failure."""


class CompressionInvariantError(RuntimeError):
    """Raised when compression cannot preserve safety or the represented
    path: an original fallback action could not be revalidated as safe
    when replayed, the final compressed pose materially disagrees with the
    input plan's final pose, or the compressed plan's recomputed total
    distance or total cost disagrees with the (validated) input plan's.
    This should never happen for a genuine `state_lattice.
    plan_lattice_path` result against the same obstacles/config; it exists
    so a bug never silently yields an unsafe, geometrically different, or
    mis-costed plan."""


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


def _expected_unit(command: str) -> str:
    return "cm" if command in motion_primitives.STRAIGHT_ACTIONS else "deg"


def _edge_cost_cm(command: str, length_cm: float, config: PlannerConfig) -> float:
    if command in _REVERSE_ACTIONS:
        return length_cm * config.reverse_penalty
    return length_cm


def _validate_input_plan(plan: LatticePlan, config: PlannerConfig) -> None:
    """Validate every field `compress_lattice_plan` relies on for a
    non-empty, `is_valid` plan, before touching it. Raises
    `InvalidLatticePlanError` for any inconsistency — this function never
    lets a lower-level `motion_primitives` exception escape; it converts
    each into a clear `InvalidLatticePlanError` instead. Called only when
    `plan.is_valid` and `plan.actions` is non-empty (the caller already
    handles failure plans and zero-action plans separately).
    """
    if plan.reason is not None:
        raise InvalidLatticePlanError(
            f"a successful plan must have reason=None, got {plan.reason!r}"
        )
    if plan.final_pose is None:
        raise InvalidLatticePlanError(
            "a successful, non-empty plan must have a final_pose"
        )

    validated_lengths = []
    for index, action in enumerate(plan.actions):
        try:
            expected_length = motion_primitives.primitive_length_cm(
                action.command, action.value, config
            )
        except (
            motion_primitives.UnsupportedActionError,
            motion_primitives.InvalidCommandValueError,
        ) as exc:
            raise InvalidLatticePlanError(
                f"action {index} ({action.command!r}, {action.value!r}) is not "
                f"a valid STM primitive: {exc}"
            ) from exc

        expected_unit = _expected_unit(action.command)
        if action.unit != expected_unit:
            raise InvalidLatticePlanError(
                f"action {index} ({action.command!r}) has unit {action.unit!r}, "
                f"expected {expected_unit!r}"
            )

        if not _values_match_strict(action.length_cm, expected_length):
            raise InvalidLatticePlanError(
                f"action {index} ({action.command!r}, {action.value!r}) has "
                f"length_cm={action.length_cm!r}, expected {expected_length!r}"
            )

        if not action.samples:
            raise InvalidLatticePlanError(f"action {index} has no samples")
        if action.samples[0] != action.start_pose:
            raise InvalidLatticePlanError(
                f"action {index}'s first sample does not equal its start_pose"
            )
        if action.samples[-1] != action.end_pose:
            raise InvalidLatticePlanError(
                f"action {index}'s last sample does not equal its end_pose"
            )

        validated_lengths.append(expected_length)

    for previous, current in zip(plan.actions, plan.actions[1:]):
        if previous.end_pose != current.start_pose:
            raise InvalidLatticePlanError(
                "plan.actions do not chain: one action's end_pose does not "
                "equal the next action's start_pose"
            )
    if plan.actions[-1].end_pose != plan.final_pose:
        raise InvalidLatticePlanError(
            "plan.final_pose does not equal the last action's end_pose"
        )

    if (
        isinstance(plan.total_distance_cm, bool)
        or not isinstance(plan.total_distance_cm, (int, float))
        or not math.isfinite(plan.total_distance_cm)
        or plan.total_distance_cm < 0
    ):
        raise InvalidLatticePlanError(
            f"total_distance_cm must be a finite, non-negative number, got "
            f"{plan.total_distance_cm!r}"
        )
    expected_distance = sum(validated_lengths)
    if not _values_match_strict(plan.total_distance_cm, expected_distance):
        raise InvalidLatticePlanError(
            f"total_distance_cm={plan.total_distance_cm!r} does not match the "
            f"sum of its action lengths ({expected_distance!r})"
        )

    if (
        isinstance(plan.total_cost, bool)
        or not isinstance(plan.total_cost, (int, float))
        or not math.isfinite(plan.total_cost)
        or plan.total_cost < 0
    ):
        raise InvalidLatticePlanError(
            f"total_cost must be a finite, non-negative number, got "
            f"{plan.total_cost!r}"
        )
    expected_cost = sum(
        _edge_cost_cm(action.command, length, config)
        for action, length in zip(plan.actions, validated_lengths)
    )
    if not _values_match_strict(plan.total_cost, expected_cost):
        raise InvalidLatticePlanError(
            f"total_cost={plan.total_cost!r} does not match the cost implied "
            f"by its actions and reverse_penalty ({expected_cost!r})"
        )


def _group_by_command(actions: tuple) -> list:
    """Split into maximal runs of adjacent actions sharing the same command."""
    groups = [[actions[0]]]
    for action in actions[1:]:
        if action.command == groups[-1][-1].command:
            groups[-1].append(action)
        else:
            groups.append([action])
    return groups


def _combined_value_is_legal(command: str, value: int, config: PlannerConfig) -> bool:
    try:
        motion_primitives.primitive_length_cm(command, value, config)
        return True
    except motion_primitives.InvalidCommandValueError:
        return False


def _chunk_by_legal_value(group: list, config: PlannerConfig) -> list:
    """Split one same-command group into maximal runs of whole actions whose
    summed value stays a legal STM value (never split one original action)."""
    command = group[0].command
    chunks = [[group[0]]]
    running_sum = group[0].value
    for action in group[1:]:
        candidate_sum = running_sum + action.value
        if _combined_value_is_legal(command, candidate_sum, config):
            chunks[-1].append(action)
            running_sum = candidate_sum
        else:
            chunks.append([action])
            running_sum = action.value
    return chunks


def _try_merge(
    current_pose: Pose,
    chunk: list,
    obstacles: tuple,
    config: PlannerConfig,
):
    """Attempt to replace `chunk` (>1 same-command actions) with one merged
    LatticeAction from `current_pose`. Returns None if the merge must be
    rejected (unsafe, or fails the strict endpoint-equivalence check)."""
    command = chunk[0].command
    combined_value = sum(action.value for action in chunk)

    merged_samples = swept_collision.safe_primitive_samples(
        current_pose, command, combined_value, obstacles, config
    )
    if merged_samples is None:
        return None
    merged_end_pose = merged_samples[-1]

    replay_pose = current_pose
    for original in chunk:
        replay_pose = motion_primitives.propagate(
            replay_pose, original.command, original.value, config
        )
    if not _poses_match_strict(merged_end_pose, replay_pose):
        return None

    length_cm = motion_primitives.primitive_length_cm(command, combined_value, config)
    return LatticeAction(
        command=command,
        value=combined_value,
        unit=_expected_unit(command),
        start_pose=current_pose,
        end_pose=merged_end_pose,
        length_cm=length_cm,
        samples=merged_samples,
    )


def _replay_original(
    current_pose: Pose,
    original: LatticeAction,
    obstacles: tuple,
    config: PlannerConfig,
) -> LatticeAction:
    """Re-validate and re-propagate one original action from `current_pose`
    (the fallback path when a merge is rejected)."""
    samples = swept_collision.safe_primitive_samples(
        current_pose, original.command, original.value, obstacles, config
    )
    if samples is None:
        raise CompressionInvariantError(
            f"original action {original.command},{original.value} could not be "
            f"revalidated as safe when replayed from the current output pose "
            f"{current_pose!r}; the input plan may not match the given "
            f"obstacles/config"
        )
    end_pose = samples[-1]
    length_cm = motion_primitives.primitive_length_cm(
        original.command, original.value, config
    )
    return LatticeAction(
        command=original.command,
        value=original.value,
        unit=_expected_unit(original.command),
        start_pose=current_pose,
        end_pose=end_pose,
        length_cm=length_cm,
        samples=samples,
    )


def _build_waypoints(start_pose: Pose, actions: list) -> tuple:
    waypoints = [start_pose]
    for action in actions:
        waypoints.extend(action.samples[1:])
    return tuple(waypoints)


def compress_lattice_plan(
    plan: LatticePlan,
    obstacles: Iterable[Obstacle],
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
) -> LatticePlan:
    """Merge adjacent same-command actions in `plan` into fewer STM commands.

    Never mutates `plan` (or its actions/poses), `obstacles`, or `config`.
    A failure plan (`is_valid=False`) or a valid zero-action plan is
    returned unchanged — there is nothing to compress. Otherwise, every
    field this function relies on is validated first (`_validate_input_plan`):
    `reason`, `final_pose`, each action's command/value/unit/length/samples,
    the action chain, and `total_distance_cm`/`total_cost` against what the
    actions actually imply. `obstacles` is consumed into a tuple up front so
    a one-shot iterable can safely be checked against every candidate
    command. Raises `InvalidLatticePlanError` if `plan` is not internally
    consistent, or `CompressionInvariantError` if safety or the represented
    path/cost cannot be preserved (see that class's docstring — should not
    happen for a genuine `state_lattice` result against the same
    obstacles/config).
    """
    if not plan.is_valid or not plan.actions:
        return plan

    _validate_input_plan(plan, config)
    obstacles = tuple(obstacles)

    groups = _group_by_command(plan.actions)
    chunks = []
    for group in groups:
        chunks.extend(_chunk_by_legal_value(group, config))

    output_actions = []
    current_pose = plan.actions[0].start_pose
    for chunk in chunks:
        merged = _try_merge(current_pose, chunk, obstacles, config) if len(chunk) > 1 else None
        if merged is not None:
            output_actions.append(merged)
            current_pose = merged.end_pose
        else:
            for original in chunk:
                replayed = _replay_original(current_pose, original, obstacles, config)
                output_actions.append(replayed)
                current_pose = replayed.end_pose

    if not _poses_match_strict(current_pose, plan.final_pose):
        raise CompressionInvariantError(
            f"compressed final pose {current_pose!r} materially disagrees with "
            f"the input plan's final pose {plan.final_pose!r}"
        )

    total_distance_cm = sum(action.length_cm for action in output_actions)
    total_cost = sum(
        _edge_cost_cm(action.command, action.length_cm, config) for action in output_actions
    )

    if not _values_match_strict(total_distance_cm, plan.total_distance_cm):
        raise CompressionInvariantError(
            f"compressed total_distance_cm={total_distance_cm!r} does not match "
            f"the input plan's total_distance_cm={plan.total_distance_cm!r}"
        )
    if not _values_match_strict(total_cost, plan.total_cost):
        raise CompressionInvariantError(
            f"compressed total_cost={total_cost!r} does not match the input "
            f"plan's total_cost={plan.total_cost!r}"
        )

    waypoints = _build_waypoints(plan.actions[0].start_pose, output_actions)

    return LatticePlan(
        is_valid=True,
        actions=tuple(output_actions),
        waypoints=waypoints,
        total_distance_cm=total_distance_cm,
        total_cost=total_cost,
        expansions=plan.expansions,
        final_pose=current_pose,
        reason=None,
    )
