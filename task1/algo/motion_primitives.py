"""Pure STM-executable motion primitives.

Propagates a rear-axle-midpoint ``Pose`` (models.Pose) through exactly the
six STM commands defined in PROTOCOL.md §4: ``F``, ``B``, ``FL``, ``FR``,
``BL``, ``BR``. Each turn uses its own physically validated radius
(PROTOCOL.md §3, ``PlannerConfig.turn_radius_{fl,fr,bl,br}_cm``) — never a
shared or mirrored radius.

This module is pure geometry: no obstacles, no collision checking, no
search, no HTTP schema, and no timing calibration. It is not wired into
``planner_interface.py`` or any other consumer; the existing (legacy,
symmetric-radius) planner is unaffected by this module's existence.

Kinematic model
----------------
Each turn command corresponds to a fixed steering lock. The lock determines
which side of the robot the turn's centre of curvature sits on, independent
of travel direction; travel direction (forward/backward) separately
determines which way the rear-reference point moves along that circle.
Combining the two signs reproduces PROTOCOL.md §3's directly observed
behaviour (FL/BR share one steering lock, FR/BL share the other):

    steer_sign   = +1 for FL and BR, -1 for FR and BL
    distance_sign = +1 for forward commands (F, FL, FR),
                    -1 for backward commands (B, BL, BR)
    heading_delta = steer_sign * distance_sign * radians(value)

The rear reference point is propagated by rotating its position around the
turn's centre of curvature by exactly ``heading_delta`` (the position and
heading turn together, since the centre is fixed in the chassis frame, not
the world frame). This reduces, for FL at 90 degrees from the origin facing
+x, to the same end pose PROTOCOL.md §7's worked example uses: ``(24.8,
24.8, 90 deg)``.
"""

import math

from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from models import Pose

STRAIGHT_ACTIONS = frozenset({"F", "B"})
TURN_ACTIONS = frozenset({"FL", "FR", "BL", "BR"})
SUPPORTED_ACTIONS = STRAIGHT_ACTIONS | TURN_ACTIONS

# +1 = the steering lock that increases heading when driving forward
#      (PROTOCOL.md §3: FL -> SERVO_MIN_US, BR -> SERVO_MIN_US)
# -1 = the steering lock that decreases heading when driving forward
#      (PROTOCOL.md §3: FR -> SERVO_MAX_US, BL -> SERVO_MAX_US)
_STEER_SIGN = {"FL": 1, "BR": 1, "FR": -1, "BL": -1}

# +1 = forward travel (F, FL, FR), -1 = backward travel (B, BL, BR)
_DISTANCE_SIGN = {"F": 1, "FL": 1, "FR": 1, "B": -1, "BL": -1, "BR": -1}

_RADIUS_ATTR = {
    "FL": "turn_radius_fl_cm",
    "FR": "turn_radius_fr_cm",
    "BL": "turn_radius_bl_cm",
    "BR": "turn_radius_br_cm",
}

# STM wire limits (PROTOCOL.md §4/§12): straight commands are 0-999 cm,
# turn commands are 0-360 degrees. This module additionally requires the
# value to be strictly positive (see _validate_value), so the effective
# accepted range is 1-999 / 1-360.
_MAX_VALUE = {"F": 999, "B": 999, "FL": 360, "FR": 360, "BL": 360, "BR": 360}


class UnsupportedActionError(ValueError):
    """Raised when an action is not one of F/B/FL/FR/BL/BR."""


class InvalidCommandValueError(ValueError):
    """Raised when a command value is not a positive integer within the
    STM wire limit for its action (1-999 cm for F/B, 1-360 deg for turns)."""


class InvalidFractionError(ValueError):
    """Raised when a fraction is not a finite real number in [0.0, 1.0]."""


def normalize_heading(angle: float) -> float:
    """Wrap an angle in radians to the half-open interval [-pi, pi).

    Matches planner_interface._normalise's formula exactly: `pi` wraps to
    `-pi`, and `-pi` itself is already in range and stays `-pi`.
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _validate_action(action: str) -> None:
    if not isinstance(action, str) or action not in SUPPORTED_ACTIONS:
        raise UnsupportedActionError(
            f"unsupported action {action!r}; expected one of "
            f"{sorted(SUPPORTED_ACTIONS)}"
        )


def _validate_value(action: str, value) -> None:
    """Validate a command value against the STM wire limits for `action`.

    `action` must already be validated by `_validate_action` before this is
    called, since the accepted maximum (999 for F/B, 360 for turns) depends
    on which command it is (PROTOCOL.md §4/§12).
    """
    # bool is a subclass of int in Python; reject it explicitly before the
    # isinstance(value, int) check would otherwise accept True/False.
    if isinstance(value, bool) or not isinstance(value, int):
        raise InvalidCommandValueError(
            f"command value must be a positive integer, got {value!r}"
        )
    max_value = _MAX_VALUE[action]
    if not (1 <= value <= max_value):
        raise InvalidCommandValueError(
            f"{action} value must be between 1 and {max_value}, got {value!r}"
        )


def _validate_fraction(fraction) -> None:
    # bool is a subclass of int/float-compatible in comparisons; reject it
    # explicitly, matching _validate_value's treatment of command values.
    if isinstance(fraction, bool) or not isinstance(fraction, (int, float)):
        raise InvalidFractionError(
            f"fraction must be a finite real number in [0.0, 1.0], got {fraction!r}"
        )
    if not math.isfinite(fraction):
        raise InvalidFractionError(
            f"fraction must be finite (no NaN/inf), got {fraction!r}"
        )
    if not (0.0 <= fraction <= 1.0):
        raise InvalidFractionError(
            f"fraction must be in [0.0, 1.0], got {fraction!r}"
        )


def turn_radius_cm(action: str, config: PlannerConfig = DEFAULT_PLANNER_CONFIG) -> float:
    """Return the physically validated radius (cm) for a turn action."""
    _validate_action(action)
    if action not in TURN_ACTIONS:
        raise UnsupportedActionError(f"{action!r} is not a turn action")
    return getattr(config, _RADIUS_ATTR[action])


def turn_center_cm(
    pose: Pose, action: str, config: PlannerConfig = DEFAULT_PLANNER_CONFIG
) -> tuple[float, float]:
    """World-frame instantaneous centre of curvature for a turn action at `pose`.

    Fixed for the whole primitive: `_propagate_core` rotates the rear
    reference (and, by rigidity, every other point on the chassis) around
    this same point by a growing angle as the commanded turn progresses, so
    every point on the footprint follows its own circular arc around it at
    its own fixed radius. Used by both `propagate`/`pose_at_fraction` and
    `swept_collision.py`'s sampling-resolution math, so there is exactly
    one place this geometry is computed.
    """
    _validate_action(action)
    if action not in TURN_ACTIONS:
        raise UnsupportedActionError(f"{action!r} is not a turn action")
    steer_sign = _STEER_SIGN[action]
    radius_cm = turn_radius_cm(action, config)
    return (
        pose.x + steer_sign * radius_cm * (-math.sin(pose.heading)),
        pose.y + steer_sign * radius_cm * math.cos(pose.heading),
    )


def primitive_length_cm(
    action: str, value: int, config: PlannerConfig = DEFAULT_PLANNER_CONFIG
) -> float:
    """Geometric travel length of one primitive.

    Straight commands (F/B) return centimetres directly. Turn commands
    (FL/FR/BL/BR) return the arc length ``radius_cm * radians(value)``.
    Never a completion time or a path-length-as-value shortcut — see
    PROTOCOL.md §7 for why arc length must not be confused with the STM
    command's own degree value.
    """
    _validate_action(action)
    _validate_value(action, value)
    if action in STRAIGHT_ACTIONS:
        return float(value)
    radius_cm = turn_radius_cm(action, config)
    return radius_cm * math.radians(value)


def _propagate_core(
    pose: Pose, action: str, value: int, fraction: float, config: PlannerConfig
) -> Pose:
    """Shared arc geometry for `propagate` (fraction=1.0) and `pose_at_fraction`.

    `action` and `value` must already be validated by the caller. `fraction`
    must already be validated too, except that `propagate` calls this
    directly with the literal `1.0` and skips fraction validation, since
    that value is not user-supplied. There is exactly one implementation of
    the arc equations here; both public entry points scale the same
    (distance or angle) by `fraction` before applying it.
    """
    if fraction == 0.0:
        # Exact fast path: no computation, no renormalisation, so the start
        # pose comes back geometrically (and bit-for-bit) unchanged.
        return Pose(x=pose.x, y=pose.y, heading=pose.heading)

    if action in STRAIGHT_ACTIONS:
        distance_cm = _DISTANCE_SIGN[action] * float(value) * fraction
        new_x = pose.x + distance_cm * math.cos(pose.heading)
        new_y = pose.y + distance_cm * math.sin(pose.heading)
        return Pose(x=new_x, y=new_y, heading=pose.heading)

    steer_sign = _STEER_SIGN[action]
    distance_sign = _DISTANCE_SIGN[action]
    theta_rad = math.radians(value) * fraction
    heading_delta = steer_sign * distance_sign * theta_rad

    heading = pose.heading
    # Centre of curvature: fixed for the whole primitive, independent of
    # fraction (see turn_center_cm's docstring).
    center_x, center_y = turn_center_cm(pose, action, config)

    # The rear reference point rotates around that fixed centre by exactly
    # heading_delta (heading and position turn together in the chassis
    # frame regardless of travel direction).
    rel_x = pose.x - center_x
    rel_y = pose.y - center_y
    cos_d = math.cos(heading_delta)
    sin_d = math.sin(heading_delta)
    rot_x = rel_x * cos_d - rel_y * sin_d
    rot_y = rel_x * sin_d + rel_y * cos_d

    new_x = center_x + rot_x
    new_y = center_y + rot_y
    new_heading = normalize_heading(heading + heading_delta)
    return Pose(x=new_x, y=new_y, heading=new_heading)


def propagate(
    pose: Pose, action: str, value: int, config: PlannerConfig = DEFAULT_PLANNER_CONFIG
) -> Pose:
    """Propagate ``pose`` (rear-axle midpoint) through one whole STM primitive.

    ``pose`` is never mutated (Pose is an immutable dataclass); a new Pose
    is always returned. Raises UnsupportedActionError /
    InvalidCommandValueError on invalid input instead of guessing.
    Equivalent to, and implemented via, ``pose_at_fraction(pose, action,
    value, 1.0, config)``.
    """
    _validate_action(action)
    _validate_value(action, value)
    return _propagate_core(pose, action, value, 1.0, config)


def pose_at_fraction(
    pose: Pose,
    action: str,
    value: int,
    fraction: float,
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
) -> Pose:
    """Continuous progress through one STM primitive, at ``fraction`` in [0, 1].

    ``action``/``value`` are the same integer STM command `propagate` takes
    (same validation, same limits) — a fractional ``fraction`` does not
    mean a fractional STM command; it means "this much of the way through
    executing that one integer command", for continuous swept-path
    sampling. ``fraction=0.0`` returns the start pose geometrically
    unchanged; ``fraction=1.0`` returns exactly what `propagate` would.
    Rejects non-finite fractions, booleans, and values outside [0.0, 1.0].
    """
    _validate_action(action)
    _validate_value(action, value)
    _validate_fraction(fraction)
    return _propagate_core(pose, action, value, float(fraction), config)


# --- Prepared uniform sampling for swept_collision ---------------------------
#
# Everything above is the reference implementation and stays unchanged; it is
# also the oracle for tests/test_primitive_sampling_equivalence.py. The two
# private helpers below serve swept_collision._sample_and_guard_cm() only.
# They reproduce
#     tuple(pose_at_fraction(pose, action, value, i / n, config)
#           for i in range(n + 1))
# bit-for-bit, computing _propagate_core's per-primitive constants (signed
# distance, start-heading cos/sin, radians(value), turn centre, relative
# rear-reference position) once instead of once per sample. Every per-sample
# expression is _propagate_core's own, in the same evaluation order.

_PLAIN_NUMBERS = (int, float)


def _uniform_sampling_supported(pose, action, value, config) -> bool:
    """True only for the planner's exact runtime types, where hoisting is unobservable.

    Anything else (Pose/PlannerConfig subclasses, duck-typed configs,
    str/int subclasses, non-plain pose fields or turn radius) must use the
    original pose_at_fraction() path, preserving its exact access and
    exception behaviour.
    """
    if not (type(pose) is Pose and type(config) is PlannerConfig
            and type(action) is str and type(value) is int):
        return False
    if not (type(pose.x) in _PLAIN_NUMBERS and type(pose.y) in _PLAIN_NUMBERS
            and type(pose.heading) in _PLAIN_NUMBERS):
        return False
    if action in TURN_ACTIONS:
        return type(getattr(config, _RADIUS_ATTR[action])) in _PLAIN_NUMBERS
    return action in STRAIGHT_ACTIONS


def _uniform_primitive_samples(
    pose: Pose, action: str, value: int, interval_count: int, config: PlannerConfig
) -> tuple:
    """Samples at fractions i / interval_count, i = 0..interval_count, bit-exact.

    Strict private preconditions: `action` and `value` were already validated
    by pose_at_fraction() (this does not validate them again), and
    `_uniform_sampling_supported(pose, action, value, config)` is True.
    `interval_count` is this helper's own argument and is validated here.
    """
    if isinstance(interval_count, bool) or not isinstance(interval_count, int) or interval_count < 1:
        raise ValueError(f"interval_count must be a positive int, got {interval_count!r}")

    samples = []
    if action in STRAIGHT_ACTIONS:
        signed_value = _DISTANCE_SIGN[action] * float(value)
        cos_h = math.cos(pose.heading)
        sin_h = math.sin(pose.heading)
        for i in range(interval_count + 1):
            fraction = i / interval_count
            if fraction == 0.0:
                samples.append(Pose(x=pose.x, y=pose.y, heading=pose.heading))
                continue
            # _propagate_core: (sign * float(value)) * fraction, then x + d * cos.
            distance_cm = signed_value * fraction
            new_x = pose.x + distance_cm * cos_h
            new_y = pose.y + distance_cm * sin_h
            samples.append(Pose(x=new_x, y=new_y, heading=pose.heading))
        return tuple(samples)

    steer_sign = _STEER_SIGN[action]
    distance_sign = _DISTANCE_SIGN[action]
    radians_value = math.radians(value)
    heading = pose.heading
    center_x, center_y = turn_center_cm(pose, action, config)
    rel_x = pose.x - center_x
    rel_y = pose.y - center_y
    for i in range(interval_count + 1):
        fraction = i / interval_count
        if fraction == 0.0:
            samples.append(Pose(x=pose.x, y=pose.y, heading=pose.heading))
            continue
        theta_rad = radians_value * fraction
        heading_delta = steer_sign * distance_sign * theta_rad
        cos_d = math.cos(heading_delta)
        sin_d = math.sin(heading_delta)
        rot_x = rel_x * cos_d - rel_y * sin_d
        rot_y = rel_x * sin_d + rel_y * cos_d
        new_x = center_x + rot_x
        new_y = center_y + rot_y
        new_heading = normalize_heading(heading + heading_delta)
        samples.append(Pose(x=new_x, y=new_y, heading=new_heading))
    return tuple(samples)
