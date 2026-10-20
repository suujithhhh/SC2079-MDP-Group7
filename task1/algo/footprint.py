"""Robot footprint geometry about the rear reference point.

The pose (x, y, heading) is the rear-axle midpoint. In the robot's local
frame, +forward runs along the heading and +left is counter-clockwise from
it. The footprint spans [-rear, +front] forward and [-width/2, +width/2]
sideways; the safety margin inflates it once on every side.
"""

import math
from typing import Iterable

from config import PlannerConfig
from models import Obstacle, Pose

Point = tuple[float, float]


def local_bounds(config: PlannerConfig, include_margin: bool = True) -> tuple[float, float, float, float]:
    """Return (rear, front, right, left) local bounds; rear and right are negative."""
    margin = config.safety_margin_cm if include_margin else 0.0
    half_width = config.robot_width_cm / 2
    return (
        -config.robot_rear_extent_cm - margin,
        config.robot_front_extent_cm + margin,
        -half_width - margin,
        half_width + margin,
    )


def local_to_world(pose: Pose, forward: float, left: float) -> Point:
    cosine, sine = math.cos(pose.heading), math.sin(pose.heading)
    return (
        pose.x + forward * cosine - left * sine,
        pose.y + forward * sine + left * cosine,
    )


def footprint_corners(pose: Pose, config: PlannerConfig, include_margin: bool = True) -> list[Point]:
    """Rear-right, front-right, front-left, rear-left corners in world coordinates."""
    rear, front, right, left = local_bounds(config, include_margin)
    return [
        local_to_world(pose, forward, lateral)
        for forward, lateral in ((rear, right), (front, right), (front, left), (rear, left))
    ]


def footprint_in_arena(pose: Pose, config: PlannerConfig) -> bool:
    """True when the margin-inflated footprint lies inside the arena (touching allowed)."""
    return all(
        0 <= x <= config.arena_cm and 0 <= y <= config.arena_cm
        for x, y in footprint_corners(pose, config)
    )


def _axis_overlap(points: list[Point], rectangle: tuple[float, float, float, float], axis: Point) -> bool:
    values = [x * axis[0] + y * axis[1] for x, y in points]
    left, right, bottom, top = rectangle
    other = [
        x * axis[0] + y * axis[1]
        for x, y in ((left, bottom), (right, bottom), (right, top), (left, top))
    ]
    return max(values) > min(other) and max(other) > min(values)


def footprint_hits_obstacle(pose: Pose, obstacle: Obstacle, config: PlannerConfig) -> bool:
    """Separating-axis test of the margin-inflated footprint against the raw obstacle square."""
    points = footprint_corners(pose, config)
    rectangle = (
        obstacle.x,
        obstacle.x + config.obstacle_cm,
        obstacle.y,
        obstacle.y + config.obstacle_cm,
    )
    axes = (
        (1.0, 0.0),
        (0.0, 1.0),
        (math.cos(pose.heading), math.sin(pose.heading)),
        (-math.sin(pose.heading), math.cos(pose.heading)),
    )
    return all(_axis_overlap(points, rectangle, axis) for axis in axes)


def pose_is_safe(pose: Pose, obstacles: Iterable[Obstacle], config: PlannerConfig) -> bool:
    return footprint_in_arena(pose, config) and not any(
        footprint_hits_obstacle(pose, obstacle, config) for obstacle in obstacles
    )


def camera_position(pose: Pose, config: PlannerConfig) -> Point:
    """World position of the camera lens for a rear-reference pose."""
    return local_to_world(pose, config.camera_forward_offset_cm, config.camera_lateral_offset_cm)


# --- Prepared pose check for the swept-path hot loop -------------------------
#
# Everything above is the reference implementation and stays unchanged; it is
# also the oracle for tests/test_collision_hotpath_equivalence.py. The two
# functions below reproduce pose_is_safe()'s decision BIT-FOR-BIT, for
# swept_collision.safe_primitive_samples() only, by doing the same arithmetic
# once instead of once per obstacle:
#
# * the local bounds are the exact local_bounds() expressions, evaluated once
#   per primitive with the guarded margin passed in (no dataclasses.replace);
# * cos/sin of the heading and the four world corners are computed once per
#   pose with local_to_world()'s exact expressions, then shared by the arena
#   check and every obstacle;
# * SAT axes (1, 0) and (0, 1) are evaluated as the corners' axis-aligned
#   bounds against the obstacle's. Once the arena check has passed every
#   corner is finite, and x * 1.0 + y * 0.0 == x for finite floats, so this is
#   the same comparison on the same values, not an approximation. The rotated
#   axes use the reference projection expressions unchanged.
#
# Nothing is cached beyond one prepare_pose_check() call. Any input outside
# the proven domain (non-finite or non-plain numbers) makes
# prepare_pose_check() return None, and the caller must use the reference
# pose_is_safe() path instead.

_EXACT_INT_LIMIT = 2 ** 53


def _plain_finite(value) -> bool:
    """A plain int (not bool) exactly representable as a float, or a finite float."""
    kind = type(value)
    if kind is float:
        return math.isfinite(value)
    if kind is int:
        return -_EXACT_INT_LIMIT <= value <= _EXACT_INT_LIMIT
    return False


def prepare_pose_check(config: PlannerConfig, guard_cm: float, obstacles: tuple):
    """Prepared inputs for pose_is_safe_prepared(), or None to use the reference path.

    The margin is `config.safety_margin_cm + guard_cm`: exactly the value
    swept_collision's reference path stores in its temporary config.
    Returns `(bounds, rects, arena_cm)`.
    """
    extents = (
        config.safety_margin_cm,
        config.robot_rear_extent_cm,
        config.robot_front_extent_cm,
        config.robot_width_cm,
        config.obstacle_cm,
        config.arena_cm,
    )
    if not all(_plain_finite(value) for value in extents) or not _plain_finite(guard_cm):
        return None
    margin = config.safety_margin_cm + guard_cm
    half_width = config.robot_width_cm / 2
    bounds = (
        -config.robot_rear_extent_cm - margin,
        config.robot_front_extent_cm + margin,
        -half_width - margin,
        half_width + margin,
    )
    if not all(_plain_finite(value) for value in bounds):
        return None

    size = config.obstacle_cm
    rects = []
    for obstacle in obstacles:
        left, bottom = obstacle.x, obstacle.y
        if not (_plain_finite(left) and _plain_finite(bottom)):
            return None
        right, top = left + size, bottom + size
        if not (_plain_finite(right) and _plain_finite(top)):
            return None
        rects.append((
            left, right, bottom, top,
            min(left, right), max(left, right), min(bottom, top), max(bottom, top),
        ))
    return bounds, tuple(rects), config.arena_cm


def pose_is_safe_prepared(pose: Pose, bounds: tuple, rects: tuple, arena_cm) -> bool:
    """pose_is_safe()'s decision for inputs from prepare_pose_check() (never None)."""
    rear, front, right, left = bounds
    cosine, sine = math.cos(pose.heading), math.sin(pose.heading)
    px, py = pose.x, pose.y
    # Same corner order and expressions as footprint_corners()/local_to_world().
    x0 = px + rear * cosine - right * sine
    y0 = py + rear * sine + right * cosine
    x1 = px + front * cosine - right * sine
    y1 = py + front * sine + right * cosine
    x2 = px + front * cosine - left * sine
    y2 = py + front * sine + left * cosine
    x3 = px + rear * cosine - left * sine
    y3 = py + rear * sine + left * cosine

    # footprint_in_arena(), touching allowed.
    if not (
        0 <= x0 <= arena_cm and 0 <= y0 <= arena_cm
        and 0 <= x1 <= arena_cm and 0 <= y1 <= arena_cm
        and 0 <= x2 <= arena_cm and 0 <= y2 <= arena_cm
        and 0 <= x3 <= arena_cm and 0 <= y3 <= arena_cm
    ):
        return False
    if not rects:
        return True

    min_x, max_x = min(x0, x1, x2, x3), max(x0, x1, x2, x3)
    min_y, max_y = min(y0, y1, y2, y3), max(y0, y1, y2, y3)
    neg_sine = -sine
    for r_left, r_right, r_bottom, r_top, r_min_x, r_max_x, r_min_y, r_max_y in rects:
        # SAT axes (1, 0) and (0, 1): identical to the reference on finite values.
        if not (max_x > r_min_x and r_max_x > min_x and max_y > r_min_y and r_max_y > min_y):
            continue
        # Axis (cos, sin): the reference `x * axis[0] + y * axis[1]` projection.
        p0 = x0 * cosine + y0 * sine
        p1 = x1 * cosine + y1 * sine
        p2 = x2 * cosine + y2 * sine
        p3 = x3 * cosine + y3 * sine
        q0 = r_left * cosine + r_bottom * sine
        q1 = r_right * cosine + r_bottom * sine
        q2 = r_right * cosine + r_top * sine
        q3 = r_left * cosine + r_top * sine
        if not (max(p0, p1, p2, p3) > min(q0, q1, q2, q3) and max(q0, q1, q2, q3) > min(p0, p1, p2, p3)):
            continue
        # Axis (-sin, cos).
        p0 = x0 * neg_sine + y0 * cosine
        p1 = x1 * neg_sine + y1 * cosine
        p2 = x2 * neg_sine + y2 * cosine
        p3 = x3 * neg_sine + y3 * cosine
        q0 = r_left * neg_sine + r_bottom * cosine
        q1 = r_right * neg_sine + r_bottom * cosine
        q2 = r_right * neg_sine + r_top * cosine
        q3 = r_left * neg_sine + r_top * cosine
        if max(p0, p1, p2, p3) > min(q0, q1, q2, q3) and max(q0, q1, q2, q3) > min(p0, p1, p2, p3):
            return False
    return True


# --- Per-leg split of prepare_pose_check() ------------------------------------
#
# prepare_pose_check() rebuilds every obstacle rectangle for each primitive
# although only the guard-dependent bounds change between primitives of one
# leg. The two functions below are that same computation split in two: the
# obstacle rectangles (constant for a leg) and the per-primitive bounds. They
# use prepare_pose_check()'s exact expressions, so combining them equals it
# bit-for-bit (tests/test_leg_obstacle_geometry_equivalence.py, which keeps
# prepare_pose_check() as the oracle; it never delegates here).
#
# prepare_obstacle_rects() has a narrower domain than prepare_pose_check(): an
# exact PlannerConfig and exact Obstacle instances only. A subclass, a
# duck-typed or property-backed obstacle could return different values on each
# read, so freezing them at leg start would change behaviour; those inputs get
# None and the caller keeps the unchanged per-primitive path.

def prepare_obstacle_rects(config: PlannerConfig, obstacles: tuple):
    """Obstacle rectangles for prepare_primitive_check(), or None if out of domain."""
    if type(config) is not PlannerConfig:
        return None
    extents = (
        config.safety_margin_cm,
        config.robot_rear_extent_cm,
        config.robot_front_extent_cm,
        config.robot_width_cm,
        config.obstacle_cm,
        config.arena_cm,
    )
    if not all(_plain_finite(value) for value in extents):
        return None

    size = config.obstacle_cm
    rects = []
    for obstacle in obstacles:
        if type(obstacle) is not Obstacle:
            return None
        left, bottom = obstacle.x, obstacle.y
        if not (_plain_finite(left) and _plain_finite(bottom)):
            return None
        right, top = left + size, bottom + size
        if not (_plain_finite(right) and _plain_finite(top)):
            return None
        rects.append((
            left, right, bottom, top,
            min(left, right), max(left, right), min(bottom, top), max(bottom, top),
        ))
    return tuple(rects)


def prepare_primitive_check(config: PlannerConfig, guard_cm: float, rects: tuple):
    """prepare_pose_check()'s `(bounds, rects, arena_cm)` from prepared rectangles.

    `config` must be the exact config passed to prepare_obstacle_rects(),
    whose non-None result is `rects`. None means use the reference path.
    """
    if not _plain_finite(guard_cm):
        return None
    margin = config.safety_margin_cm + guard_cm
    half_width = config.robot_width_cm / 2
    bounds = (
        -config.robot_rear_extent_cm - margin,
        config.robot_front_extent_cm + margin,
        -half_width - margin,
        half_width + margin,
    )
    if not all(_plain_finite(value) for value in bounds):
        return None
    return bounds, rects, config.arena_cm
