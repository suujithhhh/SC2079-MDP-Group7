"""Rear-axle motion model and swept-footprint collision checks.

Same arc model as Task 1 (`align.simulate`, PROTOCOL.md §3): FL/BL raise the
heading, FR/BR lower it; each turn is a fixed-radius arc of the rear-axle
midpoint. Every move is sampled and the margin-inflated footprint is tested
against axis-aligned rectangles (obstacles, walls) with a separating-axis test.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import config

TURNS = ("FL", "FR", "BL", "BR")
TURN_SIGN = {"FL": 1, "BL": 1, "FR": -1, "BR": -1}   # heading change per degree


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    h: float   # degrees, CCW from +x

    def forward(self) -> tuple[float, float]:
        r = math.radians(self.h)
        return math.cos(r), math.sin(r)

    def right(self) -> tuple[float, float]:
        r = math.radians(self.h)
        return math.sin(r), -math.cos(r)

    def point(self, ahead: float, right: float = 0.0) -> tuple[float, float]:
        """A point `ahead` cm along the heading and `right` cm to the right."""
        (fx, fy), (rx, ry) = self.forward(), self.right()
        return self.x + ahead * fx + right * rx, self.y + ahead * fy + right * ry

    def __str__(self) -> str:
        return f"({self.x:.1f}, {self.y:.1f}, {self.h % 360:.0f} deg)"


@dataclass(frozen=True)
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float
    name: str = ""

    def __post_init__(self) -> None:
        values = (self.x0, self.y0, self.x1, self.y1)
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
            raise ValueError(f"Rect {self.name!r}: coordinates must be finite numbers, got {values}")
        if not (self.x0 < self.x1 and self.y0 < self.y1):
            raise ValueError(f"Rect {self.name!r}: needs x0 < x1 and y0 < y1, got {values}")

    def corners(self) -> list[tuple[float, float]]:
        return [(self.x0, self.y0), (self.x1, self.y0), (self.x1, self.y1), (self.x0, self.y1)]


def parse(command: str) -> tuple[str, int]:
    _, direction, value = command.split(",")
    return direction, int(value)


def apply(pose: Pose, command: str, fraction: float = 1.0) -> Pose:
    """Pose after `fraction` of one MOV command."""
    direction, value = parse(command)
    amount = value * fraction
    if direction in ("F", "B"):
        x, y = pose.point(amount if direction == "F" else -amount)
        return Pose(x, y, pose.h)
    gear = 1 if direction in ("FL", "FR") else -1
    sign = TURN_SIGN[direction]
    radius = config.TURN_RADIUS_CM[direction] / (sign * gear)
    h0 = math.radians(pose.h)
    h1 = h0 + sign * math.radians(amount)
    x = pose.x + radius * (math.sin(h1) - math.sin(h0))
    y = pose.y - radius * (math.cos(h1) - math.cos(h0))
    return Pose(x, y, pose.h + sign * amount)


def simulate(pose: Pose, commands: list[str]) -> Pose:
    for command in commands:
        pose = apply(pose, command)
    return pose


def path_length(commands: list[str]) -> float:
    total = 0.0
    for command in commands:
        direction, value = parse(command)
        total += value if direction in ("F", "B") else math.radians(value) * config.TURN_RADIUS_CM[direction]
    return total


def sweep(pose: Pose, commands: list[str], step_cm: float = 2.0, step_deg: float = 2.0) -> list[Pose]:
    """Poses along the path, including the start and every command's end."""
    poses = [pose]
    for command in commands:
        direction, value = parse(command)
        unit = step_cm if direction in ("F", "B") else step_deg
        n = max(1, math.ceil(value / unit))
        poses += [apply(pose, command, i / n) for i in range(1, n + 1)]
        pose = poses[-1]
    return poses


def footprint(pose: Pose, margin: float = config.MARGIN_CM) -> list[tuple[float, float]]:
    rear, front = -config.REAR_EXTENT_CM - margin, config.FRONT_EXTENT_CM + margin
    half = config.WIDTH_CM / 2 + margin
    return [pose.point(rear, -half), pose.point(front, -half), pose.point(front, half), pose.point(rear, half)]


def _overlap(axis, a, b) -> bool:
    pa = [axis[0] * x + axis[1] * y for x, y in a]
    pb = [axis[0] * x + axis[1] * y for x, y in b]
    return max(pa) > min(pb) and max(pb) > min(pa)


def intersects(polygon: list[tuple[float, float]], rect: Rect) -> bool:
    """Separating-axis test: convex quadrilateral vs axis-aligned rectangle."""
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    if max(xs) <= rect.x0 or min(xs) >= rect.x1 or max(ys) <= rect.y0 or min(ys) >= rect.y1:
        return False
    corners = rect.corners()
    for i in range(2):   # the polygon is a rectangle: two edge normals
        (x0, y0), (x1, y1) = polygon[i], polygon[i + 1]
        if not _overlap((y0 - y1, x1 - x0), polygon, corners):
            return False
    return True


def swept_straight(pose: Pose, command: str, margin: float = config.MARGIN_CM) -> list[tuple[float, float]]:
    """Exact area a straight move sweeps: the footprint stretched along the heading."""
    direction, value = parse(command)
    travel = value if direction == "F" else -value
    rear = -config.REAR_EXTENT_CM - margin + min(0, travel)
    front = config.FRONT_EXTENT_CM + margin + max(0, travel)
    half = config.WIDTH_CM / 2 + margin
    return [pose.point(rear, -half), pose.point(front, -half), pose.point(front, half), pose.point(rear, half)]


def swept_polygons(pose: Pose, commands: list[str], margin: float = config.MARGIN_CM
                   ) -> list[tuple[Pose, list[tuple[float, float]]]]:
    """(pose, polygon) covering the whole path: one stretched rectangle per
    straight (exact), the footprint every 2 deg along each turn."""
    result = [(pose, footprint(pose, margin))]
    for command in commands:
        direction, _ = parse(command)
        if direction in ("F", "B"):
            result.append((pose, swept_straight(pose, command, margin)))
            pose = apply(pose, command)
        else:
            poses = sweep(pose, [command])[1:]
            result += [(p, footprint(p, margin)) for p in poses]
            pose = poses[-1]
    return result


def hit_in(polygons, rects: list[Rect], dx: float = 0.0, dy: float = 0.0) -> tuple[Pose, Rect] | None:
    """First (pose, rect) contact among precomputed polygons shifted by (dx, dy)."""
    for pose, polygon in polygons:
        if dx or dy:
            polygon = [(x + dx, y + dy) for x, y in polygon]
        for rect in rects:
            if intersects(polygon, rect):
                return pose, rect
    return None


def first_hit(pose: Pose, commands: list[str], rects: list[Rect],
              margin: float = config.MARGIN_CM) -> tuple[Pose, Rect] | None:
    """First place along the path whose swept footprint touches a rectangle,
    or None if clear."""
    return hit_in(swept_polygons(pose, commands, margin), rects)


def is_clear(pose: Pose, commands: list[str], rects: list[Rect], extra: float = 0.0) -> bool:
    return first_hit(pose, commands, rects, config.MARGIN_CM + extra) is None


def robustness(pose: Pose, commands: list[str], rects: list[Rect], limit: int = 8) -> int:
    """Largest whole cm of extra margin (0..limit) with which the path stays
    clear; -1 if it is not clear even with the normal margin."""
    if not is_clear(pose, commands, rects):
        return -1
    lo, hi = 0, limit
    while lo < hi:   # clearance is monotone in the margin
        mid = (lo + hi + 1) // 2
        if is_clear(pose, commands, rects, mid):
            lo = mid
        else:
            hi = mid - 1
    return lo


def wrap180(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def reach_bound_cm(commands: list[str], margin_cm: float = config.MARGIN_CM) -> float:
    """A provable radius around the start pose that contains everything the
    margin-inflated footprint can sweep over during `commands`: the rear-axle
    point moves at most `path_length` and every footprint point is within the
    footprint's circumradius of that point. Used to size planning frames so no
    arbitrary 'big enough' sentinel is needed."""
    rear = config.REAR_EXTENT_CM + margin_cm
    front = config.FRONT_EXTENT_CM + margin_cm
    half = config.WIDTH_CM / 2 + margin_cm
    return path_length(commands) + max(math.hypot(front, half), math.hypot(rear, half))


def max_clear_straight_cm(pose: Pose, rects: list[Rect], margin_cm: float, limit_cm: float) -> float:
    """The longest forward straight, in whole cm and at most `limit_cm`, whose
    exact swept footprint (inflated by `margin_cm`) touches none of `rects`.
    0.0 when even the starting footprint is not clear. Clearance is monotone in
    the length, so this bisects."""
    if not math.isfinite(limit_cm) or limit_cm < 0 or not math.isfinite(margin_cm) or margin_cm < 0:
        raise ValueError(f"limit_cm and margin_cm must be finite and non-negative: {limit_cm}, {margin_cm}")

    def clear(n: int) -> bool:
        polygon = swept_straight(pose, f"MOV,F,{n}", margin_cm)
        return not any(intersects(polygon, rect) for rect in rects)

    if not clear(0):
        return 0.0
    lo, hi = 0, int(math.floor(limit_cm))
    if clear(hi):
        return float(hi)
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if clear(mid):
            lo = mid
        else:
            hi = mid
    return float(lo)


def validate_stopping_reserve(stopping_reserve_cm: float) -> float:
    """The worst-case travel AFTER a stop trigger fires: braking distance,
    command/control latency, sensor/filter latency, worst-case overshoot and a
    calibrated error allowance. PROVISIONAL and calibration-dependent (BD-19);
    it must be finite and strictly positive and has no production default."""
    if (not isinstance(stopping_reserve_cm, (int, float)) or isinstance(stopping_reserve_cm, bool)
            or not math.isfinite(stopping_reserve_cm) or stopping_reserve_cm <= 0):
        raise ValueError(f"stopping_reserve_cm must be finite and > 0, got {stopping_reserve_cm!r}")
    return float(stopping_reserve_cm)


def safe_trigger_limit_cm(geometric_clear_cm: float, stopping_reserve_cm: float) -> float:
    """Where a stop may be TRIGGERED so that the robot, which keeps moving for
    up to the stopping reserve, is still PHYSICALLY stopped inside the clear
    region: geometric clear distance minus the reserve. Zero or negative means
    there is no safe trigger distance (the caller fails before motion)."""
    validate_stopping_reserve(stopping_reserve_cm)
    if not math.isfinite(geometric_clear_cm) or geometric_clear_cm < 0:
        raise ValueError(f"geometric_clear_cm must be finite and >= 0, got {geometric_clear_cm!r}")
    return geometric_clear_cm - stopping_reserve_cm
