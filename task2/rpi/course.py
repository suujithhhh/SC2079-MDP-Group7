"""What the robot believes about the course, in the mission frame.

Mission frame: origin at the start rear-axle pose, +y towards the obstacles.
The obstacles' arrows sit on the course centreline and their faces point
towards the carpark, so their faces are horizontal lines y = const here.

Obstacle 2 is 10 cm deep and its CROSS-SPAN IS UNKNOWN. It is never given a
worst case and never classified as short or long. Its arrow is centred on its
long front face (D-26), so before either end is measured the only thing known
about it is that each end lies at a distance from the arrow inside the
`O2Bounds` the caller supplies (BD-02). A measured end replaces that unknown on
its side only.

Nothing is "free space" merely because it is not modelled. Every region the
robot must not enter, or cannot yet vouch for, is a typed `Boundary`, and all
kinds are HARD for route approval:

  OBSTACLE       O1, and O2's strip (its unmeasured sides run to the lateral
                 envelope);
  CARPARK_WALL   the carpark walls;
  POSSIBLE_WALL  the wall behind O2 (BD-18) and, beyond each MEASURED end, the
                 wall that may stand 50 cm outward from it (D-27);
  UNCERTAIN      beyond each UNMEASURED end, the band in which that end's wall
                 might stand: nothing in it is confirmed free;
  ENVELOPE       just outside the finite lateral planning envelope derived from
                 the span bound plus the end clearance.

Only finite numbers are used. There is no stand-in "very large" coordinate: an
extent is either derived from the supplied bounds or from a route's provable
reach (`geometry.reach_bound_cm`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import config
from geometry import Rect

WALL_THICKNESS_CM = 10.0   # drawn thickness of a wall; only its inner face matters


def _finite(name: str, value: float) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number, got {value!r}")
    return float(value)


class O2End(Enum):
    WEST = "WEST"
    EAST = "EAST"


class BoundaryKind(Enum):
    OBSTACLE = "OBSTACLE"
    CARPARK_WALL = "CARPARK_WALL"
    POSSIBLE_WALL = "POSSIBLE_WALL"
    UNCERTAIN = "UNCERTAIN"
    ENVELOPE = "ENVELOPE"


@dataclass(frozen=True)
class Boundary:
    rect: Rect
    kind: BoundaryKind


@dataclass(frozen=True)
class FreeSpace:
    """Free space around O2. Required input; no production default exists.

    end_clearance_cm     guaranteed free distance outward from each physical end
                         of O2 (confirmed 50 cm, D-27; the caller passes it); it
                         also sizes the lateral planning envelope;
    behind_clearance_cm  free distance behind O2's back face (BD-18);
    side_wall_south_cm   how far south of O2's face a side wall may reach (BD-18);
    walls                False when the arena has NO walls around O2 (team
                         statement, D-75): then the two values above must be None
                         and no possible-wall or uncertain region exists. Build it
                         with `FreeSpace.no_walls()`.
    """

    end_clearance_cm: float
    behind_clearance_cm: float | None
    side_wall_south_cm: float | None
    walls: bool = True

    def __post_init__(self) -> None:
        if _finite("end_clearance_cm", self.end_clearance_cm) <= 0:
            raise ValueError(f"end_clearance_cm must be > 0, got {self.end_clearance_cm!r}")
        if not isinstance(self.walls, bool):
            raise TypeError(f"walls must be a bool, got {self.walls!r}")
        for name in ("behind_clearance_cm", "side_wall_south_cm"):
            value = getattr(self, name)
            if not self.walls:
                if value is not None:
                    raise ValueError(f"{name} has no meaning without walls; pass None, got {value!r}")
            elif _finite(name, value if value is not None else math.nan) <= 0:
                raise ValueError(f"{name} must be > 0, got {value!r}")

    @classmethod
    def no_walls(cls, end_clearance_cm: float = config.O2_END_CLEARANCE_CM) -> "FreeSpace":
        return cls(end_clearance_cm, None, None, walls=False)


@dataclass(frozen=True)
class O2Bounds:
    """Bounds on O2's HALF span (arrow to either end). Required input (BD-02).

    min_half_span_cm  the smallest half span that is certainly true. If no
                      official minimum is known this MUST be 0 (never invented);
    max_half_span_cm  the largest allowed half span (an official upper bound, or
                      an arena-based edge-search bound).
    """

    min_half_span_cm: float
    max_half_span_cm: float

    def __post_init__(self) -> None:
        low = _finite("min_half_span_cm", self.min_half_span_cm)
        high = _finite("max_half_span_cm", self.max_half_span_cm)
        if low < 0 or high <= low:
            raise ValueError(f"need 0 <= min_half_span_cm < max_half_span_cm, got {low}, {high}")


@dataclass
class Course:
    free: FreeSpace
    bounds: O2Bounds
    centreline_x: float = field(default_factory=lambda: -config.START_X_OFFSET_CM)
    o1_cx: float | None = None
    o1_face_y: float | None = None
    o2_cx: float | None = None        # the measured x of O2's arrow
    o2_face_y: float | None = None
    o2_west: float | None = None      # MEASURED physical ends (IR); None = unknown, never a guess
    o2_east: float | None = None

    # -- O1 / carpark ----------------------------------------------------------
    def o1(self) -> Rect | None:
        if self.o1_face_y is None:
            return None
        half = config.O1_SIZE_CM / 2
        return Rect(self.o1_cx - half, self.o1_face_y, self.o1_cx + half,
                    self.o1_face_y + config.O1_SIZE_CM, "O1")

    def back_wall_y(self) -> float:
        return -config.START_REAR_FROM_BACK_WALL_CM

    def carpark_entrance_y(self) -> float:
        return self.back_wall_y() + config.CARPARK_DEPTH_CM

    def carpark_cx(self) -> float:
        return self.centreline_x

    # -- O2 --------------------------------------------------------------------
    @property
    def axis_x(self) -> float:
        """The line O2's ends are symmetric about: the arrow once ranged."""
        return self.o2_cx if self.o2_cx is not None else self.centreline_x

    def lateral_envelope(self) -> tuple[float, float]:
        """The finite sideways extent of the planning area: the axis plus or
        minus the largest allowed half span and the guaranteed end clearance."""
        half = self.bounds.max_half_span_cm + self.free.end_clearance_cm
        return self.axis_x - half, self.axis_x + half

    def o2_end(self, end: O2End) -> float | None:
        return self.o2_west if end is O2End.WEST else self.o2_east

    def set_o2_end(self, end: O2End, x: float) -> None:
        """Record a MEASURED end. Raises ValueError for an impossible
        measurement (not finite, on the wrong side of the arrow, or outside the
        span bounds): the mission fails safe rather than adopting it."""
        if not isinstance(end, O2End):
            raise TypeError(f"end must be an O2End, got {end!r}")
        if self.o2_cx is None:
            raise ValueError("O2 has not been ranged yet, so an end cannot be placed")
        x = _finite("end x", x)
        half = (self.o2_cx - x) if end is O2End.WEST else (x - self.o2_cx)
        # the lower bound allows for the camera's sideways error in the arrow position (D-75)
        low = self.bounds.min_half_span_cm - config.O2_AXIS_TOLERANCE_CM
        if not (low <= half <= self.bounds.max_half_span_cm):
            raise ValueError(f"{end.value} end at x={x:.1f} is {half:.1f} cm from the arrow, outside the allowed "
                             f"half span [{low}, {self.bounds.max_half_span_cm}]")
        if end is O2End.WEST:
            self.o2_west = x
        else:
            self.o2_east = x

    def o2(self) -> Rect | None:
        """O2's strip. An unmeasured side runs to the lateral envelope (finite)."""
        if self.o2_face_y is None or self.o2_cx is None:
            return None
        low, high = self.lateral_envelope()
        west = self.o2_west if self.o2_west is not None else low
        east = self.o2_east if self.o2_east is not None else high
        return Rect(west, self.o2_face_y, east, self.o2_face_y + config.O2_DEPTH_CM, "O2")

    def o2_back_y(self) -> float:
        return self.o2_face_y + config.O2_DEPTH_CM

    def o2_length(self) -> float | None:
        """The measured span, only once both ends are measured. A number to log;
        it is never turned into a class."""
        if self.o2_west is None or self.o2_east is None:
            return None
        return self.o2_east - self.o2_west

    def predicted_other_end(self, measured: O2End) -> float | None:
        """Where the OTHER end is expected, mirrored about the axis (D-34). A
        consistency hint only: it is never stored and never replaces detecting
        the real end."""
        x = self.o2_end(measured)
        return None if x is None else 2 * self.axis_x - x

    # -- boundaries ------------------------------------------------------------
    def boundaries(self, region: Rect | None = None) -> list[Boundary]:
        """Every hard boundary (optionally only those meeting `region`)."""
        low, high = self.lateral_envelope()
        back = self.back_wall_y()
        half = config.CARPARK_WIDTH_CM / 2
        cx = self.carpark_cx()
        top = back + config.CARPARK_DEPTH_CM
        wall = WALL_THICKNESS_CM
        found = [
            Boundary(Rect(low - wall, back - wall, high + wall, back, "carpark back wall"), BoundaryKind.CARPARK_WALL),
            Boundary(Rect(cx - half - wall, back, cx - half, top, "carpark left wall"), BoundaryKind.CARPARK_WALL),
            Boundary(Rect(cx + half, back, cx + half + wall, top, "carpark right wall"), BoundaryKind.CARPARK_WALL),
        ]
        o1 = self.o1()
        if o1 is not None:
            found.append(Boundary(o1, BoundaryKind.OBSTACLE))
        o2 = self.o2()
        if o2 is not None:
            free = self.free
            found.append(Boundary(o2, BoundaryKind.OBSTACLE))
            if not free.walls:
                # no walls: only the finite lateral envelope bounds the area; it reaches as far
                # behind O2 as it reaches sideways from the axis
                behind = self.o2_back_y() + self.bounds.max_half_span_cm + free.end_clearance_cm
            else:
                behind = self.o2_back_y() + free.behind_clearance_cm
                south = self.o2_face_y - free.side_wall_south_cm
                found.append(Boundary(Rect(low - wall, behind, high + wall, behind + wall, "wall behind O2"),
                                      BoundaryKind.POSSIBLE_WALL))
            for end, sign in ((O2End.WEST, -1), (O2End.EAST, 1)) if free.walls else ():
                label = end.value.lower()
                measured = self.o2_end(end)
                if measured is not None:
                    inner = measured + sign * free.end_clearance_cm
                    outer = inner + sign * wall
                    found.append(Boundary(Rect(min(inner, outer), south, max(inner, outer), behind,
                                               f"wall beyond O2 {label} end"), BoundaryKind.POSSIBLE_WALL))
                else:
                    near = self.axis_x + sign * (self.bounds.min_half_span_cm + free.end_clearance_cm)
                    far = self.axis_x + sign * (self.bounds.max_half_span_cm + free.end_clearance_cm)
                    found.append(Boundary(Rect(min(near, far), south, max(near, far), behind,
                                               f"unknown wall region beyond O2 {label} end"), BoundaryKind.UNCERTAIN))
            found.append(Boundary(Rect(low - wall, back - wall, low, behind + wall, "planning envelope (west)"),
                                  BoundaryKind.ENVELOPE))
            found.append(Boundary(Rect(high, back - wall, high + wall, behind + wall, "planning envelope (east)"),
                                  BoundaryKind.ENVELOPE))
        if region is not None:
            found = [b for b in found if b.rect.x0 < region.x1 and b.rect.x1 > region.x0
                     and b.rect.y0 < region.y1 and b.rect.y1 > region.y0]
        return found

    def obstacles(self) -> list[Rect]:
        return [b.rect for b in self.boundaries() if b.kind is BoundaryKind.OBSTACLE]

    def walls(self) -> list[Rect]:
        """Every hard boundary that is not an obstacle: carpark walls, possible
        walls, uncertain regions and the envelope. None of them is optional."""
        return [b.rect for b in self.boundaries() if b.kind is not BoundaryKind.OBSTACLE]

    def summary(self) -> str:
        parts = []
        if self.o1_face_y is not None:
            parts.append(f"O1 face y={self.o1_face_y:.1f} x={self.o1_cx:.1f}")
        if self.o2_face_y is not None:
            def end(x: float | None) -> str:
                return "unmeasured" if x is None else f"{x:.1f}"
            parts.append(f"O2 face y={self.o2_face_y:.1f} x={self.o2_cx:.1f} "
                         f"west end {end(self.o2_west)}, east end {end(self.o2_east)}")
        return "; ".join(parts) or "nothing known"
