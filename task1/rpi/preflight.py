"""Pre-flight check: replay the STM commands on a model of the REAL robot.

The planner (algo @ 50c72d4) plans for a 28x28 cm square turning on 25 cm
about its centre. The real robot is 24 x 19 cm, turns about its rear axle,
and the STM's turn radii are 24.5-33.7 cm, so a plan the planner thinks is
safe can still hit something. This replays every MOV in 1 cm / 1 degree
steps with the measured geometry and reports the closest approach to every
obstacle and wall, so unsafe plans are refused before the robot moves.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# Measured robot (RTOSV validation record): rear-axle reference.
FRONT_CM = 24.0
HALF_WIDTH_CM = 9.5
BODY_CENTRE_FORWARD_CM = 12.0     # body centre is 12 cm ahead of the rear axle
CAMERA_FORWARD_CM = 15.25
STM_RADIUS_CM = {"FL": 24.8, "FR": 33.7, "BL": 33.6, "BR": 24.5}
ARENA_CM = 200.0
OBSTACLE_CM = 10.0


@dataclass
class SnapCheck:
    obstacle_id: int
    camera_to_face_cm: float      # straight-line camera -> image-face centre
    off_axis_deg: float           # angle between heading and the face's centre


@dataclass
class PreflightReport:
    obstacle_clearance_cm: float = math.inf
    wall_clearance_cm: float = math.inf
    worst_command: str = ""
    snaps: list[SnapCheck] = field(default_factory=list)

    def problems(self, min_clearance_cm: float, max_view_cm: float, max_off_axis_deg: float) -> list[str]:
        found = []
        if self.obstacle_clearance_cm < min_clearance_cm:
            found.append(f"passes {self.obstacle_clearance_cm:.1f} cm from an obstacle ({self.worst_command})")
        if self.wall_clearance_cm < min_clearance_cm:
            found.append(f"passes {self.wall_clearance_cm:.1f} cm from a wall")
        for snap in self.snaps:
            if snap.camera_to_face_cm > max_view_cm or abs(snap.off_axis_deg) > max_off_axis_deg:
                found.append(f"obstacle {snap.obstacle_id} photo from {snap.camera_to_face_cm:.0f} cm, "
                             f"{snap.off_axis_deg:+.0f} deg off-axis")
        return found


def _body_points(x: float, y: float, heading: float) -> list[tuple[float, float]]:
    c, s = math.cos(heading), math.sin(heading)
    return [(x + f * c - l * s, y + f * s + l * c)
            for f in (0.0, 6.0, 12.0, 18.0, FRONT_CM)
            for l in (-HALF_WIDTH_CM, 0.0, HALF_WIDTH_CM)]


def _gap_to_box(px: float, py: float, ox: float, oy: float) -> float:
    dx = max(ox - px, 0.0, px - ox - OBSTACLE_CM)
    dy = max(oy - py, 0.0, py - oy - OBSTACLE_CM)
    return -1.0 if dx == 0.0 and dy == 0.0 else math.hypot(dx, dy)


FACE_OFFSET = {"N": (5.0, 10.0), "S": (5.0, 0.0), "E": (10.0, 5.0), "W": (0.0, 5.0)}


def check(actions: list[dict], obstacles: list[dict], start_centre: tuple[float, float, float]) -> PreflightReport:
    """`start_centre` is (x_cm, y_cm, heading_deg) of the robot's BODY centre,
    which is where the operator places it (the planner's start point)."""
    cx, cy, heading_deg = start_centre
    h = math.radians(heading_deg)
    x = cx - BODY_CENTRE_FORWARD_CM * math.cos(h)
    y = cy - BODY_CENTRE_FORWARD_CM * math.sin(h)
    by_id = {o["id"]: o for o in obstacles}
    report = PreflightReport()

    def measure(command: str) -> None:
        for px, py in _body_points(x, y, h):
            report.wall_clearance_cm = min(report.wall_clearance_cm, px, py, ARENA_CM - px, ARENA_CM - py)
            for o in obstacles:
                gap = _gap_to_box(px, py, o["x"], o["y"])
                if gap < report.obstacle_clearance_cm:
                    report.obstacle_clearance_cm, report.worst_command = gap, command

    for action in actions:
        if action["type"] == "SNAP":
            o = by_id[action["obstacle_id"]]
            fx, fy = FACE_OFFSET[o["face"]]
            face_x, face_y = o["x"] + fx, o["y"] + fy
            cam_x = x + CAMERA_FORWARD_CM * math.cos(h)
            cam_y = y + CAMERA_FORWARD_CM * math.sin(h)
            bearing = math.atan2(face_y - cam_y, face_x - cam_x)
            off_axis = math.degrees((bearing - h + math.pi) % (2 * math.pi) - math.pi)
            report.snaps.append(SnapCheck(o["id"], math.hypot(face_x - cam_x, face_y - cam_y), off_axis))
            continue
        _, direction, value = action["stm_command"].split(",")
        gear = 1 if direction in ("F", "FL", "FR") else -1
        for _ in range(int(value)):
            if direction in ("F", "B"):
                x += gear * math.cos(h)
                y += gear * math.sin(h)
            else:
                sign = 1 if direction in ("FL", "BL") else -1     # STM: FL/BL raise heading
                radius = STM_RADIUS_CM[direction] / (sign * gear)
                new_h = h + sign * math.radians(1.0)
                x += radius * (math.sin(new_h) - math.sin(h))
                y -= radius * (math.cos(new_h) - math.cos(h))
                h = new_h
            measure(action["stm_command"])
    return report
