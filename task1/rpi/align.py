"""Lateral re-alignment in front of an obstacle, before its SNAP.

The robot steers like a car (no turning on the spot), so it cannot slide
sideways. It shuffles instead, in short bursts of four moves:

    FL t, F s, FR t, B n   (shifts left)     FR t, F s, FL t, B n   (shifts right)

Forward first, into the buffer in front of the robot, so it never reverses
behind where it started (other obstacles may be there): a short turn, a short
straight and an equal counter-turn (heading unchanged), then a straight reverse
back to the same distance from the obstacle.

The forward part is capped (`max_forward_cm`, default 8): at the planner's
25 cm camera standoff the bumper is ~16 cm from the obstacle face, and some
viewing poses are as close as ~20 cm (bumper ~11.6 cm). The straight is
shortened until the burst fits. At t = 5 deg that gives s = 2 cm, ~0.4 cm
sideways per burst (s = 8 cm, ~0.9 cm, needs a 13 cm cap). Short turns alone
shift very little (the shift grows with the square of the angle); the straight
in the middle is what carries the robot sideways.

The Pi measures the offset again after every burst and stops as soon as it
is within the threshold (task1.py `Mission.align`). Before every measurement
the heading is first brought back to the plan's (`heading_fix`), because with
the camera's narrow view 1 deg of yaw looks like ~1 cm of sideways offset.

Geometry is the rear-axle arc model from PROTOCOL.md §3 (same as preflight.py).
"""

from __future__ import annotations

import math

STM_RADIUS_CM = {"FL": 24.8, "FR": 33.7, "BL": 33.6, "BR": 24.5}
DEFAULT_HFOV_DEG = 62.2          # Pi Camera Module v2; calibrate on the robot
DEFAULT_DISTANCE_CM = 25.0       # planner's camera standoff (algo/config.py)


def offset_cm(norm_x: float, distance_cm: float = DEFAULT_DISTANCE_CM,
              hfov_deg: float = DEFAULT_HFOV_DEG) -> float:
    """Image x offset (-1 left edge .. +1 right edge) -> cm to the robot's right
    at the obstacle face, by the pinhole model."""
    return distance_cm * norm_x * math.tan(math.radians(hfov_deg) / 2)


def simulate(commands: list[str]) -> tuple[float, float, float]:
    """Replay MOVs from the origin facing +y. Returns (right_cm, forward_cm, heading change deg)."""
    x, y, h = 0.0, 0.0, math.pi / 2
    for command in commands:
        _, direction, value = command.split(",")
        value = int(value)
        gear = 1 if direction in ("F", "FL", "FR") else -1
        if direction in ("F", "B"):
            x += gear * value * math.cos(h)
            y += gear * value * math.sin(h)
            continue
        sign = 1 if direction in ("FL", "BL") else -1      # FL/BL raise heading
        radius = STM_RADIUS_CM[direction] / (sign * gear)
        new_h = h + sign * math.radians(value)
        x += radius * (math.sin(new_h) - math.sin(h))
        y -= radius * (math.cos(new_h) - math.cos(h))
        h = new_h
    return x, y, math.degrees(h - math.pi / 2)


def heading_fix(error_deg: float, max_deg: int = 5) -> list[str]:
    """Moves that turn the robot by `error_deg` (+CCW), at most `max_deg`: one
    short forward turn, then a straight reverse back to the same distance if
    the turn crept 2 cm or more towards the obstacle."""
    turn = min(max_deg, round(abs(error_deg)))
    if turn == 0:
        return []
    arc = [f"MOV,{'FL' if error_deg > 0 else 'FR'},{turn}"]
    back = round(simulate(arc)[1])
    return arc + ([f"MOV,B,{back}"] if back >= 2 else [])


def burst(shift_cm: float, turn_deg: int = 5, straight_cm: int = 8, max_forward_cm: float = 8.0) -> list[str]:
    """One shuffle towards `shift_cm` (+right): turn, straight, counter-turn
    forward, then a straight reverse back to the same distance. The straight is
    shortened so the robot never goes more than `max_forward_cm` forward; [] if
    even the two turns alone do not fit."""
    if shift_cm == 0:
        return []
    first, second = ("FR", "FL") if shift_cm > 0 else ("FL", "FR")
    for straight in range(straight_cm, -1, -1):
        approach = [f"MOV,{first},{turn_deg}", *([f"MOV,F,{straight}"] if straight else []),
                    f"MOV,{second},{turn_deg}"]
        forward = simulate(approach)[1]
        if forward <= max_forward_cm:
            return [*approach, f"MOV,B,{round(forward)}"]
    return []
