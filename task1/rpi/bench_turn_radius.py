#!/usr/bin/env python3
"""Bench test: measure the real turn radius of each STM turn, by tape measure.

Wheel counts cannot give the radius (the driven rear wheels slip in turns), so
this drives a turn several times in a row and asks you where the rear-axle
midpoint ended up. With the IMU's total heading change that gives the radius
the planner should use (algo/config.py TURN_RADIUS_*_CM).

Each test, e.g. BR:10x9 = nine MOV,BR,10 in a row (about 90 deg):
  1. Put the robot down with room around it. Mark the floor under the middle
     of the rear axle, and lay a strip of tape along its heading (the start line).
  2. Press Enter: the robot runs the turns (no heading fix, PRECISION profile).
  3. Measure the rear-axle midpoint's new position from the start mark:
       forward = along the start line (+ ahead of the start, - behind it)
       left    = square to the start line (+ to the robot's original left, - right)
     and type both in cm. Enter 's' to skip a test, 'r' to redo it.

Radius from the measurement and the IMU angle t (CCW +, rear-axle arc):
  FL: forward = +R sin|t|, left = +R(1 - cos t)    FR: forward = +R sin|t|, left = -R(1 - cos t)
  BL: forward = -R sin|t|, left = -R(1 - cos t)    BR: forward = -R sin|t|, left = +R(1 - cos t) R is the least-squares fit
of both; the chord estimate |p| / (2 sin(|t|/2)) is printed alongside.

usage (on the Pi, robot on the floor, nothing else using the STM):
  ~/mdp/.venv/bin/python bench_turn_radius.py                  # default suite
  ~/mdp/.venv/bin/python bench_turn_radius.py --tests BR:10x9 BL:10x9
  ~/mdp/.venv/bin/python bench_turn_radius.py --dry-run        # list, no motion
Results: ~/mdp/bench_runs/turn_radius_<time>.json (+ the raw STM log next to it).
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from stm_link import TURN_SIGN, StmLink, open_serial

PLANNER_RADIUS_CM = {"FL": 24.8, "FR": 33.7, "BL": 33.6, "BR": 24.5}   # algo/config.py today
DEFAULT_TESTS = ["FL:10x9", "FR:10x9", "BL:10x9", "BR:10x9",
                 "FL:30x3", "FR:30x3", "BL:30x3", "BR:30x3",
                 "FL:90x1", "FR:90x1", "BL:90x1", "BR:90x1"]
PRECISION = {"set_command": "PROFILE,PRECISION", "expected_set_reply": "PROFILE,OK",
             "verify_command": "PROFILE,GET", "expected_verify_reply": "PROFILE,PRECISION"}


def parse_test(text: str) -> tuple[str, int, int]:
    """'BR:10x9' -> ('BR', 10, 9)."""
    direction, rest = text.upper().split(":")
    step, count = rest.split("X")
    if direction not in TURN_SIGN:
        raise argparse.ArgumentTypeError(f"{text}: direction must be FL, FR, BL or BR")
    return direction, int(step), int(count)


def radius_from(direction: str, turned_deg: float, forward_cm: float, left_cm: float) -> tuple[float, float]:
    """(least-squares radius, chord radius) for a rear-axle arc of `turned_deg`
    (CCW +) that ended `forward_cm` / `left_cm` from where it started."""
    t = math.radians(abs(turned_deg))
    gear = 1 if direction in ("FL", "FR") else -1
    side = TURN_SIGN[direction] * gear   # reversing while turning left drifts right
    # unit-radius endpoint for this turn type
    ux, uy = gear * math.sin(t), side * (1 - math.cos(t))
    norm = ux * ux + uy * uy
    fit = (forward_cm * ux + left_cm * uy) / norm if norm else float("nan")
    chord = math.hypot(forward_cm, left_cm) / (2 * math.sin(t / 2)) if t else float("nan")
    return fit, chord


def ask_measurement() -> tuple[float, float] | str:
    while True:
        text = input("  measured forward,left in cm (e.g. 23.5,-31), 's' skip, 'r' redo: ").strip().lower()
        if text in ("s", "r"):
            return text
        try:
            forward, left = (float(v) for v in text.replace(" ", "").split(","))
            return forward, left
        except ValueError:
            print("  two numbers separated by a comma, please")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tests", nargs="+", type=parse_test, default=[parse_test(t) for t in DEFAULT_TESTS],
                        metavar="DIR:STEPxCOUNT", help="e.g. BR:10x9 (default: 10x9, 30x3, 90x1 for each turn)")
    parser.add_argument("--stm", default="/dev/ttyACM0")
    parser.add_argument("--out", type=Path, default=Path.home() / "mdp/bench_runs")
    parser.add_argument("--dry-run", action="store_true", help="list the tests and exit without moving")
    args = parser.parse_args()

    for direction, step, count in args.tests:
        print(f"{direction}:{step}x{count}  = {count} x MOV,{direction},{step}  (~{step * count} deg)")
    if args.dry_run:
        return

    args.out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    raw = open(args.out / f"turn_radius_{stamp}_stm.log", "a")
    stm = StmLink(open_serial(args.stm), raw_log=raw)
    stm.note(f"bench turn radius {stamp}")
    reason = stm.apply_profile(PRECISION)
    if reason:
        raise SystemExit(f"STM profile setup failed: {reason}")

    results, path = [], args.out / f"turn_radius_{stamp}.json"
    queue = list(args.tests)
    while queue:
        direction, step, count = queue[0]
        print(f"\n=== {direction}:{step}x{count} ===  mark the rear-axle midpoint and the start line, then")
        if input("  press Enter to run (s = skip, q = quit): ").strip().lower() in ("s", "q"):
            if input("  really skip/quit? q quits, anything else skips: ").strip().lower() == "q":
                break
            queue.pop(0)
            continue
        stm.note(f"test {direction}:{step}x{count}")
        start = stm.heading()
        failed = None
        for _ in range(count):
            reply = stm.move(f"MOV,{direction},{step}")
            if not reply.ok:
                failed = reply.line or "no reply"
                break
            stm.heading()   # let it settle before the next turn, as in Task 1
        end = stm.heading()
        if failed:
            print(f"  STM stopped the test: {failed}. Fix and redo.")
            continue
        if start is None or end is None:
            print("  no IMU heading from the STM; cannot work out the angle. Redo.")
            continue
        turned = end - start
        print(f"  IMU: turned {turned:+.1f} deg (commanded {TURN_SIGN[direction] * step * count:+d})")
        answer = ask_measurement()
        if answer == "r":
            continue
        queue.pop(0)
        if answer == "s":
            continue
        forward, left = answer
        fit, chord = radius_from(direction, turned, forward, left)
        row = {"test": f"{direction}:{step}x{count}", "direction": direction, "step_deg": step, "count": count,
               "turned_deg": round(turned, 1), "forward_cm": forward, "left_cm": left,
               "radius_fit_cm": round(fit, 1), "radius_chord_cm": round(chord, 1),
               "planner_radius_cm": PLANNER_RADIUS_CM[direction]}
        results.append(row)
        path.write_text(json.dumps(results, indent=1))
        print(f"  radius {fit:.1f} cm (chord {chord:.1f}); planner uses {PLANNER_RADIUS_CM[direction]}")

    raw.close()
    if not results:
        return
    print(f"\n{'test':<10} {'turned':>7} {'fwd':>6} {'left':>6} {'radius':>7} {'chord':>6} {'planner':>8}")
    for r in results:
        print(f"{r['test']:<10} {r['turned_deg']:+7.1f} {r['forward_cm']:+6.1f} {r['left_cm']:+6.1f} "
              f"{r['radius_fit_cm']:7.1f} {r['radius_chord_cm']:6.1f} {r['planner_radius_cm']:8.1f}")
    print(f"\nSaved {path}")


if __name__ == "__main__":
    main()
