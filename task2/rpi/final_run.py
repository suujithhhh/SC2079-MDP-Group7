#!/usr/bin/env python3
"""Task 2, the final algorithm, built up in stages (team instruction 2026-10-13). SUPERVISED robot runs only.

  1. drive forward until the front ultrasonic reads STOP_CM (default 30 cm) from obstacle 1;
  2. read the arrow on obstacle 1 with the camera (one inference on one frame, a second only if the first finds
     no arrow at config.MIN_CONFIDENCE; still none -> FAIL_SAFE);
  3. go round obstacle 1 on the arrow's side with an S-curve into a lane beside it, still facing north
     (the S-curve and the lane straight past O1 are sweep-checked against O1);
  4. drive forward until the ultrasonic reads O2_READ_CM (default 60 cm) from obstacle 2;
  5. read the arrow on obstacle 2 (O2 is always centred on O1, team rule, so O2's mid-point is O1's centre):
     from the lane the arrow is off to the side, so the robot first turns inward (towards O1's line) by about atan(lane offset / distance), 12 deg more if it is still not seen,
     then undoes the turn with the reverse turn of the same arc (FR+BL / FL+BR retrace each other), so it
     ends where it was, facing north;
  6. turn onto O2's front face towards O2's arrow side, with the side IR level with the arrow (the arrow is
     at O2's mid-point, the one place certainly on O2: its length is unknown) and FIR_TRACK_READING_CM from
     the face;
  7. wall hug the front face with MOV,FIRL / MOV,FIRR: the STM stops at the end of O2;
  8. U-turn round that end onto the back face, the side IR back beside O2;
     (both arrows the same way: the lane is already on O2's arrow side. If the bullseye on O2's front face on
     that side was seen with O2's arrow, O2's near end is taken from it, BULLSEYE_TO_END_CM further out, and
     6-7 are skipped: the robot goes from the lane straight round that end onto the back face)
  9. wall hug the back face the other way, to O2's other end;
 10. turn 90 deg round that end to face the carpark, and stop.

    cd ~/mdp_new/rpi_task2/runtime
    ~/mdp/.venv/bin/python final_run.py

--c-curve (stages 1-3, team instruction 2026-10-14): 1 and 2 as above, then ONE fixed "C" curve round O1 per side
(C_CURVE_MOVES: out-turn, straight, a large swing-back turn, out-turn; it ends heading north again) and stop, with
no correction. The commands were set on the laptop from two robot runs (see C_CURVE_MOVES): the Pi sends them as
they are (one quick sweep check refuses them if the camera puts O1 more than C_FIXED_O1_TOL_CM off the robot's
line). They aim to end on O1's centre line, and need an O1->O2 gap of about 71 cm (not the 60 cm minimum).
Then, facing O2: to 34.2 cm by the ultrasonic, O2's arrow (one inference), and ONE 90 deg forward turn towards it
(O2_TURN, started at O2_TURN_US_CM) that ends parallel to O2's front face, about 10-12 cm from it. Then round O2
(part 3, fixed commands): if the side IR still sees O2, wall hug the front face to its end; if not (the turn carried
it past a short O2's end), reverse in ONE move to REVERSE_INSIDE_CM short of O2's shortest possible end (reverse_to_o2;
5 cm steps if the IR still sees nothing) and edge-follow forward until O2 is gone. Then the fixed U-turn
onto the back face (U_TURN_MOVES), wall hug it to O2's other end and the 90 deg turn to face the carpark
(FACE_CARPARK_TURN, after the shortest clear lead-in); stop. --no-wall-hug stops after the turn onto O2.

--android: start from the tablet instead of Enter. The Pi listens on Bluetooth RFCOMM channel 1 (stop the
checkpoint bridge first: sudo systemctl stop rfcomm), sends MSG,[Pi ready: press START] once the camera and the STM
are ready, and starts on START, ALG:START or RESUME (the Task 1 words); the result goes back as one MSG line.

Forward straights of FAST_STRAIGHT_CM (70) or more run in the TASK2TEST profile, everything else in --profile (PRECISION):
the Pi sets the profile before and after each such straight (--no-fast-straights turns it off).
Back into the carpark: one straight by the shorter of the ultrasonic and the Pi's own tracked distance to the back wall when
they agree within 5 cm (--no-one-shot-park turns it off; the robot's back is placed 5 cm from the wall at the start).

Ctrl-C sends STOP (not bench-validated while moving): someone stays at the power switch. Every planned
move is sweep-checked before it is sent; any problem stops the run with no further movement.
"""

from __future__ import annotations

import argparse
import itertools
import math
import queue
import signal
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import quote, urlparse

import config
from geometry import Pose, Rect, apply, hit_in, is_clear, path_length, simulate, swept_polygons
from ir import IrSensor, IrStatus
from perception import decide_arrow, kind_of, largest, lateral_cm
from stm_link import MAX_HEADING_FIX_DEG, corrected_turn

STOP_CM = 30.0                  # ultrasonic face -> O1 face where O1's arrow is read
O2_READ_CM = 60.0               # ultrasonic face -> O2 face where O2's arrow is read
STOP_TOLERANCE_CM = 1.5
APPROACH_MAX_STEPS = 6
APPROACH_REMEASURE_CM = 60.0    # further than this: drive all but APPROACH_SHORT_CM, then measure again
APPROACH_SHORT_CM = 15.0
US_JUMP_CM = 8.0                # a reading this far from (last reading - the straight just driven) is taken again
US_MISSED_ECHO_CM = 60.0        # ... and, still longer than that and beyond this, taken as a missed echo
FIRST_READING_MAX_CM = 300.0    # a start reading must be an obstacle straight ahead
EXTRA_MARGIN_CM = 2.0           # on top of config.MARGIN_CM for every planned move
# Long forward straights drive in the faster TASK2TEST profile (team-tested accurate for distance, 2026-10-15); every
# other move, turns and edge-follows included, stays in the base profile. The Pi switches just before such a straight
# and back right after it. The firmware starts its slow-down 80 cm before the end in TASK2TEST (60 cm in PRECISION), so
# a shorter straight never reaches the higher speed and gains nothing.
FAST_STRAIGHT_CM = 70
FAST_PROFILE = "TASK2TEST"
ARROW_LOOKS = 2                 # one inference decides each arrow; a second only if the first finds no arrow at
                                # config.MIN_CONFIDENCE (team choice 2026-10-14: a clear RIGHT arrow on O1 scored 0.38)
INWARD_RETRY = False            # the lane route's extra, wider second look at O2 stays off
PRE_REVERSES_CM = (0, 5, 10, 15, 20)   # a short reverse first, only if no S-curve fits without one
INWARD_MIN_DEG, INWARD_MAX_DEG = 5, 30   # the inward turn for reading O2 (team instruction 2026-10-13)
INWARD_MORE_DEG = 12            # extra turn when the arrow is still not in view
LANE_RUN_CM = 80
LANE_OFFSET_CM = 31.0           # lane centre -> O1 centre: room for a reverse inward turn of up to 42 deg beside
                                # O1 (BR needs 30, BL 28; offline sweep, margin 3 + 2 cm)                # the lane must stay clear of O1 this far on (the robot then drives past O1)
# --- wall hug round O2 (stages 6-10) ---
HUG_CM = config.FIR_TRACK_READING_CM   # side IR face -> O2 face while hugging
FIR_CAP_CM = 150                # PROVISIONAL: the most a hug may drive; a missed edge drives this far
FRONT_START_STEP_CM = 6.0       # front hug start points tried, from 3 cm short of the arrow outward along O2
O2_CENTRE_CHECK_CM = 6.0        # warn when O2's arrow is seen this far from O1's centre (O2 is centred on O1)
US_CONE_HALF_DEG = 15.0         # PROVISIONAL (HC-SR04 datasheet): what the ultrasonic saw lies within this cone
BACK_START_CM = tuple(range(14, 81, 6))   # back hug starts with the side IR this far in from the near end:
                                # further in pulls the U-turn's last arc inward, away from a wall
FAR_END_MARGIN_CM = 10.0        # ... but at least this far short of the far end O2's mid-point arrow implies
REANCHOR_MAX_CM = 8.0           # a static IR reading may move O2's face at most this far from the plan
END_WALL_CM = config.O2_END_CLEARANCE_CM   # a wall may stand this far beyond each end of O2 (briefing)
O2_MIN_HALF_CM = 10.0           # O2 reaches at least this far each side of its arrow (team statement 2026-10-14:
                                # O2 is at least 20 cm long)
WALL_REACH_CM = 60.0            # such a wall is assumed to reach this far north and south of O2
UNKNOWN_CM = 400.0              # O2 is assumed to run on this far past an end not yet measured
PRE_LIMIT_CM, MID_LIMIT_CM = 60.0, 150.0   # straights the planner may put before / between the arcs
ARC_STEP_DEG = 3
S_CURVE_DEG = range(6, 91, 6)   # sidestep S-curves the planner may start with
FINAL_STRAIGHT_CM = (0, 10, 20, 30, 45)    # straight along the new heading at the end of a planned turn
FINAL_CLEAR_CM = range(0, 61)   # straight past O2's second end before the final turn
THREE_POINT_DEG = range(20, 80, 5), range(10, 61, 5)   # (forward arc, reverse arc) of a three-point turn
# same arrow side twice: O2's near end from the bullseye on its front face, no front-face hug
BULLSEYE_TO_END_CM = 5.0        # team value 2026-10-14 (not measured): front-face bullseye centre -> O2's end
BULLSEYE_TOL_CM = 3.0           # PROVISIONAL camera error allowed on that end: O2 longer, a wall nearer, by this
                                # (5 leaves no clear route round O2's left end with the wider right turns)
BULLSEYE_MIN_FROM_ARROW_CM = 10.0   # a bullseye nearer the arrow than this is not taken as an end marker
END_FACE_LEAD_CM = 10.0         # the end-face edge-follow starts with the side IR this far short of O2's front face
END_FACE_CAP_CM = 40            # ... and drives at most this far (lead + 10 cm face + the STM's 4+4 scans + margin)
END_FACE_STOP_PAST_CM = 5.0     # where it is expected to stop past the back corner (planning the U-turn ahead)


ARROW_SERVER_PORT = 5001         # arrow_server.py's default port on the PC


class ArrowUploader:
    """Sends each arrow picture (the RAW frame with its bounding boxes) to arrow_server.py on the PC, in a
    background thread, so a slow or absent PC never holds up the robot. Failures are logged, never raised."""

    def __init__(self, pc: str, run_id: str, log=print, timeout_s: float = 5.0) -> None:
        base = pc if "://" in pc else f"http://{pc}"
        if urlparse(base).port is None:
            base = f"{base.rstrip('/')}:{ARROW_SERVER_PORT}"
        self.url = base.rstrip("/") + "/images"
        self.run_id, self.log, self.timeout_s = run_id, log, timeout_s
        self.sent: list[int] = []
        self._jobs: queue.Queue = queue.Queue()
        self._thread = threading.Thread(target=self._work, name="arrow-upload", daemon=True)
        self._thread.start()

    def send(self, obstacle_id: int, arrow: str, path: Path | None) -> None:
        if path is None or not Path(path).is_file():
            self.log(f"arrow picture {obstacle_id}: no image file to send")
            return
        self._jobs.put((obstacle_id, arrow, Path(path)))

    def close(self, wait_s: float = 10.0) -> None:
        """Let queued pictures go (at most `wait_s`), then stop."""
        self._jobs.put(None)
        self._thread.join(timeout=wait_s)

    def _work(self) -> None:
        while (job := self._jobs.get()) is not None:
            obstacle_id, arrow, path = job
            url = (f"{self.url}?run_id={quote(self.run_id)}&obstacle_id={obstacle_id}&arrow={arrow}")
            request = urllib.request.Request(url, data=path.read_bytes(), method="POST",
                                             headers={"Content-Type": "image/jpeg"})
            try:
                with urllib.request.urlopen(request, timeout=self.timeout_s) as reply:
                    reply.read()
                self.sent.append(obstacle_id)
                self.log(f"arrow picture {obstacle_id} ({arrow}) shown on the PC")
            except (urllib.error.URLError, OSError) as exc:
                self.log(f"arrow picture {obstacle_id} NOT sent to {self.url}: {exc}")


class RunFail(RuntimeError):
    pass


class NoPlan(RunFail):
    """No clear path was found; nothing was sent for it."""


@dataclass
class Bypass:
    moves: list[str]
    end: Pose
    length_cm: float


def plan_bypass(side: str, o1_x: float, o1_face: float, start: Pose = Pose(0.0, 0.0, 90.0)) -> Bypass | None:
    """The shortest S-curve (out-turn a, straight s, back-turn a: ends heading north again) into a lane at least
    LANE_OFFSET_CM to `side` of O1's centre, from which a straight LANE_RUN_CM on is clear of O1 (margin +
    EXTRA_MARGIN_CM). A short reverse first (PRE_REVERSES_CM) only when nothing fits without one."""
    if side not in ("LEFT", "RIGHT"):
        raise ValueError(f"side must be LEFT or RIGHT, got {side!r}")
    out, back = ("FL", "FR") if side == "LEFT" else ("FR", "FL")
    half = config.O1_SIZE_CM / 2
    o1 = Rect(o1_x - half, o1_face, o1_x + half, o1_face + config.O1_SIZE_CM, "O1")
    for k in PRE_REVERSES_CM:
        prefix = [f"MOV,B,{k}"] if k else []
        begin = simulate(start, prefix)
        best = None
        for a in range(5, 91):
            for s in range(0, 61, 2):
                moves = [f"MOV,{out},{a}"] + ([f"MOV,F,{s}"] if s else []) + [f"MOV,{back},{a}"]
                length = path_length(moves)
                if best is not None and length >= best[0]:
                    continue
                if abs(simulate(begin, moves).x - o1_x) < LANE_OFFSET_CM - 0.5:
                    continue
                if is_clear(begin, moves + [f"MOV,F,{LANE_RUN_CM}"], [o1], EXTRA_MARGIN_CM):
                    best = (length, moves)
        if best is not None:
            full = prefix + best[1]
            return Bypass(full, simulate(start, full), path_length(full))
    return None


# --- the C curve round O1 (--c-curve) -----------------------------------------------------------
C_MIN_O2_GAP_CM = 60.0          # team statement: O1 back face -> O2 front face is never less than this
C_O2_HALF_WIDTH_CM = 200.0      # O2 is planned as a wall this wide (its span is unknown)
C_END_OFFSETS_CM = range(0, 16) # end offsets from O1's centre line tried, nearest first (either side)
C_O1_EXTRA_SAFETY_CM = 4.0      # team instruction 2026-10-14: the C grazed O1 on the robot with 5 cm planned
C_O1_MARGIN_CM = config.MARGIN_CM + EXTRA_MARGIN_CM + C_O1_EXTRA_SAFETY_CM   # 9 cm from O1 for the whole C
C_O2_MARGIN_CM = config.MARGIN_CM                     # 3 cm (team instruction 2026-10-14): a centred end fits the
                                                      # 60 cm gap only without the extra 2 cm towards O2
C_END_TOLERANCE_CM = 1.0


@dataclass
class CCurve:
    moves: list[str]
    end: Pose
    length_cm: float
    offset_cm: float                   # how far the end is from O1's centre line (to the passing side)
    params: tuple[int, int, int, int]  # (reverse first, a, s, b)


def c_curve_moves(side: str, a: int, s: int, b: int) -> list[str]:
    out, back = ("FL", "FR") if side == "LEFT" else ("FR", "FL")
    return ([f"MOV,{out},{a}"] + ([f"MOV,F,{s}"] if s else []) + [f"MOV,{back},{b}"]
            + ([f"MOV,{out},{b - a}"] if b > a else []))


_C_CANDIDATES: dict[str, list[tuple[float, float, float, tuple[int, int, int], list[str]]]] = {}


def c_curve_candidates(side: str) -> list[tuple[float, float, float, tuple[int, int, int], list[str]]]:
    """Every (a, s, b) C curve on `side` from the origin heading north: (end dx, end dy, path length, params,
    moves), shortest first. The shape does not depend on where it starts, so it is computed once per side."""
    if side not in _C_CANDIDATES:
        out = []
        for a in range(10, 91, 2):
            for s in range(0, 41, 3):
                for b in range(a + 1, a + 121):
                    moves = c_curve_moves(side, a, s, b)
                    end = simulate(Pose(0.0, 0.0, 90.0), moves)
                    out.append((end.x, end.y, path_length(moves), (a, s, b), moves))
        out.sort(key=lambda c: c[2])
        _C_CANDIDATES[side] = out
    return _C_CANDIDATES[side]


_C_SWEPT: dict[tuple[tuple[int, int, int], float], list] = {}


def c_curve_clear(side: str, params: tuple[int, int, int], moves: list[str], begin: Pose, rects: list[Rect],
                  margin_cm: float) -> bool:
    """Whether a C curve from `begin` (heading north) is clear of `rects` by `margin_cm`. Its swept outline is
    computed once from the origin (the shape does not depend on where it starts) and the rects are shifted."""
    key = ((side,) + params, margin_cm)
    polygons = _C_SWEPT.get(key)
    if polygons is None:
        polygons = _C_SWEPT[key] = swept_polygons(Pose(0.0, 0.0, 90.0), moves, margin_cm)
    shifted = [Rect(r.x0 - begin.x, r.y0 - begin.y, r.x1 - begin.x, r.y1 - begin.y, r.name) for r in rects]
    return hit_in(polygons, shifted) is None


def plan_c_curve(side: str, o1_x: float, o1_face: float, start: Pose = Pose(0.0, 0.0, 90.0),
                 o2_gap_cm: float = C_MIN_O2_GAP_CM, min_offset_cm: float = 0.0) -> CCurve | None:
    """The C curve round O1 on `side` (see the module docstring) from `start` (heading north), with no
    correction afterwards: for each end offset in C_END_OFFSETS_CM (nearest O1's centre line first, on either
    side of it), with no reverse first or then each short straight reverse in PRE_REVERSES_CM, the shortest
    (a, s, b) whose swept footprint clears O1 by C_O1_MARGIN_CM and an O2 wall `o2_gap_cm` behind O1 by
    C_O2_MARGIN_CM, ending heading north with the rear axle past O1's back face. `min_offset_cm` skips offsets
    already known not to fit (the nominal plan's offset less a few cm), to keep the search short on the Pi."""
    if side not in ("LEFT", "RIGHT"):
        raise ValueError(f"side must be LEFT or RIGHT, got {side!r}")
    if abs((start.h - 90.0 + 180.0) % 360.0 - 180.0) > 0.5:
        raise ValueError(f"a C curve starts heading north, not {start.h}")
    half = config.O1_SIZE_CM / 2
    o1_back = o1_face + config.O1_SIZE_CM
    o1 = [Rect(o1_x - half, o1_face, o1_x + half, o1_back, "O1")]
    o2 = [Rect(o1_x - C_O2_HALF_WIDTH_CM, o1_back + o2_gap_cm, o1_x + C_O2_HALF_WIDTH_CM,
               o1_back + o2_gap_cm + config.O2_DEPTH_CM, "O2")]
    sign = -1 if side == "LEFT" else 1
    front_limit = o1_back + o2_gap_cm - C_O2_MARGIN_CM
    candidates = c_curve_candidates(side)
    starts = [([f"MOV,B,{k}"] if k else [], k) for k in PRE_REVERSES_CM]
    for offset in (o for o in C_END_OFFSETS_CM if o >= min_offset_cm):
        for prefix, k in starts:
            begin = simulate(start, prefix)
            for dx, dy, length, params, moves in candidates:
                off = sign * (begin.x + dx - o1_x)
                if abs(abs(off) - offset) > C_END_TOLERANCE_CM or begin.y + dy < o1_back + config.MARGIN_CM:
                    continue
                if begin.y + dy + config.FRONT_EXTENT_CM > front_limit:   # cheap: the end alone already reaches O2
                    continue
                if (c_curve_clear(side, params, moves, begin, o1, C_O1_MARGIN_CM)
                        and c_curve_clear(side, params, moves, begin, o2, C_O2_MARGIN_CM)):
                    full = prefix + moves
                    return CCurve(full, simulate(start, full), length + k, off, (k,) + params)
    return None


# The C curve is FIXED per side (team instruction 2026-10-14: the robot always stops STOP_CM from O1, so the Pi
# sends the same commands every time and computes nothing). Worked out on the laptop with plan_c_curve's shapes as
# the curve nearest O1's centre line that stays clear of O1 (C_O1_MARGIN_CM) and of an O2 at the 60 cm gap
# (C_O2_MARGIN_CM) for every stop in 28.5-31.5 cm from the ultrasonic and O1 up to C_FIXED_O1_TOL_CM either side
# of the robot's line (tests/test_final_run.py checks this). They end about 16 cm (LEFT) / 18 cm (RIGHT) to the
# passing side of O1's centre line.
#
# Robot runs 2026-10-14 with the first set (swing-back FR,74 / FL,79) ended 18.9 cm (LEFT) and 14.5 cm (RIGHT) off
# O2's centre with good clearance round O1, so the swing-back turn (and the last turn with it, to end heading north)
# was made larger by the degrees that move the model's end onto the centre line from those measured ends: +24 deg
# (LEFT) and +18 deg (RIGHT). In the model they keep 7-14 cm from O1 over the tolerance below, but they reach about
# 13 cm further forward: they need an O1->O2 gap of about 75 cm (LEFT, with the stop tolerance) / 72 cm (RIGHT) with a 3 cm margin
# (C_CURVE_O2_GAP_CM). A course with O2 at the 60 cm minimum is NOT clear for them.
#
# The next robot run (2026-10-14) nearly touched O2: the curves were re-chosen to swing out less and so swing back
# less, reaching 5-8 cm less far forward, with the same end point (the one calibrated from the two runs above) and
# 5 cm from O1 for any stop 28.5-31.5 cm and O1 up to C_FIXED_O1_TOL_CM either side of the robot's line (the camera
# no longer measures it: the robot must face O1).
# The run after that ended centred on both sides but came close to O1 when passing right: the RIGHT curve was
# re-chosen with the same end point and 7 cm from O1 (LEFT unchanged), reaching 2.6 cm further forward.
# The next run ended slightly right of centre after a RIGHT pass: its swing-back and last turn got 3 deg more
# (about 2.5 cm further left in the model), reaching 1.6 cm further forward.
C_FIXED_O1_TOL_CM = 1.5
# After the C curve (team instruction 2026-10-14): O2's arrow is read with the ultrasonic at O2_TURN_US_CM["RIGHT"],
# then ONE 90 deg forward turn towards the arrow, started where the ultrasonic reads O2_TURN_US_CM for that side, so
# that the turn's radius brings the robot parallel to O2's front face. Worked out on the laptop: the turning front
# corner keeps 3 cm from O2; the robot's side then ends about 12 cm (LEFT, the tighter turn) / 10.5 cm (RIGHT) from
# O2 (side IR about 4 cm more), the rear axle 24.8 cm left / 33.7 cm right of where it turned.
O2_TURN = {"LEFT": "MOV,FL,90", "RIGHT": "MOV,FR,90"}
# Robot run 2026-10-14: RIGHT left the side IR 14 cm from O2 (as wanted), LEFT 19 cm: the LEFT turn now starts
# 5 cm closer (team instruction). In the model its turning front corner would then reach about 2 cm into O2; the real
# left turn has ended further out than the model, so the real corner clearance is about 1 cm. Watch it.
O2_TURN_US_CM = {"LEFT": 21.8, "RIGHT": 34.2}
O2_TURN_TOLERANCE_CM = 1.0      # tighter than the O1 stop: every cm short is a cm less between the corner and O2
C_CURVE_O1_CHECK_CM = config.MARGIN_CM + EXTRA_MARGIN_CM   # the pre-curve check keeps 5 cm from O1
# Robot run 2026-10-14 23:21 with O1 60 cm from the carpark and O2 60 cm behind O1 (the worst case): the RIGHT curve
# (B,5 FR,48 F,18 FL,108 FR,60, reaching 65.7 cm past O1's back face in the model) ran into O2; LEFT (61.7 cm in the
# model) works there on the robot (team-reported). RIGHT was re-chosen to reach no further than LEFT (61.2 cm) with the
# same 6 cm from O1 over the stop / O1 tolerance; it ends ~4 cm further right than the centred one (model).
C_CURVE_O2_GAP_CM = 66.5         # in the MODEL both curves need O2 this far behind O1 (61.7 cm reach + the 1.5 cm stop
                                 # tolerance + 3 cm); on the robot LEFT clears a 60 cm gap (the real turns are tighter)
C_CURVE_MOVES = {
    "LEFT": ["MOV,B,15", "MOV,FL,52", "MOV,F,18", "MOV,FR,122", "MOV,FL,70"],
    "RIGHT": ["MOV,B,3", "MOV,FR,36", "MOV,F,27", "MOV,FL,84", "MOV,FR,48"],
}


# Part 3 (--c-curve, team plan 2026-10-14): round O2 and face the carpark, FIXED commands. After the turn onto O2's
# front face a static IR,GET decides: the side IR still sees O2 -> wall hug the front face to O2's end; it does not (the
# turn carried the IR past a short O2's end) -> reverse straight until it sees O2, then the same edge-follow forward
# (slow, the STM's FIR speed cap) stops just past the end. Both then U-turn onto the back face, wall hug it to O2's
# other end and turn 90 deg to face the carpark. The lead-in straights put the rear axle near O2's end before each
# turn (the STM stops a FIR with the side IR, 19 cm ahead of the axle, just past the end).
O2_BESIDE_CM = config.STM_IR_VISIBLE_CM   # a static side IR reading at or below this: the robot is beside O2
REVERSE_STEP_CM = 5             # case 1: reverse in these steps, an IR,GET after each ...
REVERSE_MAX_CM = 60             # ... at most this far (the turns leave the side IR at most ~53 cm past a 20 cm O2's end)
REVERSE_BACK_MAX_CM = 25        # ... on the back face (only a short O2 leaves the IR past its far end after the U-turn)
REVERSE_PAST_CM = 5             # ... and this much more once it sees O2, so the FIR confirms O2 before its end
FIR_STOP_PAST_CM = 2.0          # a FIR stops at least this far past the edge it confirms (robot logs; see hug)
EDGE_CAP_CM = 40                # case 1: the forward FIR from just inside O2's end drives at most this far
# Case 1 in ONE reverse (robot run 2026-10-15: six stops of 5 cm, about 6.5 s for 30 cm): O2 is centred on O1 and
# reaches at least o2_min_half_cm either side of its centre, so a side IR that sees nothing is past O2's end and one
# straight back to REVERSE_INSIDE_CM short of that guaranteed reach puts it beside O2. The run found O2 with the IR
# 6.7 cm inside its end and the FIR confirmed it. One IR,GET checks it; if O2 is still not seen, the 5 cm steps go on.
REVERSE_ONE_MOVE = True
REVERSE_INSIDE_CM = 7.0         # the one reverse leaves the side IR this far inside O2's (shortest possible) end
REVERSE_EDGE_MARGIN_CM = 8      # ... and the FIR after it may drive this far more than that reverse (it stops at the end)
# Model values (FL radius 24.8 cm, FR 33.7 cm), not yet run on the robot: tune from measured end positions.
# RIGHT (heading east, O2 on the left): FL round the east end. LEFT (heading west, O2 on the right): FR is wider
# (2 x 33.7 cm across), more than O2's 10 cm depth and the two face gaps need. The U-turn leaves the side IR about 20 cm inside
# the near end; a reverse after it to bring the IR nearer that end (robot run 2026-10-14 21:31: B,15) left the IR seeing
# nothing, the real FIR stopping further past the end than modelled. A short O2 whose far end the IR is then past is
# found by reversing, as on the front face.
U_TURN_MOVES = {
    # one motion (team instruction 2026-10-14: no reversing, one sharp turn): FL,180 carries the robot 2 x 24.8 cm across
    # O2's 10 cm depth, back-face gap 8.6-11.6 cm for face gaps 9-12 cm (model); clear for lead-ins 7-15 cm
    "RIGHT": ["MOV,F,9", "MOV,FL,180"],
    # Team instruction 2026-10-14: a sharp U-turn round O2's 10 cm end; it need not stay in IR range of the end face.
    # FR 30, back 8, FR 75, BR 15 (reverse right also turns clockwise, on the tighter 24.5 cm radius), FR 60. Model:
    # swings ~33 cm out past O2's end (F,7 FR,90 B,19 FR,90 on the robot, 21:36: ~44 cm, up to ~20 cm from O2), keeps
    # 3 cm from O2 and a wall 50 cm out for face gaps 9-12.5 cm (robot: IR 13.4-16.8 cm) and a FIR stopping 2-7 cm
    # past its measured edge; back-face gap 8-11 cm (IR 12-15 cm). Not yet run on the robot.
    "LEFT": ["MOV,F,6", "MOV,FR,30", "MOV,B,8", "MOV,FR,75", "MOV,BR,15", "MOV,FR,60"],
}
# The 90 deg turn round O2's far end to face the carpark, as tight as it can be (team instructions 2026-10-14: end at
# most 15 cm from O2; then "make the turn sharper, do not worry about the clearance with the obstacle"). The FIR stops
# with the robot already past O2's end, and FR (33.7 cm radius) then left the LEFT run (22:25) far from O2. So a
# straight before the turn, forward or back (FACE_CARPARK_LEAD_CM), puts the robot's side FACE_CARPARK_TARGET_GAP_CM
# from O2's end face after the turn (planned, from the measured far end), keeping only FACE_CARPARK_MARGIN_CM from O2.
FACE_CARPARK_TURN = {
    "RIGHT": "MOV,FL,90",                   # heading west along the back face: left, round the west end
    "LEFT": "MOV,FR,90",                    # heading east along the back face: right, round the east end
}
FACE_CARPARK_LEAD_CM = {"RIGHT": range(0, 16), "LEFT": range(-20, 16)}   # only FR (LEFT) needs to back up first
# LEFT may also split the turn: FR,a forward past O2's corner, then BR,90-a (reverse right, also clockwise) back into
# place beside the end face. One FR,90 cannot come nearer than ~3 cm without its inside cutting O2's corner; FR,45 +
# BR,45 plans to ~0-1 cm (model, team instruction 2026-10-14 22:30: "make it tighter, there's still a lot of room").
FACE_CARPARK_SPLIT_DEG = {"RIGHT": (), "LEFT": tuple(range(30, 90, 5))}
# ... but no further out past O2's end than this (robot run 22:37: F,8 FR,45 BR,45 swung ~41 cm past the end, at the
# side wall; reversing first, e.g. B,4 FR,65 BR,25, ends as close and reaches ~31 cm, like one FR,90 after backing up)
FACE_CARPARK_MAX_REACH_CM = {"RIGHT": float("inf"), "LEFT": 32.0}
# Facing the carpark, the rear axle is then brought level with O2's back face (+ this, towards the carpark) by one
# straight. Robot runs 2026-10-14: LEFT (FR 33.7 cm) ended slightly behind O2, RIGHT (FL 24.8 cm) slightly in front;
# model: rear 4.7 cm past the back face / 5.3 cm short of it. The back face is the one the side IR measured.
FACE_CARPARK_REAR_FROM_BACK_CM = 0.0
# Planned gaps (from the Pi's measured end). Robot runs 2026-10-14, wheel to O2's end face, both ending level with
# the back face: LEFT 22:50 planned ~1 cm -> 13 cm (team: good). RIGHT (straight before FL,90 -> wheel gap): F,3 ->
# 9 cm, F,7 -> 15 cm, F,6 -> 19 and 16 cm: ~2 cm per cm of straight, +-1.5 cm run to run. RIGHT aims for 5 cm (~F,4
# on the robot) so both end ~13 cm out (team instruction: "make the last turn slightly wider to match the distance").
FACE_CARPARK_TARGET_GAP_CM = {"RIGHT": 5.0, "LEFT": 1.0}
FACE_CARPARK_MARGIN_CM = {"RIGHT": 1.5, "LEFT": 0.5}       # from the measured end, which is O2's end at most
FACE_CARPARK_MAX_GAP_CM = 15.0
FIXED_O2_MARGIN_CM = config.MARGIN_CM   # the fixed U-turn and last turn keep this from O2 and a possible wall
# Part 4 (--c-curve, team plan 2026-10-14): back into the carpark. The gap between the obstacles is known from the
# start: the fixed C curve ends a known distance past O1's back face (its planned end), and the ultrasonic then reads
# O2 (the sensor face is ULTRASONIC_FORWARD_CM ahead of the rear axle, which is at the back of the car). O2's length
# comes from the two FIR legs (wheel encoders), so facing the carpark the robot knows how far it is to the side of
# O1's centre line (O2 centred on O1) and where O1 is. It drives a straight, then one S-curve onto that line (its angle
# set by the offset, FR+FL or FL+FR), the straight the shortest that lets the S-curve clear O1 by RETURN_O1_MARGIN_CM,
# then straights by the ultrasonic until it reads PARK_US_CM from the carpark's back wall.
RETURN_O1_MARGIN_CM = config.MARGIN_CM + EXTRA_MARGIN_CM   # 5 cm: O1's position carries the C curve's odometry
RETURN_LEAD_MAX_CM = 200
PARK_US_CM = 15.0               # ultrasonic face -> carpark back wall at the end: the car's front ~10 cm from it
PARK_US_TOLERANCE_CM = 1.5
# One straight into the carpark (instead of stopping short and measuring again): the Pi's own pose (the start placement,
# config.START_REAR_FROM_BACK_WALL_CM, plus every planned move) puts the back wall a distance ahead; when that agrees with
# the first ultrasonic reading it drives the SHORTER of the two less PARK_US_CM, so a reading that is too long cannot take
# it further than the Pi's own tracking allows. Disagreeing by more than PARK_TRACK_AGREE_CM: the stop-short way.
PARK_TRACK_AGREE_CM = 5.0
PARK_FACING_TOL_DEG = 5.0       # the tracked distance is only used facing the back wall (south) within this


def c_curve_safe(start: Pose, moves: list[str], o1_x: float, o1_face: float) -> bool:
    """One sweep check of one fixed C curve against O1 (C_CURVE_O1_CHECK_CM) and an O2 wall C_CURVE_O2_GAP_CM behind
    O1 (C_O2_MARGIN_CM)."""
    half = config.O1_SIZE_CM / 2
    back = o1_face + config.O1_SIZE_CM
    o1 = [Rect(o1_x - half, o1_face, o1_x + half, back, "O1")]
    o2 = [Rect(o1_x - C_O2_HALF_WIDTH_CM, back + C_CURVE_O2_GAP_CM, o1_x + C_O2_HALF_WIDTH_CM,
               back + C_CURVE_O2_GAP_CM + config.O2_DEPTH_CM, "O2")]
    polygons_o1 = swept_polygons(start, moves, C_CURVE_O1_CHECK_CM)
    return hit_in(polygons_o1, o1) is None and hit_in(swept_polygons(start, moves, C_O2_MARGIN_CM), o2) is None


# --- wall hug geometry --------------------------------------------------------------------------
def clear_fast(start: Pose, moves: list[str], rects: list[Rect]) -> bool:
    """is_clear(start, moves, rects, EXTRA_MARGIN_CM), stopping at the first move that touches: the planners
    reject most candidates, nearly all in their first moves."""
    pose = start
    for command in moves:
        if hit_in(swept_polygons(pose, [command], config.MARGIN_CM + EXTRA_MARGIN_CM), rects) is not None:
            return False
        pose = apply(pose, command)
    return True


def is_fast_straight(command: str) -> bool:
    """A forward straight (MOV,F,<cm>) of at least FAST_STRAIGHT_CM: the only moves that run in FAST_PROFILE."""
    parts = command.split(",")
    return len(parts) == 3 and parts[:2] == ["MOV", "F"] and parts[2].isdigit() and int(parts[2]) >= FAST_STRAIGHT_CM


def straight_moves(cm: float) -> list[str]:
    value = round(abs(cm))
    return [f"MOV,{'F' if cm > 0 else 'B'},{value}"] if value else []


def sensor_point(pose: Pose, sensor: str) -> tuple[float, float]:
    """The side IR's face (IR_LATERAL_CM is +left; Pose.point takes +right)."""
    return pose.point(config.IR_FORWARD_CM[sensor], -config.IR_LATERAL_CM[sensor])


def beam_y(pose: Pose, sensor: str) -> float:
    """y component of the side IR's beam direction (left IR looks left)."""
    rx, ry = pose.right()
    return ry if sensor == "R" else -ry


STRAIGHT_FIX_MIN_DEG = 1.0      # straights start with a fix turn only when the IMU is this far off the plan's heading


def straight_heading_fix(command: str, expected_deg: float, actual_deg: float) -> tuple[list[str], float]:
    """The moves for a straight that starts with the IMU `actual_deg` instead of the plan's `expected_deg`: a short
    turn in the straight's gear back to the plan's heading (at most MAX_HEADING_FIX_DEG), then the straight less the
    distance that turn covered along the heading, so it ends where planned (the turn's sideways drift is under 0.5 cm).
    The STM holds the heading it starts a straight with, so a heading error left there runs the whole straight crooked.
    Returns the moves and the error (deg, +CCW); the straight alone when the error is under STRAIGHT_FIX_MIN_DEG or the
    straight is shorter than the turn covers."""
    _, direction, value = command.split(",")
    error = (expected_deg - actual_deg + 180.0) % 360.0 - 180.0
    turn = round(min(MAX_HEADING_FIX_DEG, abs(error)))
    if abs(error) < STRAIGHT_FIX_MIN_DEG or turn == 0:
        return [command], error
    arc = f"MOV,{direction}{'L' if error > 0 else 'R'},{turn}"
    covered = round(abs(simulate(Pose(0.0, 0.0, 90.0), [arc]).y))
    if covered > int(value):
        return [command], error
    rest = int(value) - covered
    return [arc] + ([f"MOV,{direction},{rest}"] if rest else []), error


def axle_for_sensor(sensor_xy: tuple[float, float], heading: float, sensor: str) -> tuple[float, float]:
    """Where the rear axle must be for the side IR to be at `sensor_xy` with this heading."""
    sx, sy = sensor_point(Pose(0.0, 0.0, heading), sensor)
    return sensor_xy[0] - sx, sensor_xy[1] - sy


def hug_side(arrow2: str) -> tuple[int, str, str]:
    """(along, sensor, turn letter) for going round O2 on the `arrow2` side: RIGHT runs east (+x) with O2 on
    the robot's left (left IR, turns round O2 to the left); LEFT runs west with O2 on the right."""
    return (1, "L", "L") if arrow2 == "RIGHT" else (-1, "R", "R")


def _turn(letter: str, gear: str, deg: int) -> str:
    return f"MOV,{gear}{letter},{deg}"


def _arcs_to(heading_from: float, heading_to: float) -> list[list[str]]:
    """The turns (each way, each gear) that take `heading_from` to `heading_to`, split at 180 deg."""
    out = []
    for letter, sign in (("L", 1), ("R", -1)):
        deg = round((sign * (heading_to - heading_from)) % 360)
        if 0 < deg <= 270:
            for gear in "FB":
                out.append([_turn(letter, gear, d) for d in ((deg,) if deg <= 180 else (180, deg - 180))])
    return out


def plan_to(start: Pose, heading: float, targets: list[tuple[float, float]], rects: list[Rect],
            pre_limit: float = PRE_LIMIT_CM) -> tuple[list[str], int] | None:
    """The shortest sweep-clear path that ends with the rear axle on one of `targets` facing `heading` (returns
    the moves and that target's index). Each candidate is A + [straight u1] + B + [straight u2] + C + [straight
    t], with A, B, C fixed arcs and t one of FINAL_STRAIGHT_CM; u1 runs along the heading after A and u2 along
    the heading after B, so the end point is linear in (u1, u2) and is solved exactly (to the whole cm). The
    shapes:
      - turn:  A empty (u1 is a straight first, at most `pre_limit`), B one arc in either gear or a three-point
               pair (forward then reverse, same steering: both turn the same way, for a tight turn), C the
               last arc(s), either gear;
      - S-curve then turn: A and B a forward S-curve (out a, back a: a sidestep, u1 its diagonal), u2 a
               straight on, C the last arc(s)."""
    shapes = []
    for first in ([[_turn(letter, gear, a1)] for gear, letter in itertools.product("FB", "LR")
                   for a1 in range(ARC_STEP_DEG, 180, ARC_STEP_DEG)]
                  + [[_turn(letter, "F", a), _turn(letter, "B", b)] for letter in "LR"
                     for a, b in itertools.product(*THREE_POINT_DEG)]):
        shapes.append(([], first))
    for letter, a in itertools.product("LR", S_CURVE_DEG):
        shapes.append(([_turn(letter, "F", a)], [_turn("R" if letter == "L" else "L", "F", a)]))
    hx, hy = Pose(0.0, 0.0, heading).forward()
    candidates = []
    for arcs_a, arcs_b in shapes:
        p1 = simulate(start, arcs_a)
        p2 = simulate(p1, arcs_b)
        (vx, vy), (wx, wy) = p1.forward(), p2.forward()
        det = vx * wy - vy * wx
        if abs(det) < 0.2:
            continue
        limit1 = MID_LIMIT_CM if arcs_a else pre_limit
        for arcs_c in _arcs_to(p2.h, heading):
            end0 = simulate(p2, arcs_c)
            arc_length = path_length(arcs_a + arcs_b + arcs_c)
            for (i, target), t in itertools.product(enumerate(targets), FINAL_STRAIGHT_CM):
                dx, dy = target[0] - t * hx - end0.x, target[1] - t * hy - end0.y
                u1, u2 = (dx * wy - wx * dy) / det, (vx * dy - vy * dx) / det
                if abs(u1) > limit1 or abs(u2) > MID_LIMIT_CM or (arcs_a and u1 < -0.5):
                    continue
                candidates.append((arc_length + abs(u1) + abs(u2) + t, i, arcs_a, u1, arcs_b, u2, arcs_c, t))
    candidates.sort(key=lambda c: (c[0], c[1]))
    for _, i, arcs_a, u1, arcs_b, u2, arcs_c, t in candidates:
        moves = arcs_a + straight_moves(u1) + arcs_b + straight_moves(u2) + arcs_c + straight_moves(t)
        if clear_fast(start, moves, rects):
            return moves, i
    return None


def plan_final_turn(start: Pose, letter: str, rects: list[Rect]) -> list[str] | None:
    """Straight on past O2's end, then 90 deg towards O2 (`letter`): one arc, or a three-point turn (forward a,
    reverse b, forward 90 - a - b: a reverse arc with the same steering keeps turning the same way) where one
    arc swings too wide. The shortest sweep-clear one."""
    shapes = [[_turn(letter, "F", 90)]]
    for a, b in itertools.product(*THREE_POINT_DEG):
        if a + b < 90:
            shapes.append([_turn(letter, "F", a), _turn(letter, "B", b), _turn(letter, "F", 90 - a - b)])
    candidates = sorted((path_length(straight_moves(c) + shape), straight_moves(c) + shape)
                        for c in FINAL_CLEAR_CM for shape in shapes)
    for _, moves in candidates:
        if clear_fast(start, moves, rects):
            return moves
    return None


class FinalRun:
    """The staged run on any STM-like object with move/heading/measure_front (Task2Stm, or sim.FakeStm) and a
    camera with look(tag, frames)."""

    def __init__(self, stm, camera, stop_cm: float = STOP_CM, o2_read_cm: float = O2_READ_CM, log=print,
                 heading_fix: bool = True, wall_hug: bool = True, c_curve: bool = False,
                 fast_straights: bool = False, base_profile: str = "PRECISION", park_one_shot: bool = False) -> None:
        self.stm, self.camera, self.log = stm, camera, log
        self.park_one_shot = park_one_shot                                        # one straight into the carpark when tracking agrees
        self.start_rear_from_wall_cm = config.START_REAR_FROM_BACK_WALL_CM        # where the robot was placed (tracking)
        self.fast_straights = fast_straights and base_profile != FAST_PROFILE   # long forward straights in FAST_PROFILE
        self.base_profile = base_profile                                          # the profile everything else runs in
        self.stop_cm, self.o2_read_cm = stop_cm, o2_read_cm
        self.heading_fix = heading_fix
        self.wall_hug = wall_hug                  # False: stop after reading O2's arrow (stages 1-5)
        self.c_curve = c_curve                    # True: stages 1-3 of the centred plan (C curve round O1), then stop
        self.pose = Pose(0.0, 0.0, 90.0)          # rear axle; +y north, towards the obstacles
        self.imu0: float | None = None
        self.arrow: str | None = None
        self.arrow2: str | None = None
        self.o1_x: float | None = None
        self.o1_face: float | None = None
        self.o2_face: float | None = None         # y of O2's front face (ultrasonic, then the side IR)
        self.o2_x: float | None = None            # O2's mid-point (its arrow) = O1's centre (team rule)
        self.o2_ends: dict[int, float] = {}       # measured end x, keyed by the direction it lies in (+1 east)
        self.o2_end_tol: dict[int, float] = {}    # how far off an end may be (camera-estimated ends only)
        self.o2_bullseyes: list[float] = []       # x of the bullseyes seen on O2's front face with its arrow
        self.fir_cap_cm = FIR_CAP_CM
        self.uploader: ArrowUploader | None = None   # sends the arrow pictures to the PC page, if set
        self.arrow_looks: dict[int, object] = {}      # obstacle -> the camera look its arrow was read from
        self.o2_min_half_cm = O2_MIN_HALF_CM
        self.o1_o2_gap_cm: float | None = None    # O1's back face -> O2's front face (C curve end + ultrasonic)
        self.result: dict = {}

    # -- helpers ---------------------------------------------------------------------------------
    def front_cm(self, tag: str) -> float:
        reading = self.stm.measure_front()
        self.log(f"ultrasonic {tag}: {reading.distance_cm} cm ({reading.used}/{reading.seen} lines"
                 + (f", {reading.problem}" if reading.problem else "") + ")")
        if not reading.ok:
            raise RunFail(f"no ultrasonic reading at {tag}: {reading.problem}")
        return reading.distance_cm

    def send(self, command: str) -> None:
        """Send one planned move. With the heading fix, a turn is adjusted to end at the plan's heading and a
        straight starts with a short turn back to it (straight_heading_fix); the pose stays the planned one."""
        direction = command.split(",")[1]
        moves = [command]
        if self.heading_fix and self.imu0 is not None and direction in ("F", "B", "FL", "FR", "BL", "BR"):
            actual = self.stm.heading()
            if actual is not None:
                expected = self.imu0 + (self.pose.h - 90.0)
                if direction in ("F", "B"):
                    moves, error = straight_heading_fix(command, expected, actual)
                else:
                    sent, error = corrected_turn(command, expected, actual)
                    moves = [sent]
                if moves != [command]:
                    self.log(f"heading fix {command} -> {' + '.join(moves)} ({error:+.1f} deg)")
        for sent in moves:
            fast = self.fast_straights and is_fast_straight(sent) and self.switch_profile(FAST_PROFILE, sent)
            reply = self.stm.move(sent)
            if not reply.ok:
                raise RunFail(f"{sent}: {reply.line or reply.status}")
            if fast and not self.switch_profile(self.base_profile, sent, must=True):
                raise RunFail(f"{sent}: could not go back to {self.base_profile}")
        self.pose = simulate(self.pose, [command])
        self.log(f"{' + '.join(moves)}: pose {self.pose}")

    def switch_profile(self, profile: str, why: str, must: bool = False) -> bool:
        """Set and verify the STM motion profile (idle, between moves). True when it is `profile`. A switch to the fast
        profile that fails is undone (back to the base profile; if that fails too the run stops) and the straight
        runs in the base profile: False. Going back to the base profile is `must`: a failure stops the run."""
        reason = self.stm.apply_profile(profile)
        if reason is None:
            self.log(f"profile {profile} for {why}" if profile == FAST_PROFILE else f"profile {profile} again")
            return True
        if must:
            self.log(f"WARNING: could not set profile {profile}: {reason}")
            return False
        self.log(f"WARNING: could not set profile {profile} for {why} ({reason}): driving it in {self.base_profile}")
        if self.stm.apply_profile(self.base_profile) is not None:
            raise RunFail(f"profile {profile} failed and {self.base_profile} could not be set again: the STM profile is unknown")
        return False

    def o1_rect(self) -> list[Rect]:
        if self.o1_face is None or self.o1_x is None:
            return []
        half = config.O1_SIZE_CM / 2
        return [Rect(self.o1_x - half, self.o1_face, self.o1_x + half, self.o1_face + config.O1_SIZE_CM, "O1")]

    def straight(self, cm: float, why: str) -> None:
        value = round(abs(cm))
        if not value:
            return
        command = f"MOV,{'F' if cm > 0 else 'B'},{value}"
        if self.o1_rect() and not is_clear(self.pose, [command], self.o1_rect(), EXTRA_MARGIN_CM):
            raise RunFail(f"{why}: {command} would come within {config.MARGIN_CM + EXTRA_MARGIN_CM:g} cm of O1")
        self.send(command)

    def tracked_wall_cm(self) -> float | None:
        """Ultrasonic face -> the carpark's back wall by the Pi's own pose (the start placement plus every planned move),
        or None when the robot is not facing the wall (south, within PARK_FACING_TOL_DEG)."""
        if abs((self.pose.h - 270.0 + 180.0) % 360.0 - 180.0) > PARK_FACING_TOL_DEG:
            return None
        return self.pose.point(config.ULTRASONIC_FORWARD_CM)[1] + self.start_rear_from_wall_cm

    def use_tracking(self, move: float, gap: float, distance: float, tracked_cm: float, wanted_cm: float, tag: str,
                     first: bool) -> float:
        """The move drive_to makes when the Pi's own distance to the same surface (`tracked_cm`) is known. A split
        approach (more than APPROACH_REMEASURE_CM to go, where `move` stops APPROACH_SHORT_CM short):
          - the two distances agree within PARK_TRACK_AGREE_CM: ONE straight to `wanted_cm` by the shorter of them;
          - they do not: `move`, but never further than the tracking allows (robot run 23:43 read 152 cm at ~70 cm from
            the wall); a tracked wall too near for any move gives 0, so nothing is driven until the readings agree.
        A shorter approach is `move` as it is. Both distances are logged once and kept in the result."""
        agree = abs(distance - tracked_cm) <= PARK_TRACK_AGREE_CM
        split = gap > APPROACH_REMEASURE_CM
        if first:
            self.result.update(park_tracked_cm=round(tracked_cm, 1), park_start_ultrasonic_cm=round(distance, 1),
                               park_one_shot=agree and split)
            self.log(f"{tag}: the Pi's own position puts the back wall {tracked_cm:.1f} cm ahead, the ultrasonic "
                     f"{distance:.1f} cm ({tracked_cm - distance:+.1f}): "
                     + ("one straight by the shorter" if agree and split else
                        "no merge" if not split else "stopping short, not past what the tracking allows"))
        if not split:
            return move
        if agree:
            return min(distance, tracked_cm) - wanted_cm
        return min(move, max(0.0, tracked_cm - wanted_cm - APPROACH_SHORT_CM))

    def drive_to(self, wanted_cm: float, tag: str, tolerance_cm: float = STOP_TOLERANCE_CM,
                 tracked_cm: float | None = None) -> float:
        """Straights until the ultrasonic reads `wanted_cm` (within `tolerance_cm`); returns the reading. With
        `tracked_cm` (the Pi's own distance to the same surface) a split approach may be one straight, and is never
        driven past what the tracking allows (use_tracking)."""
        distance = self.front_cm(f"{tag} start")
        if distance > FIRST_READING_MAX_CM:
            raise RunFail(f"the ultrasonic reads {distance:.0f} cm at the start of {tag}: nothing straight ahead?")
        first = True
        for _ in range(APPROACH_MAX_STEPS):
            gap = distance - wanted_cm
            if abs(gap) <= tolerance_cm:
                return distance
            move = gap if gap <= APPROACH_REMEASURE_CM else gap - APPROACH_SHORT_CM
            if tracked_cm is not None:
                move = self.use_tracking(move, gap, distance, tracked_cm, wanted_cm, tag, first)
            first = False
            self.straight(move, tag)
            if tracked_cm is not None:
                tracked_cm -= move                      # the wall is that much nearer (the planned move, as in the pose)
            expected = distance - (round(abs(move)) if move > 0 else -round(abs(move)))
            distance = self.front_cm(tag)
            if abs(distance - expected) > US_JUMP_CM:
                # Robot run 2026-10-14 23:43: 82.6 cm, F,13, then 152 cm (twice, so it passed the agreement check) at
                # ~70 cm from the carpark wall; F,67 drove into it. A reading that disagrees with the move just made
                # is taken again; if it still disagrees, it is believed unless it is longer than the move allows and
                # far (US_MISSED_ECHO_CM): a missed echo, as above, then the move is trusted. A shorter reading is
                # something really there (e.g. a side wall in the cone); near, the cone sees only what is ahead.
                again = self.front_cm(f"{tag} again")
                if abs(again - expected) > US_JUMP_CM and again > expected and again > US_MISSED_ECHO_CM:
                    self.log(f"WARNING: {tag}: the ultrasonic reads {distance:.1f} / {again:.1f} cm after a "
                             f"{move:+.0f} cm move from {expected + round(move):.1f} cm: using {expected:.1f} cm")
                    distance = expected
                else:
                    distance = again
        if abs(distance - wanted_cm) <= tolerance_cm:      # the last reading counts too
            return distance
        raise RunFail(f"{tag}: not within {tolerance_cm:g} cm of {wanted_cm:g} cm after {APPROACH_MAX_STEPS} moves")

    def read(self, tag: str, depth_cam: float):
        """Up to ARROW_LOOKS looks; (side, lateral cm right of the axis at `depth_cam`, look)."""
        for i in range(ARROW_LOOKS):
            look = self.camera.look(f"{tag}{i}", 1)
            side, votes = decide_arrow(look.frames)
            if side is not None:
                box = largest(look.frames[0], "arrow")
                return side, lateral_cm(box.norm_x(), depth_cam), look
            self.log(f"{tag} look {i}: no arrow ({votes})")
        raise RunFail(f"no arrow seen at {tag} after {ARROW_LOOKS} looks")

    # -- the run ---------------------------------------------------------------------------------
    def run(self) -> bool:
        try:
            if self.heading_fix:
                self.imu0 = self.stm.heading()
            self.approach_o1()
            self.read_o1()
            if self.c_curve:
                self.c_curve_o1()
                self.read_o2_centred()
                self.turn_onto_o2()
                if not self.wall_hug:
                    self.log(f"DONE: C curve round O1 ({self.arrow}), O2 arrow {self.arrow2}, along O2's front face; "
                             f"pose {self.pose}")
                    return True
                self.round_o2_fixed()
                self.return_to_carpark()
                self.log(f"DONE: arrows O1 {self.arrow}, O2 {self.arrow2}; O2 ends {self.o2_ends}; in the "
                         f"carpark, pose {self.pose}")
                return True
            self.bypass_o1()
            self.approach_o2()
            self.read_o2()
            if self.wall_hug:
                if not self.round_from_lane():
                    self.enter_front_face()
                    self.hug("front")
                    self.u_turn()
                self.hug("back")
                self.face_carpark()
        except RunFail as exc:
            self.log(f"FAIL_SAFE: {exc}")
            self.result["fail"] = str(exc)
            return False
        self.log(f"DONE: arrows O1 {self.arrow}, O2 {self.arrow2}; O2 ends {self.o2_ends}; pose {self.pose}")
        return True

    def approach_o1(self) -> None:
        distance = self.drive_to(self.stop_cm, "approach O1")
        self.o1_face = self.pose.y + config.ULTRASONIC_FORWARD_CM + distance
        self.log(f"at O1's reading position: {distance:.1f} cm from O1")

    def read_o1(self) -> None:
        depth_cam = self.o1_face - self.pose.y - config.CAMERA_FORWARD_CM
        side, _, look = self.read("o1_arrow", depth_cam)
        self.arrow = side
        self.show_arrow(1, side, look)
        # Team instruction 2026-10-14: the camera only reads arrow directions, never distances. O1 is taken as
        # straight ahead of the robot (its distance comes from the ultrasonic).
        self.o1_x = self.pose.x
        self.result.update(o1_arrow=side, o1_image=str(look.annotated_path) if look.annotated_path else None)
        self.log(f"arrow on O1: {side}; O1 taken as straight ahead, "
                 f"{self.o1_face - self.pose.y - config.ULTRASONIC_FORWARD_CM:.1f} cm away (ultrasonic)")

    def c_curve_o1(self) -> None:
        moves = C_CURVE_MOVES[self.arrow]
        off = self.o1_x - self.pose.x
        # one quick sweep check (milliseconds) of the fixed curve against the measured O1 refuses it only if O1 is in
        # its way: O1 off to the side the curve swings out to (robot run 2026-10-14: O1 7.9 cm right of the line was
        # refused for a LEFT pass by a plain +-3 cm rule although the curve clears it)
        if not c_curve_safe(self.pose, moves, self.o1_x, self.o1_face):
            raise RunFail(f"O1 measured {off:+.1f} cm right of the robot's line, {self.o1_face - self.pose.y - config.ULTRASONIC_FORWARD_CM:.1f} cm "
                          f"away: outside what the fixed {self.arrow} C curve is safe for")
        end = simulate(self.pose, moves)
        sign = -1 if self.arrow == "LEFT" else 1
        plan = CCurve(moves, end, path_length(moves), sign * (end.x - self.o1_x), (0, 0, 0, 0))
        o2_gap = (self.o1_face + config.O1_SIZE_CM + C_MIN_O2_GAP_CM) - plan.end.y - config.FRONT_EXTENT_CM   # may be < 0
        self.result.update(c_curve=plan.moves, c_end_offset_cm=round(plan.offset_cm, 1),
                           c_end_right_of_o1_cm=round(plan.end.x - self.o1_x, 1),
                           c_end_past_o1_back_cm=round(plan.end.y - self.o1_face - config.O1_SIZE_CM, 1))
        self.log(f"C curve round O1 {self.arrow} (fixed): {plan.moves} ({plan.length_cm:.0f} cm). Planned end: rear axle {plan.end.x - self.o1_x:+.1f} cm right of "
                 f"O1's centre line, {plan.end.y - self.o1_face - config.O1_SIZE_CM:.1f} cm past O1's back face; front "
                 f"{o2_gap:.1f} cm short of an O2 at the 60 cm gap")
        for command in plan.moves:
            self.send(command)

    def read_o2_centred(self) -> None:
        """After the C curve (facing O2): to O2_TURN_US_CM["RIGHT"] by the ultrasonic, then O2's arrow, one
        inference (the camera is then about as far from O2 as it was from O1)."""
        distance = self.drive_to(O2_TURN_US_CM["RIGHT"], "O2 reading position", O2_TURN_TOLERANCE_CM)
        self.o2_face = self.pose.y + config.ULTRASONIC_FORWARD_CM + distance
        depth_cam = self.o2_face - self.pose.y - config.CAMERA_FORWARD_CM
        side, _, look = self.read("o2_arrow", depth_cam)
        self.arrow2 = side
        self.show_arrow(2, side, look)
        self.o1_o2_gap_cm = self.o2_face - (self.o1_face + config.O1_SIZE_CM)
        self.log(f"gap between the obstacles {self.o1_o2_gap_cm:.1f} cm (C curve end "
                 f"{self.pose.y - self.o1_face - config.O1_SIZE_CM:.1f} cm past O1's back face + ultrasonic "
                 f"{distance:.1f} cm + sensor {config.ULTRASONIC_FORWARD_CM:g} cm ahead of the rear axle)")
        self.result.update(o1_o2_gap_cm=round(self.o1_o2_gap_cm, 1))
        self.result.update(o2_arrow=side, o2_ultrasonic_cm=round(distance, 1),
                           o2_image=str(look.annotated_path) if look.annotated_path else None)
        self.log(f"arrow on O2: {side}; O2 {distance:.1f} cm away (ultrasonic)")

    def turn_onto_o2(self) -> None:
        """One 90 deg forward turn towards O2's arrow, from where the ultrasonic reads O2_TURN_US_CM for that side:
        it ends parallel to O2's front face, the robot's side about O2_TURN_SIDE_GAP_CM from it."""
        command = O2_TURN[self.arrow2]
        distance = self.drive_to(O2_TURN_US_CM[self.arrow2], f"before the {self.arrow2} turn", O2_TURN_TOLERANCE_CM)
        self.o2_face = self.pose.y + config.ULTRASONIC_FORWARD_CM + distance
        self.send(command)
        side_gap = self.o2_face - self.pose.y - config.WIDTH_CM / 2
        self.result.update(o2_turn=command, o2_side_gap_cm=round(side_gap, 1))
        self.log(f"turned {command} onto O2's front face: the robot's side about {side_gap:.1f} cm from O2 (planned)")

    def round_o2_fixed(self) -> None:
        """Part 3: from along O2's front face, round O2's end on the arrow side, along its back face and 90 deg at
        its other end to face the carpark (see U_TURN_MOVES)."""
        _, sensor, _ = hug_side(self.arrow2)
        self.o2_x = self.o1_x                     # team rule: O2 is centred on O1
        if self.beside_o2(sensor, "after the turn onto O2"):
            self.result["o2_front"] = "hug"
            self.log("still beside O2: wall hug its front face to the end")
            self.hug("front")
        else:
            self.result["o2_front"] = "reverse"
            cap = self.reverse_to_o2(sensor)
            self.result["o2_front_edge_cap_cm"] = cap
            self.hug("front", cap)
        self.fixed(U_TURN_MOVES[self.arrow2], "U-turn round O2's end onto its back face")
        # (A reverse to O2's middle when the pose put the IR near O2's far end, robot run 22:34, backed the IR past
        # the NEAR end: the dead-reckoned x after the C curve and the turn onto O2 was ~10 cm off. Removed.)
        if not self.beside_o2(sensor, "after the U-turn"):
            self.log("the side IR does not see O2: reverse until it does")
            self.reverse_onto_o2(sensor, "back")
        self.hug("back")
        self.face_carpark_sharp()

    def return_to_carpark(self) -> None:
        """Part 4: facing the carpark beside O2's far end, onto O1's centre line with one S-curve that clears O1, then
        straights by the ultrasonic to PARK_US_CM from the carpark's back wall."""
        centre = (self.o2_ends[1] + self.o2_ends[-1]) / 2       # O2's middle = O1's centre line (team rule)
        o1_back = self.o2_face - self.o1_o2_gap_cm               # O2's face as the side IR re-anchored it
        half = config.O1_SIZE_CM / 2
        rects = [Rect(centre - half, o1_back - config.O1_SIZE_CM, centre + half, o1_back, "O1")]   # (driving away from O2)
        offset = self.pose.x - centre                            # + east of the line
        rx, _ = self.pose.right()
        first, second = ("R", "L") if offset * rx < 0 else ("L", "R")   # turn towards the line first
        across = config.TURN_RADIUS_CM[f"F{first}"] + config.TURN_RADIUS_CM[f"F{second}"]
        sideways = abs(offset)
        if sideways <= across:
            angle, middle = round(math.degrees(math.acos(1 - sideways / across))), 0
        else:
            angle, middle = 90, round(sideways - across)
        s_curve = ([f"MOV,F{first},{angle}"] + straight_moves(middle) + [f"MOV,F{second},{angle}"]) if angle else []
        self.log(f"facing the carpark {offset:+.1f} cm from O1's centre line (O2 {self.o2_ends[1] - self.o2_ends[-1]:.1f} "
                 f"cm long); O1's back face {self.pose.y - o1_back:.1f} cm ahead of the rear axle")
        o1_front = o1_back - config.O1_SIZE_CM
        for lead in range(RETURN_LEAD_MAX_CM + 1):
            moves = straight_moves(lead) + s_curve
            end = simulate(self.pose, moves)
            past = (end.y - (o1_front - RETURN_O1_MARGIN_CM)) * self.pose.forward()[1] >= 0   # rear axle past O1
            if past and hit_in(swept_polygons(self.pose, moves, RETURN_O1_MARGIN_CM), rects) is None:
                break
        else:
            raise RunFail(f"no straight up to {RETURN_LEAD_MAX_CM} cm lets {s_curve} clear O1")
        self.result.update(return_moves=moves, return_end_off_line_cm=round(end.x - centre, 1))
        self.log(f"onto O1's centre line: {moves} (ends {end.x - centre:+.1f} cm off it, planned)")
        for command in moves:
            self.send(command)
        distance = self.drive_to(PARK_US_CM, "into the carpark", PARK_US_TOLERANCE_CM,
                                 tracked_cm=self.tracked_wall_cm() if self.park_one_shot else None)
        self.result["park_ultrasonic_cm"] = round(distance, 1)
        self.log(f"in the carpark: ultrasonic {distance:.1f} cm from the back wall")

    def face_carpark_sharp(self) -> None:
        """The last turn: a FACE_CARPARK_LEAD_CM straight (forward or back) then FACE_CARPARK_TURN, the one whose
        planned end is nearest FACE_CARPARK_TARGET_GAP_CM from O2's end face and that keeps FACE_CARPARK_MARGIN_CM
        from O2 and a possible wall (one sweep check per straight, milliseconds)."""
        along, _, _ = hug_side(self.arrow2)
        turn = FACE_CARPARK_TURN[self.arrow2]
        far = -along                                  # the far end lies this way
        best = None                                   # (how far off the target, |straight|, moves, gap)
        leads = FACE_CARPARK_LEAD_CM[self.arrow2]
        letter = turn.split(",")[1][1]
        shapes = [[turn]] + [[f"MOV,F{letter},{a}", f"MOV,B{letter},{90 - a}"] for a in FACE_CARPARK_SPLIT_DEG[self.arrow2]]
        rects = self.o2_rects()
        for lead, shape in itertools.product(leads, shapes):
            moves = straight_moves(lead) + shape
            end = simulate(self.pose, moves)
            gap = far * (end.x - self.o2_ends[far]) - config.WIDTH_CM / 2
            if gap < 0:
                continue
            polygons = swept_polygons(self.pose, moves, 0.0)
            reach = max(far * x for _, polygon in polygons for x, _ in polygon) - far * self.o2_ends[far]
            if reach > FACE_CARPARK_MAX_REACH_CM[self.arrow2] or \
                    hit_in(swept_polygons(self.pose, moves, FACE_CARPARK_MARGIN_CM[self.arrow2]), rects) is not None:
                continue
            key = (round(abs(gap - FACE_CARPARK_TARGET_GAP_CM[self.arrow2]), 1), len(shape), abs(lead), moves, gap)
            if best is None or key[:3] < best[:3]:
                best = key
        if best is None:
            raise RunFail(f"no straight of {leads[0]} to {leads[-1]} cm lets {turn} "
                          f"clear O2 and a possible wall")
        *_, moves, gap = best
        self.result["carpark_turn"], self.result["end_gap_to_o2_cm"] = moves, round(gap, 1)
        if gap > FACE_CARPARK_MAX_GAP_CM:
            self.log(f"WARNING: the turn to face the carpark ends {gap:.1f} cm from O2's end face (planned)")
        self.fixed(moves, f"turn round O2's other end to face the carpark (robot side {gap:.1f} cm from O2, planned)",
                   margin_cm=FACE_CARPARK_MARGIN_CM[self.arrow2])
        back_face = self.o2_face + config.O2_DEPTH_CM
        along_y = self.pose.forward()[1]                      # -1 facing the carpark (south)
        level = straight_moves((self.pose.y - (back_face - FACE_CARPARK_REAR_FROM_BACK_CM)) * -along_y)
        self.result["carpark_level"] = level
        if level:
            self.fixed(level, f"level with O2: rear axle {self.pose.y - back_face:+.1f} cm from O2's back face",
                       margin_cm=FACE_CARPARK_MARGIN_CM[self.arrow2])

    def beside_o2(self, sensor: str, tag: str) -> bool:
        """One static IR,GET: True if the side IR sees O2 (at most O2_BESIDE_CM), False if it sees nothing near
        (FAR, or further than that); anything untrusted (NEAR, a fault, a stale or lost reply) stops the run."""
        reading = self.stm.read_ir().reading(IrSensor(sensor))
        self.log(f"side IR {sensor} {tag}: {reading.status.value} {reading.distance_cm} cm"
                 + (" (FAR)" if reading.far else ""))
        if reading.status is IrStatus.VALID_DISTANCE:
            return reading.distance_cm <= O2_BESIDE_CM
        if reading.far or reading.status is IrStatus.TRUSTWORTHY_NO_WALL:
            return False
        raise RunFail(f"{tag}: the {sensor} IR reading is not usable ({reading.status.value})")

    def reverse_to_o2(self, sensor: str) -> int:
        """Front face, the side IR sees nothing (the turn carried it past O2's end): back along the face in ONE
        straight to REVERSE_INSIDE_CM short of where O2's end certainly is (O2 reaches at least o2_min_half_cm either
        side of O2's centre, which is O1's), then one IR,GET. Not seen (or no such distance up to REVERSE_MAX_CM), the
        5 cm steps of reverse_onto_o2 go on from there. Returns the cap for the edge-follow that follows."""
        along, _, _ = hug_side(self.arrow2)
        beyond = along * (sensor_point(self.pose, sensor)[0] - self.o2_x)     # the side IR past O2's centre, outward
        reach = max(0.0, self.o2_min_half_cm - REVERSE_INSIDE_CM)
        cm = round(beyond - reach)
        if not REVERSE_ONE_MOVE or not 1 <= cm <= REVERSE_MAX_CM:
            self.log("already past O2's end: reverse until the side IR sees O2")
            self.reverse_onto_o2(sensor)
            return EDGE_CAP_CM
        self.log(f"already past O2's end, the side IR {beyond:.1f} cm beyond O2's centre: one reverse of {cm} cm "
                 f"(to {reach:.1f} cm from the centre)")
        self.reverse(cm, "front", "reverse onto O2")
        if self.beside_o2(sensor, f"after reversing {cm} cm"):
            self.result["o2_front_reverse_cm"] = cm
            return max(EDGE_CAP_CM, cm + REVERSE_EDGE_MARGIN_CM)
        self.log("the side IR still does not see O2: reverse on in steps")
        self.reverse_onto_o2(sensor, backed=cm)
        return EDGE_CAP_CM

    def reverse_onto_o2(self, sensor: str, face: str = "front", backed: int = 0) -> None:
        """Straight back in REVERSE_STEP_CM steps until the side IR sees O2, then REVERSE_PAST_CM more; each step
        checked against O1, O2 and a possible wall beyond a measured end. `backed` is what was already reversed
        (counted against the most it may go)."""
        most = REVERSE_MAX_CM if face == "front" else REVERSE_BACK_MAX_CM
        while backed < most:
            step = min(REVERSE_STEP_CM, most - backed)
            self.reverse(step, face, "reverse to find O2")
            backed += step
            if self.beside_o2(sensor, f"after reversing {backed} cm"):
                self.reverse(REVERSE_PAST_CM, face, "reverse onto O2")
                self.result[f"o2_{face}_reverse_cm"] = backed + REVERSE_PAST_CM
                return
        raise RunFail(f"the side IR did not see O2 after reversing {backed} cm")

    def reverse(self, cm: int, face: str, why: str) -> None:
        """Front face: checked against O1 only (it backs along O2 towards its middle). Back face: also against O2 and
        the possible wall beyond its measured near end, which it backs towards."""
        if face == "front":
            self.straight(-cm, why)
        else:
            self.fixed([f"MOV,B,{cm}"], f"{why} (back face)", measured_walls_only=True)

    def fixed(self, moves: list[str], why: str, measured_walls_only: bool = False,
              margin_cm: float = FIXED_O2_MARGIN_CM) -> None:
        """One sweep check of fixed moves against O2 (its measured ends) and a possible wall END_WALL_CM beyond each
        end (FIXED_O2_MARGIN_CM), then send them."""
        rects = self.o2_rects()
        if measured_walls_only:   # the wall beyond an unmeasured end is placed for a robot facing north: leave it out
            rects = [r for r in rects if not (r.name.startswith("possible wall")
                                              and (1 if r.name.endswith("east") else -1) not in self.o2_ends)]
        if hit_in(swept_polygons(self.pose, moves, margin_cm), rects) is not None:
            raise RunFail(f"{why}: {moves} would come within {margin_cm:g} cm of O2 or a possible wall")
        self.log(f"{why}: {moves}")
        for command in moves:
            self.send(command)

    def bypass_o1(self) -> None:
        started = time.monotonic()
        plan = plan_bypass(self.arrow, self.o1_x, self.o1_face, self.pose)
        if plan is None:
            raise RunFail(f"no S-curve round O1 ({self.arrow}) clears it")
        self.log(f"bypass O1 {self.arrow}: {plan.moves} ({plan.length_cm:.0f} cm, lane {plan.end.x - self.o1_x:+.1f} cm "
                 f"from O1's centre, planned in {time.monotonic() - started:.2f} s)")
        for command in plan.moves:
            self.send(command)

    def approach_o2(self) -> None:
        distance = self.drive_to(self.o2_read_cm, "approach O2")
        self.o2_face = self.pose.y + config.ULTRASONIC_FORWARD_CM + distance
        self.result["o2_ultrasonic_cm"] = round(distance, 1)
        self.log(f"at O2's reading position: {distance:.1f} cm from O2")

    def turn(self, command: str, why: str) -> None:
        if not is_clear(self.pose, [command], self.o1_rect(), EXTRA_MARGIN_CM):
            raise RunFail(f"{why}: {command} would come within {config.MARGIN_CM + EXTRA_MARGIN_CM:g} cm of O1")
        self.log(why)
        self.send(command)

    def read_o2(self) -> None:
        depth_cam = self.result["o2_ultrasonic_cm"] + config.ULTRASONIC_FORWARD_CM - config.CAMERA_FORWARD_CM
        offset = abs(self.pose.x - self.o1_x)
        first = max(INWARD_MIN_DEG, min(INWARD_MAX_DEG, round(math.degrees(math.atan2(offset, depth_cam)))))
        # Passed left: O2's middle is to the right. A reverse inward turn (BR, undone by FL: same arc) swings
        # the rear AWAY from O1, which may still be beside the robot; the forward one (FR / BL) only if that
        # is the one that clears O1 for the whole inward rotation.
        pairs = [("BR", "FL"), ("FR", "BL")] if self.arrow == "LEFT" else [("BL", "FR"), ("FL", "BR")]
        total = first + INWARD_MORE_DEG
        clear = [pair for pair in pairs if is_clear(self.pose, [f"MOV,{pair[0]},{total}"], self.o1_rect(), EXTRA_MARGIN_CM)]
        if not clear:
            raise RunFail(f"no inward turn of {total} deg clears O1")
        inward, undo = clear[0]
        turned, found, failure = 0, None, None
        try:
            for extra in (first,) + ((INWARD_MORE_DEG,) if INWARD_RETRY else ()):
                self.turn(f"MOV,{inward},{extra}", f"turn {extra} deg inward to see O2's arrow")
                turned += extra
                cam = self.pose.point(config.CAMERA_FORWARD_CM)
                (fx, fy), (rx, ry) = self.pose.forward(), self.pose.right()
                depth = (self.o2_face - cam[1]) / fy               # along the turned camera axis to O2's face
                try:
                    found = self.read(f"o2_arrow_{turned}deg_", depth)
                    def on_face(lateral: float) -> float:          # the camera ray onto the face
                        t = (self.o2_face - cam[1]) / (fy * depth + ry * lateral)
                        return cam[0] + t * (fx * depth + rx * lateral)

                    camera_x = on_face(found[1])
                    self.o2_bullseyes = sorted(on_face(lateral_cm(d.norm_x(), depth)) for d in found[2].frames[0]
                                               if kind_of(d.label) == "marker" and d.conf >= config.MIN_CONFIDENCE)
                    break
                except RunFail as exc:
                    failure = exc
        finally:
            if turned:
                self.turn(f"MOV,{undo},{turned}", f"undo the {turned} deg inward turn")
        if found is None:
            raise failure or RunFail("no arrow seen on O2")
        side, lateral, look = found
        self.arrow2 = side
        self.show_arrow(2, side, look)
        # Team rule (2026-10-14): O2 is always centred on O1, so its mid-point is O1's centre, which the camera
        # measured from 30 cm; the arrow seen from the lane is only a cross-check.
        self.o2_x = self.o1_x
        if abs(camera_x - self.o2_x) > O2_CENTRE_CHECK_CM:
            self.log(f"WARNING: O2's arrow seen at x {camera_x:+.1f}, {camera_x - self.o2_x:+.1f} cm from O1's centre "
                     f"(O2 is assumed centred on O1)")
        along, _, _ = hug_side(side)
        if side == self.arrow and not any(along * (x - self.o2_x) >= BULLSEYE_MIN_FROM_ARROW_CM
                                          for x in self.o2_bullseyes):
            self.look_for_bullseyes()
        self.result.update(o2_arrow=side, o2_turn_deg=turned, o2_arrow_camera_x=round(camera_x, 1),
                           o2_image=str(look.annotated_path) if look.annotated_path else None)
        self.log(f"arrow on O2: {side}, seen at x {camera_x:+.1f} cm, O2 centred on O1 at {self.o2_x:+.1f} (read after "
                 f"a {turned} deg inward turn); "
                 f"bullseyes at x {[round(x, 1) for x in self.o2_bullseyes]}")

    def show_arrow(self, obstacle_id: int, side: str, look) -> None:
        """Keep the look an arrow was read from, and send its picture to the PC page (rules 8 and 10)."""
        self.arrow_looks[obstacle_id] = look
        if self.uploader is not None:
            self.uploader.send(obstacle_id, side, getattr(look, "annotated_path", None))

    def look_for_bullseyes(self) -> None:
        """One more frame, facing north from the lane: the inward turn for the arrow can leave the bullseye on
        O2's outer end out of frame."""
        cam = self.pose.point(config.CAMERA_FORWARD_CM)
        depth = self.o2_face - cam[1]
        look = self.camera.look("o2_bullseye", 1)
        seen = [cam[0] + lateral_cm(d.norm_x(), depth) for d in look.frames[0]
                if kind_of(d.label) == "marker" and d.conf >= config.MIN_CONFIDENCE]
        self.o2_bullseyes = sorted(self.o2_bullseyes + seen)
        self.log(f"bullseye look facing north: x {[round(x, 1) for x in seen]}")

    # -- wall hug round O2 -----------------------------------------------------------------------
    def o2_rects(self) -> list[Rect]:
        """O2 (measured ends where known, else UNKNOWN_CM on past the arrow) and the wall that may stand
        END_WALL_CM beyond each end: beyond the measured end (O2 longer and the wall nearer by its tolerance), or, before it is measured, beyond the shortest O2
        that still carries the arrow (the nearest that wall can legally be), but never where the robot is."""
        west = self.o2_ends.get(-1, self.o2_x - UNKNOWN_CM) - self.o2_end_tol.get(-1, 0.0)
        east = self.o2_ends.get(1, self.o2_x + UNKNOWN_CM) + self.o2_end_tol.get(1, 0.0)
        face = self.o2_face
        rects = self.o1_rect() + [Rect(west, face, east, face + config.O2_DEPTH_CM, "O2")]
        for side in (1, -1):
            wall = (self.o2_ends.get(side, self.o2_x + side * self.o2_min_half_cm)
                    + side * (END_WALL_CM - self.o2_end_tol.get(side, 0.0)))
            if side not in self.o2_ends:      # the robot stands where it is, so no wall is there
                clear = self.pose.x + side * (config.WIDTH_CM / 2 + config.MARGIN_CM + EXTRA_MARGIN_CM + 0.5)
                wall = max(wall, clear) if side > 0 else min(wall, clear)
            rects.append(Rect(min(wall, wall + side * 10), face - WALL_REACH_CM, max(wall, wall + side * 10),
                              face + config.O2_DEPTH_CM + WALL_REACH_CM, f"possible wall {'east' if side > 0 else 'west'}"))
        return rects

    def execute(self, moves: list[str], why: str) -> None:
        if not is_clear(self.pose, moves, self.o2_rects(), EXTRA_MARGIN_CM):
            raise RunFail(f"{why}: {moves} is not clear of O1/O2/a possible wall")
        self.log(f"{why}: {moves}")
        for command in moves:
            self.send(command)

    @staticmethod
    def hug_target(sensor_x: float, face_y: float, north: bool, heading: float, sensor: str) -> tuple[float, float]:
        """Rear axle position with the side IR at `sensor_x`, HUG_CM north (back face) or south (front face) of
        the face at `face_y`."""
        return axle_for_sensor((sensor_x, face_y + (HUG_CM if north else -HUG_CM)), heading, sensor)

    def front_starts(self, along: int) -> list[float]:
        """Side IR x positions where O2 certainly is, so the front hug can start there: from just short of the
        arrow (O2's mid-point) outward to o2_min_half_cm, or to where the ultrasonic, from the lane, saw O2
        within its cone, whichever is further."""
        lane_x = self.pose.x
        cone = self.result["o2_ultrasonic_cm"] * math.tan(math.radians(US_CONE_HALF_DEG))
        seen = lane_x - cone if lane_x > self.o2_x else lane_x + cone      # the cone edge nearest the arrow
        reach = max(along * (seen - self.o2_x), self.o2_min_half_cm)
        starts, k = [], -3.0
        while k <= max(reach - 3.0, -3.0):
            starts.append(self.o2_x + along * k)
            k += FRONT_START_STEP_CM
        return starts

    def enter_front_face(self) -> None:
        """Stage 6: from the reading position, run along O2's front face towards the arrow side with the side IR
        on a part of O2 that is certainly there (front_starts): the shortest clear path. Odometry only: from the
        lane O2 may be off the edge of the ultrasonic's cone (a short O2), so its readings are not trusted here;
        the side IR re-anchors the face before the hug."""
        along, sensor, _ = hug_side(self.arrow2)
        heading = 0.0 if along > 0 else 180.0
        started = time.monotonic()
        starts = self.front_starts(along)
        targets = [self.hug_target(x, self.o2_face, False, heading, sensor) for x in starts]
        found = plan_to(self.pose, heading, targets, self.o2_rects())
        if found is None:
            raise RunFail(f"no path onto O2's front face ({self.arrow2}) clears O1/O2")
        plan, i = found
        self.log(f"front face entry: {plan} ({path_length(plan):.0f} cm, side IR at x {starts[i]:+.1f}, arrow at "
                 f"{self.o2_x:+.1f}; planned in {time.monotonic() - started:.2f} s)")
        self.execute(plan, "onto O2's front face")

    def ir_beside(self, sensor: str, tag: str) -> float:
        """One static IR,GET: the side IR must see O2 within the STM's VISIBLE range; returns the face's y."""
        reading = self.stm.read_ir().reading(IrSensor(sensor))
        self.log(f"side IR {sensor} {tag}: {reading.status.value} {reading.distance_cm} cm"
                 + (" (FAR)" if reading.far else ""))
        if reading.status is not IrStatus.VALID_DISTANCE or reading.distance_cm > config.STM_IR_VISIBLE_CM:
            raise RunFail(f"{tag}: the {sensor} IR does not see O2 within {config.STM_IR_VISIBLE_CM:g} cm")
        return sensor_point(self.pose, sensor)[1] + beam_y(self.pose, sensor) * reading.distance_cm

    def hug(self, face: str, cap_cm: int | None = None) -> None:
        """Stages 7 and 9: one MOV,FIRL/FIRR along O2's front or back face to the end of O2 ahead. The end is
        where the side IR was when the STM confirmed the wall gone (IREDGE CC), so O2 ends at or before it."""
        along, sensor, _ = hug_side(self.arrow2)
        direction = along if face == "front" else -along
        planned = self.o2_face if face == "front" else self.o2_face + config.O2_DEPTH_CM
        measured = self.ir_beside(sensor, f"before the {face} hug")
        if abs(measured - planned) > REANCHOR_MAX_CM:
            raise RunFail(f"{face} hug: the side IR puts O2's {face} face {measured - planned:+.1f} cm from the plan")
        self.o2_face = measured if face == "front" else measured - config.O2_DEPTH_CM
        start = self.pose
        start_x = sensor_point(start, sensor)[0]
        cap = self.fir_cap_cm if cap_cm is None else cap_cm
        self.log(f"{face} hug: MOV,FIR{sensor} cap {cap} cm from sensor x {start_x:+.1f}")
        result = self.stm.follow_edge(sensor, cap)
        # The robot is at least where the STM confirmed the edge: today's firmware gives the travel from the last
        # MM,D sample before braking, which can be short of it (robot run 2026-10-14: back face 27.4 cm, edge 30.0 cm;
        # the Pi then put the robot short of O2's end and refused the turn to face the carpark).
        # It also runs on after that: END came 65-120 ms after IREDGE at 19-32 cm/s (robot runs 2026-10-14), plus braking.
        reached = (result.edge_at_cm + FIR_STOP_PAST_CM) if result.edge_at_cm is not None else 0.0
        self.pose = Pose(*start.point(max(result.travel_cm, reached)), start.h)
        self.log(f"{face} hug: {result.status} travel {result.travel_cm:.1f} cm, edge at {result.edge_at_cm} cm "
                 f"{result.line} {' '.join(result.problems)}".rstrip())
        if result.status != "EDGE" or result.edge_at_cm is None:
            raise RunFail(f"{face} hug ended {result.status} {result.line}".rstrip())
        end = start_x + direction * max(0.0, result.edge_at_cm - config.IR_EDGE_LAG_CM)   # O2 ends before the STM confirms it
        if direction * (end - self.o2_x) <= 0:
            raise RunFail(f"{face} hug: an end at x {end:+.1f} is not past the arrow at {self.o2_x:+.1f}")
        self.o2_ends[direction] = end
        self.result[f"o2_end_{'east' if direction > 0 else 'west'}_x"] = round(end, 1)
        self.log(f"O2 ends {'east' if direction > 0 else 'west'} at x {end:+.1f} cm; pose {self.pose}")
        if len(self.o2_ends) == 2:
            span = self.o2_ends[1] - self.o2_ends[-1]
            self.result["o2_span_cm"] = round(span, 1)
            self.log(f"O2 span {span:.1f} cm, centre {(self.o2_ends[1] + self.o2_ends[-1]) / 2:+.1f} "
                     f"(arrow at {self.o2_x:+.1f})")

    def bullseye_end(self) -> bool:
        """Both arrows the same way: the lane is already on O2's arrow side. O2's near end is taken from the
        bullseye on its front face on that side (the furthest out, if more than one), so the robot can go
        straight round it with no front-face hug. False (and nothing changes) if none was seen."""
        along, _, _ = hug_side(self.arrow2)
        out = [x for x in self.o2_bullseyes if along * (x - self.o2_x) >= BULLSEYE_MIN_FROM_ARROW_CM]
        if not out:
            self.log(f"no bullseye seen on O2's {self.arrow2.lower()} side: front face entry and hug instead")
            return False
        marker = max(out, key=lambda x: along * x)
        end = marker + along * BULLSEYE_TO_END_CM
        self.o2_ends[along], self.o2_end_tol[along] = end, BULLSEYE_TOL_CM
        self.result[f"o2_end_{'east' if along > 0 else 'west'}_x"] = round(end, 1)
        self.result["o2_near_end_from"] = "bullseye"
        self.log(f"O2 ends {'east' if along > 0 else 'west'} at x {end:+.1f} cm (bullseye at {marker:+.1f}, "
                 f"+-{BULLSEYE_TOL_CM:g} cm): no front-face hug")
        return True

    def round_from_lane(self) -> bool:
        """Both arrows the same way and the near end known from its bullseye: from the lane, line up beside O2's
        end face (the short 10 cm side at that end), edge-follow it north with the side IR (MOV,FIRL / MOV,FIRR:
        the STM stops once the face is seen and then gone, i.e. past O2's back corner), then U-turn onto the back
        face. Both the line-up and the U-turn from where the edge-follow should stop are planned before anything
        moves; False, with nothing sent, when this does not apply or either has no clear path."""
        if self.arrow2 != self.arrow or not self.bullseye_end():
            return False
        along, sensor, _ = hug_side(self.arrow2)
        end = self.o2_ends[along]
        started = time.monotonic()
        lineup = axle_for_sensor((end + along * HUG_CM, self.o2_face - END_FACE_LEAD_CM), 90.0, sensor)
        found = plan_to(self.pose, 90.0, [lineup], self.o2_rects(), pre_limit=MID_LIMIT_CM)
        stop = Pose(*Pose(*lineup, 90.0).point(END_FACE_LEAD_CM + config.O2_DEPTH_CM + END_FACE_STOP_PAST_CM), 90.0)
        after = self.plan_u_turn(stop) if found is not None else None
        if after is None:
            del self.o2_ends[along], self.o2_end_tol[along]
            self.result.pop("o2_near_end_from", None)
            self.log(f"no clear {'line-up beside O2s end face' if found is None else 'U-turn after the end face'} "
                     f"from the lane: front face entry and hug instead")
            return False
        plan, _ = found
        self.log(f"end face line-up: {plan} ({path_length(plan):.0f} cm, planned with the U-turn in "
                 f"{time.monotonic() - started:.2f} s)")
        self.execute(plan, "line up beside O2's end face")
        self.follow_end_face()
        try:
            self.u_turn()
        except NoPlan as exc:              # the robot has moved: no fallback any more
            raise RunFail(f"{exc} after the end face") from None
        return True

    def follow_end_face(self) -> None:
        """One MOV,FIRL / MOV,FIRR north beside O2's end face. The STM confirms the face (four VISIBLE scans) and
        then its end (four CLEAR scans) and stops: the side IR has just passed O2's back corner. That corner's y
        (IREDGE CC, less IR_EDGE_LAG_CM) re-anchors O2's faces for the U-turn."""
        along, sensor, _ = hug_side(self.arrow2)
        start = self.pose
        sensor_y = sensor_point(start, sensor)[1]
        self.log(f"end face: MOV,FIR{sensor} cap {END_FACE_CAP_CM} cm from sensor y {sensor_y:.1f} "
                 f"(O2's front face at {self.o2_face:.1f})")
        result = self.stm.follow_edge(sensor, END_FACE_CAP_CM)
        self.pose = Pose(*start.point(result.travel_cm), start.h)
        self.log(f"end face: {result.status} travel {result.travel_cm:.1f} cm, edge at {result.edge_at_cm} cm "
                 f"{result.line} {' '.join(result.problems)}".rstrip())
        if result.status != "EDGE" or result.edge_at_cm is None:
            raise RunFail(f"end face follow ended {result.status} {result.line}".rstrip())
        back = sensor_y + result.edge_at_cm - config.IR_EDGE_LAG_CM
        planned = self.o2_face + config.O2_DEPTH_CM
        if abs(back - planned) > REANCHOR_MAX_CM:
            raise RunFail(f"end face: O2's back corner at y {back:.1f}, {back - planned:+.1f} cm from the plan")
        self.o2_face = back - config.O2_DEPTH_CM
        self.result["o2_back_from_end_face_y"] = round(back, 1)
        self.log(f"O2's back corner at y {back:.1f} ({back - planned:+.1f} cm from the ultrasonic); pose {self.pose}")

    def plan_u_turn(self, start: Pose) -> tuple[list[str], int, list[int]] | None:
        """The U-turn of u_turn from `start`: (moves, index into the insets, the insets tried), or None."""
        along, sensor, _ = hug_side(self.arrow2)
        heading = 180.0 if along > 0 else 0.0
        end = self.o2_ends[along]
        tol = self.o2_end_tol.get(along, 0.0)
        far = 2 * (abs(end - self.o2_x) - tol) - FAR_END_MARGIN_CM      # the shortest O2 the arrow allows
        insets = [inset for inset in BACK_START_CM if tol + 5.0 <= inset <= max(far, abs(end - self.o2_x))]
        found = plan_to(start, heading, [self.hug_target(end - along * inset, self.o2_face + config.O2_DEPTH_CM,
                                                         True, heading, sensor) for inset in insets],
                        self.o2_rects(), pre_limit=MID_LIMIT_CM)
        return None if found is None else (found[0], found[1], insets)

    def u_turn(self) -> None:
        """Stage 8: round the near end onto the back face, facing back, with the side IR HUG_CM from the back face
        and one of BACK_START_CM inside the end: the shortest clear one. The arrow is at O2's mid-point, so the
        far end is about as far beyond the arrow as the near end is before it; the side IR starts
        FAR_END_MARGIN_CM short of that, or of the arrow if O2 may be very short (o2_min_half_cm). Plans from
        wherever the robot is (on the front face after its hug, or just past the end face)."""
        started = time.monotonic()
        found = self.plan_u_turn(self.pose)
        if found is None:
            raise NoPlan("no U-turn round O2's end clears O2 and a possible wall")
        plan, i, insets = found
        self.log(f"U-turn: {plan} ({path_length(plan):.0f} cm, side IR {insets[i]} cm inside the end, planned in "
                 f"{time.monotonic() - started:.2f} s)")
        self.execute(plan, "U-turn onto O2's back face")

    def face_carpark(self) -> None:
        """Stage 10: past O2's second end, 90 deg towards O2 to face the carpark (south), and stop."""
        _, _, letter = hug_side(self.arrow2)
        plan = plan_final_turn(self.pose, letter, self.o2_rects())
        if plan is None:
            raise RunFail("no turn to face the carpark clears O2 and a possible wall")
        self.execute(plan, "turn to face the carpark")


# --- program ------------------------------------------------------------------------------------
START_WORDS = {"START", "ALG:START", "RESUME"}   # same as rpi_task1/task1.py


def wait_for_android_start(android) -> None:
    """Block until the tablet sends a start word; anything sent before the Pi was ready is dropped."""
    while True:
        try:
            android.inbox.get_nowait()
        except queue.Empty:
            break
    android.send("MSG,[Pi ready: press START]")
    print("Waiting for START from the tablet (Ctrl+C cancels)")
    while True:
        try:
            line = android.inbox.get(timeout=0.5)
        except queue.Empty:
            continue
        if line == "__CONNECTED__":
            android.send("MSG,[Pi ready: press START]")
        elif line.upper() in START_WORDS:
            return
        elif line != "__DISCONNECTED__":
            android.send(f"MSG,[Ignored: {line[:40]}]")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stop-cm", type=float, default=STOP_CM, help="ultrasonic reading at which O1's arrow is read")
    parser.add_argument("--o2-read-cm", type=float, default=O2_READ_CM, help="ultrasonic reading at which O2's arrow is read")
    parser.add_argument("--stm", default="/dev/ttyACM0")
    parser.add_argument("--model", type=Path, default=Path(__file__).resolve().parent / "task2_ncnn_model")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--out", type=Path, default=Path.home() / "mdp/task2_runs")
    parser.add_argument("--profile", choices=("PRECISION", "STANDARD", "TASK2TEST"), default="PRECISION")
    parser.add_argument("--ultrasonic-forward", type=float, default=config.ULTRASONIC_FORWARD_CM)
    parser.add_argument("--no-heading-fix", dest="heading_fix", action="store_false")
    parser.add_argument("--no-one-shot-park", dest="park_one_shot", action="store_false",
                        help="always stop short and measure again when driving back into the carpark; by default one "
                             "straight is driven when the Pi's own position agrees with the ultrasonic")
    parser.add_argument("--no-fast-straights", dest="fast_straights", action="store_false",
                        help=f"drive every move in --profile; by default forward straights of {FAST_STRAIGHT_CM} cm or "
                             f"more run in {FAST_PROFILE}")
    parser.add_argument("--c-curve", dest="c_curve", action="store_true", default=True,
                        help="(the default, team instruction 2026-10-15) approach O1, its arrow, the fixed C curve "
                             "round O1, O2's arrow, the turn onto O2, round O2, back into the carpark")
    parser.add_argument("--lane-route", dest="c_curve", action="store_false",
                        help="the older lane route instead of the C curve (simulation only)")
    parser.add_argument("--no-wall-hug", dest="wall_hug", action="store_false",
                        help="stop after the turn onto O2 (lane route: after reading O2's arrow)")
    parser.add_argument("--pc", metavar="LAPTOP",
                        help="laptop running arrow_server.py (address or URL; port 5001 if none): shows the two "
                             "arrow pictures there, as the rules require")
    parser.add_argument("--fir-cap-cm", type=int, default=FIR_CAP_CM, help="the most one wall hug may drive")
    parser.add_argument("--o2-min-half-cm", type=float, default=O2_MIN_HALF_CM,
                        help="O2 certainly reaches this far each side of its arrow (half its shortest length)")
    parser.add_argument("--android", action="store_true",
                        help="start on START from the tablet over Bluetooth instead of Enter "
                             "(sudo systemctl stop rfcomm first)")
    parser.add_argument("--bt-channel", type=int, default=1)
    args = parser.parse_args()
    config.ULTRASONIC_FORWARD_CM = args.ultrasonic_forward


    from perception import Camera
    from stm_link import StmLink, open_serial
    from stm_task2 import Task2Stm

    run_dir = args.out / (time.strftime("%Y%m%d-%H%M%S-") + "final")
    run_dir.mkdir(parents=True, exist_ok=True)
    if args.c_curve and args.stop_cm != STOP_CM:
        print(f"Not starting: the fixed C curves are for --stop-cm {STOP_CM:g}")
        return
    android = None
    if args.android:
        from android_link import AndroidLink
        android = AndroidLink(args.bt_channel)
        android.start()                                # the tablet can connect while the camera loads
    started = time.monotonic()
    print("Loading the camera and the model ...")
    camera = Camera(args.model, imgsz=args.imgsz)
    print(f"camera ready ({time.monotonic() - started:.0f} s)")
    camera.out_dir = run_dir
    port = open_serial(args.stm)
    raw_log = open(args.out / "stm_telemetry.log", "a")
    link = StmLink(port, raw_log=raw_log)
    last_signal = [0.0]

    def on_signal(signum: int, _frame: object) -> None:
        now = time.monotonic()
        if now - last_signal[0] < 2.0:
            raise KeyboardInterrupt
        last_signal[0] = now
        link.request_stop(signal.Signals(signum).name)
        print(f"\n{signal.Signals(signum).name}: emergency STOP requested")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        if not link.emergency_stop("fence STOP before the run"):
            print("Not starting: no ACK,STOPPED to the fence STOP")
            return
        if not link.resync():
            print("Not starting: the STM line did not go quiet")
            return
        stm = Task2Stm(link)
        stm.fir_legacy_travel = True                   # today's firmware END has no DL/DR (D-76, supervised only)
        link.note(f"final_run {run_dir.name}")
        reason = stm.apply_profile(args.profile)
        if reason:
            print(f"Not starting: profile: {reason}")
            return
        reason = stm.set_uspark(False)                 # never armed for ordinary driving
        if reason:
            print(f"Not starting: {reason}")
            return
        reason = stm.probe_usd_get()                   # USD,GET (fast) if the firmware has it, else the US, stream
        print("front ultrasonic: USD,GET queries" if reason is None
              else f"front ultrasonic: US, bench stream ({reason})")
        if not link.resync():                          # an old firmware's ERR leaves nothing behind, but be sure
            print("Not starting: the STM line did not go quiet after the USD,GET probe")
            return
        run = FinalRun(stm, camera, args.stop_cm, args.o2_read_cm, heading_fix=args.heading_fix, wall_hug=args.wall_hug,
                       c_curve=args.c_curve, fast_straights=args.fast_straights, base_profile=args.profile,
                       park_one_shot=args.park_one_shot)
        run.fir_cap_cm = args.fir_cap_cm
        run.o2_min_half_cm = args.o2_min_half_cm
        if args.pc:
            run.uploader = ArrowUploader(args.pc, run_dir.name)
            print(f"arrow pictures go to {run.uploader.url}")
        else:
            print("No --pc: the arrow pictures are only saved on the Pi (the rules need them on a screen)")
        if android is not None:
            wait_for_android_start(android)
        else:
            input("Robot in the carpark facing O1? Press Enter to start (Ctrl+C cancels) ")
        try:
            ok = run.run()
        except Exception as exc:                       # a refused link (STOP), a camera failure, ...
            print(f"FAIL_SAFE: {exc!r}")
            ok = False
        print(f"result: {'DONE' if ok else 'FAIL_SAFE'} {run.result}")
        if android is not None:
            arrows = ", ".join(f"O{n} {run.result[f'o{n}_arrow']}" for n in (1, 2) if f"o{n}_arrow" in run.result)
            android.send(f"MSG,[Run {'done' if ok else 'stopped'}{': ' + arrows if arrows else ''}]")
        looks = [run.arrow_looks[n] for n in (1, 2) if n in run.arrow_looks]
        stitched = camera.stitch(looks) if looks else None
        print(f"Arrow pictures tiled on the Pi: {stitched}" if stitched else "No arrow picture to tile")
        if run.uploader is not None:
            run.uploader.close()
            print(f"Arrow pictures shown on the PC: obstacles {sorted(run.uploader.sent) or 'none'}")
        print(f"Run folder: {run_dir}")
    except KeyboardInterrupt:
        print("INTERRUPTED")
    finally:
        try:
            print("STOP confirmed" if link.emergency_stop("fence STOP at exit")
                  else "STOP NOT CONFIRMED: secure the robot by hand")
        except Exception as exc:
            print(f"STOP at exit failed: {exc!r}: secure the robot by hand")
        if android is not None:
            android.stop()
        camera.close()
        port.close()
        raw_log.close()


if __name__ == "__main__":
    main()
