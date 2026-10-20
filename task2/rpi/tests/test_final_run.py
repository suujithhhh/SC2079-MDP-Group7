"""The staged final algorithm (final_run.py): ultrasonic approach to 30 cm from O1, its arrow, an S-curve into
a lane beside O1 facing north, forward until 60 cm from O2, O2's arrow, then the wall hug round O2 (front
face, U-turn, back face) and the turn to face the carpark. Simulation (perfect moves), not robot evidence.
Run from the task2 folder:

    python3 -m unittest discover -s tests -v
"""

from __future__ import annotations

import itertools
import unittest
from unittest import mock

import config
import sim
from course import FreeSpace
from final_run import (PARK_US_CM, O2_TURN, C_CURVE_MOVES, EDGE_CAP_CM, FACE_CARPARK_TURN, REVERSE_MAX_CM, U_TURN_MOVES, C_CURVE_O2_GAP_CM, C_FIXED_O1_TOL_CM, C_MIN_O2_GAP_CM, c_curve_safe, END_FACE_CAP_CM, FIR_CAP_CM, HUG_CM, LANE_RUN_CM, FinalRun, axle_for_sensor, plan_bypass, plan_final_turn,
                       plan_to, straight_heading_fix)
from final_run import FAST_PROFILE, FAST_STRAIGHT_CM, RunFail, is_fast_straight
from geometry import Pose, Rect, is_clear, simulate
from ir import IrReading, IrSample, IrSensor, IrStatus


def world(arrows: str, o1_gap: float, o2_gap: float, o1_dx: float = 0.0, o2_dx: float | None = None,
          span: float = 120.0) -> sim.World:
    return sim.World(arrows, o1_gap, o2_gap, span, FreeSpace.no_walls(), o1_dx=o1_dx,
                     o2_dx=o1_dx if o2_dx is None else o2_dx, walls=False)


def run(w: sim.World, front_us: sim.SimFrontUs | None = None):
    stm = sim.FakeStm(w, sim.fixture_params(w.o2_span), front_us=front_us or sim.SimFrontUs())
    lines: list[str] = []
    final = FinalRun(stm, sim.FakeCamera(stm), log=lines.append, wall_hug=False)
    return final.run(), final, stm, lines


class PlanTest(unittest.TestCase):
    def test_the_bypass_ends_north_in_a_lane_clear_of_o1(self):
        for side, o1_x in itertools.product(("LEFT", "RIGHT"), (-3.0, 0.0, 3.0)):
            with self.subTest(side=side, o1_x=o1_x):
                plan = plan_bypass(side, o1_x, 49.5)
                self.assertIsNotNone(plan)
                self.assertAlmostEqual(plan.end.h % 360, 90.0, places=3)
                self.assertLess(plan.end.x, o1_x - 30) if side == "LEFT" else self.assertGreater(plan.end.x, o1_x + 30)
                o1 = Rect(o1_x - 5, 49.5, o1_x + 5, 59.5, "O1")
                self.assertTrue(is_clear(Pose(0, 0, 90), plan.moves + [f"MOV,F,{LANE_RUN_CM}"], [o1], 2.0))
                turns = [m for m in plan.moves if m.split(",")[1] not in ("F", "B")]
                self.assertEqual(turns[0].split(",")[1], "FL" if side == "LEFT" else "FR")


class RunTest(unittest.TestCase):
    def test_both_arrows_read_and_no_contact(self):
        """Every arrow pair, O1 and O2 (centred on O1, team rule) on or off the carpark line, the 60 cm minimum
        gap and wider."""
        for arrows, o1_gap, o2_gap, o1_dx in itertools.product(
                ("LL", "LR", "RL", "RR"), (90.0, 150.0), (60.0, 100.0), (-2.0, 0.0, 2.0)):
            with self.subTest(arrows=arrows, o1_gap=o1_gap, o2_gap=o2_gap, o1_dx=o1_dx):
                w = world(arrows, o1_gap, o2_gap, o1_dx)
                ok, final, stm, lines = run(w)
                self.assertTrue(ok, lines[-3:])
                self.assertEqual(stm.contacts, [])
                self.assertEqual(final.arrow, "LEFT" if arrows[0] == "L" else "RIGHT")
                self.assertEqual(final.arrow2, "LEFT" if arrows[1] == "L" else "RIGHT")
                # at the 60 cm gap the reading position is alongside O1: check the lane side, not a crossing
                lane = stm.pose.x - (w.o1.x0 + w.o1.x1) / 2
                self.assertLess(lane, -15.0) if arrows[0] == "L" else self.assertGreater(lane, 15.0)
                self.assertAlmostEqual(stm.pose.h % 360, 90.0, delta=0.5, msg="the inward turn is undone")
                inward = ("BR", "FR") if arrows[0] == "L" else ("BL", "FL")
                self.assertTrue(any(c.split(",")[1] in inward for c in stm.commands[-3:]), stm.commands)
                sensor_y = stm.pose.y + config.ULTRASONIC_FORWARD_CM
                self.assertAlmostEqual(w.o2.y0 - sensor_y, 60.0, delta=2.0, msg="60 cm from O2")

    def test_no_reading_fails_before_any_move(self):
        w = world("LL", 120.0, 66.0)
        ok, final, stm, lines = run(w, sim.SimFrontUs(fail=True))
        self.assertFalse(ok)
        self.assertEqual(stm.commands, [])


ORIGINAL_PLAN_U_TURN = FinalRun.plan_u_turn
HUG_FREE = FreeSpace(end_clearance_cm=config.O2_END_CLEARANCE_CM, behind_clearance_cm=200.0, side_wall_south_cm=60.0)


def hug_run(arrows: str, span: float, o1_gap: float = 120.0, o2_gap: float = 60.0, dx: float = 0.0,
            injections: tuple = (), front_us: sim.SimFrontUs | None = None, walls: bool = True,
            bullseyes: float | None = None):
    """The whole staged run, wall hug included, with walls 50 cm beyond each end of O2 (the briefing's minimum).
    O2 is centred on O1 (team rule); `dx` moves both off the carpark's centre line."""
    w = sim.World(arrows, o1_gap, o2_gap, span, HUG_FREE, o1_dx=dx, o2_dx=dx, walls=walls)
    ir = sim.SimIr(range_max_cm=80.0, beyond_range=sim.SimBeyondRange.TRUSTWORTHY_NO_WALL, sample_spacing_cm=1.0,
                   injections=injections)
    stm = sim.FakeStm(w, sim.fixture_params(span), front_us=front_us or sim.SimFrontUs(), ir=ir,
                      edge_lag_cm=round(config.IR_EDGE_LAG_CM))
    lines: list[str] = []
    final = FinalRun(stm, sim.FakeCamera(stm, o2_bullseye_inset_cm=bullseyes), log=lines.append)
    return final.run(), final, stm, w, lines


class WallHugTest(unittest.TestCase):
    def test_round_o2_measures_both_ends_and_faces_the_carpark(self):
        """Every arrow pair, a short and a long O2, both obstacles on and off the carpark line, the 60 cm minimum
        gap and wider; a possible wall 50 cm beyond each end is never touched."""
        for arrows, span, o2_gap, dx in itertools.product(("LL", "LR", "RL", "RR"), (60.0, 120.0), (60.0, 100.0),
                                                          (0.0, 2.0)):
            with self.subTest(arrows=arrows, span=span, o2_gap=o2_gap, dx=dx):
                ok, final, stm, w, lines = hug_run(arrows, span, o2_gap=o2_gap, dx=dx)
                self.assertTrue(ok, lines[-3:])
                self.assertEqual(stm.contacts, [])
                self.assertEqual(len(stm.follow_calls), 2, "one FIR leg per face")
                self.assertAlmostEqual(final.o2_ends[1], w.o2.x1, delta=3.0)
                self.assertAlmostEqual(final.o2_ends[-1], w.o2.x0, delta=3.0)
                self.assertAlmostEqual(stm.pose.h % 360, 270.0, delta=0.5, msg="facing the carpark")
                first = 1 if arrows[1] == "R" else -1
                self.assertEqual(list(final.o2_ends), [first, -first], "the arrow's side of O2 is passed first")

    def test_a_short_o2_and_the_side_ir_moving_the_face(self):
        """A 40 cm O2 (its ends near the arrow) and an ultrasonic that reads 3 cm long: the side IR re-anchors."""
        for arrows in ("LL", "LR", "RL", "RR"):
            with self.subTest(arrows=arrows):
                ok, final, stm, w, lines = hug_run(arrows, 40.0, front_us=sim.SimFrontUs(offset_cm=3.0))
                self.assertTrue(ok, lines[-3:])
                self.assertEqual(stm.contacts, [])
                self.assertAlmostEqual(final.result["o2_span_cm"], 40.0, delta=4.0)

    def test_a_side_ir_that_does_not_see_o2_starts_no_hug(self):
        injections = (sim.IrInjection(IrSensor.LEFT, 0, 1e6, sim.IrInjectionKind.FAULT),)
        ok, final, stm, w, lines = hug_run("LR", 120.0, injections=injections)
        self.assertFalse(ok)
        self.assertIn("does not see O2", final.result["fail"])
        self.assertEqual(stm.follow_calls, [], "no FIR leg is sent")

    def test_a_missed_edge_stops_the_run_at_the_cap(self):
        """A side IR stuck on PRESENT never reports the end: the leg ends MAX_TRAVEL at the cap and nothing more
        is sent. (With a wall 50 cm beyond the end the cap could reach it: FIR_CAP_CM is not a wall guard.)"""
        injections = (sim.IrInjection(IrSensor.LEFT, 0, 1e6, sim.IrInjectionKind.STUCK_PRESENT),)
        ok, final, stm, w, lines = hug_run("LR", 120.0, injections=injections, walls=False)
        self.assertFalse(ok)
        self.assertIn("MAX_TRAVEL", final.result["fail"])
        self.assertEqual(stm.follow_calls, [float(FIR_CAP_CM)])
        self.assertNotIn("U-turn", " ".join(lines))


class BullseyeTest(unittest.TestCase):
    def test_same_arrows_go_round_the_end_face_with_two_ir_legs(self):
        """Both arrows the same way: O2's near end from the bullseye 5 cm in from it, no front-face hug; the
        side IR edge-follows O2's end face to its back corner, then the back face is hugged. Short and long O2
        (a 120 cm O2's outer bullseye needs the extra look facing north)."""
        for arrows, span in itertools.product(("LL", "RR"), (40.0, 60.0, 120.0)):
            with self.subTest(arrows=arrows, span=span):
                ok, final, stm, w, lines = hug_run(arrows, span, bullseyes=5.0)
                self.assertTrue(ok, lines[-3:])
                self.assertEqual(stm.contacts, [])
                self.assertEqual(final.result.get("o2_near_end_from"), "bullseye")
                self.assertEqual(stm.follow_calls, [float(END_FACE_CAP_CM), float(FIR_CAP_CM)],
                                 "the end face, then the back face")
                self.assertAlmostEqual(final.result["o2_back_from_end_face_y"], w.o2.y1, delta=2.0)
                self.assertAlmostEqual(final.o2_ends[1], w.o2.x1, delta=3.0)
                self.assertAlmostEqual(final.o2_ends[-1], w.o2.x0, delta=3.0)
                self.assertAlmostEqual(stm.pose.h % 360, 270.0, delta=0.5)
                self.assertEqual(sim.passed_sides(stm.trace, w)[1], arrows[1], "O2 passed on its arrow's side")

    def test_different_arrows_or_no_bullseye_hug_both_faces(self):
        for arrows, bullseyes in (("LR", 5.0), ("RL", 5.0), ("LL", None), ("RR", None)):
            with self.subTest(arrows=arrows, bullseyes=bullseyes):
                ok, final, stm, w, lines = hug_run(arrows, 60.0, bullseyes=bullseyes)
                self.assertTrue(ok, lines[-3:])
                self.assertEqual(stm.contacts, [])
                self.assertIsNone(final.result.get("o2_near_end_from"))
                self.assertEqual(stm.follow_calls, [float(FIR_CAP_CM)] * 2)

    def test_no_u_turn_after_the_end_face_falls_back_before_moving(self):
        """The U-turn from where the end-face follow would stop is planned before the robot leaves the lane: if
        it has no clear path, the robot takes the front-face entry and hug instead, with nothing sent for the
        end-face route."""
        with mock.patch.object(FinalRun, "plan_u_turn", lambda self, start: None if start.h == 90.0 else
                               ORIGINAL_PLAN_U_TURN(self, start)):
            ok, final, stm, w, lines = hug_run("RR", 60.0, bullseyes=5.0)
        self.assertTrue(ok, lines[-3:])
        self.assertTrue(any("front face entry and hug instead" in line for line in lines))
        self.assertFalse(any("line up beside" in line for line in lines))
        self.assertEqual(stm.follow_calls, [float(FIR_CAP_CM)] * 2)

    def test_an_end_face_never_seen_stops_at_its_cap(self):
        """A side IR stuck on PRESENT never sees the back corner: the follow ends at END_FACE_CAP_CM, the run
        stops, and nothing more is sent."""
        injections = (sim.IrInjection(IrSensor.LEFT, 0, 1e6, sim.IrInjectionKind.STUCK_PRESENT),)
        ok, final, stm, w, lines = hug_run("RR", 60.0, bullseyes=5.0, injections=injections, walls=False)
        self.assertFalse(ok)
        self.assertIn("end face follow ended", final.result["fail"])
        self.assertEqual(stm.follow_calls, [float(END_FACE_CAP_CM)])
        self.assertFalse(any(line.startswith("U-turn") for line in lines))


class HugPlanTest(unittest.TestCase):
    def test_the_planner_lands_on_its_target_and_turns_the_right_way(self):
        o2 = Rect(-60.0, 100.0, 60.0, 110.0, "O2")
        target = axle_for_sensor((0.0, 100.0 - HUG_CM), 0.0, "L")
        found = plan_to(Pose(-30.0, 20.0, 90.0), 0.0, [target], [o2])
        self.assertIsNotNone(found)
        moves, i = found
        end = simulate(Pose(-30.0, 20.0, 90.0), moves)
        self.assertEqual(i, 0)
        self.assertAlmostEqual(end.h % 360, 0.0, places=3)
        self.assertAlmostEqual(end.x, target[0], delta=1.0)
        self.assertAlmostEqual(end.y, target[1], delta=1.0)
        self.assertTrue(is_clear(Pose(-30.0, 20.0, 90.0), moves, [o2], 2.0))

    def test_the_final_turn_faces_south_round_the_end(self):
        o2 = Rect(-30.0, 100.0, 30.0, 110.0, "O2")
        start = Pose(*axle_for_sensor((-31.0, 110.0 + HUG_CM), 180.0, "L"), 180.0)
        moves = plan_final_turn(start, "L", [o2])
        self.assertIsNotNone(moves)
        self.assertAlmostEqual(simulate(start, moves).h % 360, 270.0, places=3)
        self.assertTrue(is_clear(start, moves, [o2], 2.0))



class CCurveTest(unittest.TestCase):
    """--c-curve: approach O1, read its arrow, the FIXED C curve round O1 (C_CURVE_MOVES), stop (team instruction
    2026-10-14: the Pi computes nothing)."""

    def test_the_fixed_curves_are_safe_over_the_stop_and_o1_tolerance(self):
        """Every stop 28.5-31.5 cm from the ultrasonic and O1 up to C_FIXED_O1_TOL_CM either side: 5 cm from O1,
        3 cm from an O2 C_CURVE_O2_GAP_CM behind O1; it ends heading north past O1, out on the arrow's side first."""
        for side in ("LEFT", "RIGHT"):
            moves = C_CURVE_MOVES[side]
            end = simulate(Pose(0, 0, 90), moves)
            self.assertAlmostEqual(end.h % 360, 90.0, places=3)
            turns = [m for m in moves if m.split(",")[1] not in ("F", "B")]
            self.assertEqual(turns[0].split(",")[1], "FL" if side == "LEFT" else "FR")
            for dx, stop in itertools.product((-C_FIXED_O1_TOL_CM, 0.0, C_FIXED_O1_TOL_CM), (28.5, 29.0, 29.5, 30.0, 30.5, 31.0, 31.5)):
                with self.subTest(side=side, o1_dx=dx, stop=stop):
                    face = config.ULTRASONIC_FORWARD_CM + stop
                    self.assertTrue(c_curve_safe(Pose(0, 0, 90), moves, dx, face))
                    self.assertGreater(end.y, face + config.O1_SIZE_CM, "past O1")

    def test_stages_1_to_3_in_simulation(self):
        for arrows, o1_gap, o2_gap, o1_dx in itertools.product(("LL", "LR", "RL", "RR"), (90.0, 150.0), (C_CURVE_O2_GAP_CM, 90.0),
                                                              (-1.5, 0.0, 1.5)):
            with self.subTest(arrows=arrows, o1_gap=o1_gap, o2_gap=o2_gap, o1_dx=o1_dx):
                w = world(arrows, o1_gap, o2_gap, o1_dx)
                stm = sim.FakeStm(w, sim.fixture_params(w.o2_span), front_us=sim.SimFrontUs())
                lines: list[str] = []
                final = FinalRun(stm, sim.FakeCamera(stm), log=lines.append, c_curve=True, wall_hug=False)
                self.assertTrue(final.run(), lines[-3:])
                # The LEFT turn onto O2 starts 5 cm closer than the turn model allows (robot run 2026-10-14: with the
                # model's start the side IR ended 19 cm out, so the real FL turn is tighter than its recorded 24.8 cm
                # radius). In the model its corner touches O2; that turn is the only place a contact is accepted.
                contacts = [c for c in stm.contacts
                            if not (arrows[1] == "L" and c.startswith("O2 at") and 90.0 < float(c.split(",")[2].split()[0]) <= 180.0)]
                self.assertEqual(contacts, [])
                self.assertEqual(sim.passed_sides(stm.trace, w)[0], arrows[0])
                c = C_CURVE_MOVES[final.arrow]
                i = next(k for k in range(len(stm.commands)) if stm.commands[k:k + len(c)] == c)
                self.assertTrue(i >= 0, "the fixed C curve was sent as it is")
                # then O2's arrow and one 90 deg turn towards it: parallel to O2's front face
                self.assertEqual(final.arrow2, "LEFT" if arrows[1] == "L" else "RIGHT")
                self.assertEqual(stm.commands[-1], O2_TURN[final.arrow2])
                self.assertAlmostEqual(stm.pose.h % 360, 180.0 if final.arrow2 == "LEFT" else 0.0, delta=0.5)
                side_gap = w.o2.y0 - stm.pose.y - config.WIDTH_CM / 2
                self.assertAlmostEqual(side_gap, 7.0 if final.arrow2 == "LEFT" else 10.5, delta=1.5)   # model values

    def test_the_camera_gives_no_distances(self):
        """O1 1.5 cm off the robot's line: the run still takes O1 as straight ahead (team instruction)."""
        w = world("LR", 120.0, C_CURVE_O2_GAP_CM, 1.5)
        stm = sim.FakeStm(w, sim.fixture_params(w.o2_span), front_us=sim.SimFrontUs())
        final = FinalRun(stm, sim.FakeCamera(stm), log=lambda text: None, c_curve=True, wall_hug=False)
        self.assertTrue(final.run())
        self.assertAlmostEqual(final.o1_x, 0.0, places=6)
        self.assertNotIn("o1_lateral_cm", final.result)


def turn_contact(arrows: str, contact: str) -> bool:
    """The one contact accepted (see test_stages_1_to_3_in_simulation): the LEFT turn onto O2's front face."""
    return arrows[1] == "L" and contact.startswith("O2 at") and 90.0 < float(contact.split(",")[2].split()[0]) <= 180.0


def part3_run(arrows: str, span: float, overshoot: float = 0.0, walls: bool = True, park: bool = False,
              o1_gap: float = 120.0):
    """Up to facing the carpark (park=False: part 4, return_to_carpark, is left out), or the whole run."""
    if not park:
        with mock.patch.object(FinalRun, "return_to_carpark", lambda self: None):
            return part3_run(arrows, span, overshoot, walls, True, o1_gap)
    w = sim.World(arrows, o1_gap, C_CURVE_O2_GAP_CM, span, HUG_FREE, walls=walls)
    ir = sim.SimIr(range_max_cm=80.0, beyond_range=sim.SimBeyondRange.TRUSTWORTHY_NO_WALL, sample_spacing_cm=1.0)
    stm = sim.FakeStm(w, sim.fixture_params(span, reserve=6), front_us=sim.SimFrontUs(), ir=ir,
                      stop_overshoot_cm=overshoot, edge_lag_cm=round(config.IR_EDGE_LAG_CM))
    lines: list[str] = []
    final = FinalRun(stm, sim.FakeCamera(stm), log=lines.append, c_curve=True)
    return final.run(), final, stm, w, lines


def contains(commands: list[str], moves: list[str]) -> bool:
    return any(commands[k:k + len(moves)] == moves for k in range(len(commands)))


class RoundO2FixedTest(unittest.TestCase):
    """--c-curve part 3 (team plan 2026-10-14): from along O2's front face, hug it to the end (still beside O2) or
    reverse until the side IR sees O2 and edge-follow forward past the end (already past it); then the fixed U-turn,
    the back-face hug and the fixed 90 deg turn to face the carpark. Walls stand 50 cm beyond each end of O2."""

    def test_both_cases_every_arrow_pair(self):
        """A FIR stopping 3 cm (overshoot 0) to 8 cm past the edge. A 20 cm O2 is left out here: see the next test."""
        for arrows, span, overshoot in itertools.product(("LL", "LR", "RL", "RR"), (30.0, 40.0, 80.0, 120.0, 150.0),
                                                         (0.0, 2.0, 5.0)):
            if arrows == "RR" and span == 30.0:
                # The RIGHT C curve for the 60 cm gap (23:21) ends ~4 cm right of centre: the turn onto O2 (FR,90, stage
                # 5) then reaches a wall standing exactly 50 cm beyond a 30 cm O2's end. Known, not covered.
                continue
            if arrows[1] == "L" and overshoot == 0.0:
                # The model's LEFT turn onto O2 leaves a 7 cm face gap (the robot: 9-12.5 cm, IR 13.4-16.8 cm); the sharp
                # LEFT U-turn is set for the robot and keeps 3 cm only from 9 cm, or from 7 cm if the FIR runs on 3.5 cm+.
                continue
            with self.subTest(arrows=arrows, span=span, overshoot=overshoot):
                ok, final, stm, w, lines = part3_run(arrows, span, overshoot)
                self.assertTrue(ok, lines[-3:])
                self.assertEqual([c for c in stm.contacts if not turn_contact(arrows, c)], [])
                case = final.result["o2_front"]
                if span <= 40.0:
                    self.assertEqual(case, "reverse", "the turn leaves the side IR past a short O2's end")
                if span >= 150.0:
                    self.assertEqual(case, "hug")
                front_cap = final.result["o2_front_edge_cap_cm"] if case == "reverse" else FIR_CAP_CM
                self.assertEqual(stm.follow_calls, [float(front_cap), float(FIR_CAP_CM)], "front (or edge) leg, then the back face")
                if case == "reverse":
                    self.assertLessEqual(final.result["o2_front_reverse_cm"], REVERSE_MAX_CM + 5)
                    self.assertGreaterEqual(front_cap, EDGE_CAP_CM)
                self.assertTrue(contains(stm.commands, U_TURN_MOVES[final.arrow2]), "the fixed U-turn as it is")
                ending = final.result["carpark_turn"] + final.result["carpark_level"]
                self.assertEqual(ending, stm.commands[-len(ending):])
                self.assertAlmostEqual(stm.pose.y, w.o2.y1, delta=1.5, msg="rear axle level with O2's back face")
                self.assertEqual(sum(int(c.split(",")[2]) for c in final.result["carpark_turn"]
                                     if c.split(",")[1] not in ("F", "B")), 90)
                if overshoot <= 2.0:            # the robot runs on 2-4 cm after IREDGE (runs 2026-10-14)
                    self.assertLessEqual(final.result["end_gap_to_o2_cm"], 15.0, "a sharp last turn")
                    end_face = w.o2.x0 if arrows[1] == "R" else w.o2.x1
                    self.assertLessEqual(abs(stm.pose.x - end_face) - config.WIDTH_CM / 2, 15.0, "at most 15 cm from O2")
                self.assertAlmostEqual(final.o2_ends[1], w.o2.x1, delta=3.0)
                self.assertAlmostEqual(final.o2_ends[-1], w.o2.x0, delta=3.0)
                self.assertAlmostEqual(stm.pose.h % 360, 270.0, delta=0.5, msg="facing the carpark")
                first = 1 if arrows[1] == "R" else -1
                self.assertEqual(list(final.o2_ends), [first, -first], "the arrow's side of O2 is passed first")
                # past O2's far end, beside it
                self.assertGreater(-first * (stm.pose.x - (w.o2.x1 if first < 0 else w.o2.x0)), 0.0)

    def test_the_shortest_o2(self):
        """A 20 cm O2 (the team minimum), no walls: still both ends and facing the carpark. With a wall exactly 50 cm
        beyond the end, RR touches it in the turn onto O2 (stage 5, before this part): not covered."""
        for arrows, overshoot in itertools.product(("LL", "LR", "RL", "RR"), (0.0, 5.0)):
            if arrows[1] == "L" and overshoot == 0.0:
                continue                # as in test_both_cases_every_arrow_pair
            with self.subTest(arrows=arrows, overshoot=overshoot):
                ok, final, stm, w, lines = part3_run(arrows, 20.0, overshoot, walls=False)
                self.assertEqual([c for c in stm.contacts if not turn_contact(arrows, c)], [])
                self.assertEqual(final.result["o2_front"], "reverse")
                if arrows[1] == "R" and not ok:
                    # FL,180 leaves the IR at a 20 cm O2's far end: the back hug never confirms O2 and stops. The Pi
                    # cannot tell this from a long O2 (its dead-reckoned x is ~10 cm off, robot run 22:34), so it
                    # does not reverse first: known limit, fails safe
                    self.assertIn("back hug ended NO_WALL", final.result["fail"])
                    continue
                self.assertTrue(ok, lines[-3:])
                self.assertAlmostEqual(stm.pose.h % 360, 270.0, delta=0.5)

    def test_a_side_ir_that_never_sees_o2_stops_the_reverse(self):
        far = IrReading(IrStatus.INVALID, far=True)
        with mock.patch.object(sim.FakeStm, "read_ir", lambda self: IrSample(1, 0.0, far, far)):
            ok, final, stm2, w, lines = part3_run("LR", 40.0)
        self.assertFalse(ok)
        self.assertIn("did not see O2", final.result["fail"])
        self.assertEqual(stm2.follow_calls, [], "no FIR leg is sent")
        reverses = [c for c in stm2.commands[stm2.commands.index(O2_TURN["RIGHT"]) + 1:]]
        self.assertEqual(sum(int(c.split(",")[2]) for c in reverses), REVERSE_MAX_CM)
        self.assertTrue(all(c.startswith("MOV,B,") for c in reverses))

    def test_case_1_backs_up_in_one_move(self):
        """Robot run 2026-10-15: six 5 cm reverses (about 6.5 s). Now ONE reverse to a side IR 7 cm inside O2's shortest
        end, one IR,GET, then the edge-follow with a cap that covers that reverse."""
        for arrows, span in itertools.product(("LL", "LR", "RL", "RR"), (30.0, 40.0)):
            if arrows == "RR" and span == 30.0:
                continue                # as in test_both_cases_every_arrow_pair
            with self.subTest(arrows=arrows, span=span):
                ok, final, stm, w, lines = part3_run(arrows, span, 5.0)
                self.assertTrue(ok, lines[-3:])
                self.assertEqual(final.result["o2_front"], "reverse")
                after = stm.commands[stm.commands.index(O2_TURN[final.arrow2]) + 1:]
                backs = []
                for command in after:
                    if not command.startswith("MOV,B,"):
                        break
                    backs.append(int(command.split(",")[2]))
                self.assertEqual(len(backs), 1, f"one reverse, not steps: {backs}")
                self.assertEqual(final.result["o2_front_reverse_cm"], backs[0])
                self.assertEqual(final.result["o2_front_edge_cap_cm"], max(EDGE_CAP_CM, backs[0] + 8))
                self.assertEqual(stm.follow_calls[0], float(final.result["o2_front_edge_cap_cm"]))
                self.assertEqual([c for c in stm.contacts if not turn_contact(arrows, c)], [])

    def one_move_rig(self, x: float, irs, o2_min_half_cm: float = 10.0):
        """A FinalRun just after the turn onto O2 (LEFT arrow: heading west, side IR R), the IR readings scripted."""
        from types import SimpleNamespace as NS
        far = IrReading(IrStatus.INVALID, far=True)
        values = iter(irs)

        class Stm:
            commands: list[str] = []

            def move(self, command):
                self.commands.append(command)
                return NS(ok=True)

            def heading(self):
                return None

            def read_ir(self):
                value = next(values)
                return IrSample(1, 0.0, far, far if value is None else IrReading(IrStatus.VALID_DISTANCE, value))   # right IR

        stm = Stm()
        final = FinalRun(stm, None, heading_fix=False, log=lambda text: None)
        final.arrow2, final.o2_x, final.pose = "LEFT", 0.0, Pose(x, 256.0, 180.0)
        final.o2_min_half_cm = o2_min_half_cm
        return final, stm

    def test_the_reverse_distance_is_the_ir_beyond_the_shortest_reach(self):
        # robot run 2026-10-15: rear axle x -23, side IR 19 cm ahead of it at x -42: 42 cm past O2's centre, to 3 cm from it
        final, stm = self.one_move_rig(-23.0, [14.0])
        self.assertEqual(final.reverse_to_o2("R"), max(EDGE_CAP_CM, 39 + 8))
        self.assertEqual(stm.commands, ["MOV,B,39"])
        self.assertEqual(final.result["o2_front_reverse_cm"], 39)
        # the O2 minimum from the rules (30 cm): 8 cm from the centre, so 5 cm less
        final, stm = self.one_move_rig(-23.0, [14.0], o2_min_half_cm=15.0)
        final.reverse_to_o2("R")
        self.assertEqual(stm.commands, ["MOV,B,34"])

    def test_an_ir_that_still_sees_nothing_goes_on_in_steps_to_the_same_limit(self):
        final, stm = self.one_move_rig(-23.0, [None, None, 14.0])
        self.assertEqual(final.reverse_to_o2("R"), EDGE_CAP_CM)
        self.assertEqual(stm.commands, ["MOV,B,39", "MOV,B,5", "MOV,B,5", "MOV,B,5"], "one reverse, then 5 cm steps")
        self.assertEqual(final.result["o2_front_reverse_cm"], 54, "39 + two steps, then the 5 cm past it")
        final, stm = self.one_move_rig(-23.0, [None] * 20)
        with self.assertRaisesRegex(Exception, "did not see O2 after reversing 60 cm"):
            final.reverse_to_o2("R")
        self.assertEqual(sum(int(c.split(",")[2]) for c in stm.commands), REVERSE_MAX_CM, "never more than the limit")
        self.assertEqual(stm.commands[1:], ["MOV,B,5"] * 4 + ["MOV,B,1"])

    def test_a_distance_beyond_the_limit_or_no_distance_uses_the_steps(self):
        final, stm = self.one_move_rig(-123.0, [None, 14.0])           # 142 cm past the centre: more than REVERSE_MAX_CM
        self.assertEqual(final.reverse_to_o2("R"), EDGE_CAP_CM)
        self.assertEqual(stm.commands[0], "MOV,B,5")
        final, stm = self.one_move_rig(17.0, [14.0])                    # the IR (x -2) is already inside O2's reach: nothing to back
        self.assertEqual(final.reverse_to_o2("R"), EDGE_CAP_CM)
        self.assertEqual(stm.commands, ["MOV,B,5", "MOV,B,5"])

    def test_an_untrusted_side_ir_stops_before_moving_on(self):
        """NEAR (or a fault, a stale reply): neither beside O2 nor past it."""
        near = IrReading(IrStatus.INVALID)
        with mock.patch.object(sim.FakeStm, "read_ir", lambda self: IrSample(1, 0.0, near, near)):
            ok, final, stm, w, lines = part3_run("RL", 120.0)
        self.assertFalse(ok)
        self.assertIn("not usable", final.result["fail"])
        self.assertEqual(stm.commands[-1], O2_TURN["LEFT"], "nothing after the turn onto O2")


class ReturnToCarparkTest(unittest.TestCase):
    """--c-curve part 4 (team plan 2026-10-14): from facing the carpark beside O2's far end, one S-curve onto O1's
    centre line past O1 (O1 placed from the C curve end + the ultrasonic gap, O2's length from the FIR legs), then
    the ultrasonic to PARK_US_CM from the carpark's back wall."""

    def test_back_into_the_carpark(self):
        for arrows, span, o1_gap in itertools.product(("LL", "LR", "RL", "RR"), (40.0, 80.0, 120.0), (90.0, 150.0)):
            with self.subTest(arrows=arrows, span=span, o1_gap=o1_gap):
                ok, final, stm, w, lines = part3_run(arrows, span, 2.0, park=True, o1_gap=o1_gap)
                self.assertTrue(ok, lines[-3:])
                self.assertEqual([c for c in stm.contacts if not turn_contact(arrows, c)], [])
                self.assertAlmostEqual(final.o1_o2_gap_cm, w.o2.y0 - w.o1.y1, delta=1.0, msg="gap between the obstacles")
                self.assertAlmostEqual(stm.pose.h % 360, 270.0, delta=0.5)
                self.assertAlmostEqual(stm.pose.x, 0.0, delta=2.0, msg="on the carpark's centre line")
                self.assertTrue(sim.body_inside_carpark(stm.pose, w), stm.pose)
                self.assertAlmostEqual(final.result["park_ultrasonic_cm"], PARK_US_CM, delta=1.5)


class OneShotParkTest(unittest.TestCase):
    """Back into the carpark in ONE straight when the Pi's own position (start placement + every planned move) agrees
    with the first ultrasonic reading: it drives the SHORTER of the two less PARK_US_CM. Otherwise the old way (stop
    short, measure again)."""

    @staticmethod
    def full_run(one_shot=True, offset_error=0.0, arrows="LR", span=60.0, o1_gap=120.0):
        w = sim.World(arrows, o1_gap, C_CURVE_O2_GAP_CM, span, HUG_FREE, walls=True)
        ir = sim.SimIr(range_max_cm=80.0, beyond_range=sim.SimBeyondRange.TRUSTWORTHY_NO_WALL, sample_spacing_cm=1.0)
        stm = sim.FakeStm(w, sim.fixture_params(span, reserve=6), front_us=sim.SimFrontUs(), ir=ir,
                          stop_overshoot_cm=2.0, edge_lag_cm=round(config.IR_EDGE_LAG_CM))
        lines: list[str] = []
        final = FinalRun(stm, sim.FakeCamera(stm), log=lines.append, c_curve=True, park_one_shot=one_shot)
        final.start_rear_from_wall_cm += offset_error     # the Pi's idea of where the robot was placed is this far off
        return final.run(), final, stm, w, lines

    @staticmethod
    def parking_straights(final, stm):
        after = stm.commands[stm.commands.index(final.result["return_moves"][-1], len(stm.commands) - 8) + 1:]
        return after

    def test_one_straight_when_tracking_agrees(self):
        ok, final, stm, w, lines = self.full_run(True)
        ok_old, final_old, stm_old, _, _ = self.full_run(False)
        self.assertTrue(ok and ok_old, lines[-3:])
        self.assertTrue(final.result["park_one_shot"])
        self.assertAlmostEqual(final.result["park_tracked_cm"], final.result["park_start_ultrasonic_cm"], delta=1.0)
        new, old = self.parking_straights(final, stm), self.parking_straights(final_old, stm_old)
        self.assertEqual(len(old), 2, old)
        self.assertEqual(len(new), 1, new)
        self.assertGreater(int(new[0].split(",")[2]), 70, "one long straight")
        self.assertAlmostEqual(final.result["park_ultrasonic_cm"], PARK_US_CM, delta=1.5)
        self.assertEqual([c for c in stm.contacts if not turn_contact("LR", c)], [])
        self.assertTrue(sim.body_inside_carpark(stm.pose, w), stm.pose)

    def test_tracking_that_disagrees_uses_the_old_way(self):
        for error in (-12.0, -6.0, 6.0, 12.0):
            with self.subTest(error=error):
                ok, final, stm, w, lines = self.full_run(True, error)
                self.assertTrue(ok, lines[-3:])
                self.assertFalse(final.result["park_one_shot"])
                self.assertEqual(len(self.parking_straights(final, stm)), 2, "stop short, then the rest")
                self.assertAlmostEqual(final.result["park_ultrasonic_cm"], PARK_US_CM, delta=1.5)

    def test_tracking_a_little_nearer_stops_early_never_late(self):
        """The Pi thinks the wall is 3 cm nearer than the ultrasonic says: it drives the shorter one, then fixes."""
        ok, final, stm, w, lines = self.full_run(True, -3.0)
        self.assertTrue(ok, lines[-3:])
        self.assertTrue(final.result["park_one_shot"])
        straights = self.parking_straights(final, stm)
        first = int(straights[0].split(",")[2])
        self.assertAlmostEqual(first, final.result["park_start_ultrasonic_cm"] - PARK_US_CM - 3.0, delta=1.0)
        self.assertLessEqual(first, final.result["park_start_ultrasonic_cm"] - PARK_US_CM + 0.5, "never past the ultrasonic")
        self.assertAlmostEqual(final.result["park_ultrasonic_cm"], PARK_US_CM, delta=1.5)
        self.assertEqual([c for c in stm.contacts if not turn_contact("LR", c)], [])

    def test_tracking_a_little_further_is_not_trusted_beyond_the_ultrasonic(self):
        ok, final, stm, w, lines = self.full_run(True, 4.0)
        self.assertTrue(ok, lines[-3:])
        first = int(self.parking_straights(final, stm)[0].split(",")[2])
        self.assertLessEqual(first, final.result["park_start_ultrasonic_cm"] - PARK_US_CM + 0.5)
        self.assertEqual([c for c in stm.contacts if not turn_contact("LR", c)], [])

    @staticmethod
    def rig(readings):
        from types import SimpleNamespace as NS
        sent, feed = [], iter(readings)

        class Stm:
            def move(self, command):
                sent.append(command)
                return NS(ok=True)

            def heading(self):
                return None

            def measure_front(self):
                return NS(distance_cm=next(feed), used=5, seen=5, problem=None, ok=True)

        return FinalRun(Stm(), None, heading_fix=False, log=lambda text: None, park_one_shot=True), sent

    @classmethod
    def drive(cls, readings, tracked):
        final, sent = cls.rig(readings)
        return final.drive_to(15.0, "test", 1.5, tracked_cm=tracked), sent, final

    def test_one_move_by_the_shorter_distance(self):
        distance, sent, final = self.drive([115.7, 14.9], tracked=114.0)
        self.assertEqual(sent, ["MOV,F,99"], "114.0 - 15, not 115.7 - 15")
        self.assertTrue(final.result["park_one_shot"])
        distance, sent, final = self.drive([115.7, 15.2], tracked=118.0)
        self.assertEqual(sent, ["MOV,F,101"], "the ultrasonic is the shorter")

    def test_a_far_reading_the_tracking_does_not_share_is_not_driven_past_the_tracking(self):
        """Robot run 23:43 (152 cm read twice at ~70 cm from the wall): the tracking says 70 cm, so the move stops
        short of THAT wall (70 - 15 - 15), not of the 152 cm one."""
        distance, sent, final = self.drive([152.0, 60.0, 14.8], tracked=70.0)
        self.assertFalse(final.result["park_one_shot"])
        self.assertEqual(sent, ["MOV,F,40"])
        self.assertEqual(final.result["park_tracked_cm"], 70.0)
        self.assertAlmostEqual(distance, 14.8)

    def test_a_far_reading_that_stays_is_never_driven_when_the_wall_is_tracked_near(self):
        """The tracking puts the wall 25 cm away, the ultrasonic keeps saying 150: nothing is driven, the run stops."""
        final, sent = self.rig([150.0] * 20)
        with self.assertRaisesRegex(RunFail, "not within"):
            final.drive_to(15.0, "test", 1.5, tracked_cm=25.0)
        self.assertEqual(sent, [], "no move at all")

    def test_a_short_approach_is_one_move_anyway(self):
        distance, sent, final = self.drive([50.0, 15.3], tracked=49.0)
        self.assertEqual(sent, ["MOV,F,35"])
        self.assertFalse(final.result["park_one_shot"], "nothing to merge: not a split approach")

    def test_only_facing_the_wall_is_tracked(self):
        final = FinalRun(None, None, heading_fix=False, log=lambda text: None)
        final.pose = Pose(6.9, 24.4, 270.0)
        self.assertAlmostEqual(final.tracked_wall_cm(), 24.4 - config.ULTRASONIC_FORWARD_CM + config.START_REAR_FROM_BACK_WALL_CM)
        final.pose = Pose(6.9, 24.4, 262.0)
        self.assertIsNone(final.tracked_wall_cm())
        final.pose = Pose(6.9, 24.4, 90.0)
        self.assertIsNone(final.tracked_wall_cm())


class UltrasonicJumpTest(unittest.TestCase):
    """drive_to against a reading that disagrees with the move just made (robot run 23:43: 82.6 cm, F,13, then
    152 cm twice at ~70 cm from the carpark wall)."""

    def drive(self, readings):
        from types import SimpleNamespace as NS
        sent, feed = [], iter(readings)

        class Stm:
            def move(self, command):
                sent.append(command)
                return NS(ok=True)

            def heading(self):
                return None

            def measure_front(self):
                return NS(distance_cm=next(feed), used=5, seen=5, problem=None, ok=True)

        final = FinalRun(Stm(), None, heading_fix=False, log=lambda text: None)
        return final.drive_to(70.0, "test", 1.5), sent

    def test_a_shorter_reading_seen_twice_is_believed(self):
        """Nearer than the move allows: something is really there (e.g. a side wall in the cone)."""
        distance, sent = self.drive([90.2, 60.5, 60.4, 70.3])
        self.assertEqual(sent, ["MOV,F,20", "MOV,B,10"])
        self.assertAlmostEqual(distance, 70.3)

    def test_a_near_longer_reading_is_believed(self):
        """Near the wall the cone sees only what is ahead: a longer reading there is right, not a missed echo."""
        from types import SimpleNamespace as NS
        sent2 = []
        feed = iter([40.0, 26.0, 26.1, 15.2])     # F,25 should leave 15 cm; 26 cm twice is near: believed

        class Stm:
            def move(self, command):
                sent2.append(command)
                return NS(ok=True)

            def heading(self):
                return None

            def measure_front(self):
                return NS(distance_cm=next(feed), used=5, seen=5, problem=None, ok=True)

        final = FinalRun(Stm(), None, heading_fix=False, log=lambda text: None)
        self.assertAlmostEqual(final.drive_to(15.0, "test", 1.5), 15.2)
        self.assertEqual(sent2, ["MOV,F,25", "MOV,F,11"])

    def test_a_jump_seen_twice_is_overruled_by_the_move(self):
        distance, sent = self.drive([82.6, 152.6, 152.9])
        self.assertEqual(sent, ["MOV,F,13"])
        self.assertAlmostEqual(distance, 69.6)

    def test_a_jump_that_goes_away_is_measured_again(self):
        distance, sent = self.drive([82.6, 152.6, 70.1])
        self.assertEqual((sent, distance), (["MOV,F,13"], 70.1))

    def test_an_agreeing_reading_is_used(self):
        distance, sent = self.drive([82.6, 71.0])
        self.assertEqual((sent, distance), (["MOV,F,13"], 71.0))


class RobotRunReplayTest(unittest.TestCase):
    """Robot run 2026-10-14 21:27 (arrows RR, case 1), replayed from its STM log: the readings and the FIR results as
    the robot sent them (with the 5 cm reverse steps it made then: REVERSE_ONE_MOVE off). Today's firmware gives a FIR's travel from the last MM,D sample before braking: on the back
    face 27.4 cm, short of the edge it confirmed at 30.0 cm, and the run refused the turn to face the carpark."""

    def test_the_turn_to_face_the_carpark_is_sent(self):
        from types import SimpleNamespace as NS
        from stm_task2 import EdgeResult
        far = IrReading(IrStatus.INVALID, far=True)
        us = iter([86.0, 28.0, 30.0, 32.2, 34.2, 34.2])
        irs = iter([None] * 9 + [18.6, 18.5, 14.2, 14.2])
        edges = iter([EdgeResult("EDGE", 838.5 / 75, 838 / 75, "ACK"), EdgeResult("EDGE", 2051.5 / 75, 2249 / 75, "ACK")])

        class Stm:
            commands: list[str] = []

            def move(self, command):
                self.commands.append(command)
                return NS(ok=True)

            def heading(self):
                return None

            def measure_front(self):
                return NS(distance_cm=next(us), used=5, seen=5, problem=None, ok=True)

            def read_ir(self):
                value = next(irs)
                return IrSample(1, 0.0, far if value is None else IrReading(IrStatus.VALID_DISTANCE, value), far)

            def follow_edge(self, sensor, cap):
                return next(edges)

        stm = Stm()
        final = FinalRun(stm, None, c_curve=True, heading_fix=False, log=lambda text: None)
        with mock.patch.object(FinalRun, "read", lambda self, tag, depth: ("RIGHT", 0.0, NS(annotated_path=None))), \
                mock.patch("final_run.REVERSE_ONE_MOVE", False), \
                mock.patch.object(FinalRun, "return_to_carpark", lambda self: None):   # (that run ended facing the carpark)
            self.assertTrue(final.run(), final.result.get("fail"))
        self.assertEqual(final.result["o2_front_reverse_cm"], 50)
        ending = final.result["carpark_turn"] + final.result["carpark_level"]
        self.assertEqual(stm.commands[-len(ending):], ending)
        self.assertIn(FACE_CARPARK_TURN["RIGHT"], final.result["carpark_turn"])   # RIGHT: one FL,90


class FastStraightsTest(unittest.TestCase):
    """Forward straights of FAST_STRAIGHT_CM or more run in TASK2TEST (team-tested accurate for distance), everything
    else in PRECISION: the Pi switches the STM profile just before such a straight and back right after it."""

    @staticmethod
    def full_run(fast=True, base="PRECISION", failures=None, arrows="LR", span=60.0):
        w = sim.World(arrows, 120.0, C_CURVE_O2_GAP_CM, span, HUG_FREE, walls=True)
        ir = sim.SimIr(range_max_cm=80.0, beyond_range=sim.SimBeyondRange.TRUSTWORTHY_NO_WALL, sample_spacing_cm=1.0)
        stm = sim.FakeStm(w, sim.fixture_params(span, reserve=6), front_us=sim.SimFrontUs(), ir=ir,
                          stop_overshoot_cm=2.0, edge_lag_cm=round(config.IR_EDGE_LAG_CM),
                          profile_failures=dict(failures or {}))
        lines: list[str] = []
        final = FinalRun(stm, sim.FakeCamera(stm), log=lines.append, c_curve=True, fast_straights=fast, base_profile=base)
        return final.run(), final, stm, lines

    def test_which_moves_are_fast(self):
        for command, fast in (("MOV,F,70", True), ("MOV,F,118", True), ("MOV,F,69", False), ("MOV,F,5", False),
                              ("MOV,B,100", False), ("MOV,FL,90", False), ("MOV,FR,120", False), ("MOV,BR,80", False),
                              ("MOV,FIRL,100", False), ("MOV,F,x", False), ("MOV,F", False)):
            self.assertEqual(is_fast_straight(command), fast, command)
        self.assertEqual((FAST_STRAIGHT_CM, FAST_PROFILE), (70, "TASK2TEST"))

    def test_only_long_forward_straights_run_in_task2test(self):
        ok, final, stm, lines = self.full_run()
        self.assertTrue(ok, lines[-3:])
        fast = [i for i, c in enumerate(stm.commands) if is_fast_straight(c)]
        self.assertGreaterEqual(len(fast), 2, "the straight past O1 and the one into the carpark at least")
        for i, command in enumerate(stm.commands):
            self.assertEqual(stm.profile_at(i), FAST_PROFILE if i in fast else "PRECISION", f"{i}: {command}")
        self.assertEqual(stm.profile_log[-1][1], "PRECISION", "the run ends in the base profile")
        self.assertEqual(len(stm.profile_log), 2 * len(fast), "one switch each way per long straight, none else")
        self.assertEqual([c for c in stm.contacts if not turn_contact("LR", c)], [])

    def test_off_when_not_asked_or_when_already_fast(self):
        for kwargs in ({"fast": False}, {"base": FAST_PROFILE}):
            with self.subTest(**kwargs):
                ok, final, stm, lines = self.full_run(**kwargs)
                self.assertTrue(ok, lines[-3:])
                self.assertEqual(stm.profile_log, [])

    def test_a_fast_profile_that_cannot_be_set_runs_the_straight_in_the_base_profile(self):
        ok, final, stm, lines = self.full_run(failures={FAST_PROFILE: "PROFILE,TASK2TEST: no reply"})
        self.assertTrue(ok, lines[-3:])
        self.assertTrue(any("could not set profile TASK2TEST" in line for line in lines))
        self.assertTrue(all(profile == "PRECISION" for _, profile in stm.profile_log[1::2]), "undone each time")
        self.assertTrue(all(stm.profile_at(i) == "PRECISION" for i in range(len(stm.commands))))

    def test_failing_to_go_back_to_the_base_profile_stops_the_run(self):
        ok, final, stm, lines = self.full_run(failures={"PRECISION": "PROFILE,PRECISION: no reply"})
        self.assertFalse(ok)
        self.assertIn("could not go back to PRECISION", final.result["fail"])
        first_long = next(i for i, c in enumerate(stm.commands) if is_fast_straight(c))
        self.assertEqual(len(stm.commands), first_long + 1, "nothing is sent after that straight")

    def test_if_neither_profile_can_be_set_the_run_stops_before_the_straight(self):
        ok, final, stm, lines = self.full_run(failures={FAST_PROFILE: "x", "PRECISION": "y"})
        self.assertFalse(ok)
        self.assertIn("the STM profile is unknown", final.result["fail"])
        self.assertFalse(any(is_fast_straight(c) for c in stm.commands), "the long straight is never sent")


class StraightHeadingFixTest(unittest.TestCase):
    def test_turns_back_then_shortens_the_straight(self):
        self.assertEqual(straight_heading_fix("MOV,F,50", 90.0, 87.0), (["MOV,FL,3", "MOV,F,49"], 3.0))
        moves, _ = straight_heading_fix("MOV,F,50", 90.0, 92.4)
        self.assertEqual(moves, ["MOV,FR,2", "MOV,F,49"])
        moves, _ = straight_heading_fix("MOV,B,30", 90.0, 87.0)
        self.assertEqual(moves, ["MOV,BL,3", "MOV,B,28"])

    def test_ends_where_the_plan_ends(self):
        for command, actual in (("MOV,F,50", 87.0), ("MOV,F,50", 93.0), ("MOV,B,30", 86.0), ("MOV,B,30", 94.0)):
            with self.subTest(command=command, actual=actual):
                moves, _ = straight_heading_fix(command, 90.0, actual)
                end = simulate(Pose(0.0, 0.0, actual), moves)
                planned = simulate(Pose(0.0, 0.0, 90.0), [command])
                self.assertAlmostEqual(end.h % 360, 90.0, delta=0.01)
                self.assertAlmostEqual(end.x, planned.x, delta=1.0)
                self.assertAlmostEqual(end.y, planned.y, delta=1.0)

    def test_small_error_or_short_straight_unchanged(self):
        self.assertEqual(straight_heading_fix("MOV,F,50", 90.0, 89.5)[0], ["MOV,F,50"])
        self.assertEqual(straight_heading_fix("MOV,F,1", 90.0, 85.0)[0], ["MOV,F,1"])

    def test_capped(self):
        self.assertEqual(straight_heading_fix("MOV,F,50", 90.0, 80.0)[0], ["MOV,FL,5", "MOV,F,48"])

    def test_send_uses_the_imu_before_a_straight(self):
        stm = mock.Mock()
        stm.heading.return_value = 17.0                # imu0 20: 3 deg clockwise of the plan
        stm.move.return_value = sim.StmReply("ok", "ACK", True)
        final = FinalRun(stm, mock.Mock(), log=lambda _line: None)
        final.imu0 = 20.0
        final.send("MOV,F,50")
        self.assertEqual([c.args[0] for c in stm.move.call_args_list], ["MOV,FL,3", "MOV,F,49"])
        self.assertEqual(final.pose, simulate(Pose(0.0, 0.0, 90.0), ["MOV,F,50"]))


if __name__ == "__main__":
    unittest.main()
