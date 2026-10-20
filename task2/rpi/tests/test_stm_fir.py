"""Phase 3b.5a tests: the `MOV,FIRL` / `MOV,FIRR` wrapper (PROTOCOL.md §4.6, §4.7, §6.2.5, D-56).
Run from the task2 folder:

    python3 -m unittest discover -s tests -v

Host only: a deterministic fake port (sim.py) replays the frozen spellings. They prove the Pi's handling of
the completion record, not the robot. FIR has NO hardware authority (D-57 / PD-02).
"""

from __future__ import annotations

import unittest

import sim
import stm_fir
from ir import EdgeOutcome
from stm_link import StmLink
from stm_protocol import LinkState, MotionOutcomeUnknownError
from stm_task2 import Task2Stm

CPC = 75.0   # config.COUNTS_PER_CM


def rig():
    clock = sim.FakeClock()
    port = sim.FakeSerialPort(clock)
    notes: list[str] = []
    link = StmLink(port, log=lambda _: None, clock=clock, sleep=clock.advance)
    link.note = notes.append
    return Task2Stm(link, clock=clock), link, port, notes


def start(d="FIRL", cap=60):
    return f"MM,EV:START,D:{d},T:0,V:{cap}"


def iredge(d="FIRL", r="EDGE", cc=2250, sa=0):
    sc = -1 if r == "NO_OBSTACLE" else 300
    cc = cc if r == "EDGE" else -1
    return f"IREDGE,D:{d},R:{r},SA:{sa},SC:{sc},SR:2655,CC:{cc},CR:{-1 if cc < 0 else 397},UN:0,AMB:0,MISS:1,RDY:30,W:NA"


def end(d="FIRL", result="OK", dl=2700, dr=2700):
    return f"MM,EV:END,D:{d},T:2854,RESULT:{result},DL:{dl},DR:{dr}"


def success(d="FIRL", cap=60):
    return [start(d, cap), "MM,D:F,T:100,ST:DIST", iredge(d), end(d), "ACK"]


class CommandTest(unittest.TestCase):
    def test_sensor_side_selects_the_command(self):
        self.assertEqual(stm_fir.fir_command("L", 60), "MOV,FIRL,60")
        self.assertEqual(stm_fir.fir_command("R", 1), "MOV,FIRR,1")
        self.assertEqual(stm_fir.fir_command("R", 999), "MOV,FIRR,999")

    def test_invalid_requests_send_nothing(self):
        stm, _, port, _ = rig()
        for sensor, cap in (("X", 60), ("l", 60), ("L", 0), ("L", 1000), ("L", -5), ("L", 60.0), ("L", 60.5),
                            ("L", True), ("L", "60")):
            with self.subTest(sensor=sensor, cap=cap), self.assertRaises(ValueError):
                stm.follow_edge(sensor, cap)
        self.assertEqual(port.written, [])

    def test_strict_grammar(self):
        for text in ("MOV,FIRL,060", "MOV,FIRL,0", "MOV,FIRL,1000", "MOV,firl,60", "MOV,FIRL,60 ", "MOV,FIR,60",
                     "MOV,FIRL,+60", "MOV,FIRL,6.0"):
            with self.subTest(text), self.assertRaises(ValueError):
                stm_fir.validate_fir(text)

    def test_pi_timeout_is_stm_deadline_plus_ready_wait_plus_margin(self):
        self.assertAlmostEqual(stm_fir.fir_timeout_s("MOV,FIRL,60"), 5.0 + 0.15 * 60 + 0.3 + 3.0)

    def test_no_hardware_authority(self):
        self.assertTrue(stm_fir.FIR_CAPABILITY.implemented)
        self.assertFalse(stm_fir.FIR_CAPABILITY.bench_validated)
        self.assertFalse(stm_fir.FIR_CAPABILITY.hardware_authority)


class RecordParserTest(unittest.TestCase):
    def test_end_needs_dl_dr_exactly(self):
        self.assertIsNotNone(stm_fir.parse_end(end()))
        self.assertEqual(stm_fir.parse_end(end(dl=-12, dr=0)).dl, -12)
        for text in ("MM,EV:END,D:FIRL,T:2854,RESULT:OK",                       # today's firmware: no DL/DR
                     "MM,EV:END,D:FIRL,T:2854,RESULT:OK,DL:-0,DR:5",
                     "MM,EV:END,D:FIRL,T:2854,RESULT:OK,DL:2147483648,DR:5",
                     "MM,EV:END,D:FIRL,T:2854,RESULT:OK,DL:5,DR:5,X:1",
                     "MM,EV:END,D:FIRL,T:2854,RESULT:OK,DR:5,DL:5",
                     "MM,EV:END,D:F,T:2854,RESULT:OK,DL:5,DR:5"):
            with self.subTest(text):
                self.assertIsNone(stm_fir.parse_end(text))

    def test_iredge_grammar(self):
        parsed = stm_fir.parse_iredge(iredge("FIRR", cc=3704))
        self.assertEqual((parsed.direction, parsed.result, parsed.clear_counts), ("FIRR", "EDGE", 3704))
        self.assertIsNotNone(stm_fir.parse_iredge(
            "IREDGE,D:FIRR,R:EDGE,SA:1,SC:1,SR:2655,CC:3704,CR:397,UN:0,AMB:0,MISS:1,RDY:30,W:NA"))  # handoff §5.3
        for text in ("IREDGE,D:FIRL,R:EDGE", iredge().replace("W:NA", "W:XX"), iredge() + ",X:1",
                     iredge().replace("SA:0", "SA:2"), iredge().replace("D:FIRL", "D:F")):
            with self.subTest(text):
                self.assertIsNone(stm_fir.parse_iredge(text))


class OutcomeTest(unittest.TestCase):
    def run_fir(self, lines, sensor="L", cap=60):
        stm, link, port, notes = rig()
        port.script(stm_fir.fir_command(sensor, cap), lines)
        return stm.follow_edge(sensor, cap), link, port, notes

    def test_complete_record_is_an_edge(self):
        for sensor, d in (("L", "FIRL"), ("R", "FIRR")):
            with self.subTest(sensor):
                result, link, port, _ = self.run_fir(success(d), sensor)
                self.assertEqual(result.status, "EDGE")
                self.assertIs(EdgeOutcome(result.status), EdgeOutcome.EDGE)
                self.assertTrue(result.pose_known)
                self.assertAlmostEqual(result.travel_cm, 2700 / CPC)
                self.assertAlmostEqual(result.edge_at_cm, 2250 / CPC)
                self.assertEqual(result.problems, [])
                self.assertEqual(port.commands_written(), [f"MOV,{d},60"])
                self.assertIs(link.state, LinkState.IDLE)
                self.assertNotIn("unknown", [e.kind for e in link.evidence], "IREDGE is expected, not unknown")

    def test_todays_firmware_end_without_dl_dr_is_a_protocol_error(self):
        lines = [start(), iredge(), "MM,EV:END,D:FIRL,T:2854,RESULT:OK", "ACK"]
        result, link, _, notes = self.run_fir(lines)
        self.assertEqual(result.status, "PROTOCOL_ERROR")
        self.assertFalse(result.pose_known)
        self.assertTrue(any("DL/DR" in p for p in result.problems))
        self.assertTrue(notes, "the rejection is written to the raw log")
        self.assertIs(link.state, LinkState.IDLE, "the terminal was attributable: the robot is not moving")

    def test_ack_without_a_complete_ordered_record_is_never_success(self):
        cases = {
            "plain ACK": [start(), "ACK"],
            "no IREDGE": [start(), end(), "ACK"],
            "no END": [start(), iredge(), "ACK"],
            "IREDGE after END": [start(), end(), iredge(), "ACK"],
            "wrong side record": [start(), iredge("FIRR"), end("FIRR"), "ACK"],
            "IREDGE R:NO_EDGE": [start(), iredge(r="NO_EDGE"), end(), "ACK"],
            "CC:-1": [start(), iredge().replace("CC:2250", "CC:-1"), end(), "ACK"],
            "END RESULT:STOPPED": [start(), iredge(), end(result="STOPPED"), "ACK"],
            "two ENDs": [start(), iredge(), end(), end(), "ACK"],
            "two IREDGEs": [start(), iredge(), iredge(), end(), "ACK"],
            "malformed IREDGE": [start(), "IREDGE,D:FIRL,R:EDGE", end(), "ACK"],
            "no START": [iredge(), end(), "ACK"],
        }
        for name, lines in cases.items():
            with self.subTest(name):
                result, *_ = self.run_fir(lines)
                self.assertEqual(result.status, "PROTOCOL_ERROR")
                self.assertFalse(result.pose_known)
                self.assertTrue(result.problems)
                with self.assertRaises(ValueError):
                    EdgeOutcome(result.status)          # the mission treats it as unsupported: fail safe

    def test_ir_errors_map_to_wrapper_outcomes_with_uncertain_pose(self):
        for final, status, decision, end_result in (("ERR,IR_NO_OBSTACLE", "NO_WALL", "NO_OBSTACLE", "IR_NO_OBSTACLE"),
                                                    ("ERR,IR_NO_EDGE", "MAX_TRAVEL", "NO_EDGE", "IR_NO_EDGE")):
            with self.subTest(final):
                result, *_ = self.run_fir([start(), iredge(r=decision), end(result=end_result, dl=4500, dr=4500), final])
                self.assertEqual(result.status, status)
                self.assertFalse(result.pose_known)
                self.assertAlmostEqual(result.travel_cm, 60.0)
                self.assertIsNone(result.edge_at_cm)
                result, *_ = self.run_fir([final])                         # no START: an anomaly
                self.assertEqual(result.status, "PROTOCOL_ERROR")
                self.assertFalse(result.pose_known)
                result, *_ = self.run_fir([start(), iredge(r="EDGE"), end(result=end_result), final])
                self.assertEqual(result.status, "PROTOCOL_ERROR", "decision disagrees with the terminal")

    def test_ir_sensor_before_and_after_start(self):
        result, *_ = self.run_fir([iredge(r="SENSOR").replace("W:NA", "W:READY"), "ERR,IR_SENSOR"])
        self.assertEqual(result.status, "SENSOR_FAULT")
        self.assertTrue(result.pose_known, "the ready wait moved nothing")
        result, *_ = self.run_fir([start(), iredge(r="SENSOR").replace("W:NA", "W:DATA"),
                                   end(result="IR_SENSOR", dl=900, dr=900), "ERR,IR_SENSOR"])
        self.assertEqual(result.status, "SENSOR_FAULT")
        self.assertFalse(result.pose_known)
        self.assertAlmostEqual(result.travel_cm, 12.0)

    def test_ack_stopped_ends_the_move_and_is_never_success(self):
        result, link, port, _ = self.run_fir([start(), end(result="STOPPED", dl=600, dr=600), "ACK,STOPPED"])
        self.assertEqual(result.status, "STOPPED")
        self.assertFalse(result.pose_known)
        self.assertIs(link.state, LinkState.IDLE, "ACK,STOPPED is this move's terminal, not a timeout")
        self.assertFalse(link.motion_outcome_unknown)
        result, *_ = self.run_fir(["ACK,STOPPED"])                      # stopped in the ready wait
        self.assertEqual(result.status, "STOPPED")

    def test_ack_stopped_still_does_not_end_an_ordinary_mov(self):
        _, link, port, _ = rig()
        port.script("MOV,F,10", ["MM,EV:START,D:F,T:0,V:10", "ACK,STOPPED"])
        self.assertTrue(link.send_motion("MOV,F,10").timed_out)

    def test_generic_errors(self):
        result, *_ = self.run_fir(["ERR,BUSY"])
        self.assertEqual(result.status, "ERROR")
        self.assertTrue(result.pose_known)
        result, *_ = self.run_fir([start(), "ERR,MOTION_TIMEOUT"])
        self.assertEqual(result.status, "ERROR")
        self.assertFalse(result.pose_known)

    def test_timeout_latches_and_nothing_is_resent(self):
        stm, link, port, _ = rig()
        port.script("MOV,FIRL,60", [start(), iredge()])                  # no END, no terminal
        result = stm.follow_edge("L", 60)
        self.assertEqual(result.status, "TIMEOUT")
        self.assertFalse(result.pose_known)
        self.assertTrue(link.motion_outcome_unknown)
        with self.assertRaises(MotionOutcomeUnknownError):
            stm.follow_edge("L", 60)
        self.assertEqual(port.commands_written(), ["MOV,FIRL,60"])

    def test_telemetry_is_kept_in_order(self):
        lines = success()
        result, *_ = self.run_fir(lines)
        self.assertEqual(result.telemetry, lines[:-1])


class LegacyTravelTest(unittest.TestCase):
    """D-76 (supervised runs only): today's END has no DL/DR; the travel comes from the last MM,D sample."""

    LINES = [start(), "MM,D:FIRL,T:900,ST:DIST,TH:0,H:0,HE:0,GZ:0,SP:1500,LT:2600,LP:2400,LE:200,LO:0,RT:2600,"
                      "RP:2500,RE:100,RO:0", iredge(), "MM,EV:END,D:FIRL,T:2854,RESULT:OK", "ACK"]

    def test_off_by_default(self):
        stm, _, port, _ = rig()
        port.script("MOV,FIRL,60", self.LINES)
        self.assertEqual(stm.follow_edge("L", 60).status, "PROTOCOL_ERROR")

    def test_supervised_override(self):
        stm, _, port, _ = rig()
        stm.fir_legacy_travel = True
        port.script("MOV,FIRL,60", self.LINES)
        result = stm.follow_edge("L", 60)
        self.assertEqual(result.status, "EDGE")
        self.assertAlmostEqual(result.travel_cm, 2450 / CPC)
        self.assertFalse(result.pose_known, "approximate travel: never a trusted pose")
        self.assertTrue(result.problems)
        port.script("MOV,FIRL,60", [start(), iredge(), "MM,EV:END,D:FIRL,T:2854,RESULT:OK", "ACK"])
        self.assertEqual(stm.follow_edge("L", 60).status, "PROTOCOL_ERROR", "no MM,D sample: no travel")


if __name__ == "__main__":
    unittest.main()
