"""Final parking with the STM's predictive USPARK stop (TASK2_USPARK_ALGO_INTEGRATION.md §12).
Run from the task2 folder:

    python3 -m unittest discover -s tests -v

Host only: a deterministic fake port replays the spellings of the authority-build firmware (`app_comm.c`),
and the simulator's USPARK fixture reproduces the team-reported stop. Neither proves the robot.
"""

from __future__ import annotations

import unittest

import config
import sim
import stm_park
from stm_link import StmLink
from stm_park import ParkingOutcome
from stm_protocol import LinkState, MotionOutcomeUnknownError
from stm_task2 import Task2Stm

CMD = config.PARK_FINAL_COMMAND
START = "MM,EV:START,D:F,T:0,V:100"
PRED = "USPARK,STOP,R:PRED,D:354,V:4452,FM:00,C:138,TH:190,P:NORM"      # the reported final cause line
END = "MM,EV:END,D:F,T:1500,RESULT:STOPPED"
GET_OK = "USPARK,E:1,ST:UNARMED,CN:0,FM:00,TH:300,RA:320,AR:500"
TRACE = "USPARK,TRACE,EV:PAD,W:1,M:AUTH,P:NORM,TH:190,R:ACC"


def rig():
    clock = sim.FakeClock()
    port = sim.FakeSerialPort(clock, read_step_s=0.05)
    notes: list[str] = []
    link = StmLink(port, log=lambda _: None, clock=clock, sleep=clock.advance)
    link.note = notes.append
    return Task2Stm(link, clock=clock, sleep=clock.advance), link, port


def park(lines):
    stm, link, port = rig()
    port.script(CMD, lines)
    return stm.park_final(), link, port


class ParserTest(unittest.TestCase):
    def test_get_and_arming_checks(self):
        status = stm_park.parse_get(GET_OK)
        self.assertEqual((status.enabled, status.state, status.stop_mm, status.rearm_mm, status.arm_mm),
                         (True, "UNARMED", 300, 320, 500))
        self.assertIsNone(stm_park.armed_problem(status))
        for line, why in (("USPARK,E:0,ST:DISABLED,CN:0,FM:00,TH:300,RA:320,AR:500", "not enabled"),
                          ("USPARK,E:1,ST:UNARMED,CN:0,FM:04,TH:300,RA:320,AR:500", "fault"),
                          ("USPARK,E:1,ST:UNARMED,CN:0,FM:00,TH:250,RA:320,AR:500", "thresholds"),
                          ("USPARK,E:1,ST:ARMED,CN:0,FM:00,TH:300,RA:320,AR:500", "state"),
                          ("USPARK,E:1,ST:UNARMED,CN:1,FM:00,TH:300,RA:320,AR:500", "state")):
            with self.subTest(line):
                self.assertIn(why, stm_park.armed_problem(stm_park.parse_get(line)))
        for line in ("USPARK,E:1,ST:UNARMED,CN:0,FM:00,TH:300,RA:320", GET_OK + ",X:1", GET_OK.replace("ST:UNARMED", "ST:?"),
                     TRACE, "USPARK,OK"):
            with self.subTest(line):
                self.assertIsNone(stm_park.parse_get(line))
        self.assertIn("missing", stm_park.armed_problem(None))

    def test_cause_lines(self):
        pred = stm_park.parse_cause(PRED)
        self.assertEqual((pred.reason, pred.distance_mm, pred.speed_cps, pred.fault_mask, pred.predicted_clearance_mm,
                          pred.threshold_mm, pred.path), ("PRED", 354, 4452, 0, 138, 190, "NORM"))
        self.assertIsNone(stm_park.parse_cause(PRED.replace("C:138", "C:NA")).predicted_clearance_mm)
        legacy = stm_park.parse_cause("USPARK,STOP,R:NEAR,D:NA,V:3000,FM:00")
        self.assertEqual((legacy.reason, legacy.distance_mm, legacy.path), ("NEAR", None, None))
        for line in (PRED + ",X:1", PRED.replace("P:NORM", "P:SLOW"), PRED.replace("D:354", "D:-354"),
                     PRED.replace("FM:00", "FM:0"), "USPARK,STOP,R:PRED", "USPARK,STOP,R:LATE,D:1,V:1,FM:00", TRACE):
            with self.subTest(line):
                self.assertIsNone(stm_park.parse_cause(line))


class TransactionTest(unittest.TestCase):
    def test_complete_chain_is_parked(self):
        for lines in ([START, PRED, END, "ACK,STOPPED"],
                      [START, TRACE, PRED, TRACE, END.replace("STOPPED", "STOPPED,DL:4500,DR:4510"), "ACK,STOPPED"]):
            with self.subTest(lines):
                result, link, port = park(lines)
                self.assertIs(result.outcome, ParkingOutcome.PARKED, result.reason)
                self.assertFalse(result.pose_known, "the planned 100 cm is never applied")
                self.assertTrue(result.start_seen and result.stopped_end_seen)
                self.assertEqual(result.cause.path, "NORM")
                self.assertEqual(port.commands_written(), [CMD])
                self.assertIs(link.state, LinkState.IDLE)
                self.assertNotIn("unknown", [e.kind for e in link.evidence], "USPARK lines are expected evidence")

    def test_protective_and_fault_stops_are_not_parked(self):
        cases = {
            "FAST": ([START, PRED.replace("TH:190,P:NORM", "TH:250,P:FAST"), END, "ACK,STOPPED"], ParkingOutcome.PROTECTIVE_STOP),
            "OVER": ([START, PRED.replace("C:138,TH:190,P:NORM", "C:NA,TH:0,P:OVER"), END, "ACK,STOPPED"], ParkingOutcome.PROTECTIVE_STOP),
            "legacy NEAR": ([START, "USPARK,STOP,R:NEAR,D:290,V:3000,FM:00", END, "ACK,STOPPED"], ParkingOutcome.PROTECTIVE_STOP),
            "fault line": ([START, "USPARK,FAULT,FM:04", "USPARK,STOP,R:FAULT,D:NA,V:3000,FM:04", END, "ACK,STOPPED"],
                           ParkingOutcome.SENSOR_FAULT),
            "fault mask": ([START, PRED.replace("FM:00", "FM:04"), END, "ACK,STOPPED"], ParkingOutcome.SENSOR_FAULT),
            "plain ACK": ([START, "MM,EV:END,D:F,T:3000,RESULT:OK", "ACK"], ParkingOutcome.NORMAL_COMPLETION),
        }
        for name, (lines, outcome) in cases.items():
            with self.subTest(name):
                result, *_ = park(lines)
                self.assertIs(result.outcome, outcome, result.reason)
                self.assertFalse(result.pose_known)

    def test_incomplete_or_disordered_evidence_is_a_protocol_error(self):
        cases = {
            "no cause": [START, END, "ACK,STOPPED"],
            "no END": [START, PRED, "ACK,STOPPED"],
            "no START": [PRED, END, "ACK,STOPPED"],
            "two causes": [START, PRED, PRED, END, "ACK,STOPPED"],
            "two ENDs": [START, PRED, END, END, "ACK,STOPPED"],
            "cause after END": [START, END, PRED, "ACK,STOPPED"],
            "END not STOPPED": [START, PRED, END.replace("STOPPED", "OK"), "ACK,STOPPED"],
            "END wrong direction": [START, PRED, END.replace("D:F", "D:B"), "ACK,STOPPED"],
            "malformed cause": [START, "USPARK,STOP,R:PRED,D:354", END, "ACK,STOPPED"],
            "other trigger": [START, PRED.replace("TH:190", "TH:250"), END, "ACK,STOPPED"],
        }
        for name, lines in cases.items():
            with self.subTest(name):
                result, *_ = park(lines)
                self.assertIs(result.outcome, ParkingOutcome.PROTOCOL_ERROR, result.reason)
                self.assertFalse(result.pose_known)

    def test_errors_and_timeout(self):
        result, link, _ = park(["ERR,BUSY"])
        self.assertIs(result.outcome, ParkingOutcome.STM_ERROR)
        self.assertTrue(result.pose_known, "rejected before any movement")
        result, *_ = park([START, "ERR,IMU_NOT_READY"])
        self.assertFalse(result.pose_known)
        result, *_ = park(["ERR,MOTION_TIMEOUT"])
        self.assertFalse(result.pose_known, "MOTION_TIMEOUT never leaves the pose known")
        stm, link, port = rig()
        port.script(CMD, [START, PRED])                              # no END, no terminal
        result = stm.park_final()
        self.assertIs(result.outcome, ParkingOutcome.TIMEOUT)
        self.assertTrue(link.motion_outcome_unknown)
        with self.assertRaises(MotionOutcomeUnknownError):
            stm.park_final()
        self.assertEqual(port.commands_written(), [CMD], "never resent")

    def test_only_the_final_command(self):
        _, link, port = rig()
        for command in ("MOV,F,110", "MOV,F,180", "MOV,F,99", "MOV,B,100"):
            with self.subTest(command), self.assertRaises(ValueError):
                link.send_parking_motion(command)
        self.assertEqual(port.written, [])

    def test_ack_stopped_still_never_ends_an_ordinary_move(self):
        _, link, port = rig()
        port.script("MOV,F,10", ["MM,EV:START,D:F,T:0,V:10", "ACK,STOPPED"])
        self.assertTrue(link.send_motion("MOV,F,10").timed_out)


class ArmingTest(unittest.TestCase):
    def arm(self, get_reply="USPARK,E:1,ST:UNARMED,CN:0,FM:00,TH:300,RA:320,AR:500", set1="USPARK,OK"):
        stm, link, port = rig()
        port.script("PROFILE,TASK2TEST", ["PROFILE,OK"])
        port.script("PROFILE,GET", ["PROFILE,TASK2TEST"])
        port.script("USPARK,0", [TRACE, "USPARK,OK"])
        port.script("USPARK,1", [set1])
        port.script("USPARK,GET", [get_reply])
        return stm.prepare_final_parking(), port

    def test_sequence(self):
        reason, port = self.arm()
        self.assertIsNone(reason)
        self.assertEqual(port.commands_written(), ["PROFILE,TASK2TEST", "PROFILE,GET", "USPARK,0", "USPARK,1", "USPARK,GET"])

    def test_refusals(self):
        reason, port = self.arm(set1="ERR,US_FAULT")
        self.assertIn("ERR,US_FAULT", reason)
        self.assertNotIn("USPARK,GET", port.commands_written())
        reason, _ = self.arm(get_reply="USPARK,E:1,ST:UNARMED,CN:0,FM:00,TH:250,RA:320,AR:500")
        self.assertIn("thresholds", reason)


if __name__ == "__main__":
    unittest.main()
