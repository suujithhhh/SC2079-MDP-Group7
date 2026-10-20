"""Hardware-free tests for the STM link's ultrasonic parking-protection (USPARK) support.
Run from rpi_task1/:
    python -m unittest discover -s tests -v

Protocol: RTOSV/PROTOCOL.md section 3 (STOP, `ACK,STOPPED`) and 3A (USPARK).
`ACK,STOPPED` ends a MOV the RPi did not stop itself, so it must be a terminal,
non-successful result with the pose unknown, and the cause lines that precede
it must be kept.
"""

import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:                                    # discovery with `-s tests` imports siblings by bare name
    from test_rpi import PROFILE_OK, RIGHT, FakeClock, FakeSerial, align_mission, mission
except ImportError:                     # `python -m unittest tests.test_stm_uspark` from the parent
    from tests.test_rpi import PROFILE_OK, RIGHT, FakeClock, FakeSerial, align_mission, mission

from stm_link import StmLink, StmReply, parse_uspark_get  # noqa: E402

START = "MM,EV:START,D:F,T:0,V:10"
SAMPLE = ("MM,D:F,T:300,ST:RUN,TH:0,H:900,HE:0,GZ:0,SP:600,LT:200,LP:300,LE:0,LO:0,RT:200,RP:300,RE:0,RO:0")
END_STOPPED = "MM,EV:END,D:F,T:410,RESULT:STOPPED"
STOP_NEAR = "USPARK,STOP,R:NEAR,D:195,V:310,FM:00"
FAULT = "USPARK,FAULT,FM:04"
STOP_FAULT = "USPARK,STOP,R:FAULT,D:NA,V:0,FM:04"
GET_LINE = "USPARK,E:1,ST:ARMED,CN:0,FM:00,TH:200,RA:220,AR:400"


def stm_with(script, raw_log=None, logged=None):
    port = FakeSerial(script)
    clock = FakeClock()
    stm = StmLink(port, log=(logged.append if logged is not None else (lambda _: None)),
                  clock=clock, sleep=lambda _: None, raw_log=raw_log)
    return stm, port, clock


class StoppedResultTests(unittest.TestCase):
    def test_normal_mov_telemetry_then_ack(self):
        stm, port, _ = stm_with({"MOV,F,10": [START, SAMPLE, "MM,EV:END,D:F,T:400,RESULT:OK", "ACK"]})
        reply = stm.move("MOV,F,10")
        self.assertEqual((reply.status, reply.line), ("ok", "ACK"))
        self.assertTrue(reply.ok)
        self.assertFalse(reply.pose_unknown)
        self.assertFalse(reply.stopped)
        self.assertTrue(reply.motion_started)
        self.assertEqual(reply.telemetry, [START, SAMPLE, "MM,EV:END,D:F,T:400,RESULT:OK"])
        self.assertIsNone(reply.uspark_stop_line)
        self.assertIsNone(reply.uspark_fault_line)
        self.assertIsNone(reply.uspark_stop_reason)
        self.assertEqual(port.written, ["MOV,F,10"])

    def test_automatic_stop_sequence(self):
        sequence = [START, STOP_NEAR, SAMPLE, END_STOPPED, "ACK,STOPPED"]
        stm, port, _ = stm_with({"MOV,F,10": sequence})
        reply = stm.move("MOV,F,10")
        self.assertEqual(reply.status, "stopped")
        self.assertFalse(reply.ok)
        self.assertTrue(reply.pose_unknown)
        self.assertTrue(reply.stopped)
        self.assertEqual(reply.line, "ACK,STOPPED")
        self.assertTrue(reply.motion_started)
        self.assertEqual(reply.telemetry, sequence[:-1])          # every preceding line, in order
        self.assertEqual(reply.uspark_stop_line, STOP_NEAR)
        self.assertEqual(reply.uspark_stop_reason, "NEAR")
        self.assertIsNone(reply.uspark_fault_line)
        self.assertEqual(reply.malformed_lines, 0)
        self.assertEqual(port.written, ["MOV,F,10"])              # not resent

    def test_fault_then_automatic_stop(self):
        sequence = [START, FAULT, STOP_FAULT, END_STOPPED, "ACK,STOPPED"]
        stm, port, _ = stm_with({"MOV,F,10": sequence})
        reply = stm.move("MOV,F,10")
        self.assertEqual((reply.status, reply.line), ("stopped", "ACK,STOPPED"))
        self.assertTrue(reply.pose_unknown)
        self.assertEqual(reply.telemetry, sequence[:-1])
        self.assertEqual(reply.uspark_fault_line, FAULT)
        self.assertEqual(reply.uspark_stop_line, STOP_FAULT)
        self.assertEqual(reply.uspark_stop_reason, "FAULT")
        self.assertEqual(port.written, ["MOV,F,10"])

    def test_every_stop_reason_is_parsed(self):
        for reason, distance in (("NEAR", "199"), ("INVALID", "310"), ("NOUPDATE", "NA"), ("FAULT", "NA")):
            line = f"USPARK,STOP,R:{reason},D:{distance},V:-5,FM:00"
            stm, _, _ = stm_with({"MOV,F,10": [START, line, END_STOPPED, "ACK,STOPPED"]})
            reply = stm.move("MOV,F,10")
            self.assertEqual(reply.uspark_stop_reason, reason, line)
            self.assertEqual(reply.uspark_stop_line, line)

    def test_unknown_or_missing_reason_is_none_but_the_line_is_kept(self):
        for line in ("USPARK,STOP,R:BOGUS,D:1,V:1,FM:00", "USPARK,STOP,D:195,V:1,FM:00", "USPARK,STOP,"):
            stm, _, _ = stm_with({"MOV,F,10": [START, line, END_STOPPED, "ACK,STOPPED"]})
            reply = stm.move("MOV,F,10")
            self.assertEqual(reply.status, "stopped", line)
            self.assertEqual(reply.uspark_stop_line, line)
            self.assertIsNone(reply.uspark_stop_reason, line)

    def test_manual_stop_has_no_uspark_cause(self):
        sequence = [START, SAMPLE, END_STOPPED, "ACK,STOPPED"]
        stm, port, _ = stm_with({"MOV,F,10": sequence})
        reply = stm.move("MOV,F,10")
        self.assertEqual((reply.status, reply.line), ("stopped", "ACK,STOPPED"))
        self.assertFalse(reply.ok)
        self.assertTrue(reply.pose_unknown)
        self.assertIsNone(reply.uspark_stop_line)
        self.assertIsNone(reply.uspark_fault_line)
        self.assertIsNone(reply.uspark_stop_reason)
        self.assertEqual(reply.telemetry, sequence[:-1])
        self.assertEqual(port.written, ["MOV,F,10"])

    def test_stop_before_the_movement_started_still_leaves_the_pose_unknown(self):
        stm, _, _ = stm_with({"MOV,F,10": ["ACK,STOPPED"]})
        reply = stm.move("MOV,F,10")
        self.assertEqual(reply.status, "stopped")
        self.assertFalse(reply.motion_started)
        self.assertTrue(reply.pose_unknown)           # never "rejected before moving": the robot may have started

    def test_ack_and_ack_stopped_are_different_exact_tokens(self):
        stm, _, _ = stm_with({"MOV,F,10": ["ACK"], "MOV,B,10": ["ACK,STOPPED"]})
        ack, stopped = stm.move("MOV,F,10"), stm.move("MOV,B,10")
        self.assertEqual((ack.status, ack.ok, ack.pose_unknown), ("ok", True, False))
        self.assertEqual((stopped.status, stopped.ok, stopped.pose_unknown), ("stopped", False, True))

    def test_near_miss_tokens_are_telemetry_not_terminal(self):
        for line in ("ACK,STOPPED ", " ACK,STOPPED", "ACK,STOP", "ACK,STOPPED,1", "ACK,", "ACK ", "ack,stopped",
                     "ACK,Stopped", "XACK,STOPPED", "ACK,STOPPEDX", "STOPPED", "ACK:STOPPED", "ACK, STOPPED"):
            stm, port, _ = stm_with({"MOV,F,10": [START, line]})
            reply = stm.move("MOV,F,10")
            self.assertEqual(reply.status, "timeout", repr(line))
            self.assertIn(line, reply.telemetry, repr(line))
            self.assertEqual(port.written, ["MOV,F,10"], repr(line))

    def test_stopped_is_terminal_and_does_not_wait_for_the_timeout(self):
        stm, port, clock = stm_with({"MOV,F,10": [START, STOP_NEAR, END_STOPPED, "ACK,STOPPED", "ACK"]})
        reply = stm.move("MOV,F,10")
        self.assertEqual(reply.status, "stopped")
        self.assertEqual(port.pending, ["ACK"])        # nothing after the stop was read
        self.assertLess(clock.now, 10.0)               # far below the 8.5 s+ MOV deadline in clock ticks of 0.5

    def test_silence_still_times_out_with_a_cause_line_but_no_terminal(self):
        # the old behaviour for a link that never delivers ACK,STOPPED: a timeout, pose unknown, no resend
        stm, port, _ = stm_with({"MOV,F,10": [START, STOP_NEAR, END_STOPPED]})
        reply = stm.move("MOV,F,10")
        self.assertEqual(reply.status, "timeout")
        self.assertTrue(reply.pose_unknown)
        self.assertEqual(reply.uspark_stop_line, STOP_NEAR)
        self.assertEqual(port.written, ["MOV,F,10"])

    def test_err_after_a_cause_line_keeps_error_behaviour(self):
        stm, _, _ = stm_with({"MOV,F,10": [START, STOP_NEAR, "ERR,MOTION_TIMEOUT"]})
        reply = stm.move("MOV,F,10")
        self.assertEqual((reply.status, reply.line), ("error", "ERR,MOTION_TIMEOUT"))
        self.assertTrue(reply.pose_unknown)            # motion had started
        self.assertEqual(reply.uspark_stop_reason, "NEAR")

    def test_err_before_motion_keeps_error_behaviour(self):
        stm, _, _ = stm_with({"MOV,F,10": ["ERR,BAD_CMD"]})
        reply = stm.move("MOV,F,10")
        self.assertEqual((reply.status, reply.line, reply.pose_unknown), ("error", "ERR,BAD_CMD", False))

    def test_cause_lines_stay_in_the_raw_log_in_order(self):
        sequence = [START, FAULT, STOP_FAULT, SAMPLE, END_STOPPED, "ACK,STOPPED"]
        raw, logged = io.StringIO(), []
        stm, _, _ = stm_with({"MOV,F,10": sequence}, raw_log=raw, logged=logged)
        stm.note("mission m1")
        stm.move("MOV,F,10")
        lines = raw.getvalue().splitlines()
        self.assertEqual(lines[0], "## mission m1")
        self.assertTrue(lines[1].endswith("<- MOV,F,10"))
        arrived = [line.split(" -> ", 1)[1] for line in lines[2:]]
        self.assertEqual(arrived, sequence)
        self.assertTrue(any(STOP_FAULT in entry for entry in logged))     # the operator log shows the cause
        self.assertTrue(any(FAULT in entry for entry in logged))
        self.assertTrue(any("ACK,STOPPED" in entry for entry in logged))

    def test_reply_is_backward_compatible(self):
        reply = StmReply("ok")
        self.assertEqual((reply.line, reply.telemetry, reply.motion_started, reply.detail, reply.malformed_lines),
                         ("", [], False, "", 0))
        self.assertIsNone(reply.uspark_stop_line)
        self.assertIsNone(reply.uspark_fault_line)
        self.assertIsNone(reply.uspark_stop_reason)
        self.assertTrue(StmReply("timeout").pose_unknown)
        positional = StmReply("error", "ERR,BUSY", True, ["MM,EV:START"], "d", 2)     # old positional order
        self.assertEqual((positional.line, positional.motion_started, positional.telemetry), ("ERR,BUSY", True, ["MM,EV:START"]))


class UsparkControlTests(unittest.TestCase):
    def test_enable_success_is_the_exact_line(self):
        stm, port, _ = stm_with({"USPARK,1": ["USPARK,OK"]})
        reply = stm.uspark_enable()
        self.assertEqual((reply.status, reply.line, reply.ok), ("ok", "USPARK,OK", True))
        self.assertEqual(port.written, ["USPARK,1"])

    def test_enable_busy_is_an_ordinary_error_without_retry(self):
        stm, port, _ = stm_with({"USPARK,1": ["ERR,BUSY", "USPARK,OK"]})
        reply = stm.uspark_enable()
        self.assertEqual((reply.status, reply.line, reply.ok), ("error", "ERR,BUSY", False))
        self.assertEqual(port.written, ["USPARK,1"])
        self.assertEqual(port.pending, ["USPARK,OK"])          # the later line was not consumed or acted on

    def test_enable_us_fault_is_an_ordinary_error_without_retry(self):
        stm, port, _ = stm_with({"USPARK,1": ["ERR,US_FAULT"]})
        reply = stm.uspark_enable()
        self.assertEqual((reply.status, reply.line, reply.ok), ("error", "ERR,US_FAULT", False))
        self.assertEqual(port.written, ["USPARK,1"])

    def test_enable_silence_times_out_without_retry(self):
        stm, port, _ = stm_with({})
        reply = stm.uspark_enable()
        self.assertEqual((reply.status, reply.ok), ("timeout", False))
        self.assertEqual(port.written, ["USPARK,1"])

    def test_enable_bad_cmd_is_an_error(self):
        stm, _, _ = stm_with({"USPARK,1": ["ERR,BAD_CMD"]})
        self.assertEqual(stm.uspark_enable().line, "ERR,BAD_CMD")

    def test_only_the_exact_ok_line_succeeds(self):
        for line in ("USPARK,OKAY", "USPARK,OK ", " USPARK,OK", "USPARK,ok", "USPARK,1", "USPARK,0", "OK",
                     "USPARK,FAULT,FM:04", GET_LINE, "PROFILE,OK", "ACK", "USPARK,OK,1"):
            for call, command in (("uspark_enable", "USPARK,1"), ("uspark_disable", "USPARK,0")):
                stm, port, _ = stm_with({command: [line]})
                reply = getattr(stm, call)()
                self.assertEqual(reply.status, "timeout", f"{command} accepted {line!r}")
                self.assertIn(line, reply.telemetry)
                self.assertEqual(port.written, [command])

    def test_unrelated_telemetry_before_the_reply_is_kept_not_accepted(self):
        stm, _, _ = stm_with({"USPARK,1": [FAULT, "MM,D:F,T:1", "ERR,US_FAULT"]})
        reply = stm.uspark_enable()
        self.assertEqual((reply.status, reply.line), ("error", "ERR,US_FAULT"))
        self.assertEqual(reply.telemetry, [FAULT, "MM,D:F,T:1"])

    def test_disable(self):
        stm, port, _ = stm_with({"USPARK,0": ["USPARK,OK"]})
        reply = stm.uspark_disable()
        self.assertEqual((reply.status, reply.line, reply.ok), ("ok", "USPARK,OK", True))
        self.assertEqual(port.written, ["USPARK,0"])

    def test_disable_error_is_returned_without_retry(self):
        stm, port, _ = stm_with({"USPARK,0": ["ERR,BAD_CMD"]})
        reply = stm.uspark_disable()
        self.assertEqual((reply.status, reply.line), ("error", "ERR,BAD_CMD"))
        self.assertEqual(port.written, ["USPARK,0"])

    def test_get_returns_the_exact_state_line(self):
        stm, port, _ = stm_with({"USPARK,GET": [GET_LINE]})
        reply = stm.uspark_get()
        self.assertEqual((reply.status, reply.line, reply.ok), ("ok", GET_LINE, True))
        self.assertEqual(port.written, ["USPARK,GET"])
        self.assertEqual(parse_uspark_get(reply.line),
                         {"E": "1", "ST": "ARMED", "CN": "0", "FM": "00", "TH": "200", "RA": "220", "AR": "400"})

    def test_get_accepts_every_documented_state(self):
        for state in ("DISABLED", "UNARMED", "ARMED", "NEAR1", "NEAR2", "LATCHED"):
            line = f"USPARK,E:{0 if state == 'DISABLED' else 1},ST:{state},CN:1,FM:0A,TH:200,RA:220,AR:400"
            stm, _, _ = stm_with({"USPARK,GET": [line]})
            reply = stm.uspark_get()
            self.assertEqual((reply.status, reply.line), ("ok", line), state)
            self.assertEqual(parse_uspark_get(line)["ST"], state)

    def test_get_does_not_accept_other_lines(self):
        for line in ("USPARK,OK", "USPARK,E:2,ST:ARMED,CN:0,FM:00,TH:200,RA:220,AR:400",
                     "USPARK,E:1,ST:BOGUS,CN:0,FM:00,TH:200,RA:220,AR:400",
                     "USPARK,E:1,ST:ARMED,CN:0,FM:00,TH:200,RA:220",            # truncated
                     GET_LINE + " ", GET_LINE + ",X:1", "usPARK,E:1,ST:ARMED,CN:0,FM:00,TH:200,RA:220,AR:400",
                     "IMU,Y:1,GZ:0", "USPARK,STOP,R:NEAR,D:195,V:1,FM:00"):
            stm, port, _ = stm_with({"USPARK,GET": [line]})
            reply = stm.uspark_get()
            self.assertEqual(reply.status, "timeout", repr(line))
            self.assertEqual(port.written, ["USPARK,GET"])

    def test_get_error_is_returned(self):
        stm, _, _ = stm_with({"USPARK,GET": ["ERR,BAD_CMD"]})
        self.assertEqual(stm.uspark_get().status, "error")

    def test_parse_uspark_get_rejects_non_state_lines(self):
        self.assertIsNone(parse_uspark_get("USPARK,OK"))
        self.assertIsNone(parse_uspark_get(""))
        self.assertIsNone(parse_uspark_get(GET_LINE + "\n"))

    def test_no_control_exchange_sends_anything_else(self):
        stm, port, _ = stm_with({"USPARK,1": ["USPARK,OK"], "USPARK,GET": [GET_LINE], "USPARK,0": ["USPARK,OK"]})
        stm.uspark_enable()
        stm.uspark_get()
        stm.uspark_disable()
        self.assertEqual(port.written, ["USPARK,1", "USPARK,GET", "USPARK,0"])


class NoAutomaticRetryTests(unittest.TestCase):
    def test_a_stopped_move_is_sent_exactly_once(self):
        stm, port, _ = stm_with({"MOV,F,10": [START, STOP_NEAR, END_STOPPED, "ACK,STOPPED"]})
        stm.move("MOV,F,10")
        self.assertEqual(port.written, ["MOV,F,10"])

    def test_each_outcome_sends_its_command_once(self):
        cases = {"ok": ["ACK"], "stopped": ["ACK,STOPPED"], "error": ["ERR,BUSY"], "timeout": []}
        for status, lines in cases.items():
            stm, port, _ = stm_with({"MOV,B,10": lines})
            self.assertEqual(stm.move("MOV,B,10").status, status)
            self.assertEqual(port.written, ["MOV,B,10"], status)


class MissionAbortTests(unittest.TestCase):
    """The existing task1.py callers only test `reply.ok`, so a stopped MOV must abort the mission safely."""

    STOP_REPLY = [START, STOP_NEAR, SAMPLE, END_STOPPED, "ACK,STOPPED"]

    def test_stopped_mov_ends_the_mission_with_the_pose_unknown(self):
        m, android, port = mission({**PROFILE_OK, "MOV,F,30": self.STOP_REPLY, "MOV,FR,90": ["ACK"]}, {3: "20", 5: "11"})
        m.execute()
        self.assertEqual(port.written[2:], ["MOV,F,30"])                  # not resent, next MOV never sent
        self.assertTrue(any("Stopped at seq 0" in s and "ACK,STOPPED" in s and "position unknown" in s
                            for s in android.sent))
        self.assertEqual(sum(1 for s in android.sent if s.startswith("ROBOT")), 1)   # only the start pose
        self.assertFalse(any(s.startswith("TARGET") for s in android.sent))
        self.assertFalse(any("Mission done" in s for s in android.sent))
        self.assertEqual(m.vision.submitted, [])

    def test_stopped_heading_fix_ends_the_mission(self):
        m, android, port = mission({**PROFILE_OK, "MOV,FL,3": ["ACK,STOPPED"]}, {3: "20"}, headings=[0.0, -3.0, -0.4])
        m.plan["actions"] = [{"seq": 0, "type": "SNAP", "obstacle_id": 3}]
        m.execute()
        self.assertEqual(port.written[2:], ["IMU,GET", "IMU,GET", "MOV,FL,3"])   # no resend, no second look
        self.assertTrue(any("heading fix" in s and "position unknown" in s for s in android.sent))
        self.assertEqual(m.vision.submitted, [])

    def test_stopped_alignment_burst_ends_the_mission(self):
        m, android, port = align_mission([0.5], {RIGHT[0]: ["ACK,STOPPED"]})
        m.execute()
        self.assertEqual(port.written[2:], ["MOV,F,30", RIGHT[0]])
        self.assertTrue(any("alignment" in s and "position unknown" in s for s in android.sent))
        self.assertFalse(any(s.startswith("TARGET") for s in android.sent))

    def test_task1_never_sends_uspark_by_itself(self):
        m, _, port = mission({**PROFILE_OK, "MOV,F,30": ["ACK"], "MOV,FR,90": ["ACK"]}, {3: "20", 5: "11"})
        m.execute()
        self.assertFalse(any(command.startswith("USPARK") for command in port.written))


if __name__ == "__main__":
    unittest.main()
