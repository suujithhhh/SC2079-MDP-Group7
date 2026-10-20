"""StmLink and Mission over the byte-level UART transport, with fake ports only.
Run from rpi_task1/:  python -m unittest discover -s tests -v
"""

import argparse
import json
import sys
import threading
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))   # sibling test modules (test_rpi, test_uart)

from stm_link import MAX_MALFORMED_LINES, StmLink, StmReply, mov_timeout_s, open_serial  # noqa: E402
from task1 import Mission  # noqa: E402
from uart import STM_MAX_RX_LINE_CHARS, UartCommandError, UartTransport  # noqa: E402

from test_rpi import PLAN, SETUP, FakeAndroid, FakeClock, FakeSerial, FakeVision, link  # noqa: E402
from test_uart import BlockingPort, BlockingPortNoCancel, ManualClock, ScriptedPort  # noqa: E402

PROFILE = {"PROFILE,PRECISION": [b"PROFILE,OK\n"], "PROFILE,GET": [b"PROFILE,PRECISION\n"]}
STANDARD = {"set_command": "PROFILE,STANDARD", "expected_set_reply": "PROFILE,OK",
            "verify_command": "PROFILE,GET", "expected_verify_reply": "PROFILE,STANDARD"}
START = b"MM,EV:START,D:F,T:0,V:30\n"
SAMPLE = (b"MM,D:F,T:100,ST:DIST,TH:900,H:900,HE:0,GZ:0,SP:1530,LT:90,LP:80,LE:10,LO:5,"
          b"RT:90,RP:81,RE:9,RO:5\n")


def stm_on(port, clock):
    uart = UartTransport(port, clock=clock)
    return StmLink(uart, log=lambda _: None, clock=clock, sleep=lambda _: None), uart


def stm_with(replies=None):
    clock = ManualClock()
    port = ScriptedPort(clock, replies)
    stm, _ = stm_on(port, clock)
    return stm, port, clock


class HandshakePort(ScriptedPort):
    """Flags any MOV written while the previous MOV's ACK/ERR has not been read."""

    def __init__(self, clock=None, replies=None, read_delay=0.0):
        super().__init__(clock, replies)
        self.outstanding = False
        self.violations = []
        self.tail = b""
        self.read_delay = read_delay

    def write(self, data):
        if data.startswith(b"MOV"):
            if self.outstanding:
                self.violations.append(bytes(data))
            self.outstanding = True
            self.tail = b""
        return super().write(data)

    def read(self, size):
        if self.read_delay:
            threading.Event().wait(self.read_delay)
        chunk = super().read(size)
        self.tail = (self.tail + chunk)[-64:]
        if self.tail.endswith(b"ACK\n") or b"ERR," in self.tail:
            self.outstanding = False
        return chunk


class MovTests(unittest.TestCase):
    def test_normal_move_sends_the_exact_line_and_returns_on_ack(self):
        stm, port, _ = stm_with({"MOV,F,30": [START, SAMPLE, b"ACK\n"]})
        reply = stm.move("MOV,F,30")
        self.assertTrue(reply.ok)
        self.assertEqual(port.written, [b"MOV,F,30\n"])
        self.assertTrue(reply.motion_started)
        self.assertEqual(len(reply.telemetry), 2)           # ACK itself is not telemetry
        self.assertFalse(reply.pose_unknown)

    def test_fragmented_ack_and_telemetry(self):
        chunks = [bytes([b]) for b in START + SAMPLE + b"ACK\n"]
        stm, port, _ = stm_with({"MOV,F,30": chunks})
        self.assertTrue(stm.move("MOV,F,30").ok)

    def test_several_replies_in_one_read(self):
        stm, port, _ = stm_with({"MOV,F,30": [START + SAMPLE + b"TB,x\nACK\n"]})
        reply = stm.move("MOV,F,30")
        self.assertTrue(reply.ok)
        self.assertEqual(len(port.read_sizes), 1)
        self.assertEqual(reply.telemetry[-1], "TB,x")

    def test_crlf_replies(self):
        stm, _, _ = stm_with({"MOV,F,30": [b"MM,EV:START,D:F,T:0,V:30\r\nACK\r\n"]})
        reply = stm.move("MOV,F,30")
        self.assertTrue(reply.ok)
        self.assertEqual(reply.telemetry, ["MM,EV:START,D:F,T:0,V:30"])

    def test_error_after_several_telemetry_lines_is_terminal(self):
        stm, port, _ = stm_with({"MOV,FL,90": [START, SAMPLE, SAMPLE, b"ERR,MOTION_TIMEOUT\nACK\n"]})
        reply = stm.move("MOV,FL,90")
        self.assertEqual((reply.status, reply.line), ("error", "ERR,MOTION_TIMEOUT"))
        self.assertTrue(reply.motion_started)
        self.assertTrue(reply.pose_unknown)
        self.assertEqual(len(reply.telemetry), 3)

    def test_busy_and_bad_cmd_and_pre_motion_errors(self):
        for error, unknown in (("ERR,BUSY", True), ("ERR,BAD_CMD", False), ("ERR,PID_NOT_SET", False),
                               ("ERR,IMU_NOT_READY", False)):
            stm, port, _ = stm_with({"MOV,F,30": [error.encode() + b"\n"]})
            reply = stm.move("MOV,F,30")
            self.assertEqual((reply.status, reply.line), ("error", error), error)
            self.assertFalse(reply.motion_started)
            self.assertEqual(reply.pose_unknown, unknown, error)
            self.assertEqual(len(port.written), 1)

    def test_imu_not_ready_after_start_means_pose_unknown(self):
        stm, _, _ = stm_with({"MOV,F,30": [START, b"ERR,IMU_NOT_READY\n"]})
        reply = stm.move("MOV,F,30")
        self.assertTrue(reply.motion_started)
        self.assertTrue(reply.pose_unknown)

    def test_motion_started_is_only_set_by_the_start_marker(self):
        stm, _, _ = stm_with({"MOV,F,30": [SAMPLE, b"AK,x\nACK\n"]})
        self.assertFalse(stm.move("MOV,F,30").motion_started)

    def test_other_lines_are_never_a_move_reply(self):
        for lines in (["DONE"], ["OK"], ["ack"], ["ACK:Done"], ["ACK,1"], ["PROFILE,OK"], ["ENC,RST,OK"],
                      ["MM,EV:END,D:F,T:1,RESULT:OK"], ["MD,x", "MS,x", "HS,x", "RTG,x", "MP,x", "SPC,x",
                                                         "STP,x", "SFF,x", "TL,x", "TR,x", "TS,x", "TB,x", "AK,x"]):
            stm, port, _ = stm_with({"MOV,F,30": [(line + "\n").encode() for line in lines]})
            reply = stm.move("MOV,F,30")
            self.assertEqual(reply.status, "timeout", lines)
            self.assertFalse(reply.ok)
            self.assertEqual(len(port.written), 1)

    def test_timeout_before_motion_is_known_to_start_never_resends(self):
        stm, port, clock = stm_with({})
        reply = stm.move("MOV,F,30")
        self.assertEqual(reply.status, "timeout")
        self.assertFalse(reply.motion_started)
        self.assertTrue(reply.pose_unknown)                 # still ambiguous: the reply may just be lost
        self.assertEqual(port.written, [b"MOV,F,30\n"])
        self.assertGreaterEqual(clock.now, mov_timeout_s("MOV,F,30") - 0.2)

    def test_timeout_after_start_marks_motion_started_and_never_resends(self):
        stm, port, _ = stm_with({"MOV,F,30": [START, SAMPLE]})
        reply = stm.move("MOV,F,30")
        self.assertEqual(reply.status, "timeout")
        self.assertTrue(reply.motion_started)
        self.assertTrue(reply.pose_unknown)
        self.assertEqual(port.written, [b"MOV,F,30\n"])

    def test_disconnect_before_reply(self):
        stm, port, _ = stm_with({})
        port.error_when_empty = OSError("device reports readiness to read but returned no data")
        reply = stm.move("MOV,F,30")
        self.assertEqual(reply.status, "disconnected")
        self.assertEqual(reply.line, "UART disconnected")
        self.assertIn("readiness", reply.detail)
        self.assertTrue(reply.pose_unknown)
        self.assertEqual(port.written, [b"MOV,F,30\n"])
        again = stm.move("MOV,B,10")                         # the link stays dead; nothing is sent
        self.assertEqual(again.status, "disconnected")
        self.assertEqual(port.written, [b"MOV,F,30\n"])

    def test_disconnect_after_start(self):
        stm, port, _ = stm_with({"MOV,F,30": [START]})
        port.error_when_empty = OSError("gone")
        reply = stm.move("MOV,F,30")
        self.assertEqual(reply.status, "disconnected")
        self.assertTrue(reply.motion_started)
        self.assertEqual(len(port.written), 1)

    def test_write_failure_and_short_write(self):
        stm, port, _ = stm_with({})
        port.write_error = OSError("write failed")
        reply = stm.move("MOV,F,30")
        self.assertEqual(reply.status, "write_failed")
        self.assertTrue(reply.pose_unknown)
        self.assertEqual(port.written, [])
        stm, port, _ = stm_with({"MOV,F,30": [b"ACK\n"]})
        port.short_write = 3
        reply = stm.move("MOV,F,30")
        self.assertEqual(reply.status, "write_failed")
        self.assertEqual(len(port.written), 1)              # not retried, and the queued ACK is not believed

    def test_garbled_ack_is_not_success(self):
        stm, _, _ = stm_with({"MOV,F,30": [b"AC\xffK\n"]})
        reply = stm.move("MOV,F,30")
        self.assertEqual(reply.status, "timeout")
        self.assertEqual(reply.malformed_lines, 1)

    def test_malformed_telemetry_is_skipped_and_the_real_ack_is_used(self):
        stm, _, _ = stm_with({"MOV,F,30": [b"\xfe\xfdgarbage\n", START, b"M\x00M,x\n", b"ACK\n"]})
        reply = stm.move("MOV,F,30")
        self.assertTrue(reply.ok)
        self.assertEqual(reply.malformed_lines, 2)

    def test_too_many_malformed_lines_end_the_exchange(self):
        stm, port, _ = stm_with({"MOV,F,30": [b"\xff\n"] * MAX_MALFORMED_LINES + [b"ACK\n"]})
        reply = stm.move("MOV,F,30")
        self.assertEqual(reply.status, "protocol")
        self.assertFalse(reply.ok)
        self.assertTrue(reply.pose_unknown)
        self.assertEqual(len(port.written), 1)

    def test_overlong_line_is_skipped_and_ack_still_arrives(self):
        stm, _, _ = stm_with({"MOV,F,30": [b"MM," + b"9" * (STM_MAX_RX_LINE_CHARS + 50) + b"\n", b"ACK\n"]})
        reply = stm.move("MOV,F,30")
        self.assertTrue(reply.ok)
        self.assertEqual(reply.malformed_lines, 1)

    def test_invalid_mov_is_rejected_before_the_wire(self):
        stm, port, _ = stm_with({})
        for bad in ("MOV,L,90", "MOV,F,1000", "MOV,F,30\n", "SNAP", "STOP"):
            with self.assertRaises(ValueError):
                stm.move(bad)
        self.assertEqual(port.written, [])

    def test_raw_command_text_errors_are_programmer_errors_not_replies(self):
        stm, port, _ = stm_with({})
        with self.assertRaises(UartCommandError):
            stm.query("IMU,GET\nMOV,F,99", lambda line: True)
        self.assertEqual(port.written, [])

    def test_stale_input_is_discarded_before_each_command(self):
        stm, port, _ = stm_with({"MOV,F,30": [b"ACK\n"]})
        port.feed(b"ACK\n", b"ERR,BUSY\n")                    # leftovers from before this command
        reply = stm.move("MOV,F,30")
        self.assertTrue(reply.ok)
        self.assertEqual(port.resets, 1)


class SetupTests(unittest.TestCase):
    def test_profile_set_then_verify_exact_exchange(self):
        stm, port, _ = stm_with(PROFILE)
        self.assertIsNone(stm.apply_profile(SETUP))
        self.assertEqual(port.written, [b"PROFILE,PRECISION\n", b"PROFILE,GET\n"])

    def test_standard_profile_exchange(self):
        stm, port, _ = stm_with({"PROFILE,STANDARD": [b"PROFILE,OK\n"], "PROFILE,GET": [b"PROFILE,STANDARD\n"]})
        self.assertIsNone(stm.apply_profile(STANDARD))
        self.assertEqual(port.written, [b"PROFILE,STANDARD\n", b"PROFILE,GET\n"])

    def test_profile_get_with_telemetry_and_fragments(self):
        stm, _, _ = stm_with({"PROFILE,PRECISION": [b"PROFILE", b",OK\r\n"],
                              "PROFILE,GET": [b"MM,x\n", b"PROFILE,PRECI", b"SION\n"]})
        self.assertIsNone(stm.apply_profile(SETUP))

    def test_failures_abort_setup(self):
        cases = {
            "custom": ({"PROFILE,PRECISION": [b"PROFILE,OK\n"], "PROFILE,GET": [b"PROFILE,CUSTOM\n"]}, 2),
            "wrong profile": ({"PROFILE,PRECISION": [b"PROFILE,OK\n"], "PROFILE,GET": [b"PROFILE,STANDARD\n"]}, 2),
            "busy on set": ({"PROFILE,PRECISION": [b"ERR,BUSY\n"]}, 1),
            "bad cmd on set": ({"PROFILE,PRECISION": [b"ERR,BAD_CMD\n"]}, 1),
            "busy on get": ({"PROFILE,PRECISION": [b"PROFILE,OK\n"], "PROFILE,GET": [b"ERR,BUSY\n"]}, 2),
            "bad cmd on get": ({"PROFILE,PRECISION": [b"PROFILE,OK\n"], "PROFILE,GET": [b"ERR,BAD_CMD\n"]}, 2),
            "no reply to set": ({}, 1),
            "no reply to get": ({"PROFILE,PRECISION": [b"PROFILE,OK\n"]}, 2),
            "set answered with a name": ({"PROFILE,PRECISION": [b"PROFILE,PRECISION\n"],
                                          "PROFILE,GET": [b"PROFILE,PRECISION\n"]}, 1),
            "unrelated line": ({"PROFILE,PRECISION": [b"ACK\n", b"DONE\n"]}, 1),
        }
        for name, (replies, writes) in cases.items():
            stm, port, _ = stm_with(replies)
            reason = stm.apply_profile(SETUP)
            self.assertIsNotNone(reason, name)
            self.assertEqual(len(port.written), writes, name)

    def test_link_failure_aborts_setup_with_a_reason(self):
        stm, port, _ = stm_with({})
        port.error_when_empty = OSError("gone")
        self.assertIn("UART disconnected", stm.apply_profile(SETUP))
        self.assertEqual(port.written, [b"PROFILE,PRECISION\n"])

    def test_only_the_documented_profile_exchanges_are_ever_sent(self):
        for setup in ({**SETUP, "set_command": "MOV,F,10"},
                      {**SETUP, "set_command": "PROFILE,CUSTOM", "expected_verify_reply": "PROFILE,CUSTOM"},
                      {**SETUP, "set_command": "PROFILE,precision"},
                      {**SETUP, "verify_command": "TURNSPEED,GET"},
                      {**SETUP, "expected_set_reply": "ACK"},
                      {**SETUP, "expected_verify_reply": "PROFILE,STANDARD"},
                      {"set_command": "PROFILE,PRECISION"}, None):
            stm, port, _ = stm_with(PROFILE)
            self.assertIsNotNone(stm.apply_profile(setup), setup)
            self.assertEqual(port.written, [], setup)

    def test_imu_query(self):
        stm, port, _ = stm_with({"IMU,GET": [b"IMU,Y:905,GZ:3,SIGN:+1\n"]})
        self.assertEqual(stm.heading(), 90.5)
        self.assertEqual(port.written, [b"IMU,GET\n"])
        stm, port, _ = stm_with({"IMU,GET": [b"ERR,IMU\n"]})
        self.assertIsNone(stm.heading())


class ConcurrencyAndShutdownTests(unittest.TestCase):
    def test_concurrent_callers_never_interleave_or_steal_replies(self):
        port = HandshakePort(replies={"MOV,F,10": [b"MM,EV:START,D:F,T:0,V:10\n", b"AC", b"K\n"],
                                      "MOV,B,10": [b"MM,EV:START,D:B,T:0,V:10\n", b"ERR,MOTION_TIMEOUT\n"]},
                             read_delay=0.001)
        stm = StmLink(UartTransport(port), log=lambda _: None)
        results, errors = [], []

        def worker(command):
            try:
                for _ in range(4):
                    results.append((command, stm.move(command)))
            except Exception as exc:       # noqa: BLE001
                errors.append(exc)

        threads = [threading.Thread(target=worker, args=(c,)) for c in ("MOV,F,10", "MOV,B,10") * 3]
        for t in threads:
            t.start()
        for t in threads:
            t.join(30)
        self.assertEqual(errors, [])
        self.assertEqual(port.violations, [])               # never two commands in flight
        self.assertEqual(len(results), 24)
        for command, reply in results:
            self.assertEqual(reply.line, "ACK" if command == "MOV,F,10" else "ERR,MOTION_TIMEOUT", command)
        written = [w.decode().strip() for w in port.written]
        self.assertEqual(sorted(written), sorted(c for c, _ in results))   # exactly one write per call

    def test_setup_and_move_from_two_threads_stay_separate(self):
        port = HandshakePort(replies={**PROFILE, "MOV,F,10": [b"ACK\n"]}, read_delay=0.001)
        stm = StmLink(UartTransport(port), log=lambda _: None)
        outcome = {}
        t1 = threading.Thread(target=lambda: outcome.setdefault("setup", stm.apply_profile(SETUP)))
        t2 = threading.Thread(target=lambda: outcome.setdefault("move", stm.move("MOV,F,10")))
        t1.start(); t2.start(); t1.join(10); t2.join(10)   # noqa: E702
        self.assertIsNone(outcome["setup"])
        self.assertTrue(outcome["move"].ok)
        written = [w.decode().strip() for w in port.written]
        if written[0] == "PROFILE,PRECISION":
            self.assertEqual(written[:2], ["PROFILE,PRECISION", "PROFILE,GET"])   # set+verify never split
        else:
            self.assertEqual(written, ["MOV,F,10", "PROFILE,PRECISION", "PROFILE,GET"])

    def check_shutdown_during_move(self, port):
        uart = UartTransport(port)
        stm = StmLink(uart, log=lambda _: None)
        holder = {}
        thread = threading.Thread(target=lambda: holder.setdefault("reply", stm.move("MOV,F,100")))
        thread.start()
        self.assertTrue(port.waiting.wait(2))
        uart.shutdown()
        thread.join(3)
        self.assertFalse(thread.is_alive())
        reply = holder["reply"]
        self.assertEqual(reply.status, "shutdown")
        self.assertTrue(reply.pose_unknown)
        self.assertEqual(port.written, [b"MOV,F,100\n"])    # shutdown sent nothing, nothing was retried
        after = stm.move("MOV,F,5")
        self.assertEqual(after.status, "shutdown")
        self.assertEqual(port.written, [b"MOV,F,100\n"])

    def test_shutdown_unblocks_a_waiting_move(self):
        self.check_shutdown_during_move(BlockingPort(timeout=10))

    def test_shutdown_unblocks_a_waiting_move_without_cancel_read(self):
        self.check_shutdown_during_move(BlockingPortNoCancel(timeout=0.05))


class CompatibilityTests(unittest.TestCase):
    def test_legacy_readline_port_still_works_under_the_transport(self):
        stm, port = link({"MOV,F,30": ["MM,EV:START", "ACK"]})
        reply = stm.move("MOV,F,30")
        self.assertTrue(reply.ok)
        self.assertEqual(port.written, ["MOV,F,30"])
        self.assertIsInstance(stm.uart, UartTransport)

    def test_pyserial_style_byte_port_is_accepted_directly(self):
        clock = ManualClock()
        port = ScriptedPort(clock, {"MOV,F,30": [b"ACK\n"]})
        stm = StmLink(port, log=lambda _: None, clock=clock)
        self.assertTrue(stm.move("MOV,F,30").ok)

    def test_reply_object_keeps_its_old_shape(self):
        reply = StmReply(status="ok", line="ACK")
        self.assertTrue(reply.ok)
        self.assertEqual((reply.motion_started, reply.telemetry), (False, []))

    def test_open_serial_keeps_its_signature_and_returns_a_closeable_transport(self):
        port = ScriptedPort()
        try:
            import serial  # noqa: F401
        except ImportError:
            self.skipTest("pyserial not installed")
        with mock.patch("serial.Serial", return_value=port) as ctor:
            uart = open_serial("/dev/ttyACM0")
        self.assertEqual(ctor.call_args.kwargs["baudrate"], 115200)
        uart.send_line("IMU,GET")
        self.assertEqual(port.written, [b"IMU,GET\n"])
        uart.close()
        self.assertTrue(port.closed)

    def test_task1_default_device_is_unchanged_and_by_id_is_documented(self):
        import task1
        with mock.patch.object(sys, "argv", ["task1.py", "--planner", "10.0.0.1"]):
            args = task1.parse_args()
        self.assertEqual(args.stm, "/dev/ttyACM0")
        with mock.patch.object(sys, "argv", ["task1.py", "--planner", "10.0.0.1", "--stm", "/dev/serial/by-id/x"]):
            self.assertEqual(task1.parse_args().stm, "/dev/serial/by-id/x")


def byte_mission(replies, plan=None, ids=None, port=None):
    clock = ManualClock()
    port = port or HandshakePort(clock, replies)
    port.clock = clock
    stm, _ = stm_on(port, clock)
    args = argparse.Namespace(profile="PRECISION", heading_fix=False, align=False, align_max_bursts=8,
                              align_threshold=5.0, align_turn=5, align_straight=8, align_max_forward=8.0,
                              align_distance=25.0, hfov=62.2)
    android = FakeAndroid()
    vision = FakeVision({3: "20", 5: "11"} if ids is None else ids)
    mission = Mission(args, android, stm, vision)
    mission.plan = json.loads(json.dumps(plan or PLAN))
    return mission, android, port, vision


MOVES = {**PROFILE,
         "MOV,F,30": [START, SAMPLE, b"AC", b"K\n"],
         "MOV,FR,90": [b"MM,EV:START,D:FR,T:0,V:90\n", b"ACK\n"]}


class MissionOverUartTests(unittest.TestCase):
    def test_snap_is_handled_locally_and_never_reaches_the_uart(self):
        mission, android, port, vision = byte_mission(MOVES)
        mission.execute()
        self.assertEqual(vision.submitted, [3, 5])           # both photos were taken by the RPi
        self.assertEqual(port.written, [b"PROFILE,PRECISION\n", b"PROFILE,GET\n", b"MOV,F,30\n", b"MOV,FR,90\n"])
        self.assertFalse(any(b"SNAP" in w for w in port.written))
        self.assertTrue(android.sent[-1].startswith("MSG,[Mission done"))

    def test_snap_only_plan_writes_no_motion_to_the_stm(self):
        plan = {"mission_id": "m2", "stm_setup": SETUP, "start_pose": PLAN["start_pose"],
                "actions": [{"seq": 0, "type": "SNAP", "obstacle_id": 3}, {"seq": 1, "type": "SNAP", "obstacle_id": 5}]}
        mission, _, port, vision = byte_mission(PROFILE, plan)
        mission.execute()
        self.assertEqual(vision.submitted, [3, 5])
        self.assertEqual(port.written, [b"PROFILE,PRECISION\n", b"PROFILE,GET\n"])   # setup only

    def test_next_mov_waits_for_the_previous_ack(self):
        plan = json.loads(json.dumps(PLAN))
        plan["actions"] = [a for a in plan["actions"] if a["type"] == "MOV"] + [
            {"seq": 4, "type": "MOV", "stm_command": "MOV,B,10", "start_pose": plan["actions"][2]["end_pose"],
             "end_pose": plan["actions"][2]["end_pose"]}]
        mission, android, port, _ = byte_mission({**MOVES, "MOV,B,10": [b"MM,EV:START,D:B,T:0,V:10\n", b"ACK\n"]},
                                                 plan)
        mission.execute()
        self.assertEqual(port.violations, [])
        self.assertEqual([w.decode().strip() for w in port.written[2:]], ["MOV,F,30", "MOV,FR,90", "MOV,B,10"])
        self.assertEqual(sum(s.startswith("ROBOT") for s in android.sent), 1 + 3)   # start pose + one per ACK

    def test_no_next_mov_after_a_timeout_and_no_resend(self):
        mission, android, port, _ = byte_mission({**PROFILE, "MOV,F,30": [START]})
        mission.execute()
        self.assertEqual([w.decode().strip() for w in port.written[2:]], ["MOV,F,30"])
        self.assertTrue(any("position unknown" in s for s in android.sent))
        self.assertEqual(sum(s.startswith("ROBOT") for s in android.sent), 1)      # no pose advance

    def test_no_next_mov_after_an_stm_error(self):
        mission, android, port, _ = byte_mission({**MOVES, "MOV,F,30": [START, b"ERR,MOTION_TIMEOUT\n"]})
        mission.execute()
        self.assertEqual([w.decode().strip() for w in port.written[2:]], ["MOV,F,30"])
        self.assertTrue(any("ERR,MOTION_TIMEOUT" in s and "position unknown" in s for s in android.sent))

    def test_disconnect_mid_mission_stops_cleanly_without_resend(self):
        port = HandshakePort(replies={**PROFILE, "MOV,F,30": [START]})
        mission, android, _, _ = byte_mission(None, port=port)
        port.error_when_empty = OSError("USB device unplugged")
        mission.execute()
        self.assertEqual([w.decode().strip() for w in port.written[2:]], ["MOV,F,30"])
        self.assertTrue(any("UART disconnected" in s and "position unknown" in s for s in android.sent))
        self.assertFalse(any(s.startswith("MSG,[Mission done") for s in android.sent))

    def test_profile_failure_sends_no_mov(self):
        for get_reply in (b"PROFILE,CUSTOM\n", b"PROFILE,STANDARD\n", b"ERR,BUSY\n", b""):
            mission, android, port, _ = byte_mission({"PROFILE,PRECISION": [b"PROFILE,OK\n"],
                                                      "PROFILE,GET": [get_reply] if get_reply else []})
            mission.execute()
            self.assertFalse(any(w.startswith(b"MOV") for w in port.written), get_reply)
            self.assertTrue(any(s.startswith("MSG,[Aborted before moving") for s in android.sent))

    def test_a_plan_is_not_sent_to_the_stm_as_a_batch(self):
        mission, _, port, _ = byte_mission(MOVES)
        mission.stm.apply_profile(SETUP)
        self.assertEqual(len(port.written), 2)               # arming the plan sends setup only, no MOV yet
        self.assertFalse(any(w.startswith(b"MOV") for w in port.written))


class LegacyFakeStillCoversMissionTests(unittest.TestCase):
    """The pre-uart.py FakeSerial/FakeClock fixtures keep working unchanged."""

    def test_fixtures_import_and_run(self):
        stm = StmLink(FakeSerial({"MOV,F,30": ["ACK"]}), log=lambda _: None, clock=FakeClock())
        self.assertTrue(stm.move("MOV,F,30").ok)


if __name__ == "__main__":
    unittest.main()
