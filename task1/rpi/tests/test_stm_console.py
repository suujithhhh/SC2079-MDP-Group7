"""stm_console.py with a fake port (no hardware). Run from rpi_task1/:
    python -m unittest discover -s tests -v
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import stm_console  # noqa: E402
from stm_console import HELP, is_telemetry, normalise, run_command  # noqa: E402
from uart import UartCommandError, UartTransport  # noqa: E402

from test_stm_uart import SAMPLE, START  # noqa: E402
from test_uart import ManualClock, ScriptedPort  # noqa: E402


def console(replies=None):
    clock = ManualClock()
    port = ScriptedPort(clock, replies)
    return UartTransport(port, clock=clock), port, clock


class ConsoleTests(unittest.TestCase):
    def test_normalise(self):
        self.assertEqual(normalise("mov,f,30"), "MOV,F,30")
        self.assertEqual(normalise("  spprof get "), "SPPROF,GET")
        self.assertEqual(normalise("stopctrl 120 1800 1"), "STOPCTRL,120,1800,1")
        self.assertEqual(normalise("mov , fl , 90"), "MOV,FL,90")
        self.assertEqual(normalise("   "), "")

    def test_mov_prints_everything_and_stops_at_ack(self):
        uart, port, _ = console({"MOV,F,30": [START, SAMPLE, b"ACK\n", b"LATE\n"]})
        shown = []
        lines = run_command(uart, "MOV,F,30", out=shown.append)
        self.assertEqual(port.written, [b"MOV,F,30\n"])
        self.assertEqual([ln for ln in lines], [START.decode().strip(), SAMPLE.decode().strip(), "ACK"])
        self.assertTrue(shown[2].endswith("ACK"))
        self.assertTrue(any("wheels L 80 R 81" in s for s in shown))      # one-line move summary

    def test_telemetry_can_be_hidden_but_is_still_returned(self):
        uart, _, _ = console({"MOV,F,30": [START, SAMPLE, b"ACK\n"]})
        shown = []
        lines = run_command(uart, "MOV,F,30", out=shown.append, show_telemetry=False)
        self.assertEqual(len(lines), 3)
        self.assertEqual([s.split()[-1] for s in shown if s.strip().endswith("ACK")], ["ACK"])
        self.assertFalse(any(is_telemetry(s.split(None, 1)[-1]) for s in shown if "s  " in s[:12]))

    def test_error_ends_a_move(self):
        uart, _, _ = console({"MOV,F,30": [b"ERR,BUSY\n"]})
        shown = []
        lines = run_command(uart, "MOV,F,30", out=shown.append)
        self.assertEqual(lines, ["ERR,BUSY"])

    def test_mov_timeout_is_reported_and_not_resent(self):
        uart, port, clock = console({"MOV,F,30": [START]})
        shown = []
        run_command(uart, "MOV,F,30", out=shown.append, clock=clock)
        self.assertEqual(port.written, [b"MOV,F,30\n"])
        self.assertTrue(any("NOT resent" in s for s in shown))

    def test_invalid_mov_is_not_sent(self):
        uart, port, _ = console()
        shown = []
        self.assertIsNone(run_command(uart, "MOV,F,1000", out=shown.append))
        self.assertIsNone(run_command(uart, "MOV,L,90", out=shown.append))
        self.assertEqual(port.written, [])

    def test_multi_line_reply_is_shown_whole_then_ends_on_silence(self):
        uart, port, clock = console({"PID,GET": [b"PID,L,1.0,0.0,0.0\n", b"PID,R,1.0,0.0,0.0\n",
                                                 b"PID,S,2.0,0.0,0.0\n"]})
        lines = run_command(uart, "PID,GET", out=lambda _: None, clock=clock)
        self.assertEqual(len(lines), 3)
        self.assertLess(clock.now, 2.0)                       # did not wait the full first-reply window

    def test_get_reply_for_tuning_commands(self):
        uart, port, clock = console({"SPPROF,GET": [b"SPPROF,D:4500,A:1200,E:1\n"]})
        self.assertEqual(run_command(uart, "SPPROF,GET", out=lambda _: None, clock=clock),
                         ["SPPROF,D:4500,A:1200,E:1"])
        self.assertEqual(port.written, [b"SPPROF,GET\n"])

    def test_no_reply_is_reported(self):
        uart, _, clock = console()
        shown = []
        self.assertEqual(run_command(uart, "SPPROF,GET", out=shown.append, clock=clock), [])
        self.assertIn("(no reply)", shown)

    def test_overlong_command_is_rejected_by_the_transport(self):
        uart, port, _ = console()
        with self.assertRaises(UartCommandError):
            run_command(uart, "X" * 40, out=lambda _: None)
        self.assertEqual(port.written, [])

    def test_stale_input_is_dropped_before_sending(self):
        uart, port, _ = console({"IMU,GET": [b"IMU,Y:10,GZ:0,SIGN:+1\n"]})
        port.feed(b"ACK\n")
        shown = []
        lines = run_command(uart, "IMU,GET", out=shown.append)
        self.assertEqual(lines, ["IMU,Y:10,GZ:0,SIGN:+1"])
        self.assertEqual(port.resets, 1)


STOPPED = b"ACK,STOPPED\n"
END_STOPPED = b"MM,EV:END,D:F,T:410,RESULT:STOPPED\n"
STOP_NEAR = b"USPARK,STOP,R:NEAR,D:195,V:310,FM:00\n"
FAULT = b"USPARK,FAULT,FM:04\n"
STOP_FAULT = b"USPARK,STOP,R:FAULT,D:NA,V:0,FM:04\n"
GET_LINE = b"USPARK,E:1,ST:ARMED,CN:0,FM:00,TH:200,RA:220,AR:400\n"
NEAR_MISSES = (b"ACK,STOPPED \n", b" ACK,STOPPED\n", b"ACK,STOP\n", b"ACK,STOPPED,1\n", b"ACK,\n", b"ACK \n",
               b"ack,stopped\n", b"ACK,Stopped\n", b"XACK,STOPPED\n", b"ACK,STOPPEDX\n", b"STOPPED\n",
               b"ACK:STOPPED\n", b"ACK, STOPPED\n")


def text(chunk):
    return chunk.decode().rstrip("\n")


class StopProtocolTests(unittest.TestCase):
    def test_mov_ending_with_ack_is_normal_success(self):
        uart, port, _ = console({"MOV,F,30": [START, SAMPLE, b"MM,EV:END,D:F,T:400,RESULT:OK\n", b"ACK\n", b"LATE\n"]})
        shown = []
        lines = run_command(uart, "MOV,F,30", out=shown.append)
        self.assertEqual(lines[-1], "ACK")
        self.assertNotIn("LATE", lines)
        self.assertEqual(port.written, [b"MOV,F,30\n"])
        self.assertFalse(any("STOPPED" in s for s in shown))     # no stop message for a normal completion
        self.assertFalse(any("TIMEOUT" in s for s in shown))

    def test_mov_ending_with_ack_stopped_is_terminal_immediately(self):
        uart, port, clock = console({"MOV,F,30": [START, SAMPLE, END_STOPPED, STOPPED, b"LATE\n"]})
        shown = []
        lines = run_command(uart, "MOV,F,30", out=shown.append, clock=clock)
        self.assertEqual(lines, [text(START), text(SAMPLE), text(END_STOPPED), "ACK,STOPPED"])
        self.assertLess(clock.now, 1.0)                          # nowhere near the 8.5 s+ MOV timeout
        self.assertEqual(port.written, [b"MOV,F,30\n"])          # not resent
        self.assertFalse(any("TIMEOUT" in s for s in shown))
        message = [s for s in shown if "MOVEMENT STOPPED" in s]
        self.assertEqual(len(message), 1)
        self.assertIn("MOV,F,30 did not complete", message[0])
        self.assertIn("must NOT be assumed", message[0])
        self.assertTrue(any("no USPARK,STOP cause line" in s for s in shown))     # manual stop: no cause

    def test_uspark_cause_lines_are_shown_before_ack_stopped(self):
        uart, port, clock = console({"MOV,F,30": [START, FAULT, STOP_FAULT, SAMPLE, END_STOPPED, STOPPED]})
        shown = []
        lines = run_command(uart, "MOV,F,30", out=shown.append, clock=clock)
        self.assertEqual(lines, [text(c) for c in (START, FAULT, STOP_FAULT, SAMPLE, END_STOPPED, STOPPED)])
        order = [next(i for i, s in enumerate(shown) if s.endswith(text(c)))
                 for c in (START, FAULT, STOP_FAULT, END_STOPPED, STOPPED)]
        self.assertEqual(order, sorted(order))                    # displayed in the order received
        self.assertTrue(any("automatic ultrasonic parking stop: " + text(STOP_FAULT) in s for s in shown))
        self.assertTrue(any("ultrasonic fault: " + text(FAULT) in s for s in shown))
        self.assertTrue(any("MOVEMENT STOPPED" in s for s in shown))
        self.assertEqual(port.written, [b"MOV,F,30\n"])

    def test_near_stop_cause_is_summarised(self):
        uart, _, clock = console({"MOV,F,30": [START, STOP_NEAR, END_STOPPED, STOPPED]})
        shown = []
        run_command(uart, "MOV,F,30", out=shown.append, clock=clock)
        self.assertTrue(any("automatic ultrasonic parking stop: " + text(STOP_NEAR) in s for s in shown))
        self.assertFalse(any("no USPARK,STOP cause line" in s for s in shown))

    def test_stop_sequence_is_visible_even_with_telemetry_hidden(self):
        uart, _, clock = console({"MOV,F,30": [START, SAMPLE, FAULT, STOP_NEAR, END_STOPPED, STOPPED]})
        shown = []
        lines = run_command(uart, "MOV,F,30", out=shown.append, show_telemetry=False, clock=clock)
        self.assertEqual(len(lines), 6)                           # everything is still returned
        joined = "\n".join(shown)
        self.assertIn(text(FAULT), joined)                        # USPARK lines are not telemetry: always shown
        self.assertIn(text(STOP_NEAR), joined)
        self.assertIn(text(END_STOPPED), joined)                  # hidden MM line is recapped with the stop message
        self.assertIn("MOVEMENT STOPPED", joined)
        self.assertNotIn(text(SAMPLE), joined)                    # ordinary samples stay hidden

    def test_near_miss_tokens_do_not_end_a_move(self):
        for line in NEAR_MISSES:
            uart, port, clock = console({"MOV,F,30": [START, line]})
            shown = []
            lines = run_command(uart, "MOV,F,30", out=shown.append, clock=clock)
            self.assertIn(text(line), lines, line)
            self.assertTrue(any("TIMEOUT" in s and "NOT resent" in s for s in shown), line)
            self.assertFalse(any("MOVEMENT STOPPED" in s for s in shown), line)
            self.assertEqual(port.written, [b"MOV,F,30\n"], line)

    def test_direct_stop_ends_at_ack_stopped_without_waiting(self):
        uart, port, clock = console({"STOP": [STOPPED, b"LATE\n"]})
        shown = []
        lines = run_command(uart, "STOP", out=shown.append, clock=clock)
        self.assertEqual(lines, ["ACK,STOPPED"])
        self.assertLess(clock.now, 1.0)                           # no 0.6 s silence window, no 3 s timeout
        self.assertEqual(port.written, [b"STOP\n"])
        self.assertTrue(any("STOPPED (ACK,STOPPED): STOP acknowledged" in s and "must NOT be assumed" in s
                            for s in shown))

    def test_direct_stop_shows_the_interrupted_moves_lines_first(self):
        uart, port, clock = console({"STOP": [SAMPLE, END_STOPPED, STOPPED]})
        shown = []
        lines = run_command(uart, "STOP", out=shown.append, clock=clock)
        self.assertEqual(lines, [text(SAMPLE), text(END_STOPPED), "ACK,STOPPED"])
        self.assertEqual(port.written, [b"STOP\n"])

    def test_a_crossing_ack_does_not_end_stop(self):
        # PROTOCOL.md 3.6: the earlier movement's own ACK may cross with STOP; keep reading to ACK,STOPPED
        uart, _, clock = console({"STOP": [b"ACK\n", STOPPED]})
        lines = run_command(uart, "STOP", out=lambda _: None, clock=clock)
        self.assertEqual(lines, ["ACK", "ACK,STOPPED"])

    def test_near_miss_tokens_do_not_end_stop(self):
        for line in NEAR_MISSES:
            uart, port, clock = console({"STOP": [line]})
            shown = []
            run_command(uart, "STOP", out=shown.append, clock=clock)
            self.assertTrue(any("TIMEOUT" in s and "NOT resent" in s for s in shown), line)
            self.assertFalse(any("STOP acknowledged" in s for s in shown), line)
            self.assertEqual(port.written, [b"STOP\n"], line)

    def test_stop_without_a_reply_is_reported_and_not_resent(self):
        uart, port, clock = console()
        shown = []
        lines = run_command(uart, "STOP", out=shown.append, clock=clock)
        self.assertEqual(lines, [])
        self.assertTrue(any("no ACK,STOPPED" in s and "NOT resent" in s for s in shown))
        self.assertEqual(port.written, [b"STOP\n"])

    def test_err_still_ends_stop_and_mov(self):
        for command in ("STOP", "MOV,F,30"):
            uart, port, clock = console({command: [b"ERR,BAD_CMD\n", b"LATE\n"]})
            self.assertEqual(run_command(uart, command, out=lambda _: None, clock=clock), ["ERR,BAD_CMD"])
            self.assertEqual(port.written, [command.encode() + b"\n"])

    def test_uspark_control_replies(self):
        cases = {"USPARK,1": [b"USPARK,OK\n"], "USPARK,0": [b"USPARK,OK\n"], "USPARK,GET": [GET_LINE]}
        for command, reply in cases.items():
            uart, port, clock = console({command: reply})
            shown = []
            lines = run_command(uart, command, out=shown.append, clock=clock)
            self.assertEqual(lines, [text(reply[0])], command)
            self.assertTrue(any(s.endswith(text(reply[0])) for s in shown), command)
            self.assertLess(clock.now, 2.0, command)
            self.assertEqual(port.written, [command.encode() + b"\n"], command)      # sent once
            self.assertFalse(any("STOPPED" in s for s in shown), command)

    def test_uspark_enable_errors_end_at_once_and_are_not_retried(self):
        for error in (b"ERR,BUSY\n", b"ERR,US_FAULT\n"):
            uart, port, clock = console({"USPARK,1": [error, b"USPARK,OK\n"]})
            lines = run_command(uart, "USPARK,1", out=lambda _: None, clock=clock)
            self.assertEqual(lines, [text(error)])
            self.assertLess(clock.now, 0.5)
            self.assertEqual(port.written, [b"USPARK,1\n"])

    def test_ack_stopped_means_nothing_for_other_commands(self):
        uart, port, clock = console({"IMU,GET": [STOPPED]})
        shown = []
        lines = run_command(uart, "IMU,GET", out=shown.append, clock=clock)
        self.assertEqual(lines, ["ACK,STOPPED"])                  # shown, finished by silence as before
        self.assertFalse(any("STOPPED (ACK" in s or "MOVEMENT" in s for s in shown))

    def test_plain_ack_still_ends_other_commands(self):
        uart, port, clock = console({"IMU,ZERO": [b"ACK\n", b"LATE\n"]})
        self.assertEqual(run_command(uart, "IMU,ZERO", out=lambda _: None, clock=clock), ["ACK"])

    def test_normalise_new_commands(self):
        self.assertEqual(normalise("stop"), "STOP")
        self.assertEqual(normalise(" uspark 1 "), "USPARK,1")
        self.assertEqual(normalise("uspark, 0"), "USPARK,0")
        self.assertEqual(normalise("uspark get"), "USPARK,GET")
        self.assertEqual(normalise("profile task2test"), "PROFILE,TASK2TEST")

    def test_help_lists_the_stop_and_uspark_commands_and_drops_the_old_claim(self):
        for entry in ("STOP", "USPARK,1", "USPARK,0", "USPARK,GET", "PROFILE,PRECISION|STANDARD|TASK2TEST|GET",
                      "ACK,STOPPED"):
            self.assertIn(entry, HELP)
        source = Path(stm_console.__file__).read_text()
        for outdated in ("no stop command", "no\nstop command"):
            self.assertNotIn(outdated, HELP)
            self.assertNotIn(outdated, stm_console.__doc__)
            self.assertNotIn(outdated, source)
        self.assertNotIn("There is no stop", source)


class DelayPort(ScriptedPort):
    """ScriptedPort whose reply list may contain floats: that many seconds of silence."""

    @property
    def in_waiting(self):
        return len(self.rx[0]) if self.rx and isinstance(self.rx[0], bytes) else 0

    def read(self, size):
        with self.lock:
            if self.rx and isinstance(self.rx[0], float):
                self.clock.advance(self.rx.popleft())
                return b""
        return super().read(size)


TRACE_LINE = b"USPARK,TRACE,EV:PRI,N:1,T:12570,SEQ:7,PT:NA,PQ:NA,RAW:900,FS:1,FI:900,K:0,H:0009\n"
TRACE_DONE = b"USPARK,TRACE,EV:DONE,N:20,T:13100,W:1,R:OK,DR:0,RF:0,ZT:0,SK:0\n"


class TraceDrainTests(unittest.TestCase):
    """After an automatic parking stop the trace follows ACK,STOPPED. A firmware with the shadow
    predictor sends its shadow block first, so EV:DONE can arrive about 6 s after ACK,STOPPED."""

    def run_stream(self, items):
        clock = ManualClock()
        port = DelayPort(clock, {"MOV,F,30": items})
        uart = UartTransport(port, clock=clock)
        shown = []
        lines = run_command(uart, "MOV,F,30", out=shown.append, clock=clock)
        return lines, shown, port, clock

    def test_the_drain_bound_is_ten_seconds(self):
        self.assertEqual(stm_console.TRACE_DRAIN_S, 10.0)
        self.assertNotIn("5 s after", stm_console.__doc__)

    def test_a_trace_that_ends_about_six_seconds_after_ack_stopped_is_complete(self):
        lines, shown, port, clock = self.run_stream(
            [START, STOP_NEAR, END_STOPPED, STOPPED, 4.0, TRACE_LINE, 2.0, TRACE_LINE, TRACE_DONE, b"LATE\n"])
        self.assertEqual(lines[-1], text(TRACE_DONE))
        self.assertNotIn("LATE", lines)
        self.assertGreaterEqual(clock.now, 6.0)
        self.assertLess(clock.now, 7.0)
        self.assertTrue(any("USPARK trace complete (3 lines, through EV:DONE)" in s for s in shown))
        self.assertFalse(any("WARNING" in s for s in shown))
        self.assertEqual(port.written, [b"MOV,F,30\n"])          # nothing was sent while draining

    def test_done_nine_seconds_after_ack_stopped_is_still_collected(self):
        lines, shown, port, _ = self.run_stream([START, STOP_NEAR, END_STOPPED, STOPPED, 9.0, TRACE_DONE])
        self.assertEqual(lines[-1], text(TRACE_DONE))
        self.assertFalse(any("WARNING" in s for s in shown))
        self.assertEqual(port.written, [b"MOV,F,30\n"])

    def test_no_done_within_ten_seconds_warns_and_sends_nothing(self):
        # the DONE line arrives 12 s after ACK,STOPPED (24 silences of 0.5 s): too late
        lines, shown, port, clock = self.run_stream(
            [START, STOP_NEAR, END_STOPPED, STOPPED, TRACE_LINE] + [0.5] * 24 + [TRACE_DONE])
        self.assertNotIn(text(TRACE_DONE), lines)
        self.assertTrue(any("WARNING: USPARK trace incomplete" in s and "within 10.0 s" in s for s in shown))
        self.assertGreaterEqual(clock.now, 10.0)
        self.assertLess(clock.now, 10.6)
        self.assertEqual(port.written, [b"MOV,F,30\n"])          # no resend, no STOP

    def test_a_manual_stop_still_ends_at_ack_stopped(self):
        lines, shown, port, clock = self.run_stream([START, END_STOPPED, STOPPED, TRACE_LINE, TRACE_DONE])
        self.assertEqual(lines[-1], "ACK,STOPPED")
        self.assertLess(clock.now, 1.0)
        self.assertFalse(any("collecting the USPARK trace" in s for s in shown))


if __name__ == "__main__":
    unittest.main()
