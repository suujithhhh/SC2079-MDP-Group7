"""Phase 3a tests: generic STM transport hardening. Run from the task2 folder:

    python3 -m unittest discover -s tests -v

These tests cover ONLY lines frozen in the checked-out PROTOCOL.md. They do not cover STOP, `ACK,STOPPED`,
IR, ultrasonic, health, READY/START or encoder travel: those are Phase 3b decisions (plan section 12). They
prove the Pi's serial transport, not the robot. The fake port and clock are deterministic (sim.py).
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

import sim
import stm_link
from stm_link import StmLink
from stm_protocol import (LineKind, LinkBusyError, LinkClosedError, LinkRefusedError, LinkState, MotionOutcomeUnknownError, MotionResult, classify_line,
                          pose_known_after)
from stm_task2 import Task2Stm

PROTOCOL_MD = (Path(__file__).resolve().parents[3] / "PROTOCOL.md").read_text(encoding="utf-8")
MOVE = "MOV,F,10"


def rig(log=None, **port_kwargs):
    clock = sim.FakeClock()
    port = sim.FakeSerialPort(clock, **port_kwargs)
    link = StmLink(port, log=log or (lambda _: None), clock=clock, sleep=clock.advance)
    return link, port, clock


def frozen_token(text: str) -> str:
    """The part of a scripted line that PROTOCOL.md names: a whole line for `ACK`, `ERR,<code>` and
    `PROFILE,<reply>`, the event marker for `MM,EV:`, and the family prefix for other `MM,` telemetry."""
    if text.startswith("MM,EV:START"):
        return "MM,EV:START"
    if text.startswith("MM,EV:END"):
        return "MM,EV:END"
    if text.startswith("MM,"):
        return "MM,"
    return text


def query_timeout(link: StmLink) -> None:
    """A query-only timeout: no movement was ever started, so nothing latches."""
    reply = link.query("PROFILE,GET", lambda line: line.startswith("PROFILE,"), timeout_s=1.0)
    assert reply.status == "timeout", reply


def kinds(link: StmLink) -> list[str]:
    return [event.kind for event in link.evidence]


class ClassifierTest(unittest.TestCase):
    def test_exact_ack_and_error(self):
        self.assertIs(classify_line("ACK", 0.0).kind, LineKind.ACK)
        for text in ("ERR,BUSY", "ERR,BAD_CMD", "ERR,MOTION_TIMEOUT", "ERR,IMU_NOT_READY", "ERR,PID_NOT_SET"):
            self.assertIs(classify_line(text, 0.0).kind, LineKind.ERROR, text)

    def test_ack_prefixed_text_is_not_an_exact_ack(self):
        for text in ("ACK,STOPPED", "ACK,", "ACKNOWLEDGE", "ACK 1", "ACK1", "ack", "Ack", "NACK", "ACK;", "XACK"):
            self.assertIs(classify_line(text, 0.0).kind, LineKind.UNKNOWN, text)

    def test_malformed_error_lines_are_not_replies(self):
        for text in ("ERR", "ERR,", "ERROR,BUSY", "err,BUSY", " ERR,BUSY"):
            self.assertIsNot(classify_line(text, 0.0).kind, LineKind.ERROR, text)

    def test_frozen_telemetry_families(self):
        for prefix in ("MM,", "AK,", "TB,", "MD,", "MS,", "HS,", "RTG,", "MP,", "SPC,", "STP,", "SFF,", "TL,",
                       "TR,", "TS,"):
            self.assertIs(classify_line(prefix + "X:1", 0.0).kind, LineKind.TELEMETRY, prefix)
        for text in ("MM,EV:START,D:F,T:0,V:10", "MM,EV:END,D:F,T:1200,RESULT:OK"):
            self.assertIs(classify_line(text, 0.0).kind, LineKind.MOTION_EVENT, text)

    def test_setup_replies_and_unknown_lines(self):
        for text in ("PROFILE,OK", "PROFILE,PRECISION", "PROFILE,CUSTOM", "IMU,Y:900,GZ:0"):
            self.assertIs(classify_line(text, 0.0).kind, LineKind.QUERY_REPLY, text)
        for text in ("IR,S:7,L:NA,R:120,F:00", "STOP", "READY", "garbage", "\x00\x01", "MOVE,F"):
            self.assertIs(classify_line(text, 0.0).kind, LineKind.UNKNOWN, text)

    def test_the_classifier_carries_no_unfrozen_task2_spelling(self):
        source = (Path(stm_link.__file__).parent / "stm_protocol.py").read_text(encoding="utf-8")
        code = re.sub(r'"""[\s\S]*?"""', "", source)          # docstrings may name what is NOT implemented
        for forbidden in ("STOPPED", "FIRL", "FIRR", "IR_NO", "IREDGE", "USPARK", '"READY"', "IR,GET", '"STOP"'):
            self.assertNotIn(forbidden, code, forbidden)


class FrozenFakePortTest(unittest.TestCase):
    def test_the_generic_fake_speaks_only_frozen_lines(self):
        link, port, _ = rig()
        port.script(MOVE, sim.ordinary_move_lines("F", 10))
        port.script("MOV,FL,90", sim.ordinary_move_lines("FL", 90))
        port.script("MOV,B,5", ["ERR,BUSY"])
        port.script("PROFILE,PRECISION", ["PROFILE,OK"])
        port.script("PROFILE,GET", ["PROFILE,PRECISION"])
        for command in (MOVE, "MOV,FL,90", "MOV,B,5"):
            link.send_motion(command)
        link.query("PROFILE,PRECISION", lambda line: line.startswith("PROFILE,"))
        link.query("PROFILE,GET", lambda line: line.startswith("PROFILE,"))
        self.assertTrue(port.emitted)
        for text in port.emitted:
            self.assertIsNot(classify_line(text, 0.0).kind, LineKind.UNKNOWN, text)
            self.assertIn(frozen_token(text), PROTOCOL_MD, text)
        for command in port.commands_written():
            self.assertIn(command.split(",")[0] + ",", PROTOCOL_MD, command)


class SendingTest(unittest.TestCase):
    def test_nothing_ever_clears_the_input_buffer(self):
        link, port, _ = rig()
        port.script(MOVE, sim.ordinary_move_lines())
        port.script("PROFILE,GET", ["PROFILE,PRECISION"])
        link.send_motion(MOVE)
        link.query("PROFILE,GET", lambda line: line.startswith("PROFILE,"))
        port.feed("MM,D:F,T:9")
        self.assertTrue(link.resync(0.2, 1.0))
        self.assertEqual(port.input_clears, 0)
        source = Path(stm_link.__file__).read_text(encoding="utf-8")
        self.assertNotIn("reset_input_buffer", source, "the link has no way to clear input at all")

    def test_exact_ack_completes_an_ordinary_motion(self):
        link, port, _ = rig()
        port.script(MOVE, sim.ordinary_move_lines())
        result = link.send_motion(MOVE)
        self.assertIsInstance(result, MotionResult)
        self.assertTrue(result.ok)
        self.assertEqual(result.final.text, "ACK")
        self.assertTrue(result.motion_started)
        self.assertFalse(result.timed_out)
        self.assertTrue(result.pose_known)
        self.assertIs(link.state, LinkState.IDLE)
        self.assertEqual(port.commands_written(), [MOVE])

    def test_a_reply_already_buffered_behind_telemetry_is_not_lost(self):
        link, port, _ = rig()
        port.script(MOVE, sim.ordinary_move_lines() + [])       # the whole answer is in the buffer at once
        result = link.send_motion(MOVE)
        self.assertEqual(result.final.text, "ACK")
        self.assertEqual(len(result.telemetry), 3, "START, one MM, END")
        self.assertEqual(port.input_clears, 0)

    def test_a_stray_reply_waiting_before_a_write_is_kept_as_evidence_not_dropped(self):
        link, port, _ = rig()
        port.feed("ACK")                                          # nobody asked for it
        with self.assertRaises(LinkRefusedError):
            link.send_motion(MOVE)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertEqual(port.commands_written(), [], "nothing was written")
        self.assertIn("unexpected_terminal", kinds(link))
        self.assertTrue(any("ACK" in e.text for e in link.evidence))
        self.assertEqual(port.input_clears, 0)

    def test_telemetry_never_completes_a_motion(self):
        link, port, _ = rig()
        families = ["MM,D:F,T:1", "AK,T:1", "TB,T:1", "MD,T:1", "MS,T:1", "HS,T:1", "RTG,T:1", "MP,T:1",
                    "SPC,T:1", "STP,T:1", "SFF,T:1", "TL,T:1", "TR,T:1", "TS,T:1",
                    "MM,EV:START,D:F,T:0,V:10", "MM,EV:END,D:F,T:1200,RESULT:OK"]
        port.script(MOVE, families)
        result = link.send_motion(MOVE)
        self.assertIsNone(result.final)
        self.assertTrue(result.timed_out)
        self.assertEqual([line.text for line in result.telemetry], families)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)

    def test_telemetry_before_the_real_ack_does_not_end_it_early(self):
        link, port, _ = rig()
        port.script(MOVE, ["MM,D:F,T:1", "TB,T:1", "MM,EV:END,D:F,T:1200,RESULT:OK", "ACK"])
        result = link.send_motion(MOVE)
        self.assertEqual(result.final.text, "ACK")
        self.assertEqual(len(result.telemetry), 3)

    def test_ack_prefixed_text_never_completes_a_motion(self):
        for text in ("ACK,STOPPED", "ACKNOWLEDGE", "ACK,", "ack"):
            with self.subTest(text):
                link, port, _ = rig()
                port.script(MOVE, ["MM,EV:START,D:F,T:0,V:10", text])
                result = link.send_motion(MOVE)
                self.assertIsNone(result.final)
                self.assertTrue(result.timed_out)
                self.assertFalse(result.pose_known)
                self.assertIn(text, [line.text for line in result.telemetry])

    def test_unknown_lines_are_logged_and_do_not_complete_a_motion(self):
        link, port, _ = rig()
        port.script(MOVE, ["MM,EV:START,D:F,T:0,V:10", "garbage", "IR,S:7,L:NA,R:120,F:00", "ACK"])
        result = link.send_motion(MOVE)
        self.assertEqual(result.final.text, "ACK", "only the exact ACK completes it")
        unknown = [line for line in result.telemetry if line.kind is LineKind.UNKNOWN]
        self.assertEqual([line.text for line in unknown], ["garbage", "IR,S:7,L:NA,R:120,F:00"])
        self.assertEqual(kinds(link).count("unknown"), 2)

    def test_error_before_start_means_nothing_moved_after_start_means_pose_unknown(self):
        link, port, _ = rig()
        port.script(MOVE, ["ERR,BUSY"])
        before = link.send_motion(MOVE)
        self.assertFalse(before.motion_started)
        self.assertTrue(before.pose_known)
        self.assertEqual(before.final.kind, LineKind.ERROR)
        port.script(MOVE, ["MM,EV:START,D:F,T:0,V:10", "ERR,MOTION_TIMEOUT"])
        after = link.send_motion(MOVE)
        self.assertTrue(after.motion_started)
        self.assertFalse(after.pose_known)
        self.assertIs(link.state, LinkState.IDLE, "a terminal ERR is an attributable outcome")

    def test_invalid_commands_are_refused_before_anything_is_written(self):
        link, port, _ = rig()
        for command in ("MOVE,F,10", "MOV,F,1000", "STOP", "MOV,FIRL,10", "MOV,F,-1", ""):
            with self.assertRaises(ValueError, msg=command):
                link.send_motion(command)
        self.assertEqual(port.written, [])
        self.assertIs(link.state, LinkState.IDLE)


class OneCommandAtATimeTest(unittest.TestCase):
    def test_a_second_motion_is_refused_while_one_is_pending(self):
        link, port, _ = rig()
        seen = []

        def second():
            if not seen:
                seen.append(True)
                with self.assertRaises(LinkBusyError):
                    link.send_motion("MOV,B,5")
                self.assertIs(link.state, LinkState.MOTION_PENDING)

        port.script(MOVE, sim.ordinary_move_lines())
        port.on_read = second
        result = link.send_motion(MOVE)
        self.assertTrue(seen and result.ok)
        self.assertEqual(port.commands_written(), [MOVE], "the refused motion was never written")

    def test_a_query_is_refused_while_a_motion_is_pending(self):
        link, port, _ = rig()
        seen = []

        def ask():
            if not seen:
                seen.append(True)
                with self.assertRaises(LinkBusyError):
                    link.query("PROFILE,GET", lambda line: True)
                with self.assertRaises(LinkBusyError):
                    link.resync(0.1, 0.5)

        port.script(MOVE, sim.ordinary_move_lines())
        port.on_read = ask
        self.assertTrue(link.send_motion(MOVE).ok)
        self.assertEqual(port.commands_written(), [MOVE])

    def test_a_motion_is_refused_while_a_query_is_pending(self):
        link, port, _ = rig()
        seen = []

        def move():
            if not seen:
                seen.append(True)
                with self.assertRaises(LinkBusyError):
                    link.send_motion(MOVE)

        port.script("PROFILE,GET", ["PROFILE,PRECISION"])
        port.on_read = move
        self.assertTrue(link.query("PROFILE,GET", lambda line: line.startswith("PROFILE,")).ok)
        self.assertEqual(port.commands_written(), ["PROFILE,GET"])


class QueryTest(unittest.TestCase):
    def test_query_ok_error_and_telemetry_handling(self):
        link, port, _ = rig()
        port.script("PROFILE,GET", ["MM,D:F,T:1", "PROFILE,STANDARD"])
        reply = link.query("PROFILE,GET", lambda line: line.startswith("PROFILE,"))
        self.assertTrue(reply.ok)
        self.assertEqual(reply.line, "PROFILE,STANDARD")
        self.assertEqual(reply.telemetry, ["MM,D:F,T:1"])
        port.script("PROFILE,PRECISION", ["ERR,BUSY"])
        reply = link.query("PROFILE,PRECISION", lambda line: line.startswith("PROFILE,"))
        self.assertEqual((reply.status, reply.line), ("error", "ERR,BUSY"))
        self.assertIs(link.state, LinkState.IDLE)

    def test_a_query_timeout_cannot_be_attributed_and_desynchronises(self):
        link, port, _ = rig()
        reply = link.query("PROFILE,GET", lambda line: line.startswith("PROFILE,"), timeout_s=1.0)
        self.assertEqual(reply.status, "timeout")
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertEqual(link.abandoned, "PROFILE,GET")

    def test_a_stray_ack_during_a_query_desynchronises(self):
        link, port, _ = rig()
        port.script("PROFILE,GET", ["ACK"])
        reply = link.query("PROFILE,GET", lambda line: line.startswith("PROFILE,"))
        self.assertEqual(reply.status, "desynchronized")
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)

    def test_apply_profile_goes_through_the_hardened_link(self):
        link, port, _ = rig()
        port.script("PROFILE,PRECISION", ["PROFILE,OK"])
        port.script("PROFILE,GET", ["PROFILE,PRECISION"])
        stm = Task2Stm(link)
        self.assertIsNone(stm.apply_profile("PRECISION"))
        link2, port2, _ = rig()
        port2.script("PROFILE,PRECISION", ["PROFILE,OK"])
        port2.script("PROFILE,GET", ["PROFILE,CUSTOM"])
        self.assertIn("expected PROFILE,PRECISION", Task2Stm(link2).apply_profile("PRECISION"))


class TimeoutAndDesyncTest(unittest.TestCase):
    def timed_out(self):
        link, port, clock = rig()
        result = link.send_motion(MOVE)             # the robot never answers
        return link, port, clock, result

    def test_a_motion_timeout_enters_desynchronized_with_the_pose_unknown(self):
        link, port, clock, result = self.timed_out()
        self.assertTrue(result.timed_out)
        self.assertIsNone(result.final)
        self.assertFalse(result.motion_started, "no START was seen, yet the pose is still unknown")
        self.assertFalse(result.pose_known)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertTrue(link.motion_outcome_unknown, "a movement timeout latches")
        self.assertEqual(link.abandoned, MOVE)
        self.assertGreaterEqual(clock.now, stm_link.mov_timeout_s(MOVE))
        self.assertIn("timeout", kinds(link))

    def test_the_movement_is_never_resent(self):
        link, port, _, _ = self.timed_out()
        for _ in range(3):
            with self.assertRaises(LinkRefusedError):
                link.send_motion(MOVE)
        self.assertEqual(port.commands_written(), [MOVE])

    def test_every_motion_and_query_is_refused_while_desynchronized(self):
        link, port, _, _ = self.timed_out()
        for command in (MOVE, "MOV,B,5", "MOV,FL,90"):
            with self.assertRaises(LinkRefusedError):
                link.send_motion(command)
        with self.assertRaises(LinkRefusedError):
            link.query("PROFILE,GET", lambda line: True)
        self.assertIsNone(link.heading(), "the heading query is refused too")
        self.assertIn("refused", Task2Stm(link).apply_profile("PRECISION"))
        self.assertEqual(port.commands_written(), [MOVE])

    def test_a_late_ack_cannot_complete_a_newer_command_nor_restore_normal_operation(self):
        link, port, _, _ = self.timed_out()
        port.feed("ACK")                                            # the old move answers, far too late
        port.script("MOV,B,5", sim.ordinary_move_lines("B", 5))
        with self.assertRaises(LinkRefusedError):
            link.send_motion("MOV,B,5")
        self.assertIs(link.state, LinkState.DESYNCHRONIZED, "a late reply restores nothing")
        self.assertEqual(port.commands_written(), [MOVE], "the newer command was never written")
        self.assertEqual(port.in_waiting, len(b"ACK\n"), "the late ACK is still waiting, unconsumed")

    def test_resync_logs_the_late_reply_against_the_timed_out_command_but_stays_latched(self):
        link, port, _, _ = self.timed_out()
        port.feed("MM,EV:END,D:F,T:9000,RESULT:TIMEOUT")
        port.feed("ERR,MOTION_TIMEOUT")
        self.assertFalse(link.resync(0.5, 3.0), "a late reply does not show the robot has stopped")
        late = [e for e in link.evidence if e.kind == "late_terminal"]
        self.assertEqual(len(late), 1)
        self.assertIn("ERR,MOTION_TIMEOUT", late[0].text)
        self.assertIn(MOVE, late[0].text)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertTrue(link.motion_outcome_unknown)
        self.assertEqual(link.abandoned, MOVE)
        port.script("MOV,B,5", sim.ordinary_move_lines("B", 5))
        with self.assertRaises(MotionOutcomeUnknownError):
            link.send_motion("MOV,B,5")
        self.assertEqual(port.commands_written(), [MOVE])

    def test_a_late_reply_after_a_query_resync_is_caught_before_the_next_write(self):
        link, port, _ = rig()
        query_timeout(link)
        self.assertTrue(link.resync(0.5, 3.0))
        port.feed("ACK")                                            # arrives after the quiet interval
        with self.assertRaises(LinkRefusedError) as caught:
            link.send_motion(MOVE)
        self.assertNotIsInstance(caught.exception, MotionOutcomeUnknownError, "no movement was ever started")
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertFalse(link.motion_outcome_unknown)
        self.assertEqual(port.commands_written(), ["PROFILE,GET"])

    def test_an_unknown_ack_like_line_does_not_end_a_pending_motion_and_times_it_out(self):
        link, port, _ = rig()
        port.script(MOVE, ["MM,EV:START,D:F,T:0,V:10", "ACK,STOPPED"])
        result = link.send_motion(MOVE)
        self.assertTrue(result.timed_out)
        self.assertFalse(result.pose_known)
        with self.assertRaises(LinkRefusedError):
            link.send_motion("MOV,B,5")


class ResyncTest(unittest.TestCase):
    def test_resync_is_bounded_and_needs_a_quiet_interval(self):
        link, port, clock = rig()
        query_timeout(link)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)

        def chatter():                                           # a line on every read: never quiet
            port.feed("MM,D:F,T:1")
            clock.advance(0.05)

        port.on_read = chatter
        before = clock.now
        self.assertFalse(link.resync(0.5, 2.0))
        self.assertIs(link.state, LinkState.DESYNCHRONIZED, "still refused")
        self.assertLessEqual(clock.now - before, 2.5, "bounded by max_s")
        self.assertIn("resync_failed", kinds(link))
        with self.assertRaises(LinkRefusedError):
            link.send_motion(MOVE)

    def test_quiet_resync_from_idle_and_after_a_query_timeout(self):
        link, port, clock = rig()
        before = clock.now
        self.assertTrue(link.resync(0.5, 3.0))
        self.assertGreaterEqual(clock.now - before, 0.5, "a documented quiet interval, not an instant answer")
        query_timeout(link)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertTrue(link.resync(0.5, 3.0))
        self.assertIs(link.state, LinkState.IDLE)
        self.assertIsNone(link.abandoned)
        port.script(MOVE, sim.ordinary_move_lines())
        self.assertTrue(link.send_motion(MOVE).ok)

    def test_resync_validates_its_bounds(self):
        link, _, _ = rig()
        for quiet, limit in ((0, 1), (-1, 1), (2, 1), (None, 1), ("a", 1)):
            with self.assertRaises(ValueError, msg=(quiet, limit)):
                link.resync(quiet, limit)

    def test_resync_does_not_clear_and_keeps_every_line_as_evidence(self):
        link, port, _ = rig()
        query_timeout(link)
        for text in ("MM,D:F,T:1", "garbage", "ACK"):
            port.feed(text)
        self.assertTrue(link.resync(0.5, 3.0))
        self.assertEqual(port.input_clears, 0)
        self.assertEqual(port.in_waiting, 0, "every waiting line was read")
        self.assertIn("unknown", kinds(link))
        self.assertIn("late_terminal", kinds(link))

    def test_resync_refuses_when_closed(self):
        link, _, _ = rig()
        link.close()
        with self.assertRaises(LinkClosedError):
            link.resync(0.1, 0.5)


class ReconnectTest(unittest.TestCase):
    def test_reconnect_after_a_query_only_desync_starts_clean_without_discarding_evidence(self):
        link, port, clock = rig()
        query_timeout(link)
        port.feed("ACK")
        old_evidence = link.evidence
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        port2 = sim.FakeSerialPort(clock)
        link.reconnect(port2)
        self.assertIs(link.state, LinkState.IDLE)
        self.assertIsNone(link.abandoned)
        self.assertEqual(link.evidence[:len(old_evidence)], old_evidence, "nothing was discarded")
        self.assertEqual(link.evidence[-1].kind, "reconnect")
        self.assertIn("PROFILE,GET", link.evidence[-1].text, "the abandoned command is recorded")
        port2.script("MOV,B,5", sim.ordinary_move_lines("B", 5))
        self.assertTrue(link.send_motion("MOV,B,5").ok)
        self.assertEqual(port2.commands_written(), ["MOV,B,5"])
        self.assertEqual(port.commands_written(), ["PROFILE,GET"], "the old port is not written again")


class UnknownMotionOutcomeLatchTest(unittest.TestCase):
    """A quiet line proves nothing about a robot that may still be moving (review correction 2)."""

    def latched(self):
        link, port, clock = rig()
        link.send_motion(MOVE)                       # the robot never answers
        return link, port, clock

    def assert_everything_refused(self, link, port, written):
        for command in (MOVE, "MOV,B,5", "MOV,FL,90"):
            with self.assertRaises(LinkRefusedError, msg=command):
                link.send_motion(command)
        with self.assertRaises(LinkRefusedError):
            link.query("PROFILE,GET", lambda line: True)
        self.assertIsNone(link.heading())
        self.assertIn("refused", Task2Stm(link).apply_profile("PRECISION"))
        self.assertEqual(port.commands_written(), written, "nothing further was written")

    def test_1_a_motion_timeout_sets_the_latch(self):
        link, port, _ = self.latched()
        self.assertTrue(link.motion_outcome_unknown)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertIn("motion_outcome_unknown", kinds(link))
        self.assertEqual(link.abandoned, MOVE)

    def test_2_a_quiet_resync_cannot_unlock_the_link(self):
        link, port, clock = self.latched()
        for _ in range(3):
            self.assertFalse(link.resync(0.5, 3.0), "a perfectly quiet line is not a stopped robot")
            self.assertIs(link.state, LinkState.DESYNCHRONIZED)
            self.assertTrue(link.motion_outcome_unknown)
            self.assertEqual(link.abandoned, MOVE)
        self.assertIn("resync_blocked", kinds(link))
        self.assert_everything_refused(link, port, [MOVE])

    def test_2b_a_late_reply_during_resync_is_evidence_only(self):
        link, port, _ = self.latched()
        for text in ("MM,EV:END,D:F,T:9000,RESULT:OK", "ACK", "garbage"):
            port.feed(text)
        self.assertFalse(link.resync(0.5, 3.0))
        self.assertEqual(port.in_waiting, 0, "everything was read and logged")
        self.assertIn("late_terminal", kinds(link))
        self.assertIn("unknown", kinds(link))
        self.assertTrue(link.motion_outcome_unknown)
        self.assert_everything_refused(link, port, [MOVE])

    def test_3_reconnect_cannot_unlock_the_same_link(self):
        link, port, clock = self.latched()
        old_evidence = link.evidence
        port2 = sim.FakeSerialPort(clock)
        link.reconnect(port2)
        self.assertTrue(link.motion_outcome_unknown)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertEqual(link.abandoned, MOVE)
        self.assertEqual(link.evidence[:len(old_evidence)], old_evidence, "evidence preserved")
        self.assertIn("STILL UNKNOWN", link.evidence[-1].text)
        self.assertFalse(link.resync(0.5, 3.0), "and a quiet new port does not unlock it either")
        self.assert_everything_refused(link, port2, [])
        self.assertEqual(port.commands_written(), [MOVE])

    def test_4_later_motion_and_query_requests_stay_rejected_with_the_specific_error(self):
        link, port, clock = self.latched()
        link.resync(0.5, 3.0)
        link.reconnect(sim.FakeSerialPort(clock))
        with self.assertRaises(MotionOutcomeUnknownError):
            link.send_motion(MOVE)
        with self.assertRaises(MotionOutcomeUnknownError):
            link.query("PROFILE,GET", lambda line: True)

    def test_there_is_no_way_to_clear_the_latch_in_phase_3a(self):
        link, _, _ = self.latched()
        with self.assertRaises(AttributeError):
            link.motion_outcome_unknown = False
        public = [name for name in dir(StmLink) if not name.startswith("_")]
        for name in public:
            self.assertFalse(any(word in name.lower() for word in ("clear", "unlatch", "acknowledge", "confirm", "stopped")),
                             f"{name} looks like a way to clear the latch")
        # The STOP procedure (PROTOCOL.md §6.2.3) is the only thing that touches the latch, and even a
        # confirmed STOP holds the motion gate for the life of the link: no movement is ever admitted again.
        self.assertEqual(sorted(public), sorted(["abandoned", "apply_profile", "close", "emergency_stop", "evidence",
                                                  "heading", "listen", "motion_gate_held", "motion_outcome_unknown",
                                                  "move", "note", "query", "reconnect", "request_stop", "resync",
                                                  "send_motion", "send_parking_motion", "send_sensor_motion",
                                                  "state", "stop_requested"]))

    def test_5_a_query_timeout_can_still_be_resynchronised_per_d51(self):
        link, port, _ = rig()
        query_timeout(link)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertFalse(link.motion_outcome_unknown, "no movement was initiated")
        self.assertTrue(link.resync(0.5, 3.0))
        self.assertIs(link.state, LinkState.IDLE)
        port.script(MOVE, sim.ordinary_move_lines())
        self.assertTrue(link.send_motion(MOVE).ok)

    def test_6_a_read_failure_while_a_movement_is_pending_gets_the_same_treatment(self):
        link, port, clock = rig()
        port.fail_reads = True
        with self.assertRaises(LinkClosedError):
            link.send_motion(MOVE)
        self.assertIs(link.state, LinkState.CLOSED)
        self.assertTrue(link.motion_outcome_unknown)
        self.assertEqual(link.abandoned, MOVE)
        self.assertIn("motion_outcome_unknown", kinds(link))
        port.fail_reads = False
        with self.assertRaises(LinkRefusedError):
            link.send_motion(MOVE)
        port2 = sim.FakeSerialPort(clock)
        link.reconnect(port2)
        self.assertTrue(link.motion_outcome_unknown)
        self.assertFalse(link.resync(0.5, 3.0))
        self.assert_everything_refused(link, port2, [])

    def test_6b_a_write_failure_of_a_movement_is_latched_too(self):
        link, port, clock = rig()
        port.fail_writes = True
        with self.assertRaises(LinkClosedError):
            link.send_motion(MOVE)
        self.assertIs(link.state, LinkState.CLOSED)
        self.assertTrue(link.motion_outcome_unknown, "bytes may have reached the STM before the failure")
        link.reconnect(sim.FakeSerialPort(clock))
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        with self.assertRaises(MotionOutcomeUnknownError):
            link.send_motion(MOVE)

    def test_6c_a_serial_failure_with_no_movement_pending_does_not_latch(self):
        link, port, clock = rig()
        port.fail_reads = True
        with self.assertRaises(LinkClosedError):
            link.query("PROFILE,GET", lambda line: True)
        self.assertIs(link.state, LinkState.CLOSED)
        self.assertFalse(link.motion_outcome_unknown, "a query moves nothing")
        port2 = sim.FakeSerialPort(clock)
        link.reconnect(port2)
        self.assertIs(link.state, LinkState.IDLE)
        port2.script(MOVE, sim.ordinary_move_lines())
        self.assertTrue(link.send_motion(MOVE).ok)

    def test_a_terminal_error_or_ack_never_latches(self):
        link, port, _ = rig()
        port.script(MOVE, ["MM,EV:START,D:F,T:0,V:10", "ERR,MOTION_TIMEOUT"])
        result = link.send_motion(MOVE)
        self.assertFalse(result.pose_known)
        self.assertFalse(link.motion_outcome_unknown, "an STM-reported ERR is an attributable outcome")
        self.assertIs(link.state, LinkState.IDLE)

    def test_evidence_is_never_dropped_along_the_way(self):
        link, port, clock = self.latched()
        snapshots = [link.evidence]
        port.feed("garbage")
        link.resync(0.5, 3.0)
        snapshots.append(link.evidence)
        link.reconnect(sim.FakeSerialPort(clock))
        snapshots.append(link.evidence)
        with self.assertRaises(LinkRefusedError):
            link.send_motion(MOVE)
        snapshots.append(link.evidence)
        for earlier, later in zip(snapshots, snapshots[1:]):
            self.assertEqual(later[:len(earlier)], earlier)
        self.assertGreater(len(snapshots[-1]), len(snapshots[0]))


START = "MM,EV:START,D:F,T:0,V:10"


class ErrorPoseSemanticsTest(unittest.TestCase):
    """PROTOCOL.md section 5: which terminal replies leave the dead-reckoned pose trustworthy."""

    # (terminal line, pose_known with no START seen, pose_known with START seen)
    TABLE = [
        ("ACK", True, True),
        ("ERR,MOTION_TIMEOUT", False, False),
        ("ERR,IMU_NOT_READY", True, False),
        ("ERR,BUSY", True, False),
        ("ERR,BAD_CMD", True, False),
        ("ERR,PID_NOT_SET", True, False),
        ("ERR,SOMETHING_NEW", False, False),
        ("ERR,US_FAULT", False, False),
        ("ERR,x", False, False),
    ]

    def test_the_pure_table(self):
        for text, without_start, with_start in self.TABLE:
            line = classify_line(text, 0.0)
            self.assertEqual(pose_known_after(line, False), without_start, (text, "no START"))
            self.assertEqual(pose_known_after(line, True), with_start, (text, "START seen"))

    def test_through_the_link(self):
        for text, without_start, with_start in self.TABLE:
            for started, expected in ((False, without_start), (True, with_start)):
                with self.subTest(text=text, start=started):
                    link, port, _ = rig()
                    port.script(MOVE, ([START] if started else []) + [text])
                    result = link.send_motion(MOVE)
                    self.assertEqual(result.pose_known, expected)
                    self.assertEqual(result.motion_started, started)
                    self.assertFalse(result.timed_out)
                    self.assertFalse(link.motion_outcome_unknown, "a terminal STM reply is attributable")
                    self.assertIs(link.state, LinkState.IDLE)
                    self.assertEqual(port.commands_written(), [MOVE])

    def test_1_motion_timeout_without_start_is_pose_unknown(self):
        link, port, _ = rig()
        port.script(MOVE, ["ERR,MOTION_TIMEOUT"])
        result = link.send_motion(MOVE)
        self.assertFalse(result.motion_started, "START telemetry may simply have been lost")
        self.assertFalse(result.pose_known)

    def test_the_move_adapter_carries_the_value(self):
        link, port, _ = rig()
        port.script(MOVE, ["ERR,MOTION_TIMEOUT"])
        reply = link.move(MOVE)
        self.assertEqual((reply.status, reply.motion_started, reply.pose_known), ("error", False, False))

class InterruptedTransactionTest(unittest.TestCase):
    """Anything that interrupts an in-flight transaction before an attributable terminal reply."""

    def interrupt_on_read(self, port, exc, on_call=2):
        calls = []

        def hook():
            calls.append(1)
            if len(calls) == on_call:
                raise exc

        port.on_read = hook

    def assert_latched_and_locked(self, link, port, written=(MOVE,)):
        self.assertTrue(link.motion_outcome_unknown)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertEqual(link.abandoned, written[0])
        self.assertIn("motion_outcome_unknown", kinds(link))
        # resync and reconnect cannot unlock it
        self.assertFalse(link.resync(0.2, 1.0))
        link.reconnect(sim.FakeSerialPort(port.clock))
        self.assertTrue(link.motion_outcome_unknown)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        # and nothing further is admitted or written
        with self.assertRaises(MotionOutcomeUnknownError):
            link.send_motion(MOVE)
        with self.assertRaises(MotionOutcomeUnknownError):
            link.query("PROFILE,GET", lambda line: True)
        self.assertEqual(port.commands_written(), list(written), "no later command was written, nothing was resent")

    def test_1_keyboard_interrupt_while_waiting_latches_and_is_reraised(self):
        link, port, _ = rig()
        original = KeyboardInterrupt("ctrl-c")
        self.interrupt_on_read(port, original)
        with self.assertRaises(KeyboardInterrupt) as caught:
            link.send_motion(MOVE)
        self.assertIs(caught.exception, original, "the original exception, unchanged")
        self.assert_latched_and_locked(link, port)

    def test_2_system_exit_during_a_pending_movement_behaves_the_same(self):
        link, port, _ = rig()
        original = SystemExit(3)
        self.interrupt_on_read(port, original)
        with self.assertRaises(SystemExit) as caught:
            link.send_motion(MOVE)
        self.assertIs(caught.exception, original)
        self.assert_latched_and_locked(link, port)

    def test_3a_a_non_oserror_port_failure_during_the_write_latches(self):
        for after_record in (False, True):
            with self.subTest(bytes_went_out=after_record):
                link, port, _ = rig()
                original = RuntimeError("driver bug")
                port.write_error, port.write_after_record = original, after_record
                with self.assertRaises(RuntimeError) as caught:
                    link.send_motion(MOVE)
                self.assertIs(caught.exception, original)
                self.assertTrue(link.motion_outcome_unknown, "the command may have reached the STM")
                self.assertIs(link.state, LinkState.DESYNCHRONIZED)
                self.assertEqual(link.abandoned, MOVE)
                port.write_error = None
                with self.assertRaises(MotionOutcomeUnknownError):
                    link.send_motion(MOVE)
                self.assertEqual(len(port.commands_written()), 1 if after_record else 0)

    def test_3b_a_failure_immediately_after_the_write_latches(self):
        def log(text):
            if text.startswith("STM <-"):
                raise RuntimeError("logger died right after the write")

        link, port, _ = rig(log=log)
        with self.assertRaises(RuntimeError):
            link.send_motion(MOVE)
        self.assertEqual(port.commands_written(), [MOVE])
        self.assert_latched_and_locked(link, port)

    def test_3c_a_non_oserror_read_failure_latches(self):
        link, port, _ = rig()
        port.read_error = ValueError("decoder bug")
        with self.assertRaises(ValueError):
            link.send_motion(MOVE)
        port.read_error = None                       # the fake port recovers; the latch must not
        self.assert_latched_and_locked(link, port)

    def test_3d_a_failure_that_lands_between_the_write_and_the_wait_latches(self):
        link, port, clock = rig()
        original = RuntimeError("clock failed")
        calls = []
        real = link._clock

        def clock_that_fails():
            calls.append(1)
            if len(calls) == 2:                  # the deadline computation, right after the write
                raise original
            return real()

        link._clock = clock_that_fails
        with self.assertRaises(RuntimeError) as caught:
            link.send_motion(MOVE)
        self.assertIs(caught.exception, original)
        link._clock = real
        self.assert_latched_and_locked(link, port)

    def test_4_close_with_a_pending_movement_latches_before_clearing(self):
        link, port, _ = rig()
        seen = []

        def close_once():
            if not seen:
                seen.append(True)
                link.close()

        port.on_read = close_once
        with self.assertRaises(MotionOutcomeUnknownError):
            link.send_motion(MOVE)
        self.assertTrue(link.motion_outcome_unknown)
        self.assertIs(link.state, LinkState.CLOSED)
        self.assertEqual(link.abandoned, MOVE)
        link.reconnect(sim.FakeSerialPort(port.clock))
        self.assertIs(link.state, LinkState.DESYNCHRONIZED, "a new port does not unlock it")
        self.assertEqual(port.commands_written(), [MOVE])

    def test_4b_a_reply_that_arrives_after_close_cannot_complete_the_movement(self):
        link, port, _ = rig()
        port.script(MOVE, ["ACK"])
        port.on_read = lambda: link.close()
        with self.assertRaises(MotionOutcomeUnknownError):
            link.send_motion(MOVE)
        self.assertTrue(link.motion_outcome_unknown)

    def test_5_close_while_idle_does_not_latch(self):
        link, port, clock = rig()
        link.close()
        self.assertIs(link.state, LinkState.CLOSED)
        self.assertFalse(link.motion_outcome_unknown)
        with self.assertRaises(LinkClosedError):
            link.send_motion(MOVE)
        port2 = sim.FakeSerialPort(clock)
        link.reconnect(port2)
        self.assertIs(link.state, LinkState.IDLE)
        port2.script(MOVE, sim.ordinary_move_lines())
        self.assertTrue(link.send_motion(MOVE).ok)

    def test_5b_close_during_a_query_does_not_latch(self):
        link, port, _ = rig()
        port.on_read = lambda: link.close()
        with self.assertRaises(LinkClosedError):
            link.query("PROFILE,GET", lambda line: True)
        self.assertIs(link.state, LinkState.CLOSED)
        self.assertFalse(link.motion_outcome_unknown)
        self.assertEqual(link.abandoned, "PROFILE,GET")

    def test_6_an_interrupted_query_desynchronises_without_the_motion_latch(self):
        for exc in (KeyboardInterrupt("ctrl-c"), SystemExit(2), RuntimeError("boom")):
            with self.subTest(type(exc).__name__):
                link, port, _ = rig()
                self.interrupt_on_read(port, exc)
                with self.assertRaises(type(exc)) as caught:
                    link.query("PROFILE,GET", lambda line: line.startswith("PROFILE,"))
                self.assertIs(caught.exception, exc)
                self.assertIsNot(link.state, LinkState.QUERY_PENDING, "never stuck pending")
                self.assertIs(link.state, LinkState.DESYNCHRONIZED)
                self.assertFalse(link.motion_outcome_unknown, "a query moves nothing")
                self.assertEqual(link.abandoned, "PROFILE,GET")
                with self.assertRaises(LinkRefusedError):
                    link.send_motion(MOVE)
                port.on_read = None
                self.assertTrue(link.resync(0.5, 3.0), "ordinary desynchronisation is recoverable")
                port.script(MOVE, sim.ordinary_move_lines())
                self.assertTrue(link.send_motion(MOVE).ok)

    def test_6b_a_failing_accept_callback_leaves_a_query_desynchronised(self):
        link, port, _ = rig()
        port.script("PROFILE,GET", ["PROFILE,PRECISION"])

        def accept(line):
            raise RuntimeError("callback bug")

        with self.assertRaises(RuntimeError):
            link.query("PROFILE,GET", accept)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertFalse(link.motion_outcome_unknown)

    def test_7_resync_and_reconnect_cannot_unlock_an_interrupted_movement(self):
        link, port, clock = rig()
        self.interrupt_on_read(port, KeyboardInterrupt())
        with self.assertRaises(KeyboardInterrupt):
            link.send_motion(MOVE)
        port.on_read = None
        port.feed("ACK")                                  # even a late ACK proves nothing about the robot
        self.assertFalse(link.resync(0.5, 3.0))
        self.assertTrue(link.motion_outcome_unknown)
        port2 = sim.FakeSerialPort(clock)
        link.reconnect(port2)
        self.assertFalse(link.resync(0.5, 3.0))
        self.assertTrue(link.motion_outcome_unknown)
        self.assertEqual(port2.commands_written(), [])
        self.assertEqual(port.commands_written(), [MOVE])

    def test_8_a_logging_failure_after_the_terminal_reply_is_accepted_does_not_latch(self):
        fired = []

        def log(text):
            if text.startswith("STM -> ") and not fired:
                fired.append(True)
                raise RuntimeError("logger died after the reply")

        link, port, _ = rig(log=log)
        port.script(MOVE, sim.ordinary_move_lines())
        with self.assertRaises(RuntimeError):
            link.send_motion(MOVE)
        self.assertFalse(link.motion_outcome_unknown, "the ACK was already accepted")
        self.assertIs(link.state, LinkState.IDLE)
        port.script("MOV,B,5", sim.ordinary_move_lines("B", 5))
        link.send_motion("MOV,B,5")
        self.assertEqual(port.commands_written(), [MOVE, "MOV,B,5"])

    def test_the_movement_is_never_resent_after_any_interruption(self):
        for exc in (KeyboardInterrupt(), SystemExit(1), RuntimeError("x")):
            link, port, _ = rig()
            self.interrupt_on_read(port, exc)
            with self.assertRaises(type(exc)):
                link.send_motion(MOVE)
            for _ in range(2):
                with self.assertRaises(LinkRefusedError):
                    link.send_motion(MOVE)
            self.assertEqual(port.commands_written(), [MOVE])

    def test_a_clean_run_is_unaffected(self):
        link, port, _ = rig()
        port.script(MOVE, sim.ordinary_move_lines())
        self.assertTrue(link.send_motion(MOVE).ok)
        self.assertIs(link.state, LinkState.IDLE)
        self.assertFalse(link.motion_outcome_unknown)


if __name__ == "__main__":
    unittest.main()
