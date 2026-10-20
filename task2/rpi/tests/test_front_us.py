"""The front ultrasonic from the `US,` bench stream (stm_us), bench lines on the link, the emergency STOP
(PROTOCOL.md §4.2.1, §6.2.3) and the mission's use of the ultrasonic. Run from the task2 folder:

    python3 -m unittest discover -s tests -v

The `US,` strings are copied from the robot (rstovv2 firmware, read-only capture 2026-10-13). The serial
line is the fake port; the mission runs in the simulator, which executes moves perfectly.
"""

from __future__ import annotations

import unittest

import config
import sim
import stm_us
from stm_link import StmLink
from stm_protocol import EmergencyStopped, LineKind, LinkRefusedError, LinkState
from stm_task2 import Task2Stm

GOOD = ("US,ST:IDLE,R:OK,PW:2753,RAW:470,FLT:{flt},V:1,F:1,AGE:{age},SEQ:{seq},CYC:{seq},OK:30,NR:0,NF:822,"
        "NEAR:0,FAR:0,EDG:0,LATE:0,ES:0,STL:0,SRS:0,SPU:0,SCP:0,FM:{fm}")
NOFALL = ("US,ST:IDLE,R:NOFALL,PW:NA,RAW:NA,FLT:453,V:0,F:0,AGE:243,SEQ:846,CYC:846,OK:27,NR:0,NF:819,NEAR:0,"
          "FAR:0,EDG:0,LATE:0,ES:0,STL:0,SRS:0,SPU:0,SCP:0,FM:00")
MOVE = "MOV,F,10"


def us(flt: int, seq: int, age: int = 40, fm: str = "00") -> str:
    return GOOD.format(flt=flt, seq=seq, age=age, fm=fm)


def parsed(*texts: str) -> list[stm_us.UsLine]:
    return [stm_us.parse_us(t) for t in texts]


def rig():
    clock = sim.FakeClock()
    port = sim.FakeSerialPort(clock)
    return StmLink(port, log=lambda text: None, clock=clock, sleep=clock.advance), port, clock


class ParseTest(unittest.TestCase):
    def test_a_real_line(self):
        line = stm_us.parse_us(us(470, 852))
        self.assertEqual((line.filtered_mm, line.age_ms, line.seq, line.fault_mask), (470, 40, 852, 0))
        self.assertIsNone(line.trusted())

    def test_untrusted_lines(self):
        self.assertIn("no fresh echo", stm_us.parse_us(NOFALL).trusted())
        self.assertIn("fault", stm_us.parse_us(us(470, 1, fm="40")).trusted())
        self.assertIn("AGE", stm_us.parse_us(us(470, 1, age=config.US_MAX_AGE_MS + 1)).trusted())

    def test_malformed_lines(self):
        for text in (us(470, 1)[:-1], us(470, 1) + ",X:1", us(470, 1).replace("FLT:470", "FLT:-4"),
                     "USPARK,STOP,R:PRED", "US,ST:IDLE"):
            self.assertIsNone(stm_us.parse_us(text), text)


class SummariseTest(unittest.TestCase):
    def test_median_of_agreeing_values_rejects_an_outlier(self):
        reading = stm_us.summarise(parsed(us(470, 1), us(480, 2), us(144, 3), us(476, 4)))
        self.assertAlmostEqual(reading.distance_cm, 47.6)
        self.assertEqual(reading.used, 3)

    def test_too_few_agreeing_or_duplicates_give_no_reading(self):
        self.assertIsNone(stm_us.summarise(parsed(us(470, 1), us(300, 2), us(600, 3))).distance_cm)
        self.assertIsNone(stm_us.summarise(parsed(us(470, 1), us(470, 1), us(470, 1))).distance_cm,
                          "one SEQ counts once")
        self.assertIsNone(stm_us.summarise(parsed(NOFALL, NOFALL, NOFALL)).distance_cm)
        self.assertIn("no US, lines", stm_us.summarise([]).problem)


class BenchLineLinkTest(unittest.TestCase):
    def test_bench_lines_are_bench_and_do_not_block_resync(self):
        link, port, clock = rig()

        def stream():                       # a US, line every read, like the 5 Hz firmware stream
            port.feed(us(470, int(clock.now * 10) + 1))
            clock.advance(0.05)

        port.on_read = stream
        self.assertTrue(link.resync(0.5, 3.0))
        self.assertIs(link.state, LinkState.IDLE)
        self.assertTrue(all(line.kind is LineKind.BENCH for line in link.recent_us))
        self.assertFalse([e for e in link.evidence if e.kind == "unknown"])

    def test_measure_front_listens_without_writing(self):
        link, port, _ = rig()
        for seq, flt in enumerate((470, 480, 476, 144, 471), start=1):
            port.feed(us(flt, seq))
        reading = Task2Stm(link).measure_front()
        self.assertAlmostEqual(reading.distance_cm, 47.35, msg="median of 47.0, 48.0, 47.6, 47.1; 14.4 rejected")
        self.assertEqual(port.commands_written(), [])

    def test_measure_front_without_a_stream(self):
        link, port, _ = rig()
        reading = Task2Stm(link).measure_front(timeout_s=1.0)
        self.assertIsNone(reading.distance_cm)
        self.assertEqual(reading.seen, 0)


def usd(mm: int | str, seq: int, age: int = 20, st: str = "OK", f: str = "00") -> str:
    """A USD,GET reply in the frozen §4.8 spelling (as app_comm.c AppComm_TrySendUsdGet formats it)."""
    return f"USD,S:{seq},ST:{st},D:{mm},AGE:{age},F:{f}"


def stm_rig():
    clock = sim.FakeClock()
    port = sim.FakeSerialPort(clock, read_step_s=0.01)
    link = StmLink(port, log=lambda text: None, clock=clock, sleep=clock.advance)
    return Task2Stm(link, clock=clock, sleep=clock.advance), link, port, clock


class UsdParseTest(unittest.TestCase):
    def test_replies_the_firmware_sends(self):
        line = stm_us.parse_usd(usd(468, 812, age=14))
        self.assertEqual((line.filtered_mm, line.age_ms, line.seq, line.fault_mask), (468, 14, 812, 0))
        self.assertIsNone(line.trusted())
        for text in (usd("NEAR", 5), usd("FAR", 5), usd("NA", 5, st="WARM"), usd("NA", 0, st="STALE", f="40"),
                     usd("NA", 5, st="ERR", f="40")):
            self.assertIsNotNone(stm_us.parse_usd(text), text)
            self.assertIsNotNone(stm_us.parse_usd(text).trusted(), f"{text} is not a distance")
        fault = stm_us.parse_usd(usd("NA", 1, st="FAULT", f="C0"))
        self.assertEqual(fault.fault_mask, 0x80)
        self.assertIn("fault", fault.trusted())
        self.assertIn("AGE", stm_us.parse_usd(usd(468, 5, age=config.US_MAX_AGE_MS + 1)).trusted())

    def test_grammar_and_state_rules(self):
        for text in (usd(468, 5)[:-1], usd(468, 5) + ",X:1", usd(9, 5), usd(10000, 5), usd("NA", 5),
                     usd(468, 5, f="40"), usd(468, 5, st="WARM"), usd("NA", 5, st="WARM", f="40"),
                     usd("NA", 5, st="STALE"), usd(468, 5, st="STALE", f="40"), "USD,S:01,ST:OK,D:468,AGE:1,F:00",
                     "US,GET", us(470, 1)):
            self.assertIsNone(stm_us.parse_usd(text), text)


class UsdQueryTest(unittest.TestCase):
    def test_probe_turns_queries_on_and_measure_front_uses_them(self):
        stm, link, port, clock = stm_rig()
        port.script("USD,GET", [usd(470, 10)])
        self.assertIsNone(stm.probe_usd_get())
        self.assertTrue(stm.has_usd_get)
        for seq, mm in ((10, 470), (11, 480), (11, 480), (12, 144), (13, 476), (14, 471)):
            port.script("USD,GET", [usd(mm, seq)])
        started = clock()
        reading = stm.measure_front()
        self.assertAlmostEqual(reading.distance_cm, 47.6, msg="47.0 repeated S skipped, 14.4 rejected")
        self.assertEqual((reading.used, reading.seen), (3, 4))
        self.assertEqual(port.commands_written(), ["USD,GET"] * 7, "the probe, then stops as soon as three agree")
        self.assertLess(clock() - started, 0.3)
        self.assertIs(link.state, LinkState.IDLE)

    def test_old_firmware_keeps_listening(self):
        stm, link, port, _ = stm_rig()
        port.script("USD,GET", ["ERR,BAD_CMD"])
        self.assertIn("ERR,BAD_CMD", stm.probe_usd_get())
        self.assertFalse(stm.has_usd_get)
        self.assertIs(link.state, LinkState.IDLE)
        for seq, flt in enumerate((470, 480, 476), start=1):
            port.feed(us(flt, seq))
        self.assertAlmostEqual(stm.measure_front(lines=3).distance_cm, 47.6)
        self.assertEqual(port.commands_written(), ["USD,GET"], "the bench stream is only listened to")

    def test_no_distance_is_no_reading(self):
        stm, _, port, _ = stm_rig()
        stm.has_usd_get = True
        for seq in range(1, 200):
            port.script("USD,GET", [usd("FAR", seq)])
        reading = stm.measure_front(timeout_s=1.0)
        self.assertIsNone(reading.distance_cm)
        self.assertIn("no fresh echo", reading.problem)

    def test_lost_reply_or_backwards_s_ends_it(self):
        stm, link, port, _ = stm_rig()
        stm.has_usd_get = True
        port.script("USD,GET", [usd(470, 10)])                     # then silence
        reading = stm.measure_front()
        self.assertIsNone(reading.distance_cm)
        self.assertIn("timeout", reading.problem)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertEqual(port.commands_written(), ["USD,GET"] * 2, "nothing resent after a lost reply")

        stm, _, port, _ = stm_rig()
        stm.has_usd_get = True
        port.script("USD,GET", [usd(470, 10)])
        port.script("USD,GET", [usd(470, 9)])
        reading = stm.measure_front()
        self.assertIsNone(reading.distance_cm)
        self.assertIn("S:9 after S:10", reading.problem)

    def test_bench_lines_during_a_query_are_not_the_reply(self):
        stm, link, port, _ = stm_rig()
        port.script("USD,GET", [us(470, 3), usd(470, 10)])         # a bench US, line arrives first
        self.assertIsNone(stm.probe_usd_get())
        self.assertIs(link.state, LinkState.IDLE)


class EmergencyStopTest(unittest.TestCase):
    def test_fence_stop_at_idle(self):
        link, port, _ = rig()
        port.script("STOP", ["ACK,STOPPED"])
        self.assertTrue(link.emergency_stop("fence"))
        self.assertFalse(link.motion_gate_held, "nothing was moving")
        port.script(MOVE, sim.ordinary_move_lines())
        self.assertTrue(link.send_motion(MOVE).ok)

    def test_a_stop_requested_during_a_move_stops_it_and_holds_the_gate(self):
        link, port, _ = rig()
        port.script(MOVE, ["MM,EV:START,D:F,T:0,V:10"])
        port.script("STOP", ["MM,EV:END,D:F,T:300,RESULT:STOPPED", "ACK,STOPPED"])
        port.on_read = lambda: link.request_stop("SIGINT") if port.commands_written() == [MOVE] else None
        with self.assertRaises(EmergencyStopped):
            link.send_motion(MOVE)
        self.assertEqual(port.commands_written(), [MOVE, "STOP"])
        self.assertEqual(link.stop_state, "CONFIRMED")
        self.assertTrue(link.motion_gate_held)
        with self.assertRaises(EmergencyStopped):
            link.send_motion(MOVE)

    def test_a_stop_requested_at_idle_still_ends_all_movement(self):
        link, port, _ = rig()
        port.script("STOP", ["ACK,STOPPED"])
        link.request_stop("SIGTERM")
        with self.assertRaises(EmergencyStopped):
            link.query("PROFILE,GET", lambda line: True)
        with self.assertRaises(EmergencyStopped):
            link.send_motion(MOVE)

    def test_no_reply_is_unconfirmed_after_three_attempts(self):
        link, port, _ = rig()
        self.assertFalse(link.emergency_stop("test"))
        self.assertEqual(port.commands_written(), ["STOP"] * 3)
        self.assertEqual(link.stop_state, "UNCONFIRMED")
        self.assertTrue(link.motion_outcome_unknown)
        with self.assertRaises(LinkRefusedError):
            link.send_motion(MOVE)


if __name__ == "__main__":
    unittest.main()
