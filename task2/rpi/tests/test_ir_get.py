"""Phase 3b.5a tests: `Task2Stm.read_ir` on the frozen `IR,GET` query (PROTOCOL.md §4.5, §4.11.4, D-51).
Run from the task2 folder:

    python3 -m unittest discover -s tests -v

Host only: a deterministic fake port (sim.py) replays the firmware's reply spelling. Read-only: no motion.
"""

from __future__ import annotations

import unittest

import sim
from ir import IrStatus
from stm_link import StmLink
from stm_protocol import LinkRefusedError, LinkState
from stm_task2 import IR_GET_CAPABILITY, Task2Stm


def reply(seq: int, left: str = "145", right: str = "FAR", age: int = 14) -> str:
    return f"IR,S:{seq},ST:OK,L:{left},R:{right},AGE:{age},F:00"


def rig():
    clock = sim.FakeClock()
    port = sim.FakeSerialPort(clock, read_step_s=0.01)
    notes: list[str] = []
    link = StmLink(port, log=lambda _: None, clock=clock, sleep=clock.advance)
    link.note = notes.append
    return Task2Stm(link, clock=clock, sleep=clock.advance), link, port, clock, notes


class ReadIrTest(unittest.TestCase):
    def test_one_query_one_sample(self):
        stm, link, port, _, notes = rig()
        port.script("IR,GET", ["MM,D:F,T:1", reply(1532)])        # telemetry first is skipped, never the reply
        sample = stm.read_ir()
        self.assertEqual(port.commands_written(), ["IR,GET"])
        self.assertEqual((sample.seq, sample.age_ms, sample.problem), (1532, 14, None))
        self.assertEqual((sample.left.status, sample.left.distance_cm), (IrStatus.VALID_DISTANCE, 14.5))
        self.assertEqual(sample.right.status, IrStatus.INVALID)
        self.assertIs(stm.last_ir, sample)
        self.assertIs(link.state, LinkState.IDLE)
        self.assertEqual(notes, [])

    def test_sequence_must_increase(self):
        stm, _, port, _, notes = rig()
        for seq in (10, 60, 60, 59, 0):
            port.script("IR,GET", [reply(seq)])
        self.assertEqual(stm.read_ir().left.status, IrStatus.VALID_DISTANCE)
        self.assertEqual(stm.read_ir().left.status, IrStatus.VALID_DISTANCE)
        repeated = stm.read_ir()
        self.assertEqual((repeated.left.status, repeated.right.status), (IrStatus.STALE, IrStatus.STALE))
        older = stm.read_ir()
        self.assertEqual((older.left.status, older.right.status), (IrStatus.INVALID, IrStatus.INVALID))
        zero = stm.read_ir()
        self.assertEqual(zero.left.status, IrStatus.STALE)
        self.assertEqual(len(notes), 3, "each rejected reply is written to the raw log")

    def test_at_most_ten_per_second(self):
        stm, _, port, clock, _ = rig()
        port.script("IR,GET", [reply(1)])
        port.script("IR,GET", [reply(2)])
        stm.read_ir()
        first = clock()
        stm.read_ir()                                  # replies are instant: only the rate limit moves time
        self.assertGreaterEqual(clock() - first, 0.1 - 1e-9)

    def test_lost_reply_desynchronises_and_is_not_resent(self):
        stm, link, port, _, notes = rig()
        sample = stm.read_ir()                                     # nothing scripted: silence
        self.assertEqual((sample.left.status, sample.right.status), (IrStatus.INVALID, IrStatus.INVALID))
        self.assertIn("timeout", sample.problem)
        self.assertIs(link.state, LinkState.DESYNCHRONIZED)
        self.assertFalse(link.motion_outcome_unknown, "a query moved nothing")
        again = stm.read_ir()                                      # refused, not resent
        self.assertIn("refused", again.problem)
        self.assertEqual(port.commands_written(), ["IR,GET"])
        with self.assertRaises(LinkRefusedError):
            link.send_motion("MOV,F,10")                           # the next motion is refused too
        self.assertTrue(link.resync(), "a quiet resync recovers a query timeout (D-51)")

    def test_err_reply_is_untrusted(self):
        stm, link, port, _, _ = rig()
        port.script("IR,GET", ["ERR,BAD_CMD"])                     # firmware built without the IR feature
        sample = stm.read_ir()
        self.assertEqual(sample.left.status, IrStatus.INVALID)
        self.assertIn("ERR,BAD_CMD", sample.problem)
        self.assertIs(link.state, LinkState.IDLE)

    def test_malformed_and_stale_replies(self):
        stm, _, port, _, _ = rig()
        port.script("IR,GET", ["IR,S:5,ST:OK,L:NA,R:150,AGE:5,F:00"])
        port.script("IR,GET", [reply(6, age=500)])
        self.assertIn("malformed", stm.read_ir().problem)
        self.assertEqual(stm.read_ir().left.status, IrStatus.STALE)

    def test_never_during_a_motion(self):
        stm, link, port, _, _ = rig()
        port.script("MOV,F,10", ["MM,EV:START,D:F,T:0,V:10"])     # no terminal: the move times out
        link.send_motion("MOV,F,10")
        self.assertTrue(link.motion_outcome_unknown)
        sample = stm.read_ir()
        self.assertIn("refused", sample.problem)
        self.assertEqual(port.commands_written(), ["MOV,F,10"])

    def test_capability(self):
        self.assertTrue(IR_GET_CAPABILITY.implemented)
        self.assertFalse(IR_GET_CAPABILITY.bench_validated)
        self.assertFalse(IR_GET_CAPABILITY.hardware_authority, "read-only: no motion authority")


if __name__ == "__main__":
    unittest.main()
