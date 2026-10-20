"""RPi <-> STM32 serial link (protocol from RTOSV/Core/Src/vision_rx.c and app_comm.c).

PHASE 3a (transport hardening only; NOT hardware-ready). One command in flight
at a time. For MOV, only an exact `ACK` and `ERR,...` are terminal (Phase 3b.5a:
`send_sensor_motion` carries `MOV,FIRL/FIRR` and also ends on an exact
`ACK,STOPPED`, never a success; see stm_fir.py); every other
line (`MM,`, `AK,`, `TB,`, `MD,`, ...) is telemetry or unknown, is written
unchanged to the raw log (if one is given) and kept as evidence. A command whose
outcome is unknown is never resent.

Receive path. There is ONE place that reads the port (`_read`) and ONE
classifier (`stm_protocol.classify_line`). The link never discards buffered
input: before it writes it reads and classifies whatever is already waiting (if
the port can say how much is waiting), so a stale line is evidence rather than
the reply of the next command. The protocol has no command IDs, so any outcome
the Pi cannot attribute puts the link into DESYNCHRONIZED: every motion and
query is refused until `resync()` has seen a quiet interval, or the port is
replaced with `reconnect()` -- EXCEPT after an unknown MOTION outcome.

Unknown motion outcome (safety latch). A movement-command timeout, a serial
failure while a movement may have been active, and ANY exception or interruption
(KeyboardInterrupt, SystemExit, a non-OSError port or callback failure) or
`close()` after a motion transaction was installed and before an attributable
terminal reply was accepted, set `motion_outcome_unknown`; the original exception
is re-raised unchanged. Once a terminal reply is accepted a later logging failure
changes nothing. A pending QUERY that is interrupted only desynchronises. A
quiet serial line proves nothing about whether the robot is still moving, so
this latch is never cleared by `resync()` or `reconnect()` on the same StmLink,
and no command is admitted while it is set. There is deliberately no "robot
stopped" acknowledgement API: only a frozen, positively confirmed STOP/recovery
procedure (Phase 3b) may clear it. A query-only timeout moved nothing, so it only
desynchronises the link and a quiet `resync()` still recovers it.

Emergency STOP (PROTOCOL.md §4.2.1, §6.2.3). `request_stop()` only sets a flag, so a
signal handler may call it (no serial I/O in a handler). The reading thread notices the
flag at its next read (every serial timeout, 0.1 s) or command, writes `STOP` itself and
waits for the exact `ACK,STOPPED` (resent up to STOP_MAX_ATTEMPTS times; STOP is
idempotent). `emergency_stop()` is also called directly: a fence STOP at start-up and
when the program ends. A STOP that may have crossed a movement holds the motion gate for
the life of the link (every later MOV is refused); an unconfirmed STOP also latches the
unknown motion outcome. Implemented, NOT yet bench-validated on the robot (L3).

Bench lines. `US,`, `IRRAW,` and `USPARK,TRACE,` (firmware bench logs, §6.2.2) are
classified BENCH: kept as evidence, never terminal, never a reply, and they do not count
as activity for `resync()`, so a firmware that streams them can still be resynchronised.
The latest `US,` lines are kept for `stm_us` (`recent_us`, `listen`).
"""

from __future__ import annotations

import contextlib
import re
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Protocol, TextIO

import config
import stm_fir
from stm_protocol import (EmergencyStopped, LineKind, LinkBusyError, LinkClosedError, LinkError, LinkEvent,
                          LinkRefusedError, LinkState, MotionOutcomeUnknownError, MotionResult, StmLine,
                          classify_line, pose_known_after)

MOV_PATTERN = re.compile(r"^MOV,(F|B|FL|FR|BL|BR),(\d{1,3})$")
MAX_LINE_CHARS = 31          # STM rejects longer lines with ERR,BAD_CMD
STRAIGHT_MAX_CM = 999
TURN_MAX_DEG = 360
RPI_TIMEOUT_MARGIN_S = 3.0   # on top of the STM's own deadline
COUNTS_PER_CM = 75.0         # firmware DEFAULT_MOVCAL_SLOPE_COUNTS_PER_CM (rough cm only)
TURN_SIGN = {"FL": 1, "BL": 1, "FR": -1, "BR": -1}   # heading change per degree (PROTOCOL.md §3)
MAX_HEADING_FIX_DEG = 5.0    # larger errors are capped, not chased
SETTLED_YAW_RATE_DPS = 1.0   # IMU,GET is trusted once |GZ| is at most this
RESYNC_QUIET_S = 0.5         # PROVISIONAL: the line must be silent this long to count as quiet
RESYNC_MAX_WAIT_S = 3.0      # PROVISIONAL: give up (stay DESYNCHRONIZED) after this long
MAX_DRAIN_LINES = 1000       # bound on the pre-write read of already-waiting lines
BENCH_PREFIXES = ("US,", "IRRAW,", "USPARK,TRACE,")   # PROTOCOL.md §6.2.2 bench lines
US_PREFIX = "US,"
RECENT_US_LINES = 50
STOP_COMMAND = "STOP"                    # PROTOCOL.md §4.2.1, exact
STOP_REPLY_TIMEOUT_S = 0.5               # PROVISIONAL (D-59), calibrated at L3
STOP_MAX_ATTEMPTS = 3                    # PROVISIONAL (D-59)
STOP_DRAIN_S = 0.3                       # PROVISIONAL (D-59)


class SerialPort(Protocol):
    """What the link needs from a port. A port may also offer `in_waiting` (bytes
    already received, as pyserial does); the link uses it, when present, to read
    waiting lines before it writes. There is deliberately no way to clear input."""

    def write(self, data: bytes) -> int: ...
    def readline(self) -> bytes: ...


@dataclass
class StmReply:
    """Outcome of one command (the shape the mission and the older tests use).

    status is "ok", "error", "timeout" or "desynchronized". `motion_started`
    records whether `MM,EV:START` was seen, i.e. whether the robot may have moved
    before an error. `pose_known` is False when the outcome cannot be trusted
    (a timeout, or an error after motion started).
    """

    status: str
    line: str = ""
    motion_started: bool = False
    telemetry: list[str] = field(default_factory=list)
    pose_known: bool = True

    @property
    def ok(self) -> bool:
        return self.status == "ok"


def validate_mov(command: str) -> tuple[str, int]:
    """Check a MOV string against the STM grammar before it goes on the wire."""
    match = MOV_PATTERN.fullmatch(command)
    if match is None or len(command) > MAX_LINE_CHARS:
        raise ValueError(f"not a valid STM MOV command: {command!r}")
    direction, value = match.group(1), int(match.group(2))
    limit = STRAIGHT_MAX_CM if direction in ("F", "B") else TURN_MAX_DEG
    if value > limit:
        raise ValueError(f"{command!r} exceeds the STM limit of {limit}")
    return direction, value


def mov_timeout_s(command: str) -> float:
    """STM deadline (5 s + 0.15 s/cm or + 0.10 s/deg) plus an RPi margin."""
    direction, value = validate_mov(command)
    per_unit = 0.15 if direction in ("F", "B") else 0.10
    return 5.0 + per_unit * value + RPI_TIMEOUT_MARGIN_S


def _fields(line: str) -> dict[str, str]:
    return dict(part.split(":", 1) for part in line.split(",")[1:] if ":" in part)


def summarise_motion(telemetry: list[str]) -> str | None:
    """One line from a move's `MM,` telemetry: IMU heading at the first and last
    sample (and the STM's target), each wheel's travel, and the time taken."""
    samples = [_fields(line) for line in telemetry if line.startswith("MM,D:")]
    end = next((_fields(line) for line in telemetry if line.startswith("MM,EV:END")), None)
    if not samples:
        return None
    first, last = samples[0], samples[-1]
    try:
        left, right = int(last["LP"]), int(last["RP"])
        text = (f"heading {int(first['H']) / 10:.1f} -> {int(last['H']) / 10:.1f} "
                f"(target {int(last['TH']) / 10:.1f}), wheels L {left} R {right} counts "
                f"(~{left / COUNTS_PER_CM:.1f} / {right / COUNTS_PER_CM:.1f} cm)")
    except (KeyError, ValueError):
        return None
    if end is not None:
        text += f", {end.get('T', '?')} ms {end.get('RESULT', '')}".rstrip()
    return text + f", {len(samples)} samples"


def corrected_turn(command: str, expected_deg: float, actual_deg: float,
                   max_fix_deg: float = MAX_HEADING_FIX_DEG) -> tuple[str, float]:
    """Adjust a turn so it ends at the plan's heading instead of the planned
    amount from wherever the robot is now.

    The STM turns by `value` from its current heading and stops a little short
    (~0.65 deg measured), so errors add up turn after turn. `expected_deg` is
    the IMU heading the plan expects before this turn, `actual_deg` the IMU
    reading. Returns the command to send and the error (deg, +CCW) it absorbs.
    """
    direction, value = validate_mov(command)
    sign = TURN_SIGN[direction]
    error = (expected_deg - actual_deg + 180.0) % 360.0 - 180.0
    error = max(-max_fix_deg, min(max_fix_deg, error))
    fixed = max(1, min(TURN_MAX_DEG, round(value + sign * error)))
    return f"MOV,{direction},{fixed}", error


@dataclass
class _Transaction:
    command: str
    kind: str            # "motion" or "query"
    started_s: float


class StmLink:
    def __init__(
        self,
        port: SerialPort,
        log: Callable[[str], None] = print,
        clock: Callable[[], float] = time.monotonic,
        raw_log: TextIO | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._port = port
        self._log = log
        self._clock = clock
        self._raw_log = raw_log
        self._sleep = sleep
        self._state = LinkState.IDLE
        self._txn: _Transaction | None = None          # the one outstanding command
        self._abandoned: _Transaction | None = None    # the last command whose outcome was lost
        self._evidence: list[LinkEvent] = []
        self._motion_unknown = False                   # the persistent safety latch (see the module docstring)
        self.idle_lines = 0                            # telemetry-class lines read while nothing was outstanding
        self._stop_requested: str | None = None        # set by request_stop() (signal-safe), acted on by the reader
        self._gate_held = False                        # a STOP may have crossed a movement: no MOV, ever
        self.stop_state = "NONE"                       # NONE, CONFIRMED or UNCONFIRMED (the last STOP procedure)
        self.recent_us: deque[StmLine] = deque(maxlen=RECENT_US_LINES)

    # -- state and evidence ---------------------------------------------------------------
    @property
    def state(self) -> LinkState:
        return self._state

    @property
    def motion_outcome_unknown(self) -> bool:
        """True once a movement's outcome became unknown. Read-only: nothing in this class clears it."""
        return self._motion_unknown

    @property
    def abandoned(self) -> str | None:
        """The command whose outcome was lost (timed out), if the link has not yet been
        resynchronised or reconnected since."""
        return self._abandoned.command if self._abandoned is not None else None

    @property
    def evidence(self) -> tuple[LinkEvent, ...]:
        """Unknown, late, unexpected and state-change events, in order. Never cleared."""
        return tuple(self._evidence)

    def _note_event(self, kind: str, text: str) -> None:
        self._evidence.append(LinkEvent(self._clock(), kind, text))
        self._raw(f"## {kind}: {text}")
        self._log(f"STM link {kind}: {text}")

    def note(self, text: str) -> None:
        """Write a marker (e.g. the mission id) into the raw telemetry log."""
        if self._raw_log is not None:
            self._raw_log.write(f"## {text}\n")
            self._raw_log.flush()

    def _raw(self, text: str) -> None:
        if self._raw_log is not None:
            self._raw_log.write(f"{self._clock():.3f} {text}\n")

    def _desynchronise(self, why: str) -> None:
        if self._state is not LinkState.DESYNCHRONIZED:
            self._state = LinkState.DESYNCHRONIZED
            self._note_event("desync", why)

    def _latch_motion_unknown(self, why: str, emit: bool = True) -> None:
        """Set the safety latch and make sure the link refuses everything. The flag and the state change
        come first. `emit=False` records the evidence in memory only (plus a best-effort raw-log line),
        so that it cannot fail and replace the exception that is being handled."""
        first = not self._motion_unknown
        self._motion_unknown = True
        if self._state is not LinkState.CLOSED:
            self._state = LinkState.DESYNCHRONIZED
        if first:
            if emit:
                self._note_event("motion_outcome_unknown", why)
            else:
                self._evidence.append(LinkEvent(self._clock(), "motion_outcome_unknown", why))
                with contextlib.suppress(OSError, ValueError):      # logging I/O only
                    self._raw(f"## motion_outcome_unknown: {why}")

    def _abandon_pending(self, command: str, kind: str, why: str) -> None:
        """The transaction of `command` is lost without an attributable terminal reply. Memory only: no
        I/O that could raise. A motion latches; a query only desynchronises."""
        self._abandoned = self._txn if self._txn is not None else _Transaction(command, kind, self._clock())
        self._txn = None
        if kind == "motion":
            self._latch_motion_unknown(why, emit=False)
        else:
            self._state = LinkState.DESYNCHRONIZED
            self._evidence.append(LinkEvent(self._clock(), "desync", why))
            with contextlib.suppress(OSError, ValueError):
                self._raw(f"## desync: {why}")

    def _require_pending(self, expected: LinkState, command: str) -> None:
        """close() or reconnect() may run from a callback while the transaction waits."""
        if self._state is not expected:
            if self._motion_unknown:
                raise MotionOutcomeUnknownError(f"{command}: the link left {expected.value} while it was waiting")
            raise LinkClosedError(f"{command}: the link left {expected.value} while it was waiting")

    def _port_failed(self, what: str, exc: OSError) -> None:
        """A serial read/write failed. If a movement may be active the outcome is unknown (latched)."""
        motion_may_be_active = self._txn is not None and self._txn.kind == "motion"
        if motion_may_be_active:
            self._abandoned = self._txn
            self._latch_motion_unknown(f"{what} failed while {self._txn.command} may be active: {exc!r}", emit=False)
        self._state, self._txn = LinkState.CLOSED, None
        self._note_event("closed", f"{what} failed: {exc!r}")

    def close(self) -> None:
        """Mark the link closed (the caller closes the port). Every command is refused. Closing while a
        movement is pending latches the unknown motion outcome first; closing while idle, or during a
        query, does not."""
        before = self._state
        if before is LinkState.MOTION_PENDING:
            self._abandon_pending(self._txn.command if self._txn else "", "motion",
                                  "link closed while a movement was pending; pose unknown")
        elif before is LinkState.QUERY_PENDING:
            self._abandoned, self._txn = self._txn, None
        self._state = LinkState.CLOSED
        self._txn = None
        if before is not LinkState.CLOSED:
            self._note_event("closed", f"link closed from {before.value}")

    def reconnect(self, port: SerialPort) -> None:
        """Adopt a new port. With no unknown motion outcome the link starts clean (IDLE, nothing
        outstanding). With `motion_outcome_unknown` set it stays DESYNCHRONIZED and refuses everything:
        a new connection does not tell the Pi whether the robot is still moving. The evidence of the old
        session, including the abandoned command, stays."""
        if self._state is LinkState.MOTION_PENDING:
            self._abandon_pending(self._txn.command if self._txn else "", "motion",
                                  "reconnect while a movement was pending; pose unknown")
        lost = f", abandoned {self._abandoned.command}" if self._abandoned is not None else ""
        latched = ", motion outcome STILL UNKNOWN (latched)" if self._motion_unknown else ""
        self._note_event("reconnect", f"new port; the old link state was {self._state.value}{lost}{latched}")
        self._port = port
        self._txn = None
        if self._motion_unknown:
            self._state = LinkState.DESYNCHRONIZED
        else:
            self._state = LinkState.IDLE
            self._abandoned = None

    # -- the one receive path -----------------------------------------------------------------
    def _read(self) -> StmLine | None:
        """Read and classify one line. None when nothing arrived (timeout, or a blank line)."""
        try:
            raw = self._port.readline()
        except OSError as exc:
            self._port_failed("read", exc)
            raise LinkClosedError(f"serial read failed: {exc!r}") from exc
        if not raw:
            return None
        text = raw.decode("ascii", errors="replace").strip()
        if not text:
            return None
        return self._classified(text)

    def _classified(self, text: str) -> StmLine:
        """Log and classify one received line; bench lines become BENCH and `US,` lines are remembered."""
        self._raw(f"-> {text}")
        line = classify_line(text, self._clock())
        if line.kind is LineKind.UNKNOWN and text.startswith(BENCH_PREFIXES):
            line = StmLine(text, LineKind.BENCH, line.received_at_s)
            if text.startswith(US_PREFIX):
                self.recent_us.append(line)
        return line

    def _write(self, command: str) -> None:
        try:
            self._port.write(f"{command}\n".encode("ascii"))
        except OSError as exc:
            self._port_failed("write", exc)
            raise LinkClosedError(f"serial write failed: {exc!r}") from exc
        self._raw(f"<- {command}")
        self._log(f"STM <- {command}")

    def _waiting_bytes(self) -> int:
        try:
            return int(getattr(self._port, "in_waiting", 0) or 0)
        except (OSError, ValueError, TypeError):
            return 0

    def _unattributed(self, line: StmLine) -> None:
        """A line that arrived while no command was outstanding (or during resync)."""
        against = f" (against timed-out {self._abandoned.command})" if self._abandoned is not None else ""
        if line.kind in (LineKind.ACK, LineKind.ERROR):
            if self._state is LinkState.IDLE:
                self._note_event("unexpected_terminal", f"{line.text} while idle")
                self._desynchronise(f"terminal reply {line.text!r} with no command outstanding")
            else:
                self._note_event("late_terminal", f"{line.text}{against}")
        elif line.kind is LineKind.UNKNOWN:
            self._note_event("unknown", f"{line.text}{against}")
        elif line.kind is LineKind.QUERY_REPLY:
            self._note_event("unsolicited_reply", f"{line.text}{against}")
        else:
            self.idle_lines += 1          # telemetry between commands: already in the raw log

    def _drain_waiting(self) -> None:
        """Read (and classify, never discard) lines already waiting before a write."""
        for _ in range(MAX_DRAIN_LINES):
            if self._state is not LinkState.IDLE or self._waiting_bytes() <= 0:
                return
            line = self._read()
            if line is None:
                return
            self._unattributed(line)

    def _admit(self, what: str, motion: bool = False) -> None:
        """Raise unless a new command may be written now (read waiting lines first)."""
        self._honour_stop_request()
        if motion and self._gate_held:
            raise EmergencyStopped(f"{what} refused: the motion gate is held after a STOP ({self.stop_state})")
        if self._state in (LinkState.MOTION_PENDING, LinkState.QUERY_PENDING):
            raise LinkBusyError(f"{what} refused: {self._state.value} (one command in flight)")
        if self._motion_unknown:
            raise MotionOutcomeUnknownError(
                f"{what} refused: a movement's outcome is unknown (latched); a quiet line or a new port "
                f"does not clear it")
        if self._state is LinkState.CLOSED:
            raise LinkClosedError(f"{what} refused: link closed")
        if self._state is LinkState.DESYNCHRONIZED:
            raise LinkRefusedError(f"{what} refused: link DESYNCHRONIZED (resync or reconnect first)")
        self._drain_waiting()
        if self._state is LinkState.DESYNCHRONIZED:
            raise LinkRefusedError(f"{what} refused: link DESYNCHRONIZED by a line that was already waiting")
        if self._state is LinkState.CLOSED:
            raise LinkClosedError(f"{what} refused: link closed")

    # -- commands -------------------------------------------------------------------------------
    def send_motion(self, command: str) -> MotionResult:
        """Send one MOV and wait for its terminal reply (exact `ACK` or `ERR,...`).

        Raises ValueError for an invalid command (nothing sent), LinkBusyError if a command is
        outstanding, LinkRefusedError if DESYNCHRONIZED, LinkClosedError if the port failed. On a
        timeout it returns a result with `timed_out`, pose unknown, and the link is
        DESYNCHRONIZED. A motion is never resent.
        """
        return self._motion_transaction(command, mov_timeout_s(command), ())

    def send_sensor_motion(self, command: str) -> MotionResult:
        """Send one IR-terminated move (`MOV,FIRL/FIRR,<max_cm>`, PROTOCOL.md §4.6) and wait for its
        terminal reply. Same rules as `send_motion` (one in flight, never resent, a timeout latches the
        unknown motion outcome), with its own strict validator and Pi timeout (§6.2.5), and one more
        terminal: an exact `ACK,STOPPED` (§4.2.1), which is never a success. `IREDGE,` lines are kept
        as part of the completion record; the caller (`stm_fir.evaluate`) judges the record."""
        return self._motion_transaction(command, stm_fir.fir_timeout_s(command), (stm_fir.STOPPED_REPLY,),
                                        record_prefixes=("IREDGE,",))

    def send_parking_motion(self, command: str) -> MotionResult:
        """The final USPARK parking movement (TASK2_USPARK_ALGO_INTEGRATION.md §7.2). Only the configured
        final command is accepted (no general USPARK distance API). Same rules as `send_motion`, plus an
        exact `ACK,STOPPED` terminal, which is NEVER a success here either: `stm_park.evaluate` decides
        whether the cause/END/reply chain is the accepted terminal parking stop. `USPARK,` lines are
        expected evidence."""
        if command != config.PARK_FINAL_COMMAND:
            raise ValueError(f"the parking transaction only sends {config.PARK_FINAL_COMMAND!r}, got {command!r}")
        return self._motion_transaction(command, mov_timeout_s(command), (stm_fir.STOPPED_REPLY,),
                                        record_prefixes=("USPARK,",))

    def _motion_transaction(self, command: str, timeout_s: float, extra_terminals: tuple[str, ...],
                            record_prefixes: tuple[str, ...] = ()) -> MotionResult:
        """One motion command on the wire. `extra_terminals` are exact lines that also end it (beyond
        `ACK` / `ERR,...`); `record_prefixes` mark expected lines that are not noted as unknown."""
        self._admit(command, motion=True)
        final: StmLine | None = None
        started = False
        telemetry: list[StmLine] = []
        try:
            self._state = LinkState.MOTION_PENDING
            self._txn = _Transaction(command, "motion", self._clock())
            self._write(command)
            deadline = self._clock() + timeout_s
            while self._clock() < deadline:
                self._honour_stop_request()
                self._require_pending(LinkState.MOTION_PENDING, command)
                line = self._read()
                self._require_pending(LinkState.MOTION_PENDING, command)
                if line is None:
                    continue
                if line.kind in (LineKind.ACK, LineKind.ERROR) or line.text in extra_terminals:
                    final = line
                    self._txn = None
                    self._state = LinkState.IDLE      # attributable from here on: later logging cannot undo it
                    break
                if line.kind is LineKind.MOTION_EVENT and line.text.startswith("MM,EV:START"):
                    started = True
                elif line.kind is LineKind.UNKNOWN and not line.text.startswith(record_prefixes):
                    self._note_event("unknown", f"{line.text} during {command}")
                telemetry.append(line)
            if final is None:
                self._abandoned, self._txn = self._txn, None
                self._latch_motion_unknown(f"no terminal reply to {command} before the Pi deadline; pose unknown")
        except BaseException as exc:
            # Any exception or interruption before an attributable terminal reply (KeyboardInterrupt,
            # SystemExit, a non-OSError port or callback failure, a failed write): the command may have
            # reached the STM, so the movement outcome is unknown. Transition, then re-raise the original.
            if self._state is LinkState.MOTION_PENDING:
                self._abandon_pending(command, "motion",
                                      f"{command} interrupted by {type(exc).__name__} before a terminal reply; pose unknown")
            raise
        if final is None:
            self._note_event("timeout", f"no terminal reply to {command} before the Pi deadline")
            self._log(f"STM -> TIMEOUT ({command})")
            result = MotionResult(command, None, tuple(telemetry), started, True, False)
        else:
            self._log(f"STM -> {final.text} ({command})")
            result = MotionResult(command, final, tuple(telemetry), started, False, pose_known_after(final, started))
        summary = summarise_motion([line.text for line in result.telemetry])
        if summary:
            self._log(f"    {summary}")
        if self._raw_log is not None:
            self._raw_log.flush()
        return result

    def move(self, command: str) -> StmReply:
        """`send_motion` in the shape the mission uses."""
        result = self.send_motion(command)
        if result.timed_out:
            status = "timeout"
        else:
            status = "ok" if result.ok else "error"
        return StmReply(status, result.final.text if result.final else "", result.motion_started,
                        [line.text for line in result.telemetry], result.pose_known)

    def query(self, command: str, accept: Callable[[str], bool], timeout_s: float = 3.0) -> StmReply:
        """Send a setup command (e.g. PROFILE,GET) and wait for a line `accept` matches
        or any `ERR,...`. Other lines are kept as telemetry. Refused while a motion is
        pending. A timeout, or a stray `ACK` (not a reply to any query), makes the
        outcome unattributable: the link becomes DESYNCHRONIZED."""
        self._admit(command)
        reply = StmReply(status="timeout")
        try:
            self._state = LinkState.QUERY_PENDING
            self._txn = _Transaction(command, "query", self._clock())
            self._write(command)
            deadline = self._clock() + timeout_s
            while self._clock() < deadline:
                self._honour_stop_request()
                self._require_pending(LinkState.QUERY_PENDING, command)
                line = self._read()
                self._require_pending(LinkState.QUERY_PENDING, command)
                if line is None:
                    continue
                if line.kind is LineKind.ERROR:
                    reply.status, reply.line = "error", line.text
                    break
                if line.kind is LineKind.ACK:
                    reply.status, reply.line = "desynchronized", line.text
                    self._note_event("unexpected_terminal", f"{line.text} while waiting for the reply to {command}")
                    break
                if accept(line.text):
                    reply.status, reply.line = "ok", line.text
                    break
                if line.kind is LineKind.UNKNOWN:
                    self._note_event("unknown", f"{line.text} during {command}")
                reply.telemetry.append(line.text)
        except BaseException as exc:
            # An interrupted query moved nothing: ordinary DESYNCHRONIZED (resync can recover it), never the
            # motion latch. It must not be left QUERY_PENDING. Re-raise the original.
            if self._state is LinkState.QUERY_PENDING:
                self._abandon_pending(command, "query", f"{command} interrupted by {type(exc).__name__}")
            raise
        self._txn = None
        if reply.status in ("timeout", "desynchronized"):
            self._abandoned = _Transaction(command, "query", self._clock())
            self._state = LinkState.IDLE        # so _desynchronise records the change
            self._desynchronise(f"{reply.status} on {command}")
        else:
            self._state = LinkState.IDLE
        self._log(f"STM -> {reply.line or 'TIMEOUT'} ({command})")
        return reply

    def resync(self, quiet_s: float = RESYNC_QUIET_S, max_s: float = RESYNC_MAX_WAIT_S) -> bool:
        """Read and classify (never discard) incoming lines until none has arrived for
        `quiet_s`, waiting at most `max_s` in total. Returns True and the link is IDLE when the
        line went quiet; returns False and the state is unchanged when it did not.

        Allowed only when no command is outstanding (IDLE or DESYNCHRONIZED). It does not restore
        any pose, and a late terminal reply seen here is logged against the timed-out command; it
        is never delivered to a newer command.

        If `motion_outcome_unknown` is set it still drains and logs, but returns False and leaves the
        link refusing everything: silence on the line does not show that a timed-out movement stopped.
        """
        if not (isinstance(quiet_s, (int, float)) and isinstance(max_s, (int, float))
                and 0 < quiet_s <= max_s):
            raise ValueError(f"need 0 < quiet_s <= max_s, got {quiet_s!r}, {max_s!r}")
        if self._state is LinkState.CLOSED:
            raise LinkClosedError("resync refused: link closed")
        if self._state in (LinkState.MOTION_PENDING, LinkState.QUERY_PENDING):
            raise LinkBusyError(f"resync refused: {self._state.value}")
        start = last_activity = self._clock()
        while True:
            now = self._clock()
            if now - last_activity >= quiet_s:
                break
            if now - start >= max_s:
                self._note_event("resync_failed", f"no quiet interval of {quiet_s:g} s within {max_s:g} s")
                return False
            line = self._read()
            if line is not None:
                self._unattributed(line)
                if line.kind is not LineKind.BENCH:     # a bench stream never goes quiet; it is not a reply
                    last_activity = self._clock()
        if self._motion_unknown:
            self._note_event("resync_blocked", f"line quiet for {quiet_s:g} s but a movement's outcome is "
                                               f"unknown (latched); link stays {self._state.value}")
            return False
        was = self._state
        self._state = LinkState.IDLE
        self._abandoned = None
        self._note_event("resync", f"quiet for {quiet_s:g} s; link IDLE (was {was.value})")
        return True

    # -- emergency STOP (PROTOCOL.md §4.2.1, §6.2.3) -----------------------------------------
    @property
    def motion_gate_held(self) -> bool:
        return self._gate_held

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested is not None

    def request_stop(self, reason: str) -> None:
        """Ask for an emergency STOP. Signal-safe: it only sets a flag; the reading thread writes STOP at its
        next read or command (within one serial timeout)."""
        if self._stop_requested is None:
            self._stop_requested = reason

    def _honour_stop_request(self) -> None:
        reason = self._stop_requested
        if reason is None:
            return
        confirmed = self.emergency_stop(reason)
        self._gate_held = True              # an operator stop ends every movement for this link, even at idle
        raise EmergencyStopped(f"emergency STOP ({reason}): {'confirmed' if confirmed else 'UNCONFIRMED'}")

    def emergency_stop(self, reason: str) -> bool:
        """Write `STOP` (bypassing the one-command rule) and wait for the exact `ACK,STOPPED`; resend up to
        STOP_MAX_ATTEMPTS times. Returns True when confirmed. Never raises for the port: a dead port is an
        unconfirmed stop. A pending command is ended (its crossing reply is kept as evidence). If a movement
        may have been running (pending, latched unknown, or a START/END seen around the reply) the motion gate
        is held for the life of the link; a confirmed fence STOP at idle with nothing moving changes nothing
        else."""
        self._stop_requested = None
        pending = self._txn
        moving = self._motion_unknown or (pending is not None and pending.kind == "motion")
        self._note_event("stop", f"STOP requested: {reason}"
                                 + (f" (pending {pending.command})" if pending is not None else ""))
        if self._state is LinkState.CLOSED:
            return self._stop_result(False, True, "the port is closed")
        confirmed, attempts = False, 0
        try:
            while not confirmed and attempts < STOP_MAX_ATTEMPTS:
                attempts += 1
                self._port.write(f"{STOP_COMMAND}\n".encode("ascii"))
                self._raw(f"<- {STOP_COMMAND}")
                self._log(f"STM <- {STOP_COMMAND} (attempt {attempts})")
                deadline = self._clock() + STOP_REPLY_TIMEOUT_S
                while self._clock() < deadline:
                    line = self._stop_read()
                    if line is None:
                        continue
                    if line.text == stm_fir.STOPPED_REPLY:
                        confirmed = True
                        break
                    moving |= self._stop_evidence(line)
            if confirmed:
                extra = 0
                deadline = self._clock() + STOP_DRAIN_S
                while self._clock() < deadline:
                    line = self._stop_read()
                    if line is None:
                        continue
                    if line.text == stm_fir.STOPPED_REPLY:
                        extra += 1
                        if extra >= attempts:
                            self._note_event("stop_anomaly", f"{extra} extra ACK,STOPPED after {attempts} STOP(s)")
                    else:
                        moving |= self._stop_evidence(line)
        except (OSError, LinkClosedError) as exc:
            if self._state is not LinkState.CLOSED:
                self._state = LinkState.CLOSED
                self._note_event("closed", f"serial failure during STOP: {exc!r}")
            return self._stop_result(False, True, f"serial failure: {exc!r}")
        return self._stop_result(confirmed, moving, f"after {attempts} attempt(s)")

    def _stop_read(self) -> StmLine | None:
        try:
            raw = self._port.readline()
        except OSError as exc:
            raise LinkClosedError(f"serial read failed: {exc!r}") from exc
        text = raw.decode("ascii", errors="replace").strip() if raw else ""
        return self._classified(text) if text else None

    def _stop_evidence(self, line: StmLine) -> bool:
        """A line read during the STOP procedure. True when it shows a movement around the stop."""
        if line.kind in (LineKind.ACK, LineKind.ERROR):
            self._note_event("crossing_terminal", f"{line.text} crossed the STOP")
        elif line.kind is LineKind.UNKNOWN:
            self._note_event("unknown", f"{line.text} during STOP")
        return line.text.startswith(("MM,EV:START", "MM,EV:END"))

    def _stop_result(self, confirmed: bool, moving: bool, detail: str) -> bool:
        self._txn = None
        if confirmed:
            self.stop_state = "CONFIRMED"
            if moving:
                self._gate_held = True
                self._motion_unknown = False        # stopped (braked and released), but the pose is UNCERTAIN
            if self._state is not LinkState.CLOSED:
                self._state = LinkState.IDLE
            self._note_event("stop_confirmed", f"ACK,STOPPED {detail}"
                                               + ("; motion gate HELD" if self._gate_held else ""))
        else:
            self.stop_state = "UNCONFIRMED"
            self._gate_held = True
            self._latch_motion_unknown(f"STOP unconfirmed {detail}", emit=False)
            self._note_event("stop_unconfirmed", f"no ACK,STOPPED {detail}: SECURE THE ROBOT PHYSICALLY")
        return confirmed

    # -- bench lines ------------------------------------------------------------------------------
    def listen(self, accept: Callable[[StmLine], bool], count: int, timeout_s: float) -> list[StmLine]:
        """Read (idle only, nothing written) until `count` lines that `accept` matches arrived, or `timeout_s`.
        Every other line is handled as an unattributed line (a stray terminal still desynchronises)."""
        self._honour_stop_request()
        if self._state is not LinkState.IDLE:
            raise LinkRefusedError(f"listen refused: link {self._state.value}")
        found: list[StmLine] = []
        deadline = self._clock() + timeout_s
        while len(found) < count and self._clock() < deadline and self._state is LinkState.IDLE:
            self._honour_stop_request()
            line = self._read()
            if line is None:
                continue
            if accept(line):
                found.append(line)
            else:
                self._unattributed(line)
        return found

    def heading(self, settle_s: float = 1.5) -> float | None:
        """Current IMU heading in degrees (`IMU,GET` -> `IMU,Y:<tenths>,GZ:<tenths>,...`),
        or None if the STM did not report one (or the link refused the query).

        The robot is often still coasting out of a turn when ACK arrives, so
        this asks again every 0.1 s until the yaw rate (GZ) has died down, for
        at most `settle_s`, and then returns the latest reading.
        """
        heading = None
        deadline = self._clock() + settle_s
        while True:
            try:
                reply = self.query("IMU,GET", lambda line: line.startswith("IMU,"))
            except (LinkRefusedError, LinkClosedError):
                return heading
            if not reply.ok:
                return heading
            try:
                fields = _fields(reply.line)
                heading, yaw_rate = int(fields["Y"]) / 10, int(fields.get("GZ", "0")) / 10
            except (KeyError, ValueError):
                return heading
            if abs(yaw_rate) <= SETTLED_YAW_RATE_DPS or self._clock() >= deadline:
                return heading
            self._sleep(0.1)

    def apply_profile(self, stm_setup: dict) -> str | None:
        """Set and verify the STM motion profile before the first MOV.

        Returns None on success, otherwise the reason to abort. Any reply other
        than the exact expected one aborts; nothing is retried.
        """
        steps = (
            (stm_setup["set_command"], stm_setup["expected_set_reply"]),
            (stm_setup["verify_command"], stm_setup["expected_verify_reply"]),
        )
        for command, expected in steps:
            try:
                reply = self.query(command, lambda line: line.startswith("PROFILE,"))
            except (LinkRefusedError, LinkClosedError) as exc:
                return f"{command}: {exc}"
            if not reply.ok:
                return f"{command}: {reply.line or 'no reply'}"
            if reply.line != expected:
                return f"{command}: expected {expected}, got {reply.line}"
        return None


def open_serial(device: str, baud: int = 115200) -> SerialPort:
    import serial  # pyserial; imported here so tests need no hardware

    return serial.Serial(device, baud, timeout=0.1, write_timeout=1.0)
