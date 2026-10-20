"""RPi <-> STM32 serial link (protocol from RTOSV/Core/Src/vision_rx.c and app_comm.c).

One command in flight at a time. For MOV, only the exact lines `ACK`,
`ACK,STOPPED` and `ERR,...` are terminal; every other line (`MM,`, `AK,`,
`TB,`, `MD,`, `USPARK,STOP,...`, ...) is telemetry: it is written unchanged to
the raw log (if one is given) and summarised as one line per move (heading and
wheel travel from the `MM,` samples). `ACK,STOPPED` is a stop (a manual `STOP`
or an automatic ultrasonic parking stop, PROTOCOL.md sections 3 and 3A): it ends
the MOV at once with status "stopped", which is never success and leaves the
pose unknown. A command whose outcome is unknown is never resent.

The serial port itself (open/close, ASCII line write, byte buffering and line
framing, disconnects, shutdown) lives in `uart.py`; this module is the STM
protocol on top of it. A transport failure ends the command with a non-ok
`StmReply` (status "disconnected", "write_failed", "protocol" or "shutdown")
instead of an exception, so callers keep using `reply.ok`.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol, TextIO

from uart import (UartCommandError, UartConfig, UartDisconnectedError, UartError, UartFramingError,
                  UartShutdownError, UartTransport, UartWriteError, open_uart)

MOV_PATTERN = re.compile(r"^MOV,(F|B|FL|FR|BL|BR),(\d{1,3})$")
MAX_LINE_CHARS = 31          # STM rejects longer lines with ERR,BAD_CMD
STRAIGHT_MAX_CM = 999
TURN_MAX_DEG = 360
RPI_TIMEOUT_MARGIN_S = 3.0   # on top of the STM's own deadline
COUNTS_PER_CM = 75.0         # firmware DEFAULT_MOVCAL_SLOPE_COUNTS_PER_CM (rough cm only)
TURN_SIGN = {"FL": 1, "BL": 1, "FR": -1, "BR": -1}   # heading change per degree (PROTOCOL.md §3)
MAX_HEADING_FIX_DEG = 5.0    # larger errors are capped, not chased
SETTLED_YAW_RATE_DPS = 1.0   # IMU,GET is trusted once |GZ| is at most this
MAX_MALFORMED_LINES = 5      # garbled/overlong lines tolerated in one exchange before giving up
PROFILE_NAMES = ("PRECISION", "STANDARD")   # PROTOCOL.md §4.1; CUSTOM is reported, never set
# Rejections the STM sends before it starts moving; with no `MM,EV:START` seen the pose is unchanged.
REJECTED_BEFORE_MOTION = ("ERR,BAD_CMD", "ERR,PID_NOT_SET", "ERR,IMU_NOT_READY")
ACK_LINE = "ACK"
STOPPED_LINE = "ACK,STOPPED"          # terminal "stopped" (manual STOP or USPARK), PROTOCOL.md 3.3, 3A.3
USPARK_OK_LINE = "USPARK,OK"
USPARK_STOP_PREFIX = "USPARK,STOP,"   # non-terminal cause line: R:<cause>,D:<mm|NA>,V:<counts/s>,FM:<hex>
USPARK_FAULT_PREFIX = "USPARK,FAULT,"  # non-terminal, once per fault episode: FM:<hex>
USPARK_STOP_REASONS = ("NEAR", "INVALID", "NOUPDATE", "FAULT")
# USPARK,GET reply (PROTOCOL.md 3A.1); TH/RA/AR are fixed firmware thresholds, so any number is accepted.
USPARK_GET_PATTERN = re.compile(
    r"^USPARK,E:([01]),ST:(DISABLED|UNARMED|ARMED|NEAR1|NEAR2|LATCHED),CN:([01]),"
    r"FM:([0-9A-Fa-f]{1,8}),TH:(\d{1,4}),RA:(\d{1,4}),AR:(\d{1,4})$")


class SerialPort(Protocol):
    """Legacy line-oriented port (what `StmLink` took before `uart.py`); still accepted."""

    def write(self, data: bytes) -> int: ...
    def readline(self) -> bytes: ...
    def reset_input_buffer(self) -> None: ...


@dataclass
class StmReply:
    """Outcome of one command.

    status is "ok", "error", "stopped" or "timeout", or, when the serial link
    itself failed, "disconnected", "write_failed", "protocol" (too many garbled
    lines) or "shutdown"; in those cases `line` is a short "UART ..." text and
    `detail` has the cause. "stopped" (MOV only) means the exact line
    `ACK,STOPPED`: the STM braked the movement, after a manual `STOP` or an
    automatic ultrasonic parking stop. It is a terminal result, not success
    (`ok` is False) and the pose is unknown. `motion_started` records whether
    `MM,EV:START` was seen, i.e. whether the robot may have moved before an
    error (and so whether its pose is still known).

    The USPARK cause lines stay in `telemetry` (and in the raw log) like every
    other non-terminal line; `uspark_stop_line` / `uspark_fault_line` just point
    at them (None when absent, e.g. a manual STOP) and `uspark_stop_reason` is
    the parsed `R:` value.
    """

    status: str
    line: str = ""
    motion_started: bool = False
    telemetry: list[str] = field(default_factory=list)
    detail: str = ""
    malformed_lines: int = 0
    uspark_stop_line: str | None = None    # "USPARK,STOP,R:...,D:...,V:...,FM:..." seen during a MOV
    uspark_fault_line: str | None = None   # "USPARK,FAULT,FM:..." seen during a MOV

    @property
    def ok(self) -> bool:
        return self.status == "ok"

    @property
    def stopped(self) -> bool:
        """True for the terminal `ACK,STOPPED` (never success; see `status`)."""
        return self.status == "stopped"

    @property
    def uspark_stop_reason(self) -> str | None:
        """NEAR, INVALID, NOUPDATE or FAULT from the `USPARK,STOP` line, else None
        (no such line, or one that does not carry a known reason)."""
        if self.uspark_stop_line is None:
            return None
        reason = _fields(self.uspark_stop_line).get("R")
        return reason if reason in USPARK_STOP_REASONS else None

    @property
    def pose_unknown(self) -> bool:
        """For a MOV: True unless it succeeded or the STM rejected it before moving.

        A timeout, a stop (`ACK,STOPPED`), a lost or failed link, a write that
        may have been partial, and any error after `MM,EV:START` leave the
        outcome ambiguous, so the caller must not assume the robot is where the
        plan says (and must not resend; there are no command ids on the wire).
        Only meaningful for MOV replies.
        """
        if self.ok:
            return False
        if self.status == "stopped":
            return True
        return not (self.status == "error" and not self.motion_started and self.line in REJECTED_BEFORE_MOTION)


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


def parse_uspark_get(line: str) -> dict[str, str] | None:
    """Fields of an exact `USPARK,GET` reply (E, ST, CN, FM, TH, RA, AR as text),
    or None if `line` is not one."""
    match = USPARK_GET_PATTERN.fullmatch(line)
    if match is None:
        return None
    return dict(zip(("E", "ST", "CN", "FM", "TH", "RA", "AR"), match.groups()))


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


class _LinePort:
    """Lets a legacy `readline()` port (a test fake, or any older object) sit
    under `UartTransport`: each `readline()` result is handed over as one read."""

    def __init__(self, port: SerialPort) -> None:
        self._port = port

    def read(self, size: int) -> bytes:
        return self._port.readline()

    def write(self, data: bytes) -> int | None:
        return self._port.write(data)

    def reset_input_buffer(self) -> None:
        self._port.reset_input_buffer()

    def close(self) -> None:
        close = getattr(self._port, "close", None)
        if close is not None:
            close()


def _as_transport(port, clock: Callable[[], float]) -> UartTransport:
    if isinstance(port, UartTransport):
        return port
    if hasattr(port, "read"):                 # a byte-level port such as pyserial's Serial
        return UartTransport(port, clock=clock)
    return UartTransport(_LinePort(port), clock=clock)


class StmLink:
    def __init__(
        self,
        port: UartTransport | SerialPort,
        log: Callable[[str], None] = print,
        clock: Callable[[], float] = time.monotonic,
        raw_log: TextIO | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._uart = _as_transport(port, clock)
        self._log = log
        self._clock = clock
        self._raw_log = raw_log
        self._sleep = sleep

    @property
    def uart(self) -> UartTransport:
        return self._uart

    def note(self, text: str) -> None:
        """Write a marker (e.g. the mission id) into the raw telemetry log."""
        if self._raw_log is not None:
            self._raw_log.write(f"## {text}\n")
            self._raw_log.flush()

    def _raw(self, text: str) -> None:
        if self._raw_log is not None:
            self._raw_log.write(f"{self._clock():.3f} {text}\n")

    def _send_line(self, command: str) -> None:
        # Anything still buffered is stale: the STM is idle between commands (one in
        # flight), so a leftover line must not be mistaken for this command's reply.
        stale = self._uart.discard_input()
        if stale:
            self._raw(f"## discarded {stale} stale bytes before {command}")
        self._uart.send_line(command)
        self._raw(f"<- {command}")
        self._log(f"STM <- {command}")

    @staticmethod
    def _link_failure(reply: StmReply, error: UartError) -> None:
        if isinstance(error, UartShutdownError):
            reply.status, reply.line = "shutdown", "UART shut down"
        elif isinstance(error, UartWriteError):
            reply.status, reply.line = "write_failed", "UART write failed"
        elif isinstance(error, UartDisconnectedError):
            reply.status, reply.line = "disconnected", "UART disconnected"
        else:
            reply.status, reply.line = "protocol", "UART protocol error"
        reply.detail = str(error)

    def _exchange(self, command: str, timeout_s: float, classify: Callable[[str], str | None]) -> StmReply:
        """One complete transaction: send `command`, then read lines until
        `classify` calls one terminal ("ok"/"error"), the time runs out, or the
        link fails. Every other line is telemetry. Nothing is ever resent."""
        reply = StmReply(status="timeout")
        with self._uart.transaction():
            try:
                self._send_line(command)
            except UartCommandError:
                raise              # bad text, nothing was written: a caller bug
            except UartError as error:
                self._link_failure(reply, error)
                return self._finished(command, reply)
            deadline = self._clock() + timeout_s
            while True:
                remaining = deadline - self._clock()
                if remaining <= 0:
                    break
                try:
                    line = self._uart.read_line(remaining)
                except UartFramingError as error:
                    # A garbled line can never count as a reply; keep waiting for a good one.
                    reply.malformed_lines += 1
                    self._raw(f"!! {error}")
                    if reply.malformed_lines >= MAX_MALFORMED_LINES:
                        reply.status, reply.line = "protocol", "UART too many malformed lines"
                        reply.detail = str(error)
                        break
                    continue
                except UartError as error:
                    self._link_failure(reply, error)
                    break
                if not line:
                    continue
                self._raw(f"-> {line}")
                verdict = classify(line)
                if verdict is not None:
                    reply.status, reply.line = verdict, line
                    break
                if line.startswith("MM,EV:START"):
                    reply.motion_started = True
                reply.telemetry.append(line)
        return self._finished(command, reply)

    def _finished(self, command: str, reply: StmReply) -> StmReply:
        shown = reply.line or "TIMEOUT"
        self._log(f"STM -> {shown} ({command})" + (f" [{reply.detail}]" if reply.detail else ""))
        if reply.malformed_lines:
            self._log(f"    {reply.malformed_lines} malformed line(s) ignored")
        return reply

    def move(self, command: str) -> StmReply:
        """Send one MOV and wait for its terminal reply: the exact line `ACK`
        ("ok"), the exact line `ACK,STOPPED` ("stopped": braked by a manual STOP
        or an automatic USPARK stop; not success, pose unknown) or `ERR,...`
        ("error"). Everything before it is telemetry, kept in order; a
        `USPARK,STOP` / `USPARK,FAULT` line is also exposed on the reply.
        The MOV is never resent."""
        timeout_s = mov_timeout_s(command)

        def classify(line: str) -> str | None:
            if line == ACK_LINE:
                return "ok"
            if line == STOPPED_LINE:     # exact token: it also starts with "ACK", but is not completion
                return "stopped"
            if line.startswith("ERR,"):
                return "error"
            return None

        reply = self._exchange(command, timeout_s, classify)
        for line in reply.telemetry:
            if reply.uspark_stop_line is None and line.startswith(USPARK_STOP_PREFIX):
                reply.uspark_stop_line = line
            elif reply.uspark_fault_line is None and line.startswith(USPARK_FAULT_PREFIX):
                reply.uspark_fault_line = line
        summary = summarise_motion(reply.telemetry)
        if summary:
            self._log(f"    {summary}")
        for cause in (reply.uspark_fault_line, reply.uspark_stop_line):
            if cause is not None:
                self._log(f"    {cause}")
        if self._raw_log is not None:
            self._raw_log.flush()
        return reply

    def _uspark(self, command: str, accept: Callable[[str], bool]) -> StmReply:
        """One USPARK control exchange: the exact success line or any `ERR,...`
        (`ERR,BUSY`, `ERR,US_FAULT`, `ERR,BAD_CMD`); any other line is telemetry.
        Sent once and never retried; the caller decides what an error means."""

        def classify(line: str) -> str | None:
            if line.startswith("ERR,"):
                return "error"
            return "ok" if accept(line) else None

        return self._exchange(command, 3.0, classify)

    def uspark_enable(self) -> StmReply:
        """`USPARK,1` -> exactly `USPARK,OK`, or `ERR,BUSY` (a movement is queued,
        running or braking, or a STOP is outstanding) / `ERR,US_FAULT` (ultrasonic
        driver faulted) as an ordinary error reply. Idle only; RAM only on the STM,
        so it must be sent again after any STM reset. Not retried."""
        return self._uspark("USPARK,1", lambda line: line == USPARK_OK_LINE)

    def uspark_disable(self) -> StmReply:
        """`USPARK,0` -> exactly `USPARK,OK` (accepted at any time; it never
        cancels a STOP that was already accepted), or `ERR,...`. Not retried."""
        return self._uspark("USPARK,0", lambda line: line == USPARK_OK_LINE)

    def uspark_get(self) -> StmReply:
        """`USPARK,GET` -> the exact `USPARK,E:<0|1>,ST:<state>,CN:<0|1>,FM:<hex>,
        TH:..,RA:..,AR:..` line (see `parse_uspark_get`), or `ERR,...`. A bare
        `USPARK,OK` or any other line is not an answer to GET. Not retried."""
        return self._uspark("USPARK,GET", lambda line: USPARK_GET_PATTERN.fullmatch(line) is not None)

    def query(self, command: str, accept: Callable[[str], bool], timeout_s: float = 3.0) -> StmReply:
        """Send a setup command (e.g. PROFILE,GET) and wait for a line `accept` matches
        or any `ERR,...`. Unrelated lines are treated as telemetry."""

        def classify(line: str) -> str | None:
            if line.startswith("ERR,"):
                return "error"
            return "ok" if accept(line) else None

        return self._exchange(command, timeout_s, classify)

    def heading(self, settle_s: float = 1.5) -> float | None:
        """Current IMU heading in degrees (`IMU,GET` -> `IMU,Y:<tenths>,GZ:<tenths>,...`),
        or None if the STM did not report one.

        The robot is often still coasting out of a turn when ACK arrives, so
        this asks again every 0.1 s until the yaw rate (GZ) has died down, for
        at most `settle_s`, and then returns the latest reading.
        """
        heading = None
        deadline = self._clock() + settle_s
        while True:
            reply = self.query("IMU,GET", lambda line: line.startswith("IMU,"))
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

        Returns None on success, otherwise the reason to abort. Only the exact
        PROTOCOL.md §4.1 exchanges are sent (PROFILE,PRECISION or PROFILE,STANDARD
        -> PROFILE,OK, then PROFILE,GET -> the same profile name). Any other reply
        (PROFILE,CUSTOM, the wrong name, ERR,..., silence) aborts; nothing is retried.
        """
        try:
            set_command, expected_set = stm_setup["set_command"], stm_setup["expected_set_reply"]
            verify_command, expected_verify = stm_setup["verify_command"], stm_setup["expected_verify_reply"]
        except (KeyError, TypeError):
            return "stm_setup is incomplete"
        name = set_command[len("PROFILE,"):] if str(set_command).startswith("PROFILE,") else None
        if (name not in PROFILE_NAMES or expected_set != "PROFILE,OK" or verify_command != "PROFILE,GET"
                or expected_verify != f"PROFILE,{name}"):
            return f"unsupported stm_setup: {set_command!r} / {verify_command!r}"
        steps = ((set_command, expected_set), (verify_command, expected_verify))
        with self._uart.transaction():   # set and verify stay together
            for command, expected in steps:
                reply = self.query(command, lambda line: line.startswith("PROFILE,"))
                if not reply.ok:
                    return f"{command}: {reply.line or 'no reply'}"
                if reply.line != expected:
                    return f"{command}: expected {expected}, got {reply.line}"
        return None


def open_serial(device: str, baud: int = 115200) -> UartTransport:
    """Open the STM serial device (pyserial, 8N1) as a `UartTransport`.

    Kept under its old name for `task1.py`; raises `uart.UartOpenError`.
    """
    return open_uart(device, UartConfig(baudrate=baud))
