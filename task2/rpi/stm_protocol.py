"""Generic STM line classification and link-state types (Phase 3a).

PURE LOGIC, no I/O. It knows only lines that the checked-out PROTOCOL.md
already freezes (section 4 framing and replies, 4.1 PROFILE setup, 5 and 10
terminal handling). It deliberately contains NO Task 2 sensor operation, no
STOP, no `ACK,STOPPED`, no IR or ultrasonic spelling and no READY/START: those
are Phase 3b decisions (plan section 12). A line it does not know is UNKNOWN,
never a reply.

Terminal rule for a motion (PROTOCOL.md section 4, 10): exactly `ACK`, or an
`ERR,<code>` line. Nothing else ends a motion: not telemetry, not an
`ACK`-prefixed line such as `ACK,STOPPED`, not an unknown line.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class LinkState(Enum):
    IDLE = "IDLE"                          # nothing outstanding; commands accepted
    MOTION_PENDING = "MOTION_PENDING"      # one motion command is in flight
    QUERY_PENDING = "QUERY_PENDING"        # one setup/query command is in flight
    DESYNCHRONIZED = "DESYNCHRONIZED"      # an outcome is unknown; every command is refused
    CLOSED = "CLOSED"                      # the port failed or was closed


class LineKind(Enum):
    ACK = "ACK"                  # exactly `ACK`
    ERROR = "ERROR"              # `ERR,<code>`
    MOTION_EVENT = "MOTION_EVENT"  # `MM,EV:START...` / `MM,EV:END...` (telemetry, never terminal)
    TELEMETRY = "TELEMETRY"      # the frozen telemetry families (PROTOCOL.md section 10)
    QUERY_REPLY = "QUERY_REPLY"  # `PROFILE,...` / `IMU,...`: the reply of a setup/query command
    BENCH = "BENCH"              # a bench-log line (PROTOCOL.md §6.2.2); set by the link, never terminal
    UNKNOWN = "UNKNOWN"          # anything else: logged, never a reply


# PROTOCOL.md section 10: "Any MM,/AK,/TB,/MD,/MS,/HS,/RTG,/MP,/SPC,/STP,/SFF,/TL,/TR,/TS, line".
TELEMETRY_PREFIXES = ("MM,", "AK,", "TB,", "MD,", "MS,", "HS,", "RTG,", "MP,", "SPC,", "STP,", "SFF,",
                      "TL,", "TR,", "TS,")
MOTION_EVENT_PREFIXES = ("MM,EV:START", "MM,EV:END")
QUERY_REPLY_PREFIXES = ("PROFILE,", "IMU,")


@dataclass(frozen=True)
class StmLine:
    text: str
    kind: LineKind
    received_at_s: float   # the Pi's monotonic receipt time


def classify_line(text: str, received_at_s: float) -> StmLine:
    """Classify one received line (already stripped of its terminator). Matching
    is exact where the protocol is exact: only the whole line `ACK` is an ACK."""
    if text == "ACK":
        kind = LineKind.ACK
    elif text.startswith("ERR,") and len(text) > len("ERR,"):
        kind = LineKind.ERROR
    elif text.startswith(MOTION_EVENT_PREFIXES):
        kind = LineKind.MOTION_EVENT
    elif text.startswith(TELEMETRY_PREFIXES):
        kind = LineKind.TELEMETRY
    elif text.startswith(QUERY_REPLY_PREFIXES):
        kind = LineKind.QUERY_REPLY
    else:
        kind = LineKind.UNKNOWN
    return StmLine(text, kind, received_at_s)


# PROTOCOL.md section 5: the ERR codes of a motion command. A code in PRE_MOTION_ERRORS is a rejection (the
# command did not parse, the STM was busy, or PID gains were missing) and IMU_NOT_READY may be a rejection
# before motion. MOTION_TIMEOUT is the STM's own deadline expiring: the robot may have moved.
PRE_MOTION_ERRORS = frozenset({"BAD_CMD", "BUSY", "PID_NOT_SET"})
IMU_NOT_READY = "IMU_NOT_READY"
MOTION_TIMEOUT = "MOTION_TIMEOUT"


def error_code(line: StmLine) -> str | None:
    """The `<code>` of an `ERR,<code>` line, else None."""
    return line.text[len("ERR,"):] if line.kind is LineKind.ERROR else None


def pose_known_after(final: StmLine, motion_started: bool) -> bool:
    """Whether the mission may keep trusting its dead-reckoned pose after this terminal reply
    (PROTOCOL.md section 5). The absence of `MM,EV:START` is never proof that nothing moved on its own:
    START telemetry may have been lost, so it only matters for the codes that the protocol says can be
    a rejection before motion.

      ACK                              known
      ERR,MOTION_TIMEOUT               UNKNOWN, whether or not START was seen
      ERR,IMU_NOT_READY                known only if no START was seen
      ERR,BUSY / BAD_CMD / PID_NOT_SET known only if no START was seen (a START is an anomaly: unknown)
      any other ERR,<code>             UNKNOWN: the protocol does not prove that nothing moved

    A terminal ERR is an attributable outcome; it does not set the unknown-motion latch, and the mission
    aborts on it and respects this value.
    """
    if final.kind is LineKind.ACK:
        return True
    code = error_code(final)
    if code is None or code == MOTION_TIMEOUT:
        return False
    if code == IMU_NOT_READY or code in PRE_MOTION_ERRORS:
        return not motion_started
    return False


def is_terminal(line: StmLine) -> bool:
    """True only for the replies PROTOCOL.md section 4 freezes as terminal for a motion."""
    return line.kind in (LineKind.ACK, LineKind.ERROR)


@dataclass(frozen=True)
class MotionResult:
    """Outcome of one motion command on the wire.

    final           the terminal line, or None on a timeout;
    telemetry       every other line received during the move, in order (unknown lines included);
    motion_started  `MM,EV:START` was seen, i.e. the robot may have moved;
    timed_out       no terminal reply before the Pi deadline (the outcome is UNKNOWN);
    pose_known      the mission may keep trusting its dead-reckoned pose. False after a timeout, and
                    after any error that arrived once motion had started.
    """

    command: str
    final: StmLine | None
    telemetry: tuple[StmLine, ...]
    motion_started: bool
    timed_out: bool
    pose_known: bool

    @property
    def ok(self) -> bool:
        return self.final is not None and self.final.kind is LineKind.ACK


@dataclass(frozen=True)
class LinkEvent:
    """One piece of evidence the link keeps instead of silently discarding it."""

    at_s: float
    kind: str      # "unknown", "unexpected_terminal", "late_terminal", "timeout", "desync", "resync", ...
    text: str


class LinkError(RuntimeError):
    """The link refused or could not carry out a request. Nothing was sent unless stated."""


class LinkBusyError(LinkError):
    """A command is already outstanding (one in flight, PROTOCOL.md section 4)."""


class LinkRefusedError(LinkError):
    """The link is DESYNCHRONIZED: every command is refused until a quiet resynchronisation."""


class MotionOutcomeUnknownError(LinkRefusedError):
    """A movement may be active or have moved and its outcome is unknown (a motion timeout, or a serial
    failure while a movement may have been active). The link refuses everything until a frozen, positively
    confirmed STOP/recovery procedure exists; a quiet serial line proves nothing about the robot."""


class LinkClosedError(LinkError):
    """The port failed or the link was closed."""


class EmergencyStopped(LinkRefusedError):
    """An operator (or the program) requested an emergency stop; the link ran the stop procedure and
    refuses every further movement for its lifetime. The pose is UNCERTAIN."""


@dataclass(frozen=True)
class EmergencyStopCapability:
    """Whether the checked-out protocol freezes AND the Pi implements an emergency STOP.

    The old Task 2 program (task2.py, removed) refused to run a hardware mission unless `available` was True;
    final_run.py does not use this. It is False because PROTOCOL.md freezes no STOP command or reply. Phase 3b
    replaces `EMERGENCY_STOP` below only after the STOP contract is frozen in PROTOCOL.md, implemented in
    the link and tested; no STOP string is defined here.
    """

    available: bool
    reason: str


EMERGENCY_STOP = EmergencyStopCapability(
    available=False,
    reason="the STOP of PROTOCOL.md §4.2.1 is implemented in the link (Ctrl-C / SIGTERM send it) but not yet "
           "bench-validated on the robot (L3); runs stay supervised",
)
