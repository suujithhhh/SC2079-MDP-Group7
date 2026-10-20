"""Final parking with the STM's predictive ultrasonic stop (USPARK), TASK2_USPARK_ALGO_INTEGRATION.md.

PURE LOGIC, no I/O. The link (`stm_link.send_parking_motion`) carries the one final movement; this module
holds the spellings the firmware prints (checked against the authority-build source, `app_comm.c`) and
turns the lines of that transaction into one `ParkingResult`.

    USPARK,0 / USPARK,1        -> USPARK,OK  (USPARK,1 may answer ERR,BUSY or ERR,US_FAULT)
    USPARK,GET                 -> USPARK,E:<0|1>,ST:<state>,CN:<0|1>,FM:<hex>,TH:<mm>,RA:<mm>,AR:<mm>
    cause (evidence, never a terminal):
        USPARK,STOP,R:PRED,D:<mm>,V:<cps>,FM:<hex>,C:<mm|NA>,TH:<mm>,P:<NORM|FAST|OVER>
        USPARK,STOP,R:<FAULT|NOUPDATE|INVALID|NEAR>,D:<mm|NA>,V:<cps>,FM:<hex>      (legacy detector)
        USPARK,FAULT,FM:<hex>
    USPARK,TRACE,...           bench diagnostics: logged, never needed for a decision

PARKED is the ONLY positive result, and only for the complete chain of the final `MOV,F,100`, in order,
after its write and before its terminal (handoff §3.4):

    MM,EV:START,D:F,...
    USPARK,STOP,R:PRED,...,FM:00,...,TH:190,P:NORM     (exactly one cause)
    MM,EV:END,D:F,...,RESULT:STOPPED[,DL:..,DR:..]      (exactly one END)
    ACK,STOPPED                                         (exact)

`ACK,STOPPED` stays a non-success everywhere else, and the planned 100 cm is never applied to the pose.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from enum import Enum

import config
from stm_fir import Capability
from stm_protocol import MotionResult, pose_known_after

STOPPED_REPLY = "ACK,STOPPED"
OK_REPLY = "USPARK,OK"
USPARK_STATES = ("DISABLED", "UNARMED", "ARMED", "NEAR1", "NEAR2", "LATCHED")
_INT = r"(?:0|[1-9][0-9]{0,9})"
_SINT = r"(?:-?(?:0|[1-9][0-9]{0,9}))"
_HEX = r"([0-9A-F]{2,})"
GET_PATTERN = re.compile(rf"^USPARK,E:(0|1),ST:({'|'.join(USPARK_STATES)}),CN:(0|1),FM:{_HEX},"
                         rf"TH:({_INT}),RA:({_INT}),AR:({_INT})$")
PRED_PATTERN = re.compile(rf"^USPARK,STOP,R:PRED,D:({_INT}),V:({_SINT}),FM:{_HEX},C:(NA|{_SINT}),"
                          rf"TH:({_SINT}),P:(NORM|FAST|OVER)$")
LEGACY_PATTERN = re.compile(rf"^USPARK,STOP,R:(FAULT|NOUPDATE|INVALID|NEAR),D:(NA|{_INT}),V:({_SINT}),FM:{_HEX}$")
FAULT_PATTERN = re.compile(rf"^USPARK,FAULT,FM:{_HEX}$")
END_PATTERN = re.compile(rf"^MM,EV:END,D:F,T:{_INT},RESULT:([A-Z_]+)(?:,DL:{_SINT},DR:{_SINT})?$")


class ParkingOutcome(Enum):
    PARKED = "PARKED"                        # the complete PRED/NORM/190 chain: terminal mission success
    NORMAL_COMPLETION = "NORMAL_COMPLETION"  # exact ACK: all 100 cm driven without the expected stop
    PROTECTIVE_STOP = "PROTECTIVE_STOP"      # PRED FAST/OVER or a legacy cause: stopped safely, not accepted
    SENSOR_FAULT = "SENSOR_FAULT"            # USPARK,FAULT, a fault cause or a non-zero fault mask
    PROTOCOL_ERROR = "PROTOCOL_ERROR"        # missing, duplicated, malformed or out-of-order evidence
    TIMEOUT = "TIMEOUT"                      # no terminal reply: outcome unknown
    STM_ERROR = "STM_ERROR"                  # an ERR,... terminal


@dataclass(frozen=True)
class UsParkStatus:
    enabled: bool
    state: str
    confirmed_near: bool
    fault_mask: int
    stop_mm: int
    rearm_mm: int
    arm_mm: int
    line: str


@dataclass(frozen=True)
class UsParkCause:
    reason: str                     # PRED, or the legacy FAULT / NOUPDATE / INVALID / NEAR
    distance_mm: int | None         # filtered distance AT THE TRIGGER, never the final clearance
    speed_cps: int
    fault_mask: int
    predicted_clearance_mm: int | None   # PRED only: the conservative model value, never a measurement
    threshold_mm: int | None
    path: str | None                # PRED only: NORM / FAST / OVER
    line: str


@dataclass
class ParkingResult:
    outcome: ParkingOutcome
    pose_known: bool                # always False after any movement: the 100 cm is never applied
    reason: str
    cause: UsParkCause | None = None
    start_seen: bool = False
    stopped_end_seen: bool = False
    final: str = ""
    telemetry: list[str] = field(default_factory=list)


USPARK_PARKING_CAPABILITY = Capability(
    implemented=True, bench_validated=False, hardware_authority=False,
    reason="Pi side host-tested only; the exact 100 cm staging sequence still needs the guarded three-run "
           "acceptance (handoff §13) on the final integrated binary")


def parse_get(line: str) -> UsParkStatus | None:
    match = GET_PATTERN.fullmatch(line)
    if match is None:
        return None
    e, st, cn, fm, th, ra, ar = match.groups()
    return UsParkStatus(e == "1", st, cn == "1", int(fm, 16), int(th), int(ra), int(ar), line)


def armed_problem(status: UsParkStatus | None) -> str | None:
    """Why a `USPARK,GET` after `USPARK,1` is not ready for the final movement (handoff §3.3), or None."""
    if status is None:
        return "USPARK,GET reply missing or malformed"
    expected = (config.PARK_USPARK_TH_MM, config.PARK_USPARK_RA_MM, config.PARK_USPARK_AR_MM)
    if not status.enabled:
        return f"USPARK not enabled: {status.line}"
    if status.fault_mask != 0:
        return f"USPARK fault mask {status.fault_mask:02X}: {status.line}"
    if (status.stop_mm, status.rearm_mm, status.arm_mm) != expected:
        return f"USPARK thresholds are not {expected}: {status.line}"
    if status.state != "UNARMED" or status.confirmed_near:
        return f"USPARK state {status.state} (CN:{int(status.confirmed_near)}) is not the pre-movement UNARMED: {status.line}"
    return None


def parse_cause(line: str) -> UsParkCause | None:
    match = PRED_PATTERN.fullmatch(line)
    if match is not None:
        d, v, fm, c, th, p = match.groups()
        return UsParkCause("PRED", int(d), int(v), int(fm, 16), None if c == "NA" else int(c), int(th), p, line)
    match = LEGACY_PATTERN.fullmatch(line)
    if match is not None:
        r, d, v, fm = match.groups()
        return UsParkCause(r, None if d == "NA" else int(d), int(v), int(fm, 16), None, None, None, line)
    return None


def evaluate(command: str, result: MotionResult) -> ParkingResult:
    """Classify the final parking transaction (handoff §3.4, §4, §9). Only PARKED is success."""
    lines = [line.text for line in result.telemetry]
    final = result.final.text if result.final is not None else ""
    start_at = [i for i, t in enumerate(lines) if t.startswith("MM,EV:START")]
    end_at = [i for i, t in enumerate(lines) if t.startswith("MM,EV:END")]
    cause_at = [i for i, t in enumerate(lines) if t.startswith("USPARK,STOP,")]
    faults = [t for t in lines if t.startswith("USPARK,FAULT")]
    started = result.motion_started
    causes = [parse_cause(lines[i]) for i in cause_at]
    ends = [END_PATTERN.fullmatch(lines[i]) for i in end_at]
    stopped_end = len(ends) == 1 and ends[0] is not None and ends[0].group(1) == "STOPPED"
    cause = causes[0] if len(causes) == 1 else None

    def done(outcome: ParkingOutcome, why: str, pose_known: bool = False) -> ParkingResult:
        return ParkingResult(outcome, pose_known, why, cause, started, stopped_end, final, lines)

    if result.timed_out or result.final is None:
        return done(ParkingOutcome.TIMEOUT, "no terminal reply before the Pi deadline; outcome unknown")
    if final.startswith("ERR,"):
        known = pose_known_after(result.final, started) and not end_at and not cause_at
        return done(ParkingOutcome.STM_ERROR, f"{final}" + ("" if known else " after the movement started"), known)
    if faults or any(c is not None and (c.fault_mask != 0 or c.reason == "FAULT") for c in causes):
        return done(ParkingOutcome.SENSOR_FAULT, "ultrasonic fault: " + "; ".join(faults + [c.line for c in causes if c]))
    if final == "ACK":
        return done(ParkingOutcome.NORMAL_COMPLETION, "ACK: the full 100 cm was driven without the expected stop")
    # final is exactly ACK,STOPPED
    if not cause_at:
        return done(ParkingOutcome.PROTOCOL_ERROR, "ACK,STOPPED with no USPARK cause: unknown automatic stop")
    if len(cause_at) > 1 or len(end_at) != 1 or len(start_at) != 1:
        return done(ParkingOutcome.PROTOCOL_ERROR, f"expected one START, one cause and one END; got "
                    f"{len(start_at)} / {len(cause_at)} / {len(end_at)}")
    if cause is None:
        return done(ParkingOutcome.PROTOCOL_ERROR, f"malformed USPARK cause: {lines[cause_at[0]]}")
    if ends[0] is None:
        return done(ParkingOutcome.PROTOCOL_ERROR, f"malformed or wrong-direction END: {lines[end_at[0]]}")
    if not stopped_end:
        return done(ParkingOutcome.PROTOCOL_ERROR, f"END is not RESULT:STOPPED: {lines[end_at[0]]}")
    if not start_at[0] < cause_at[0] < end_at[0]:
        return done(ParkingOutcome.PROTOCOL_ERROR, "START, cause and END are out of order")
    if cause.reason != "PRED":
        return done(ParkingOutcome.PROTECTIVE_STOP, f"legacy USPARK stop R:{cause.reason}: safe, not accepted")
    if cause.path != config.PARK_EXPECTED_PATH:
        return done(ParkingOutcome.PROTECTIVE_STOP, f"predictive stop on path {cause.path}: safe, not accepted")
    if cause.threshold_mm != config.PARK_EXPECTED_TRIGGER_MM:
        return done(ParkingOutcome.PROTOCOL_ERROR, f"predictive stop with TH:{cause.threshold_mm}, expected "
                    f"{config.PARK_EXPECTED_TRIGGER_MM} (a different firmware build?)")
    return done(ParkingOutcome.PARKED, "predictive USPARK stop PRED/NORM/190 with a stopped END and ACK,STOPPED")
