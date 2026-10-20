"""`MOV,FIRL` / `MOV,FIRR`: the IR-terminated forward move (PROTOCOL.md §4.6, §4.7, §6.2.5).

PURE LOGIC, no I/O. The link (`stm_link.send_sensor_motion`) carries the transaction; this module
validates the command, gives its Pi timeout and turns the lines the transaction collected into one
outcome. The spellings here are the ones PROTOCOL.md freezes; nothing is invented.

Normal success needs the complete ORDERED completion record (D-56), every `D` equal to the command's,
all read after the write and before the terminal:

    IREDGE,D:<FIRL|FIRR>,R:EDGE,...,CC:<n >= 0>,...
    MM,EV:END,D:<FIRL|FIRR>,T:<ms>,RESULT:OK,DL:<n>,DR:<n>
    ACK

A plain `ACK` without that record (no `IREDGE`, no `END`, an `END` without `DL`/`DR`, a disagreeing
`D`/`R`/`RESULT`, the wrong order) is a PROTOCOL_ERROR with the pose UNKNOWN. In particular firmware
that predates `DL`/`DR` (Phase 3b.4a; the firmware in this repository prints
`MM,EV:END,D:..,T:..,RESULT:..` only) makes every FIR success a PROTOCOL_ERROR: it fails safe
(PROTOCOL.md §4.7 "Old firmware").

Capability (D-57 / PD-02): implemented on the Pi for host and simulator use; NOT bench validated, and
NO hardware authority. The mission gate (`task2.require_emergency_stop`) keeps every hardware mission
off the robot; nothing in this module may be taken as permission to command FIR on the robot.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

import config
from stm_protocol import MotionResult, pose_known_after

FIR_PATTERN = re.compile(r"^MOV,(FIRL|FIRR),([1-9][0-9]{0,2})$")
FIR_MAX_CM = 999
FIR_COMMAND = {"L": "FIRL", "R": "FIRR"}          # sensor side -> direction token
STM_DEADLINE_BASE_S = 5.0                          # §6.2.5 / D-66: STM deadline 5 s + 0.15 s per max_cm
STM_DEADLINE_PER_CM_S = 0.15
STM_READY_WAIT_S = 0.3
PI_MARGIN_S = 3.0                                  # PROVISIONAL (D-66), tuned during hardware validation
STOPPED_REPLY = "ACK,STOPPED"                      # §4.2.1: terminal of a stop, never a success

IREDGE_PATTERN = re.compile(
    r"^IREDGE,D:(FIRL|FIRR),R:(EDGE|NO_OBSTACLE|NO_EDGE|SENSOR),SA:(0|1),"
    r"SC:(-1|0|[1-9][0-9]{0,9}),SR:(-1|0|[1-9][0-9]{0,9}),CC:(-1|0|[1-9][0-9]{0,9}),CR:(-1|0|[1-9][0-9]{0,9}),"
    r"UN:(0|[1-9][0-9]{0,9}),AMB:(0|[1-9][0-9]{0,9}),MISS:(0|[1-9][0-9]{0,9}),RDY:(0|[1-9][0-9]{0,9}),"
    r"W:(FAULT|READY|DATA|CFG|NA)$")
# §4.7 strict grammar, restricted to the FIR directions.
END_PATTERN = re.compile(
    r"^MM,EV:END,D:(FIRL|FIRR),T:(0|[1-9][0-9]{0,9}),RESULT:([A-Z_]+),"
    r"DL:(-?(?:0|[1-9][0-9]{0,9})),DR:(-?(?:0|[1-9][0-9]{0,9}))$")
INT32_MIN, INT32_MAX = -2**31, 2**31 - 1
# Today's firmware END (no DL/DR). Accepted ONLY with `legacy_travel=True` (supervised runs, D-76): the
# travel then comes from the last `MM,D:` sample's LP/RP (wheel displacement counts since the start, sampled
# up to ~100 ms before braking ends) and the pose is never trusted.
LEGACY_END_PATTERN = re.compile(r"^MM,EV:END,D:(FIRL|FIRR),T:(0|[1-9][0-9]{0,9}),RESULT:([A-Z_]+)$")
MMD_LP_RP = re.compile(r"^MM,D:(?:FIRL|FIRR),.*,LP:(-?[0-9]{1,10}),.*,RP:(-?[0-9]{1,10}),")

# The firmware's decision (IREDGE R) and END RESULT that go with each ERR terminal (§4.6 table).
IR_ERRORS = {
    "ERR,IR_NO_OBSTACLE": ("NO_WALL", "NO_OBSTACLE", "IR_NO_OBSTACLE"),
    "ERR,IR_NO_EDGE": ("MAX_TRAVEL", "NO_EDGE", "IR_NO_EDGE"),
    "ERR,IR_SENSOR": ("SENSOR_FAULT", "SENSOR", "IR_SENSOR"),
}


@dataclass(frozen=True)
class Capability:
    implemented: bool
    bench_validated: bool
    hardware_authority: bool
    reason: str


FIR_CAPABILITY = Capability(
    implemented=True, bench_validated=False, hardware_authority=False,
    reason="FIR hardware authority is blocked (D-57 / PD-02: a disconnected or floating IR input can read as "
           "CLEAR); host and simulator use only")


def fir_command(sensor: str, max_cm: int) -> str:
    """`MOV,FIRL,<max_cm>` for the LEFT sensor, `MOV,FIRR,<max_cm>` for the RIGHT. `max_cm` must be an
    int 1-999: it is the maximum forward travel, never an obstacle length, and is never formatted from a
    float. Raises ValueError otherwise (nothing is sent)."""
    if sensor not in FIR_COMMAND:
        raise ValueError(f"sensor must be 'L' or 'R', got {sensor!r}")
    if not isinstance(max_cm, int) or isinstance(max_cm, bool) or not 1 <= max_cm <= FIR_MAX_CM:
        raise ValueError(f"max_cm must be an int 1-{FIR_MAX_CM}, got {max_cm!r}")
    command = f"MOV,{FIR_COMMAND[sensor]},{max_cm}"
    validate_fir(command)
    return command


def validate_fir(command: str) -> tuple[str, int]:
    """Check a FIR string against the frozen grammar. Returns (direction, max_cm)."""
    match = FIR_PATTERN.fullmatch(command)
    if match is None:
        raise ValueError(f"not a valid STM FIR command: {command!r}")
    return match.group(1), int(match.group(2))


def fir_timeout_s(command: str) -> float:
    """Pi timeout (§6.2.5, D-66): STM deadline + STM ready wait + margin."""
    _, max_cm = validate_fir(command)
    return STM_DEADLINE_BASE_S + STM_DEADLINE_PER_CM_S * max_cm + STM_READY_WAIT_S + PI_MARGIN_S


@dataclass(frozen=True)
class IrEdgeLine:
    direction: str
    result: str
    started_visible: bool
    seen_counts: int
    clear_counts: int
    text: str


@dataclass(frozen=True)
class EndRecord:
    direction: str
    elapsed_ms: int
    result: str
    dl: int
    dr: int
    text: str


def parse_iredge(text: str) -> IrEdgeLine | None:
    match = IREDGE_PATTERN.fullmatch(text)
    if match is None:
        return None
    g = match.groups()
    return IrEdgeLine(g[0], g[1], g[2] == "1", int(g[3]), int(g[5]), text)


def parse_end(text: str) -> EndRecord | None:
    """A §4.7 `MM,EV:END` line with `DL`/`DR`; None if it does not match exactly (an END without
    `DL`/`DR`, `-0`, a value outside int32, any extra or missing key)."""
    match = END_PATTERN.fullmatch(text)
    if match is None:
        return None
    direction, elapsed, result, dl, dr = match.groups()
    values = [int(dl), int(dr)]
    if any(v == "-0" for v in (dl, dr)) or not all(INT32_MIN <= v <= INT32_MAX for v in values) \
            or int(elapsed) > INT32_MAX:
        return None
    return EndRecord(direction, int(elapsed), result, values[0], values[1], text)


@dataclass
class FirOutcome:
    """What one FIR transaction proved.

    status       EDGE, NO_WALL, MAX_TRAVEL, SENSOR_FAULT (the `ir.EdgeOutcome` values the mission
                 understands), or STOPPED, TIMEOUT, PROTOCOL_ERROR, ERROR (which it treats as failures);
    pose_known   False whenever the robot may have moved without a trusted record;
    travel_cm    brake-end travel from `DL`/`DR` (D-58: not guaranteed final physical travel), 0.0 when
                 no valid END was received;
    edge_at_cm   travel at the edge confirmation (`IREDGE` CC), only for EDGE;
    problems     why a record was rejected (empty for a clean outcome).
    """

    status: str
    pose_known: bool
    travel_cm: float = 0.0
    edge_at_cm: float | None = None
    line: str = ""
    problems: list[str] = field(default_factory=list)


def _counts_to_cm(counts: float) -> float:
    return counts / config.COUNTS_PER_CM


def _legacy_end(text: str, lines: list[str]) -> EndRecord | None:
    match = LEGACY_END_PATTERN.fullmatch(text)
    samples = [MMD_LP_RP.match(line) for line in lines if line.startswith("MM,D:")]
    samples = [s for s in samples if s is not None]
    if match is None or not samples:
        return None
    lp, rp = int(samples[-1].group(1)), int(samples[-1].group(2))
    return EndRecord(match.group(1), int(match.group(2)), match.group(3), lp, rp, text)


def evaluate(command: str, result: MotionResult, legacy_travel: bool = False) -> FirOutcome:
    """Turn the lines of one FIR transaction into an outcome (PROTOCOL.md §4.6 table and D-56). With
    `legacy_travel` (supervised runs only, D-76) an END without DL/DR is accepted with the travel taken from
    the last MM,D sample, and the pose is reported UNKNOWN even for an EDGE."""
    direction, _ = validate_fir(command)
    lines = [line.text for line in result.telemetry]
    final = result.final.text if result.final is not None else ""
    started = result.motion_started
    problems: list[str] = []

    iredge_at = [(i, text) for i, text in enumerate(lines) if text.startswith("IREDGE,")]
    end_at = [(i, text) for i, text in enumerate(lines) if text.startswith("MM,EV:END")]
    iredges = [(i, parse_iredge(text), text) for i, text in iredge_at]
    ends = [(i, parse_end(text) or (_legacy_end(text, lines[:i]) if legacy_travel else None), text)
            for i, text in end_at]
    for _, parsed, text in iredges:
        if parsed is None:
            problems.append(f"malformed IREDGE: {text}")
        elif parsed.direction != direction:
            problems.append(f"IREDGE for {parsed.direction}, command was {direction}")
    for _, parsed, text in ends:
        if parsed is None:
            problems.append(f"END without a valid DL/DR record (§4.7): {text}")
        elif parsed.direction != direction:
            problems.append(f"END for {parsed.direction}, command was {direction}")
    if len(iredges) > 1:
        problems.append(f"{len(iredges)} IREDGE lines, expected at most one")
    if len(ends) > 1:
        problems.append(f"{len(ends)} END lines, expected at most one")

    end = ends[0][1] if len(ends) == 1 else None
    travel = _counts_to_cm((end.dl + end.dr) / 2) if end is not None and end.direction == direction else 0.0

    if result.timed_out or result.final is None:
        return FirOutcome("TIMEOUT", False, travel, None, "", problems + ["no terminal reply before the Pi deadline"])

    if final == STOPPED_REPLY:
        # a STOP crossed the move (or it was still waiting for the sensor): never a success, pose uncertain
        return FirOutcome("STOPPED", False, travel, None, final, problems)

    if final == "ACK":
        if not started:
            problems.append("ACK without MM,EV:START")
        if len(iredges) != 1 or iredges[0][1] is None:
            problems.append("ACK without exactly one valid IREDGE")
        if len(ends) != 1 or ends[0][1] is None:
            problems.append("ACK without exactly one valid END record")
        if not problems:
            (edge_index, edge, _), (end_index, end_rec, _) = iredges[0], ends[0]
            if edge.result != "EDGE":
                problems.append(f"ACK with IREDGE R:{edge.result}")
            if edge.clear_counts < 0:
                problems.append("ACK with IREDGE CC:-1 (edge never confirmed)")
            if end_rec.result != "OK":
                problems.append(f"ACK with END RESULT:{end_rec.result}")
            if edge_index > end_index:
                problems.append("IREDGE arrived after END")
        if problems:
            return FirOutcome("PROTOCOL_ERROR", False, travel, None, final, problems)
        legacy = parse_end(ends[0][2]) is None
        return FirOutcome("EDGE", not legacy, travel, _counts_to_cm(iredges[0][1].clear_counts), final,
                          ["legacy END without DL/DR: travel from the last MM,D sample (D-76)"] if legacy else [])

    if final in IR_ERRORS:
        status, decision, end_result = IR_ERRORS[final]
        for _, parsed, _ in iredges:
            if parsed is not None and parsed.result != decision:
                problems.append(f"{final} with IREDGE R:{parsed.result}")
        for _, parsed, _ in ends:
            if parsed is not None and parsed.result != end_result:
                problems.append(f"{final} with END RESULT:{parsed.result}")
        if final == "ERR,IR_SENSOR":
            if not started and ends:
                problems.append("END without MM,EV:START")
            if problems:
                return FirOutcome("PROTOCOL_ERROR", False, travel, None, final, problems)
            # from the ready wait nothing moved (no START): the pose stays known
            return FirOutcome(status, not started, travel, None, final, [])
        if not started:
            problems.append(f"{final} without MM,EV:START")
        if problems:
            return FirOutcome("PROTOCOL_ERROR", False, travel, None, final, problems)
        return FirOutcome(status, False, travel, None, final, [])

    # a generic ERR (BAD_CMD, BUSY, PID_NOT_SET, IMU_NOT_READY, MOTION_TIMEOUT, ...): §5 rules
    known = pose_known_after(result.final, started) and not iredges and not ends
    return FirOutcome("ERROR", known, travel, None, final, problems)
