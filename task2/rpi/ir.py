"""Side-IR semantics: protocol-independent readings and the edge detector.

`IrReading`, `IrSample`, `classify` and `EdgeDetector` are a semantic model
(D-23). The only code that knows any text is `parse_ir`, the adapter for the
frozen `IR,GET` reply (PROTOCOL.md §4.5, Phase 3b.5a):
`IR,S:<seq>,ST:<OK|WARM|STALE|ERR|FAULT>,L:<mm|NEAR|FAR|NA>,R:<..>,AGE:<ms|NA>,F:<hex>`.

The rules this module enforces (PLAN §6):

  * `NA`, invalid, stale, duplicate-sequence, ambiguous and faulty readings are
    never WALL_GONE. The only inputs that mean WALL_GONE are a fresh
    `VALID_DISTANCE` at or beyond the "absent" threshold and a fresh
    `TRUSTWORTHY_NO_WALL`.
  * An edge is stable valid WALL_PRESENT followed by stable trusted WALL_GONE.
  * `NO_WALL` needs a complete window of fresh trusted GONE samples and no
    PRESENT within the acquisition bound. Untrusted data never produces it.

Time is the Pi's monotonic receipt time (`received_at_s`), never wall-clock
time. How old a reading was when the STM took it cannot be proven until the
final protocol supplies enough information. Sequence numbers are compared as
plain integers: wrap-around is deliberately not modelled before Phase 3.
"""

from __future__ import annotations

import math
import re
from collections import Counter
from dataclasses import dataclass
from enum import Enum

import config


class IrSensor(Enum):
    LEFT = "L"
    RIGHT = "R"


class IrStatus(Enum):
    VALID_DISTANCE = "VALID_DISTANCE"            # a fresh, healthy, numeric distance
    TRUSTWORTHY_NO_WALL = "TRUSTWORTHY_NO_WALL"  # the sensor positively reports no wall in range (not NA)
    INVALID = "INVALID"                          # NA, zero, garbled, out of order: not trustworthy
    STALE = "STALE"                              # too old, or a repeated sequence
    FAULT = "FAULT"                              # the sensor reports a fault


class WallClass(Enum):
    PRESENT = "WALL_PRESENT"
    GONE = "WALL_GONE"
    AMBIGUOUS = "AMBIGUOUS"    # a valid distance between the two thresholds
    UNTRUSTED = "UNTRUSTED"    # INVALID, STALE or FAULT: says nothing about the wall


def _finite_number(name: str, value: float) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number, got {value!r}")
    return float(value)


@dataclass(frozen=True)
class IrReading:
    """One sensor, one sample."""

    status: IrStatus
    distance_cm: float | None = None   # finite and > 0 exactly when VALID_DISTANCE
    coarse: bool = False               # a VALID_DISTANCE beyond the trusted range: "far enough" only (§4.5)
    far: bool = False                  # INVALID because the STM said FAR (nothing within ~40 cm); D-80

    def __post_init__(self) -> None:
        if not isinstance(self.status, IrStatus):
            raise TypeError(f"status must be an IrStatus, got {self.status!r}")
        if self.coarse and self.status is not IrStatus.VALID_DISTANCE:
            raise ValueError("only a VALID_DISTANCE can be coarse")
        if self.far and self.status is not IrStatus.INVALID:
            raise ValueError("only an INVALID reading can be FAR")
        if self.status is IrStatus.VALID_DISTANCE:
            if _finite_number("distance_cm", self.distance_cm if self.distance_cm is not None else math.nan) <= 0:
                raise ValueError(f"a VALID_DISTANCE needs a distance > 0, got {self.distance_cm!r}")
        elif self.distance_cm is not None:
            raise ValueError(f"{self.status.value} carries no distance, got {self.distance_cm!r}")


@dataclass(frozen=True)
class IrSample:
    """One report for both sensors."""

    seq: int | None            # as reported; None when the report has no sequence
    received_at_s: float       # Pi time.monotonic() at receipt (never wall-clock time)
    left: IrReading
    right: IrReading
    age_ms: int | None = None  # the STM's AGE: ms since its newest publication (None if NA or unknown)
    problem: str | None = None # why the report is untrusted as a whole (malformed, stale, no reply, ...)

    def __post_init__(self) -> None:
        _finite_number("received_at_s", self.received_at_s)

    def reading(self, sensor: IrSensor) -> IrReading:
        return self.left if sensor is IrSensor.LEFT else self.right


@dataclass(frozen=True)
class IrThresholds:
    """The WALL_PRESENT / WALL_GONE distances (BD-03, uncalibrated). Always an
    explicit parameter; there is no production default."""

    present_cm: float
    absent_cm: float

    def __post_init__(self) -> None:
        present = _finite_number("present_cm", self.present_cm)
        absent = _finite_number("absent_cm", self.absent_cm)
        if not 0 < present < absent:
            raise ValueError(f"need 0 < present_cm < absent_cm, got {present}, {absent}")

    @classmethod
    def provisional(cls) -> "IrThresholds":
        """The uncalibrated first guesses in config.py, for simulation fixtures
        only. A real run must pass thresholds explicitly."""
        return cls(config.IR_PRESENT_CM, config.IR_ABSENT_CM)


def classify(reading: IrReading, thresholds: IrThresholds) -> WallClass:
    """One (already freshness-checked) reading -> a wall class."""
    if reading.status is IrStatus.VALID_DISTANCE:
        if reading.coarse:      # "far enough" only: never a measured gap, never WALL_PRESENT (§4.5)
            return WallClass.GONE if reading.distance_cm >= thresholds.absent_cm else WallClass.AMBIGUOUS
        if reading.distance_cm <= thresholds.present_cm:
            return WallClass.PRESENT
        if reading.distance_cm >= thresholds.absent_cm:
            return WallClass.GONE
        return WallClass.AMBIGUOUS
    if reading.status is IrStatus.TRUSTWORTHY_NO_WALL:
        return WallClass.GONE
    return WallClass.UNTRUSTED


# --- the edge detector -------------------------------------------------------------
class EdgeState(Enum):
    SEEKING = "SEEKING"
    ARMED = "ARMED"
    DONE = "DONE"


class EdgeOutcome(Enum):
    INPUT_INVALID = "INPUT_INVALID"
    SENSOR_FAULT = "SENSOR_FAULT"
    SENSOR_UNTRUSTWORTHY = "SENSOR_UNTRUSTWORTHY"
    SAFETY_LIMIT = "SAFETY_LIMIT"
    EDGE = "EDGE"
    NO_WALL = "NO_WALL"
    MAX_TRAVEL = "MAX_TRAVEL"


class UntrustReason(Enum):
    INVALID = "INVALID"
    STALE_AGE = "STALE_AGE"
    DUPLICATE_SEQ = "DUPLICATE_SEQ"
    DECREASING_SEQ = "DECREASING_SEQ"
    MISSING_SEQ = "MISSING_SEQ"
    FUTURE_TIME = "FUTURE_TIME"
    AMBIGUOUS = "AMBIGUOUS"
    SILENCE = "SILENCE"
    INCONCLUSIVE_ACQUISITION = "INCONCLUSIVE_ACQUISITION"


# Terminal precedence when several conditions hold in one update (PLAN §4):
# the lowest rank wins. A safety fault is never hidden behind a generic MAX_TRAVEL.
_RANK = {
    EdgeOutcome.INPUT_INVALID: 1,
    EdgeOutcome.SENSOR_FAULT: 2,
    EdgeOutcome.SENSOR_UNTRUSTWORTHY: 3,
    EdgeOutcome.SAFETY_LIMIT: 4,
    EdgeOutcome.EDGE: 5,
    EdgeOutcome.NO_WALL: 6,
    EdgeOutcome.MAX_TRAVEL: 7,
}


@dataclass(frozen=True)
class EdgeDetectorConfig:
    """Every number is a parameter (BD-03..06); none has a production default."""

    thresholds: IrThresholds
    present_samples: int         # N consecutive fresh PRESENT samples arm the detector
    gone_samples: int            # M consecutive fresh trusted GONE samples confirm an edge
    no_wall_samples: int         # a trusted GONE window this long, at the acquisition limit, is NO_WALL
    max_age_s: float             # a sample older than this on receipt is STALE
    max_silence_s: float         # no sample for longer than this is SENSOR_UNTRUSTWORTHY
    acquisition_limit_cm: float  # stable PRESENT must be acquired within this travel
    max_untrusted_samples: int   # more untrusted samples in a row than this: SENSOR_UNTRUSTWORTHY
    max_untrusted_cm: float      # more travel than this since the last trusted sample: SENSOR_UNTRUSTWORTHY

    def __post_init__(self) -> None:
        if not isinstance(self.thresholds, IrThresholds):
            raise TypeError("thresholds must be IrThresholds")
        for name in ("present_samples", "gone_samples", "no_wall_samples", "max_untrusted_samples"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be an integer >= 1, got {value!r}")
        for name in ("max_age_s", "max_silence_s", "acquisition_limit_cm", "max_untrusted_cm"):
            if _finite_number(name, getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be > 0, got {getattr(self, name)!r}")


@dataclass(frozen=True)
class EdgeLimits:
    """The two bounds are SEPARATE. The search limit (BD-02) is how far an
    allowed end can be; the safety limit is the collision-safe trigger distance
    (it already has the stopping reserve taken off). None = no independent
    safety trigger exists for this leg."""

    search_limit_cm: float
    safety_limit_cm: float | None = None

    def __post_init__(self) -> None:
        if _finite_number("search_limit_cm", self.search_limit_cm) <= 0:
            raise ValueError(f"search_limit_cm must be > 0, got {self.search_limit_cm!r}")
        if self.safety_limit_cm is not None and _finite_number("safety_limit_cm", self.safety_limit_cm) <= 0:
            raise ValueError(f"safety_limit_cm must be > 0 or None, got {self.safety_limit_cm!r}")


class EdgeDetector:
    """The Pi-side reference model of the STM edge-follow operation (the real
    operation is Phase 3). Fed samples with the travel at which each arrived."""

    def __init__(self, active: IrSensor, config_: EdgeDetectorConfig, limits: EdgeLimits, start_s: float) -> None:
        if not isinstance(active, IrSensor):
            raise TypeError("active must be an IrSensor")
        if not isinstance(config_, EdgeDetectorConfig):
            raise TypeError("config_ must be an EdgeDetectorConfig")
        self.active = active
        self.cfg = config_
        self.reset(limits, start_s)

    def reset(self, limits: EdgeLimits, start_s: float) -> None:
        """Back to SEEKING with every counter cleared."""
        if not isinstance(limits, EdgeLimits):
            raise TypeError("limits must be EdgeLimits")
        _finite_number("start_s", start_s)
        self.limits = limits
        self.state = EdgeState.SEEKING
        self.outcome: EdgeOutcome | None = None
        self.reason: UntrustReason | None = None
        self.armed_at_cm: float | None = None
        self.edge_at_cm: float | None = None
        self.inactive_issues: Counter[IrStatus] = Counter()
        self._present_run = 0
        self._present_start: float | None = None
        self._gone_run = 0
        self._gone_start: float | None = None
        self._untrusted_n = 0
        self._last_trusted_cm = 0.0
        self._last_seq: int | None = None
        self._last_sample_s = start_s
        self._last_seen_s = start_s
        self._last_travel = 0.0

    # -- inputs ------------------------------------------------------------------
    def update(self, sample: IrSample, travel_cm: float, now_s: float) -> EdgeState:
        if self.state is EdgeState.DONE:
            return self.state
        if self._bad_input(travel_cm, now_s):
            return self._finish([(EdgeOutcome.INPUT_INVALID, None)], None)
        conds: list[tuple[EdgeOutcome, UntrustReason | None]] = []
        if now_s - self._last_sample_s > self.cfg.max_silence_s:
            conds.append((EdgeOutcome.SENSOR_UNTRUSTWORTHY, UntrustReason.SILENCE))
        self._last_sample_s = now_s
        self._last_seen_s, self._last_travel = now_s, travel_cm
        pending_edge = self._count(sample, travel_cm, now_s, conds)
        self._limit_conditions(travel_cm, conds)
        return self._finish(conds, pending_edge)

    def check(self, travel_cm: float, now_s: float) -> EdgeState:
        """Silence and the limits, with no new sample."""
        if self.state is EdgeState.DONE:
            return self.state
        if self._bad_input(travel_cm, now_s):
            return self._finish([(EdgeOutcome.INPUT_INVALID, None)], None)
        self._last_seen_s, self._last_travel = now_s, travel_cm
        conds: list[tuple[EdgeOutcome, UntrustReason | None]] = []
        if now_s - self._last_sample_s > self.cfg.max_silence_s:
            conds.append((EdgeOutcome.SENSOR_UNTRUSTWORTHY, UntrustReason.SILENCE))
        self._limit_conditions(travel_cm, conds)
        return self._finish(conds, None)

    # -- internals ---------------------------------------------------------------
    def _bad_input(self, travel_cm: float, now_s: float) -> bool:
        try:
            _finite_number("travel_cm", travel_cm)
            _finite_number("now_s", now_s)
        except ValueError:
            return True
        return travel_cm < self._last_travel or now_s < self._last_seen_s

    def _assess(self, sample: IrSample, now_s: float):
        """Sample-level freshness first, then each sensor. Returns
        {sensor: (status, distance, reason)}. FAULT is never hidden."""
        problem: tuple[IrStatus, UntrustReason] | None = None
        seq = sample.seq
        valid_seq = isinstance(seq, int) and not isinstance(seq, bool) and seq >= 0
        if sample.received_at_s > now_s:
            problem = (IrStatus.INVALID, UntrustReason.FUTURE_TIME)
        elif not valid_seq:
            problem = (IrStatus.INVALID, UntrustReason.MISSING_SEQ)
        elif self._last_seq is not None and seq == self._last_seq:
            problem = (IrStatus.STALE, UntrustReason.DUPLICATE_SEQ)
        elif self._last_seq is not None and seq < self._last_seq:
            problem = (IrStatus.INVALID, UntrustReason.DECREASING_SEQ)
        elif now_s - sample.received_at_s > self.cfg.max_age_s:
            problem = (IrStatus.STALE, UntrustReason.STALE_AGE)
        if valid_seq and sample.received_at_s <= now_s and (self._last_seq is None or seq > self._last_seq):
            self._last_seq = seq
        result = {}
        for sensor in IrSensor:
            reading = sample.reading(sensor)
            if reading.status is IrStatus.FAULT:
                result[sensor] = (IrStatus.FAULT, None, None)
            elif problem is not None:
                result[sensor] = (problem[0], None, problem[1])
            elif reading.status is IrStatus.STALE:
                result[sensor] = (IrStatus.STALE, None, UntrustReason.STALE_AGE)
            elif reading.status is IrStatus.INVALID:
                result[sensor] = (IrStatus.INVALID, None, UntrustReason.INVALID)
            else:
                result[sensor] = (reading.status, reading.distance_cm, None)
        return result

    def _count(self, sample: IrSample, travel: float, now_s: float, conds) -> float | None:
        """Apply one sample to the counters (transition table rows 2, 4-6, 10). A
        sample at exactly the acquisition limit is counted; one strictly beyond it
        while SEEKING is checked for sequence and FAULT only, so the decision at the
        limit uses just the evidence gathered at or before it."""
        assessed = self._assess(sample, now_s)
        for sensor, (status, _, _) in assessed.items():
            if sensor is not self.active and status in (IrStatus.INVALID, IrStatus.STALE, IrStatus.FAULT):
                self.inactive_issues[status] += 1      # row 10: never changes the detector state
        status, distance, reason = assessed[self.active]
        if status is IrStatus.FAULT:
            self._reset_runs()
            conds.append((EdgeOutcome.SENSOR_FAULT, None))
            return None
        if self.state is EdgeState.SEEKING and travel > self.cfg.acquisition_limit_cm:
            return None          # beyond the acquisition limit: this sample is no evidence for arming or a GONE window
        wall = classify(IrReading(status, distance), self.cfg.thresholds)
        pending_edge = None
        if wall is WallClass.PRESENT:
            self._gone_run, self._gone_start = 0, None
            self._untrusted_n, self._last_trusted_cm = 0, travel
            if self.state is EdgeState.SEEKING:
                if self._present_run == 0:
                    self._present_start = travel
                self._present_run += 1
                if self._present_run >= self.cfg.present_samples:
                    self.state = EdgeState.ARMED
                    self.armed_at_cm = self._present_start
                    self._reset_runs()
        elif wall is WallClass.GONE:
            self._present_run, self._present_start = 0, None
            self._untrusted_n, self._last_trusted_cm = 0, travel
            if self._gone_run == 0:
                self._gone_start = travel
            self._gone_run += 1
            if self.state is EdgeState.ARMED and self._gone_run >= self.cfg.gone_samples:
                conds.append((EdgeOutcome.EDGE, None))
                pending_edge = self._gone_start
        else:                                            # AMBIGUOUS or UNTRUSTED
            self._reset_runs()
            self._untrusted_n += 1
            if (self._untrusted_n > self.cfg.max_untrusted_samples
                    or travel - self._last_trusted_cm > self.cfg.max_untrusted_cm):
                conds.append((EdgeOutcome.SENSOR_UNTRUSTWORTHY, reason or UntrustReason.AMBIGUOUS))
        return pending_edge

    def _reset_runs(self) -> None:
        self._present_run, self._present_start = 0, None
        self._gone_run, self._gone_start = 0, None

    def _limit_conditions(self, travel: float, conds) -> None:
        if self.state is EdgeState.SEEKING and travel >= self.cfg.acquisition_limit_cm:
            if self._gone_run >= self.cfg.no_wall_samples:
                conds.append((EdgeOutcome.NO_WALL, None))
            else:
                conds.append((EdgeOutcome.SENSOR_UNTRUSTWORTHY, UntrustReason.INCONCLUSIVE_ACQUISITION))
        if self.limits.safety_limit_cm is not None and travel >= self.limits.safety_limit_cm:
            conds.append((EdgeOutcome.SAFETY_LIMIT, None))
        if travel >= self.limits.search_limit_cm:
            conds.append((EdgeOutcome.MAX_TRAVEL, None))

    def _finish(self, conds, pending_edge: float | None) -> EdgeState:
        if not conds:
            return self.state
        outcome, reason = min(conds, key=lambda c: _RANK[c[0]])      # stable: first of equal rank wins
        self.state, self.outcome, self.reason = EdgeState.DONE, outcome, reason
        if outcome is EdgeOutcome.EDGE:
            self.edge_at_cm = pending_edge
        return self.state


# --- the IR,GET adapter (PROTOCOL.md §4.5) ---------------------------------------------
IR_GET_PATTERN = re.compile(
    r"^IR,S:(0|[1-9][0-9]{0,9}),ST:(OK|WARM|STALE|ERR|FAULT),L:(NEAR|FAR|NA|[1-9][0-9]{2}),"
    r"R:(NEAR|FAR|NA|[1-9][0-9]{2}),AGE:(NA|0|[1-9][0-9]{0,4}),F:([0-9A-F]{2,})$")
IR_TRUSTED_MM = (100, 200)    # [USER-REPORTED] trusted range, from the sensor face
IR_COARSE_MAX_MM = 400        # 201-400: VALID_DISTANCE flagged coarse
IR_FRESH_BIT = 0x40           # F bit: "the newest publication is not a fresh good reading"
_NOT_OK = {"WARM": IrStatus.INVALID, "STALE": IrStatus.STALE, "ERR": IrStatus.INVALID, "FAULT": IrStatus.FAULT}


def untrusted_sample(received_at_s: float, problem: str, status: IrStatus = IrStatus.INVALID,
                     seq: int | None = None, age_ms: int | None = None) -> IrSample:
    """Both sensors untrusted, with the reason (a malformed, stale or missing reply)."""
    return IrSample(seq, received_at_s, IrReading(status), IrReading(status), age_ms, problem)


def parse_ir(line: str, received_at_s: float, max_age_ms: int = config.IR_GET_MAX_AGE_MS) -> IrSample | None:
    """The `IR,GET` reply -> semantic statuses (PROTOCOL.md §4.5). None if the line is not an `IR,` line.

      ST:OK    100-200 mm  VALID_DISTANCE (trusted);  201-400 mm  VALID_DISTANCE, coarse;
               NEAR / FAR  INVALID (FAR is never TRUSTWORTHY_NO_WALL, D-49);
      ST:WARM / ERR  INVALID;  ST:STALE  STALE;  ST:FAULT  FAULT for BOTH sensors.

    A reply that breaks the grammar (full match, `F` 00 for OK/WARM and with bit 40 otherwise, a distance
    only with OK and `NA` only without it, a number outside 100-400) is MALFORMED: both sensors INVALID,
    with `problem` set. An `AGE` above `max_age_ms` (PROVISIONAL, 150 ms) makes both STALE. The sequence
    rule (S > 0 and strictly increasing) needs the previous reply and is applied by the caller
    (`stm_task2.Task2Stm.read_ir`). It never produces TRUSTWORTHY_NO_WALL.
    """
    if not line.startswith("IR,"):
        return None
    match = IR_GET_PATTERN.fullmatch(line)
    if match is None:
        return untrusted_sample(received_at_s, f"malformed IR reply: {line}")
    seq, state, left, right, age, mask = match.groups()
    seq, age_ms, mask = int(seq), (None if age == "NA" else int(age)), int(mask, 16)

    def malformed(why: str) -> IrSample:
        return untrusted_sample(received_at_s, f"malformed IR reply ({why}): {line}", seq=seq, age_ms=age_ms)

    if state in ("OK", "WARM"):
        if mask != 0:
            return malformed(f"F:{match.group(6)} with ST:{state}")
    elif not mask & IR_FRESH_BIT:
        return malformed(f"F:{match.group(6)} without bit 40 for ST:{state}")
    if state != "OK":
        if left != "NA" or right != "NA":
            return malformed(f"a value with ST:{state}")
        return untrusted_sample(received_at_s, None, _NOT_OK[state], seq, age_ms)
    if age_ms is None:
        return malformed("AGE:NA with ST:OK")

    def reading(text: str) -> IrReading | None:
        if text in ("NEAR", "FAR"):
            return IrReading(IrStatus.INVALID, far=text == "FAR")
        if text == "NA":
            return None
        mm = int(text)
        if not IR_TRUSTED_MM[0] <= mm <= IR_COARSE_MAX_MM:
            return None
        return IrReading(IrStatus.VALID_DISTANCE, mm / 10.0, coarse=mm > IR_TRUSTED_MM[1])

    readings = reading(left), reading(right)
    if None in readings:
        return malformed("NA or an out-of-range distance with ST:OK")
    if age_ms > max_age_ms:
        return untrusted_sample(received_at_s, f"stale IR reply: AGE {age_ms} ms > {max_age_ms} ms",
                                IrStatus.STALE, seq, age_ms)
    return IrSample(seq, received_at_s, readings[0], readings[1], age_ms)
