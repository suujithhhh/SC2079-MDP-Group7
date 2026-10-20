"""Task 2 STM interface: the Task 1 `StmLink` plus the sensor operations
Task 2 needs.

What the current firmware (basemdpfixedstraightness) supports is used as is:
`MOV,F/B/FL/FR/BL/BR`, `IMU,GET`, `PROFILE,...`. One MOV in flight, only
`ACK` / `ERR,...` terminal, nothing resent (stm_link.py).

`follow_edge` is implemented on `MOV,FIRL` / `MOV,FIRR` (PROTOCOL.md §4.6, via
`stm_fir`): implemented for host and simulator use, NOT bench validated and with
NO hardware authority (D-57 / PD-02); the mission gate keeps it off the robot.

`set_uspark` / `prepare_final_parking` / `park_final` implement the selected final
parking (USPARK predictive stop around `MOV,F,100`, TASK2_USPARK_ALGO_INTEGRATION.md).

`read_ir` is implemented on `IR,GET` (PROTOCOL.md §4.5): a read-only query,
idle only, at most 10 Hz, one request outstanding. It never creates an edge.

The other sensor operations (ultrasonic, health) are still PLACEHOLDERS: their `has_*` flag (config.STM_HAS_*) is False and the mission
does not call them. When one is built: implement the method body, set its flag,
and add a test with the real reply strings.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import config
import stm_fir
import stm_park
import stm_us
from ir import IrSample, IrStatus, parse_ir, untrusted_sample
from stm_link import StmLink, StmReply
from stm_protocol import EmergencyStopped, LinkError

IR_GET_CAPABILITY = stm_fir.Capability(
    implemented=True, bench_validated=False, hardware_authority=False,
    reason="read-only; static IR,GET queries after a separately authorized flash (L6a); no motion authority")


class SensorUnavailable(NotImplementedError):
    """The STM firmware does not implement this sensor operation yet."""


@dataclass
class EdgeResult:
    """Outcome of one STM edge-follow operation.

    status: EDGE (obstacle confirmed, then its far edge), NO_WALL (obstacle never
    confirmed), MAX_TRAVEL (edge never confirmed), SENSOR_FAULT, or a failure the
    mission does not accept: STOPPED, TIMEOUT, PROTOCOL_ERROR, ERROR.
    travel_cm: brake-end encoder travel (`DL`/`DR`, D-58), 0.0 if not reported.
    edge_at_cm: travel at which the edge was confirmed (before braking); None if
    not reported. pose_known: False whenever the robot may have moved without a
    trusted record. problems: why a completion record was rejected.
    """

    status: str
    travel_cm: float = 0.0
    edge_at_cm: float | None = None
    line: str = ""
    telemetry: list[str] = field(default_factory=list)
    pose_known: bool = True
    problems: list[str] = field(default_factory=list)


@dataclass
class StopResult:
    """Outcome of an ultrasonic-terminated forward move.
    status: OK (stopped at the distance), MAX_TRAVEL, SENSOR_FAULT, TIMEOUT, ERROR."""

    status: str
    travel_cm: float = 0.0
    final_cm: float | None = None
    line: str = ""


class Task2Stm:
    # D-76, supervised runs only (final_run.py sets it): accept FIR ENDs without DL/DR. Off by default.
    fir_legacy_travel = False

    def __init__(self, link: StmLink, clock=time.monotonic, sleep=time.sleep) -> None:
        self.link = link
        self.clock = clock
        self.sleep = sleep
        self.has_ir = IR_GET_CAPABILITY.implemented
        self.has_edge_follow = stm_fir.FIR_CAPABILITY.implemented   # NOT hardware authority (D-57)
        self.has_ultrasonic = config.STM_HAS_ULTRASONIC
        self.has_uspark_parking = stm_park.USPARK_PARKING_CAPABILITY.implemented
        self.has_ultrasonic_stop = config.STM_HAS_ULTRASONIC_STOP
        self.has_sensor_health = config.STM_HAS_SENSOR_HEALTH
        # the `US,` bench stream (stm_us); preflight turns it off when the firmware sends none
        self.has_front_us = True
        # `USD,GET` (PROTOCOL.md §4.8): set by probe_usd_get() when the firmware answers it; measure_front then
        # queries instead of listening to the bench stream
        self.has_usd_get = False
        self._last_usd_seq: int | None = None         # S strictly increasing per link session, like IR,GET
        self.last_ir: IrSample | None = None
        self._last_ir_seq: int | None = None          # sequence rule: strictly increasing per link session
        self._last_ir_request_s: float | None = None

    # -- implemented today -------------------------------------------------------
    def move(self, command: str) -> StmReply:
        return self.link.move(command)

    def heading(self) -> float | None:
        return self.link.heading()

    def apply_profile(self, profile: str) -> str | None:
        return self.link.apply_profile({
            "set_command": f"PROFILE,{profile}", "expected_set_reply": "PROFILE,OK",
            "verify_command": "PROFILE,GET", "expected_verify_reply": f"PROFILE,{profile}",
        })

    def note(self, text: str) -> None:
        self.link.note(text)

    # -- final parking (USPARK) -----------------------------------------------------------
    def set_uspark(self, enabled: bool) -> str | None:
        """`USPARK,1` / `USPARK,0` -> exactly `USPARK,OK`. Returns None, or why it failed (nothing retried)."""
        command = "USPARK,1" if enabled else "USPARK,0"
        try:
            reply = self.link.query(command, lambda line: line == stm_park.OK_REPLY)
        except LinkError as exc:
            return f"{command}: {exc}"
        return None if reply.ok else f"{command}: {reply.line or reply.status}"

    def get_uspark(self) -> stm_park.UsParkStatus | None:
        try:
            reply = self.link.query("USPARK,GET", lambda line: line.startswith("USPARK,E:"))
        except LinkError:
            return None
        return stm_park.parse_get(reply.line) if reply.ok else None

    def prepare_final_parking(self) -> str | None:
        """Idle, staged: select and verify TASK2TEST, reset USPARK (0), enable it (1) and verify the GET
        fields (handoff §3.3). Returns None when armed, otherwise the reason (the caller fails safe)."""
        reason = self.apply_profile(config.PARK_PROFILE)
        if reason:
            return f"profile: {reason}"
        for enabled in (False, True):
            reason = self.set_uspark(enabled)
            if reason:
                return reason
        status = self.get_uspark()
        problem = stm_park.armed_problem(status)
        if problem:
            return problem
        self.note(f"USPARK armed for final parking: {status.line}; expecting firmware trigger "
                  f"{config.PARK_EXPECTED_TRIGGER_MM} (authority build)")
        return None

    def park_final(self) -> stm_park.ParkingResult:
        """The one final `MOV,F,100` through the dedicated parking transaction. Never resent."""
        command = config.PARK_FINAL_COMMAND
        result = stm_park.evaluate(command, self.link.send_parking_motion(command))
        self.note(f"final parking {result.outcome.value}: {result.reason}")
        return result

    # -- sensor operations ---------------------------------------------------------------
    def sensor_health(self) -> dict[str, bool]:
        """PLACEHOLDER. Expected: one query answered with the health of the IMU,
        encoders, ultrasonic and both IR sensors, e.g. {"IMU": True, "IR_L": True, ...}."""
        raise SensorUnavailable("sensor health query is not in the STM protocol yet")

    def read_ir(self) -> IrSample:
        """One `IR,GET` query -> IrSample (PROTOCOL.md §4.5). Idle only, one request outstanding,
        at most 10 Hz (waits if called sooner). Never raises for a bad answer: a missing reply,
        an `ERR` (e.g. `ERR,BAD_CMD` from firmware built without IR), a refused link, a malformed
        or stale reply, or a sequence that does not increase all give a sample with BOTH sensors
        untrusted and `problem` set. A lost reply leaves the link DESYNCHRONIZED (D-51), so the
        next motion is refused; nothing is resent."""
        if self._last_ir_request_s is not None:
            wait = self._last_ir_request_s + config.IR_GET_MIN_INTERVAL_S - self.clock()
            if wait > 0:
                self.sleep(wait)
        self._last_ir_request_s = self.clock()
        try:
            reply = self.link.query("IR,GET", lambda line: line.startswith("IR,"), config.IR_GET_TIMEOUT_S)
        except LinkError as exc:
            sample = untrusted_sample(self.clock(), f"IR,GET refused: {exc}")
        else:
            if reply.ok:
                sample = self._check_sequence(parse_ir(reply.line, self.clock()))
            else:
                sample = untrusted_sample(self.clock(), f"IR,GET {reply.status}: {reply.line or 'no reply'}")
        if sample.problem:
            self.link.note(f"IR: {sample.problem}")
        self.last_ir = sample
        return sample

    def _check_sequence(self, sample: IrSample) -> IrSample:
        """S must be > 0 and strictly greater than the previous reply's S in this link session:
        equal is a stale duplicate, smaller (or 0) is invalid."""
        if sample.seq is None or sample.problem:
            return sample
        previous, self._last_ir_seq = self._last_ir_seq, max(sample.seq, self._last_ir_seq or 0)
        if sample.seq == 0:
            return untrusted_sample(sample.received_at_s, "IR S:0 (nothing published yet)", IrStatus.STALE,
                                    sample.seq, sample.age_ms)
        if previous is not None and sample.seq == previous:
            return untrusted_sample(sample.received_at_s, f"IR S:{sample.seq} repeated", IrStatus.STALE,
                                    sample.seq, sample.age_ms)
        if previous is not None and sample.seq < previous:
            return untrusted_sample(sample.received_at_s, f"IR S:{sample.seq} after S:{previous}",
                                    IrStatus.INVALID, sample.seq, sample.age_ms)
        return sample

    def probe_usd_get(self) -> str | None:
        """Idle, before the run: one `USD,GET`. A reply in the §4.8 grammar turns `has_usd_get` on (None
        returned); anything else (old firmware answers ERR,BAD_CMD) leaves it off and returns why, so
        measure_front keeps listening to the bench stream. A lost reply desynchronises the link (D-51)."""
        self.has_usd_get = False
        try:
            reply = self.link.query("USD,GET", lambda line: line.startswith("USD,"), config.USD_GET_TIMEOUT_S)
        except LinkError as exc:
            return f"USD,GET refused: {exc}"
        if not reply.ok:
            return f"USD,GET {reply.status}: {reply.line or 'no reply'}"
        line = stm_us.parse_usd(reply.line, self.clock())
        if line is None:
            return f"USD,GET malformed reply: {reply.line}"
        self._last_usd_seq = line.seq
        self.has_usd_get = True
        return None

    def measure_front(self, lines: int = config.US_SAMPLE_LINES,
                      timeout_s: float = config.US_SAMPLE_TIMEOUT_S) -> stm_us.FrontReading:
        """Robot stationary and idle: with `USD,GET` (has_usd_get) query it until US_MIN_AGREEING new
        publications agree (front_by_query); otherwise listen to `lines` new `US,` bench lines (about 200 ms
        apart). Either way summarised by stm_us.summarise. Never raises for a bad reading; a refused link gives
        a reading with `problem` set (an emergency STOP request is still honoured by the link and raises)."""
        if self.has_usd_get:
            return self.front_by_query(timeout_s)
        try:
            found = self.link.listen(lambda line: line.text.startswith("US,"), lines, timeout_s)
        except EmergencyStopped:
            raise
        except LinkError as exc:
            return stm_us.FrontReading(None, problem=f"listen refused: {exc}")
        parsed = [stm_us.parse_us(line.text, line.received_at_s) for line in found]
        bad = sum(1 for item in parsed if item is None)
        reading = stm_us.summarise([item for item in parsed if item is not None], seen=len(found))
        if bad:
            reading.problem = (reading.problem + "; " if reading.problem else "") + f"{bad} malformed US, line(s)"
        self.link.note(f"US: {reading.distance_cm} cm from {reading.used}/{reading.seen} lines"
                       + (f" ({reading.problem})" if reading.problem else ""))
        return reading

    def front_by_query(self, timeout_s: float = config.US_SAMPLE_TIMEOUT_S) -> stm_us.FrontReading:
        """`USD,GET` at most every USD_GET_MIN_INTERVAL_S until the replies with a new S (a repeated S is the same
        publication and is skipped) give a reading (stm_us.summarise), or `timeout_s`. One request outstanding,
        idle only. An ERR, a lost reply (the link desynchronises) or an S going backwards ends it with no
        reading; nothing is resent after a lost reply."""
        deadline = self.clock() + timeout_s
        found: list[stm_us.UsLine] = []
        seen, problems = 0, []
        last_request: float | None = None
        while self.clock() < deadline:
            if last_request is not None:
                wait = last_request + config.USD_GET_MIN_INTERVAL_S - self.clock()
                if wait > 0:
                    self.sleep(wait)
            last_request = self.clock()
            try:
                reply = self.link.query("USD,GET", lambda line: line.startswith("USD,"), config.USD_GET_TIMEOUT_S)
            except EmergencyStopped:
                raise
            except LinkError as exc:
                problems.append(f"USD,GET refused: {exc}")
                break
            seen += 1
            if not reply.ok:
                problems.append(f"USD,GET {reply.status}: {reply.line or 'no reply'}")
                break
            line = stm_us.parse_usd(reply.line, self.clock())
            if line is None:
                problems.append(f"malformed USD reply {reply.line}")
                continue
            if self._last_usd_seq is not None and line.seq < self._last_usd_seq:
                problems.append(f"USD S:{line.seq} after S:{self._last_usd_seq}")
                break
            if line.seq == self._last_usd_seq:
                continue                                   # no new publication yet
            self._last_usd_seq = line.seq
            found.append(line)
            reading = stm_us.summarise(found, seen=len(found))
            if reading.ok:
                break
        reading = stm_us.summarise(found, seen=len(found)) if found else stm_us.FrontReading(
            None, 0, 0, [], f"no new USD publication in {seen} queries")
        if problems:
            reading.problem = "; ".join([p for p in [reading.problem] if p] + problems)
        self.link.note(f"US (USD,GET): {reading.distance_cm} cm from {reading.used}/{reading.seen} publications, "
                       f"{seen} queries" + (f" ({reading.problem})" if reading.problem else ""))
        return reading

    def read_ultrasonic(self) -> float | None:
        """PLACEHOLDER. Expected: front ultrasonic distance in cm from the sensor
        face; None for a trustworthy out-of-range; raise on a sensor fault."""
        raise SensorUnavailable("ultrasonic is not in the STM protocol yet")

    def follow_edge(self, sensor: str, max_cm: int) -> EdgeResult:
        """One `MOV,FIRL,<max_cm>` (sensor "L") or `MOV,FIRR,<max_cm>` (sensor "R"):
        drive forward until the STM confirms the obstacle and then its far edge,
        never more than `max_cm` (an int 1-999, the collision-safe cap the caller
        computed; never an obstacle length). ONE command, ONE terminal reply,
        never resent. Success only with the complete completion record (D-56).
        Raises ValueError for a bad sensor or cap (nothing sent) and the link's
        errors if it refuses."""
        command = stm_fir.fir_command(sensor, max_cm)
        result = self.link.send_sensor_motion(command)
        lines = [line.text for line in result.telemetry]
        outcome = stm_fir.evaluate(command, result, legacy_travel=self.fir_legacy_travel)
        for problem in outcome.problems:
            self.link.note(f"FIR {command}: {problem}")
        return EdgeResult(outcome.status, outcome.travel_cm, outcome.edge_at_cm, outcome.line, lines,
                          outcome.pose_known, outcome.problems)

    def forward_until(self, stop_cm: float, max_cm: float) -> StopResult:
        """PLACEHOLDER: drive forward with heading hold, slowing near the target,
        and stop when the front ultrasonic reads `stop_cm`; never travel more
        than `max_cm` (collision-safe if the ultrasonic fails)."""
        raise SensorUnavailable("ultrasonic stop is not in the STM protocol yet")
