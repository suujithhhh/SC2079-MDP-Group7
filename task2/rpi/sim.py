"""Offline Task 2 simulator: a TRUE hidden course, a fake STM that drives the
true pose with the same arc model, and a fake camera that sees the arrows and
the bullseyes by the pinhole model. Used by the tests (final_run's included).

It checks the algorithm, not the real robot's accuracy.

The world holds the TRUE walls, so contacts are always counted against the
truth, never against what the planner believes. Every value in the FIXTURE_*
helpers is a SIMULATION FIXTURE chosen for a test, not an arena fact, not an
official bound and not evidence about the competition course (BD-01, BD-02,
BD-18 stay open).

Sensor models are simulator-internal in Phase 2: the side IR is a ray cast with
a range limit, three choices for what lies beyond the range (NA, a trustworthy
"no wall", or a large distance), sampling spacing, latency, and injected
INVALID / DUPLICATE / STALE / FUTURE / DECREASING-sequence / FAULT / SILENCE /
AMBIGUOUS / STUCK-PRESENT behaviour. The ultrasonic has a valid range and a
mode. The fake STM enforces ONE travel cap per edge leg (the Pi sends the
smaller of the search limit and the collision-safe limit) and, after any
trigger, keeps moving for `stop_overshoot_cm` rounded UP to whole cm (that
effective distance is what is moved, reported and compared with the reserve); a
configuration whose effective overshoot exceeds the stopping reserve is rejected, never declared collision-safe.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable

import config
from course import FreeSpace, O2Bounds
from geometry import Pose, Rect, footprint, intersects, sweep
from ir import (EdgeDetector, EdgeDetectorConfig, EdgeLimits, EdgeState, IrReading, IrSample, IrSensor, IrStatus,
                IrThresholds)
from perception import Detection, Look
import stm_park
import stm_us
from stm_link import StmReply, validate_mov
from stm_protocol import MotionResult, classify_line
from stm_task2 import EdgeResult, SensorUnavailable, StopResult

ARROW_LABEL = {side: label for label, side in config.ARROW_IDS.items()}

# --- simulation fixtures (NOT arena facts) --------------------------------------------
@dataclass(frozen=True)
class SimParams:
    """The simulator's fixture parameters: free space around O2, O2 half-span bounds, IR thresholds, the initial
    pose uncertainty, the stopping reserve the fake STM may overshoot by, and the edge-confirm allowance."""

    free: FreeSpace
    o2_bounds: O2Bounds
    ir_thresholds: IrThresholds
    initial_pose_uncertainty_cm: float
    stopping_reserve_cm: float
    edge_confirm_allowance_cm: float


FIXTURE_FREE = FreeSpace(end_clearance_cm=config.O2_END_CLEARANCE_CM, behind_clearance_cm=50.0,
                         side_wall_south_cm=60.0)


def fixture_params(span: float, *, min_half: float | None = None, max_half: float | None = None,
                   uncertainty: float = 0.0, reserve: float = 5.0, allowance: float = 20.0,
                   free: FreeSpace = FIXTURE_FREE, thresholds: IrThresholds | None = None) -> SimParams:
    """Explicit, labelled fixture values. By default the half-span bounds
    bracket the TRUE span of this scenario (min = span/2 - 10, max = span/2 + 30):
    a regression fixture, not a statement about the arena. Zero uncertainty is
    the idealised simulation case."""
    half = span / 2
    return SimParams(
        free=free,
        o2_bounds=O2Bounds(max(0.0, half - 10) if min_half is None else min_half,
                           half + 30 if max_half is None else max_half),
        ir_thresholds=thresholds or IrThresholds.provisional(),
        initial_pose_uncertainty_cm=uncertainty,
        stopping_reserve_cm=reserve,
        edge_confirm_allowance_cm=allowance)


def fixture_edge_cfg(params: SimParams, **overrides) -> EdgeDetectorConfig:
    values = dict(thresholds=params.ir_thresholds, present_samples=3, gone_samples=3, no_wall_samples=3,
                  max_age_s=0.5, max_silence_s=0.5, acquisition_limit_cm=40.0,
                  max_untrusted_samples=5, max_untrusted_cm=15.0)
    values.update(overrides)
    return EdgeDetectorConfig(**values)


# --- the true world ---------------------------------------------------------------------
@dataclass
class World:
    arrows: str                   # O1 then O2
    o1_gap: float                 # carpark entrance -> O1 face
    o2_gap: float                 # O1 back face -> O2 face
    o2_span: float                # the TRUE cross-span: required, never defaulted
    free: FreeSpace               # the TRUE free space (walls stand exactly this far away)
    o1_dx: float = 0.0            # obstacles placed off the carpark centre line (placement tolerance)
    o2_dx: float = 0.0
    walls: bool = True            # walls behind / beside O2
    range_error: float = 0.0      # camera depth scale error, e.g. 0.05 = reads 5 % far
    lateral_error_cm: float = 0.0 # camera puts every target this much further right than it is

    def __post_init__(self) -> None:
        if self.arrows not in ("LL", "LR", "RL", "RR"):
            raise ValueError(f"arrows must be one of LL, LR, RL, RR, got {self.arrows!r}")
        if not isinstance(self.o2_span, (int, float)) or not math.isfinite(self.o2_span) or self.o2_span <= 0:
            raise ValueError(f"o2_span must be finite and > 0, got {self.o2_span!r}")
        if not isinstance(self.free, FreeSpace):
            raise TypeError("free must be a FreeSpace")

    @property
    def back_y(self) -> float:
        return -config.START_REAR_FROM_BACK_WALL_CM

    @property
    def entrance_y(self) -> float:
        return self.back_y + config.CARPARK_DEPTH_CM

    @property
    def o1(self) -> Rect:
        face = self.entrance_y + self.o1_gap
        return Rect(self.o1_dx - 5, face, self.o1_dx + 5, face + 10, "O1")

    @property
    def o2(self) -> Rect:
        face = self.o1.y1 + self.o2_gap
        half = self.o2_span / 2
        return Rect(self.o2_dx - half, face, self.o2_dx + half, face + config.O2_DEPTH_CM, "O2")

    def rects(self) -> list[Rect]:
        half = config.CARPARK_WIDTH_CM / 2
        wide = self.o2_span / 2 + abs(self.o2_dx) + self.free.end_clearance_cm + 200.0   # the truth's own extent
        rects = [self.o1, self.o2,
                 Rect(-wide, self.back_y - 10, wide, self.back_y, "carpark back wall"),
                 Rect(-half - 10, self.back_y, -half, self.entrance_y, "carpark left wall"),
                 Rect(half, self.back_y, half + 10, self.entrance_y, "carpark right wall")]
        if self.walls:
            o2 = self.o2
            behind = o2.y1 + self.free.behind_clearance_cm
            south = o2.y0 - self.free.side_wall_south_cm
            west = o2.x0 - self.free.end_clearance_cm
            east = o2.x1 + self.free.end_clearance_cm
            rects += [Rect(-wide, behind, wide, behind + 10, "wall behind O2"),
                      Rect(west - 10, south, west, behind, "wall west of O2"),
                      Rect(east, south, east + 10, behind, "wall east of O2")]
        return rects


def ray_distance(origin: tuple[float, float], direction: tuple[float, float], rects: list[Rect]) -> float | None:
    """Distance along a ray to the first rectangle it meets (0.0 when the origin
    is inside one); None when it meets none. Slab method; finite arithmetic only."""
    best = None
    for rect in rects:
        t_near, t_far = -1e12, 1e12
        hit = True
        for o, d, low, high in ((origin[0], direction[0], rect.x0, rect.x1),
                                (origin[1], direction[1], rect.y0, rect.y1)):
            if abs(d) < 1e-12:
                if not low <= o <= high:
                    hit = False
                    break
                continue
            t1, t2 = (low - o) / d, (high - o) / d
            t_near, t_far = max(t_near, min(t1, t2)), min(t_far, max(t1, t2))
        if not hit or t_far < max(t_near, 0.0):
            continue
        t = max(t_near, 0.0)
        if best is None or t < best:
            best = t
    return best


# --- simulator-internal sensor models ------------------------------------------------------
class SimBeyondRange(Enum):
    INVALID_NA = "INVALID_NA"                          # nothing in range reads as NA (INVALID)
    TRUSTWORTHY_NO_WALL = "TRUSTWORTHY_NO_WALL"        # nothing in range reads as a trustworthy no-wall
    MAX_DISTANCE = "MAX_DISTANCE"                      # nothing in range reads as the maximum distance


class IrInjectionKind(Enum):
    INVALID = "INVALID"                  # per sensor
    FAULT = "FAULT"                      # per sensor
    AMBIGUOUS = "AMBIGUOUS"              # per sensor: a distance between the thresholds
    STUCK_PRESENT = "STUCK_PRESENT"      # per sensor: reports PRESENT whatever is there
    DUPLICATE = "DUPLICATE"              # whole sample: repeated sequence number
    DECREASING_SEQ = "DECREASING_SEQ"    # whole sample
    STALE_AGE = "STALE_AGE"              # whole sample: received long after it was taken
    FUTURE_TIME = "FUTURE_TIME"          # whole sample: timestamp in the future
    SILENCE = "SILENCE"                  # whole sample: nothing is sent


SAMPLE_LEVEL = {IrInjectionKind.DUPLICATE, IrInjectionKind.DECREASING_SEQ, IrInjectionKind.STALE_AGE,
                IrInjectionKind.FUTURE_TIME, IrInjectionKind.SILENCE}


@dataclass(frozen=True)
class IrInjection:
    sensor: IrSensor        # ignored for the whole-sample kinds
    from_cm: float          # active while the edge leg's travel is in [from_cm, to_cm)
    to_cm: float
    kind: IrInjectionKind


@dataclass(frozen=True)
class SimIr:
    range_max_cm: float
    beyond_range: SimBeyondRange
    sample_spacing_cm: float
    latency_samples: int = 0
    injections: tuple[IrInjection, ...] = ()


class SimUsMode(Enum):
    OK = "OK"
    FAULT = "FAULT"
    INVALID = "INVALID"
    STALE = "STALE"


@dataclass(frozen=True)
class SimUltrasonic:
    min_range_cm: float = 2.0
    max_range_cm: float = 300.0
    mode: SimUsMode = SimUsMode.OK


@dataclass(frozen=True)
class SimFrontUs:
    """The `US,` bench-stream ultrasonic (stm_us): the nearest true surface inside a cone of
    `half_angle_deg` straight ahead, read by the Pi while stationary. `offset_cm` is a constant error
    (fixture), `fail` gives no trusted reading."""
    half_angle_deg: float = 15.0
    max_range_cm: float = 400.0
    offset_cm: float = 0.0
    fail: bool = False


@dataclass(frozen=True)
class SimUspark:
    """The STM's predictive USPARK stop around the final MOV,F,100 (a FIXTURE of the team-reported behaviour,
    not a physics model): the robot stops with the ultrasonic face `clearance_cm` from the first surface ahead
    and the STM reports a cause line `R:<reason>`, `TH:<trigger_mm>`, `P:<path>` (PRED) with END STOPPED and
    ACK,STOPPED. `lines`, when given, replaces the whole reply for a negative test."""
    clearance_cm: float = config.PARK_VALIDATED_CLEARANCE_CM
    reason: str = "PRED"
    path: str = "NORM"
    trigger_mm: int = config.PARK_EXPECTED_TRIGGER_MM
    lines: tuple[str, ...] | None = None


@dataclass
class FakeStm:
    world: World
    params: SimParams
    ir: SimIr | None = None
    edge_cfg: EdgeDetectorConfig | None = None
    ultrasonic: SimUltrasonic | None = None
    uspark: SimUspark | None = None
    front_us: SimFrontUs | None = None
    stop_overshoot_cm: float = 0.0           # travel after a stop trigger
    edge_lag_cm: int = 0                     # a FIR confirms the edge this much later (the robot: ~3 cm, config.IR_EDGE_LAG_CM)
    allow_overshoot_beyond_reserve: bool = False   # ONLY for negative tests; the run is then NOT declared safe
    s_per_cm: float = 0.02                   # deterministic simulated time per cm of travel
    pose: Pose = field(default_factory=lambda: Pose(0.0, 0.0, 90.0))
    contacts: list[str] = field(default_factory=list)
    trace: list[Pose] = field(default_factory=list)
    commands: list[str] = field(default_factory=list)
    follow_calls: list[float] = field(default_factory=list)   # every travel cap the Pi sent to follow_edge
    follow_legs: list[tuple[Pose, float]] = field(default_factory=list)   # (start pose, total travel incl. overshoot)
    motion_count: int = 0                    # every motion the fake performed (moves, legs, stops)
    uspark_enabled: bool = False
    uspark_violations: list[str] = field(default_factory=list)   # ordinary moves sent while USPARK was on
    uspark_sets: list[bool] = field(default_factory=list)          # every USPARK,0/1 the Pi sent
    parking_commands: list[str] = field(default_factory=list)
    imu_offset: float = 37.0
    profile_log: list[tuple[int, str]] = field(default_factory=list)   # (moves sent so far, profile) per apply_profile
    profile_failures: dict[str, str] = field(default_factory=dict)     # profile -> reason: apply_profile fails with it

    def __post_init__(self) -> None:
        if self.ir is not None and self.edge_cfg is None:
            self.edge_cfg = fixture_edge_cfg(self.params)
        self.has_ir = self.has_edge_follow = self.ir is not None
        self.has_ultrasonic = self.has_ultrasonic_stop = self.ultrasonic is not None
        self.has_sensor_health = False
        self.has_front_us = self.front_us is not None
        self.front_readings: list[float | None] = []
        self.has_uspark_parking = self.uspark is not None
        overshoot = self.stop_overshoot_cm
        if isinstance(overshoot, bool) or not isinstance(overshoot, (int, float)) or not math.isfinite(overshoot) or overshoot < 0:
            raise ValueError(f"stop_overshoot_cm must be finite and >= 0, got {overshoot!r}")
        if not self.declared_safe and not self.allow_overshoot_beyond_reserve:
            raise ValueError(f"simulated stopping overshoot {self.stop_overshoot_cm} cm (moves {self.effective_overshoot_cm} cm "
                             f"in whole-cm steps) exceeds the configured stopping reserve "
                             f"{self.params.stopping_reserve_cm} cm: not a collision-safe configuration")
        self._clock_s = 0.0
        self.trace.append(self.pose)

    @property
    def effective_overshoot_cm(self) -> int:
        """What the simulator really moves after a stop trigger: whole-cm commands, so the ceiling."""
        return math.ceil(self.stop_overshoot_cm)

    @property
    def declared_safe(self) -> bool:
        """Only a stopping overshoot within the reserve can be declared collision-safe. The
        EFFECTIVE (rounded-up) overshoot is compared, i.e. the distance actually moved."""
        return self.effective_overshoot_cm <= self.params.stopping_reserve_cm

    def _drive(self, commands: list[str]) -> None:
        self.motion_count += 1
        poses = sweep(self.pose, commands, 1.0, 1.0)
        for p in poses:
            polygon = footprint(p, 0.0)
            for rect in self.world.rects():
                if intersects(polygon, rect):
                    self.contacts.append(f"{rect.name} at {p}")
        self.trace += poses[1:]
        self.pose = poses[-1]

    def _overshoot(self) -> int:
        n = self.effective_overshoot_cm
        if n > 0:
            self._drive([f"MOV,F,{n}"])
        return n

    def move(self, command: str) -> StmReply:
        validate_mov(command)
        if self.uspark_enabled:
            self.uspark_violations.append(command)
        self.commands.append(command)
        self._drive([command])
        return StmReply("ok", "ACK", True)

    def heading(self) -> float:
        return (self.pose.h + self.imu_offset) % 360

    def apply_profile(self, profile: str) -> str | None:
        self.profile_log.append((len(self.commands), profile))
        return self.profile_failures.get(profile)

    def profile_at(self, index: int, base: str = "PRECISION") -> str:
        """The profile in force for the move at `index` in `commands` (the last one set before it)."""
        current = base
        for count, profile in self.profile_log:
            if count <= index and profile not in self.profile_failures:
                current = profile
        return current

    def note(self, text: str) -> None:
        pass

    # -- side IR ------------------------------------------------------------------------------
    def _beam(self, sensor: str) -> float | None:
        """Raw distance from the sensor FACE along its beam to the first true
        surface (no range limit); None if the beam meets nothing."""
        ahead = config.IR_FORWARD_CM[sensor]
        origin = self.pose.point(ahead, -config.IR_LATERAL_CM[sensor])
        rx, ry = self.pose.right()
        sign = 1 if sensor == "R" else -1
        return ray_distance(origin, (sign * rx, sign * ry), self.world.rects())

    def _truth(self, sensor: IrSensor) -> IrReading:
        ir = self.ir
        distance = self._beam(sensor.value)
        if distance is None or distance > ir.range_max_cm:
            if ir.beyond_range is SimBeyondRange.INVALID_NA:
                return IrReading(IrStatus.INVALID)
            if ir.beyond_range is SimBeyondRange.TRUSTWORTHY_NO_WALL:
                return IrReading(IrStatus.TRUSTWORTHY_NO_WALL)
            return IrReading(IrStatus.VALID_DISTANCE, ir.range_max_cm)
        return IrReading(IrStatus.VALID_DISTANCE, max(distance, 0.01))

    def read_ir(self) -> IrSample:
        """One static IR,GET at the current pose: the ray-cast truth, with the sensor-level injections that
        are active at the start of a leg (travel 0) applied, e.g. a FAULT on the active sensor."""
        if self.ir is None:
            raise SensorUnavailable("the simulator was built without a side IR")
        self._ir_get_seq = getattr(self, "_ir_get_seq", 0) + 1
        th = self.params.ir_thresholds

        def shaped(reading: IrReading, which: IrSensor) -> IrReading:
            kinds = {i.kind for i in self.ir.injections
                     if i.from_cm <= 0 < i.to_cm and i.kind not in SAMPLE_LEVEL and i.sensor is which}
            if IrInjectionKind.FAULT in kinds:
                return IrReading(IrStatus.FAULT)
            if IrInjectionKind.STUCK_PRESENT in kinds:
                return IrReading(IrStatus.VALID_DISTANCE, th.present_cm / 2)
            if IrInjectionKind.INVALID in kinds:
                return IrReading(IrStatus.INVALID)
            if IrInjectionKind.AMBIGUOUS in kinds:
                return IrReading(IrStatus.VALID_DISTANCE, (th.present_cm + th.absent_cm) / 2)
            return reading

        def static(which: IrSensor) -> IrReading:
            distance = self._beam(which.value)        # the STM reports FAR when nothing is within ~40 cm
            if distance is None or distance > config.IR_FAR_CM:
                return IrReading(IrStatus.INVALID, far=True)
            return self._truth(which)

        return IrSample(self._ir_get_seq, self._clock_s, shaped(static(IrSensor.LEFT), IrSensor.LEFT),
                        shaped(static(IrSensor.RIGHT), IrSensor.RIGHT))

    def follow_edge(self, sensor: str, max_cm: float) -> EdgeResult:
        """One bounded edge-follow leg: the reference EdgeDetector decides, the
        fake STM drives 1 cm at a time, and after the trigger it keeps moving for
        `stop_overshoot_cm`. The Pi sends ONE cap (the smaller of the search limit
        and the collision-safe limit); the fake reports MAX_TRAVEL when it is hit."""
        if self.ir is None or self.edge_cfg is None:
            raise SensorUnavailable("the simulator was built without a side IR")
        self.follow_calls.append(float(max_cm))
        self.motion_count += 1
        leg_start = self.pose
        ir, th = self.ir, self.edge_cfg.thresholds
        t0 = self._clock_s
        det = EdgeDetector(IrSensor(sensor), self.edge_cfg, EdgeLimits(search_limit_cm=float(max_cm)), start_s=t0)
        travel, next_sample, seq = 0, 0.0, 0
        pending: list[tuple[IrReading, IrReading]] = []
        while True:
            now = t0 + travel * self.s_per_cm
            if travel >= next_sample:
                next_sample += ir.sample_spacing_cm
                pending.append((self._truth(IrSensor.LEFT), self._truth(IrSensor.RIGHT)))
                lag = ir.latency_samples
                left, right = pending[-1 - lag] if len(pending) > lag else (IrReading(IrStatus.INVALID),) * 2
                active = [i for i in ir.injections if i.from_cm <= travel < i.to_cm]
                whole = {i.kind for i in active if i.kind in SAMPLE_LEVEL}

                def shaped(reading: IrReading, which: IrSensor) -> IrReading:
                    kinds = {i.kind for i in active if i.kind not in SAMPLE_LEVEL and i.sensor is which}
                    if IrInjectionKind.FAULT in kinds:
                        return IrReading(IrStatus.FAULT)
                    if IrInjectionKind.STUCK_PRESENT in kinds:
                        return IrReading(IrStatus.VALID_DISTANCE, th.present_cm / 2)
                    if IrInjectionKind.INVALID in kinds:
                        return IrReading(IrStatus.INVALID)
                    if IrInjectionKind.AMBIGUOUS in kinds:
                        return IrReading(IrStatus.VALID_DISTANCE, (th.present_cm + th.absent_cm) / 2)
                    return reading

                decreasing = IrInjectionKind.DECREASING_SEQ in whole
                if not decreasing and not (IrInjectionKind.DUPLICATE in whole and seq > 0):
                    seq += 1
                sent_seq = max(seq - 1, 0) if decreasing else seq   # decreasing: always below the last good one
                received = now
                if IrInjectionKind.FUTURE_TIME in whole:
                    received = now + 1.0
                elif IrInjectionKind.STALE_AGE in whole:
                    received = now - 10 * self.edge_cfg.max_age_s
                if IrInjectionKind.SILENCE not in whole:
                    det.update(IrSample(sent_seq, received, shaped(left, IrSensor.LEFT), shaped(right, IrSensor.RIGHT)),
                               float(travel), now)
            if det.state is EdgeState.DONE:
                break
            det.check(float(travel), now)
            if det.state is EdgeState.DONE:
                break
            self._drive(["MOV,F,1"])
            travel += 1
        lag = self.edge_lag_cm if det.edge_at_cm is not None else 0
        if lag:
            self._drive([f"MOV,F,{lag}"])
            travel += lag
        self._clock_s = t0 + travel * self.s_per_cm
        total = travel + self._overshoot()
        self.follow_legs.append((leg_start, float(total)))
        return EdgeResult(det.outcome.value, float(total),
                          edge_at_cm=None if det.edge_at_cm is None else det.edge_at_cm + lag,
                          line=f"sim:{det.reason.value if det.reason else ''}")

    # -- ultrasonic -------------------------------------------------------------------------------
    def read_ultrasonic(self) -> float | None:
        """Front sensor: distance straight ahead to the first true surface;
        None unless the reading is valid (in range and the sensor healthy)."""
        us = self.ultrasonic
        if us is None or us.mode is not SimUsMode.OK:
            return None
        origin = self.pose.point(config.ULTRASONIC_FORWARD_CM)
        distance = ray_distance(origin, self.pose.forward(), self.world.rects())
        if distance is None or distance < us.min_range_cm or distance > us.max_range_cm:
            return None
        return distance

    def measure_front(self) -> stm_us.FrontReading:
        us = self.front_us
        if us is None:
            raise SensorUnavailable("the simulator was built without the US, stream")
        if us.fail:
            self.front_readings.append(None)
            return stm_us.FrontReading(None, 0, 5, [], "sim: no trusted line")
        origin = self.pose.point(config.ULTRASONIC_FORWARD_CM)
        best = None
        steps = 15
        for k in range(steps + 1):
            angle = math.radians(self.pose.h - us.half_angle_deg + 2 * us.half_angle_deg * k / steps)
            d = ray_distance(origin, (math.cos(angle), math.sin(angle)), self.world.rects())
            if d is not None and (best is None or d < best):
                best = d
        if best is None or best > us.max_range_cm:
            self.front_readings.append(None)
            return stm_us.FrontReading(None, 0, 5, [], "sim: nothing in range")
        value = best + us.offset_cm
        self.front_readings.append(value)
        return stm_us.FrontReading(value, 5, 5, [value] * 5)

    # -- final parking (USPARK) ------------------------------------------------------------
    def set_uspark(self, enabled: bool) -> str | None:
        if self.uspark is None:
            raise SensorUnavailable("the simulator was built without USPARK")
        self.uspark_sets.append(enabled)
        self.uspark_enabled = enabled
        return None

    def prepare_final_parking(self) -> str | None:
        self.set_uspark(False)
        return self.set_uspark(True)

    def park_final(self) -> stm_park.ParkingResult:
        """Drive MOV,F,100 until the fixture's stop, then classify the reply with the REAL evaluator."""
        if self.uspark is None:
            raise SensorUnavailable("the simulator was built without USPARK")
        us = self.uspark
        command = config.PARK_FINAL_COMMAND
        self.parking_commands.append(command)
        origin = self.pose.point(config.ULTRASONIC_FORWARD_CM)
        ahead = ray_distance(origin, self.pose.forward(), self.world.rects())
        travel = 100 if ahead is None else max(0, min(100, math.floor(ahead - us.clearance_cm)))
        if travel:
            self._drive([f"MOV,F,{travel}"])
        if us.lines is not None:
            lines = list(us.lines)
        elif us.reason == "PRED":
            lines = ["MM,EV:START,D:F,T:0,V:100",
                     f"USPARK,STOP,R:PRED,D:354,V:4452,FM:00,C:138,TH:{us.trigger_mm},P:{us.path}",
                     "MM,EV:END,D:F,T:1500,RESULT:STOPPED", stm_park.STOPPED_REPLY]
        else:
            lines = ["MM,EV:START,D:F,T:0,V:100", f"USPARK,STOP,R:{us.reason},D:290,V:3000,FM:00",
                     "MM,EV:END,D:F,T:1500,RESULT:STOPPED", stm_park.STOPPED_REPLY]
        stm_lines = [classify_line(text, self._clock_s) for text in lines]
        last = stm_lines[-1].text if stm_lines else ""
        final = stm_lines[-1] if last in ("ACK", stm_park.STOPPED_REPLY) or last.startswith("ERR,") else None
        body = stm_lines[:-1] if final is not None else stm_lines
        started = any(line.text.startswith("MM,EV:START") for line in body)
        result = MotionResult(command, final, tuple(body), started, final is None, False)
        return stm_park.evaluate(command, result)

    def forward_until(self, stop_cm: float, max_cm: float) -> StopResult:
        if self.ultrasonic is None:
            raise SensorUnavailable("the simulator was built without an ultrasonic")
        if self.ultrasonic.mode is SimUsMode.FAULT:
            return StopResult("SENSOR_FAULT", 0.0, None, "sim: ultrasonic fault")
        reading = self.read_ultrasonic()
        if reading is None or reading - stop_cm > max_cm:
            travel, status = float(max_cm), "MAX_TRAVEL"
        else:
            travel, status = max(0.0, reading - stop_cm), "OK"
        n = max(0, round(travel))
        if n:
            self._drive([f"MOV,F,{n}"])
        total = n + self._overshoot()
        return StopResult(status, float(total), self.read_ultrasonic())


# --- a generic fake serial line (Phase 3a) ---------------------------------------------------
@dataclass
class FakeClock:
    """A deterministic clock. FakeSerialPort advances it by `read_step_s` for every readline()
    that finds nothing, exactly like a real port's read timeout."""

    now: float = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def ordinary_move_lines(direction: str = "F", value: int = 10, final: str = "ACK") -> list[str]:
    """The lines the checked-out PROTOCOL.md (sections 4, 5, 10) says an ordinary motion produces:
    `MM,EV:START`, `MM,` telemetry, `MM,EV:END`, then one terminal reply (`ACK` or an `ERR,...`)."""
    result = "OK" if final == "ACK" else "TIMEOUT"
    return [f"MM,EV:START,D:{direction},T:0,V:{value}",
            f"MM,D:{direction},T:100,ST:DIST,TH:900,H:900,HE:0,GZ:0,SP:1500,LT:750,LP:700,LE:50,LO:0,RT:750,RP:700,RE:50,RO:0",
            f"MM,EV:END,D:{direction},T:1200,RESULT:{result}",
            final]


class FakeSerialPort:
    """A generic fake of the STM serial line for the transport tests.

    It speaks ONLY lines frozen in the checked-out PROTOCOL.md: `ACK`, `ERR,...`, `MM,` telemetry and
    events, and `PROFILE,...` replies. It knows nothing about STOP, IR, ultrasonic or READY/START.
    `script(command, lines)` queues the reply to ONE future write of that command (a command with no
    script is answered with silence). `feed(line)` injects a line at once (a late, stray or unknown line
    for a negative test). `emitted` lists every scripted line, so a test can check that only frozen
    strings were used. `on_read` runs at the start of every readline(), to act mid-transaction.
    """

    def __init__(self, clock: FakeClock, read_step_s: float = 0.1) -> None:
        self.clock = clock
        self.read_step_s = read_step_s
        self.rx: deque[bytes] = deque()
        self.written: list[bytes] = []
        self.scripts: dict[str, deque[list[str]]] = {}
        self.emitted: list[str] = []
        self.injected: list[str] = []
        self.input_clears = 0
        self.on_read: Callable[[], None] | None = None
        self.fail_reads = False
        self.fail_writes = False
        self.write_error: BaseException | None = None      # raised by write(), after the bytes are recorded if `write_after_record`
        self.write_after_record = False
        self.read_error: BaseException | None = None       # raised by readline()

    def script(self, command: str, lines: list[str]) -> None:
        self.scripts.setdefault(command, deque()).append(list(lines))

    def feed(self, line: str) -> None:
        self.injected.append(line)
        self.rx.append(line.encode("ascii", errors="replace") + b"\n")

    @property
    def in_waiting(self) -> int:
        return sum(len(chunk) for chunk in self.rx)

    def write(self, data: bytes) -> int:
        if self.fail_writes:
            raise OSError("fake write failure")
        if self.write_error is not None and not self.write_after_record:
            raise self.write_error
        self.written.append(data)
        if self.write_error is not None:                    # the bytes went out, then the call failed
            raise self.write_error
        command = data.decode("ascii").strip()
        queue = self.scripts.get(command)
        if queue:
            for line in queue.popleft():
                self.emitted.append(line)
                self.rx.append(line.encode("ascii") + b"\n")
        return len(data)

    def readline(self) -> bytes:
        if self.on_read is not None:
            self.on_read()
        if self.fail_reads:
            raise OSError("fake read failure")
        if self.read_error is not None:
            raise self.read_error
        if self.rx:
            return self.rx.popleft()
        self.clock.advance(self.read_step_s)
        return b""

    def reset_input_buffer(self) -> None:
        """A trap, not part of the link's port contract: the hardened link must never call it."""
        self.input_clears += 1
        self.rx.clear()

    def commands_written(self) -> list[str]:
        return [chunk.decode("ascii").strip() for chunk in self.written]


@dataclass
class FakeCamera:
    stm: FakeStm
    looks: list[Look] = field(default_factory=list)
    o2_bullseye_inset_cm: float | None = None   # FIXTURE: bullseyes on O2's front face this far in from each end

    def _detection(self, label: str, x: float, y: float, kind: str) -> Detection | None:
        cam = self.stm.pose.point(config.CAMERA_FORWARD_CM)
        fx, fy = self.stm.pose.forward()
        rx, ry = self.stm.pose.right()
        depth = (x - cam[0]) * fx + (y - cam[1]) * fy
        lateral = (x - cam[0]) * rx + (y - cam[1]) * ry + self.stm.world.lateral_error_cm
        if depth < 8 or depth > 250:
            return None
        norm = lateral / (depth * math.tan(math.radians(config.HFOV_DEG) / 2))
        if abs(norm) > 0.95:
            return None
        height = config.RANGE_K[kind] / (depth * (1 + self.stm.world.range_error))
        cx = (norm + 1) * config.FRAME_WIDTH / 2
        return Detection(label, 0.9, cx - height / 2, 240 - height / 2, cx + height / 2, 240 + height / 2)

    def look(self, tag: str, frames: int = 1) -> Look:
        world, pose = self.stm.world, self.stm.pose
        detections = []
        facing_north = math.sin(math.radians(pose.h)) > 0.5
        if facing_north:
            for rect, arrow in ((world.o1, world.arrows[0]), (world.o2, world.arrows[1])):
                if pose.y < rect.y0:
                    d = self._detection(ARROW_LABEL["LEFT" if arrow == "L" else "RIGHT"],
                                        (rect.x0 + rect.x1) / 2, rect.y0, "arrow")
                    if d:
                        detections.append(d)
            o2 = world.o2
            if self.o2_bullseye_inset_cm is not None and pose.y < o2.y0:
                for x in (o2.x0 + self.o2_bullseye_inset_cm, o2.x1 - self.o2_bullseye_inset_cm):
                    d = self._detection(config.BULLSEYE_LABEL, x, o2.y0, "marker")
                    if d:
                        detections.append(d)
        elif math.sin(math.radians(pose.h)) < -0.5:
            d = self._detection(config.BULLSEYE_LABEL, 0.0, world.back_y, "marker")
            if d:
                detections.append(d)
        look = Look(tag, [list(detections) for _ in range(frames)])
        self.looks.append(look)
        return look


def body_inside_carpark(pose: Pose, world: World) -> bool:
    half = config.CARPARK_WIDTH_CM / 2
    return all(-half <= x <= half and world.back_y <= y <= world.entrance_y for x, y in footprint(pose, 0.0))


def passed_sides(trace: list[Pose], world: World) -> tuple[str | None, str | None]:
    """Side of each obstacle the robot first passed (crossing its centre line)."""
    result = []
    for rect in (world.o1, world.o2):
        mid_y, mid_x = (rect.y0 + rect.y1) / 2, (rect.x0 + rect.x1) / 2
        side = None
        for a, b in zip(trace, trace[1:]):
            if a.y < mid_y <= b.y:
                side = "L" if b.x < mid_x else "R"
                break
        result.append(side)
    return result[0], result[1]


def fixture_ir(**overrides) -> SimIr:
    values = dict(range_max_cm=80.0, beyond_range=SimBeyondRange.TRUSTWORTHY_NO_WALL, sample_spacing_cm=1.0)
    values.update(overrides)
    return SimIr(**values)

