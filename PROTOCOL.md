# SC2079 MDP T7 — Integration Protocol

Canonical coordinate system, robot geometry, and inter-subsystem wire formats
for Android, the PC planner, the Raspberry Pi (RPi) and the STM32 firmware.
This is the single source of truth for wire formats; where any other
document (including the handoff doc or the integration checklist) disagrees,
this file wins. References to `CODEBASE_INDEX.md`, `TASK2_*.md` and `RTOSV/` point to the team's internal working repo (not included in this showcase; `RTOSV/` is `task1/stm` / `task2/stm` here) for the wider
repository map and file-level detail.

**Status legend**, applied to every numbered item below:

| Tag | Meaning |
|---|---|
| **[IMPLEMENTED]** | Exists in tracked source today, verified by reading it |
| **[TARGET v2]** | Not implemented anywhere yet; this is what the next stages must build |
| **[VALIDATED]** | Physically confirmed on the robot (STM index §7), for that exact configuration |
| **[PROVISIONAL]** | A team-supplied value not yet independently validated |
| **[UNRESOLVED]** | An open decision; do not assume an answer |
| **[FROZEN]** | Spelling, grammar and semantics agreed by the team (Task 2 operations, §4.2–§4.9). Says nothing about implementation: every [FROZEN] item also carries one of the status tags |
| **[MERGED]** | Exists in this branch's firmware source (`RTOSV/`), merged from `rtosv-task2@f9f058e` in Phase 3b.2. Host-tested on the source branch; **not flashed from this branch; bench pending**. Physical results reported by the team stay [USER-REPORTED] |
| **[BLOCKED]** | Frozen, but must not be implemented on hardware or used on the robot until the named blocker is resolved |
| **[USER-REPORTED]** | A hardware result reported by the team; no logs are in this repository, not independently verified, never upgraded to [VALIDATED] here |

This document is protocol-v2 in scope (see §0). Command/mission IDs are a
protocol-v2 concept and belong to the RPi/PC wrapper — they are never sent
to the STM.

## 0. Version and scope

- Protocol version: **v2** in this document's own terms. The PC planner's
  HTTP JSON schema separately carries its own `protocol_version` field
  (currently `1`); §7 below defines the `2` schema this document expects.
- Scope: Android ↔ RPi, RPi ↔ PC planner, RPi ↔ STM32. This document does
  **not** redefine the STM32 UART line protocol — §4 quotes it from
  [`RTOSV/Core/Src/vision_rx.c`](RTOSV/Core/Src/vision_rx.c) and
  [`RTOSV/Core/Src/app_motion.c`](RTOSV/Core/Src/app_motion.c); the STM
  firmware is the foundation (see the STM README) and this document adapts to
  it, never the reverse.
- **What exists today** [IMPLEMENTED]:
  - the STM firmware's motion behaviour and UART protocol (§3–§4);
  - the PC planner's HTTP API, `protocol_version` 1 (`algo/`);
  - the rear-reference footprint and camera viewing-pose geometry (§2,
    `algo/footprint.py`, `algo/targets.py`) — already used by the
    planner's collision and viewing-target code, even though the
    planner's own *turn model* is still the old symmetric 25 cm one, not
    the four measured radii;
  - the web UI (`algo/web/arena.js`) already draws the measured 24×19 cm
    robot shape, the rear-reference marker and the camera marker, using
    its own duplicated JS constants (§9).
- **Also implemented**: the four-radius executable pipeline and bounded
  prefix-tree planner (`algo/bounded_prefix_planning.py`), exposed by the
  **planning-only** protocol-v2 HTTP endpoint (`POST /plan/v2`) — see §7.
  It accepts 1-8 obstacles and returns a verified complete STM command batch,
  else the best verified partial one (most obstacles, `complete:false`), or a
  clear failure, inside its bounded response window. It remains
  experimental: one four-obstacle validation fixture missed the planning
  budget, and it executes nothing against the STM — the RPi program is not in this repo (§6.1).
  Protocol v1 (`POST /plan`) is unaffected. The automatic webpage (`/`) plans
  through `POST /plan/v2` only and treats a result as executable only when
  `valid` and `complete` are both true; v1 remains on the manual debugging
  page (`/manual.html`).
- **Task 2 STM/RPi operations** (§4.2–§4.10, §6.2): frozen on 2026-10-13 as spellings and contracts. STOP, `IR,GET` and
  `MOV,FIRL`/`MOV,FIRR` exist in this branch's firmware source since the Phase 3b.2 merge from `rtosv-task2@f9f058e`
  (**[MERGED]**: host-tested on the source branch, not flashed from here, bench pending). Measured travel and `USD,GET`
  exist nowhere (**[TARGET]**). The old `MOV,FUS` target is superseded for final parking. Predictive USPARK authority
  exists only in separate source `a26ec79` and local candidate `a94832e`; neither is integrated here, and the dedicated
  Pi transaction is **[TARGET]**. FIR hardware authority is **blocked** (§4.6). **No Task 2 hardware mission is permitted**
  and none is validated.
- **What remains unimplemented in this repo** [TARGET v2]: RPi orchestration
  (the team's RPi program is kept outside this repo and recorded in §6.1; in
  this repo `mission_id` in §7 is caller-supplied), execution/`ACK` handling, and serving
  robot/camera geometry from the backend instead of duplicating it in JS
  (§9). Sections marked [TARGET v2] describe what future stages must
  build, not current behaviour.

## 1. Canonical coordinate system (frozen)

These conventions are fixed for every subsystem going forward. Changing any
of them is a protocol change, and must be made here first (§14).

1. **Arena origin**: bottom-left corner of the arena.
2. **Physical units**: centimetres, for every physical (non-grid) value.
3. **+x**: East.
4. **+y**: North.
5. **Heading**: measured counter-clockwise from +x.
   - E = 0°, N = 90°, W = 180°, S = 270°.
6. **Robot pose (canonical position)**: the **rear-axle midpoint**, not the
   robot's centre and not any corner of its footprint. Every `(x, y,
   heading)` triple exchanged between subsystems under this protocol refers
   to this point, unless explicitly marked otherwise (see §8 for the one
   known exception: the legacy Android branch).
7. **Obstacle grid coordinate**: `(x, y)` identifies the obstacle's
   **lower-left 10 cm cell**, in the same grid Android and the planner use
   (1 grid unit = 10 cm).
8. **Obstacle physical centre**: `(10·x + 5, 10·y + 5)` cm, derived from the
   grid coordinate in item 7.
9. **Obstacle size**: 10×10 cm, axis-aligned.
10. **Obstacle image face**: one of N/E/S/W, naming the arena-facing side of
    the obstacle that carries the image.

These match [`algo/config.py`](algo/config.py)'s existing pose convention
(0 = East, counter-clockwise) and `footprint.py`'s rear-reference pose
[IMPLEMENTED], and the handoff document's team convention in
`CODEBASE_INDEX.md` §14.

## 2. Robot geometry (measured, rear-reference frame)

All measurements are relative to the rear-axle midpoint (§1 item 6), in the
robot's local frame: **+forward** along the current heading, **+left**
counter-clockwise from forward.

| Quantity | Value | Status |
|---|---|---|
| Front extent | 24.0 cm | **[PROVISIONAL]** team-measured |
| Rear extent | 0.0 cm | **[PROVISIONAL]** team-measured |
| Width | 19.0 cm | **[PROVISIONAL]** team-measured |
| Safety margin | 3.0 cm | **[PROVISIONAL]** team-chosen, applied once. Raised from 1.0 cm after the robot hit obstacles along the way; 5.0 cm was tried and set back to 3.0 cm by team decision (2026-10-03). The robot was earlier reported hitting obstacles at 3 cm; on 2026-10-03 the team reported 3 cm working on the robot with a B3 plan (user-reported, not validated) |
| Camera forward offset | 15.25 cm | **[PROVISIONAL]** midpoint of a measured 15–15.5 cm range |
| Camera lateral offset | 0.0 cm | **[PROVISIONAL]** team-measured, centred |
| Camera height | 11 cm | **[PROVISIONAL]**, documentation only — not used in 2D planning |
| Camera pitch | ≈0° | **[PROVISIONAL]**, visually estimated, documentation only — not used in 2D planning |
| Side IR sensors (Task 2) | Left: forward +19.0 cm, lateral +5.25 cm. Right: forward +19.0 cm, lateral −5.25 cm. Both point exactly sideways, ≈8.4 cm above the floor. Derivation: front plane 24.0 cm; the sensor spans ≈3.5–6.5 cm behind it (centre 5.0 cm behind: 24.0 − 5.0 = 19.0); half-width 9.5 cm − 5.25 cm = **4.25 cm inset**, so the wheels protrude 4.25 cm beyond each sensor face and a reading = side gap to the wall + 4.25 cm. The earlier 45 mm face-recess value (in the source branch's `PROTOCOL.md` and in comments inside the imported `RTOSV/`) is **superseded** (D-61) | **[PROVISIONAL]** team-measured (2026-10-12), to be re-measured; used only by `task2/` (`IR_*` in `task2/config.py`). The merged STM source answers `IR,GET` (§4.5; not flashed from this branch) |
| Front ultrasonic (Task 2) | Forward offset 19.5 cm (**195 mm**) from the rear axle to the sensor face; reading + 195 mm = rear-axle distance to the surface (supersedes the provisional 240 mm, 2026-10-13). Selected final-parking target (§4.3, USPARK-H1): stage at **100 cm** from the sensor face, then issue exactly `MOV,F,100` under the pinned predictive-USPARK build. Reported terminal clearance is about **20 cm**; hardware acceptance rejects below **15 cm**. This supersedes the unimplemented 120 mm `MOV,FUS` plan for final parking only | Offset **[TEAM-MEASURED]** 2026-10-13 (BD-12; lateral and height not recorded). Trigger-190 result and ≈20 cm clearance are **[USER-REPORTED]**; the exact 100 cm staging sequence is not yet validated on the final integrated binary |
| Camera standoff (lens to obstacle face) | 25 cm | **[PROVISIONAL]**, not physically validated, calibrated later. Raised from 20 cm to keep the robot further from the obstacle face |
| IR sensors: beam direction | LEFT sensor faces robot-left (+left), RIGHT sensor faces robot-right (-left) | **[PROVISIONAL]** user-confirmed 2026-10-12: both face sideways |

These are implemented in [`algo/config.py`](algo/config.py) and
[`algo/footprint.py`](algo/footprint.py) as `ROBOT_FRONT_EXTENT_CM`,
`ROBOT_REAR_EXTENT_CM`, `ROBOT_WIDTH_CM`, `SAFETY_MARGIN_CM`,
`CAMERA_FORWARD_OFFSET_CM`, `CAMERA_LATERAL_OFFSET_CM` and
`CAMERA_STANDOFF_CM` [IMPLEMENTED]. Camera height and pitch are recorded
here only; no field for them exists in `PlannerConfig`, and none should be
added unless a real 2D-planning consumer needs one.

IR distance readings (§4.5) are measured from the SENSOR FACE along the beam (the
calibration reference, user-confirmed 2026-10-12). The STM does not subtract any
offset. The sensor faces are inset 4.25 cm from the wheel side (derivation in the Side IR row;
**[PROVISIONAL]**, D-61), so a consumer that needs the clearance between the tyre and the
obstacle computes `reading_mm - 42.5`. The **trusted operational range is 100-200 mm from the
face [USER-REPORTED]**; above 200 mm treat the reading as "far enough" only. (The source branch's 45 mm
recess and `sensor_distance_mm - 45` are superseded and not used.)

The inflated (margin-added) footprint spans, in local coordinates:
`forward ∈ [-3.0, 27.0]` cm, `left ∈ [-12.5, 12.5]` cm.

## 3. STM turn semantics (verified, not to be re-derived elsewhere)

The STM executes exactly four turn primitives, each a fixed steering-lock
arc. The following table is copied from source and the physical validation
record; do not restate it differently in any other document.

| Command | Travel | Heading change | Steering endpoint | Radius | Status |
|---|---|---|---|---|---|
| `FL` | Forward | Increase | Servo `SERVO_MIN_US` | 24.8 cm | **[VALIDATED]** |
| `FR` | Forward | Decrease | Servo `SERVO_MAX_US` | 33.7 cm | **[VALIDATED]** |
| `BL` | Reverse | Increase | Servo `SERVO_MAX_US` | 33.6 cm | **[VALIDATED]** |
| `BR` | Reverse | Decrease | Servo `SERVO_MIN_US` | 24.5 cm | **[VALIDATED]** |

Source basis [IMPLEMENTED, from `RTOSV/Core/Src/app_motion.c`]:
`AppMotion_TurnServoEndpointUs()` maps FL/BR to `SERVO_MIN_US` and FR/BL to
`SERVO_MAX_US`. The turn-entry code computes
`targetHeadingDeg = headingNow ± command->value`, adding for FL/BL and
subtracting for FR/BR. `case APP_MOTION_TURN:` treats BL/BR as
`isReverseTurn` (encoder targets ramp backward), while FL/FR ramp forward.
The radii themselves are the physically measured values recorded in
`RTOSV/CODEBASE_INDEX.md` §7 [VALIDATED].

**Read this carefully: `BL` does not mean "reverse with the front wheels
steered the same way as `FL`."** `BL` shares its **servo endpoint with
`FR`** (`SERVO_MAX_US`), not with `FL`. The letter after the direction
describes which way the robot's **nose swings while reversing**, not which
steering lock is engaged. Concretely:

- `BL`: reverses, and the nose swings **left** (heading increases) — using
  the **same steering lock as `FR`**, which is why its radius (33.6 cm) is
  close to `FR`'s (33.7 cm).
- `BR`: reverses, and the nose swings **right** (heading decreases) — using
  the **same steering lock as `FL`**, which is why its radius (24.5 cm) is
  close to `FL`'s (24.8 cm).

The heading sign (+1 counter-clockwise) is [IMPLEMENTED, `app_imu.c`:
`headingSign = 1` at boot, flipped only by `IMU,FLIP`]. The physical turn
direction itself is **[VALIDATED]**: the team bench-tested all four
commands at 10, 30, 45, 60, 75, 90° (plus 180/270/360°, STM index §7), and
the recorded behaviour confirms real chassis motion, not just a
source-code convention:

- `FL`: heading increases, and the nose turns **left**.
- `FR`: heading decreases, and the nose turns **right**.
- `BL`: reverses, heading increases (nose swings left), steering endpoint
  matches `FR`.
- `BR`: reverses, heading decreases (nose swings right), steering endpoint
  matches `FL`.

This is a source-verified naming convention **plus** a physically
validated direction, and the two should not be conflated: the servo
endpoint mapping and the ± heading arithmetic come from reading
`app_motion.c` [IMPLEMENTED]; that this produces the described left/right
motion on the actual chassis is [VALIDATED] from the bench tests, not
re-derived from source alone.

## 4. STM32 UART interface (implemented, unchanged by this document)

Source: [`RTOSV/Core/Inc/vision_rx.h`](RTOSV/Core/Inc/vision_rx.h),
[`RTOSV/Core/Src/vision_rx.c`](RTOSV/Core/Src/vision_rx.c),
[`RTOSV/Core/Src/app_comm.c`](RTOSV/Core/Src/app_comm.c). Full command
reference: [`RTOSV/CODEBASE_INDEX.md`](RTOSV/CODEBASE_INDEX.md) §8.

**[IMPLEMENTED]** Mission-relevant commands:

```
MOV,F,<integer cm>       0-999, straight forward
MOV,B,<integer cm>       0-999, straight backward
MOV,FL,<integer degrees> 0-360, forward-left turn  (see §3)
MOV,FR,<integer degrees> 0-360, forward-right turn (see §3)
MOV,BL,<integer degrees> 0-360, backward-left turn (see §3)
MOV,BR,<integer degrees> 0-360, backward-right turn (see §3)
```

**[IMPLEMENTED]** Framing and behaviour:

- ASCII, `\n`-terminated (`\r\n` also accepted), at most 31 characters per
  line, 115200 8N1.
- **Exactly one command in flight.** The STM rejects a new motion command
  with `ERR,BUSY` if it is not idle or a command is already queued.
- Only `ACK` and `ERR,...` are **terminal** for a motion command, plus `ACK,STOPPED` (the terminal reply of a
  stop, §4.2.1; it is not success). Every other line
  (`MM,`/`AK,`/`TB,`/`MD,`/`USPARK,STOP`/etc.) is telemetry and must be
  filtered, never treated as a reply (§10 gives the exact list to expect
  between commands).
- **Wire-level STOP.** Firmware built before the Phase 3b.2 merge has no wire-level STOP (`MOTION_STOP` was only an
  internal sentinel). This branch's firmware source now has the Task 2 priority `STOP`, answered by exactly one
  `ACK,STOPPED` (frozen in §4.2.1) **[MERGED]**: merged from `rtosv-task2@f9f058e`, host-tested on the source branch,
  **not flashed from this branch, bench pending**. The Task 2 Pi does not send it yet (Phase 3b.3a) and must compare
  `ACK,STOPPED` exactly (§6.2.2).
- There are **no STM command IDs**. Every line is a bare command or reply
  with no sequence number.
- The STM never resends anything on its own, and this document requires the
  RPi wrapper to match that: **never automatically resend a command whose
  outcome is unknown** (§6).

This section must not be edited to work around anything upstream. If the
STM firmware needs to change, that is a separate, explicitly authorized
STM change (see the STM README), not a protocol edit. §4.2–§4.10 are the agreed
Task 2 additions; each says which firmware, if any, implements it.

## 4.1 Mission setup commands: STM motion profile

**[IMPLEMENTED, STM side]** Source: the `PROFILE,` block of
[`RTOSV/Core/Src/vision_rx.c`](RTOSV/Core/Src/vision_rx.c) and
`on_motion_profile_set()`/`on_motion_profile_get()` in
[`RTOSV/Core/Src/app_comm.c`](RTOSV/Core/Src/app_comm.c); background in
[`RTOSV/CODEBASE_INDEX.md`](RTOSV/CODEBASE_INDEX.md) §6/§8/§9. The RPi's use
of these commands is **[TARGET v2]** (§6 item 8). The STM is unchanged by
this section.

```
PROFILE,PRECISION   -> PROFILE,OK     power-on default
PROFILE,STANDARD    -> PROFILE,OK
PROFILE,TASK2TEST   -> PROFILE,OK     physically tuned Task 2 test profile, never the default
PROFILE,GET         -> PROFILE,PRECISION | PROFILE,STANDARD | PROFILE,TASK2TEST | PROFILE,CUSTOM
```

- Matching is exact and uppercase. Any other `PROFILE,...` line (wrong case,
  unknown name, extra fields) gets `ERR,BAD_CMD`. `CUSTOM` is only ever
  reported by `PROFILE,GET`; it has no SET form.
- SET is **idle-only**: `ERR,BUSY` when the STM is not idle. GET is always
  answered.
- Settings are RAM-only and are lost on reset; the boot default is
  PRECISION. If an individual tuning SET later changes a profile-owned
  field, `PROFILE,GET` reports `CUSTOM` until the next explicit
  `PROFILE,PRECISION`/`PROFILE,STANDARD`.
- **Terminal replies for these setup commands** are `PROFILE,OK`,
  `PROFILE,<name>` and `ERR,...`; no `ACK` is sent for a `PROFILE` command.
  §4's "only `ACK` and `ERR,...` are terminal" describes motion commands.
- **Scope.** Applying either profile restores the **complete** set of
  profile-owned settings (steering PID, WSYNC, HCTRL, RTGUARD, DPROF, SPCTRL,
  SPPROF, STOPCTRL, MOVCAL and the forward/backward SFF;
  `AppMotion_ApplyProfile()` in `app_motion.c`) [SRC]. In the current
  `PROFILE_PRECISION_*`/`PROFILE_STANDARD_*` constants only three values
  differ [SRC]: `SPCTRL_TARGET_SPEED` (4000 vs 6000),
  `SPPROF_DECEL_START_COUNTS` (4500 vs 6000) and
  `STOPCTRL_BRAKE_ENTRY_SPEED` (1700 vs 1800). The behavioural difference
  between the two named profiles is therefore currently limited to those
  three straight-motion values; every other profile-owned setting is still
  applied, but has an equal value in both. `PRESTEER`, `TURNSPEED`,
  `ACKERMANN`, `TURNBRAKE`, the servo endpoints and the four measured turn
  radii (§3) are **not** profile-owned and are never touched by a profile.
  The planner's estimated poses are identical under both profiles; any
  physical difference between them is not modelled.
- **TASK2TEST** **[IMPLEMENTED, a test profile only]** bundles the same
  profile-owned settings as the other two with Task 2 values (steering Kp 60,
  WSYNC 0.80/20.0, SPCTRL target 8000 counts/s, STOPCTRL entry 2500 counts/s,
  its own SFF configs; `RTOSV/CODEBASE_INDEX.md` §6). It is not the
  production-final Task 2 profile and is not part of the Task 1 planner's
  motion model. **Task 2 selection (D-62):** `PRECISION` remains the initial Task 2 profile; `TASK2TEST` is
  implemented but **not initially selected**.
- **Evidence status.** STANDARD's straight-motion accuracy is
  **user-reported as acceptable only**: its test conditions (distances,
  surface, battery, tyres, date) were not recorded, so it is **not** in the
  STM physical-validation record and no **[VALIDATED]** claim is made. No
  PRECISION-vs-STANDARD timing model exists, so no speed or mission-time
  benefit is claimed (§13 item 10). A request that names no profile
  resolves to PRECISION (§7), the STM's own boot profile, so the unvalidated
  STANDARD profile is used only when a requester names it.

### 4.1.1 `IMU,GET`: heading query used by preflight and turn correction

**[IMPLEMENTED]** (verified from `on_imu_get()` in `RTOSV/Core/Src/app_comm.c`; unchanged by this document).

```
IMU,GET
IMU,Y:<tenths of a degree>,GZ:<tenths of a degree per second>,SIGN:<+1|-1>      (IMU ready)
ERR,IMU                                                                          (IMU snapshot not ready)
```

`Y` and `GZ` are signed decimal integers in tenths. It is a query: the reply is the only reply for that query, it is
answered in any motion state, and `ERR,IMU` here answers the query, it is not a movement failure. The Pi trusts a
heading only after `|GZ|` has settled (`task2/stm_link.py` `SETTLED_YAW_RATE_DPS`, 1.0 °/s). A missing, malformed
or `ERR,IMU` reply means no heading: the caller fails safe (D-28), never sends a turn uncorrected.

## 4.2 Task 2 STM extensions: STOP, IR, travel, ultrasonic (frozen 2026-10-13; status per item)

**Read this first.** §4.2–§4.9 freeze the **spellings, grammar, terminal behaviour and pose consequences** of
the Task 2 operations, as decided by the team on 2026-10-13 (decisions D-56 to D-72 in
[`TASK2_FINAL_IMPLEMENTATION_PLAN.md`](TASK2_FINAL_IMPLEMENTATION_PLAN.md) §18; reasoning in
[`TASK2_PHASE3B_IMPLEMENTATION_PLAN.md`](TASK2_PHASE3B_IMPLEMENTATION_PLAN.md) §3). **Freezing a spelling is not an
implementation claim.** Each item carries its own status; §4.10 is the one-page summary.

- **STOP, `IR,GET` and `MOV,FIRL`/`MOV,FIRR` exist in this branch's `RTOSV/` firmware source** since the Phase 3b.2
  merge of `rtosv-task2@f9f058e` (**[MERGED]**): host-tested on the source branch, **not flashed from this branch,
  bench pending**. `USD,GET` (§4.8) was added to this branch's source on 2026-10-15 (host-tested, not flashed).
  Measured travel (§4.7) and `MOV,FUS` (§4.9) exist in **no** firmware: **[TARGET]**; `MOV,FUS` is also **[BLOCKED]**.
- Hardware results that the team reported for the `f9f058e` source-branch binary (STOP operated on the robot;
  the IR `100–200 mm` range checked functionally; LEFT and RIGHT FIR edge successes; priority STOP during FIR;
  `IR_NO_OBSTACLE`; `IR_NO_EDGE`; and floor checks on both sides) are **[USER-REPORTED]**. The imported handoff
  [`TASK2_IR_ALGO_HANDOFF.md`](TASK2_IR_ALGO_HANDOFF.md) supplies the claim and protocol examples, but not the
  complete logs, run count, stopping-offset distribution, heading spread or false-edge statistics. None of
  these claims is independently verified or upgraded to **[VALIDATED]** here, and the integrated branch has not
  been flashed.
- Predictive USPARK source exists separately at `stm/uspark-phase3@a26ec79624df3014afd7a4ecfc8dae08cbdc3054`,
  and local candidate merge the integrated Task 2 STM build (`a94832e`) combines it with
  the FIR line. Neither is merged into this Task 2 branch. The team reports a trigger-190 `MOV,F,100` run from about
  80 cm ending at about 20 cm without contact **[USER-REPORTED]**; the corrected source commit itself says host-tested
  and not flashed, and the complete final-run log is not in this repository. See
  [`TASK2_USPARK_ALGO_INTEGRATION.md`](TASK2_USPARK_ALGO_INTEGRATION.md).
- **Normal sensor-bounded completion is not an emergency STOP.** A movement that ends because a sensor condition
  was met ends with `ACK` (plus the records below). `ACK,STOPPED` is only the reply to `STOP`, or to an
  automatic or floor stop (§4.3, §4.9). It is never normal movement success. The only terminal-success exception is
  the dedicated final-parking transaction in §4.3 with the complete frozen context and evidence chain; no planned
  100 cm pose update is applied.
- The full Task 2 mission gate stays **closed** (§6.2.8). Freezing these spellings opens no hardware path.
- The O2 cross-span is **unknown** (D-02). No operation below assumes a span; the caller supplies every maximum
  travel.
- STM-internal detail (detection timing tables, calibration curves, USPARK trace formats) is **not** restated in
  §4.2–§4.9; §4.11 records where each paragraph of the source branch's protocol text went, and
  `RTOSV/CODEBASE_INDEX.md` (imported unedited) carries the STM-internal detail. Where §4.11 or the STM documents
  differ from §4.2–§4.9, §4.2–§4.9 win for Task 2.

### 4.2.1 `STOP`: priority emergency stop

**Status.** Spelling and contract **[FROZEN]**. STM **[MERGED]** (in this branch's firmware source since Phase 3b.2; host-tested on the source branch; not flashed
from here; bench pending; operation on the robot **[USER-REPORTED]**). Pi support **[TARGET]** (Phase 3b.3a).

| Item | Contract |
|---|---|
| Request | Exactly `STOP` (four uppercase characters + terminator). Every variant (`STOP,`, `STOP,1`, `STOP` followed by a space, a leading space, `stop`, `Stop`, `STOPX`, any control character) is `ERR,BAD_CMD` and changes nothing. `STOPCTRL,...` is an unrelated tuning command |
| Priority | Never `ERR,BUSY`. Accepted in every motion state and handled within about one motion-task period (about 10 ms), before any queued command. A queued, not-started movement is cancelled and gets no reply of its own. A running movement is braked with the existing 120 ms active brake and reported **only** through `ACK,STOPPED` (its own `ACK`/`ERR` is never sent) |
| Reply | `ACK,STOPPED`, exactly once per completed stop operation: after the brake window (about 150 ms) when moving, after about 25 ms when idle |
| Lines before the reply | Anything published before STOP was accepted (a crossing `ACK`/`ERR,...`, a pending `MM,EV:START`) is delivered **before** `ACK,STOPPED`. If a START was sent for the interrupted movement, exactly one `MM,EV:END,D:<dir>,T:<ms>,RESULT:STOPPED[,DL:<n>,DR:<n>]` precedes `ACK,STOPPED` (the `DL`/`DR` pair only once §4.7 exists). If the movement had begun but its START was not yet sent, START, END, then `ACK,STOPPED`. If nothing was running: no START, no END. Nothing for the interrupted command follows `ACK,STOPPED` |
| Duplicates | A STOP while a stop is outstanding is coalesced (no second brake, no second reply). A STOP after `ACK,STOPPED` is a new operation with its own reply. STOP is idempotent, so the Pi may resend it |
| While outstanding | `MOV` and idle-only/tuning SET commands are `ERR,BUSY`; read-only `...,GET` are answered, except that `IR,GET` is answered after `ACK,STOPPED` |
| Meaning of `ACK,STOPPED` | The brake phase finished and the motors were released. **Not** zero velocity, **not** a position |
| Pose consequence | A STOP that crossed a movement makes the pose **UNCERTAIN** (D-29); the mission aborts and never resumes. A fence STOP sent at idle with no movement pending and no START/END around its reply leaves a known pose known |
| Pi matching | `ACK,STOPPED` is matched exactly. It is **never** a movement success and never completes a query (§6.2.2) |
| USPARK interaction | Disabled for ordinary Task 2 motion. A separately pinned predictive-authority build may be used only by the dedicated terminal-parking transaction in §4.3; it does not change emergency-STOP semantics |

### 4.2.2 Interaction of `STOP` with the baseline "no STOP" statement

§4 describes the validated base, whose firmware (built before the Phase 3b.2 merge) has no wire-level STOP.
§4.2.1 describes the contract that the Pi is written against and that this branch's firmware source now implements
(not flashed from here). A Pi that sends `STOP` to an image built without it receives
`ERR,BAD_CMD` (an unknown command): it cannot confirm a stop and must treat the robot as not stopped. Preflight
therefore requires a confirmed `ACK,STOPPED` (§6.2.6), and the firmware commit that supplied it is pinned and
logged (D-63).

## 4.3 `USPARK`: final predictive parking [SELECTED TARGET; STM candidate branch; Pi IMPLEMENTED host/sim]

`USPARK,0|1|GET` and the legacy `USPARK,STOP,`/`USPARK,FAULT,` lines exist in this branch's firmware source from
`rtosv-task2@f9f058e`. Predictive authority is **not** in this Task 2 branch: corrected source is
`stm/uspark-phase3@a26ec79624df3014afd7a4ecfc8dae08cbdc3054`; local candidate merge
the integrated Task 2 STM build (`a94832e`) is not merged or flashed here.

USPARK is disabled at preflight and throughout O1/O2 driving, return and alignment. It is enabled only while idle in
the dedicated final-parking state. This narrowly supersedes D-64's “never enable USPARK” rule; it does not create a
general automatic-stop success path.

### 4.3.1 Selected final-parking sequence

1. Align and preserve at least 50 cm of straight approach.
2. With USPARK disabled, stage the ultrasonic face approximately **100 cm** from the parking wall. With the measured
   195 mm forward offset this is rear axle approximately 119.5 cm from the wall.
3. Select and verify `PROFILE,TASK2TEST`.
4. Send `USPARK,0`, `USPARK,1`, `USPARK,GET`; require `USPARK,OK`, enabled, `FM:00`, `TH:300`, `RA:320`, `AR:500`.
5. Send exactly `MOV,F,100`, once, through a dedicated parking transaction.

The team reports that a trigger-190 authority image stopped a `MOV,F,100` run begun at about 80 cm with about 20 cm
physical clearance and no contact **[USER-REPORTED]**. A faster `MOV,F,110` selected the protective FAST path and stopped
near 30 cm; an earlier `MOV,F,180` policy contacted the target and must never be used. The exact 100 cm staging sequence
still requires the guarded final-binary acceptance in `TASK2_USPARK_ALGO_INTEGRATION.md` §13.

### 4.3.2 Successful terminal evidence

Only the following ordered evidence, attributable to the dedicated `PARK_FORWARD` transaction, yields terminal
`PARKED`:

```text
MM,EV:START,D:F,...
USPARK,STOP,R:PRED,D:<mm>,V:<cps>,FM:00,C:<mm>,TH:190,P:NORM
MM,EV:END,D:F,T:<ms>,RESULT:STOPPED[,...]
ACK,STOPPED
```

The cause and END are evidence, not terminal replies. `ACK,STOPPED` is exact and is not globally successful. The Pi
does not apply `MOV,F,100` to the pose; the terminal pose is parking-constrained (or unknown if that representation is
not available), and no later movement is permitted.

`P:FAST`, `P:OVER`, legacy `R:NEAR`, any fault/non-zero `FM`, malformed/duplicate/missing evidence, plain `ACK`, timeout,
or an unattributed `ACK,STOPPED` is a safe termination but **not successful parking**. The pose is uncertain/unknown and
no camera-only or movement fallback is allowed. Manual/operator `STOP` always remains emergency cancellation.

`USPARK,TRACE,` is optional bench telemetry: log it when present, but never require it for production classification.

## 4.4 `IRRAW` and bench lines [MERGED; bench builds only; telemetry]

`IRRAW,...`, `US,...` and `USPARK,TRACE,...` are bench diagnostic lines. They are never terminal and are
logged. Note the prefix pairs `IR,`/`IRRAW,` and `US,`/`USD,`/`USFUS,`/`USPARK,`: classification is by the exact
prefix including the comma (§6.2.2).

## 4.5 `IR,GET`: side IR distances (read-only)

**Status.** Spelling and reply grammar **[FROZEN]**. STM **[MERGED]** (`APP_IR_ENABLE` defaults to 1, so the IR task is in the default image; host-tested on the source
branch; not flashed from here). The `100–200 mm` trusted range is **[USER-REPORTED]**; the whole 100–400 mm range has
**not** passed independent accuracy validation. Pi support **[IMPLEMENTED]** host-tested (Phase 3b.5a: `ir.parse_ir`, `Task2Stm.read_ir`; not run against the robot). **IR is not an emergency-stop
sensor.** Read-only: no motion authority.

```
IR,GET
IR,S:<seq>,ST:<OK|WARM|STALE|ERR|FAULT>,L:<mm|NEAR|FAR|NA>,R:<mm|NEAR|FAR|NA>,AGE:<ms|NA>,F:<hex>
```

Strict Pi grammar (full match, otherwise the reply is **malformed**):

```
^IR,S:(0|[1-9][0-9]{0,9}),ST:(OK|WARM|STALE|ERR|FAULT),L:(NEAR|FAR|NA|[1-9][0-9]{2}),R:(NEAR|FAR|NA|[1-9][0-9]{2}),AGE:(NA|0|[1-9][0-9]{0,4}),F:([0-9A-F]{2,})$
```

| `ST` | `L`/`R` | Pi status of that sensor |
|---|---|---|
| `OK` | integer 100–200 | `VALID_DISTANCE` (trusted, **[USER-REPORTED]**) |
| `OK` | integer 201–400 | `VALID_DISTANCE`, flagged `coarse`: "far enough" only, never a measured gap, never an edge |
| `OK` | `NEAR` | `INVALID` (blind or fold-back region) |
| `OK` | `FAR` | `INVALID` (`FAR` is **never** `TRUSTWORTHY_NO_WALL` while an unplugged or faulty sensor cannot be distinguished from it, D-49) |
| `OK` | `NA` or anything else | malformed |
| `WARM` | `NA` | `INVALID` (warming) |
| `STALE` | `NA` | `STALE` |
| `ERR` | `NA` | `INVALID` (transient scan error) |
| `FAULT` | `NA` | `FAULT` for **both** sensors (the fault mask is acquisition-wide) |
| not `OK` | a number | malformed |

- `S` must be greater than 0 and strictly greater than the previous reply's `S` in the same link session (equal =
  stale duplicate, smaller = invalid). `AGE` must not exceed `IR_GET_MAX_AGE_MS` (**[PROVISIONAL]** 150 ms). `F` is
  `00` for `OK`/`WARM` and contains `40` otherwise. A malformed reply is logged and treated as invalid for both
  sensors.
- Calibrated sensor distance is measured **from the sensor face**. Side-gap to the wall = reading − 4.25 cm
  (§2). The 45 mm recess in the branch's own text is **superseded** (D-61) and is not used.
- The Pi status mapping above applies accepted decisions D-23 and D-49; the freshness limit is provisional. One request outstanding; idle only on the Pi; at most 10 Hz; never during a movement. A timeout puts the
  link in `DESYNCHRONIZED` and a quiet `resync()` must complete before the next request (D-51). A reply may be
  held behind a stop (up to about 200 ms).
- **Pose consequence:** none (read-only). It never creates an edge: the Pi never derives an edge from polled
  `IR,GET` values.

## 4.6 `MOV,FIRL` / `MOV,FIRR`: forward to the far edge of an obstacle

**Status.** Spelling, grammar and completion record **[FROZEN]**. STM **[MERGED]** (host-tested on the source
branch). The team reports that the `f9f058e` source-branch binary was exercised on the robot for LEFT and RIGHT
edge success, priority STOP during FIR, no-obstacle, no-edge and floor checks **[USER-REPORTED]**; this integrated
branch has not been flashed and the evidence limitations below remain. Pi wrapper
`follow_edge(sensor, max_cm)` **[IMPLEMENTED]** (Phase 3b.5a, host and simulator only: `task2/stm_fir.py`,
`StmLink.send_sensor_motion`; against firmware whose `MM,EV:END` has no `DL`/`DR` (this repository's `RTOSV/` today) every FIR success is a `PROTOCOL_ERROR` and the mission fails safe). **Hardware authority: [BLOCKED]. FIR must not be commanded on the robot** (not by the
bench harness, not by a mission) until the disconnection/floating-input blocker below is resolved (D-57).

```
MOV,FIRL,<max_cm>     obstacle on the robot's LEFT  (left IR sensor)
MOV,FIRR,<max_cm>     obstacle on the robot's RIGHT (right IR sensor)
```

`max_cm` is an integer 1–999 (digits only; no sign, space, fraction or extra field), the **maximum forward
travel**. Anything else is `ERR,BAD_CMD`. Preconditions are those of `MOV,F` (`ERR,IMU_NOT_READY`,
`ERR,PID_NOT_SET`, `ERR,BUSY`), plus a speed-controlled profile. The speed is capped by the STM at 3000 counts/s.
The Pi chooses `max_cm` from the caller; no obstacle length is assumed anywhere (O2's span is unknown, D-02).

| Outcome (STM) | `IREDGE` `R` | END `RESULT` | Terminal | START seen | Wrapper `EdgeResult` | Pose |
|---|---|---|---|---|---|---|
| Edge confirmed, brake complete | `EDGE` | `OK` | `ACK` | yes | `EDGE` (with travel and edge position) | known; displacement per §4.7 |
| Max travel, obstacle never confirmed | `NO_OBSTACLE` | `IR_NO_OBSTACLE` | `ERR,IR_NO_OBSTACLE` | yes | `NO_WALL` | UNCERTAIN |
| Max travel, edge never confirmed | `NO_EDGE` | `IR_NO_EDGE` | `ERR,IR_NO_EDGE` | yes | `MAX_TRAVEL` | UNCERTAIN |
| Sensor unusable in the ready wait | `SENSOR` (may follow) | none | `ERR,IR_SENSOR` | no | `SENSOR_FAULT` | known |
| Sensor unusable while driving | `SENSOR` | `IR_SENSOR` | `ERR,IR_SENSOR` | yes | `SENSOR_FAULT` | UNCERTAIN |
| `STOP` | none | `STOPPED` | `ACK,STOPPED` | maybe | `STOPPED` | UNCERTAIN |
| Pi timeout | — | — | — | — | timeout | UNKNOWN, latch, STOP procedure |
| Generic `ERR,IMU_NOT_READY`, `PID_NOT_SET`, `BUSY`, `BAD_CMD`, `MOTION_TIMEOUT` | — | as §5 | as §5 | as §5 | as §5 | as §5 |

An `ERR,IR_NO_OBSTACLE`/`ERR,IR_NO_EDGE` with no START seen is an anomaly: pose UNKNOWN, confirming STOP. No
automatic retry once the command is written. `ERR,IR_*` arrive after a brake, so the wrapper treats them as
failures with uncertain pose, not as completions.

**Completion record (D-56).** A plain `ACK` is **not** a successful FIR completion. Normal success requires the
complete **ordered** record, all read after the command's write and before its terminal, with every `D` equal to the
command's:

```
IREDGE,D:<FIRL|FIRR>,R:EDGE,SA:<0|1>,SC:<n>,SR:<n>,CC:<n>,CR:<n>,UN:<n>,AMB:<n>,MISS:<n>,RDY:<ms>,W:<FAULT|READY|DATA|CFG|NA>
MM,EV:END,D:<FIRL|FIRR>,T:<ms>,RESULT:OK,DL:<n>,DR:<n>
ACK
```

`CC` is the mean wheel displacement in encoder counts, from the start of driving to the edge confirmation
(`CC ≥ 0` required; `-1` means "never"). `IREDGE` is telemetry (never terminal, never a command). An `ACK` without both
records, or with `R`, `RESULT` or `D` disagreeing, is a **protocol anomaly**: the wrapper returns `PROTOCOL_ERROR`,
pose **UNKNOWN**, and issues a confirming STOP. No new `ACK,...` terminal is introduced (every extra `ACK`-prefixed
token multiplies the hazard that code treats `ACK,STOPPED` as success). The STM change that guarantees the decision
line is flushed before `END` (a pending `IREDGE` is never dropped by the END of the same movement) is Phase 3b.4a
and does not exist yet **[TARGET]**; until then a missing `IREDGE` is an anomaly, not a success.

**Edge authority and the hardware blocker (D-57 — PD-02 NOT accepted).** The edge decision comes only from the
STM-local detector (four consecutive fresh raw scans VISIBLE, then four CLEAR; CLEAR includes `FAR`). The Pi never
derives an edge from `IR,GET` polling, `NA` is never "wall gone", and Pi-side plausibility checks can reject an edge
but never create one. **That is not sufficient for hardware**: the current STM cannot distinguish a real edge from
a disconnected or floating analogue input, which can read as `FAR`/CLEAR, and a single passing disconnect test proves
nothing about the next disconnection (D-49 is not met). FIR hardware authority therefore stays **false** until
**either**

- **(a)** a repeatable, documented hardware or firmware **fault-detection mechanism** exists that prevents a
  disconnection or floating input from producing an accepted CLEAR (with host tests and a bench demonstration of the
  mechanism itself); **or**
- **(b)** a separately reviewed **safety argument and mitigation** is approved by the team and recorded as its own
  decision.

Until then: host and simulator work on FIR is allowed; static no-motion IR characterisation is allowed; no FIR
motion on the robot; mission hardware authority for FIR is false.

**Imported hardware handoff evidence (2026-10-13; [USER-REPORTED]).** The team reports that firmware
`f9f058e` produced healthy `IR,GET` output, successful LEFT `FIRL` and RIGHT `FIRR` edge stops, priority STOP
during FIR, `ERR,IR_NO_OBSTACLE`, `ERR,IR_NO_EDGE`, and floor checks for both sides. A representative RIGHT
run includes `IREDGE,D:FIRR,R:EDGE,...,CC:3704,...` followed by `MM,EV:END,...,RESULT:OK` and `ACK`.
The handoff is preserved in [`TASK2_IR_ALGO_HANDOFF.md`](TASK2_IR_ALGO_HANDOFF.md). It does **not** include the
complete logs, repeat count, measured stopping-offset distribution, heading spread or false-edge statistics.
It also explicitly confirms that an unplugged analogue sensor may float to a plausible value. Consequently:

- the evidence establishes functional source-branch operation only;
- it does not validate the final integrated binary;
- it does not satisfy D-57/PD-02 or grant FIR hardware authority;
- it does not replace L6a characterisation, a resolution of PD-02, L6b on the final binary, or the required
  measured-travel work in Phase 3b.4;
- the handoff's approximate 45 mm tyre inset is superseded by the provisional 4.25 cm geometry in §2/D-61.

## 4.7 Measured movement travel on `MM,EV:END` (`DL`/`DR`)

**Status.** Spelling **[FROZEN]**; STM **[TARGET]** (Phase 3b.4a; **not implemented on any branch or in this branch's firmware**: the
merged `RTOSV/` firmware zeroes these values); Pi parser **[TARGET]** (Phase 3b.4b). Needs an explicit STM-change
authorization and a bench test. Without it, no sensor-bounded completion can be accepted.

```
MM,EV:END,D:<F|B|FL|FR|BL|BR|FIRL|FIRR|FUS>,T:<ms>,RESULT:<r>,DL:<int>,DR:<int>
```

Strict Pi grammar:

```
^MM,EV:END,D:(F|B|FL|FR|BL|BR|FIRL|FIRR|FUS),T:(0|[1-9][0-9]{0,9}),RESULT:([A-Z_]+),DL:(-?(0|[1-9][0-9]{0,9})),DR:(-?(0|[1-9][0-9]{0,9}))$
```

Values must fit `int32`; `-0` is malformed. `RESULT` is one of `OK`, `TIMEOUT`, `IMU_NOT_READY`, `PID_NOT_SET`, `STOPPED`,
`IR_NO_OBSTACLE`, `IR_NO_EDGE`, `IR_SENSOR` and, for `MOV,FUS`, `US_NO_TARGET`, `US_STALE`, `US_INVALID`,
`US_FAULT`.

- **Meaning (D-58).** `DL`/`DR` are the signed **brake-end encoder displacement** of the left and right wheel in
  encoder counts, from the movement's start counts to the end of the STM's 120 ms brake window; forward travel is
  positive on both wheels, reverse negative. They are **not guaranteed final physical travel**: the wheels may still
  roll after the motors are released.
- **Residual travel (gate G-SETTLE).** The hardware level that measures the residual travel (L5 of the Phase 3b
  plan) establishes a **conservative bound**, per movement type and speed, with the configuration recorded. If the
  bound is acceptable it becomes an explicit term of the Phase 4 pose uncertainty. If it is not, **sensor-bounded
  moves** (`MOV,FIRL/FIRR`, `MOV,FUS`) gain a bounded STM **encoder-settle stage** (conditional subphase 3b.4c): the
  settle timeout outcome gets its own reported spelling, which is **not** chosen now (a later protocol amendment
  defines it). Ordinary `MOV,F/B/FL/FR/BL/BR` keep their validated timing. **Emergency STOP keeps its existing
  brake contract** (§4.2.1) and needs no settle stage because its pose is UNCERTAIN and never reused.
- **Keys.** `DL`/`DR`, not `EL`/`ER` (those already mean raw absolute counters in `USPARK,TRACE` lines).
- **Ordering rule (Pi).** The travel record of movement M is the **single** `MM,EV:END` read after M's write and
  before M's terminal (`ACK`, `ERR,<code>`, or `ACK,STOPPED` when a STOP crossed M). Its `D` equals M's direction and
  its `RESULT` agrees with the terminal (`OK`↔`ACK`, `STOPPED`↔`ACK,STOPPED`, `TIMEOUT`↔`ERR,MOTION_TIMEOUT`,
  `IR_NO_EDGE`↔`ERR,IR_NO_EDGE`, and so on).
- **Present** on every `MM,EV:END`. No END means no travel: commands rejected before START, and a STOP from idle.
- **Missing or malformed** (zero or two ENDs, missing, extra or non-integer keys, `D`/`RESULT` mismatch): travel is
  **untrusted**: pose **UNKNOWN**, confirming STOP, fail safe (D-28, D-48). Never fall back to `MM,D:` samples,
  `ENC,GET` or commanded values.
- **Length.** Worst case 82 bytes including the terminator, below the STM's 256-byte buffer (a host test must assert
  it).
- **Old firmware.** Firmware without `DL`/`DR` makes the first sensor-bounded movement fail safe. No protocol-revision
  query is added now (D-63); the firmware commit is pinned in the run log.
- **Pose consequence.** This section defines the field only. The Pose tracker that consumes it (position advance,
  gates, uncertainty) is Phase 4; Phase 3b delivers the parsed, validated record to callers and changes no mission
  pose logic.

## 4.8 `USD,GET`: front ultrasonic read (read-only)

**Status.** Spelling and grammar **[FROZEN]**. STM **[IMPLEMENTED]** host-tested in this branch's `RTOSV/`
(`task2-ash-dev`, 2026-10-15: `vision_rx.c` + `app_comm.c` `AppComm_TrySendUsdGet`, `tests/comm/app_comm_usd_get_test.c`
and `tests/parser/vision_rx_ir_test.c` pass; **not flashed, not bench-checked**; the CommTask stack script was updated
but not run, it needs the Debug ELF). An earlier host-tested version existed only in the `task2rstov` working tree
(2026-10-13). Driver state -> `ST` mapping (first match wins): driver FAULT or a non-zero
fault mask -> `FAULT` (`F` = mask | 40); nothing published or the newest result older than 200 ms -> `STALE`
(`F:40`); latest VALID with the 3-sample median full -> `OK` + filtered mm (below 10 -> `NEAR`); latest VALID, filter
not full -> `WARM`; latest TOO_NEAR -> `OK,D:NEAR`; latest TOO_FAR or NO_FALL (the HC-SR04 no-echo case: the echo
line stays high to the timeout) -> `OK,D:FAR`; any other result, NO_RISE included -> `ERR` (`F:40`). Same reply slot
and deferral as `IR,GET` (behind every terminal reply, never while a STOP is outstanding, at most 4 owed, extras
dropped silently). Pi **[IMPLEMENTED]** host-tested: `Task2Stm.probe_usd_get()` (one idle query before the run turns
it on) and `front_by_query()`, which `measure_front()` uses instead of listening (repeated `S` skipped, an `S` going
backwards or a lost reply ends the reading; same trust and agreement rules as the bench lines). Read-only: no motion
authority.
**Interim Pi source (D-77, 2026-10-13; still the fallback when the probe gets no `USD,` reply):** the team's flashed `rstovv2` build (= `task2rstov` without `USD,GET`, plus
`APP_ULTRASONIC_BENCH_LOG=1` and a TASK2TEST boot profile) has no `USD,GET`, so the Pi reads the front distance by
**listening** to the unsolicited bench `US,` line (about 5 Hz, `app_comm.c` `AppComm_TrySendUltrasonicLog`; grammar
in `task2/stm_us.py`). A line is used only with `FM:00`, `F:1`, a numeric `FLT` and `AGE` <= 150 ms; one reading is the
median of at least 3 such lines (distinct `SEQ`) agreeing within 4 cm. Stationary and idle only. Read-only capture on
the robot 2026-10-13: lines arrive, values plausible, about 40 % of echoes `R:NOFALL` at ~47 cm **[VERIFIED transport;
distance accuracy not checked against a tape measure]**. The prefix `USD,` is chosen so it cannot collide with the bench `US,ST:...` lines or the `USPARK,`
family; `US,GET` is rejected.

```
USD,GET
USD,S:<seq>,ST:<OK|WARM|STALE|ERR|FAULT>,D:<mm|NEAR|FAR|NA>,AGE:<ms|NA>,F:<hex>
```

Strict Pi grammar (full match, otherwise malformed):

```
^USD,S:(0|[1-9][0-9]{0,9}),ST:(OK|WARM|STALE|ERR|FAULT),D:(NEAR|FAR|NA|[1-9][0-9]{1,3}),AGE:(NA|0|[1-9][0-9]{0,4}),F:([0-9A-F]{2,})$
```

| `ST` | `D` | Pi status |
|---|---|---|
| `OK` | integer mm from the transducer face | `VALID_DISTANCE` |
| `OK` | `NEAR`, `FAR` | `INVALID` (untrusted) |
| `OK` | `NA` | malformed |
| `WARM`, `ERR` | `NA` | `INVALID` |
| `STALE` | `NA` | `STALE` |
| `FAULT` | `NA` | `FAULT` |
| not `OK` | a number | malformed |

- `S` strictly increasing within a link session; `AGE` ≤ the Pi's freshness limit; `F` is `00` for `OK`/`WARM`, and
  contains `40` otherwise (same rules as §4.5). The numeric range, and how the driver's states map onto `ST`, are
  fixed by the STM design (3b.6b, BD-07) **before** the STM is changed: the Pi accepts only `OK` + an integer as a
  distance until then.
- One request outstanding; idle only; reply held behind a stop; line at most about 50 bytes.
- **Pose consequence:** none (read-only). It is the **fresh post-stop confirmation** read for parking: a reading
  whose `S` is newer than the stop decision's `SEQ` and whose `AGE` is small.

## 4.9 `MOV,FUS`: superseded final-parking proposal

**Status.** Spelling, grammar and outcome set were frozen as a **[TARGET]**, but USPARK-H1 selects §4.3 instead for
final parking. `MOV,FUS` remains unimplemented and is not required by the selected Task 2 mission. This section is
retained to document the earlier proposal; no hardware or Pi implementation should be started without a new review.
The original blocker was: STM **[BLOCKED]** (Phase 3b.7a), because the
predictive braking decision depends on the offline predictor on `stm/uspark-phase2-shadow`, which is shadow-only and
**not approved** for movement authority (PD-16 open). **No hardware use.** Pi wrapper
`forward_until(*, max_cm: int, target_mm: int)` **[TARGET]** (Phase 3b.7b, host/simulator only; keyword-only, so the
legacy positional `forward_until(stop_cm, max_cm)` calls in `task2/task2.py` cannot silently bind to the new
meaning; those callers migrate in Phases 4 and 7, D-67).

```
MOV,FUS,<max_cm>,<target_mm>       longest line "MOV,FUS,999,999" = 15 characters
```

`max_cm` integer 1–999 (**maximum travel**, always enforced); `target_mm` integer 50–999: the distance from the
**ultrasonic face** at which the robot must stop. Digits only, no sign, space or fraction, no leading zero. Any other
form, including a centimetre value sent as millimetres that falls below 50, is `ERR,BAD_CMD` and nothing moves. The Pi
types both fields as integers and never formats them from floats.

| Stage | Behaviour |
|---|---|
| Admission | As `MOV,F` (`ERR,BUSY`, `ERR,IMU_NOT_READY`, `ERR,PID_NOT_SET`) plus the range checks |
| Ready wait (stationary) | Two consecutive fresh, valid, fault-free ultrasonic publications within a bound, else `ERR,US_STALE`/`ERR,US_INVALID`/`ERR,US_FAULT` with **no START**. First valid reading ≤ target → `ERR,US_TOO_CLOSE`, no START |
| Driving | `MM,EV:START,D:FUS,...`; heading hold and encoders as `MOV,F`; `max_cm` always enforced |
| Normal completion | STM-local **predictive braking** stops the robot at the target; then the decision line, `MM,EV:END,D:FUS,...,RESULT:OK,DL:<n>,DR:<n>`, `ACK` |
| No target | `max_cm` reached first → brake → `ERR,US_NO_TARGET` |
| Sensor failure while driving | no new publication within a bound → `ERR,US_STALE`; N consecutive invalid → `ERR,US_INVALID`; driver fault → `ERR,US_FAULT`; always after a brake |
| Independent floor stop | filtered distance ≤ `FLOOR_MM` → priority STOP path: `USFUS,STOP,R:FLOOR,D:<mm>`, END `RESULT:STOPPED`, `ACK,STOPPED`. A safety stop, **not** a completion. `FLOOR_MM` **[PROVISIONAL]** must satisfy `FLOOR_MM ≥ 50 mm + calibrated post-trigger travel`, so a floor stop still ends inside the allowed 50–200 mm window |
| Manual `STOP` | as §4.2.1 |
| Final evidence | `USFUS,` decision line flushed before END; a fresh post-stop `USD,GET` by the Pi |

Decision and cause lines (telemetry, never terminal):

```
USFUS,R:<OK|NO_TARGET|TOO_CLOSE|STALE|INVALID|FAULT>,D0:<mm|NA>,DT:<mm|NA>,TC:<counts|-1>,SEQ:<n>
USFUS,STOP,R:FLOOR,D:<mm>
```

`D0` first valid reading, `DT` reading at the trigger, `TC` mean wheel counts at the trigger (`-1` if none), `SEQ` the
sequence number of the publication that fixed the decision.

| Outcome | START | Lines before the terminal | Terminal | Wrapper status | Pose |
|---|---|---|---|---|---|
| Target reached | yes | `USFUS,R:OK,...`, END `OK` | `ACK` | `OK` | known; displacement per §4.7 |
| Max travel first | yes | `USFUS,R:NO_TARGET,...`, END `US_NO_TARGET` | `ERR,US_NO_TARGET` | `NO_TARGET` | UNCERTAIN |
| Too close at start | no | `USFUS,R:TOO_CLOSE,...` | `ERR,US_TOO_CLOSE` | `TOO_CLOSE` | known |
| Stale/invalid/fault in the ready wait | no | `USFUS,R:<...>` | `ERR,US_STALE` / `ERR,US_INVALID` / `ERR,US_FAULT` | `STALE`/`INVALID`/`FAULT` | known |
| Stale/invalid/fault while driving | yes | `USFUS,R:<...>`, END | same codes | same | UNCERTAIN |
| Floor stop | yes | `USFUS,STOP,R:FLOOR,...`, END `STOPPED` | `ACK,STOPPED` | `FLOOR_STOP` | UNCERTAIN |
| Manual STOP | maybe | END if started | `ACK,STOPPED` | `STOPPED` | UNCERTAIN |
| Complete-record violation (missing/mismatched decision, END or terminal) | — | — | — | `PROTOCOL_ERROR` | UNKNOWN, confirming STOP |

`US_TOO_CLOSE` never has an END (no START). Wrappers report the outcome and pose validity; the **calling state**
decides what the mission does (Phases 4 and 7). `MOV,FUS` does not use USPARK and USPARK is never enabled with it.
"Measure once with `USD,GET`, drive `MOV,F,<n>`, confirm with `USD,GET`" is a blind fixed-distance move and is
**not** an ultrasonic-bounded completion.

**Historical parking values (D-60; superseded for the selected mission by USPARK-H1).** This proposal used a nominal
120 mm target and 50–200 mm range. The selected §4.3 path instead stages at 100 cm, sends exactly `MOV,F,100`, expects
approximately 200 mm terminal clearance and rejects hardware acceptance below 150 mm. The ultrasonic forward offset
(**195 mm**, rear axle to sensor face) is **[TEAM-MEASURED]** 2026-10-13 (BD-12).

## 4.10 Status summary of §4.2–§4.9

| Item | Spelling | STM source in this branch's `RTOSV/` | Pi (this repository) | Hardware use |
|---|---|---|---|---|
| `STOP` / `ACK,STOPPED` | **[FROZEN]** | **[MERGED]**; host-tested on the source branch; not flashed; bench pending; operated on the robot **[USER-REPORTED]** | **[TARGET]** 3b.3a | only after a separately authorized flash and the STOP hardware levels; mission gate closed |
| `IR,GET` | **[FROZEN]** | **[MERGED]**; 100–200 mm **[USER-REPORTED]** | **[IMPLEMENTED]** 3b.5a (host-tested) | static queries after a separately authorized flash |
| `MOV,FIRL`/`FIRR` + `IREDGE` + completion record | **[FROZEN]** | **[MERGED]**; source-branch live cases **[USER-REPORTED]**; integrated branch not flashed | **[IMPLEMENTED]** 3b.5a (host/sim; success needs `DL`/`DR`, §4.7) | **[BLOCKED]** (PD-02 (a)/(b), D-57) |
| `DL`/`DR` on END | **[FROZEN]** | absent (the merged source zeroes these values) | **[TARGET]** 3b.4b | after 3b.4a is authorized, built and flashed |
| `USD,GET` | **[FROZEN]** | **[IMPLEMENTED]** host-tested 2026-10-15; not flashed | **[IMPLEMENTED]** host-tested (`probe_usd_get`, `front_by_query`); bench stream fallback | static queries after a separately authorized flash |
| `MOV,FUS` + `USFUS,` | frozen historical proposal | absent | superseded for final parking | not required; no implementation without new review |
| Predictive `USPARK,...` + exact `MOV,F,100` | §4.3 selected contract | separate STM candidate `a26ec79`; local combined candidate `a94832e`; not merged here; trigger-190 run **[USER-REPORTED]** | **[IMPLEMENTED]** host/sim (`task2/stm_park.py`, `StmLink.send_parking_motion`, `Task2Stm.prepare_final_parking`/`park_final`, states `STAGE_FOR_USPARK`/`ARM_USPARK`/`PARK_FORWARD`); no Pi STOP yet | final parking only; exact 100 cm-stage acceptance pending; mission gate closed |
| `PROFILE,TASK2TEST` | **[FROZEN]** spelling only | **[MERGED]** | not used initially (D-62) | — |

`PROFILE,TASK2TEST`: the merged STM source accepts it as a test profile; the Task 2 Pi uses `PROFILE,PRECISION` until
testing supports another profile (D-62).

## 4.11 STM-side reference merged from `rtosv-task2@f9f058e` (Phase 3b.2)

This section records what became of the source branch's own protocol text (`rtosv-task2@f9f058e:PROTOCOL.md`
§4.2–§4.6) when that branch was merged. **§4.2–§4.9 above are normative for Task 2 and win wherever this section or
the STM documents differ.** The STM-internal detail (calibration curves, filter and detector timing, USPARK
trace formats, resource budgets) lives in `RTOSV/CODEBASE_INDEX.md`, imported byte-identically with the firmware.

### 4.11.1 Disposition of the source branch's sections

| Source section | Disposition here |
|---|---|
| §4.2.1–§4.2.4, §4.2.6 (STOP request, priority, reply, timing, duplicates, ordering) | Covered by §4.2.1; the timing table is in `RTOSV/CODEBASE_INDEX.md` |
| §4.2.5 (commands while a stop is outstanding) | Imported unedited: §4.11.2 |
| §4.2.7 (RPi integration requirements, "not yet implemented") | Superseded by §4.2.1 (exact matching of `ACK,STOPPED`) and §6.2; the "deferred" wording is obsolete |
| §4.3 (USPARK) | D-64's former "not used" decision is superseded for final parking only by USPARK-H1. The selected Pi transaction is **[IMPLEMENTED]** host/sim only (not run on the robot); source `a26ec79` and local candidate `a94832e` are not integrated here, and ordinary Task 2 motion keeps USPARK disabled |
| §4.4 (IRRAW) | §4.4 |
| §4.5 intro, §4.5.1, §4.5.2 | §4.5; the field and state tables are imported unedited: §4.11.3 |
| §4.5.3 (what the number means) | Calibration, guard band and fold-back facts stay in `RTOSV/CODEBASE_INDEX.md`. Its **45 mm face recess and `clearance_mm = sensor_distance_mm - 45` are superseded** (D-61, §2) |
| §4.5.4 (ordering, rate) | Imported, edited: §4.11.4. The timeout-then-resend advice is **not adopted** (D-51) |
| §4.6.1, §4.6.3 | §4.6 |
| §4.6.2 | Imported: §4.11.5 |
| §4.6.4 (`IREDGE,` line) | Grammar kept in §4.6. The statement that the RPi ignores `IREDGE,` is **superseded**: it is part of the completion record (D-56) |
| §4.6.5 (detection rules) | Excerpt imported: §4.11.6. The worst-case timing table and the obstacle-face-length conclusions are **not imported**: they assume an O2 length, and O2's cross-span is unknown (D-02) |
| §4.6.6 (bench validation gates) | Superseded by the Phase 3b hardware ladder; FIR hardware stays blocked (D-57) |
| §4.6.7 (RPi handling) | Superseded by §4.6 and D-56 |
| §2 IR rows of the source branch | 45 mm recess and "position not recorded" are **superseded** by §2 (D-61) |

**Stale values inside the imported `RTOSV/`.** Comments and documentation in the imported firmware tree still carry the
superseded 45 mm face recess and an "at least 30 cm long" O2 statement (for example `RTOSV/Core/Inc/app_ir.h`,
`app_ir_dist.h`, `app_comm.c`, `RTOSV/CODEBASE_INDEX.md`, `tests/ir/app_ir_edge_sweep_test.c`,
`tests/ir/calibration/README.md`). They are **superseded documentation, not normative values**: no firmware arithmetic
uses them (the STM subtracts no offset), and the tree is deliberately left byte-identical to the source commit.

### 4.11.2 Commands while a STOP is outstanding (source src §4.2.5, unedited)

"Outstanding" lasts from acceptance of STOP until its `ACK,STOPPED` has
been sent and the motion queue has been verified empty.

| Command | Reply |
|---|---|
| `MOV,...` | `ERR,BUSY` |
| Tuning and diagnostics: `PWM`, `SPD`, `HDG,F/B`, `HDG,RAW`, `TUNE,L/R/B`, `TUNE,S` | `ERR,BUSY` |
| Queued state changes: `ENC,RST`, `SERVO,<us>`, `IMU,ZERO`, `IMU,FLIP` | `ERR,BUSY` |
| Idle-only configuration SET: `WSYNC`, `HCTRL`, `RTGUARD`, `DPROF`, `SPCTRL`, `SPPROF`, `STOPCTRL`, `MOVCAL`, `PROFILE,PRECISION/STANDARD/TASK2TEST`, `SFF`, `PRESTEER`, `TURNSPEED`, `ACKERMANN`, `TURNBRAKE` | `ERR,BUSY` |
| `PID,<L/R/S>,<kp>,<ki>,<kd>` | `ERR,BUSY` while a stop is outstanding. During ordinary movement it is accepted as before |
| Every other read-only `...,GET` (including `PID,GET`, `IMU,GET`, `ENC,GET`, `SERVO,GET`, `PROFILE,GET`) | Answered normally |
| `IR,GET` | Accepted, but the reply is deferred behind the stop: it is sent after `ACK,STOPPED` (src §4.11.4) |
| `TGT`, `NUL`, `RDY` | Accepted silently, as always |
| Malformed line | `ERR,BAD_CMD` |
| Another `STOP` | Coalesced, no reply |

After `ACK,STOPPED` the next valid command is accepted normally.

### 4.11.3 `IR,GET` reply fields and states (source src §4.5.2, unedited)

```
IR,S:<seq>,ST:<OK|WARM|STALE|ERR|FAULT>,L:<mm|NEAR|FAR|NA>,R:<mm|NEAR|FAR|NA>,AGE:<ms|NA>,F:<hex>
```

Examples, each a state the driver can really produce:

```
IR,S:1532,ST:OK,L:237,R:FAR,AGE:14,F:00        healthy; LEFT 237 mm, RIGHT no target in range
IR,S:1533,ST:OK,L:NEAR,R:400,AGE:3,F:00        LEFT closer than about 95 mm; RIGHT at the 400 mm end
IR,S:1540,ST:WARM,L:NA,R:NA,AGE:6,F:00         filter still filling after boot
IR,S:1560,ST:ERR,L:NA,R:NA,AGE:5,F:44          the newest scan timed out (04) and is no fresh reading (40)
IR,S:1560,ST:STALE,L:NA,R:NA,AGE:230,F:44      the same publication 230 ms later
IR,S:0,ST:STALE,L:NA,R:NA,AGE:NA,F:40          nothing published yet
IR,S:9001,ST:FAULT,L:NA,R:NA,AGE:5,F:844       latched: ADC/DMA could not be recovered (800), last scan timed out (04), no fresh reading (40)
```

| Field | Meaning |
|---|---|
| `S` | Sequence number of the newest IR publication the reply is derived from (0 = nothing published yet). It advances about 50 per second |
| `ST` | `OK`: fresh, filter full, `L`/`R` apply. `WARM`: fresh but the filter has not filled (about the first 280 ms after boot, or after a gap of more than 100 ms in good scans). `STALE`: nothing published, or the newest publication is older than 100 ms. `ERR`: the newest scan failed (transient; the driver retries). `FAULT`: a latched acquisition fault, no measurement until reset |
| `L`, `R` | LEFT and RIGHT sensor. Integer millimetres **from the sensor face**, 100 to 400, only when `ST` is `OK` and the filtered value is within the calibrated domain plus a 5 mm guard band at each end (results inside the guard band are clamped to 100 / 400, src §4.5.3). `NEAR`: closer than about 95 mm, or saturated. `FAR`: farther than about 405 mm, or no target in range. `NA`: whenever `ST` is not `OK` |
| `AGE` | Milliseconds since the newest publication, capped at 65535; `NA` if nothing was published |
| `F` | Fault mask, hex, at least two digits: the Phase 1 bits of src §4.4 plus `40`, which means "the newest publication is not a fresh good reading". So `F` is `00` for `OK` and `WARM`, and contains `40` for `ST` `STALE`, `ERR` and `FAULT` |

**State priority** (the first match wins): `FAULT`, then `STALE`, then `ERR`, then `WARM`, then `OK`.

- A fresh failed acquisition (newest publication a failed scan, age at most 100 ms) reports `ERR`.
- The same failed publication, once its age exceeds 100 ms (IRTask stopped publishing), reports `STALE`; its transient bit stays in `F`.
- A latched fault always reports `FAULT`, at every age, until reset.

A reply is never longer than 53 bytes including the newline (a buffer bound; the host test checks it with a synthetic line the driver cannot produce).


### 4.11.4 `IR,GET` ordering, rate and reply handling (source src §4.5.4, edited)

- The STM records the request and sends the reply from the telemetry service: after any
  `ACK`/`ERR`/`ACK,STOPPED`, `MM,EV:START`, the `MM,EV:END` + terminal reply pair or
  other event line due in the same service pass, so it never delays or overtakes a
  terminal reply. While a STOP is outstanding the reply is held until after
  `ACK,STOPPED`. One reply per pass (service period 10 ms), ahead of `IRRAW` / `US,`
  bench lines.
- It is not a terminal reply: it begins with `IR,`, so an RPi that treats only `ACK`,
  `ACK,STOPPED` and `ERR,...` as terminal (§4) must simply read it and keep waiting for
  the real terminal reply of a movement.
- **Request overflow.** The RPi must keep **no more than one `IR,GET` outstanding**: send
  the next only after the previous reply arrived (and poll at 10 Hz or less; each reply is
  at most 53 bytes, about 4.6 ms at 115200 baud). The STM can hold up to **four** replies
  pending internally. A request that arrives while four are already pending **may be
  dropped, with no reply at all**. It is deliberately not answered `ERR,BUSY`: every
  `ERR,...` line is read as the terminal reply of the movement in flight, so a generic
  `ERR,BUSY` caused by a telemetry poll would be mistaken for a movement failure. An RPi
  that keeps one request outstanding never reaches the limit: the host test
  (`tests/comm/app_comm_ir_get_test.c`) polls as fast as possible and at 10 Hz, idle, across
  a movement's completion, across a STOP and across a rejected second `MOV`, and the STM's
  diagnostic counters (`AppComm_IrGetDroppedCount()`, `AppComm_IrGetPendingHighWater()`, not
  on the wire) stay at 0 dropped and 1 pending. If a reply is lost for any reason, the Pi treats it as a query timeout: the link is `DESYNCHRONIZED`, a quiet `resync()` must complete before any new request (D-51), and the Pi never resends automatically (the source branch text suggested a 500 ms timeout followed by a new request; **not adopted**).
- Latency from request to reply: one to two service passes (10-20 ms) when idle or while
  moving; while a STOP is outstanding the reply follows `ACK,STOPPED`, so up to about
  150-200 ms [INF from the 10 ms service period and the 120 ms brake window; the host
  test measured at most 14 passes of 10 ms; not measured on the robot].
- Pi wrapper: `Task2Stm.read_ir` (Phase 3b.5a): one request outstanding, idle only, at most 10 Hz, query timeout
  1 s **[PROVISIONAL]**; a lost reply desynchronises the link (D-51). Host-tested only.


### 4.11.5 FIR behaviour (source src §4.6.2, one cell edited for table safety)

| Stage | What happens |
|---|---|
| Ready wait (`IR_WAIT`) | The robot is **stationary**. It starts only after **two consecutive newly published valid scans** of the selected sensor, within **300 ms**; otherwise `ERR,IR_SENSOR` and nothing ever moved (no `MM,EV:START`, no `MM,EV:END`). A latched IR fault gives the same reply at once |
| Search | `MM,EV:START,D:FIRL,T:0,V:<max_cm>` (`D:FIRR` for the right sensor) is sent when driving begins. The detector waits for the obstacle, then for its far edge |
| Obstacle confirmed | **Four consecutive** newly published scans classed VISIBLE. The robot keeps driving |
| Edge confirmed | **Four consecutive** newly published scans classed CLEAR after the obstacle was confirmed. Both wheel targets are pulled in to the current position plus `APP_IR_EDGE_POST_CM` (default 0 cm), so the existing crossing / deceleration / 120 ms brake path ends the movement: `MM,EV:END,...,RESULT:OK` then `ACK` |
| Maximum travel reached | The ordinary end of a distance move: `ERR,IR_NO_OBSTACLE` if the obstacle was never confirmed, `ERR,IR_NO_EDGE` if it was confirmed but its edge never was. The robot brakes first |
| Sensor failure while driving | No usable (valid and fresh) scan for **200 ms**, or a latched fault: brake, `ERR,IR_SENSOR` |
| `STOP` | Highest priority at every stage (src §4.2): from `IR_WAIT` an immediate `ACK,STOPPED` with no brake; while driving the existing 120 ms brake, `MM,EV:END,...,RESULT:STOPPED` and `ACK,STOPPED`. A stopped movement reports **no** edge decision |
| Other limits | The movement's own deadline (5 s + 0.15 s/cm) still applies: `ERR,MOTION_TIMEOUT` |

An obstacle already beside the robot at the start is confirmed by the first scans (it is
reported as `SA:1` in `IREDGE`). A sensor that only ever sees open space never confirms an
obstacle: the movement runs to its maximum travel and ends `ERR,IR_NO_OBSTACLE` (it never
completes). The IR sensor is **not an emergency-stop sensor**; ultrasonic parking protection
(`USPARK`, src §4.3) is **not** applied to these movements and stays dedicated to final parking.


### 4.11.6 FIR detection rules and speed cap (source src §4.6.5, excerpt)

- **Separate from `IR,GET`.** The Phase 3 nine-sample median and `IR,GET` (src §4.5) are unchanged.
  The edge detector classifies every **newly published raw scan** itself (each publication is
  consumed once, by its sequence number; a gap resets the counts; publications made before the
  command are ignored). The selected sensor only: LEFT reads PC0, RIGHT reads PC1.
- **Classes.** Thresholds are derived at start from the same calibration curves as `IR,GET`
  (bisection over the real conversion; every count 0-4095 of both sensors is cross-checked by the
  host test). VISIBLE: `NEAR` or at most `APP_IR_EDGE_VISIBLE_MAX_MM` = 200 mm from the sensor
  face; CLEAR: `FAR` or at least `APP_IR_EDGE_CLEAR_MIN_MM` = 250 mm; in between AMBIGUOUS. At the
  default values the raw thresholds are LEFT visible >= 1536 / clear <= 1232 and RIGHT visible >= 1653
  / clear <= 1358 [SRC, host test output]. **`NEAR` counts as visible and never as clear**; an
  ERROR, stale or invalid publication, a missed scan and an AMBIGUOUS scan reset the streak and
  never count as clear.
- **Why four consecutive scans.** The Sharp GP2Y0A21YK measures only once per **38.3 +- 9.6 ms**
  with up to **5 ms** output delay (datasheet figures, verified by the user; the datasheet is not
  in this repository), so the 20 ms ADC scans see one measurement in up to three consecutive scans.
  Agreeing scans must span more than one sensor cycle (more than 47.9 ms) to be sure of two
  independent measurements: four scans span 60 ms. A 3-sample median or a 2-of-3 vote can be
  satisfied by one held measurement. `APP_IR_EDGE_SEEN_SCANS` / `_CLEAR_SCANS` below 4 do not
  compile.
- **Speed cap.** A FIR movement cruises at most `APP_IR_EDGE_SPEED_CAP_COUNTS_PER_SEC` =
  **3000 counts/s** (40 cm/s): the SPCTRL target is limited for this movement only, the configured
  target (`SPCTRL,GET`) is untouched and approach / return moves keep the profile's speed. It may be
  raised to **4500** only after the source branch's validation gates (src src §4.6.6); **6000 and 8000 are not enabled**. The source branch conditions them on obstacle-face length; that condition is **not used** here because O2's cross-span is unknown (D-02), so they stay off.

## 5. STM reply handling (implemented STM side; RPi behaviour is [TARGET v2])

| Reply | STM meaning | RPi wrapper action **[TARGET v2]** |
|---|---|---|
| `ACK` | Move completed normally | Advance the planner's estimated pose to the command's expected end pose (dead-reckoned — `ACK` confirms completion, not measured position); advance to the next command |
| `ERR,BAD_CMD` | The line did not parse | Abort the mission; this is a wrapper bug, not a transient fault |
| `ERR,BUSY` | STM was not idle, or a command was already queued | Protocol violation by the wrapper (§4: one in flight); pause and investigate, do not blindly retry |
| `ERR,PID_NOT_SET` | Movement needs PID gains that were never configured | Abort; this is a configuration fault, not something a retry fixes |
| `ERR,IMU_NOT_READY` | The IMU was not ready, either rejected before motion started or aborted mid-motion | If no `MM,EV:START` was seen for this command, nothing moved — retry is allowed once `IMU,GET` reports ready. If `MM,EV:START` was seen, the pose is now unknown — pause and replan (do not resend) |
| `ERR,MOTION_TIMEOUT` | The STM's own deadline (5 s + 0.15 s/cm or + 0.10 s/deg) expired before completion | Pose is unknown — pause and replan (do not resend) |
| `ACK,STOPPED` | Reply to the Task 2 `STOP` (§4.2.1); **not** movement success. **[MERGED]**, Pi **[TARGET]** | Match exactly. Mission aborts, pose UNCERTAIN if it crossed a movement; never resume (§6.2) |
| `ERR,IR_NO_OBSTACLE`, `ERR,IR_NO_EDGE`, `ERR,IR_SENSOR` | `MOV,FIRL/FIRR` outcomes (§4.6). **[MERGED]** | Wrapper result (`NO_WALL`, `MAX_TRAVEL`, `SENSOR_FAULT`); pose UNCERTAIN, except `ERR,IR_SENSOR` from the ready wait (no START: known). No retry |
| `ERR,US_NO_TARGET`, `ERR,US_TOO_CLOSE`, `ERR,US_STALE`, `ERR,US_INVALID`, `ERR,US_FAULT` | `MOV,FUS` outcomes (§4.9). **[TARGET]**, STM **[BLOCKED]** | Wrapper result; pose per §4.9; no retry |
| *(RPi-side timeout, no reply at all)* | Ambiguous: the command may or may not have executed | Pause. **Never resend automatically** — without command IDs, a resend could double a physical move |

The RPi wrapper does not exist yet; this table is the contract it must
implement, not a description of current behaviour. The Task 2 rows above name
outcomes that **wrappers report**; the calling mission state, not the wrapper,
decides what happens next (Phases 4–7). Any other `ERR,<code>` is unknown: pose
unknown, and a confirming STOP once §4.2.1 is available.

## 6. PC/RPi wrapper responsibilities [TARGET v2]

None of this is implemented. It defines what the (not-yet-written) RPi
application must do, and where IDs live.

1. **`mission_id`**: assigned by the **RPi**, once per plan request sent to
   the PC planner. The planner echoes it back unchanged in its result
   (§7). It identifies one obstacle-visiting run.
2. **`seq`**: assigned by the **planner**, one per emitted action within a
   mission (0-based). It identifies one STM command or one `SNAP` inside
   that mission.
3. **IDs never reach the STM wire.** The STM protocol (§4) has no ID field
   and none is added to it. `mission_id`/`seq` exist only in RPi-side logs
   and in the RPi↔PC/RPi↔Android messages.
4. **Execution model**: the RPi sends exactly one STM command, then waits
   for a terminal reply (§5) before sending the next. It never pipelines.
5. **`SNAP` is an RPi action, never an STM command.** On `SNAP`, the RPi
   captures a frame, runs recognition, and sends `TARGET,<obstacle>,<id>`
   to Android. Nothing is sent to the STM for this step.
6. **Disconnect behaviour**:
   - **PC or Android disconnects mid-mission**: let the in-flight STM
     movement finish to its terminal reply, then **pause** — do not send
     the next command until reconnection and an explicit resume.
   - **STM link is lost, or a command's outcome is uncertain** (§5's last
     row): pause and mark the current pose **unknown**. Do not guess a
     pose and do not auto-resend.
7. **After each `ACK`**, the RPi sends Android a
   `ROBOT,<x>,<y>,<direction>` update built from the command's expected
   end pose. **This is an estimated, dead-reckoned pose, not a physical
   measurement** — `ACK` only confirms the STM finished executing the
   command, it does not report where the robot physically ended up. If a
   command fails after motion started (§5: `ERR,IMU_NOT_READY` post-start,
   `ERR,MOTION_TIMEOUT`, or an RPi-side timeout), the pose becomes
   **unknown**, and no `ROBOT` update should claim a position until the
   robot is repositioned or re-localised. (Conversion to Android's grid is
   governed by §8, currently unresolved.)
8. **STM motion-profile setup** [TARGET v2]. Every `PLAN_RESULT` carries the
   resolved `motion_profile` and a top-level `stm_setup` object (§7); it is
   never an entry of `actions`. Once the complete mission has been
   downloaded, with the STM idle and before action 0, the RPi:
   1. sends `stm_setup.set_command` (plus the `\n` terminator) **once** and
      waits for exactly `stm_setup.expected_set_reply`;
   2. sends `stm_setup.verify_command` and waits for exactly
      `stm_setup.expected_verify_reply`. No `MOV` is sent before this passes;
   3. filters telemetry as usual (§10).

   It **aborts mission execution**, with no retry and no `MOV` sent, on any
   `ERR,...` (including `ERR,BUSY` and `ERR,BAD_CMD`), on a timeout, and on
   any other reply, including `PROFILE,CUSTOM` and the wrong profile name.
   It never sends a `PROFILE` command during a mission, also not after a
   replan. It performs the setup for every profile: STM settings are
   RAM-only (the STM itself boots in PRECISION), so an earlier session could
   have left another profile or CUSTOM. A requester may name the profile; a
   request that names none resolves to PRECISION (a team decision, §7). Neither
   the planner nor the RPi picks a profile from the route, mode or distance.

## 6.1 Android <-> RPi: plan request and START/READY [IMPLEMENTED OUTSIDE THIS REPO; not run on the robot]

Status. The RPi program is the team's `rpi_task1/` folder, kept outside this
repo; this section records what its source does. **Verified from source** (its
`task1.py`, `planner_client.py` and hardware-free unit tests, which pass) for the
Pi side, and from source on the Android branches `origin/android` and
`origin/android-type-obstacles-text-entry` (parser and Start-button state). **Not
verified on the robot or the tablet** (list at the end).

1. **Android -> RPi `PLAN_REQUEST`** (one JSON line, sent on Submit):

   ```json
   {"type":"PLAN_REQUEST","protocol_version":1,"mode":"B3","coordinate_units":"grid",
    "start":{"x":2,"y":1,"heading":"N"},"obstacles":[{"id":1,"x":4,"y":5,"face":"E"}]}
   ```

2. **Translation on the RPi.** The RPi **ignores Android's `protocol_version` and
   `mode`** and never forwards this body. It reads only `coordinate_units` (`grid`
   or `cm`), `start` and `obstacles`, then builds its own §7 request:
   `protocol_version` 2, `mode` from its own `--mode` option (default **B3**; B2 is
   for testing only), a new `mission_id` per plan, `coordinate_units` `cm`,
   `start.heading_deg`, `motion_profile` from its `--profile` option (default
   PRECISION) and `allow_alternative_viewing` true unless `--no-alt-viewing`. It
   posts that to `POST /plan/v2`, and refuses a `--planner` URL with any other
   path (for example the v1 `/plan`). Which planner runs is decided only by that
   URL, never by a `protocol_version` field (§7.1), and the page at `/` shows the
   version and mode of the newest request. The HTTP timeout must exceed the
   planner's 74 s ceiling (the RPi's `task1.py` uses 95 s).
3. **START/READY handshake** (one line each, ending `\n`):

   | Direction | Line | Meaning |
   |---|---|---|
   | Pi -> Android | `READY` | Plan received, STM profile set and verified (§4.1, §6 item 8), IMU answered. The robot moves as soon as `START` arrives |
   | Android -> Pi | `START` | Move now; acted on only after `READY`. Case-insensitive; `RESUME` is a synonym |
   | Pi -> Android | `MSG,[Ready: N moves, press Start]` | Human-readable twin, sent right after `READY` |
   | Pi -> Android | `MSG,[START ignored: ...]` | `START` came before `READY` or with no armed plan; the robot did not move |
   | Pi -> Android | `MSG,[Not ready: <reason>]` | STM setup failed; no `READY` is sent |

   Rules: `READY` means `START` will move the robot, and nothing moves before
   `START`. One run per `READY`. A new `PLAN_REQUEST` replaces an armed plan, and
   a newer one received while planning wins. `READY` is sent again when the
   tablet reconnects while a plan is armed or a mission is paused. Planning takes
   up to the planner's ceiling (§7), so `READY` arrives after that plus the STM
   setup. `ROBOT,<x>,<y>,<N/E/S/W>` and `TARGET,B<n>,<id>` are unchanged (§6
   items 5 and 7).
4. **Not verified on the robot or tablet:** `READY` appearing on the tablet; an
   early `START` not moving the robot; `START` after `READY` moving it with no
   setup pause; `READY` after a Bluetooth reconnect; an idle wait between `READY`
   and `START` causing no STM or IMU problem. Whether the tablet build in use
   gates its Start button on `READY` is **unknown** (the branches above do).

## 6.2 Task 2 RPi link, STOP, START/READY and recovery [TARGET]

**Decision status of this section.** Items that cite a decision (D-59, D-62, D-63, D-66, D-69) were accepted by the
team on 2026-10-13. Items marked **[PROPOSED]** are the Phase 3b plan's design (plan §6 and §7), recorded here so
the implementation has one place to read; they are not separately approved and bind an implementation only after the
relevant subphase's PLAN has been reviewed.

Scope: the Task 2 Pi program (`task2/`), STM-facing and Android-facing infrastructure only. **Nothing in this
section is implemented in this repository yet** (Phase 3b.3 to 3b.8 are open), and nothing here puts any O1, O2,
turnaround, return, bullseye or parking behaviour on the wire: those mission states belong to Phases 4 to 7 and use
the wrappers, they are not defined here. Section 6.1 (Task 1) is unchanged. The Phase 3a link
(`task2/stm_link.py`, `task2/stm_protocol.py`, committed in `b8e433b`, 185 tests) already provides one receive path, the
no-input-clearing rule, the `DESYNCHRONIZED` state and the `motion_outcome_unknown` latch (D-51, D-54, D-55); this
section extends it.

### 6.2.1 Wrapper results

Wrappers report an outcome and the pose validity (`KNOWN`, `UNCERTAIN`, `UNKNOWN`); they never decide what the
mission does next and never retry a command once it was written. Outcomes are listed in §4.5 to §4.9. A calling state
(Phases 4 to 7) maps them to `FAIL_SAFE`, `EMERGENCY_STOPPED` or a continuation.

### 6.2.2 Link rules [PROPOSED, except that signal handlers perform no serial I/O (D-59)]

1. **One reader, one thread.** Exactly one serial reader. `STOP` is written by the thread that reads. Signal handlers
   only set a flag; they perform **no serial I/O** (D-59).
2. **Classification** is by exact line or exact prefix (including the comma), first match wins: `ACK` terminal;
   `ACK,STOPPED` stop-terminal; `ERR,<code>` terminal; `MM,EV:START`/`MM,EV:END` motion records; the listed
   telemetry families; `IREDGE,`/`USFUS,` decision lines; `USFUS,STOP,`/`USPARK,STOP,`/`USPARK,FAULT,` cause lines;
   `US,`/`IRRAW,`/`USPARK,TRACE,` bench lines; `IR,`/`USD,`/`PROFILE,`/`IMU,` query replies, each only for its own
   query; anything else (including `ACK,` plus other text, `ack`, `NACK`) is **unknown** evidence and never
   terminal. `USFUS,STOP,` is tested before `USFUS,`. Lines over 512 bytes are unknown evidence.
3. **STOP overlay and motion gate.** A STOP overlay (`NONE`, `REQUESTED`, `SENT`, `CONFIRMED`, `UNCONFIRMED`) is
   independent of the command state. The motion gate is `OPEN` or `HELD`. A confirmed STOP is the only thing that
   clears the "may still be moving" latch, and it clears only that: the pose stays UNCERTAIN and the gate stays
   `HELD` for the life of the link.
4. The Pi writes only strings from the frozen set (the Phase 3b plan §5.1): every emittable string is checked
   against it and the 31-character STM limit by a test.

### 6.2.3 Emergency STOP procedure (timing values: D-59, provisional)

**Pi status (2026-10-13):** implemented in `task2/stm_link.py` (`request_stop` flag from SIGINT/SIGTERM,
`emergency_stop` written by the reading thread, fence STOP before the run and at exit, a confirming STOP after
FAIL_SAFE) and host-tested; **not bench-validated (L3)**, so the D-53 gate stays closed and robot runs go through
`task2/supervised_run.py`. Simplification: the overlay states are reduced to `stop_state` NONE/CONFIRMED/UNCONFIRMED
plus a held motion gate.

- Allowed in every link state except a dead port; it bypasses the one-command admission rule.
- `STOP_REPLY_TIMEOUT_S` **[PROVISIONAL]** 0.5 s; `STOP_MAX_ATTEMPTS` **[PROVISIONAL]** 3; `STOP_DRAIN_S`
  **[PROVISIONAL]** 0.3 s. These three values are **calibrated on the robot** at the STOP hardware level (L3) and
  recorded; they are not tuned elsewhere (D-59).
- Attempt 1 writes `STOP`. If no `ACK,STOPPED` arrived within the timeout, resend (idempotent) up to the maximum. The
  first `ACK,STOPPED` read after the first write confirms the stop; up to `attempts − 1` more during the drain window
  are attributed to the retries; more are anomalies (evidence, link desynchronised).
- No `ACK,STOPPED` after the last attempt, or a dead port that cannot be reopened: **`STOP_UNCONFIRMED`**. Everything
  except operator-triggered STOP attempts is refused; the operator is told to physically secure the robot; the
  process exits with a distinct status. Never reported as stopped.

### 6.2.4 STOP triggers [PROPOSED] and signals (D-59)

STOP is sent on: SIGINT or SIGTERM; a Pi movement timeout; a serial failure while a movement may be active (after one
bounded reopen); mission cancellation while a movement is pending; an unexpected terminal reply; an unsolicited
`ACK,STOPPED`; any outcome that makes the pose unknown (a "confirming STOP"); and the end of every mission (a "fence
STOP" at idle).

**Signals (D-59).** The first SIGINT or SIGTERM sets the stop flag; the main loop then performs the STOP procedure. A
second signal within 2 s raises `KeyboardInterrupt`, after which `finally` still attempts STOP. No handler writes to
the port.

An `ACK,STOPPED` with no STOP written while a movement is pending is normally an **automatic stop of unknown cause**:
pose UNCERTAIN, mission `EMERGENCY_STOPPED`. The only exception is the dedicated final-parking transaction of §4.3,
which returns terminal `PARKED` only for the complete matching `PRED/NORM/190 + stopped END + ACK,STOPPED` chain.
That exception never advances the pose by the commanded 100 cm and never permits further motion.

### 6.2.5 Sensor-move timeouts

Pi timeout for `MOV,FIRL/FIRR` and `MOV,FUS` = the STM deadline (5 s + 0.15 s per `max_cm`) + the STM ready wait
(0.3 s) + a margin (3 s), plus the settle timeout if the conditional settle stage (§4.7) exists. **[PROVISIONAL]**
(D-66): tuned during hardware validation; a Pi timeout is UNKNOWN pose and starts the STOP procedure.

### 6.2.6 START/READY preflight (Task 2) [PROPOSED order; profile D-62, firmware pin D-63]

**Layers stay separate.** Pi → Android `READY`; Android → Pi `START` (the existing §6.1 spellings). **No STM `READY` command is added**: the STM's
existing `RDY` token is accepted silently and proves nothing. Preflight, **before** `READY`, in this order:

1. Open the port; send a **fence `STOP`** and require `ACK,STOPPED` (establishes stopped/idle even if a previous
   process died mid-move).
2. Quiet `resync()`.
3. `PROFILE,PRECISION` → `PROFILE,OK`, then `PROFILE,GET` → exactly `PROFILE,PRECISION` (§4.1). `PRECISION` is the initial
   Task 2 profile until testing supports another (D-62).
4. `IMU,GET` answered with a heading and a settled yaw rate (§4.1.1).
5. `IR,GET`: `ST:OK`, fresh, advancing `S`, both sensors present as values.
6. `USD,GET`: `ST:OK`, fresh (only once the STM supports it; **until then this check fails**, with no degraded mode, D-28).
7. Capability check: every capability the run needs is implemented **and** bench-evidenced, **and** the mission
   certification of §6.2.8 exists.
8. Send `READY` and `MSG,[Ready: Task 2, press Start]`.

Any failure: `MSG,[Not ready: <reason>]`, no `READY`. The firmware commit that supplied STOP, `IR,GET` and travel is
**pinned and written to the run log**; no protocol-revision query exists (D-63, PD-11 deferred).

`START` (case-insensitive) is accepted **only after `READY`**; before it: `MSG,[START ignored: not ready]` and nothing
moves. `RESUME`, `ALG:START` and `BEGIN` are **not** accepted for Task 2. One run per `READY`; `READY` is re-sent if
the tablet reconnects before `START`. After `START`: capture the IMU baseline (robot stationary), one fresh
`IR,GET`/`USD,GET` freshness check, then hand over to the mission (Phases 4 to 7). **After a valid `START`, an
Android disconnection does not pause the locally protected mission** (D-62); this differs from Task 1 §6 item 6.

### 6.2.7 Recovery (one mission per process: D-62)

**One mission per process (D-62).** After `SUCCESS`, `FAIL_SAFE`, `EMERGENCY_STOPPED` or `STOP_UNCONFIRMED` the
program sends a fence STOP (if the port allows), reports, and exits with a distinct status. Recovery = the operator
secures and re-places the robot and restarts the program (full preflight, `READY`, `START`). There is **no in-process
re-arm**, no resume after a stop, and no API that reopens the motion gate.

### 6.2.8 Capability records and the mission gate (D-69)

Each capability (`STOP_TRANSPORT`, `MEASURED_TRAVEL`, `IR_QUERY`, `IR_EDGE_MOVE`, `US_QUERY`, `USPARK_FINAL`,
`READY_START`) has independent `implemented`, `bench_validated` and, for movement primitives, `hardware_authority`
records, set only by a reviewed commit that cites evidence. Phase 3b may mark a primitive implemented and
bench-validated. **That does not open the mission.** The full Task 2 mission gate (`MISSION_CERTIFIED`, and the D-53
hardware refusal) stays **closed** through Phase 3b and opens only when all of these hold: every needed capability
has `bench_validated` and `hardware_authority`, in particular FIR (§4.6 blocker resolved) and final predictive USPARK
(pinned integrated binary plus the §4.3/hand-off acceptance runs); Phases 4 to 7 are implemented and accepted; and Phase 8 certification passes (full simulation matrix through
the wire simulator with zero contacts; full-course and all-arrow-pair hardware runs; stopping reserve calibrated;
stack and heap margins measured under load; the flashed build recorded; Task 1 impact recorded; ultrasonic mounting
measured). Until then the guarded bench harness is the only code path that moves the robot, within its own limits.

## 7. Planner result schema v2 [PARTIALLY IMPLEMENTED — planning only]

The **planning** side of this schema is implemented: `POST /plan/v2`
(`algo/planning_service_v2.py`, Original Roadmap Step 5) generates it from
the four-radius executable planner. The planner-side **validation and echo**
of `mission_id` is implemented today (required, non-empty string, rejected
otherwise); only the RPi's own **assignment, storage across a mission, and
execution** of it (§5-§6) remain [TARGET v2] — no RPi exists yet to assign
one or to consume this schema.

Request shape (compact; `mission_id`/`mode`/`coordinate_units`/
`start.heading_deg` are all required; `allow_alternative_viewing` and
`motion_profile` are optional):

```jsonc
{
  "type": "PLAN_REQUEST",
  "protocol_version": 2,
  "mission_id": "<required, non-empty string>",
  "mode": "B3",                          // required: "B2" | "B3"; team default B3
  "coordinate_units": "cm",              // required: "cm" | "grid"
  "allow_alternative_viewing": false,    // optional, default false; if
                                          // present must be an actual
                                          // boolean (JSON null is rejected)
  "motion_profile": "PRECISION",         // optional: "PRECISION" | "STANDARD",
                                          // exact uppercase only; "PRECISION"
                                          // when absent; anything else (incl.
                                          // null, "CUSTOM") -> PLAN_ERROR
  "start": { "x": 0, "y": 0, "heading_deg": 0 },
  "obstacles": [ { "id": 1, "x": 0, "y": 0, "face": "N" } ]  // 1-8 items
}
```

Result shape:

```jsonc
{
  "type": "PLAN_RESULT",
  "protocol_version": 2,
  "mission_id": "<required in the request; echoed back verbatim, never invented>",
  "motion_model": "stm-4radius",
  "turn_radii_cm": { "FL": 24.8, "FR": 33.7, "BL": 33.6, "BR": 24.5 },
  "mode": "B3",
  "termination": "search_finished",
  "delivery": "on_time",
  "route_source": "final",
  "incumbents_received": 1,
  "optimality_proven": false,
  "complete_route_proven_impossible": false,
  "search_space_exhausted": false,
  "message": "Best complete verified route found. Optimality not proven.",
  "motion_profile": "PRECISION",         // resolved profile; in every PLAN_RESULT
  "stm_setup": {                          // top level, NOT an action (§4.1, §6 item 8)
    "set_command": "PROFILE,PRECISION",
    "expected_set_reply": "PROFILE,OK",
    "verify_command": "PROFILE,GET",
    "expected_verify_reply": "PROFILE,PRECISION"
  },
  "valid": true,
  "complete": true,
  "final_pose": { "x_cm": 0, "y_cm": 0, "heading_deg": 0 },
  "obstacle_order": [/* ... */],
  "skipped_obstacle_ids": [/* ... */],
  "target_poses": [/* ideal viewing pose per visited obstacle */],
  "actions": [
    {
      "seq": 0,
      "type": "MOV",
      "dir": "FL",
      "value": 90,
      "unit": "deg",
      "stm_command": "MOV,FL,90",
      "start_pose": { "x_cm": 0, "y_cm": 0, "heading_deg": 0 },
      "end_pose":   { "x_cm": 24.8, "y_cm": 24.8, "heading_deg": 90 },
      "waypoint_start_index": 0,
      "waypoint_end_index": 4
    },
    { "seq": 1, "type": "SNAP", "obstacle_id": 3, "pose": { "...": "..." } }
  ],
  "waypoints": [/* swept samples along every MOV action, for display and validation */],
  "total_distance_cm": 0, "total_cost": 0,
  "failure_reason": null
}
```

Rules for this schema:

- `mission_id` is a **required**, non-empty string in every request; the
  planner never generates one and always echoes the caller's value
  unchanged, in every response (success or clear no-route/failure).
- The server-owned bounded policy measures from request acceptance: planning
  target R0+67 s, contained-worker hard stop R0+70 s, route verification
  deadline R0+71 s, delivery deadline R0+72 s, HTTP response ceiling R0+74 s
  (from 2026-10-07, with the 200k step limit; it was 63/66/67/68/70 s earlier
  that day, 67/81/86/88/90 s until 2026-10-07, 60/74/79/81/83 s until
  2026-10-05 and 30/45/58/60 s before 2026-10-04).
  No client field can alter it. The caller's HTTP timeout must exceed the
  ceiling (the smoke client and web page use 79 s).
- **Mode: the team default is `"B3"`.** The RPi sends it in every request;
  `mode` stays required and the server has no default. B3 runs B2 first, keeps
  that route and replaces it only with a strictly shorter one, so it is never
  worse than B2. It always uses the whole planning target, so a B3 plan comes
  back about 63-66 s after the request (inferred: replies measured with 83-120 s
  limits came 0-3.7 s after the limit; not measured at 63 s). B2 stops at its first complete route.
  Measured in planner-only tests on one machine (24 generated 6-8 obstacle
  layouts with safe viewing poses; the generator's spacing rules are an
  assumption, these are not NTU maps; margin 3 cm, tolerance 5 cm / 5 deg,
  one plan at a time, laptop sleep off): a 60 s search planned 24/24 with
  every answer within 68 s and the planner never killed at the hard stop; a
  30 s search planned 22/24. B3's route cost was identical across its 30, 50,
  60 and 70 s searches on every layout planned by both, and equal to B2's on
  the 21 layouts both solved (B2 answered in a median of about 8 s). A hard
  layout needed 20-55 s for its first route and varied from
  run to run, and one run that overlapped a laptop standby returned late and
  was discarded, so timings are machine-dependent: re-measure on the
  competition laptop.
- **B3 time-to-best (measured 2026-10-07, one PC, current config: 3 cm margin,
  3 cm / 5 deg tolerance, PRECISION, alternative viewing on, 62 generated 6-8
  obstacle layouts, not NTU maps, one plan at a time):** B2's first complete
  route arrived after a median of 8.8 s, 90th percentile 27.5 s and at most 78.2 s
  (R7_17; the 9 slowest layouts repeated 3 times varied by at most 1.4 s each);
  61/62 layouts had it by 63 s and 62/62 by 90 s. B3 found no cheaper route than
  B2's on any layout within 120 s; in a 600 s pilot on 6 layouts it improved 1
  (R6_03: 635 -> 525 at 331 s, -> 521 at 482 s). So with the 63 s target B3 returns
  B2's route. R7_17 needs about 79 s and is not planned: no ceiling at or below
  70 s plans it (the 67 s target lost it too). Killing the worker and confirming
  its death took about 0.01 s in planner-only tests. Re-measure on the competition
  laptop; the raw data is not in the repo.
- **Goal tolerance: 3 cm position / 5 deg heading** (`config.py`
  `LATTICE_GOAL_POSITION_TOLERANCE_CM`; it was 5 cm / 5 deg until 2026-10-06, 3 cm
  from 2026-10-06 15:00, 5 cm again for testing from 15:43 that day, and 3 cm again
  from 2026-10-07).
  A leg may end anywhere inside this region around its ideal viewing pose, and
  the search tends to stop at the near edge of it, so a 5 cm region left the
  camera up to about 5 cm off the obstacle centre. Measured on one PC with B3
  (67 s search, 3 cm margin, alternative viewing on, 62 generated 6-8 obstacle
  layouts, the generator's spacing rules are an assumption, not NTU maps):
  at snaps looking straight at the obstacle the camera's sideways offset was
  at most 4.98 cm with 5 cm (27% over 3 cm) and at most 2.95 cm with 3 cm (none
  over 3 cm); both planned 60 of 62 layouts, every reply took under 75 s, and
  routes were 5.8% longer on average (up to 20%). A 3 deg heading tolerance
  gave the same offsets, so the heading tolerance stays 5 deg. Alternative
  viewing poses are deliberately off-centre and unaffected (about 3% of snaps,
  up to 7-10 cm). A 1 cm position tolerance found no route on a test map.
  These are planner estimates; robot drift is not modelled and the offsets
  have not been measured on the robot.
- `valid:true` is emitted only for an independently replay-verified, on-time
  route after verified worker-tree cleanup. `complete:true` means it visits
  every obstacle. Partial fallback (from 2026-10-07): when no complete route is
  found within the planning budget, the best partial route the search found
  (most obstacles visited, then lowest cost) gets the same full replay and is
  sent with `valid:true`, `complete:false`, `route_source:"partial"` and the
  missed obstacles in `skipped_obstacle_ids`; a complete route always wins.
  Every other ordinary outcome is HTTP 200 `PLAN_RESULT` with `valid:false`,
  `complete:false`, a clear `message`/`failure_reason`, and empty
  `actions`/`waypoints`; unverified or late (diagnostic) geometry is never
  serialized. The RPi executes whenever `valid` is true, and should tell the
  operator which obstacles a partial route skips; `SNAP` stays an RPi/camera
  action and is never an STM command.
- Every pose object anywhere in this schema (`target_poses`, `final_pose`,
  every action's `start_pose`/`end_pose`, every SNAP's `pose`, every
  `waypoints` entry) is `{ "x_cm", "y_cm", "heading_deg" }` —
  **`heading_deg`**, normalised to `[0.0, 360.0)` (never `heading_rad`,
  never negative, never `-0.0`), matching §1's own degree-based definition.
  This differs from protocol v1's `heading_rad` convention by design.
- Every `MOV` action carries an **integer** `value` (§4) and its **exact**
  `stm_command` string — the RPi sends that string unmodified. `unit` is
  `"cm"` for `F`/`B` and `"deg"` for `FL`/`FR`/`BL`/`BR`.
- `start_pose`/`end_pose` are the planner's **estimated** rear-reference
  poses (§1 item 6) — dead-reckoned from the same integer that will be
  sent to the STM, not from a continuous pre-rounding value, so the model
  and the physical command agree exactly. Never described as "achieved,"
  "observed" or "measured": nothing in this schema, and no `ACK`, reports
  where the robot physically is.
- `waypoint_start_index`/`waypoint_end_index` are inclusive indices into
  the one, response-wide `waypoints` array, covering that MOV's entire
  swept path including a boundary pose shared with the previous action —
  not just newly appended samples. This is the mapping a future webpage
  needs to draw a completed/current/remaining route without reconstructing
  curves client-side. `SNAP` actions carry no such range ("no swept path");
  a `SNAP`'s `pose` is the corresponding leg's *estimated* final pose, not
  its ideal viewing target.
- **No timing field exists in this schema.** No calibrated timing model
  has been built for the four-radius planner; `total_cost` is a purely
  geometric-length-plus-reverse-penalty planning cost and must never be
  described as, or converted to, time.
- `SNAP` actions never carry `stm_command`, `unit` or a turn/straight
  `value` — they are RPi-only (§6 item 5).
- **`motion_profile`** is an optional request field: exactly `"PRECISION"` or
  `"STANDARD"`, uppercase only. Absent resolves to `"PRECISION"` (the team's
  default and the STM's own boot profile; it was `"STANDARD"` from 2026-10-03
  to 2026-10-05, and the RPi's setup command still sets and verifies it). `null`,
  other cases, `"CUSTOM"`, the empty string, any other value and any
  non-string is a request error (`PLAN_ERROR`). A requester may name
  `"STANDARD"` explicitly; the planner never picks a profile from the route,
  mode or distance.
- Every `PLAN_RESULT` (complete, partial and no-route alike) echoes the
  resolved `motion_profile` and carries a top-level `stm_setup` object with
  exactly `set_command`, `expected_set_reply`, `verify_command` and
  `expected_verify_reply` (§4.1; plain strings, no newline: the RPi adds
  it). `PLAN_ERROR` carries neither field.
- `stm_setup` is **not a path action**: no `seq`, pose or waypoint range, and
  it never appears inside `actions`. The profile is never passed to the route
  search and changes nothing it computes: obstacle order, geometry, radii,
  collision checking, `MOV` values, estimated poses, waypoints and `seq`
  numbers are identical for both profiles. What a profile changes on the
  STM is described in §4.1; the planner does not model it.
- `waypoints` are the swept collision-check samples, reused for display and
  for independently re-verifying the plan's safety.
- **Experimental bounded scope**: `POST /plan/v2` accepts 1-8 obstacles.
  Bounded validation passed the tested feasible 6-8 obstacle layouts plus
  containment, hard-stop and soak checks. The original 4-8 adoption gate
  still fails because `A_spread_04` returned no route within budget. This is
  evidence for the tested layouts, not a claim that every Task 1 map succeeds.

## 7.1 RAW camera images, RPi to PC [PC side IMPLEMENTED; RPi sender is TARGET v2]

After each `SNAP` the RPi may send the photo it took to the PC, which shows it
live on the web page. This is **display only**: the PC does no recognition,
reads no character, draws no bounding box and chooses no "best" image. The RPi
sends the JPEG it wants shown (for the NTU RAW-image evidence, the full camera
view; the RPi may draw the recognised character and box into it). The RPi may
also report the image ID it recognised as `target_id`; the PC shows it as given
and never checks it against the picture. The ID that counts for NTU is the one
the RPi sends to Android (`TARGET,<obstacle>,<id>`), not this display copy.
Nothing here is an STM command and nothing here changes a plan.

Upload:

```
POST /images?mission_id=<id>&obstacle_id=<n>[&target_id=<id>]
Content-Type: image/jpeg
Content-Length: <bytes>        (required)

<the JPEG bytes>
```

- `mission_id`: the same non-empty string the RPi used in the `/plan/v2`
  request for this run (§6 item 1). At most 128 characters, no control
  characters, percent-encoded in the URL.
- `obstacle_id`: an integer 1-8 (decimal digits only): the obstacle just
  snapped, as in the `SNAP` action's `obstacle_id` (§7).
- `target_id` (optional): the image ID the RPi recognised, an integer **11-40**
  (the NTU image-ID range), two decimal digits, no sign, space or leading zero,
  given at most once. Absent means "no ID reported" (recognition has not run or
  found nothing). Present but invalid, the whole upload is rejected with 400
  and nothing is stored.
- **Image requirement** (the RPi's side): JPEG, **1280 x 720**, quality 85,
  aim below 1 MB. The PC enforces **format and size only**: `Content-Type` is
  exactly `image/jpeg`, the body starts with `FF D8 FF` and ends with `FF D9`,
  and it is at most **2 097 152 bytes** (a 2 MB hard limit above the 1 MB aim).
  The resolution is **not** enforced, and the PC does not decode the image.

Replies (JSON, `Cache-Control: no-store`):

| Status | Body | Meaning |
|---|---|---|
| 200 | `{"type":"IMAGE_STORED","image_id","mission_id","obstacle_id","target_id","bytes","replaced"}` | Stored. `target_id` is the reported ID or `null`. `replaced` is true when it replaced an earlier photo of the same obstacle |
| 400 | `{"type":"IMAGE_ERROR","error"}` | Bad or missing `mission_id`/`obstacle_id`, bad or repeated `target_id`, missing or non-numeric `Content-Length`, empty body, or not a complete JPEG |
| 413 | `IMAGE_ERROR` | Larger than the 2 MB limit (rejected from `Content-Length`, before the body is read) |
| 415 | `IMAGE_ERROR` | `Content-Type` is not `image/jpeg` |

Storage rules (all in PC memory; **nothing is written to disk**, and a server
restart clears everything):

- At most **one photo per obstacle**: a newer photo of the same obstacle
  replaces the older one **completely, including its `target_id`**. If
  recognition finishes after the first upload, the RPi uploads the photo again
  with `target_id`; an upload without one clears an earlier ID.
- A photo whose `mission_id` differs from the stored set **starts a new set**
  and clears the old photos. So there are never more than 8 photos (at most
  16 MB) held.
- The PC chooses `image_id` (`<obstacle_id>-<n>`, `n` increasing); a client never
  names a file.

Reading (used by the web page):

- `GET /images` returns `{"type":"IMAGE_LIST","mission_id","revision","images":[{"image_id","obstacle_id","target_id","bytes","received_at_s"}]}`.
  `mission_id` is `null` and `images` is empty when nothing was uploaded.
  `revision` increases on every stored photo. `received_at_s` is the PC's wall
  clock in seconds and is informational only.
- `GET /images/<image_id>` returns the JPEG (`image/jpeg`, `X-Content-Type-Options: nosniff`),
  or 404 for an unknown id.
- The page shows only photos whose `mission_id` equals the mission id in
  `/state/v2`, so an earlier run's photos disappear when a new plan starts. A
  tile reads "Obstacle 3" or, with a `target_id`, "Obstacle 3 · ID 22".

**Planner-version indicator (PC web page only; not an RPi interface).** The PC runs the old
protocol v1 planner at `POST /plan` and protocol v2 at `POST /plan/v2`. **The URL the caller posts
to chooses the planner, never the `protocol_version` field in the body**: a v2 body posted to `/plan`
is a v1 request and is rejected, and a v1 body posted to `/plan/v2` is a v2 request and is rejected.
`GET /state/versions` returns `{"type":"PLAN_VERSIONS","last_request":null}` before any plan request,
else `{"type":"PLAN_VERSIONS","last_request":{"protocol_version":1|2,"mission_id":<string|null>,
"mode":<string|null>,"source":"rpi"|"browser","age_s":<seconds since it arrived>}}`. The server records the newest `/plan`
(v1) or `/plan/v2` (v2) request whose body it could read; `mission_id` is v2 only (null for v1, capped
at 128 characters); `mode` is the request's `mode` string as sent (null when absent or not a string,
capped at 16 characters), which the page compares with the team default B3; `source` is `"browser"` when the User-Agent contains `Mozilla`, else `"rpi"` (any
non-browser client, such as the RPi script or curl). The page at `/` lights a V1 or V2 circle for that
request and always lights V2 for what it draws (it only reads `/state/v2`), with a warning when the
newest request was V1 or was a different v2 mission from the one on screen. It is display only: nothing
plans or executes from it.

Not implemented: recognition itself (the PC only displays a reported `target_id`),
the box as data (the RPi draws it into the JPEG), picking one
representative image per character, saving to disk, and any Android gallery
(§8 is unresolved). There is no authentication, like the rest of this service.

## 8. Android coordinate convention [UNRESOLVED — legacy incompatibility]

The only Android implementation that exists is on the unmerged
`origin/android` branch, and it uses a **different, incompatible**
convention from §1 item 6:

- Its `start`/`ROBOT` coordinate is the **bottom-left cell of the robot's
  3×3 grid footprint**, not the rear-axle midpoint.
- Its assumed footprint is 3×3 cells (30×30 cm) with a 3-cell turn radius,
  both marked unconfirmed in its own docs.

**This document does not define a conversion formula between that
coordinate and the canonical rear-reference pose.** Inventing one here
would silently encode an assumption about where the 3×3 box sits relative
to the physical robot, and that has not been measured. Under protocol v2:

- **Android and every new subsystem must send the rear-reference
  coordinate directly**, in the units of §1 (cm, or the existing grid
  convention with an explicit statement of which point the grid cell
  identifies).
- The current Android branch's bottom-left-cell convention is **legacy and
  incompatible** with this protocol until it is adapted — either by
  changing Android to emit the rear reference, or by defining and
  validating a conversion once the robot's footprint placement inside
  Android's 3×3 box is actually measured. **That decision is deferred**
  (§13, decision 6).

**Task 1 fixed start (team convention, user-stated 2026-10-07; not measured).**
Android always sends the start `(2, 1)` facing N, and the RPi passes it to the
planner unchanged as the rear-axle midpoint, i.e. `(20, 10)` cm facing North (§6.1).
No bottom-left-cell conversion is applied. This holds only if the robot is
physically placed that way every run: rear edge 10 cm from the bottom wall,
centre line 20 cm from the left wall, facing North (the rear extent is 0 cm,
§2). A different placement shifts every move by the same amount. The `ROBOT`
updates the RPi sends assume Android's 3×3 box is centred on the robot body
(documented in the RPi program, not measured); they are display only.

## 9. Web visualisation: current drawing vs. backend geometry [TARGET v2]

**[IMPLEMENTED], client-side today:**
[`algo/web/arena.js`](algo/web/arena.js), shared by both web pages, already
draws, at one consistent cm→px scale:

- the 200×200 cm arena grid and 10×10 cm obstacles, including each
  obstacle's correct image-facing edge;
- the robot's true asymmetric **24×19 cm** footprint (not a centred
  square), positioned at the rear-reference pose;
- an explicit rear-reference marker;
- an explicit camera-lens marker, placed using the 15.25 cm forward /
  0 cm lateral offset (§2).

All of this comes from a **duplicated copy** of the robot/camera numbers,
hand-kept in sync with [`algo/config.py`](algo/config.py) as JS literals
(`ROBOT_FRONT_EXTENT_CM`, `CAMERA_FORWARD_OFFSET_CM`, etc.) — see
`CODEBASE_INDEX.md` §18. This is a maintenance risk, not a missing
capability: the drawing itself is correct today; the risk is that two
files must be hand-edited every time a measurement changes.

**[TARGET v2], not implemented:**

- **Serving geometry from the Python backend** as the single source of
  truth, so the page stops maintaining its own copy:

```jsonc
{
  "arena_cm": 200,
  "obstacle_cm": 10,
  "robot": {
    "front_extent_cm": 24.0,
    "rear_extent_cm": 0.0,
    "width_cm": 19.0,
    "status": "provisional"
  },
  "safety_margin_cm": { "value": 3.0, "status": "provisional" },
  "camera": {
    "forward_offset_cm": 15.25,
    "lateral_offset_cm": 0.0,
    "standoff_cm": 25.0,
    "status": "provisional"
  },
  "turn_radii_cm": {
    "FL": 24.8, "FR": 33.7, "BL": 33.6, "BR": 24.5,
    "status": "validated"
  }
}
```

  Each group carries a `status` of `"validated"` or `"provisional"`
  (§2/§3), so the page can render a visible distinction rather than
  presenting every number with equal confidence.
- **Drawing the 3 cm safety envelope** around the raw footprint — not
  drawn today.
- **Drawing planned paths using the four measured per-direction radii**
  (§3) — today's displayed paths come from the planner's old
  symmetric-Dubins result and don't reflect §3 at all.
- **Optionally, a provisional camera-to-face line** at each viewing pose,
  showing the (currently unvalidated) 25 cm standoff.

This section does not claim the backend geometry-object API exists; only
the client-side drawing listed under [IMPLEMENTED] above is real today.

## 10. Terminal response handling (consolidated)

Restates §5 as a flat table for quick reference during RPi implementation.

| Response | Terminal? | Action |
|---|---|---|
| `ACK` | Yes | Advance |
| `ERR,BAD_CMD` | Yes | Abort mission |
| `ERR,BUSY` | Yes | Pause, investigate wrapper bug |
| `ERR,PID_NOT_SET` | Yes | Abort mission |
| `ERR,IMU_NOT_READY` | Yes | Retry once IMU ready, only if no `MM,EV:START` seen; otherwise pause, pose unknown |
| `ERR,MOTION_TIMEOUT` | Yes | Pause, pose unknown |
| RPi-side timeout (no reply) | N/A — ambiguous | Pause, pose unknown, never auto-resend |
| `ACK,STOPPED` | Yes (stop only) | Not a success. Abort, pose UNCERTAIN (§4.2.1, §6.2) |
| `ERR,IR_NO_OBSTACLE` / `ERR,IR_NO_EDGE` / `ERR,IR_SENSOR` | Yes | Wrapper outcome; pose UNCERTAIN (known if `IR_SENSOR` before START); no retry (§4.6) |
| `ERR,US_NO_TARGET` / `ERR,US_TOO_CLOSE` / `ERR,US_STALE` / `ERR,US_INVALID` / `ERR,US_FAULT` | Yes | Wrapper outcome; pose per §4.9; no retry |
| `IREDGE,`, `USFUS,` (decision), `USFUS,STOP,`, `USPARK,STOP,`/`USPARK,FAULT,` (cause) | No | Record; part of the completion record or cause evidence (§4.6, §4.9) |
| `IR,...`, `USD,...`, `IMU,...` | No (query reply) | Reply to its own query only (§4.5, §4.8, §4.1.1) |
| `PROFILE,OK` (after a `PROFILE,PRECISION`/`PROFILE,STANDARD` setup command) | Yes, setup command only | Send `stm_setup.verify_command` (§6 item 8) |
| `PROFILE,PRECISION` / `PROFILE,STANDARD` (after `PROFILE,GET`) | Yes, setup command only | Proceed only if it equals `expected_verify_reply`; otherwise abort |
| `PROFILE,CUSTOM` (after `PROFILE,GET`) | Yes, setup command only | Abort mission (§6 item 8) |
| Any `MM,`/`AK,`/`TB,`/`MD,`/`MS,`/`HS,`/`RTG,`/`MP,`/`SPC,`/`STP,`/`SFF,`/`TL,`/`TR,`/`TS,` line, `USPARK,STOP`/`USPARK,FAULT` cause lines, and bench-build `US,`/`USPARK,TRACE,`/`IRRAW,` lines | No | Log or discard; never treated as a reply |

For the §4.1 setup commands, any `ERR,...` or a timeout aborts mission execution
(§6 item 8) instead of the actions listed above.

## 11. What is and is not implemented today

| Layer | Status |
|---|---|
| STM firmware, motion behaviour and UART protocol, §3–§5 | **[IMPLEMENTED]**, physically validated foundation |
| PC planner HTTP API, protocol_version 1 | **[IMPLEMENTED]**, symmetric 25 cm turn model (`CODEBASE_INDEX.md` §7) |
| Rear-reference footprint and camera viewing-pose geometry, §2 | **[IMPLEMENTED]**, used by planner collision/viewing-target code today |
| Web true-scale robot/camera drawing, §9 | **[IMPLEMENTED]**, client-side, from duplicated JS constants |
| PC planner result schema v2 (§7) | **[PARTIALLY IMPLEMENTED]** — bounded 1-8 planning via `POST /plan/v2`; tested 6-8 layouts passed, but the original 4-8 adoption gate fails on one four-obstacle fixture. Includes `motion_profile`/`stm_setup`; RPi execution is [TARGET v2] |
| RPi wrapper: sequencing, IDs, disconnect handling (§6) | **[IMPLEMENTED OUTSIDE THIS REPO]** — the team's `rpi_task1/` program does it (§6.1); unit-tested only, not recorded as run on the robot |
| Android <-> RPi plan request and `READY`/`START` handshake (§6.1) | **[IMPLEMENTED OUTSIDE THIS REPO]** — Pi side verified from source and unit tests; Android branches parse `READY`; not run on the tablet or robot |
| Four-radius executable planner | **[IMPLEMENTED]**, standalone (`algo/executable_route_optimizer.py`); legacy `POST /plan` still uses the old symmetric 25 cm model |
| RAW camera image upload and live gallery, §7.1 | PC side **[IMPLEMENTED]** (`POST /images`, `GET /images`); the RPi sender is **[TARGET v2]**; no recognition (an optional RPi-reported `target_id`, 11-40, is shown as given) |
| Backend-served web geometry object, §9 | **[TARGET v2]** — page still hard-codes its own copy |
| Task 2 `STOP` / `ACK,STOPPED` (§4.2) | Spelling **[FROZEN]**; STM **[MERGED]** (in this branch's firmware source; host-tested on the source branch; not flashed; bench pending), operated on the robot **[USER-REPORTED]**; Pi **[TARGET]** |
| Task 2 `IR,GET` (§4.5) | Spelling **[FROZEN]**; STM **[MERGED]** (`APP_IR_ENABLE` default 1; host-tested on the source branch; not flashed); 100–200 mm **[USER-REPORTED]**, 100–400 mm not independently validated, blind below 100 mm, telemetry only, not an emergency-stop sensor; Pi **[IMPLEMENTED]** (host-tested, Phase 3b.5a) |
| Task 2 `MOV,FIRL`/`MOV,FIRR` (§4.6) | Spelling **[FROZEN]**; STM **[MERGED]**; LEFT/RIGHT success, STOP, no-obstacle, no-edge and floor cases on the source-branch binary **[USER-REPORTED]**; integrated branch not flashed; Pi **[IMPLEMENTED]** (host/simulator; success needs `DL`/`DR`, §4.7); **hardware [BLOCKED]** (disconnection/floating-input blocker, D-57) |
| STM ultrasonic driver and opt-in `USPARK` (§4.3, §4.11) | Legacy source **[MERGED]**. Corrected predictive authority is on separate `a26ec79`; local combined candidate `a94832e` is not merged here. Trigger-190 `MOV,F,100` and ≈20 cm clearance are **[USER-REPORTED]**. Pi dedicated final-parking support **[TARGET]**; exact 100 cm-stage acceptance pending |
| Measured travel `DL`/`DR` (§4.7) | Spelling **[FROZEN]**; **[TARGET]** on both sides (no STM branch has it) |
| `USD,GET` (§4.8) | Spelling **[FROZEN]**; **[TARGET]** on both sides |
| `MOV,FUS` (§4.9) | Frozen historical proposal; unimplemented and superseded by §4.3 for final parking |
| Task 2 RPi link, START/READY, recovery (§6.2) | **[TARGET]** (Phase 3b); the Task 2 mission gate stays **closed** until Phases 4–7 and Phase 8 certification |
| Android rear-reference coordinates (§8) | **[UNRESOLVED]** — current branch uses an incompatible convention |

## 12. Numeric contract summary

This describes what the executable planner (`motion_primitives.py`) emits
into a v2 `actions` list — not the STM wire parser's own acceptance range
(§4), which separately accepts 0. `motion_primitives.py` enforces
`1 <= value` and never emits a zero-valued command; a zero-length movement
is planning-useless, never STM-invalid.

- Straight commands: **integer** centimetres, 1–999.
- Turn commands: **integer** degrees, 1–360, one of `FL`/`FR`/`BL`/`BR`.
- Every emitted `stm_command` string is derived from the same integer used
  to compute `start_pose`/`end_pose` — never from a pre-rounding continuous
  value, so replaying the commands reproduces the poses exactly.
- No half-steer, fractional-degree, or non-STM command (`FW`/`BW`/`TURN,L`/
  `TURN,R`/etc.) may appear in an `actions` list under this protocol.

## 13. Unresolved decisions (do not assume answers)

1. **Small-turn radius behaviour.** Whether the measured radii (§3) hold
   for turns well under 90° (pre-steer and brake roll-out may matter more
   at small angles).
2. **Minimum reliable straight distance.** Below what `MOV,F,<n>` the
   STM's accuracy is unknown.
3. **Timing model.** No calibrated timing model exists for either schema:
   v1's `estimated_time_s` is a legacy, uncalibrated estimate, and v2 (§7)
   deliberately carries no timing field at all (its `total_cost` is planning
   cost, never time). A future timing model, calibrated against real
   `MM,EV:END,...,T:<ms>` logs, remains unresolved and undesigned.
4. **Camera standoff, pitch and lateral offset validation.** All currently
   provisional (§2).
5. **STM↔RPi physical link.** How USART3 reaches the RPi's USB port and
   what device node it becomes (`CODEBASE_INDEX.md` §6, §18 #2).
6. **Android coordinate conversion, or Android protocol change (§8).**
   Not decided, and not assumed, in this document.
7. **RPi-side timeout duration** for the ambiguous no-reply case in §5/§10.
8. **Whether `IMU,ZERO`/`IMU,GET` are used between missions** for drift
   correction, and if so, when.
9. **Obstacle-order search practicality for 4-8 obstacles.** The old
   permutation endpoint was replaced by the contained bounded prefix-tree
   search for protocol v2. Its validation returned safe complete routes for
   every feasible tested 6-8 obstacle layout inside the response ceiling,
   but `A_spread_04` found no route within the planning budget (and
   `D_seeded_04_s1` also returned none). Therefore
   all-layout practicality remains unresolved; the endpoint never runs beyond
   its deadline, and since 2026-10-07 sends the best verified partial route
   (`complete:false`) when no complete one is found in time.

10. **PRECISION vs STANDARD motion profile (§4.1).** No calibrated timing
    model exists for either profile (item 3), so no speed-up or mission-time
    claim is made for STANDARD. Its straight-motion accuracy is
    user-reported as acceptable only, with unrecorded test conditions, and is
    not in the STM validation record. Item 2's minimum reliable straight
    distance has not been re-validated per profile.

11. **FIR hardware authority (§4.6).** Blocked until a repeatable, documented fault-detection mechanism prevents a
    disconnection or floating input from producing an accepted CLEAR, or a separately reviewed safety argument and
    mitigation is approved by the team (D-57). Not accepted, not assumed.
12. **Residual travel after the brake window (§4.7).** The bound, and whether the conditional encoder-settle stage
    (3b.4c) is needed, is decided at the hardware level that measures it (gate G-SETTLE). The settle-timeout spelling
    is not chosen.
13. **STOP timing values (§6.2.3).** Reply timeout 0.5 s, 3 attempts and 0.3 s drain are provisional until calibrated on
    the robot.
14. **Predictive USPARK integration (§4.3).** Corrected source is pinned at `a26ec79`, but the Task 2 branch does not
    contain it; local combined candidate `a94832e` is not the mission branch and is not recorded as flashed. The final
    trigger-190 result is user-reported, and the exact 100 cm-stage three-run acceptance, Pi dedicated transaction and
    final integrated build record remain open. The former `MOV,FUS`/PD-16 route is superseded for final parking.
15. **Ultrasonic mounting offset (240 mm), free space behind O2, independent protection and the stopping reserve,
    Task 1 impact of the integrated firmware.** Open (BD-12, BD-18, BD-19, PD-18); the O2 cross-span stays unknown
    (D-02).
16. **IR range.** The 100–200 mm trusted range is user-reported; the 100–400 mm range is not independently validated.

Basic FL/FR/BL/BR travel, heading-change direction and steering-endpoint
sharing are **not** on this list — they are physically validated (§3), not
open questions.

## 14. Change control

This file is versioned like any other protocol artifact:

- Any change to §1 (coordinates), §3 (turn semantics) or §4 (STM wire
  format) is a breaking change: update this document, then update every
  consumer (planner, RPi, Android) before merging any of them.
- §4.2–§4.9 and §6.2 were frozen on 2026-10-13 (decisions D-56 to D-72). Changing a frozen spelling, grammar or
  terminal rule there is a breaking change like any §4 edit; the status tags change only when evidence exists.
- `CODEBASE_INDEX.md` §11 links here for the wire-format section and must
  not duplicate the tables above — only summarise and point here.
- Resolve an item in §13 by editing that item in place (don't delete the
  history of what was unresolved) and updating any section whose content
  depended on the assumption.
