"""Task 2 constants.

Units: cm and degrees. Poses are the rear-axle midpoint (PROTOCOL.md §1),
heading counter-clockwise from +x (east). The mission frame has its origin at
the robot's start pose, +y straight ahead (towards the obstacles), so the
start heading is 90.

Values marked CALIBRATE are first guesses that must be measured on the robot
before a timed run. Values marked FROM PROTOCOL come from PROTOCOL.md and are
team-measured. Values marked PROVISIONAL are not frozen anywhere.

Unresolved safety parameters (the O2 span bounds, BD-02; the free space behind
O2 and the side-wall southward extent, BD-18; the IR thresholds, BD-03; the
initial pose uncertainty; the stopping reserve) deliberately have NO value in
this file. They are required inputs of `course.FreeSpace`, `course.O2Bounds`,
`ir.IrThresholds` and the simulator's `sim.SimParams`.
"""

from __future__ import annotations

# --- Robot (FROM PROTOCOL, §2/§3) ----------------------------------------
FRONT_EXTENT_CM = 24.0
REAR_EXTENT_CM = 0.0
WIDTH_CM = 19.0
MARGIN_CM = 3.0                 # applied once, to the robot footprint
CAMERA_FORWARD_CM = 15.25
TURN_RADIUS_CM = {"FL": 24.8, "FR": 33.7, "BL": 33.6, "BR": 24.5}
COUNTS_PER_CM = 75.0            # firmware MOVCAL slope; logs, and FIR DL/DR/CC counts -> cm (approximate)

# --- Ultrasonic mounting (FROM PROTOCOL §2, team-measured 2026-10-13, BD-12) ---
ULTRASONIC_FORWARD_CM = 19.5    # rear axle -> transducer faces

# --- Course (task rules) ---------------------------------------------------
O1_SIZE_CM = 10.0               # 10 x 10
O2_DEPTH_CM = 10.0              # obstacle 2 is 10 cm deep; its cross-span is UNKNOWN and never classified
O2_END_CLEARANCE_CM = 50.0      # guaranteed free space outward from each physical end of O2 (D-27)
O2_AXIS_TOLERANCE_CM = 5.0      # D-75: a measured O2 end may be this much nearer the camera's arrow than the minimum half span
CARPARK_WIDTH_CM = 60.0         # inside, across the robot
CARPARK_DEPTH_CM = 50.0         # inside, along the robot
START_REAR_FROM_BACK_WALL_CM = 5.0   # the team places the robot's back this far from the carpark's back wall (2026-10-15).
                                     # CALIBRATE: the run logs the Pi's tracked wall distance next to the ultrasonic one at the park
START_X_OFFSET_CM = 0.0         # robot centred in the carpark at the start

# Class names of the Task 2 model (task2_ncnn_model: "bullseye", "left", "right").
ARROW_IDS = {"right": "RIGHT", "left": "LEFT"}
BULLSEYE_LABEL = "bullseye"

# --- Camera ----------------------------------------------------------------
HFOV_DEG = 62.2                 # Pi Camera v2; CALIBRATE
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# depth (cm, along the camera axis) = K / box height (px). K = f_px * symbol
# height. Default assumes ~8 cm symbols; CALIBRATE (the calibration tool went with the old task2.py).
RANGE_K = {"arrow": 4250.0, "marker": 4250.0}
MIN_CONFIDENCE = 0.5

# Arrow decision: one inference on the frame taken at the reading position;
# its largest confident arrow decides. No arrow -> FAIL_SAFE (UNKNOWN never
# becomes a default).
VOTE_MIN = 1

# --- Approach (camera ranging; ultrasonic once the STM has it) --------------
O1_READ_DEPTH_CM = 45.0         # at least this rear axle -> O1 face when reading (more if the bypass needs it)
O2_READ_DEPTH_CAM_CM = 45.0     # camera -> O2 face when reading from the lane (keeps the arrow in frame)
MAX_READ_NORM_X = 0.75          # ... and further back if the arrow would sit beyond this in the frame
APPROACH_TOLERANCE_CM = 3.0     # closer than this to the target depth: stop ranging
APPROACH_REFINE_CM = 40.0       # further than this: stop short and range again
APPROACH_REFINE_SHORT_CM = 20.0
BLIND_STEP_CM = 20              # nothing seen: creep forward this much ...
BLIND_STEP_MAX_CM = 40          # ... at most this much in total
APPROACH_MAX_LOOKS = 6

# --- Route -----------------------------------------------------------------
O1_LANE_CM = 19.5               # lateral rear-axle offset from O1's centre while passing it
O2_ENTRY_EXTRA_CM = 2.0         # added to the swept-path minimum standoff before the 90 turn
EXIT_RUNOUT_CM = 30.0           # an exit template must be able to drive on this far past O2's face (not a span)
EDGE_CLEARANCE_CM = 4.0         # rear axle this far past an O2 edge before turning round it
EDGE_ACQUIRE_INSET_CM = 10.0    # D-75: before the return FIR leg, side IR this far inside the measured O2 end
EDGE_ACQUIRE_MIN_CM = 3.0       # a sensor already at least this far inside the end and the arrow needs no move
ROBUST_EXTRA_CM = 2.0           # templates must stay clear with this much extra margin
FINAL_STRAIGHT_WANTED_CM = 50.0 # handout: >= 50 cm straight approach into the carpark
FINAL_STRAIGHT_MIN_CM = 15.0    # below this the return route is rejected
PARK_ALIGN_TOLERANCE_CM = 3.0   # camera bullseye offset that is left alone
PARK_ALIGN_MIN_ROOM_CM = 50.0   # only align with the camera with this much straight left
PARK_RECHECK_CM = 20.0          # without ultrasonic: stop this far short, range the bullseye again
CAMERA_RANGE_ERROR = 0.05       # CALIBRATE: camera depth error, fraction of the depth (1 sigma)
ODOMETRY_ERROR_CM = 5.0         # CALIBRATE: dead-reckoning error at the carpark (1 sigma)

# --- Parking (FROM handout §6.7) --------------------------------------------
PARK_ULTRASONIC_CM = 12.0
PARK_ULTRASONIC_RANGE_CM = (5.0, 20.0)
PARK_REAR_FROM_WALL_CM = FRONT_EXTENT_CM + PARK_ULTRASONIC_CM   # 36 cm

# --- Final parking with the STM's predictive ultrasonic stop (TASK2_USPARK_ALGO_INTEGRATION.md) -------
# Selected behaviour, physically tested by the team on the authority build (trigger 190); NOT the 12 cm
# MOV,FUS design above, whose constants only serve the camera fallback when USPARK is unavailable.
PARK_STAGE_SENSOR_TO_WALL_CM = 100.0   # staging: ultrasonic face -> parking wall (USPARK off)
PARK_FINAL_COMMAND = "MOV,F,100"       # the ONLY final command; never MOV,F,110 / 180
PARK_PROFILE = "TASK2TEST"
PARK_EXPECTED_REASON = "PRED"
PARK_EXPECTED_PATH = "NORM"
PARK_EXPECTED_TRIGGER_MM = 190
PARK_USPARK_TH_MM, PARK_USPARK_RA_MM, PARK_USPARK_AR_MM = 300, 320, 500   # USPARK,GET legacy settings
PARK_VALIDATED_CLEARANCE_CM = 20.0     # physical evidence (team-reported), not a live reading
PARK_HARD_MIN_CLEARANCE_CM = 15.0      # acceptance limit; the final move is sweep-checked to here

# --- Heading ---------------------------------------------------------------
HEADING_FIX = True              # correct each turn with IMU,GET (as Task 1)
HEADING_HOLD_DEG = 1.5          # before a long straight, fix errors at least this big

# --- Side IR (PLACEHOLDER until the STM protocol is frozen) -----------------
# Mounting values below are team-measured but PROVISIONAL (PROTOCOL.md §2).
# Sensor mounting, team-measured (PROTOCOL.md §2): from the rear axle, +left.
# Both point exactly sideways, ~8.4 cm above the floor. The sensor faces sit
# inside the body: the wheels stick out IR_INSET_CM beyond them, so a reading
# is the robot's side gap to the wall plus IR_INSET_CM.
IR_FORWARD_CM = {"L": 19.0, "R": 19.0}
IR_LATERAL_CM = {"L": 5.25, "R": -5.25}
IR_HEIGHT_CM = 8.4              # documentation only: the beam must hit O2, not pass over it
IR_INSET_CM = WIDTH_CM / 2 - abs(IR_LATERAL_CM["L"])   # 4.25, the measured wheel protrusion
# PROVISIONAL, uncalibrated (BD-03). Consumers take `ir.IrThresholds` as an explicit
# parameter; these two numbers feed only `IrThresholds.provisional()`, for simulation.
IR_PRESENT_CM = 25.0            # valid reading <= this: WALL_PRESENT (sensor face to wall)
IR_ABSENT_CM = 40.0             # valid reading >= this: WALL_GONE (must be > present)
# IR,GET (PROTOCOL.md §4.5): distances are integer mm from the sensor face.
IR_GET_MAX_AGE_MS = 150         # PROVISIONAL: a reply whose AGE is larger is STALE
IR_GET_MIN_INTERVAL_S = 0.1     # at most 10 Hz, one request outstanding, idle only
IR_GET_TIMEOUT_S = 1.0          # PROVISIONAL: the STM answers in ~10-20 ms when idle; a lost reply desynchronises
IR_EDGE_LAG_CM = 3.0            # travel between O2's real end and the STM confirming it (IREDGE CC). Robot runs 2026-10-14:
                                # reverse-then-FIR on the front face put it at 1.5-6.5 cm (21:36) and 2.9-7.9 cm (22:09);
                                # 3 cm is the least that fits both (more would let the Pi think O2 shorter than it is)
# The STM's FIR detector (PROTOCOL.md §4.11.6, handoff §5.2): VISIBLE <= 200 mm, CLEAR >= 250 mm from the
# sensor face. A FIR leg runs with the side IR at FIR_TRACK_READING_CM (handoff §8: "preferably near 145 mm").
STM_IR_VISIBLE_CM = 20.0
FIR_TRACK_READING_CM = 14.5
FIR_TRACK_TOLERANCE_CM = 2.0    # an IR reading this far off the tracking distance is corrected first
FIR_REANCHOR_MAX_CM = 15.0
IR_FAR_CM = 40.0                # the STM's IR,GET says FAR beyond 400 mm (§4.5): "no obstacle" (D-80)      # the IR may move O2's face (from the camera's estimate) at most this far

# --- STM capabilities --------------------------------------------------------
# Flip these to True once the STM implements the command and the matching
# placeholder in stm_task2.py is filled in.
# IR,GET is implemented: see stm_task2.IR_GET_CAPABILITY (read-only, static queries).
# Edge-follow (MOV,FIRL/FIRR) is implemented: see stm_fir.FIR_CAPABILITY (no hardware authority).
STM_HAS_ULTRASONIC = False      # front distance reading
STM_HAS_ULTRASONIC_STOP = False # drive forward, stop at a front distance
STM_HAS_SENSOR_HEALTH = False   # one-shot health report

# --- Front ultrasonic from the `US,` bench stream (stm_us.py; PROVISIONAL, not bench-validated) ----------
US_MAX_AGE_MS = 150             # a line whose AGE is larger is not trusted
US_AGREE_CM = 4.0               # trusted values within this of their median agree
US_MIN_AGREEING = 3             # agreeing values needed for one reading
US_SAMPLE_LINES = 5             # US, lines per reading (~1 s at the firmware's 5 Hz)
US_SAMPLE_TIMEOUT_S = 2.0
# USD,GET (PROTOCOL.md §4.8), used instead of listening when the firmware answers it (Task2Stm.probe_usd_get)
USD_GET_MIN_INTERVAL_S = 0.03   # between queries: the sensor publishes about every 61 ms, a repeated S is skipped
USD_GET_TIMEOUT_S = 1.0         # PROVISIONAL, as IR_GET_TIMEOUT_S: a lost reply desynchronises
US_MAX_TARGET_LATERAL_CM = 10.0 # use the ultrasonic on a target only if the camera puts it this close to the axis
US_GATE_MIN_CM = 20.0           # ultrasonic accepted if within max(this, US_GATE_FRACTION x camera depth) ...
US_GATE_FRACTION = 0.35         # ... of the camera's estimate (RANGE_K is uncalibrated)
CAMERA_SCALE_LIMITS = (0.6, 1.6) # the camera-depth correction learnt at O1 is clamped to this
US_CONSISTENCY_CM = 4.0       # a wall reading must drop by the distance driven, within this
PARK_STAGE_TOLERANCE_CM = 3.0   # staged further than this beyond PARK_STAGE_SENSOR_TO_WALL_CM: drive the rest
PARK_STAGE_MAX_FIX_CM = 30.0    # ... at most this far
