from dataclasses import dataclass

ARENA_CM = 200
GRID_CELL_CM = 10
OBSTACLE_CM = 10

# Robot footprint, physically measured by the team. The planner pose
# (x, y, heading) is the rear-axle midpoint / rear reference point, not the
# centre of the robot. Local frame: +forward along the heading, +left
# counter-clockwise from it. These are raw physical extents; the safety
# margin is added separately, exactly once, in footprint.py.
ROBOT_FRONT_EXTENT_CM = 24.0
ROBOT_REAR_EXTENT_CM = 0.0
ROBOT_WIDTH_CM = 19.0
SAFETY_MARGIN_CM = 3.0  # raised from 1.0: the robot hit obstacles along the way

# Camera lens position relative to the rear reference point, physically
# measured by the team (forward measured at about 15-15.5 cm; provisional).
CAMERA_FORWARD_OFFSET_CM = 15.25
CAMERA_LATERAL_OFFSET_CM = 0.0  # +left; 0 = centred

# Lens-to-obstacle-face distance at a viewing pose. Provisional default only:
# not physically validated, to be calibrated against the camera/model.
CAMERA_STANDOFF_CM = 25.0  # raised from 20.0: stop further from the obstacle face

# Geometry the committed benchmark fixtures, and the evidence recorded for
# them, were produced with (benchmarks/fixtures.py). Historical only: the
# planner and the HTTP service never read these.
BENCHMARK_FIXTURE_SAFETY_MARGIN_CM = 1.0
BENCHMARK_FIXTURE_CAMERA_STANDOFF_CM = 20.0
BENCHMARK_FIXTURE_GOAL_POSITION_TOLERANCE_CM = 5.0

# Legacy symmetric turning radius used by the current Dubins/lattice planner
# (planner_interface.py). Not physically representative of the STM chassis;
# kept only because that legacy code still depends on it. New STM-executable
# code must use the four independent per-direction radii below instead.
MIN_TURNING_RADIUS_CM = 25

# Per-direction turn radii, physically validated on the STM chassis
# (PROTOCOL.md §3). Each is independent: never mirror or derive one from
# another. Used by motion_primitives.py, not by the legacy planner above.
TURN_RADIUS_FL_CM = 24.8
TURN_RADIUS_FR_CM = 33.7
TURN_RADIUS_BL_CM = 33.6
TURN_RADIUS_BR_CM = 24.5

FORWARD_SPEED_CM_S = 20
REVERSE_SPEED_CM_S = 14
TURNING_SPEED_CM_S = 12
REVERSE_PENALTY = 1.08
TURN_RADIUS_CM = MIN_TURNING_RADIUS_CM

EDITOR_SNAP_CM = 5
START_ZONE_CM = 40
COLLISION_CHECK_STEP_CM = 1

# Sampling-resolution bound for the new swept-primitive collision checker
# (swept_collision.py) only. This is a NUMERICAL sampling bound, not a
# second physical safety margin: it is the maximum distance, in
# centimetres, that any point on the already margin-inflated robot
# footprint may move between two adjacent sampled poses of one primitive.
# Independent of COLLISION_CHECK_STEP_CM above, which the legacy
# Dubins/lattice planner (planner_interface.py) keeps using unchanged.
SWEEP_MAX_POINT_STEP_CM = 1.0

# Search defaults for the new STM-executable state-lattice search
# (state_lattice.py) only. These are PLANNER/SEARCH tuning defaults, not
# physical measurements — unlike the turn radii above, there is nothing to
# "get wrong" about them physically; they trade search speed against path
# quality/resolution and may need retuning once the search runs against
# real maps. Independent of every legacy/collision setting above.
LATTICE_STRAIGHT_STEP_CM = 10       # integer cm per F/B successor
LATTICE_TURN_STEP_DEG = 10          # integer degrees per FL/FR/BL/BR successor
LATTICE_POSITION_BIN_CM = 5.0       # search-key position quantisation only
LATTICE_HEADING_BIN_DEG = 10        # search-key heading quantisation; must divide 360 exactly
LATTICE_GOAL_POSITION_TOLERANCE_CM = 3.0   # was 5.0 (3.0 from 2026-10-06, 5.0 while testing, 3.0 again from 2026-10-07); PROTOCOL.md section 7
LATTICE_GOAL_HEADING_TOLERANCE_DEG = 5.0
LATTICE_MAX_EXPANSIONS = 20000      # bounded search; returns a diagnostic failure, never hangs

SCALE = 3
MARGIN_PX = 50
PANEL_WIDTH_PX = 500
ANIMATION_STEP_CM = 1.5


@dataclass(frozen=True)
class PlannerConfig:
    arena_cm: float = ARENA_CM
    obstacle_cm: float = OBSTACLE_CM
    robot_front_extent_cm: float = ROBOT_FRONT_EXTENT_CM
    robot_rear_extent_cm: float = ROBOT_REAR_EXTENT_CM
    robot_width_cm: float = ROBOT_WIDTH_CM
    safety_margin_cm: float = SAFETY_MARGIN_CM
    camera_forward_offset_cm: float = CAMERA_FORWARD_OFFSET_CM
    camera_lateral_offset_cm: float = CAMERA_LATERAL_OFFSET_CM
    camera_standoff_cm: float = CAMERA_STANDOFF_CM
    minimum_turning_radius_cm: float = MIN_TURNING_RADIUS_CM
    turn_radius_fl_cm: float = TURN_RADIUS_FL_CM
    turn_radius_fr_cm: float = TURN_RADIUS_FR_CM
    turn_radius_bl_cm: float = TURN_RADIUS_BL_CM
    turn_radius_br_cm: float = TURN_RADIUS_BR_CM
    forward_speed_cm_s: float = FORWARD_SPEED_CM_S
    reverse_speed_cm_s: float = REVERSE_SPEED_CM_S
    turning_speed_cm_s: float = TURNING_SPEED_CM_S
    reverse_penalty: float = REVERSE_PENALTY
    collision_check_step_cm: float = COLLISION_CHECK_STEP_CM
    sweep_max_point_step_cm: float = SWEEP_MAX_POINT_STEP_CM
    lattice_straight_step_cm: int = LATTICE_STRAIGHT_STEP_CM
    lattice_turn_step_deg: int = LATTICE_TURN_STEP_DEG
    lattice_position_bin_cm: float = LATTICE_POSITION_BIN_CM
    lattice_heading_bin_deg: int = LATTICE_HEADING_BIN_DEG
    lattice_goal_position_tolerance_cm: float = LATTICE_GOAL_POSITION_TOLERANCE_CM
    lattice_goal_heading_tolerance_deg: float = LATTICE_GOAL_HEADING_TOLERANCE_DEG
    lattice_max_expansions: int = LATTICE_MAX_EXPANSIONS


DEFAULT_PLANNER_CONFIG = PlannerConfig()
