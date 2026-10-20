"""Conservative swept-path collision checking for the six STM primitives.

Wraps motion_primitives.py's pure geometry with sampling along the *whole*
commanded path of one `F`/`B`/`FL`/`FR`/`BL`/`BR` primitive, not just its
endpoint, and validates the full margin-inflated footprint
(footprint.pose_is_safe) at every sample. Intended as a reusable building
block for a future state-lattice planner and for web path display — it is
not wired into planner_interface.py, b3_optimizer.py, planning_service.py
or the web UI, and the legacy symmetric-radius planner is unaffected.

Why sampling alone is not enough
---------------------------------
Checking the margin-inflated footprint only at a finite set of sampled
poses cannot, by itself, rule out a collision that occurs strictly between
two adjacent samples ("tunnelling"). This module closes that gap with a
conservative, purely numerical guard — not a second physical safety
margin:

1. Samples are uniformly spaced in "fraction of the primitive" (via
   motion_primitives.pose_at_fraction), including both the exact start
   pose and the exact propagated endpoint.
2. Every point in the continuous interior of the swept path is therefore
   at most *half an interval* away (in fraction) from its nearest sample.
3. We compute a conservative upper bound on how far any point of the
   already margin-inflated footprint can move over half an interval:
   - straight primitives: half the actual interval's travel distance,
     since every footprint point translates identically (no rotation);
   - curved primitives: the (fixed, per-corner) distance from the
     primitive's instantaneous centre of curvature (motion_primitives.
     turn_center_cm) to the farthest corner of the inflated footprint,
     multiplied by half the actual interval's angle — an arc-length bound
     that is always >= the true straight-line displacement for that
     rotation.
4. Each sampled pose is checked with footprint.pose_is_safe() under a
   *temporary* PlannerConfig whose safety_margin_cm is the real physical
   margin (config.safety_margin_cm, still exactly 3 cm by default) *plus*
   that computed guard. The guard only pads the collision check; the
   samples returned to the caller carry the caller's real, un-padded
   config-implied geometry.

This guarantees that under this module's rigid constant-radius model, if
every sampled pose (checked with the inflated guard) is reported safe, the
real 1 cm-margin footprint cannot have entered an obstacle or left the
arena at any point *between* the samples either. It is intentionally
conservative: it can reject a primitive that a finer analysis would allow,
but it never accepts one that truly tunnels through an obstacle or the
arena boundary.
"""

import dataclasses
import math
from typing import Iterable

import footprint
import motion_primitives
from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from models import Obstacle, Pose


class InvalidSweepStepError(ValueError):
    """Raised when `config.sweep_max_point_step_cm` is not a finite positive number."""


def _validate_sweep_step(step_cm) -> None:
    if isinstance(step_cm, bool) or not isinstance(step_cm, (int, float)):
        raise InvalidSweepStepError(
            f"sweep_max_point_step_cm must be a finite positive number, got {step_cm!r}"
        )
    if not math.isfinite(step_cm) or step_cm <= 0:
        raise InvalidSweepStepError(
            f"sweep_max_point_step_cm must be finite and positive, got {step_cm!r}"
        )


def _max_margin_corner_radius(start: Pose, action: str, config: PlannerConfig) -> float:
    """Farthest distance from the turn's fixed centre to any inflated-footprint corner."""
    center_x, center_y = motion_primitives.turn_center_cm(start, action, config)
    corners = footprint.footprint_corners(start, config, include_margin=True)
    return max(math.hypot(cx - center_x, cy - center_y) for cx, cy in corners)


def _sample_and_guard_cm(
    start: Pose, action: str, value: int, config: PlannerConfig
) -> tuple[tuple[Pose, ...], float]:
    """Uniformly-spaced samples of one primitive, plus its anti-tunnelling guard (cm).

    Validates `action`/`value` via motion_primitives' own public API (so
    there is only one place those rules live), then `config.
    sweep_max_point_step_cm`, before computing anything else.
    """
    # Validates action/value (and this fraction, trivially) through the
    # public API rather than reaching into motion_primitives' internals.
    motion_primitives.pose_at_fraction(start, action, value, 0.0, config)
    _validate_sweep_step(config.sweep_max_point_step_cm)
    step_cm = config.sweep_max_point_step_cm

    if action in motion_primitives.STRAIGHT_ACTIONS:
        distance_cm = abs(float(value))
        interval_count = max(1, math.ceil(distance_cm / step_cm))
        interval_distance_cm = distance_cm / interval_count
        guard_cm = interval_distance_cm / 2.0
    else:
        theta_rad = math.radians(value)
        max_radius_cm = _max_margin_corner_radius(start, action, config)
        arc_bound_cm = max_radius_cm * theta_rad
        interval_count = max(1, math.ceil(arc_bound_cm / step_cm))
        interval_angle_rad = theta_rad / interval_count
        guard_cm = max_radius_cm * (interval_angle_rad / 2.0)

    if motion_primitives._uniform_sampling_supported(start, action, value, config):
        # Bit-exact prepared sampler; action/value were validated just above.
        samples = motion_primitives._uniform_primitive_samples(
            start, action, value, interval_count, config
        )
        return samples, guard_cm
    fractions = [i / interval_count for i in range(interval_count + 1)]
    samples = tuple(
        motion_primitives.pose_at_fraction(start, action, value, f, config)
        for f in fractions
    )
    return samples, guard_cm


def sample_primitive(
    start: Pose, action: str, value: int, config: PlannerConfig = DEFAULT_PLANNER_CONFIG
) -> tuple[Pose, ...]:
    """Uniformly-spaced poses along one primitive's swept path.

    Includes the exact start pose (first) and the exact `propagate()`
    endpoint (last). Spacing satisfies the `config.sweep_max_point_step_cm`
    bound for every point of the margin-inflated footprint, not just the
    rear reference (see module docstring). Returns an immutable tuple.
    """
    samples, _ = _sample_and_guard_cm(start, action, value, config)
    return samples


def safe_primitive_samples(
    start: Pose,
    action: str,
    value: int,
    obstacles: Iterable[Obstacle],
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
) -> "tuple[Pose, ...] | None":
    """Return the primitive's samples if the whole swept path is safe, else None.

    Every sampled pose is checked with footprint.pose_is_safe() under a
    temporary config whose safety margin is padded by this primitive's
    anti-tunnelling guard (module docstring); `config` itself is never
    mutated (`dataclasses.replace` builds a new instance). `obstacles` is
    consumed into a tuple up front so a one-shot iterable can safely be
    checked against every sample.

    For a plain `PlannerConfig` with finite plain-number geometry, the check
    runs through footprint.prepare_pose_check()/pose_is_safe_prepared(),
    which give pose_is_safe()'s exact decision at every sample with the same
    guarded margin, without building the temporary config. Any other config
    (a subclass, a non-dataclass object) or input goes through the reference
    path below unchanged, including its `dataclasses.replace` TypeError.
    """
    obstacles = tuple(obstacles)
    samples, guard_cm = _sample_and_guard_cm(start, action, value, config)
    if type(config) is PlannerConfig:
        prepared = footprint.prepare_pose_check(config, guard_cm, obstacles)
        if prepared is not None:
            bounds, rects, arena_cm = prepared
            check = footprint.pose_is_safe_prepared
            if all(check(pose, bounds, rects, arena_cm) for pose in samples):
                return samples
            return None
    guarded_config = dataclasses.replace(
        config, safety_margin_cm=config.safety_margin_cm + guard_cm
    )
    if all(footprint.pose_is_safe(pose, obstacles, guarded_config) for pose in samples):
        return samples
    return None


def primitive_is_safe(
    start: Pose,
    action: str,
    value: int,
    obstacles: Iterable[Obstacle],
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
) -> bool:
    """True when `safe_primitive_samples` would return a non-None result."""
    return safe_primitive_samples(start, action, value, obstacles, config) is not None


# --- Per-leg prepared obstacle geometry ---------------------------------------
#
# state_lattice.plan_lattice_path() checks six primitives per expansion against
# one fixed obstacle tuple and config. The obstacle rectangles and config
# extents are identical for all of them; only the guard-dependent bounds differ
# per primitive. A _LegCollisionGeometry is built once per leg call, lives only
# for that call (no module-level state or cache) and carries its own obstacles
# and config, so a mismatched pair cannot be passed.

@dataclasses.dataclass(frozen=True, slots=True)
class _LegCollisionGeometry:
    obstacles: tuple
    config: PlannerConfig
    rects: "tuple | None"   # None: outside the hoist domain, use the public path


def _prepare_leg_geometry(obstacles: tuple, config: PlannerConfig) -> _LegCollisionGeometry:
    return _LegCollisionGeometry(
        obstacles, config, footprint.prepare_obstacle_rects(config, obstacles)
    )


def _safe_primitive_samples_for_leg(
    start: Pose, action: str, value: int, leg: _LegCollisionGeometry
) -> "tuple[Pose, ...] | None":
    """safe_primitive_samples(start, action, value, leg.obstacles, leg.config), exactly.

    Same result, error type and error timing: validation happens inside
    _sample_and_guard_cm() first, and any input the prepared path cannot
    take falls back to the public function.
    """
    if leg.rects is None:
        return safe_primitive_samples(start, action, value, leg.obstacles, leg.config)
    samples, guard_cm = _sample_and_guard_cm(start, action, value, leg.config)
    prepared = footprint.prepare_primitive_check(leg.config, guard_cm, leg.rects)
    if prepared is None:
        return safe_primitive_samples(start, action, value, leg.obstacles, leg.config)
    bounds, rects, arena_cm = prepared
    check = footprint.pose_is_safe_prepared
    if all(check(pose, bounds, rects, arena_cm) for pose in samples):
        return samples
    return None
