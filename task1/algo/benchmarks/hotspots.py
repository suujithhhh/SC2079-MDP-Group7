"""Opt-in timing wrappers around the collision hot spots (measurement only).

`Hotspots.install()` replaces a few module-level functions with wrappers
that call the original with the same arguments, return its result
unchanged, and add its elapsed time and call count to a table. Nothing about
planning behaviour changes; only the wall time grows by the wrapper
overhead, so timings taken with hotspots installed are perturbed and must be
compared with `--profile none` runs.

Timings are INCLUSIVE and nested: `footprint_hits_obstacle` (the SAT test)
runs inside `pose_is_safe`, which runs inside `safe_primitive_samples`,
which is where the state-lattice search spends its expansions. Never add
the rows together. Swept-path samples are checked by
`pose_is_safe_prepared` (bit-exact to `pose_is_safe`, see footprint.py), so
`pose_is_safe` and `footprint_hits_obstacle` now count only the other
callers (leg start/goal and viewing-candidate checks, the reference fallback).
Likewise swept samples come from `_uniform_primitive_samples` (bit-exact to
`pose_at_fraction`), so `pose_at_fraction` counts one validation call per
primitive plus any fallback samples.
Lattice legs prepare their obstacle geometry once (`_prepare_leg_geometry`)
and check each primitive through `_safe_primitive_samples_for_leg`, so
`safe_primitive_samples` now counts only the other callers and fallbacks;
`prepare_primitive_check` runs once per primitive check.

It also counts goal tests in `state_lattice._within_goal_tolerance`:
how often the popped pose was already inside the goal position tolerance
(`position_in_tolerance`) yet failed the test (`heading_blocked`), i.e.
evidence of search effort spent on heading near the goal.
"""

import math
import time

import footprint
import motion_primitives
import state_lattice
import swept_collision

WRAPPED = (
    ("pose_is_safe", footprint),
    ("footprint_in_arena", footprint),
    ("footprint_hits_obstacle", footprint),
    ("pose_is_safe_prepared", footprint),
    ("safe_primitive_samples", swept_collision),
    ("_prepare_leg_geometry", swept_collision),
    ("_safe_primitive_samples_for_leg", swept_collision),
    ("prepare_primitive_check", footprint),
    ("pose_at_fraction", motion_primitives),
    ("_uniform_primitive_samples", motion_primitives),
)


class Hotspots:
    def __init__(self):
        self.table = {}   # label -> [calls, seconds]
        self.goal = {"goal_tests": 0, "position_in_tolerance": 0, "heading_blocked": 0}
        self._restore = []

    def _timed(self, label, function):
        row = self.table.setdefault(label, [0, 0.0])
        clock = time.perf_counter

        def wrapper(*args, **kwargs):
            start = clock()
            try:
                return function(*args, **kwargs)
            finally:
                row[1] += clock() - start
                row[0] += 1

        return wrapper

    def _goal_counter(self, function):
        goal = self.goal

        def wrapper(pose, target, config):
            result = function(pose, target, config)
            goal["goal_tests"] += 1
            if math.hypot(pose.x - target.x, pose.y - target.y) <= config.lattice_goal_position_tolerance_cm:
                goal["position_in_tolerance"] += 1
                if not result:
                    goal["heading_blocked"] += 1
            return result

        return wrapper

    def install(self):
        for name, module in WRAPPED:
            original = getattr(module, name)
            self._restore.append((module, name, original))
            setattr(module, name, self._timed(name, original))
        original = state_lattice._within_goal_tolerance
        self._restore.append((state_lattice, "_within_goal_tolerance", original))
        state_lattice._within_goal_tolerance = self._goal_counter(original)

    def uninstall(self):
        while self._restore:
            module, name, original = self._restore.pop()
            setattr(module, name, original)

    def report(self) -> dict:
        return {
            "functions": {
                label: {"calls": row[0], "seconds": row[1]} for label, row in sorted(self.table.items())
            },
            "goal_tests": dict(self.goal),
        }
