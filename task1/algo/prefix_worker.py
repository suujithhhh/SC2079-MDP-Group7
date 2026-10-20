"""Worker process for `bounded_prefix_planning.py`.

Run as a script by the parent (`python prefix_worker.py`); never started by
HTTP code and never reachable from a client. It speaks a small newline-framed
JSON protocol on its binary stdin/stdout pipes:

    frame  = one JSON object, then "\\n":  {"seq": n, "kind": K, "payload": ...}
    worker -> parent   HELLO, INCUMBENT*, then FINAL or ERROR   (seq 0, 1, 2, ...)
    parent -> worker   SPEC (seq 0), then optionally CANCEL     (seq 0, 1)

There is no digest: local-pipe integrity comes from newline completeness, strict
sequence numbers, a size cap and schema validation. `sys.stdout` inside the
worker is redirected to stderr, so a stray `print()` can never corrupt a frame.

The worker gates on SPEC: it starts no search (and no thread) until the parent
has contained it and sent one. The search is the existing prefix B2/B3 with the
real default leg planner; only the deadline value comes from the parent, and the
observation-only `on_incumbent` hook sends every complete incumbent as it is
found, so the parent still holds the best one if this process is killed.
End of parent stdin means the parent is gone: the worker exits at once.
"""

import json
import math
import os
import sys
import threading

from executable_route_optimizer import plan_executable_leg
from models import Obstacle, Pose
from route_lower_bound import MAX_BOUND_OBSTACLES
from route_prefix_search import (
    MODE_B2,
    MODE_B3,
    PrefixRoute,
    SearchLimits,
    find_prefix_b2_route,
    optimise_prefix_b3_route,
)

PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 2 * 1024 * 1024

KIND_HELLO = "HELLO"
KIND_SPEC = "SPEC"
KIND_CANCEL = "CANCEL"
KIND_INCUMBENT = "INCUMBENT"
KIND_FINAL = "FINAL"
KIND_ERROR = "ERROR"
KINDS = frozenset({KIND_HELLO, KIND_SPEC, KIND_CANCEL, KIND_INCUMBENT, KIND_FINAL, KIND_ERROR})

EXIT_PARENT_GONE = 3
EXIT_BAD_CONTROL_FRAME = 4
EXIT_FAILED = 1

_FACES = ("N", "E", "S", "W")


class FrameError(ValueError):
    """A frame violated the protocol (framing, size, JSON, sequence, kind)."""


def _reject_constant(name):
    raise FrameError(f"non-finite JSON constant {name}")


def encode_frame(seq: int, kind: str, payload) -> bytes:
    line = json.dumps({"seq": seq, "kind": kind, "payload": payload},
                      separators=(",", ":"), allow_nan=False).encode("utf-8") + b"\n"
    if len(line) > MAX_FRAME_BYTES:
        raise FrameError(f"frame of {len(line)} bytes exceeds the {MAX_FRAME_BYTES}-byte cap")
    return line


def decode_frame(line: bytes, expected_seq: int, allowed_kinds=KINDS):
    """`(kind, payload)` of one complete, newline-terminated frame, or `FrameError`."""
    if len(line) > MAX_FRAME_BYTES:
        raise FrameError("oversized frame")
    if not line.endswith(b"\n"):
        raise FrameError("truncated frame (no newline)")
    try:
        frame = json.loads(line.decode("utf-8"), parse_constant=_reject_constant)
    except (UnicodeDecodeError, ValueError) as error:
        if isinstance(error, FrameError):
            raise
        raise FrameError(f"malformed frame: {type(error).__name__}") from None
    if not isinstance(frame, dict) or set(frame) != {"seq", "kind", "payload"}:
        raise FrameError("frame must be an object with exactly seq, kind and payload")
    seq, kind = frame["seq"], frame["kind"]
    if isinstance(seq, bool) or not isinstance(seq, int) or seq != expected_seq:
        raise FrameError(f"out-of-sequence frame: expected {expected_seq}, got {seq!r}")
    if not isinstance(kind, str) or kind not in allowed_kinds:
        raise FrameError(f"unexpected frame kind {kind!r}")
    return kind, frame["payload"]


class FrameWriter:
    """Sequenced, flushed frame writer; a lock keeps frames from interleaving."""

    def __init__(self, stream):
        self._stream = stream
        self._seq = 0
        self._lock = threading.Lock()

    def send(self, kind: str, payload) -> None:
        with self._lock:
            line = encode_frame(self._seq, kind, payload)
            self._stream.write(line)
            self._stream.flush()
            self._seq += 1


def _pose_list(pose: Pose) -> list:
    return [pose.x, pose.y, pose.heading]


def route_record(route: PrefixRoute) -> dict:
    """Compact route record. Only the wire-level facts: the parent re-derives
    everything else and treats the totals below as claims to check."""
    return {
        "obstacle_order": list(route.obstacle_order),
        "candidate_indices": list(route.candidate_indices),
        "skipped_obstacle_ids": list(route.skipped_obstacle_ids),
        "total_cost": route.total_cost,
        "total_distance_cm": route.total_distance_cm,
        "final_pose": _pose_list(route.final_pose),
        "legs": [
            {
                "obstacle_id": leg.obstacle_id,
                "target_pose": _pose_list(leg.target_pose),
                "commands": [[c.direction, c.value] for c in leg.command_plan.commands],
                "final_pose": _pose_list(leg.command_plan.final_pose),
                "total_cost": leg.command_plan.total_cost,
                "total_distance_cm": leg.command_plan.total_distance_cm,
            }
            for leg in route.legs
        ],
    }


def _finite_or_none(value):
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


def final_payload(result) -> dict:
    return {
        "stop_reason": result.stop_reason,
        "work_limit_hit": result.work_limit_hit,
        "route_status": result.route_status,
        "optimality_proven": result.optimality_proven,
        "complete_route_proven_impossible": result.complete_route_proven_impossible,
        "search_space_exhausted": result.search_space_exhausted,
        "lower_bound_certified": result.lower_bound_certified,
        "bound_violation_count": result.bound_violation_count,
        "unresolved_branch_count": result.unresolved_branch_count,
        "min_unresolved_lower_bound": _finite_or_none(result.min_unresolved_lower_bound),
        "unresolved_bound_known": result.unresolved_bound_known,
        "empty_candidate_obstacle_ids": list(result.empty_candidate_obstacle_ids),
        "message": result.message,
        "elapsed_s": result.elapsed_s,
        "leg_calls": result.work.leg_calls,
        "lattice_expansions": result.work.lattice_expansions,
        "nodes_expanded": result.work.nodes_expanded,
        "incumbent_updates": result.work.incumbent_updates,
        "complete_route": None if result.complete_route is None else route_record(result.complete_route),
    }


# --------------------------------------------------------------------------
# SPEC (parent -> worker)
# --------------------------------------------------------------------------

def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def parse_spec(payload):
    """Validate a SPEC payload strictly. Returns `(mode, start, obstacles,
    allow_alternative_viewing, search_budget_s)`; raises `FrameError`."""
    keys = {"mode", "start", "obstacles", "allow_alternative_viewing", "search_budget_s"}
    if not isinstance(payload, dict) or set(payload) != keys:
        raise FrameError("SPEC must contain exactly " + ", ".join(sorted(keys)))
    mode = payload["mode"]
    if mode not in (MODE_B2, MODE_B3):
        raise FrameError(f"SPEC mode {mode!r}")
    start = payload["start"]
    if not (isinstance(start, list) and len(start) == 3 and all(_is_number(v) for v in start)):
        raise FrameError("SPEC start must be [x, y, heading]")
    raw = payload["obstacles"]
    if not (isinstance(raw, list) and 1 <= len(raw) <= MAX_BOUND_OBSTACLES):
        raise FrameError("SPEC obstacles must be a list of 1..%d entries" % MAX_BOUND_OBSTACLES)
    obstacles = []
    for item in raw:
        if not (isinstance(item, list) and len(item) == 4):
            raise FrameError("SPEC obstacle must be [id, x, y, face]")
        oid, x, y, face = item
        if isinstance(oid, bool) or not isinstance(oid, int) or oid <= 0 \
                or not _is_number(x) or not _is_number(y) or face not in _FACES:
            raise FrameError("SPEC obstacle has an invalid field")
        obstacles.append(Obstacle(oid, x, y, face))
    if not isinstance(payload["allow_alternative_viewing"], bool):
        raise FrameError("SPEC allow_alternative_viewing must be a boolean")
    budget = payload["search_budget_s"]
    if not _is_number(budget) or budget <= 0:
        raise FrameError("SPEC search_budget_s must be a finite positive number")
    return mode, Pose(*start), tuple(obstacles), payload["allow_alternative_viewing"], float(budget)


def read_spec(stdin):
    """Block until the parent's SPEC frame arrives. Returns the parsed spec,
    or `None` if the parent closed stdin without sending one."""
    line = stdin.readline(MAX_FRAME_BYTES + 1)
    if not line:
        return None
    kind, payload = decode_frame(line, 0, {KIND_SPEC})
    return parse_spec(payload)


def start_control_thread(stdin, cancel_event: threading.Event) -> threading.Thread:
    """After SPEC: a CANCEL frame sets `cancel_event`; end of stdin (the parent
    is gone) or any malformed control frame ends the process immediately."""
    def watch():
        expected = 1
        while True:
            try:
                line = stdin.readline(MAX_FRAME_BYTES + 1)
            except (OSError, ValueError):
                os._exit(EXIT_PARENT_GONE)
            if not line:
                os._exit(EXIT_PARENT_GONE)
            try:
                decode_frame(line, expected, {KIND_CANCEL})
            except FrameError:
                os._exit(EXIT_BAD_CONTROL_FRAME)
            expected += 1
            cancel_event.set()

    thread = threading.Thread(target=watch, name="prefix-worker-control", daemon=True)
    thread.start()
    return thread


def serve(stdin, stdout, *, leg_planner=plan_executable_leg) -> int:
    """The whole worker protocol. `leg_planner` exists so tests can wrap the
    real one (slow it down, stall it); `main()` always passes the real one."""
    writer = FrameWriter(stdout)
    writer.send(KIND_HELLO, {"protocol": PROTOCOL_VERSION, "pid": os.getpid()})
    try:
        spec = read_spec(stdin)
    except FrameError:
        return EXIT_BAD_CONTROL_FRAME
    if spec is None:
        return EXIT_PARENT_GONE
    mode, start, obstacles, alternatives, budget_s = spec
    cancel = threading.Event()
    start_control_thread(stdin, cancel)
    search = find_prefix_b2_route if mode == MODE_B2 else optimise_prefix_b3_route
    try:
        result = search(
            start, obstacles,
            leg_planner=leg_planner,
            allow_alternative_viewing=alternatives,
            limits=SearchLimits(wall_clock_checkpoint_limit_s=budget_s),
            cancel_requested=cancel.is_set,
            on_incumbent=lambda route, phase: writer.send(
                KIND_INCUMBENT, {"phase": phase, "route": route_record(route)}),
        )
        writer.send(KIND_FINAL, final_payload(result))
    except Exception as error:       # reported by class name only; details stay in the stderr log
        try:
            writer.send(KIND_ERROR, {"error": type(error).__name__})
        except Exception:
            pass
        return EXIT_FAILED
    return 0


def main() -> None:
    stdin, stdout = sys.stdin.buffer, sys.stdout.buffer
    sys.stdout = sys.stderr           # a stray print() must never reach the frame channel
    code = serve(stdin, stdout)
    try:
        stdout.flush()
    finally:
        os._exit(code)                # skip interpreter shutdown: the control thread holds stdin


if __name__ == "__main__":
    main()
