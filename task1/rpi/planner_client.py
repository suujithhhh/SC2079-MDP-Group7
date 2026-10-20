"""RPi -> PC planner (`POST /plan/v2`, PROTOCOL.md §7) and pose conversions.

The v2 planner plans with the STM's four measured turn radii and the robot's
measured footprint, and returns each MOV as its exact STM string
(`stm_command`), which the Pi sends unchanged. Poses are the rear-axle
midpoint in cm, heading in degrees CCW from East (PROTOCOL.md §1).

Input (Android PLAN_REQUEST JSON line or a --map file):
  * `coordinate_units`: "grid" (1 unit = 10 cm) or "cm".
  * `start`: the robot's REAR-AXLE MIDPOINT (the planner's reference point,
    e.g. 20, 10 cm) plus heading N/E/S/W. Passed to the planner unchanged.
  * `obstacles`: bottom-left corner of each 10x10 cm obstacle, and image face.
"""

from __future__ import annotations

import json
import math
import urllib.error
import urllib.request
from dataclasses import dataclass

GRID_CELL_CM = 10
FACES = ("N", "E", "S", "W")
HEADING_DEG = {"E": 0.0, "N": 90.0, "W": 180.0, "S": 270.0}

# Measured robot footprint (PROTOCOL.md §2), relative to the rear axle.
ROBOT_FRONT_CM = 24.0
ROBOT_HALF_WIDTH_CM = 9.5
BODY_CENTRE_FORWARD_CM = 12.0
@dataclass(frozen=True)
class StartPose:
    """Rear-axle midpoint in cm, heading in degrees CCW from East."""

    x_cm: float
    y_cm: float
    heading_deg: float


class PlannerError(RuntimeError):
    pass


def start_pose(start: dict, scale: float) -> StartPose:
    """Rear-axle midpoint in request units + heading letter -> pose in cm."""
    try:
        heading = HEADING_DEG[str(start["heading"]).upper()]
        x, y = float(start["x"]) * scale, float(start["y"]) * scale
    except (KeyError, TypeError, ValueError) as exc:
        raise PlannerError("PLAN_REQUEST start must have x, y and heading N/E/S/W") from exc
    return StartPose(x, y, heading)


def parse_android_request(line: str) -> tuple[str, StartPose, list[dict]]:
    """Return (mode, rear-axle start, obstacles in cm) from a PLAN_REQUEST line."""
    try:
        payload = json.loads(line)
    except json.JSONDecodeError as exc:
        raise PlannerError(f"PLAN_REQUEST is not valid JSON: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("type") != "PLAN_REQUEST":
        raise PlannerError("message is not a PLAN_REQUEST")
    units = payload.get("coordinate_units", "grid")
    if units not in ("grid", "cm"):
        raise PlannerError(f"coordinate_units must be grid or cm, not {units!r}")
    scale = GRID_CELL_CM if units == "grid" else 1
    obstacles = []
    for item in payload.get("obstacles") or []:
        face = str(item.get("face", "")).upper()
        if face not in FACES:
            raise PlannerError(f"obstacle {item.get('id')} has no valid face")
        obstacles.append({"id": int(item["id"]), "x": item["x"] * scale,
                          "y": item["y"] * scale, "face": face})
    if not obstacles:
        raise PlannerError("PLAN_REQUEST has no obstacles")
    start = start_pose(payload.get("start") or {}, scale)
    return str(payload.get("mode", "B3")), start, obstacles


def build_plan_request(
    mission_id: str,
    mode: str,
    obstacles: list[dict],
    start: StartPose,
    motion_profile: str = "PRECISION",
    allow_alternative_viewing: bool = True,
) -> dict:
    return {
        "type": "PLAN_REQUEST",
        "protocol_version": 2,
        "mission_id": mission_id,
        "mode": mode,
        "coordinate_units": "cm",
        "motion_profile": motion_profile,
        "allow_alternative_viewing": allow_alternative_viewing,
        "start": {"x": start.x_cm, "y": start.y_cm, "heading_deg": start.heading_deg},
        "obstacles": obstacles,
    }


DEFAULT_PLANNER_PORT = 5000


def planner_url(target: str) -> str:
    """`10.91.1.2`, `10.91.1.2:5000` or a full URL -> the laptop's /plan/v2 URL.
    Raises ValueError for any other path (for example the old /plan, which is protocol v1)."""
    target = target.strip().rstrip("/")
    if "://" not in target:
        target = f"http://{target}"
    scheme, rest = target.split("://", 1)
    host, _, path = rest.partition("/")
    if ":" not in host:
        host = f"{host}:{DEFAULT_PLANNER_PORT}"
    if path not in ("", "plan/v2"):
        raise ValueError(f"planner URL path must be /plan/v2, not /{path}")
    return f"{scheme}://{host}/plan/v2"


def images_url(plan_url: str) -> str:
    """The laptop's /plan/v2 URL -> its `POST /images` URL (PROTOCOL.md §7.1)."""
    scheme, rest = plan_url.split("://", 1)
    return f"{scheme}://{rest.partition('/')[0]}/images"


def request_plan(url: str, request: dict, timeout_s: float = 75.0) -> dict:
    """POST the request; return a valid PLAN_RESULT or raise PlannerError."""
    body = json.dumps(request).encode("utf-8")
    http_request = urllib.request.Request(
        url, data=body, headers={"Content-Type": "application/json"}, method="POST"
    )
    try:
        with urllib.request.urlopen(http_request, timeout=timeout_s) as response:
            result = json.loads(response.read())
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")[:200]
        raise PlannerError(f"planner HTTP {exc.code}: {detail}") from exc
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise PlannerError(f"planner unreachable at {url}: {exc}") from exc
    return _check_result(request, result)


def _check_result(request: dict, result: dict) -> dict:
    if result.get("mission_id") != request["mission_id"]:
        raise PlannerError("planner echoed a different mission_id")
    if result.get("type") != "PLAN_RESULT" or not result.get("valid"):
        reason = result.get("failure_reason") or result.get("error") or "no valid route"
        raise PlannerError(f"planner: {reason}")
    return result


def plan_locally(algo_dir: str, request: dict) -> dict:
    """Run the repo's v2 planner in this process. Laptop-side tools only: the
    Pi never plans, it calls the laptop's server with `request_plan`."""
    import sys

    if algo_dir not in sys.path:
        sys.path.insert(0, algo_dir)
    try:
        from planning_service_v2 import plan_request_v2
    except ImportError as exc:
        raise PlannerError(f"planner not found in {algo_dir}: {exc}") from exc
    return _check_result(request, plan_request_v2(request))


def heading_to_direction(heading_deg: float) -> str:
    """Nearest compass direction; E = 0, N = 90."""
    return ("E", "N", "W", "S")[int(((heading_deg % 360) + 45) // 90) % 4]


def android_robot_cell(pose: dict) -> tuple[int, int, str]:
    """Rear-axle pose -> Android ROBOT (bottom-left cell of its 3x3 box).

    Assumes Android's 3x3 box is centred on the robot body (12 cm ahead of the
    rear axle). Display only; nothing is planned from it.
    """
    heading = math.radians(pose["heading_deg"])
    centre_x = pose["x_cm"] + BODY_CENTRE_FORWARD_CM * math.cos(heading)
    centre_y = pose["y_cm"] + BODY_CENTRE_FORWARD_CM * math.sin(heading)
    return (math.floor(centre_x / GRID_CELL_CM - 1.5 + 0.5), math.floor(centre_y / GRID_CELL_CM - 1.5 + 0.5),
            heading_to_direction(pose["heading_deg"]))
