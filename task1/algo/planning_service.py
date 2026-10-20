import math
from typing import Any, Callable

from b3_optimizer import B3RouteResult, find_b2_route, optimise_b3_route
from config import DEFAULT_PLANNER_CONFIG, GRID_CELL_CM, PlannerConfig
from models import Obstacle, Pose


PROTOCOL_VERSION = 1
HEADINGS = {
    "E": 0.0,
    "N": math.pi / 2,
    "W": math.pi,
    "S": -math.pi / 2,
}
COMMAND_NAMES = {"F": "FW", "B": "BW"}

RoutePlanner = Callable[..., B3RouteResult]


class RequestError(ValueError):
    """raised on request error."""


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RequestError(f"{field} must be a number")
    if not math.isfinite(value):
        raise RequestError(f"{field} must be finite")
    return float(value)


def _heading(value: Any) -> float:
    if isinstance(value, str):
        try:
            return HEADINGS[value.upper()]
        except KeyError:
            raise RequestError(
                "start.heading must be N, E, S, W or radians") from None
    return _number(value, "start.heading")


def _coordinate(value: Any, field: str, units: str) -> float:
    coordinate = _number(value, field)
    return coordinate * GRID_CELL_CM if units == "grid" else coordinate


def parse_plan_request(
    payload: Any,
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
) -> tuple[str, Pose, list[Obstacle]]:
    if not isinstance(payload, dict):
        raise RequestError("request must be a JSON object")
    if payload.get("type") != "PLAN_REQUEST":
        raise RequestError("type must be PLAN_REQUEST")
    version = payload.get("protocol_version", PROTOCOL_VERSION)
    if version != PROTOCOL_VERSION:
        raise RequestError(f"unsupported protocol_version: {version}")
    mode = payload.get("mode", "B3")
    if not isinstance(mode, str) or mode.upper() not in ("B2", "B3"):
        raise RequestError("mode must be B2 or B3")
    mode = mode.upper()
    units = payload.get("coordinate_units", "cm")
    if units not in ("cm", "grid"):
        raise RequestError("coordinate_units must be cm or grid")

    start_data = payload.get("start")
    if not isinstance(start_data, dict):
        raise RequestError("start must be an object")
    start = Pose(
        _coordinate(start_data.get("x"), "start.x", units),
        _coordinate(start_data.get("y"), "start.y", units),
        _heading(start_data.get("heading")),
    )

    obstacle_data = payload.get("obstacles")
    if not isinstance(obstacle_data, list) or not 1 <= len(obstacle_data) <= 5:
        raise RequestError("obstacles must contain between 1 and 5 items")
    obstacles: list[Obstacle] = []
    identifiers: set[int] = set()
    for index, item in enumerate(obstacle_data):
        if not isinstance(item, dict):
            raise RequestError(f"obstacles[{index}] must be an object")
        identifier = item.get("id")
        if isinstance(identifier, bool) or not isinstance(identifier, int) or identifier <= 0:
            raise RequestError(
                f"obstacles[{index}].id must be a positive integer")
        if identifier in identifiers:
            raise RequestError(f"duplicate obstacle id: {identifier}")
        face = item.get("face")
        if not isinstance(face, str) or face.upper() not in HEADINGS:
            raise RequestError(f"obstacles[{index}].face must be N, E, S or W")
        obstacle = Obstacle(
            identifier,
            _coordinate(item.get("x"), f"obstacles[{index}].x", units),
            _coordinate(item.get("y"), f"obstacles[{index}].y", units),
            face.upper(),
        )
        if not (
            0 <= obstacle.x <= config.arena_cm - config.obstacle_cm
            and 0 <= obstacle.y <= config.arena_cm - config.obstacle_cm
        ):
            raise RequestError(f"obstacle {identifier} lies outside the arena")
        identifiers.add(identifier)
        obstacles.append(obstacle)
    return mode, start, obstacles


def _pose_payload(pose: Pose) -> dict[str, float]:
    return {
        "x_cm": pose.x,
        "y_cm": pose.y,
        "heading_rad": pose.heading,
    }


def _result_payload(mode: str, result: B3RouteResult, config: PlannerConfig) -> dict[str, Any]:
    if not result.is_valid:
        return {
            "type": "PLAN_RESULT",
            "protocol_version": PROTOCOL_VERSION,
            "mode": mode,
            "valid": False,
            "error": result.failure_reason or "No continuous route was found.",
        }

    commands: list[dict[str, Any]] = []
    waypoints: list[dict[str, float]] = []
    for obstacle_id, leg in zip(result.obstacle_order, result.leg_results):
        for command in leg.commands:
            commands.append({
                "type": COMMAND_NAMES.get(command.kind, command.kind),
                "value": round(command.value, 4),
                "value_unit": "cm_path_length",
            })
        commands.append({"type": "SNAP", "obstacle_id": obstacle_id})
        leg_poses = leg.poses if not waypoints else leg.poses[1:]
        waypoints.extend(_pose_payload(pose) for pose in leg_poses)

    return {
        "type": "PLAN_RESULT",
        "protocol_version": PROTOCOL_VERSION,
        "mode": mode,
        "valid": True,
        "complete": result.is_complete,
        "obstacle_order": result.obstacle_order,
        "visited_count": len(result.obstacle_order),
        "skipped_obstacle_ids": result.skipped_obstacle_ids,
        "target_poses": [_pose_payload(pose) for pose in result.target_poses],
        "waypoints": waypoints,
        "commands": commands,
        "estimated_time_s": round(result.estimated_time_s, 4),
        "minimum_turning_radius_cm": config.minimum_turning_radius_cm,
        "command_contract": "turn values are arc path lengths, not turn angles",
    }


def plan_request(
    payload: Any,
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
    b2_planner: RoutePlanner = find_b2_route,
    b3_planner: RoutePlanner = optimise_b3_route,
) -> dict[str, Any]:
    try:
        mode, start, obstacles = parse_plan_request(payload, config)
        planner = b2_planner if mode == "B2" else b3_planner
        result = planner(
            start,
            obstacles,
            config,
            allow_alternative_viewing=True,
        )
        return _result_payload(mode, result, config)
    except RequestError as error:
        return {
            "type": "PLAN_ERROR",
            "protocol_version": PROTOCOL_VERSION,
            "valid": False,
            "error": str(error),
        }
