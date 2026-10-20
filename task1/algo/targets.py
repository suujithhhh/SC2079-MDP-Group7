import math
from dataclasses import replace
from typing import Iterable, Tuple

from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from footprint import footprint_in_arena
from models import Obstacle, Pose

IMAGE_FACE_NAMES = {"N": "North", "E": "East", "S": "South", "W": "West"}

# Outward unit normal of each image face, and the heading of a robot facing it.
FACE_OUTWARD = {"N": (0.0, 1.0), "E": (1.0, 0.0),
                "S": (0.0, -1.0), "W": (-1.0, 0.0)}
FACE_HEADING = {"N": -math.pi / 2, "E": math.pi, "S": math.pi / 2, "W": 0.0}


def viewing_target(obstacle: Obstacle, config: PlannerConfig = DEFAULT_PLANNER_CONFIG) -> Pose:
    """Rear-reference pose that puts the camera lens camera_standoff_cm from the
    image face, on the face's centre line, with the robot facing the face."""
    side = obstacle.image_side.upper()
    if side not in FACE_OUTWARD:
        raise ValueError(
            f"Obstacle {obstacle.obstacle_id} has invalid image side: {side}")
    out_x, out_y = FACE_OUTWARD[side]
    half = config.obstacle_cm / 2
    lens_x = obstacle.x + half + out_x * (half + config.camera_standoff_cm)
    lens_y = obstacle.y + half + out_y * (half + config.camera_standoff_cm)
    # The robot faces into the face: forward = -outward, left = forward rotated +90 deg.
    forward_x, forward_y = -out_x, -out_y
    left_x, left_y = -forward_y, forward_x
    return Pose(
        lens_x - config.camera_forward_offset_cm * forward_x
        - config.camera_lateral_offset_cm * left_x,
        lens_y - config.camera_forward_offset_cm * forward_y
        - config.camera_lateral_offset_cm * left_y,
        FACE_HEADING[side],
    )


def viewing_targets(
    obstacle: Obstacle,
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
    lateral_offsets_cm: Iterable[float] = (0.0, -5.0, 5.0),
    additional_standoffs_cm: Iterable[float] = (5.0,),
) -> list[Pose]:
    primary = viewing_target(obstacle, config)
    side = obstacle.image_side.upper()
    lateral_axis = (1.0, 0.0) if side in ("N", "S") else (0.0, 1.0)
    outward_axis = {
        "N": (0.0, 1.0), "S": (0.0, -1.0),
        "E": (1.0, 0.0), "W": (-1.0, 0.0),
    }[side]
    candidates = [
        Pose(primary.x + lateral_axis[0] * offset,
             primary.y + lateral_axis[1] * offset, primary.heading)
        for offset in lateral_offsets_cm
    ]
    candidates.extend(
        Pose(primary.x + outward_axis[0] * extra,
             primary.y + outward_axis[1] * extra, primary.heading)
        for extra in additional_standoffs_cm
    )
    return list(dict.fromkeys(candidates))


def target_for_face(
    obstacle: Obstacle, image_face: str, config: PlannerConfig = DEFAULT_PLANNER_CONFIG
) -> Pose:
    return viewing_target(replace(obstacle, image_face=image_face.upper()), config)


def face_is_available(
    obstacle: Obstacle, image_face: str, config: PlannerConfig = DEFAULT_PLANNER_CONFIG
) -> bool:
    return footprint_in_arena(target_for_face(obstacle, image_face, config), config)


def obstacle_face_edge(
    obstacle: Obstacle, image_face: str | None = None, obstacle_cm: float = 10
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    face = (image_face if image_face is not None else obstacle.image_face).upper()
    if face == "N":
        return (obstacle.x, obstacle.y + obstacle_cm), (obstacle.x + obstacle_cm, obstacle.y + obstacle_cm)
    if face == "E":
        return (obstacle.x + obstacle_cm, obstacle.y), (obstacle.x + obstacle_cm, obstacle.y + obstacle_cm)
    if face == "S":
        return (obstacle.x, obstacle.y), (obstacle.x + obstacle_cm, obstacle.y)
    if face == "W":
        return (obstacle.x, obstacle.y), (obstacle.x, obstacle.y + obstacle_cm)
    raise ValueError("Image face is unset")
