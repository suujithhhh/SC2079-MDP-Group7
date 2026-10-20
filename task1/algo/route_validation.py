import math
from typing import Iterable

from config import PlannerConfig
from models import PathResult, Pose


def normalise_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def transition_turn_radius(start: Pose, end: Pose) -> float:
    chord = math.hypot(end.x - start.x, end.y - start.y)
    turn_angle = abs(normalise_angle(end.heading - start.heading))
    if turn_angle <= 1e-9:
        return math.inf
    sine = math.sin(turn_angle / 2)
    if sine <= 1e-9:
        return 0.0
    return chord / (2 * sine)


def is_continuous_car_path(result: PathResult, config: PlannerConfig) -> bool:
    if not result.is_valid or len(result.poses) < 2:
        return False
    for start, end in zip(result.poses, result.poses[1:]):
        chord = math.hypot(end.x - start.x, end.y - start.y)
        turn_angle = abs(normalise_angle(end.heading - start.heading))
        if chord <= 1e-9:
            return False
        if turn_angle > 1e-9 and transition_turn_radius(start, end) + 1e-6 < config.minimum_turning_radius_cm:
            return False
    return True


def route_is_continuous(results: Iterable[PathResult], config: PlannerConfig) -> bool:
    results = list(results)
    return bool(results) and all(is_continuous_car_path(result, config) for result in results)
