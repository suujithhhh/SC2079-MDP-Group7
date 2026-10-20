from dataclasses import replace
from typing import Iterable, Optional

from config import ARENA_CM, EDITOR_SNAP_CM, OBSTACLE_CM, START_ZONE_CM
from models import Obstacle

IMAGE_SIDES = ("N", "E", "S", "W")


def snap_to_grid(value: float) -> float:
    return round(value / EDITOR_SNAP_CM) * EDITOR_SNAP_CM


def placement_is_valid(candidate: Obstacle, obstacles: Iterable[Obstacle]) -> bool:
    if candidate.x < 0 or candidate.y < 0:
        return False
    if candidate.x + OBSTACLE_CM > ARENA_CM or candidate.y + OBSTACLE_CM > ARENA_CM:
        return False
    if candidate.x < START_ZONE_CM and candidate.y < START_ZONE_CM:
        return False
    for other in obstacles:
        if other.obstacle_id == candidate.obstacle_id:
            continue
        separated = (
            candidate.x + OBSTACLE_CM <= other.x
            or other.x + OBSTACLE_CM <= candidate.x
            or candidate.y + OBSTACLE_CM <= other.y
            or other.y + OBSTACLE_CM <= candidate.y
        )
        if not separated:
            return False
    return True


class MapEditorModel:
    def __init__(self, obstacles: Iterable[Obstacle]) -> None:
        self.obstacles = list(obstacles)

    def obstacle_at(self, x: float, y: float) -> Optional[Obstacle]:
        return next(
            (item for item in reversed(self.obstacles) if item.x <= x <=
             item.x + OBSTACLE_CM and item.y <= y <= item.y + OBSTACLE_CM),
            None,
        )

    def move(self, obstacle_id: int, x: float, y: float) -> bool:
        index = next((i for i, item in enumerate(self.obstacles)
                     if item.obstacle_id == obstacle_id), None)
        if index is None:
            return False
        candidate = replace(
            self.obstacles[index], x=snap_to_grid(x), y=snap_to_grid(y))
        if not placement_is_valid(candidate, self.obstacles):
            return False
        self.obstacles[index] = candidate
        return True

    def add(self, x: float, y: float, image_face: str = "") -> Optional[Obstacle]:
        if len(self.obstacles) >= 5:
            return None
        used_ids = {item.obstacle_id for item in self.obstacles}
        obstacle_id = next(value for value in range(1, 6)
                           if value not in used_ids)
        candidate = Obstacle(obstacle_id, snap_to_grid(x),
                             snap_to_grid(y), image_face)
        if not placement_is_valid(candidate, self.obstacles):
            return None
        self.obstacles.append(candidate)
        self.obstacles.sort(key=lambda item: item.obstacle_id)
        return candidate

    def delete(self, obstacle_id: int) -> bool:
        for index, obstacle in enumerate(self.obstacles):
            if obstacle.obstacle_id == obstacle_id:
                del self.obstacles[index]
                return True
        return False

    def cycle_face(self, obstacle_id: int) -> bool:
        index = next((i for i, item in enumerate(self.obstacles)
                     if item.obstacle_id == obstacle_id), None)
        if index is None:
            return False
        obstacle = self.obstacles[index]
        side = obstacle.image_side.upper()
        next_side = IMAGE_SIDES[0] if side not in IMAGE_SIDES else IMAGE_SIDES[(
            IMAGE_SIDES.index(side) + 1) % len(IMAGE_SIDES)]
        self.obstacles[index] = replace(obstacle, image_face=next_side)
        return True

    def set_face(self, obstacle_id: int, image_face: str) -> bool:
        image_face = image_face.upper()
        if image_face not in IMAGE_SIDES:
            return False
        for index, obstacle in enumerate(self.obstacles):
            if obstacle.obstacle_id == obstacle_id:
                self.obstacles[index] = replace(
                    obstacle, image_face=image_face)
                return True
        return False
