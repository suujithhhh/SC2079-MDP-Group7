import heapq
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Set, Tuple

from config import ANIMATION_STEP_CM, ARENA_CM, GRID_CELL_CM, OBSTACLE_CM, ROBOT_WIDTH_CM, SAFETY_MARGIN_CM
from models import Obstacle, Pose
from targets import viewing_target

GRID_SIZE_CM = 5
ROBOT_HALF_WIDTH_CM = ROBOT_WIDTH_CM / 2 + SAFETY_MARGIN_CM
ARENA_MIN_CM = ROBOT_HALF_WIDTH_CM
ARENA_MAX_CM = ARENA_CM - ROBOT_HALF_WIDTH_CM
Point = Tuple[float, float]
Cell = Tuple[int, int]


def virtual_obstacle_bounds(obstacle: Obstacle) -> Tuple[float, float, float, float]:
    return (
        obstacle.x - ROBOT_HALF_WIDTH_CM,
        obstacle.x + OBSTACLE_CM + ROBOT_HALF_WIDTH_CM,
        obstacle.y - ROBOT_HALF_WIDTH_CM,
        obstacle.y + OBSTACLE_CM + ROBOT_HALF_WIDTH_CM,
    )


def point_in_virtual_obstacle(x: float, y: float, obstacles: Iterable[Obstacle]) -> bool:
    for obstacle in obstacles:
        min_x, max_x, min_y, max_y = virtual_obstacle_bounds(obstacle)
        if min_x < x < max_x and min_y < y < max_y:
            return True
    return False


def is_valid_pose(pose: Pose, obstacles: Iterable[Obstacle]) -> bool:
    return (
        ARENA_MIN_CM <= pose.x <= ARENA_MAX_CM
        and ARENA_MIN_CM <= pose.y <= ARENA_MAX_CM
        and not point_in_virtual_obstacle(pose.x, pose.y, obstacles)
    )


def planned_step(pose: Pose, waypoint: Pose, step_cm: float = ANIMATION_STEP_CM) -> Pose:
    """Follow one already-safe waypoint without planning or obstacle steering."""
    dx = waypoint.x - pose.x
    dy = waypoint.y - pose.y
    distance = math.hypot(dx, dy)
    if distance <= step_cm:
        return Pose(waypoint.x, waypoint.y, waypoint.theta)
    return Pose(
        pose.x + step_cm * dx / distance,
        pose.y + step_cm * dy / distance,
        math.atan2(dy, dx),
    )


def _cell_center(cell: Cell) -> Point:
    return (cell[0] * GRID_SIZE_CM + GRID_SIZE_CM / 2, cell[1] * GRID_SIZE_CM + GRID_SIZE_CM / 2)


def _valid_cell(cell: Cell, obstacles: List[Obstacle]) -> bool:
    return is_valid_pose(Pose(*_cell_center(cell), 0.0), obstacles)


def _segment_is_clear(start: Point, end: Point, obstacles: List[Obstacle]) -> bool:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    samples = max(1, math.ceil(distance / (GRID_SIZE_CM / 2)))
    for index in range(samples + 1):
        fraction = index / samples
        pose = Pose(
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]),
            0.0,
        )
        if not is_valid_pose(pose, obstacles):
            return False
    return True


def _nearest_valid_cell(point: Point, obstacles: List[Obstacle]) -> Optional[Cell]:
    nearest = (round(point[0] / GRID_SIZE_CM), round(point[1] / GRID_SIZE_CM))
    candidates: List[Tuple[float, Cell]] = []
    for x in range(max(0, nearest[0] - 4), min(40, nearest[0] + 4) + 1):
        for y in range(max(0, nearest[1] - 4), min(40, nearest[1] + 4) + 1):
            cell = (x, y)
            if _valid_cell(cell, obstacles):
                center = _cell_center(cell)
                candidates.append(
                    (math.hypot(center[0] - point[0], center[1] - point[1]), cell))
    if not candidates:
        return None
    return min(candidates)[1]


def find_safe_path(start: Pose, target: Pose, obstacles: Iterable[Obstacle]) -> List[Pose]:
    obstacle_list = list(obstacles)
    if not is_valid_pose(start, obstacle_list) or not is_valid_pose(target, obstacle_list):
        return []

    start_cell = _nearest_valid_cell((start.x, start.y), obstacle_list)
    target_cell = _nearest_valid_cell((target.x, target.y), obstacle_list)
    if start_cell is None or target_cell is None:
        return []

    open_set: List[Tuple[float, float, int, int]] = [
        (0.0, 0.0, start_cell[0], start_cell[1])]
    came_from: dict[Cell, Optional[Cell]] = {start_cell: None}
    g_score = {start_cell: 0.0}

    while open_set:
        _, _, x, y = heapq.heappop(open_set)
        current = (x, y)
        if current == target_cell:
            break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (x + dx, y + dy)
            if not (0 <= neighbor[0] <= 40 and 0 <= neighbor[1] <= 40):
                continue
            if not _valid_cell(neighbor, obstacle_list):
                continue
            if not _segment_is_clear(_cell_center(current), _cell_center(neighbor), obstacle_list):
                continue
            cost = g_score[current] + GRID_SIZE_CM
            if cost < g_score.get(neighbor, math.inf):
                g_score[neighbor] = cost
                came_from[neighbor] = current
                heuristic = math.hypot(
                    neighbor[0] - target_cell[0], neighbor[1] - target_cell[1]) * GRID_SIZE_CM
                heapq.heappush(open_set, (cost + heuristic,
                               cost, neighbor[0], neighbor[1]))

    if target_cell not in came_from:
        return []
    cells: List[Cell] = []
    current: Optional[Cell] = target_cell
    while current is not None:
        cells.append(current)
        current = came_from[current]
    cells.reverse()

    points: List[Point] = [(start.x, start.y)]
    points.extend(_cell_center(cell) for cell in cells[1:])
    if points[-1] != (target.x, target.y):
        if not _segment_is_clear(points[-1], (target.x, target.y), obstacle_list):
            return []
        points.append((target.x, target.y))
    return [Pose(x, y, target.theta if (x, y) == (target.x, target.y) else 0.0) for x, y in points]


@dataclass
class RouteEngine:
    start_pose: Pose
    obstacles: List[Obstacle]

    def __post_init__(self) -> None:
        self.obstacles = list(self.obstacles)
        self.route_order: List[int] = []
        self.route_targets: List[Pose] = []
        self.route_paths: List[List[Pose]] = []
        self.waypoints: List[Pose] = []
        self.waypoint_target_indices: List[int] = []
        self.waypoint_index = 0
        self.route_index = 0
        self.pose = Pose(self.start_pose.x, self.start_pose.y,
                         self.start_pose.theta)
        self.visited_ids: List[int] = []
        self.completed_targets = 0
        self.running = False
        self.paused = False
        self.total_distance = 0.0
        self.plan()

    def plan(self) -> None:
        remaining = list(self.obstacles)
        current = self.start_pose
        while remaining:
            candidates = []
            for obstacle in remaining:
                target = viewing_target(obstacle)
                path = find_safe_path(current, target, self.obstacles)
                if path:
                    cost = sum(math.hypot(b.x - a.x, b.y - a.y)
                               for a, b in zip(path, path[1:]))
                    candidates.append((cost, obstacle, target, path))
            if not candidates:
                raise ValueError("No collision-free B.2 route exists")
            cost, obstacle, target, path = min(
                candidates, key=lambda item: item[0])
            self.route_order.append(obstacle.obstacle_id)
            self.route_targets.append(target)
            self.route_paths.append(path)
            self.total_distance += cost
            current = target
            remaining.remove(obstacle)

        for target_index, path in enumerate(self.route_paths):
            self.waypoints.extend(path[1:])
            self.waypoint_target_indices.extend(
                [target_index] * (len(path) - 1))

    def reset(self) -> None:
        self.running = False
        self.paused = False
        self.waypoint_index = 0
        self.route_index = 0
        self.pose = Pose(self.start_pose.x, self.start_pose.y,
                         self.start_pose.theta)
        self.visited_ids = []
        self.completed_targets = 0

    def toggle(self) -> None:
        if self.running:
            self.paused = not self.paused
            return
        if self.completed_targets == len(self.route_targets):
            self.reset()
        self.running = True
        self.paused = False

    def step(self) -> Pose:
        if not self.running or self.paused or self.waypoint_index >= len(self.waypoints):
            if self.waypoint_index >= len(self.waypoints):
                self.running = False
            return self.pose

        waypoint = self.waypoints[self.waypoint_index]
        next_pose = planned_step(self.pose, waypoint)
        if next_pose.x == waypoint.x and next_pose.y == waypoint.y:
            self.pose = next_pose
            target_index = self.waypoint_target_indices[self.waypoint_index]
            self.waypoint_index += 1
            segment_finished = (
                self.waypoint_index == len(self.waypoints)
                or self.waypoint_target_indices[self.waypoint_index] != target_index
            )
            if segment_finished and target_index == self.route_index:
                target = self.route_targets[target_index]
                self.pose = Pose(target.x, target.y, target.theta)
                self.visited_ids.append(self.route_order[target_index])
                self.completed_targets += 1
                self.route_index += 1
        else:
            self.pose = next_pose
        if self.waypoint_index >= len(self.waypoints):
            self.running = False
        return self.pose

    @property
    def finished(self) -> bool:
        return self.completed_targets == len(self.route_targets) and not self.running
