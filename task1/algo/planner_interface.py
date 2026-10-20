import heapq
import math
from functools import lru_cache
from typing import Callable, Optional

from config import PlannerConfig
from footprint import pose_is_safe
from models import MotionCommand, Obstacle, PathResult, Pose

CarPathPlanner = Callable[[Pose, Pose,
                           list[Obstacle], PlannerConfig], PathResult]
HEADING_BINS = 32
LATTICE_STEP_CM = 5.0
STEERING_LEVELS = (-1.0, -0.5, 0.0, 0.5, 1.0)


def _normalise(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _mod2pi(angle: float) -> float:
    return angle % (2 * math.pi)


def _safe(pose: Pose, obstacles: tuple[Obstacle, ...], config: PlannerConfig) -> bool:
    return pose_is_safe(pose, obstacles, config)


def _move(pose: Pose, gear: int, steer: int, distance: float, radius: float) -> Pose:
    signed_distance = gear * distance
    if steer == 0:
        return Pose(
            pose.x + signed_distance * math.cos(pose.heading),
            pose.y + signed_distance * math.sin(pose.heading),
            pose.heading,
        )
    delta = signed_distance * steer / radius
    heading = _normalise(pose.heading + delta)
    signed_radius = radius / steer
    return Pose(
        pose.x + signed_radius * (math.sin(heading) - math.sin(pose.heading)),
        pose.y - signed_radius * (math.cos(heading) - math.cos(pose.heading)),
        heading,
    )


def _sample(start, gear, steer, distance, obstacles, config) -> Optional[list[Pose]]:
    count = max(1, math.ceil(distance / config.collision_check_step_cm))
    poses = [
        _move(start, gear, steer, distance * index /
              count, config.minimum_turning_radius_cm)
        for index in range(1, count + 1)
    ]
    return poses if all(_safe(pose, obstacles, config) for pose in poses) else None


def _dubins_words(alpha: float, beta: float, distance: float):
    sa, sb, ca, cb = math.sin(alpha), math.sin(
        beta), math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    p2 = 2 + distance * distance - 2 * cab + 2 * distance * (sa - sb)
    if p2 >= 0:
        tmp = math.atan2(cb - ca, distance + sa - sb)
        yield "LSL", (_mod2pi(-alpha + tmp), math.sqrt(p2), _mod2pi(beta - tmp))
    p2 = 2 + distance * distance - 2 * cab + 2 * distance * (sb - sa)
    if p2 >= 0:
        tmp = math.atan2(ca - cb, distance - sa + sb)
        yield "RSR", (_mod2pi(alpha - tmp), math.sqrt(p2), _mod2pi(-beta + tmp))
    p2 = -2 + distance * distance + 2 * cab + 2 * distance * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, distance + sa + sb) - math.atan2(-2.0, p)
        yield "LSR", (_mod2pi(-alpha + tmp), p, _mod2pi(-beta + tmp))
    p2 = distance * distance - 2 + 2 * cab - 2 * distance * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, distance - sa - sb) - math.atan2(2.0, p)
        yield "RSL", (_mod2pi(alpha - tmp), p, _mod2pi(beta - tmp))
    value = (6 - distance * distance + 2 * cab + 2 * distance * (sa - sb)) / 8
    if abs(value) <= 1:
        p = _mod2pi(2 * math.pi - math.acos(value))
        t = _mod2pi(alpha - math.atan2(ca - cb, distance - sa + sb) + p / 2)
        yield "RLR", (t, p, _mod2pi(alpha - beta - t + p))
    value = (6 - distance * distance + 2 * cab + 2 * distance * (-sa + sb)) / 8
    if abs(value) <= 1:
        p = _mod2pi(2 * math.pi - math.acos(value))
        t = _mod2pi(-alpha - math.atan2(ca - cb, distance + sa - sb) + p / 2)
        yield "LRL", (t, p, _mod2pi(beta - alpha - t + p))


def _dubins_candidates(start: Pose, goal: Pose, radius: float):
    dx, dy = goal.x - start.x, goal.y - start.y
    theta = math.atan2(dy, dx)
    distance = math.hypot(dx, dy) / radius
    yield from _dubins_words(
        _mod2pi(start.heading - theta), _mod2pi(goal.heading - theta), distance
    )


def _command(gear: int, steer: float) -> str:
    turn = "L" if steer > 0 else ("R" if steer < 0 else "")
    return ("F" if gear > 0 else "B") + turn


def _motion_time(gear: int, steer: int, distance: float, config: PlannerConfig) -> float:
    speed = config.turning_speed_cm_s if steer else (
        config.forward_speed_cm_s if gear > 0 else config.reverse_speed_cm_s
    )
    return distance / speed * (config.reverse_penalty if gear < 0 else 1)


def _connector(start, goal, obstacles, config):
    candidates = []
    for gear in (1, -1):
        transformed_start = start if gear > 0 else Pose(
            start.x, start.y, _normalise(start.heading + math.pi))
        transformed_goal = goal if gear > 0 else Pose(
            goal.x, goal.y, _normalise(goal.heading + math.pi))
        for word, values in _dubins_candidates(
            transformed_start, transformed_goal, config.minimum_turning_radius_cm
        ):
            candidates.append((sum(values), gear, word, values))
    for _, gear, word, values in sorted(candidates):
        pose, path, commands = start, [start], []
        total_distance = total_time = 0.0
        for letter, value in zip(word, values):
            distance = value * config.minimum_turning_radius_cm
            if distance <= 1e-8:
                continue
            steer = 0 if letter == "S" else (1 if letter == "L" else -1)
            if gear < 0:
                steer = -steer
            sampled = _sample(pose, gear, steer, distance, obstacles, config)
            if sampled is None:
                break
            path.extend(sampled)
            pose = sampled[-1]
            commands.append(MotionCommand(_command(gear, steer), distance))
            total_distance += distance
            total_time += _motion_time(gear, steer, distance, config)
        else:
            if math.hypot(pose.x - goal.x, pose.y - goal.y) < 1e-4 and abs(_normalise(pose.heading - goal.heading)) < 1e-4:
                path[-1] = goal
                return path, commands, total_distance, total_time
    return None


def _key(pose: Pose):
    heading = round(_mod2pi(pose.heading) / (2 * math.pi)
                    * HEADING_BINS) % HEADING_BINS
    return round(pose.x / LATTICE_STEP_CM), round(pose.y / LATTICE_STEP_CM), heading


def _escape_from_constrained_goal(start, goal, obstacles, config):
    """Grow reversible primitives out of a constrained goal and join from start."""
    primitive = config.minimum_turning_radius_cm * 2 * math.pi / HEADING_BINS
    root = _key(goal)
    poses, parents, actions = {root: goal}, {root: None}, {}
    queue = [(0, 0, root)]
    serial = 0
    for expansion in range(2500):
        if not queue:
            break
        depth, _, key = heapq.heappop(queue)
        pose = poses[key]
        if depth and expansion % 3 == 0:
            approach = _connector(start, pose, obstacles, config)
            if approach:
                chain, current = [], key
                while parents[current] is not None:
                    chain.append((current, actions[current]))
                    current = parents[current]
                path, commands, distance, travel_time = approach
                current_pose = pose
                for _, (gear, steer) in chain:
                    inverse_gear = -gear
                    sampled = _sample(current_pose, inverse_gear,
                                      steer, primitive, obstacles, config)
                    if not sampled:
                        break
                    path.extend(sampled)
                    current_pose = sampled[-1]
                    commands.append(MotionCommand(
                        _command(inverse_gear, steer), primitive))
                    distance += primitive
                    travel_time += _motion_time(inverse_gear,
                                                steer, primitive, config)
                else:
                    if math.hypot(current_pose.x - goal.x, current_pose.y - goal.y) < 1e-4 and abs(_normalise(current_pose.heading - goal.heading)) < 1e-4:
                        path[-1] = goal
                        return path, commands, distance, travel_time
        if depth >= 14:
            continue
        for gear in (1, -1):
            for steer in STEERING_LEVELS:
                sampled = _sample(pose, gear, steer,
                                  primitive, obstacles, config)
                if not sampled:
                    continue
                candidate, candidate_key = sampled[-1], _key(sampled[-1])
                if candidate_key in parents:
                    continue
                poses[candidate_key], parents[candidate_key] = candidate, key
                actions[candidate_key] = (gear, steer)
                serial += 1
                heapq.heappush(queue, (depth + 1, serial, candidate_key))
    return None


@lru_cache(maxsize=512)
def _plan_cached(start: Pose, goal: Pose, obstacles: tuple[Obstacle, ...], config: PlannerConfig) -> PathResult:
    if not _safe(start, obstacles, config) or not _safe(goal, obstacles, config):
        return PathResult(is_valid=False)
    direct = _connector(start, goal, obstacles, config)
    if direct:
        return PathResult(*direct, True)
    escaped = _escape_from_constrained_goal(start, goal, obstacles, config)
    if escaped:
        return PathResult(*escaped, True)

    primitive_distance = config.minimum_turning_radius_cm * 2 * math.pi / HEADING_BINS
    start_key = _key(start)
    poses, costs = {start_key: start}, {start_key: 0.0}
    parents = {start_key: None}
    actions = {}
    queue = [(0.0, 0.0, 0, start_key)]
    serial = 0
    found = connector = None
    for expansion in range(120):
        if not queue:
            break
        _, cost, _, key = heapq.heappop(queue)
        if cost != costs.get(key):
            continue
        pose = poses[key]
        if math.hypot(goal.x - pose.x, goal.y - pose.y) <= 55:
            connector = _connector(pose, goal, obstacles, config)
            if connector:
                found = key
                break
        for gear in (1, -1):
            for steer in STEERING_LEVELS:
                sampled = _sample(pose, gear, steer,
                                  primitive_distance, obstacles, config)
                if not sampled:
                    continue
                candidate, candidate_key = sampled[-1], _key(sampled[-1])
                if candidate_key == key:
                    continue
                move_cost = _motion_time(
                    gear, steer, primitive_distance, config)
                next_cost = cost + move_cost
                if next_cost >= costs.get(candidate_key, math.inf):
                    continue
                costs[candidate_key], poses[candidate_key] = next_cost, candidate
                parents[candidate_key], actions[candidate_key] = key, (
                    gear, steer)
                distance = math.hypot(
                    goal.x - candidate.x, goal.y - candidate.y)
                angle = abs(_normalise(goal.heading - candidate.heading))
                heuristic = distance / \
                    max(config.forward_speed_cm_s, config.reverse_speed_cm_s)
                heuristic += angle * config.minimum_turning_radius_cm / config.turning_speed_cm_s
                serial += 1
                heapq.heappush(queue, (next_cost + heuristic,
                               next_cost, serial, candidate_key))
    if found is None or connector is None:
        return PathResult(is_valid=False)

    keys, current = [], found
    while current is not None:
        keys.append(current)
        current = parents[current]
    keys.reverse()
    path, commands, pose = [start], [], start
    total_distance = total_time = 0.0
    for key in keys[1:]:
        gear, steer = actions[key]
        sampled = _sample(pose, gear, steer,
                          primitive_distance, obstacles, config)
        path.extend(sampled or [])
        pose = path[-1]
        commands.append(MotionCommand(
            _command(gear, steer), primitive_distance))
        total_distance += primitive_distance
        total_time += _motion_time(gear, steer, primitive_distance, config)
    connector_path, connector_commands, connector_distance, connector_time = connector
    path.extend(connector_path[1:])
    commands.extend(connector_commands)
    return PathResult(path, commands, total_distance + connector_distance, total_time + connector_time, True)


def plan_car_path(
    start_pose: Pose,
    target_pose: Pose,
    obstacles: list[Obstacle],
    config: PlannerConfig,
) -> PathResult:
    return _plan_cached(start_pose, target_pose, tuple(obstacles), config)
