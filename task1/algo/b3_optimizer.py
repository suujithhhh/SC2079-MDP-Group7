import itertools
import math
from dataclasses import dataclass, field
from typing import List

from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from models import Obstacle, PathResult, Pose
from planner_interface import CarPathPlanner, plan_car_path
from targets import viewing_target, viewing_targets


@dataclass
class B3RouteResult:
    obstacle_order: List[int] = field(default_factory=list)
    target_poses: List[Pose] = field(default_factory=list)
    leg_results: List[PathResult] = field(default_factory=list)
    estimated_time_s: float = 0.0
    is_valid: bool = False
    orders_considered: int = 0
    valid_orders: int = 0
    rejected_orders: int = 0
    failure_reason: str = ""
    is_complete: bool = True
    skipped_obstacle_ids: List[int] = field(default_factory=list)


def _plan_order(
    start_pose: Pose,
    order: tuple[Obstacle, ...],
    obstacles: list[Obstacle],
    config: PlannerConfig,
    planner: CarPathPlanner,
) -> tuple[List[Pose], List[PathResult], bool]:
    current = start_pose
    targets: List[Pose] = []
    legs: List[PathResult] = []
    valid = True
    for obstacle in order:
        target = viewing_target(obstacle, config)
        result = planner(current, target, obstacles, config)
        targets.append(target)
        legs.append(result)
        valid = valid and result.is_valid
        current = target
    return targets, legs, valid


def find_b2_route(
    start_pose: Pose,
    obstacles: list[Obstacle],
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
    planner: CarPathPlanner = plan_car_path,
    allow_alternative_viewing: bool = False,
) -> B3RouteResult:
    result = B3RouteResult()
    rejected = 0
    for order in itertools.permutations(obstacles):
        result.orders_considered += 1
        targets, legs, valid = _plan_order(
            start_pose, order, list(obstacles), config, planner
        )
        if not valid:
            rejected += 1
            continue
        return B3RouteResult(
            obstacle_order=[item.obstacle_id for item in order],
            target_poses=targets,
            leg_results=legs,
            estimated_time_s=sum(leg.estimated_time_s for leg in legs),
            is_valid=True,
            orders_considered=result.orders_considered,
            valid_orders=1,
            rejected_orders=rejected,
        )
    result.rejected_orders = rejected
    result.failure_reason = (
        "No valid B2 route visits all five targets with continuous car motion."
    )
    if allow_alternative_viewing:
        return _find_b2_with_viewing_candidates(start_pose, obstacles, config, planner, result)
    return result


def _find_b2_with_viewing_candidates(
    start_pose: Pose,
    obstacles: list[Obstacle],
    config: PlannerConfig,
    planner: CarPathPlanner,
    primary_result: B3RouteResult,
) -> B3RouteResult:
    candidates = {
        obstacle.obstacle_id: viewing_targets(obstacle, config)
        for obstacle in obstacles
    }
    leg_cache: dict[tuple[Pose, Pose], PathResult] = {}

    def leg(start: Pose, target: Pose) -> PathResult:
        key = (start, target)
        if key not in leg_cache:
            leg_cache[key] = planner(start, target, list(obstacles), config)
        return leg_cache[key]

    rejected = primary_result.rejected_orders
    best_partial: B3RouteResult | None = None
    for order in itertools.permutations(obstacles):
        states: dict[Pose, tuple[float, List[Pose], List[PathResult]]] = {}
        for target in candidates[order[0].obstacle_id]:
            result = leg(start_pose, target)
            if result.is_valid:
                states[target] = (result.estimated_time_s, [target], [result])
        if states:
            partial_time, partial_targets, partial_legs = min(
                states.values(), key=lambda item: item[0]
            )
            best_partial = _choose_partial(
                best_partial,
                obstacles,
                [order[0].obstacle_id],
                partial_targets,
                partial_legs,
                partial_time,
            )
        for obstacle in order[1:]:
            next_states: dict[Pose, tuple[float,
                                          List[Pose], List[PathResult]]] = {}
            for target in candidates[obstacle.obstacle_id]:
                for previous, (cost, targets, legs) in states.items():
                    result = leg(previous, target)
                    if not result.is_valid:
                        continue
                    proposed = cost + result.estimated_time_s
                    incumbent = next_states.get(target)
                    if incumbent is None or proposed < incumbent[0]:
                        next_states[target] = (
                            proposed, targets + [target], legs + [result])
            states = next_states
            if not states:
                break
            partial_time, partial_targets, partial_legs = min(
                states.values(), key=lambda item: item[0]
            )
            visited_count = len(partial_targets)
            if (
                best_partial is None
                or visited_count > len(best_partial.obstacle_order)
                or (
                    visited_count == len(best_partial.obstacle_order)
                    and partial_time < best_partial.estimated_time_s
                )
            ):
                visited_ids = [
                    item.obstacle_id for item in order[:visited_count]]
                best_partial = _partial_result(
                    obstacles, visited_ids, partial_targets, partial_legs, partial_time
                )
        if not states:
            rejected += 1
            continue
        elapsed, targets, legs = min(states.values(), key=lambda item: item[0])
        return B3RouteResult(
            obstacle_order=[item.obstacle_id for item in order],
            target_poses=targets,
            leg_results=legs,
            estimated_time_s=elapsed,
            is_valid=True,
            orders_considered=primary_result.orders_considered +
            rejected - primary_result.rejected_orders + 1,
            valid_orders=1,
            rejected_orders=rejected,
        )

    if best_partial is not None:
        best_partial.orders_considered = math.factorial(len(obstacles))
        best_partial.failure_reason = (
            f"No complete B2 route exists; running the best {len(best_partial.obstacle_order)}/"
            f"{len(obstacles)} target route."
        )
        return best_partial
    primary_result.failure_reason = "No obstacle can be reached with continuous car motion."
    return primary_result


def optimise_b3_route(
    start_pose: Pose,
    obstacles: list[Obstacle],
    config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
    planner: CarPathPlanner = plan_car_path,
    allow_alternative_viewing: bool = False,
) -> B3RouteResult:
    if allow_alternative_viewing:
        return _optimise_with_viewing_candidates(start_pose, obstacles, config, planner)

    best = B3RouteResult()
    orders_considered = 0
    invalid_legs = set()
    valid_orders = 0
    for order in itertools.permutations(obstacles):
        orders_considered += 1
        targets, legs, valid = _plan_order(
            start_pose, order, list(obstacles), config, planner
        )
        if not valid:
            current = start_pose
            for target, leg in zip(targets, legs):
                if not leg.is_valid:
                    invalid_legs.add((current, target))
                current = target
            continue
        valid_orders += 1
        total_time = sum(leg.estimated_time_s for leg in legs)
        if not best.is_valid or total_time < best.estimated_time_s:
            best = B3RouteResult(
                obstacle_order=[item.obstacle_id for item in order],
                target_poses=targets,
                leg_results=legs,
                estimated_time_s=total_time,
                is_valid=True,
            )
    best.orders_considered = orders_considered
    best.valid_orders = valid_orders
    best.rejected_orders = orders_considered - valid_orders
    if not best.is_valid:
        best.failure_reason = (
            "No complete five-target ordering exists; the continuous local planner "
            f"could not connect {len(invalid_legs)} directed route legs."
        )
    return best


def _optimise_with_viewing_candidates(
    start_pose: Pose,
    obstacles: list[Obstacle],
    config: PlannerConfig,
    planner: CarPathPlanner,
) -> B3RouteResult:
    primary_result = optimise_b3_route(
        start_pose, obstacles, config, planner, allow_alternative_viewing=False
    )
    if primary_result.is_valid:
        return primary_result

    candidates = {
        obstacle.obstacle_id: viewing_targets(obstacle, config)
        for obstacle in obstacles
    }
    leg_cache: dict[tuple[Pose, Pose], PathResult] = {}

    def leg(start: Pose, target: Pose) -> PathResult:
        key = (start, target)
        if key not in leg_cache:
            leg_cache[key] = planner(start, target, list(obstacles), config)
        return leg_cache[key]

    best = B3RouteResult()
    best_partial: B3RouteResult | None = None
    valid_orders = 0
    for order in itertools.permutations(obstacles):
        states: dict[Pose, tuple[float, List[Pose], List[PathResult]]] = {}
        for target in candidates[order[0].obstacle_id]:
            result = leg(start_pose, target)
            if result.is_valid:
                states[target] = (result.estimated_time_s, [target], [result])

        if states:
            partial_time, partial_targets, partial_legs = min(
                states.values(), key=lambda item: item[0]
            )
            best_partial = _choose_partial(
                best_partial,
                obstacles,
                [order[0].obstacle_id],
                partial_targets,
                partial_legs,
                partial_time,
            )

        for obstacle in order[1:]:
            next_states: dict[Pose, tuple[float,
                                          List[Pose], List[PathResult]]] = {}
            for target in candidates[obstacle.obstacle_id]:
                for previous, (cost, targets, legs) in states.items():
                    result = leg(previous, target)
                    if not result.is_valid:
                        continue
                    proposed = cost + result.estimated_time_s
                    incumbent = next_states.get(target)
                    if incumbent is None or proposed < incumbent[0]:
                        next_states[target] = (
                            proposed, targets + [target], legs + [result])
            states = next_states
            if not states:
                break
            partial_time, partial_targets, partial_legs = min(
                states.values(), key=lambda item: item[0]
            )
            visited_count = len(partial_targets)
            best_partial = _choose_partial(
                best_partial,
                obstacles,
                [item.obstacle_id for item in order[:visited_count]],
                partial_targets,
                partial_legs,
                partial_time,
            )

        if not states:
            continue
        valid_orders += 1
        total_time, targets, legs = min(
            states.values(), key=lambda item: item[0])
        if not best.is_valid or total_time < best.estimated_time_s:
            best = B3RouteResult(
                obstacle_order=[item.obstacle_id for item in order],
                target_poses=targets,
                leg_results=legs,
                estimated_time_s=total_time,
                is_valid=True,
            )

    best.orders_considered = math.factorial(len(obstacles))
    best.valid_orders = valid_orders
    best.rejected_orders = best.orders_considered - valid_orders
    if not best.is_valid:
        if best_partial is not None:
            best = best_partial
            best.orders_considered = math.factorial(len(obstacles))
            best.valid_orders = 0
            best.rejected_orders = best.orders_considered
            best.failure_reason = (
                f"No complete B3 Hamiltonian route exists; running the shortest "
                f"route covering {len(best.obstacle_order)}/{len(obstacles)} targets."
            )
        else:
            best.failure_reason = "No obstacle can be reached with continuous car motion."
    return best


def _partial_result(
    obstacles: list[Obstacle],
    visited_ids: List[int],
    targets: List[Pose],
    legs: List[PathResult],
    elapsed: float,
) -> B3RouteResult:
    visited = set(visited_ids)
    return B3RouteResult(
        obstacle_order=list(visited_ids),
        target_poses=list(targets),
        leg_results=list(legs),
        estimated_time_s=elapsed,
        is_valid=True,
        is_complete=False,
        skipped_obstacle_ids=[
            obstacle.obstacle_id for obstacle in obstacles
            if obstacle.obstacle_id not in visited
        ],
    )


def _choose_partial(
    current: B3RouteResult | None,
    obstacles: list[Obstacle],
    visited_ids: List[int],
    targets: List[Pose],
    legs: List[PathResult],
    elapsed: float,
) -> B3RouteResult:
    if (
        current is None
        or len(visited_ids) > len(current.obstacle_order)
        or (
            len(visited_ids) == len(current.obstacle_order)
            and elapsed < current.estimated_time_s
        )
    ):
        return _partial_result(obstacles, visited_ids, targets, legs, elapsed)
    return current
