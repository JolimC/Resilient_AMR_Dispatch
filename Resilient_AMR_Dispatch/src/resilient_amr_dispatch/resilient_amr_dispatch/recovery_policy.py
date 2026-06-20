"""Bounded and explainable local recovery policy."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Iterable

from resilient_amr_dispatch.hazards import Hazard
from resilient_amr_dispatch.warehouse_map import WarehouseMap, simplify_grid_path
from resilient_amr_dispatch.warehouse_map import GridPoint


@dataclass(frozen=True)
class RecoveryResult:
    status: str
    waypoints: tuple[tuple[float, float], ...]
    reason: str


def build_recovery_exception(
    robot_id: str,
    mission_id: str,
    hazard: Hazard,
    result: RecoveryResult,
    timestamp: float | None = None,
) -> dict[str, Any]:
    succeeded = result.status == "rerouting"
    return {
        "robot_id": robot_id,
        "mission_id": mission_id,
        "event": "local_reroute" if succeeded else "local_replan_failed",
        "reason": hazard.hazard_type,
        "hazard_id": hazard.hazard_id,
        "action": (
            "replanned_local_path" if succeeded else "escalated_to_dispatch"
        ),
        "waypoint_count": len(result.waypoints),
        "timestamp": time.time() if timestamp is None else timestamp,
    }


def plan_local_recovery(
    warehouse: WarehouseMap,
    start: tuple[float, float],
    goal: tuple[float, float],
    hazards: Iterable[Hazard],
    dynamic_blocked: Iterable[GridPoint] = (),
) -> RecoveryResult:
    """Plan around all known blocked hazards while retaining the mission goal."""
    recovery_map = warehouse.copy()
    recovery_map.add_hazards(hazards, clearance=1)
    recovery_map.add_blocked(dynamic_blocked)
    start_cell = recovery_map.to_grid(start)
    goal_cell = recovery_map.to_grid(goal)
    grid_path = recovery_map.plan(start_cell, goal_cell)
    if grid_path is None:
        return RecoveryResult(
            status="blocked",
            waypoints=(),
            reason="no_collision_free_route",
        )

    simplified = simplify_grid_path(grid_path)
    waypoints = [(float(x), float(y)) for x, y in simplified[1:]]
    if not waypoints or waypoints[-1] != goal:
        waypoints.append(goal)
    return RecoveryResult(
        status="rerouting",
        waypoints=tuple(waypoints),
        reason="local_astar_route_found",
    )


def advance_along_path(
    position: tuple[float, float],
    waypoints: Iterable[tuple[float, float]],
    distance_budget: float,
) -> tuple[tuple[float, float], tuple[tuple[float, float], ...]]:
    """Move through as many waypoints as fit within one simulation tick."""
    if distance_budget < 0.0:
        raise ValueError("distance_budget must not be negative")
    current_x, current_y = position
    remaining = list(waypoints)
    budget = distance_budget
    while remaining and budget > 0.0:
        target_x, target_y = remaining[0]
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        if distance <= budget:
            current_x = target_x
            current_y = target_y
            budget -= distance
            remaining.pop(0)
        else:
            current_x += budget * dx / distance
            current_y += budget * dy / distance
            budget = 0.0
    return (current_x, current_y), tuple(remaining)


def has_cleared_hazard(
    position: tuple[float, float], goal: tuple[float, float], hazard: Hazard
) -> bool:
    """Return when the robot has progressed beyond a hazard toward its goal."""
    center_x = (hazard.bounds.x_min + hazard.bounds.x_max) / 2.0
    if goal[0] >= center_x:
        return position[0] > hazard.bounds.x_max + 1.0
    return position[0] < hazard.bounds.x_min - 1.0
