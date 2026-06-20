"""Fleet occupancy and goal-reservation policy for local collision avoidance."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Iterable

from resilient_amr_dispatch.warehouse_map import GridDimensions, GridPoint


@dataclass(frozen=True)
class PeerState:
    robot_id: str
    position: tuple[float, float]
    lifecycle: str
    goal: tuple[float, float] | None
    timestamp: float

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "PeerState":
        goal_payload = payload.get("goal")
        goal = None
        if isinstance(goal_payload, dict):
            goal = (float(goal_payload["x"]), float(goal_payload["y"]))
        return cls(
            robot_id=str(payload["robot_id"]),
            position=(float(payload["x"]), float(payload["y"])),
            lifecycle=str(payload["state"]),
            goal=goal,
            timestamp=float(payload["timestamp"]),
        )


def fleet_blocked_cells(
    peers: Iterable[PeerState],
    dimensions: GridDimensions | None = None,
    now: float | None = None,
    stale_after: float = 2.0,
    safety_radius: int = 1,
) -> set[GridPoint]:
    """Reserve current peer positions and their assigned destination cells."""
    if safety_radius < 0:
        raise ValueError("safety_radius must not be negative")
    grid = dimensions or GridDimensions()
    current_time = time.time() if now is None else now
    blocked: set[GridPoint] = set()
    for peer in peers:
        if current_time - peer.timestamp > stale_after:
            continue
        if peer.lifecycle != "idle":
            blocked.update(_safety_zone(peer.position, grid, safety_radius))
        if peer.goal is not None:
            blocked.update(_safety_zone(peer.goal, grid, safety_radius))
    return blocked


def path_cells(
    position: tuple[float, float],
    waypoints: Iterable[tuple[float, float]],
) -> set[GridPoint]:
    """Rasterize a waypoint path densely enough for grid collision checks."""
    cells: set[GridPoint] = set()
    start = position
    for end in waypoints:
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, math.ceil(distance * 2.0))
        for index in range(steps + 1):
            fraction = index / steps
            cells.add(
                (
                    round(start[0] + fraction * (end[0] - start[0])),
                    round(start[1] + fraction * (end[1] - start[1])),
                )
            )
        start = end
    return cells


def path_conflicts(
    position: tuple[float, float],
    waypoints: Iterable[tuple[float, float]],
    blocked: set[GridPoint],
) -> bool:
    return bool(path_cells(position, waypoints) & blocked)


def _safety_zone(
    position: tuple[float, float], dimensions: GridDimensions, radius: int
) -> set[GridPoint]:
    center_x = round(position[0])
    center_y = round(position[1])
    return {
        (x, y)
        for x in range(center_x - radius, center_x + radius + 1)
        for y in range(center_y - radius, center_y + radius + 1)
        if 0 <= x < dimensions.width and 0 <= y < dimensions.height
    }
