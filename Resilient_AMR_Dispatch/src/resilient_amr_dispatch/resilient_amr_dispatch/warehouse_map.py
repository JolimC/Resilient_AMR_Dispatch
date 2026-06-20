"""Discrete warehouse map and deterministic A* grid planner."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Iterable

from resilient_amr_dispatch.hazards import Bounds, Hazard


GridPoint = tuple[int, int]

SHELF_BOUNDS = (
    Bounds(25.0, 43.0, 15.0, 18.0),
    Bounds(57.0, 75.0, 15.0, 18.0),
    Bounds(25.0, 43.0, 37.0, 40.0),
    Bounds(57.0, 75.0, 37.0, 40.0),
    Bounds(25.0, 43.0, 60.0, 63.0),
    Bounds(57.0, 75.0, 60.0, 63.0),
    Bounds(25.0, 43.0, 82.0, 85.0),
    Bounds(57.0, 75.0, 82.0, 85.0),
)


@dataclass(frozen=True)
class GridDimensions:
    width: int = 100
    height: int = 100

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid dimensions must be positive")


class WarehouseMap:
    """A 100 x 100 occupancy grid containing shelves and dynamic hazards."""

    def __init__(
        self,
        dimensions: GridDimensions | None = None,
        blocked: Iterable[GridPoint] | None = None,
        include_shelves: bool = True,
    ) -> None:
        self.dimensions = dimensions or GridDimensions()
        self.blocked = set(blocked or ())
        if include_shelves:
            for bounds in SHELF_BOUNDS:
                self.add_bounds(bounds)

    def copy(self) -> "WarehouseMap":
        return WarehouseMap(
            dimensions=self.dimensions,
            blocked=self.blocked,
            include_shelves=False,
        )

    def in_bounds(self, point: GridPoint) -> bool:
        return (
            0 <= point[0] < self.dimensions.width
            and 0 <= point[1] < self.dimensions.height
        )

    def is_free(self, point: GridPoint) -> bool:
        return self.in_bounds(point) and point not in self.blocked

    def add_bounds(self, bounds: Bounds, clearance: int = 0) -> None:
        if clearance < 0:
            raise ValueError("clearance must not be negative")
        x_start = max(0, math.floor(bounds.x_min) - clearance)
        x_end = min(self.dimensions.width - 1, math.ceil(bounds.x_max) + clearance)
        y_start = max(0, math.floor(bounds.y_min) - clearance)
        y_end = min(self.dimensions.height - 1, math.ceil(bounds.y_max) + clearance)
        for x in range(x_start, x_end + 1):
            for y in range(y_start, y_end + 1):
                self.blocked.add((x, y))

    def add_hazards(self, hazards: Iterable[Hazard], clearance: int = 1) -> None:
        for hazard in hazards:
            if hazard.severity == "blocked":
                self.add_bounds(hazard.bounds, clearance=clearance)

    def add_blocked(self, points: Iterable[GridPoint]) -> None:
        self.blocked.update(point for point in points if self.in_bounds(point))

    def to_grid(self, position: tuple[float, float]) -> GridPoint:
        point = (round(position[0]), round(position[1]))
        return (
            min(max(point[0], 0), self.dimensions.width - 1),
            min(max(point[1], 0), self.dimensions.height - 1),
        )

    def neighbors(self, point: GridPoint) -> tuple[GridPoint, ...]:
        x, y = point
        candidates = ((x + 1, y), (x, y + 1), (x, y - 1), (x - 1, y))
        return tuple(candidate for candidate in candidates if self.is_free(candidate))

    def plan(self, start: GridPoint, goal: GridPoint) -> list[GridPoint] | None:
        """Find a shortest collision-free path using Manhattan-distance A*."""
        if not self.is_free(start) or not self.is_free(goal):
            return None
        if start == goal:
            return [start]

        frontier: list[tuple[int, int, int, GridPoint]] = []
        sequence = itertools.count()
        heapq.heappush(frontier, (self._heuristic(start, goal), 0, next(sequence), start))
        came_from: dict[GridPoint, GridPoint] = {}
        cost_so_far = {start: 0}

        while frontier:
            _, current_cost, _, current = heapq.heappop(frontier)
            if current == goal:
                return self._reconstruct(came_from, start, goal)
            if current_cost != cost_so_far[current]:
                continue

            for neighbor in self.neighbors(current):
                new_cost = current_cost + 1
                if new_cost >= cost_so_far.get(neighbor, math.inf):
                    continue
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + self._heuristic(neighbor, goal)
                heapq.heappush(
                    frontier, (priority, new_cost, next(sequence), neighbor)
                )
        return None

    @staticmethod
    def _heuristic(first: GridPoint, second: GridPoint) -> int:
        return abs(first[0] - second[0]) + abs(first[1] - second[1])

    @staticmethod
    def _reconstruct(
        came_from: dict[GridPoint, GridPoint], start: GridPoint, goal: GridPoint
    ) -> list[GridPoint]:
        path = [goal]
        current = goal
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


def simplify_grid_path(path: list[GridPoint]) -> list[GridPoint]:
    """Keep only endpoints and direction-changing cells from a grid path."""
    if len(path) <= 2:
        return list(path)
    simplified = [path[0]]
    previous_direction = (
        path[1][0] - path[0][0],
        path[1][1] - path[0][1],
    )
    for index in range(1, len(path) - 1):
        next_direction = (
            path[index + 1][0] - path[index][0],
            path[index + 1][1] - path[index][1],
        )
        if next_direction != previous_direction:
            simplified.append(path[index])
        previous_direction = next_direction
    simplified.append(path[-1])
    return simplified
