"""Deterministic Phase 1 warehouse mission generation."""

from dataclasses import dataclass


MIN_ROBOTS = 6
MAX_ROBOTS = 12


@dataclass(frozen=True)
class Mission:
    mission_id: str
    robot_id: str
    start_x: float
    start_y: float
    goal_x: float
    goal_y: float

    def as_order(self) -> dict[str, object]:
        return {
            "mission_id": self.mission_id,
            "robot_id": self.robot_id,
            "start": {"x": self.start_x, "y": self.start_y},
            "goal": {"x": self.goal_x, "y": self.goal_y},
            "priority": "normal",
        }


def create_missions(robot_count: int) -> list[Mission]:
    """Create evenly spaced, left-to-right missions for the baseline demo."""
    if not MIN_ROBOTS <= robot_count <= MAX_ROBOTS:
        raise ValueError(
            f"robot_count must be between {MIN_ROBOTS} and {MAX_ROBOTS}"
        )

    spacing = 80.0 / (robot_count - 1)
    missions = []
    for index in range(robot_count):
        y = 10.0 + index * spacing
        number = index + 1
        missions.append(
            Mission(
                mission_id=f"mission_{number:03d}",
                robot_id=f"amr_{number:02d}",
                start_x=10.0,
                start_y=y,
                goal_x=90.0,
                goal_y=y,
            )
        )
    return missions
