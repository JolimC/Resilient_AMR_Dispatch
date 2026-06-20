"""Pure fleet-metric aggregation used by the ROS monitor node."""

from __future__ import annotations

from dataclasses import dataclass
from statistics import mean
from typing import Any


@dataclass
class MissionRecord:
    robot_id: str
    mission_id: str
    last_seen: float
    state: str
    mission_started_at: float | None = None
    nominal_duration: float | None = None
    completed_at: float | None = None
    affected: bool = False


class FleetMetrics:
    def __init__(self, expected_robots: int) -> None:
        if expected_robots <= 0:
            raise ValueError("expected_robots must be positive")
        self.expected_robots = expected_robots
        self.missions: dict[tuple[str, str], MissionRecord] = {}
        self.hazard_ids: set[str] = set()
        self.reroute_keys: set[tuple[str, str, str]] = set()
        self.escalation_keys: set[tuple[str, str, str]] = set()
        self.stale_telemetry_alerts = 0
        self._currently_stale: set[str] = set()

    def observe_state(self, state: dict[str, Any], received_at: float) -> None:
        robot_id = str(state["robot_id"])
        mission_value = state.get("mission_id")
        if not mission_value:
            return
        mission_id = str(mission_value)
        key = (robot_id, mission_id)
        record = self.missions.get(key)
        if record is None:
            record = MissionRecord(
                robot_id=robot_id,
                mission_id=mission_id,
                last_seen=received_at,
                state=str(state["state"]),
            )
            self.missions[key] = record
        record.last_seen = received_at
        record.state = str(state["state"])
        self._currently_stale.discard(robot_id)

        if state.get("mission_started_at") is not None:
            record.mission_started_at = float(state["mission_started_at"])
        if state.get("nominal_duration") is not None:
            record.nominal_duration = float(state["nominal_duration"])
        if state.get("affected_hazard_ids") or state.get("path_affected"):
            record.affected = True
        hazard_id = state.get("active_hazard_id")
        if state.get("recovery_state") == "rerouting" and hazard_id:
            self.reroute_keys.add((robot_id, mission_id, str(hazard_id)))
        if record.state == "blocked":
            self.escalation_keys.add(
                (robot_id, mission_id, str(hazard_id or "unknown"))
            )
        if record.state == "completed":
            record.completed_at = float(state["timestamp"])

    def observe_hazard(self, payload: dict[str, Any]) -> None:
        self.hazard_ids.add(str(payload["hazard_id"]))

    def observe_exception(self, payload: dict[str, Any]) -> None:
        key = (
            str(payload.get("robot_id", "unknown")),
            str(payload.get("mission_id", "unknown")),
            str(payload.get("hazard_id", "unknown")),
        )
        if payload.get("event") == "local_reroute":
            self.reroute_keys.add(key)
        elif payload.get("event") == "local_replan_failed":
            self.escalation_keys.add(key)

    def check_stale(self, now: float, threshold: float) -> list[str]:
        newly_stale = []
        latest_by_robot: dict[str, MissionRecord] = {}
        for record in self.missions.values():
            current = latest_by_robot.get(record.robot_id)
            if current is None or record.last_seen > current.last_seen:
                latest_by_robot[record.robot_id] = record
        for robot_id, record in latest_by_robot.items():
            if record.state in ("completed", "blocked"):
                continue
            if now - record.last_seen > threshold and robot_id not in self._currently_stale:
                self._currently_stale.add(robot_id)
                self.stale_telemetry_alerts += 1
                newly_stale.append(robot_id)
        return newly_stale

    def snapshot(self) -> dict[str, Any]:
        completed = [record for record in self.missions.values() if record.state == "completed"]
        escalated_states = [
            record for record in self.missions.values() if record.state == "blocked"
        ]
        delays = []
        for record in completed:
            if (
                record.affected
                and record.completed_at is not None
                and record.mission_started_at is not None
                and record.nominal_duration is not None
            ):
                actual_duration = record.completed_at - record.mission_started_at
                delays.append(max(0.0, actual_duration - record.nominal_duration))

        terminal_count = len(completed) + len(escalated_states)
        return {
            "missions_assigned": len(self.missions),
            "missions_completed": len(completed),
            "hazards_injected": len(self.hazard_ids),
            "local_reroutes": len(self.reroute_keys),
            "escalations": max(len(self.escalation_keys), len(escalated_states)),
            "average_hazard_delay_seconds": round(mean(delays), 3) if delays else 0.0,
            "stale_telemetry_alerts": self.stale_telemetry_alerts,
            "final": terminal_count >= self.expected_robots,
        }
