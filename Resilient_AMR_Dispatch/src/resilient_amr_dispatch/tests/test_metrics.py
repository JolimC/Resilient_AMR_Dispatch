from resilient_amr_dispatch.metrics import FleetMetrics


def robot_state(
    robot_id: str,
    state: str,
    timestamp: float,
    affected: bool = False,
) -> dict[str, object]:
    return {
        "robot_id": robot_id,
        "mission_id": f"mission_{robot_id}",
        "state": state,
        "timestamp": timestamp,
        "mission_started_at": 100.0,
        "nominal_duration": 5.0,
        "affected_hazard_ids": ["spill_001"] if affected else [],
    }


def test_final_metrics_include_hazard_delay_and_recovery() -> None:
    metrics = FleetMetrics(expected_robots=2)
    metrics.observe_hazard({"hazard_id": "spill_001"})
    metrics.observe_exception(
        {
            "robot_id": "amr_01",
            "mission_id": "mission_amr_01",
            "hazard_id": "spill_001",
            "event": "local_reroute",
        }
    )
    metrics.observe_state(robot_state("amr_01", "executing", 102.0, True), 1.0)
    metrics.observe_state(robot_state("amr_02", "executing", 102.0), 1.0)
    metrics.observe_state(robot_state("amr_01", "completed", 108.0, True), 2.0)
    metrics.observe_state(robot_state("amr_02", "completed", 105.0), 2.0)

    summary = metrics.snapshot()

    assert summary == {
        "missions_assigned": 2,
        "missions_completed": 2,
        "hazards_injected": 1,
        "local_reroutes": 1,
        "escalations": 0,
        "average_hazard_delay_seconds": 3.0,
        "stale_telemetry_alerts": 0,
        "final": True,
    }


def test_stale_telemetry_alert_counts_once_per_stale_episode() -> None:
    metrics = FleetMetrics(expected_robots=1)
    state = robot_state("amr_01", "executing", 100.0)
    metrics.observe_state(state, received_at=1.0)

    assert metrics.check_stale(now=3.0, threshold=1.5) == ["amr_01"]
    assert metrics.check_stale(now=4.0, threshold=1.5) == []
    assert metrics.snapshot()["stale_telemetry_alerts"] == 1

    metrics.observe_state(state, received_at=5.0)
    assert metrics.check_stale(now=7.0, threshold=1.5) == ["amr_01"]
    assert metrics.snapshot()["stale_telemetry_alerts"] == 2


def test_blocked_robot_is_counted_as_escalation_and_terminal() -> None:
    metrics = FleetMetrics(expected_robots=1)
    metrics.observe_state(robot_state("amr_01", "blocked", 105.0, True), 1.0)

    summary = metrics.snapshot()

    assert summary["escalations"] == 1
    assert summary["final"] is True
