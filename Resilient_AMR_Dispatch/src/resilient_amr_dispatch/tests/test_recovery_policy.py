import pytest

from resilient_amr_dispatch.hazards import Bounds, Hazard, PHASE_3_SPILL
from resilient_amr_dispatch.recovery_policy import (
    advance_along_path,
    build_recovery_exception,
    has_cleared_hazard,
    plan_local_recovery,
)
from resilient_amr_dispatch.scenario import create_missions
from resilient_amr_dispatch.warehouse_map import WarehouseMap


def test_local_recovery_retains_original_goal() -> None:
    result = plan_local_recovery(
        WarehouseMap(),
        start=(20.0, 44.0),
        goal=(90.0, 44.0),
        hazards=[PHASE_3_SPILL],
    )

    assert result.status == "rerouting"
    assert result.waypoints[-1] == (90.0, 44.0)
    assert len(result.waypoints) >= 3


def test_local_recovery_escalates_when_no_route_exists() -> None:
    barrier = Hazard("barrier", "blocked_aisle", Bounds(40, 60, 0, 99))

    result = plan_local_recovery(
        WarehouseMap(include_shelves=False),
        start=(10.0, 50.0),
        goal=(90.0, 50.0),
        hazards=[barrier],
    )

    assert result.status == "blocked"
    assert result.waypoints == ()


def test_advance_along_path_consumes_multiple_short_waypoints() -> None:
    position, remaining = advance_along_path(
        (0.0, 0.0), [(1.0, 0.0), (2.0, 0.0), (4.0, 0.0)], 2.5
    )

    assert position == pytest.approx((2.5, 0.0))
    assert remaining == ((4.0, 0.0),)


def test_hazard_clearance_uses_mission_direction() -> None:
    assert has_cleared_hazard((57.0, 29.0), (90.0, 50.0), PHASE_3_SPILL)
    assert not has_cleared_hazard((55.5, 29.0), (90.0, 50.0), PHASE_3_SPILL)


def test_recovery_exception_describes_local_reroute() -> None:
    result = plan_local_recovery(
        WarehouseMap(), (20.0, 44.0), (90.0, 44.0), [PHASE_3_SPILL]
    )

    payload = build_recovery_exception(
        "amr_01", "mission_001", PHASE_3_SPILL, result, timestamp=123.0
    )

    assert payload["event"] == "local_reroute"
    assert payload["action"] == "replanned_local_path"
    assert payload["hazard_id"] == "spill_001"
    assert payload["timestamp"] == 123.0


def test_all_affected_default_missions_can_finish_after_recovery() -> None:
    affected_missions = [
        mission
        for mission in create_missions(8)
        if PHASE_3_SPILL.bounds.intersects_segment(
            (mission.start_x, mission.start_y),
            (mission.goal_x, mission.goal_y),
        )
    ]

    assert affected_missions
    for mission in affected_missions:
        position = (20.0, mission.start_y)
        goal = (mission.goal_x, mission.goal_y)
        result = plan_local_recovery(
            WarehouseMap(), position, goal, [PHASE_3_SPILL]
        )
        remaining = result.waypoints
        for _ in range(200):
            position, remaining = advance_along_path(position, remaining, 1.2)
            if not remaining:
                break
        assert result.status == "rerouting"
        assert not remaining
        assert position == pytest.approx(goal)
