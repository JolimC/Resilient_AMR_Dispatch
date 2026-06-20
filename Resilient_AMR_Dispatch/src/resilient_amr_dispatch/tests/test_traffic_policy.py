from resilient_amr_dispatch.traffic_policy import (
    PeerState,
    fleet_blocked_cells,
    path_conflicts,
)
from resilient_amr_dispatch.hazards import PHASE_3_SPILL
from resilient_amr_dispatch.recovery_policy import plan_local_recovery
from resilient_amr_dispatch.scenario import create_missions
from resilient_amr_dispatch.warehouse_map import WarehouseMap


def test_completed_robot_and_goal_have_safety_zones() -> None:
    peer = PeerState(
        robot_id="amr_03",
        position=(90.0, 33.0),
        lifecycle="completed",
        goal=(90.0, 33.0),
        timestamp=100.0,
    )

    blocked = fleet_blocked_cells([peer], now=100.5, safety_radius=1)

    assert (90, 33) in blocked
    assert (89, 32) in blocked
    assert (88, 33) not in blocked


def test_other_goal_conflicts_with_route_crossing_destination() -> None:
    peer = PeerState("amr_03", (70.0, 33.0), "executing", (90.0, 33.0), 100.0)
    blocked = fleet_blocked_cells([peer], now=100.5)

    assert path_conflicts((90.0, 28.0), [(90.0, 44.0)], blocked)


def test_stale_peer_is_not_a_dynamic_obstacle() -> None:
    peer = PeerState("amr_03", (50.0, 50.0), "executing", None, 90.0)

    assert fleet_blocked_cells([peer], now=100.0, stale_after=2.0) == set()


def test_recovery_route_avoids_other_robot_destinations() -> None:
    missions = create_missions(8)
    mission = missions[3]
    peers = [
        PeerState(
            robot_id=other.robot_id,
            position=(20.0, other.start_y),
            lifecycle="executing",
            goal=(other.goal_x, other.goal_y),
            timestamp=100.0,
        )
        for other in missions
        if other.robot_id != mission.robot_id
    ]
    blocked = fleet_blocked_cells(peers, now=100.5)

    result = plan_local_recovery(
        WarehouseMap(),
        start=(20.0, mission.start_y),
        goal=(mission.goal_x, mission.goal_y),
        hazards=[PHASE_3_SPILL],
        dynamic_blocked=blocked,
    )

    assert result.status == "rerouting"
    assert not path_conflicts((20.0, mission.start_y), result.waypoints, blocked)
