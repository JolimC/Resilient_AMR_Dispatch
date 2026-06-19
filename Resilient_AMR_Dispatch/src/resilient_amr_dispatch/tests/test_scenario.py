import pytest

from resilient_amr_dispatch.scenario import create_missions


def test_create_missions_has_unique_ids_and_valid_bounds() -> None:
    missions = create_missions(8)

    assert len(missions) == 8
    assert len({mission.robot_id for mission in missions}) == 8
    assert len({mission.mission_id for mission in missions}) == 8
    assert all(0.0 <= mission.start_y <= 100.0 for mission in missions)
    assert all(mission.start_x < mission.goal_x for mission in missions)


@pytest.mark.parametrize("robot_count", [5, 13])
def test_create_missions_rejects_robot_count_outside_demo_range(
    robot_count: int,
) -> None:
    with pytest.raises(ValueError):
        create_missions(robot_count)
