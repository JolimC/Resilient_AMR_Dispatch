import pytest

from resilient_amr_dispatch.hazards import Bounds, Hazard, PHASE_3_SPILL
from resilient_amr_dispatch.scenario import create_missions


@pytest.mark.parametrize(
    ("start", "end", "expected"),
    [
        ((10.0, 50.0), (90.0, 50.0), True),
        ((10.0, 10.0), (90.0, 10.0), False),
        ((50.0, 20.0), (50.0, 80.0), True),
        ((0.0, 0.0), (20.0, 20.0), False),
        ((45.0, 30.0), (45.0, 30.0), True),
    ],
)
def test_segment_intersection(
    start: tuple[float, float], end: tuple[float, float], expected: bool
) -> None:
    assert PHASE_3_SPILL.bounds.intersects_segment(start, end) is expected


def test_hazard_payload_round_trip() -> None:
    payload = PHASE_3_SPILL.as_payload(timestamp=123.0)

    assert Hazard.from_payload(payload) == PHASE_3_SPILL
    assert payload["timestamp"] == 123.0


def test_bounds_reject_inverted_range() -> None:
    with pytest.raises(ValueError):
        Bounds(x_min=5.0, x_max=4.0, y_min=0.0, y_max=1.0)


def test_phase_3_spill_intersects_active_demo_paths() -> None:
    affected = [
        mission
        for mission in create_missions(8)
        if PHASE_3_SPILL.bounds.intersects_segment(
            (mission.start_x, mission.start_y),
            (mission.goal_x, mission.goal_y),
        )
    ]

    assert len(affected) >= 1
