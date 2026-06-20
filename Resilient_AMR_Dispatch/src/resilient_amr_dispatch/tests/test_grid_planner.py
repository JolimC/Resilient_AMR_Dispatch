from resilient_amr_dispatch.hazards import Bounds, Hazard, PHASE_3_SPILL
from resilient_amr_dispatch.warehouse_map import WarehouseMap, simplify_grid_path


def test_astar_routes_around_phase_3_spill() -> None:
    warehouse = WarehouseMap(include_shelves=False)
    warehouse.add_hazards([PHASE_3_SPILL], clearance=1)

    path = warehouse.plan((20, 50), (90, 50))

    assert path is not None
    assert path[0] == (20, 50)
    assert path[-1] == (90, 50)
    assert all(point not in warehouse.blocked for point in path)
    assert any(y < 29 or y > 71 for _, y in path)


def test_astar_reports_no_route_through_full_barrier() -> None:
    warehouse = WarehouseMap(include_shelves=False)
    barrier = Hazard("barrier", "blocked_aisle", Bounds(40, 60, 0, 99))
    warehouse.add_hazards([barrier], clearance=0)

    assert warehouse.plan((10, 50), (90, 50)) is None


def test_simplify_grid_path_preserves_turns() -> None:
    path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]

    assert simplify_grid_path(path) == [(0, 0), (2, 0), (2, 2)]
