import logging

from tspd_osm.types import MapDataset

from tpsd_solver.routes import calc_combined_route, RouteNetNode, Vehicle, parse_list_to_route_net_nodes
from tpsd_solver.speed import calc_truck_time, calc_drone_time

logging.basicConfig(level=logging.DEBUG)

logger = logging.getLogger("root")

logger.setLevel(logging.DEBUG)


# Without drone
def test_truck_time():
    truck_routes = [
        RouteNetNode(
            start=0,
            end=1,
            vehicle=Vehicle.TRUCK,
            distance=10,
            children={}
        ),
        RouteNetNode(
            start=1,
            end=0,
            vehicle=Vehicle.TRUCK,
            distance=15,
            children={}
        )
    ]
    truck_routes[0].children[Vehicle.TRUCK] = truck_routes[1]
    truck_routes[1].children[Vehicle.TRUCK] = truck_routes[0]

    statistic = calc_combined_route(truck_routes, [], lambda x: x, lambda x: x, logger=logger.getChild("truck"))

    assert statistic.total_travel_time == 25
    pass


# Take off at start point
def test_part_combined_time():
    drone_routes = [
        RouteNetNode(
            start=0,
            end=3,
            vehicle=Vehicle.DRONE,
            distance=40,
            children={}
        ),
        RouteNetNode(
            start=3,
            end=2,
            vehicle=Vehicle.DRONE,
            distance=40,
            children={}
        ),
    ]

    truck_routes = [
        RouteNetNode(
            start=0,
            end=1,
            vehicle=Vehicle.TRUCK,
            distance=10,
            children={}
        ),
        RouteNetNode(
            start=1,
            end=2,
            vehicle=Vehicle.TRUCK,
            distance=20,
            children={}
        ),
        RouteNetNode(
            start=2,
            end=0,
            vehicle=Vehicle.TRUCK,
            distance=30,
            children={}
        )
    ]

    # Drone route
    truck_routes[-1].children[Vehicle.DRONE] = drone_routes[0]
    drone_routes[0].children[Vehicle.DRONE] = drone_routes[1]
    drone_routes[1].children[Vehicle.TRUCK] = truck_routes[2]

    # Truck route
    truck_routes[0].children[Vehicle.TRUCK] = truck_routes[1]
    truck_routes[1].children[Vehicle.TRUCK] = truck_routes[2]
    truck_routes[2].children[Vehicle.TRUCK] = truck_routes[0]

    statistic = calc_combined_route(truck_routes, drone_routes, lambda x: x, lambda x: x,
                                    logger=logger.getChild("part"))
    time = statistic.total_travel_time
    print(f"time = {time}")
    assert time == 110.0


# Take off and land at start point, full drone route
def test_full_combined_time():
    # (0,4) -> (4,1) -> (1,3) -> (3,2)
    drone_routes = [
        RouteNetNode(
            start=0,
            end=4,
            vehicle=Vehicle.DRONE,
            distance=7200,
            children={}
        ),
        RouteNetNode(
            start=4,
            end=1,
            vehicle=Vehicle.DRONE,
            distance=4600,
            children={}
        ),
        RouteNetNode(
            start=1,
            end=3,
            vehicle=Vehicle.DRONE,
            distance=5100,
            children={}
        ),
        RouteNetNode(
            start=3,
            end=2,
            vehicle=Vehicle.DRONE,
            distance=1300,
            children={}
        )
    ]

    # "(0,1) -> (1,2) -> (2,0)"
    truck_routes = [
        RouteNetNode(
            start=0,
            end=1,
            vehicle=Vehicle.TRUCK,
            distance=9300,
            children={}
        ),
        RouteNetNode(
            start=1,
            end=2,
            vehicle=Vehicle.TRUCK,
            distance=6400,
            children={}
        ),
        RouteNetNode(
            start=2,
            end=0,
            vehicle=Vehicle.TRUCK,
            distance=15400,
            children={}
        ),
    ]

    # Connect all truck routes
    truck_len = len(truck_routes)
    for idx, route in enumerate(truck_routes):
        truck_routes[idx].children[Vehicle.TRUCK] = truck_routes[(idx + 1) % truck_len]

    # Drone route
    # Node: 0->4->1
    truck_routes[-1].children[Vehicle.DRONE] = drone_routes[0]  # Truck -> (0->4)
    drone_routes[0].children[Vehicle.DRONE] = drone_routes[1]  # (0->4) -> (4->1)
    drone_routes[1].children[Vehicle.TRUCK] = truck_routes[1]  # (4->1) -> Truck

    # Node: 1->3->2
    truck_routes[0].children[Vehicle.DRONE] = drone_routes[2]  # Truck -> (1->3)
    drone_routes[2].children[Vehicle.DRONE] = drone_routes[3]  # (1->3) -> (3->2)
    drone_routes[3].children[Vehicle.TRUCK] = truck_routes[2]  # (3->2) -> Truck

    statistic = calc_combined_route(truck_routes, drone_routes, lambda x: x, lambda x: x / 2,
                                    logger=logger.getChild("part"))
    assert statistic.total_travel_time == 31100.0


# Not take off at start point
def test_case_3():
    distances = MapDataset.from_xlsx("data/case_3.xlsx")
    truck_routes, drone_routes = parse_list_to_route_net_nodes(
        [0, 1, 4],
        [(0, 2), (2, 1), (1, 3), (3, 4)],
        distances,
    )

    statistic = calc_combined_route(truck_routes, drone_routes, calc_truck_time, calc_drone_time,
                                    logger=logger.getChild("part"))
    assert statistic.total_travel_time == 3516.0

    pass
