import logging
from dataclasses import dataclass
from enum import Enum
from typing import Callable

from sortedcontainers import SortedList
from tspd_osm.types import MapDataset

from tpsd_solver.__util.timer import parse_secs_to_str


@dataclass
class Node:
    id: int
    name: str
    lat: str
    lon: str


class Vehicle(Enum):
    """
    Vehicle type.
    """
    TRUCK = 0
    DRONE = 1

    def __str__(self):
        return self.name


@dataclass
class SingleRoute:
    """
    A single route between two nodes with NO RELATIONSHIP with other routes.
    """
    start: int
    end: int
    vehicle: Vehicle
    distance: float

    def __str__(self):
        return f"{self.start} -> {self.end} (v = {self.vehicle.name}, d = {self.distance})"

    def __repr__(self):
        return self.__str__()


@dataclass
class RouteNetNode(SingleRoute):
    """
    A route between two nodes WITH RELATIONSHIP with its children routes.
    """
    children: dict[Vehicle, 'RouteNetNode']

    def __str__(self):
        return f"{self.start} -> {self.end} (v = {self.vehicle.name}, d = {self.distance})"

    def __repr__(self):
        return self.__str__()


@dataclass
class RouteStatistic:
    """
    Static Data of a combined route.
    """
    route_truck_distance: float = 0.0
    route_drone_distance: float = 0.0
    route_truck_travel_time: float = 0.0
    route_drone_travel_time: float = 0.0
    route_truck_waiting_time: float = 0.0
    route_drone_waiting_time: float = 0.0
    route_truck_node_count: int = 0
    route_drone_node_count: int = 0
    total_travel_time: float = 0.0


def calc_combined_route(truck_routes: list[RouteNetNode], drone_routes: list[RouteNetNode],
                        truck_time_fun: Callable[[float], float],
                        drone_time_fun: Callable[[float], float],
                        logger: logging.Logger = None,
                        log_container: list[str] = None) -> RouteStatistic:
    """
    Calculate the time consumption of a route based on drone routes.
    :param log_container:
    :type log_container:
    :param logger:
    :type logger:
    :param truck_routes:
    :type truck_routes:
    :param drone_routes:
    :type drone_routes:
    :param truck_time_fun:
    :type truck_time_fun:
    :param drone_time_fun:
    :type drone_time_fun:
    :return: The Statistic of the combined route.
    :rtype: RouteStatistic
    """
    statistic = RouteStatistic(
        route_truck_node_count=len(truck_routes),
        route_drone_node_count=len(drone_routes),
    )

    unvisited_truck_node_set: set[int] = set()
    drone_start_node_set: set[int] = set()
    drone_land_node_set: set[int] = set()

    for route in drone_routes:
        if Vehicle.TRUCK in route.children:
            # return route
            drone_land_node_set.add(route.end)
        else:
            drone_start_node_set.add(route.start)

    for route in truck_routes:
        unvisited_truck_node_set.add(route.start)

    logs: SortedList[tuple[float, int, str]] = SortedList(key=lambda x: (x[0], x[1]))

    def record_log(secs: float, level: int, msg: str):
        leveled_msg = '...' * level + " " + msg
        if level != 0:
            leveled_msg = " " + leveled_msg
        time_str = parse_secs_to_str(secs)
        logs.add((secs, level, f"{time_str}: {leveled_msg}"))

    current_truck_route = next(filter(lambda r: r.start == 0, truck_routes), None)

    start_station_node_id = current_truck_route.start
    unvisited_truck_node_set.discard(start_station_node_id)

    if current_truck_route is None:
        raise ValueError("No truck route starts from node 0.")

    total_time = 0.
    period_drone_time = 0.
    period_truck_time = 0.
    drone_on_the_way = False

    logs.add((total_time, 0, f"* Initial start from node {start_station_node_id}."))
    record_log(total_time + period_truck_time + 0.1, 1,
               f"Truck departure: {current_truck_route.start} "
               f"-> {current_truck_route.end}")

    # check if drone flying from start station
    first_fly_route = next(filter(lambda r: r.start == 0, drone_routes), None)
    if first_fly_route is not None:
        drone_start_node_set.discard(first_fly_route.start)
        drone_on_the_way = True
        period_drone_time += drone_time_fun(first_fly_route.distance) + drone_time_fun(
            first_fly_route.children[Vehicle.DRONE].distance)
        drone_first_fly_time = drone_time_fun(first_fly_route.distance)
        drone_return_route = first_fly_route.children[Vehicle.DRONE]
        drone_return_time = drone_time_fun(drone_return_route.distance)
        drone_path_time = drone_first_fly_time + drone_return_time

        record_log(total_time, 1, f"Drone departure: {first_fly_route.start} -> {first_fly_route.end}")
        record_log(total_time + drone_first_fly_time, 0, f"Drone arrived and delivered at node {first_fly_route.end}")
        record_log(total_time + drone_first_fly_time + 0.1, 1,
                   f"Drone departure. Route: {drone_return_route.start} -> {drone_return_route.end}")
        record_log(total_time + drone_path_time, 0,
                   f"Drone returned at node {drone_return_route.end}")
        statistic.route_drone_distance += first_fly_route.distance + drone_return_route.distance
        statistic.route_drone_travel_time += drone_path_time

    # Every turn, current_node and the route it starts is visited.
    is_last_route = False
    while True:
        c_route_end_node = current_truck_route.end
        unvisited_truck_node_set.discard(c_route_end_node)

        # deal with the distance
        period_truck_time += truck_time_fun(current_truck_route.distance)
        statistic.route_truck_distance += current_truck_route.distance

        # deal with the end node and children
        # Drone lands
        if c_route_end_node in drone_land_node_set:
            wait_time = period_truck_time - period_drone_time
            total_time += max(period_drone_time, period_truck_time)
            statistic.route_drone_travel_time += period_drone_time
            statistic.route_truck_travel_time += period_truck_time
            record_log(total_time, 1, f"Truck and drone met at node {c_route_end_node}")
            if wait_time > 0:
                record_log(total_time, 2, f"In this node: Drone waited {parse_secs_to_str(wait_time)} seconds.")
                statistic.route_drone_waiting_time += wait_time
            else:
                record_log(total_time, 2, f"In this node: Truck waited {parse_secs_to_str(-wait_time)} seconds.")
                statistic.route_truck_waiting_time += -wait_time
            drone_land_node_set.discard(c_route_end_node)
            period_drone_time = 0.
            period_truck_time = 0.
            drone_on_the_way = False

        # Deal with truck distances before drone's taking off at the end of the route
        if not drone_on_the_way:
            total_time += period_truck_time
            statistic.route_truck_travel_time += period_truck_time
            period_truck_time = 0.0

        # Deal with things after the current node.
        # Drone takes off
        if c_route_end_node in drone_start_node_set:
            drone_start_node_set.discard(c_route_end_node)
            if Vehicle.DRONE not in current_truck_route.children:
                print(
                    f"ERROR: No drone route found for truck route {current_truck_route}.\n routes: {truck_routes} and {drone_routes}")
            drone_path = current_truck_route.children[Vehicle.DRONE]
            drone_path_return = drone_path.children[Vehicle.DRONE]

            drone_path_time = drone_time_fun(drone_path.distance)
            drone_path_return_time = drone_time_fun(drone_path_return.distance)

            period_drone_time = drone_path_time + drone_path_return_time
            period_truck_time = 0.  # clean the truck time
            drone_on_the_way = True

            record_log(total_time + 0.1, 1, f"Drone departure: {c_route_end_node} -> {drone_path.end}")
            record_log(total_time + drone_path_time, 0, f"Drone arrived and delivered at node {drone_path.end}")
            record_log(total_time + drone_path_time + 0.2, 1,
                       f"Drone departure. Route: {drone_path.end} -> {drone_path_return.end}")
            record_log(total_time + period_drone_time, 0,
                       f"Drone returned at node {drone_path_return.end}")
            statistic.route_drone_distance += drone_path.distance + drone_path_return.distance
            statistic.route_drone_travel_time += period_drone_time

        # Truck goes.
        if drone_on_the_way:
            record_log(total_time + period_truck_time, 0,
                       f"Truck arrived at node {current_truck_route.end}")
            record_log(total_time + period_truck_time + 0.1, 1,
                       f"Truck departure: {current_truck_route.children[Vehicle.TRUCK].start} "
                       f"-> {current_truck_route.children[Vehicle.TRUCK].end}")
        else:
            total_time += period_truck_time
            statistic.route_truck_travel_time += period_truck_time
            period_truck_time = 0.0
            record_log(total_time + 0.1, 0, f"Truck arrived at node {current_truck_route.end}")
            if current_truck_route.end != start_station_node_id:
                record_log(total_time + 0.2, 1, f"Truck departure: {current_truck_route.children[Vehicle.TRUCK].start} "
                                                f"-> {current_truck_route.children[Vehicle.TRUCK].end}")

        if is_last_route:
            break

        current_truck_route = current_truck_route.children[Vehicle.TRUCK]
        if current_truck_route.end == start_station_node_id:
            is_last_route = True

    logs.add((total_time + 0.5, 0, f"* End of the simulation."))
    statistic.total_travel_time = total_time
    if logger is not None:
        for log in logs:
            logger.info(log[2])
    if log_container is not None:
        log_container.extend([log[2] for log in logs])
    return statistic


def split_truck_drone_routes(routes: list[RouteNetNode]) -> tuple[list[RouteNetNode], list[RouteNetNode]]:
    """
    Split a list of routes into two lists of routes, one for truck and one for drone.
    :param routes: A list of routes.
    :type routes: list[RouteNetNode]
    :return: A tuple of two lists of routes.
    :rtype: tuple[list[RouteNetNode], list[RouteNetNode]]
    """
    truck_routes = list(filter(lambda x: x.vehicle == Vehicle.TRUCK, routes))
    drone_routes = list(filter(lambda x: x.vehicle == Vehicle.DRONE, routes))
    return truck_routes, drone_routes


def parse_list_to_route_net_nodes(truck_ids: list[int], drone_route_tuples: list[tuple[int, int]],
                                  distances: MapDataset) -> tuple[list[RouteNetNode], list[RouteNetNode]]:
    if len(truck_ids) < len(drone_route_tuples) / 2:
        raise ValueError("Truck ids are not enough.")
    truck_node_length = len(truck_ids)
    truck_route_nodes: list[RouteNetNode] = [
        RouteNetNode(
            start=truck_ids[i],
            end=truck_ids[(i + 1) % truck_node_length],
            vehicle=Vehicle.TRUCK,
            distance=distances.get_road_distance(truck_ids[i], truck_ids[(i + 1) % truck_node_length]),
            children={}
        ) for i in range(truck_node_length)
    ]
    drone_route_nodes: list[RouteNetNode] = []

    for idx, truck_node in enumerate(truck_route_nodes):
        truck_node.children[Vehicle.TRUCK] = truck_route_nodes[(idx + 1) % truck_node_length]
    for (start, end) in drone_route_tuples:
        drone_current_route = RouteNetNode(
            start=start,
            end=end,
            vehicle=Vehicle.DRONE,
            distance=distances.get_euclid_distance(start, end),
            children={}
        )
        drone_last_route: RouteNetNode
        drone_next_route: RouteNetNode
        from_truck: bool
        if start in truck_ids:
            # drone starts from truck and go to the target
            # add this node to truck route and drone return route
            drone_last_route = next(filter(lambda x: x.end == start, truck_route_nodes), None)
            drone_next_route = next(filter(lambda x: x.start == end, drone_route_nodes), None)
            from_truck = True
        else:
            # drone starts from the target and go back to the truck
            # add this node to drone start route and truck return route
            drone_last_route = next(filter(lambda x: x.end == start, drone_route_nodes), None)
            drone_next_route = next(filter(lambda x: x.start == end, truck_route_nodes), None)
            from_truck = False
        if from_truck:
            drone_last_route.children[Vehicle.DRONE] = drone_current_route
            if drone_next_route is not None:
                drone_current_route.children[Vehicle.DRONE] = drone_next_route
        else:
            if drone_last_route is not None:
                drone_last_route.children[Vehicle.DRONE] = drone_current_route
            drone_current_route.children[Vehicle.TRUCK] = drone_next_route
        drone_route_nodes.append(drone_current_route)
    return truck_route_nodes, drone_route_nodes


def find_nearest_drone_node(truck_start: int, truck_end: int, drones: list[int], distances: MapDataset) -> int:
    """
    Find the nearest drone node from the truck start and end node.
    :param distances:
    :type distances:
    :param truck_start: The truck start node.
    :type truck_start: int
    :param truck_end: The truck end node.
    :type truck_end: int
    :param drones: A list of drone node ids.
    :type drones: list[int]
    :return: The nearest drone node id.
    :rtype: int
    """
    min_distance = float("inf")
    min_drone_node = -1
    for drone_node in drones:
        distance = distances.get_euclid_distance(truck_start, drone_node) \
                   + distances.get_euclid_distance(drone_node, truck_end)
        if distance < min_distance:
            min_distance = distance
            min_drone_node = drone_node
    return min_drone_node
