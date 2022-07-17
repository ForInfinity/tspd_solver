import itertools
import math
from multiprocessing.pool import ThreadPool

from sortedcontainers import SortedList

from .__util import logs
from .__util.iterator import CombinedIterator, DroneTuplesIterator
from .routes import calc_combined_time_consumption, split_truck_drone_routes, parse_list_to_route_net_nodes
from .speed import calc_truck_time, calc_drone_time
from .steps import *
from .tsp import calc_truck_tsp_route


class OldStep1(Step1TruckRoutes):

    def calculate(self) -> Iterator[list[int]]:
        num_nodes = len(self.nodes)
        num_min_truck_nodes = math.floor((len(self.nodes) - 1) / 2)
        self.logger.info(f"Calculating TSP routes for {num_nodes} nodes")
        tsp_path, tsp_time = calc_truck_tsp_route(self.distances)
        self.logger.info(f"TSP time: {tsp_time}, TSP Path: {tsp_path}")

        sorted_truck_routes: SortedList[list[int]] = \
            SortedList(key=lambda truck_nodes: self.distances.calc_road_distance(truck_nodes))

        pool_size = self.config.get('pool_size', 8)

        def filter_route(single_truck_route: tuple[int]):
            """

            :param single_truck_route: A single truck route without start node 0.
            :type single_truck_route:
            :return:
            :rtype:
            """
            # For every possible truck route, filter repeated one
            if single_truck_route[0] > single_truck_route[-1]:
                # self.logger.debug(f"filter 1: {(0,) + truck_route}")
                return
            # else:
            #     self.logger.debug(f"filter 1 else: {(0,) + truck_route}")

            single_truck_route = (0,) + single_truck_route
            truck_route_as_list: list[int] = list(single_truck_route)

            # Filter route that has a worse time than tsp time
            if self.distances.calc_road_distance(truck_route_as_list) > tsp_time:
                # self.logger.debug(
                #     f"filter 2: {truck_route} with time {self.distances.calc_road_distance(truck_route_as_list)}")
                return

            # if truck_route_as_list == [0, 1, 2]:
            #     self.logger.debug(f"found: {truck_route_as_list}")

            # Add the route to the list
            sorted_truck_routes.add(truck_route_as_list)

        for num_truck_nodes in range(num_min_truck_nodes, num_nodes + 1):
            # For every possible number of truck nodes N, calculate all possible truck routes.
            all_truck_routes: itertools.permutations = itertools.permutations(range(1, num_nodes),
                                                                              num_truck_nodes)
            self.logger.info(f"Calculating truck routes for {num_truck_nodes} truck nodes")
            pool = ThreadPool(pool_size)
            pool.map(filter_route, all_truck_routes)
            pool.close()
            pool.join()

        self.logger.info(f"Found {len(sorted_truck_routes)} possible truck routes")
        # Return the sorted list of truck routes, the order indicates the time, from shortest to longest.
        # self.logger.debug(f"sorted_truck_routes: {sorted_truck_routes}")
        return iter(sorted_truck_routes)


class RecursiveRouteNetIterator(Iterator[list[RouteNetNode]]):
    """
    An iterator that returns every possible list, which contains truck nodes and drone nodes.
    """

    def recursively_calculate_drone_routes(self, input_truck_route: list[int]) -> list[list[tuple[int, int]]]:
        """
        Calculate all possible drone routes for a truck route.
        :param input_truck_route:
        :type input_truck_route:
        :return: A list of possible taking-off and landing node. [(start_node, return_node), ...]
        :rtype:
        """
        # all_drone_node_ids = [node.id for node in calculate_drone_nodes(nodes, truck_route)]
        truck_route = input_truck_route.copy()

        if truck_route[0] != 0:
            truck_route.insert(0, 0)

        if truck_route[-1] != 0:
            truck_route.append(0)

        num_drone_nodes = len(self.node_ids) - len(input_truck_route)
        truck_node_indexes = range(0, len(truck_route))
        possible_drone_index_routes: list[list[tuple[int, int]]] = []

        # noinspection PyTypeChecker
        def recursion(current_truck_node_index: int, unused_truck_node_count: int, unused_drone_nodes_count: int,
                      drone_path: list[tuple[int, int]]):
            # add this path to the list if it is the last drone node
            if unused_drone_nodes_count == 0:
                possible_drone_index_routes.append(drone_path)
                return
            # No enough truck nodes to cover the drone nodes
            elif unused_truck_node_count <= unused_drone_nodes_count:
                # print(f"drone path abandoned: {drone_path}")
                # recursion of nodes not possible
                return
            # allocate a truck node to the drone path
            else:
                unused_truck_nodes = truck_node_indexes[current_truck_node_index:]
                possible_next_drone_route_permutations: itertools.permutations = itertools.permutations(
                    unused_truck_nodes, 2)
                # filter duplicate items due to reverted direction
                possible_next_drone_routes = []

                for route in possible_next_drone_route_permutations:
                    # filter duplicate items due to reverted direction
                    if route[0] < route[1]:
                        possible_next_drone_routes.append(route)
                for single_next_route in possible_next_drone_routes:
                    recursion(single_next_route[1],
                              unused_truck_node_count - single_next_route[1] + single_next_route[0],
                              unused_drone_nodes_count - 1,
                              drone_path + [single_next_route])

        recursion(0, len(truck_route), num_drone_nodes, [])
        possible_drone_routes = []
        for path in possible_drone_index_routes:
            routes = []
            for tup in path:
                routes.append((truck_route[tup[0]], truck_route[tup[1]]))
            possible_drone_routes.append(routes)
        if len(possible_drone_routes) == 0:
            raise Exception("No possible drone routes")
        return possible_drone_routes

    def get_drone_nodes_iterator(self, truck_node_ids: list[int]) -> Iterator[list[tuple[int, int]]]:
        start_end_tuple_list = self.recursively_calculate_drone_routes(truck_node_ids)
        drone_ids = [node_id for node_id in self.node_ids if node_id not in truck_node_ids]
        return DroneTuplesIterator(start_end_tuple_list, drone_ids)

    def __init__(self, possible_truck_routes: Iterator[list[int]], node_ids: list[int], distances: MapDataset):
        self.node_ids = node_ids
        if possible_truck_routes is None:
            raise ValueError("possible_truck_routes cannot be None")
        self.combined_iterator: CombinedIterator[list[int], list[list[tuple[int, int]]]] = CombinedIterator(
            possible_truck_routes,
            lambda x: self.get_drone_nodes_iterator(x)
        )
        self.distances = distances

    def __next__(self) -> list[RouteNetNode]:
        truck_ids, drone_route_tuples = next(self.combined_iterator)
        # parse 2 ids to RouteNetNode
        truck_route_nodes, drone_route_nodes = parse_list_to_route_net_nodes(truck_ids, drone_route_tuples,
                                                                             self.distances)

        return truck_route_nodes + drone_route_nodes


class OldStep2(Step2CombinedRoutes):

    def calculate(self) -> Iterator[list[RouteNetNode]]:
        self.logger.info(f"Calculating combined routes for truck routes")

        return RecursiveRouteNetIterator(self.possible_truck_routes, [node.id for node in self.nodes], self.distances)


class OldStep3(Step3BestRoutes):

    def calculate(self) -> list[RouteNetNode]:
        _, tsp_distance = calc_truck_tsp_route(self.distances)
        upper_bound_time = calc_truck_time(tsp_distance)
        self.logger.info(f"Calculating best route... Initial upper bound: {upper_bound_time}")
        best_routes: list[RouteNetNode] = []
        for routes in self.possible_combined_routes:
            truck_routes, drone_routes = split_truck_drone_routes(routes)
            # print(f"Truck routes: {[(route.start, route.end) for route in truck_routes]}")
            # print(f"Drone routes: {[(route.start, route.end) for route in drone_routes]}")
            sys_time = calc_combined_time_consumption(
                truck_routes=truck_routes,
                drone_routes=drone_routes,
                truck_time_fun=calc_truck_time,
                drone_time_fun=calc_drone_time,
            )
            truck_distance = self.distances.calc_road_distance([r.start for r in truck_routes])
            truck_time = calc_truck_time(truck_distance)
            if truck_time > upper_bound_time:
                self.logger.info(f"Iteration stopped because {truck_time} is worse than {upper_bound_time}")
                break

            if sys_time < upper_bound_time:
                self.logger.info(f"Found better route: {sys_time}")
                upper_bound_time = sys_time
                best_routes = routes

        if len(best_routes) == 0:
            self.logger.error(f"No better route found!")
        return best_routes


class OldSolver(TSPDSolver):
    def __init__(self, nodes: list[Node], distance_matrix: MapDataset,
                 logger: logging.Logger = logs.get_logger().getChild("old_solver")):
        super().__init__(nodes, distance_matrix, logger)

    def calculate(self) -> list[RouteNetNode]:
        self.logger.info("Calculating old solver step 1")
        step1 = OldStep1(self.nodes, self.distances, self.logger.getChild("step1"))
        step1_result = step1.calculate()
        self.logger.info("Calculating old solver step 2")
        step2 = OldStep2(self.nodes, step1_result, self.distances, self.logger.getChild("step2"))
        step2_result = step2.calculate()
        self.logger.info("Calculating old solver step 3")
        step3 = OldStep3(self.nodes, step2_result, self.distances, self.logger.getChild("step3"))
        return step3.calculate()
