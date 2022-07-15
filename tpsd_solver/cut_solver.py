import itertools
import math
from multiprocessing.pool import ThreadPool

from sortedcontainers import SortedList

from .__util import logs
from .__util.iterator import CombinedIterator
from .__util.timer import parse_secs_to_str
from .routes import calc_combined_time_consumption, split_truck_drone_routes, parse_list_to_route_net_nodes
from .speed import calc_truck_time, calc_drone_time
from .steps import *
from .tsp import calc_truck_tsp_route, calc_partial_truck_tsp_route

total_route_count = 0.0


class CutSolverStep1(Step1TruckRoutes):
    def calc_drone_potential(self, truck_route: list[int], node_idx: int) -> float:
        return self.distances.calc_road_distance(truck_route)

    def calculate(self) -> Iterator[list[int]]:
        global total_route_count
        num_nodes = len(self.nodes)
        num_min_truck_nodes = math.floor((len(self.nodes) + 1) / 2)
        # debug
        # num_min_truck_nodes = num_nodes - 5
        self.logger.info(f"Calculating TSP routes for {num_nodes} nodes")
        tsp_path, tsp_distance = calc_truck_tsp_route(self.distances)
        tsp_time = calc_truck_time(tsp_distance)
        self.logger.info(f"TSP time: {tsp_time:.2f}, TSP Path: {tsp_path}")

        sorted_truck_routes: SortedList[list[int]] = \
            SortedList(key=lambda truck_nodes: self.distances.calc_road_distance(truck_nodes))

        pool_size = self.config.get('threads', 8)
        self.logger.info(f"Using {pool_size} threads")

        def calc_combination_tsp(truck_node_combination: tuple[int]):
            """
            :param truck_node_combination:  A tuple of truck node ids without start node 0.
            :type truck_node_combination:
            :return:
            :rtype:
            """
            global total_route_count
            # For every possible truck route, filter repeated one
            # Add the route to the list

            truck_nodes = [0] + list(truck_node_combination)

            path, distance = calc_partial_truck_tsp_route(self.distances, truck_nodes)

            time_cost = calc_truck_time(distance)

            if time_cost > tsp_time:
                return

            sorted_truck_routes.add(path)
            new_len = len(sorted_truck_routes)
            if new_len > total_route_count * 1.05 or new_len > total_route_count + 500:
                print(f"\x1b[1K\rCalculating truck routes... Calculated TSP result count :{new_len}", end="",
                      flush=True)
                total_route_count = new_len

        truck_route_cut_factor = self.config.get('truck_cut_factor', 0.5)
        # Cut: Reduce the number of possible truck routes to [num_min_truck_nodes,num_min_truck_nodes*(1+cut_factor)]
        num_max_truck_nodes = int(num_min_truck_nodes * (1 + truck_route_cut_factor))
        num_max_truck_nodes = min(num_max_truck_nodes, num_nodes + 1)
        self.logger.info(
            f"Cut factor is {truck_route_cut_factor} -> iterate truck node range "
            f"from {num_min_truck_nodes} to {num_max_truck_nodes}")
        # Minimal max_truck_nodes is num_min_truck_nodes + 1
        num_max_truck_nodes = max(num_max_truck_nodes, num_min_truck_nodes + 1)
        for num_truck_nodes in range(num_min_truck_nodes,
                                     num_max_truck_nodes):
            self.logger.info(f"Calculating truck routes for {num_truck_nodes} truck nodes")
            # For every possible number of truck nodes N, calculate all possible truck routes.
            all_truck_combinations: itertools.combinations = itertools.combinations(range(1, num_nodes),
                                                                                    num_truck_nodes)
            # for r in all_truck_routes:
            #     # Filter the truck routes
            #     filter_route(r)
            # use multiprocessing to speed up the calculation
            print(f"Calculating truck routes... Initializing...", flush=True, end="")
            total_route_count = 0
            pool = ThreadPool(processes=pool_size)
            pool.map(calc_combination_tsp, all_truck_combinations)
            pool.close()
            pool.join()
            print('')

        # Return the sorted list of truck routes, the order indicates the time, from shortest to longest.
        # self.logger.debug(f"sorted_truck_routes: {sorted_truck_routes}")
        return iter(sorted_truck_routes)


class RouteNetIterator(Iterator[list[RouteNetNode]]):
    """
    An iterator that returns every possible list, which contains truck nodes and drone nodes.
    """

    def greedy_calculate_drone_routes(self, input_truck_node_ids: list[int]) -> list[list[tuple[int, int]]]:
        """
        This method:
            1. Set every possible truck node as drone's start point
            2. For every start point:
                2.1 Find the nearest drone node to the start point
                2.2 Make the drone's travelling time as close as possible to the truck's
                2.3 Move the new start point to the drone's return point and repeat step 2, until all drone nodes are
                    visited.
            3. return the list of drone routes
        :param input_truck_node_ids:
        :type input_truck_node_ids:
        :return: A list of drone routes.[(start,target),(target,return),...]
        :rtype:
        """
        truck_node_ids = input_truck_node_ids.copy()
        if truck_node_ids[-1] != 0:
            truck_node_ids.append(0)

        drone_node_ids = [nid for nid in self.node_ids if nid not in truck_node_ids]

        len_truck_nodes = len(truck_node_ids)
        len_drone_nodes = len(drone_node_ids)

        def find_sub_drone_route(start_truck_node_idx: int, available_drone_nodes: list[int]) -> list[
            list[tuple[int, int]]]:
            """
            Find the sub drone route from the start node.
            :param available_drone_nodes: The number of available drone nodes
            :type available_drone_nodes: int
            :param start_truck_node_idx: The index of the start truck node
            :type start_truck_node_idx: int
            :return: A list of sub drone routes.
            :rtype: list[list[tuple[int, int]]]
            """
            available_drone_count = len(available_drone_nodes)

            child_drone_count = available_drone_count - 1
            results: list[list[tuple[int, int]]] = []

            for current_drone_start_idx in range(start_truck_node_idx, len_truck_nodes - available_drone_count):
                # Find the nearest drone node to the start node
                current_drone_node_id = -1
                min_distance = float('inf')
                for drone_node_id in available_drone_nodes:
                    distance = self.distances.get_euclid_distance(truck_node_ids[current_drone_start_idx],
                                                                  drone_node_id)
                    if distance < min_distance:
                        min_distance = distance
                        current_drone_node_id = drone_node_id
                if current_drone_node_id == -1:
                    break

                # find the nearest landing node
                possible_drone_end_idx: list[int] = list(
                    range(current_drone_start_idx + 1, len_truck_nodes - child_drone_count))

                current_drone_landing_idx = -1
                min_distance = float('inf')
                for drone_end_idx in possible_drone_end_idx:
                    distance = self.distances.get_euclid_distance(current_drone_node_id,
                                                                  truck_node_ids[drone_end_idx])
                    if distance < min_distance:
                        min_distance = distance
                        current_drone_landing_idx = drone_end_idx

                current_result: list[tuple[int, int]] = [
                    (truck_node_ids[current_drone_start_idx], current_drone_node_id),
                    (current_drone_node_id, truck_node_ids[current_drone_landing_idx])
                ]
                child_available_drone_nodes = [nid for nid in available_drone_nodes if nid != current_drone_node_id]
                if len(child_available_drone_nodes) == 0:
                    results.append(current_result)
                else:
                    child_results = find_sub_drone_route(current_drone_landing_idx, child_available_drone_nodes)
                    results = results + [(current_result + child_result) for child_result in child_results]

            return results

        return find_sub_drone_route(0, drone_node_ids)

    def get_drone_nodes_iterator(self, truck_node_ids: list[int]) -> Iterator[list[tuple[int, int]]]:
        results = self.greedy_calculate_drone_routes(truck_node_ids)
        return iter(results)

    def __init__(self, possible_truck_routes: Iterator[list[int]], node_ids: list[int], distances: MapDataset):
        self.node_ids = node_ids
        if possible_truck_routes is None:
            raise ValueError("possible_truck_routes cannot be None")
        self.distances = distances
        self.combined_iterator: CombinedIterator[list[int], list[list[tuple[int, int]]]] = CombinedIterator(
            possible_truck_routes,
            lambda x: self.get_drone_nodes_iterator(x)
        )

    def __next__(self) -> list[RouteNetNode]:
        truck_ids, drone_route_tuples = next(self.combined_iterator)
        # parse 2 ids to RouteNetNode
        truck_route_nodes, drone_route_nodes = parse_list_to_route_net_nodes(truck_ids, drone_route_tuples,
                                                                             self.distances)

        return truck_route_nodes + drone_route_nodes


class CutSolverStep2(Step2CombinedRoutes):

    def calculate(self) -> Iterator[list[RouteNetNode]]:
        self.logger.info(f"Calculating combined routes for truck routes")

        return RouteNetIterator(self.possible_truck_routes, [node.id for node in self.nodes], self.distances)


class CutSolverStep3(Step3BestRoutes):

    def calculate(self) -> list[RouteNetNode]:
        _, tsp_distance = calc_truck_tsp_route(self.distances)
        upper_bound_time = calc_truck_time(tsp_distance)
        self.logger.info(f"Calculating best route... Initial upper bound: {parse_secs_to_str(upper_bound_time)}")
        print("\n")
        best_routes: list[RouteNetNode] = []
        for routes in self.possible_combined_routes:
            truck_routes, drone_routes = split_truck_drone_routes(routes)
            # print(f"Truck routes: {[(route.start, route.end) for route in truck_routes]}")
            # print(f"Drone routes: {[(route.start, route.end) for route in drone_routes]}")

            truck_distance = self.distances.calc_road_distance([r.start for r in truck_routes])
            truck_time = calc_truck_time(truck_distance)
            if truck_time > upper_bound_time:
                print("\n")
                self.logger.info(
                    f"Iteration stopped because {parse_secs_to_str(truck_time)} is worse than {parse_secs_to_str(upper_bound_time)}"
                    f"\n -> {[r.start for r in truck_routes]}")
                break

            sys_time = calc_combined_time_consumption(
                truck_routes=truck_routes,
                drone_routes=drone_routes,
                truck_time_fun=calc_truck_time,
                drone_time_fun=calc_drone_time,
            )

            if sys_time < upper_bound_time:
                print(f"\x1b[1K\rFound better route: {parse_secs_to_str(sys_time)}", end="")
                upper_bound_time = sys_time
                best_routes = routes

        if len(best_routes) == 0:
            self.logger.error(f"No better route found!")
        return best_routes


class CutSolver(TSPDSolver):
    """
    Solver with cutting.
    Configs:{
        "step1": {
            "threads": int, # number of threads to use for step 1, default: 8
            "truck_cut_factor": float, # factor for cutting the truck routes, 1.0 means no cutting. default: 0.5
        }
    }
    """

    def __init__(self, nodes: list[Node], distance_matrix: MapDataset,
                 logger: logging.Logger = logs.get_logger().getChild("cut_solver"), configs: dict[str, any] = None):
        super().__init__(nodes, distance_matrix, logger, configs)

    def calculate(self) -> list[RouteNetNode]:
        self.logger.info("Calculating cut solver step 1")
        step1 = CutSolverStep1(self.nodes, self.distances, self.logger.getChild("step1"),
                               config=self.config.get("step1"))
        step1_result = step1.calculate()
        self.logger.info("Calculating cut solver step 2")
        step2 = CutSolverStep2(self.nodes, step1_result, self.distances, self.logger.getChild("step2"),
                               config=self.config.get("step2"))
        step2_result = step2.calculate()
        self.logger.info("Calculating cut solver step 3")
        step3 = CutSolverStep3(self.nodes, step2_result, self.distances, self.logger.getChild("step3"),
                               config=self.config.get("step3"))
        return step3.calculate()
