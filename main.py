import logging
import os
from datetime import datetime
from typing import Optional

import tspd_osm.dataset
from dotenv import load_dotenv
from tspd_osm.graph import GraphFactory
from tspd_osm.types import MapDataset

from tpsd_solver.__util.logs import get_logger
from tpsd_solver.__util.timer import Timer, parse_secs_to_str
from tpsd_solver.cut_solver import CutSolver
from tpsd_solver.old_solver import OldSolver
from tpsd_solver.routes import Node, calc_combined_time_consumption, split_truck_drone_routes, RouteNetNode
from tpsd_solver.speed import calc_truck_time, calc_drone_time
from tpsd_solver.tsp import calc_truck_tsp_route

load_dotenv()

logger: logging.Logger = get_logger()

"""
    Contributions:
    >	Maps and Images: © OpenStreetMap contributors (https://openstreetmap.org/copyright)
    >	Distance and Time Matrix: © openrouteservice.org by HeiGIT | Map data © OpenStreetMap contributors'
"""


def write_result_to_file(result: list[str], filename: str):
    with open(filename, 'w') as f:
        f.write('\n'.join(result))


def save_graph(filename: str, dataset: MapDataset, truck_routes: list[RouteNetNode], drone_routes: list[RouteNetNode]):
    logger.info(f"Saving graph to {filename}")
    truck_edges: dict[tuple[int, int], Optional[str]] = {}
    drone_edges: dict[tuple[int, int], Optional[str]] = {}
    for route in truck_routes:
        # travel_time = calc_truck_time(route.distance)
        # travel_time = parse_secs_to_str(travel_time)
        truck_edges[(route.start, route.end)] = None  # travel_time
    for route in drone_routes:
        # travel_time = calc_drone_time(route.distance)
        # travel_time = parse_secs_to_str(travel_time)
        drone_edges[(route.start, route.end)] = None  # travel_time
    graph = GraphFactory(dataset)
    graph.draw_drone_edges(drone_edges)
    graph.draw_truck_edges(truck_edges)
    graph.save(filename)


def run_dataset(filename: str, old_solver=False, cut_factor=1.0):
    global logger
    dataset = MapDataset.from_xlsx(filename)
    nodes = [
        Node(id=node_id, name=dataset.meta.get(node_id).get('description'), lat=dataset.meta.get(node_id).get('lat'),
             lon=dataset.meta.get(node_id).get('lon')) for node_id in dataset.meta.keys()]
    timestr = datetime.now().strftime("%Y%m%d_%H%M%S")
    dataset_name = filename.split('/')[-1].split('.')[0]
    solver_name = "old-solver" if old_solver else "new-solver"
    os.makedirs(f"./logs/", exist_ok=True)

    if old_solver and len(nodes) > 11:
        logger.warning('Skipping old solver for dataset with more than 12 nodes!')
        return
    logger.info(f"Running {solver_name} for {dataset_name}")
    basic_name = f"{dataset_name}_{solver_name}_{timestr}"
    os.makedirs(f"./results/", exist_ok=True)
    with open(f'./results/{basic_name}.log', 'w') as logfile:
        def log_line(s: str):
            logfile.write(s + '\n')
            logfile.flush()

        def log_lines(s: list[str]):
            for line in s:
                logfile.write(line + '\n')
            logfile.flush()

        logfile.truncate(0)
        log_line('Calculate TSP-D with following nodes:')
        log_lines([f'\t{node.id: >2d}. lat={node.lat}, lon={node.lon} {node.name}' for node in nodes])

        with Timer(logger=logger, name="Calculation") as timer:
            if old_solver:
                solver = OldSolver(
                    nodes=nodes,
                    distance_matrix=dataset,
                    logger=logger.getChild("old_solver"),
                )
            else:
                solver = CutSolver(
                    nodes=nodes,
                    distance_matrix=dataset,
                    logger=logger.getChild("old_solver"),
                    configs={
                        'step1': {
                            "truck_cut_factor": cut_factor,
                            "threads": 32
                        }
                    }
                )
            timer.record_event_end("Create solver object")
            result = solver.calculate()
            timer.record_event_end("Calculate")
            log_lines(timer.get_logs())

        truck_routes, drone_routes = split_truck_drone_routes(result)

        print('********************************')
        logger.warning(f"**final: truck_routes: {truck_routes}")
        logger.warning(f"**final: drone_routes: {drone_routes}\n")
        logger.info(f"\tTruck: {' -> '.join([str(t.start) for t in truck_routes])}")
        logger.info(f"\tDrone: {' -> '.join([str((t.start, t.end)) for t in drone_routes])}\n")
        logger.info(f"Count: drone_nodes={int(len(drone_routes) / 2)}, truck_routes={len(truck_routes)}")

        detailed_path_logs = []
        sum_time = calc_combined_time_consumption(
            truck_routes=truck_routes,
            drone_routes=drone_routes,
            truck_time_fun=calc_truck_time,
            drone_time_fun=calc_drone_time,
            logger=logger,
            log_container=detailed_path_logs
        )
        log_line(f"\n*** Count: Truck Routes={len(truck_routes)}, Drone Routes={int(len(drone_routes) / 2)}")
        log_line(f"*** Total Delivery Time: {parse_secs_to_str(sum_time)} ({sum_time:.2f} seconds)")
        # TSP Time
        _, tsp_distance = calc_truck_tsp_route(dataset, debug_output=False)
        upper_bound_time = calc_truck_time(tsp_distance)
        log_line(f"*** Truck TSP Time (initial upper bound):  {parse_secs_to_str(upper_bound_time)} "
                 f"({upper_bound_time:.2f} seconds)")

        log_line("\n\nOptimal routes:")
        log_line(f"\tTruck: {' -> '.join([str(t.start) for t in truck_routes])}")
        log_line(f"\tDrone: {' -> '.join([str((t.start, t.end)) for t in drone_routes])}\n")

        log_lines(detailed_path_logs)
        log_line("\n\nFinished.")
        logger.info('Finished')
        save_graph(f"./results/{basic_name}.png",
                   dataset,
                   truck_routes=truck_routes,
                   drone_routes=drone_routes)


def find_all_datasets():
    for root, ds, fs in os.walk('./data/'):
        for f in fs:
            if f.endswith('.xlsx'):
                fullname = os.path.join(root, f)
                yield fullname


def main():
    global logger

    tspd_osm.dataset.fetch_data_from_geo_list(os.environ.get('GET_LIST_FILE_PATH'))
    dataset_paths = list(find_all_datasets())

    for p in dataset_paths:
        logger.info("\n\n\n\n")
        logger.info(f"****** Start processing {p} ******")
        run_dataset(p, old_solver=True)
        run_dataset(p, old_solver=False, cut_factor=0.25)


if __name__ == "__main__":
    main()
