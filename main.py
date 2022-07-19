import argparse
import logging
import os
from dataclasses import asdict
from datetime import datetime
from typing import Optional

import pandas as pd
import tspd_osm.dataset
from dotenv import load_dotenv
from tspd_osm.graph import GraphFactory
from tspd_osm.types import MapDataset

from tpsd_solver.__util.logs import get_logger
from tpsd_solver.__util.timer import Timer, parse_secs_to_str
from tpsd_solver.cut_solver import CutSolver
from tpsd_solver.old_solver import OldSolver
from tpsd_solver.routes import Node, calc_combined_route, split_truck_drone_routes, RouteNetNode
from tpsd_solver.speed import calc_truck_time, calc_drone_time
from tpsd_solver.tsp import calc_truck_tsp_route

load_dotenv()

logger: logging.Logger = get_logger()

"""
    Contributions:
    >	Maps and Images: © OpenStreetMap contributors (https://openstreetmap.org/copyright)
    >	Distance and Time Matrix: © openrouteservice.org by HeiGIT | Map data © OpenStreetMap contributors'
"""

parser = argparse.ArgumentParser("main.py")
parser.add_argument('-o', '--old-solver', dest='old_solver', action='store_true', default=False,
                    help='Run with old solver')
parser.add_argument('-n', '--new-solver', dest='new_solver', action='store_true', default=False,
                    help='Run with new solver')
parser.add_argument('-c', '--cut-factor', dest='cut_factor', type=float, default=1.0, help='Cut factor')
parser.add_argument('-g', '--geo-list', dest='geo_list', type=str, default=os.environ.get('GET_LIST_FILE_PATH'),
                    help='Geo list file path')
parser.add_argument('-k', '--ors-api-key', dest='ors_api_key', type=str,
                    default=os.environ.get('OPEN_ROUTE_SERVICE_API_KEY'),
                    help='Open Route Service API key')

args = parser.parse_args()

os.environ['OPEN_ROUTE_SERVICE_API_KEY'] = args.ors_api_key


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


def run_dataset(filename: str, old_solver=False, cut_factor=1.0) -> Optional[dict]:
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
        return None
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

        time_usage = {}
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
                    logger=logger.getChild("cut_solver"),
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
            time_usage = timer.get_time_usage()

        truck_routes, drone_routes = split_truck_drone_routes(result)

        logger.info('********************************')
        logger.info(f"**final: truck_routes: {truck_routes}")
        logger.info(f"**final: drone_routes: {drone_routes}\n")
        logger.info(f"\tTruck: {' -> '.join([str(t.start) for t in truck_routes])}")
        logger.info(f"\tDrone: {' -> '.join([str((t.start, t.end)) for t in drone_routes])}\n")
        logger.info(f"Count: drone_nodes={int(len(drone_routes) / 2)}, truck_routes={len(truck_routes)}")

        detailed_path_logs = []
        statistic = calc_combined_route(
            truck_routes=truck_routes,
            drone_routes=drone_routes,
            truck_time_fun=calc_truck_time,
            drone_time_fun=calc_drone_time,
            logger=logger,
            log_container=detailed_path_logs
        )

        sum_time = statistic.total_travel_time

        log_line(f"\n*** Count: Truck Routes={len(truck_routes)}, Drone Routes={int(len(drone_routes) / 2)}")
        log_line(f"*** Total Delivery Time: {parse_secs_to_str(sum_time)} ({sum_time:.2f} seconds)")

        # TSP Time
        _, tsp_distance = calc_truck_tsp_route(dataset, debug_output=False)
        tsp_time = calc_truck_time(tsp_distance)

        log_line(f"*** Truck TSP Time (initial upper bound):  {parse_secs_to_str(tsp_time)} "
                 f"({tsp_time:.2f} seconds)")

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

        # return a line of data
        result = {
            'dataset': dataset_name,
            'solver': solver_name,
            'cut_factor': cut_factor,
            'nodes': len(nodes),
        }
        result.update(time_usage)
        result['truck_tsp_time'] = tsp_time
        result.update(asdict(statistic))

        return result


def find_all_datasets():
    for root, ds, fs in os.walk('./data/'):
        for f in fs:
            if f.endswith('.xlsx'):
                fullname = os.path.join(root, f)
                yield fullname


def export_statistics(filename: str, statistics: list[dict]):
    frame = pd.DataFrame(statistics)
    frame.to_excel(filename, index=False)


def main():
    global logger

    tspd_osm.dataset.fetch_data_from_geo_list(args.geo_list)
    dataset_paths = list(find_all_datasets())
    start_time = datetime.now()
    start_time_str = start_time.strftime("%Y%m%d_%H%M%S")
    statics_filename = f"./results/Summary_{start_time_str}.xlsx"
    if not args.old_solver and not args.new_solver:
        logger.warning("No solver selected. All solvers will be run.")
        args.old_solver = True
        args.new_solver = True
    records = []

    for p in dataset_paths:
        logger.info("\n\n\n")
        logger.info(f"****** Start processing {p} ******")
        logger = get_logger().getChild(p)
        if args.old_solver:
            statistic = run_dataset(p, old_solver=True)
            if statistic is not None:
                records.append(statistic)
        if args.new_solver:
            statistic = run_dataset(p, old_solver=False, cut_factor=args.cut_factor)
            if statistic is not None:
                records.append(statistic)
        # Export statistics for each dataset to save data when shutting down or encountering error.
        export_statistics(statics_filename, records)


if __name__ == "__main__":
    main()
