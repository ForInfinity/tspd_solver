import logging
from typing import Iterator

from tspd_osm.types import MapDataset

from .routes import Node, RouteNetNode


class Step1TruckRoutes:
    """
     Given a list of nodes, calculate the possible truck routes.
    """

    def __init__(self, nodes: list[Node], distance_matrix: MapDataset,
                 logger: logging.Logger = logging.getLogger("Step1TruckRoutes"), config: dict[str, any] = None):
        self.nodes: list[Node] = nodes
        self.distances: MapDataset = distance_matrix
        self.logger: logging.Logger = logger
        self.config: dict[str, any] = config or {}

    def calculate(self) -> Iterator[list[int]]:
        """
        Calculate the routes.
        :return: A list of possible truck node ids, the ORDER indicates the route.
        :rtype: List[List[int]]
        """
        raise NotImplementedError('StepOneGenerateTruckRoutes.calculate() is not implemented')


class Step2CombinedRoutes:
    """
     Given a list of possible truck routes, calculate all possible combined routes (with drone routes).
    """

    def __init__(self, nodes: list[Node], possible_truck_routes: Iterator[list[int]], distance_matrix: MapDataset,
                 logger: logging.Logger = logging.getLogger("Step1TruckRoutes"), config: dict[str, any] = None):
        self.possible_truck_routes: Iterator[list[int]] = possible_truck_routes
        self.distances: MapDataset = distance_matrix
        self.nodes: list[Node] = nodes
        self.logger = logger
        self.config: dict[str, any] = config or {}

    def calculate(self) -> Iterator[list[RouteNetNode]]:
        """
        Calculate all possible combined routes.
        :return: A list of possible combi-routes of truck and drone.
        :rtype: list[Routes]
        """
        raise NotImplementedError('StepTwoCalcDroneRoutes.calculate() is not implemented')


class Step3BestRoutes:
    """
        Given a list of possible combined routes, select the best one.
    """

    def __init__(self, nodes: list[Node], possible_combined_routes: Iterator[list[RouteNetNode]],
                 distance_matrix: MapDataset, logger: logging.Logger = logging.getLogger("Step1TruckRoutes"),
                 config: dict[str, any] = None):
        self.possible_combined_routes: Iterator[list[RouteNetNode]] = possible_combined_routes
        self.distances: MapDataset = distance_matrix
        self.nodes: list[Node] = nodes
        self.logger = logger
        self.config: dict[str, any] = config or {}

    def calculate(self) -> list[RouteNetNode]:
        """
        Select the best route.
        :return: The best route.
        :rtype: Routes
        """
        raise NotImplementedError('StepThreeSelectBestRoutes.calculate() is not implemented')


class TSPDSolver:
    def __init__(self, nodes: list[Node], distance_matrix: MapDataset,
                 logger: logging.Logger = logging.getLogger("TSPDSolver"), config: dict[str, any] = None):
        self.logger = logger
        self.nodes: list[Node] = nodes
        self.distances: MapDataset = distance_matrix
        self.config: dict[str, any] = config or {}

    def calculate(self) -> list[RouteNetNode]:
        """
        Calculate the best route.
        :return: The best route.
        :rtype: Routes
        """
        raise NotImplementedError('TSPDSolver.calculate() is not implemented')
