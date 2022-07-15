from itertools import combinations

import gurobipy as gp
from gurobipy import GRB
from tspd_osm.types import MapDataset


def calc_truck_tsp_route(distances: MapDataset, debug_output=False) -> tuple[list[int], float]:
    """
    Calculate TSP Route using python_tsp library.
    :param debug_output:
    :type debug_output:
    :param distances: A distance Matrix
    :type distances:
    :return:
    :rtype:
    """

    n, dist = parse_distances_to_gurobi_input(distances)

    return calc_gurobi_tsp_route(n, dist, debug_output)
    # arr = np.array(distances.to_array(distance_type='road'))
    # permutation, distance = solve_tsp_dynamic_programming(arr)
    # return permutation, distance


def parse_distances_to_gurobi_input(distances: MapDataset) -> tuple[int, dict[tuple[int, int], float]]:
    """
    Parse distances to Gurobi input.
    :param distances: A distance Matrix
    :type distances:
    :return:
    :rtype:
    """
    n = distances.count
    dist = {(i, j): distances.get_road_distance(i, j)
            for i in range(n) for j in range(i)}
    return n, dist


def parse_partial_distances_to_gurobi_input(distances: MapDataset, nodes: list[int]) \
        -> tuple[int, dict[tuple[int, int], float]]:
    """
    Parse distances to Gurobi input.
    :param nodes:
    :type nodes:
    :param distances: A distance Matrix
    :type distances:
    :return:
    :rtype:
    """
    n = len(nodes)
    dist = {(i, j): distances.get_road_distance(nodes[i], nodes[j])
            for i in range(n) for j in range(i)}
    return n, dist


def parse_gurobi_result_to_real_index(path: list[int], nodes: list[int]) -> list[int]:
    """
    Parse Gurobi result to real index.
    :param path:
    :type path:
    :param nodes:
    :type nodes:
    :return:
    :rtype:
    """
    return [nodes[i] for i in path]


def calc_partial_truck_tsp_route(distances: MapDataset, nodes: list[int]) -> tuple[list[int], float]:
    """
    Calculate TSP Route using python_tsp library.
    :param nodes:
    :type nodes:
    :param distances: A distance Matrix
    :type distances:
    :return:
    :rtype:
    """

    n, dist = parse_partial_distances_to_gurobi_input(distances, nodes)

    raw_path, cost = calc_gurobi_tsp_route(n, dist, debug_output=False)

    return parse_gurobi_result_to_real_index(raw_path, nodes), cost


# noinspection PyArgumentList
def calc_gurobi_tsp_route(n, dist, debug_output=True) -> tuple[list[int], float]:
    # Copyright 2022, Gurobi Optimization, LLC

    # Solve a traveling salesman problem on a randomly generated set of
    # points using lazy constraints.   The base MIP model only includes
    # 'degree-2' constraints, requiring each node to have exactly
    # two incident edges.  Solutions to this model may contain subtours -
    # tours that don't visit every city.  The lazy constraint callback
    # adds new constraints to cut them off.

    # Callback - use lazy constraints to eliminate sub-tours
    def subtourelim(model, where):
        if where == GRB.Callback.MIPSOL:
            vals = model.cbGetSolution(model._vars)
            # find the shortest cycle in the selected edge list
            tour = subtour(vals)
            if len(tour) < n:
                # add subtour elimination constr. for every pair of cities in tour
                model.cbLazy(gp.quicksum(model._vars[i, j]
                                         for i, j in combinations(tour, 2))
                             <= len(tour) - 1)

    # Given a tuplelist of edges, find the shortest subtour

    def subtour(vals):
        # make a list of edges selected in the solution
        edges = gp.tuplelist((i, j) for i, j in vals.keys()
                             if vals[i, j] > 0.5)
        unvisited = list(range(n))
        cycle = range(n + 1)  # initial length has 1 more city
        while unvisited:  # true if list is non-empty
            thiscycle = []
            neighbors = unvisited
            while neighbors:
                current = neighbors[0]
                thiscycle.append(current)
                unvisited.remove(current)
                neighbors = [j for i, j in edges.select(current, '*')
                             if j in unvisited]
            if len(cycle) > len(thiscycle):
                cycle = thiscycle
        return cycle

    # Parse argument

    with gp.Env(empty=True) as env:
        if debug_output:
            env.setParam('OutputFlag', 1)
        else:
            env.setParam('OutputFlag', 0)
        env.start()
        m = gp.Model(env=env)

        # Create variables

        vars = m.addVars(dist.keys(), obj=dist, vtype=GRB.BINARY, name='e')
        for i, j in vars.keys():
            vars[j, i] = vars[i, j]  # edge in opposite direction

        # You could use Python looping constructs and m.addVar() to create
        # these decision variables instead.  The following would be equivalent
        # to the preceding m.addVars() call...
        #
        # vars = tupledict()
        # for i,j in dist.keys():
        #   vars[i,j] = m.addVar(obj=dist[i,j], vtype=GRB.BINARY,
        #                        name='e[%d,%d]'%(i,j))

        # Add degree-2 constraint

        m.addConstrs(vars.sum(i, '*') == 2 for i in range(n))

        # Using Python looping constructs, the preceding would be...
        #
        # for i in range(n):
        #   m.addConstr(sum(vars[i,j] for j in range(n)) == 2)

        # Optimize model

        m._vars = vars
        m.Params.LazyConstraints = 1
        m.optimize(subtourelim)

        vals = m.getAttr('X', vars)
        tour = subtour(vals)
        assert len(tour) == n

        # Print solution
        if debug_output:
            print('')
            print('Optimal tour: %s' % str(tour))
            print('Optimal distance: %g' % m.ObjVal)
            print('')

        return tour, m.ObjVal
