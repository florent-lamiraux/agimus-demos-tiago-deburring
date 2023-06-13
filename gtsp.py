#import ortools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

from typing import List

def baseMoves(q1: List[float], q2: List[float]) -> bool:
    return abs(q1[0]-q2[0])>1e-5 or abs(q1[1]-q2[1])>1e-5

# \goal compute the distance between two configurations
# \param q1, q2 the considered configurations
# \param jointSpeeds max speeds of the joints (for maxJointDiff)
# \param weights weights given to the different joints (for jointL2dist)
# \rval float the value of the given distance
def baseL2dist(q1: List[float], q2: List[float]) -> float:
    return abs(q2[0]-q1[0]) + abs(q2[1]-q1[1])

def maxJointDiff(q1: List[float], q2: List[float], jointSpeeds: List[float]) -> float:
    maxDist=  0
    for i in range(12):
        dist = abs( q1[4+i]-q2[4+i] ) / jointSpeeds[i]
        if dist>maxDist:
            maxDist = dist
    return maxDist

def jointL2dist(q1: List[float], q2: List[float], weights: List[float]) -> float:
    dist = 0;
    for i in range(12):
        dist += weights[i] * (q1[4+i]-q2[4+i]) * (q1[4+i]-q2[4+i])
    return sqrt(dist)

def configDist(q1: List[float], q2: List[float], jointSpeeds: List[float],
               baseMovement: bool) -> float:
    res = 0
    if baseMovement:
        res += baseL2dist(q1, q2)
        res += maxJointDiff(q1, q2,  jointSpeeds)
        res += maxJointDiff(q1, q2, jointSpeeds)
    else:
        res += maxJointDiff(q1, q2, jointSpeeds)
    return res



### OR TOOLS MODEL GENERATION

# ROUTING MODEL
def create_data_model(configurations, clusters, jointSpeeds):
    nbVertices = len(configurations)
    vertices = set([i for i in range(nbVertices)])
    data = {}
    data['distance_matrix'] = [[int(1e12) for _ in range(nbVertices)]
                               for _ in range(nbVertices)]
    for cluster in clusters:
        verticesToConsider = vertices.difference(cluster)
        for i in range(len(cluster)):
            # inner cycle
            data['distance_matrix'][cluster[i-1]][cluster[i]] = 0
            # move arcs
            for j in verticesToConsider:
                data['distance_matrix'][cluster[i-1]][j] = configDist(configurations[cluster[i]]["q"],
                                                             configurations[j]["q"],
                                                             jointSpeeds,
                                                             baseMoves(configurations[cluster[i]]["q"],
                                                                       configurations[j]["q"]))
                data['distance_matrix'][cluster[i-1]][j] = int(100000000*data['distance_matrix'][cluster[i-1]][j])
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

# SOLUTION PRINTER
def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print('Objective: {} miles'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Route distance: {}miles\n'.format(route_distance)

# SOLUTION SAVER
def get_route(solution, routing, manager):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    index = routing.Start(0)
    route = [manager.IndexToNode(index)]
    while not routing.IsEnd(index): 
        index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
    return route
    # return routes

def firstGTSPround(configurations, clusters, jointSpeeds):
    print("    creating model")
    # ROUTING MODEL
    data = create_data_model(configurations, clusters, jointSpeeds)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    # DISTANCE CALLBACK
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # SEARCH PARAMETERS
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)
    # CHRISTOFIDES, PATH_CHEAPEST_ARC, PARALLEL_CHEAPEST_INSERTION
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GREEDY_DESCENT)
    # GREEDY_DESCENT, GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 1
    search_parameters.log_search = False
    # SOLVE
    print("    solving it")
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        # print_solution(manager, routing, solution)
        print('Objective : ', solution.ObjectiveValue())
        route = get_route(solution, routing, manager)
    return data, route

def GTSPiteration(data, initSol):
    print("    creating model")
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    # DISTANCE CALLBACK
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # SEARCH PARAMETERS
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # GREEDY_DESCENT, GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 10
    search_parameters.log_search = False
    routing.CloseModelWithParameters(search_parameters)
    initial_solution = routing.ReadAssignmentFromRoutes([initSol], True) # outputs some warning
    print("    solving it")
    solution = routing.SolveFromAssignmentWithParameters(initial_solution, search_parameters)
    if solution:
        # print_solution(manager, routing, solution)
        print('Objective : ', solution.ObjectiveValue())
        route = get_route(solution, routing, manager)
    return data, route
