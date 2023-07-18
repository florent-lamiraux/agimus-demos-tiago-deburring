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
def baseL1dist(q1: List[float], q2: List[float]) -> float:
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

def configDist(q1: List[float], q2: List[float], jointSpeeds: List[float]) -> float:
    res = 0
    if baseMoves(q1, q2):
        res += baseL1dist(q1, q2)
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
    # data['distance_matrix'] = [0 for _ in range(nbVertices)]
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
                                                                      jointSpeeds)
                data['distance_matrix'][cluster[i-1]][j] = int(100000*data['distance_matrix'][cluster[i-1]][j])
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def create_data_model_bis(configurations, clusters, jointSpeeds):
    data = dict()
    data['clusters'] = clusters
    data['configurations'] = configurations
    data['jointspeeds'] = jointSpeeds
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

# def firstGTSPround(configurations, clusters, jointSpeeds, distances):
def firstGTSPround(distances):
    print("    creating model")
    # ROUTING MODEL
    data = dict()
    data['distance_matrix'] = distances
    # data['clusters'] = clusters
    # data['configurations'] = configurations
    # data['jointspeeds'] = jointSpeeds
    data['num_vehicles'] = 1
    data['depot'] = 0
    # data = create_data_model(configurations, clusters, jointSpeeds)
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
    # # For online distance computation
    # def distance_callback(from_index, to_index):
    #     # GET THE CORRECT INDICES
    #     from_node = manager.IndexToNode(from_index)
    #     to_node = manager.IndexToNode(to_index)
    #     c_from = 0
    #     while from_node not in data['clusters'][c_from]:
    #         c_from+=1
    #     # should have c_from <= len(data['clusters'])
    #     # if we are on an arc inside a cluster
    #     if to_node in data['clusters'][c_from]:
    #         fromIdx = data['clusters'][c_from].index(from_node)
    #         toIdx = data['clusters'][c_from].index(to_node)
    #         if fromIdx < len(data['clusters'][c_from])-1:
    #             if toIdx==fromIdx+1:
    #                 return int(0)
    #             else:
    #                 return int(1e12)
    #         else:
    #             if toIdx==0:
    #                 return int(0)
    #             else:
    #                 return int(1e12)
    #     # if we are on an arc leaving a cluster
    #     fromIdx = data['clusters'][c_from].index(from_node)
    #     if fromIdx < len(data['clusters'][c_from])-1:
    #         fromIdx += 1
    #     else:
    #         fromIdx = 0
    #     from_node = data['clusters'][c_from][fromIdx]
    #     # COMPUTE THE DISTANCE
    #     dist = configDist(data['configurations'][from_node]["q"],
    #                       data['configurations'][to_node]["q"],
    #                       data['jointspeeds'])
    #     return int(100000*dist)
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # SEARCH PARAMETERS
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)
    # CHRISTOFIDES, PATH_CHEAPEST_ARC, PARALLEL_CHEAPEST_INSERTION
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # GREEDY_DESCENT, GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 10
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
    # # For online distance computation
    # manager = pywrapcp.RoutingIndexManager(len(initSol),
    #                                        data['num_vehicles'], data['depot'])
    # routing = pywrapcp.RoutingModel(manager)
    # def distance_callback(from_index, to_index):
    #     # GET THE CORRECT INDICES
    #     from_node = manager.IndexToNode(from_index)
    #     to_node = manager.IndexToNode(to_index)
    #     c_from = 0
    #     while from_node not in data['clusters'][c_from]:
    #         c_from+=1
    #     # should have c_from <= len(data['clusters'])
    #     # if we are on an arc inside a cluster
    #     if to_node in data['clusters'][c_from]:
    #         fromIdx = data['clusters'][c_from].index(from_node)
    #         toIdx = data['clusters'][c_from].index(to_node)
    #         if fromIdx < len(data['clusters'][c_from])-1:
    #             if toIdx==fromIdx+1:
    #                 return int(0)
    #             else:
    #                 return int(1e12)
    #         else:
    #             if toIdx==0:
    #                 return int(0)
    #             else:
    #                 return int(1e12)
    #     # if we are on an arc leaving a cluster
    #     fromIdx = data['clusters'][c_from].index(from_node)
    #     if fromIdx < len(data['clusters'][c_from])-1:
    #         fromIdx += 1
    #     else:
    #         fromIdx = 0
    #     from_node = data['clusters'][c_from][fromIdx]
    #     # COMPUTE THE DISTANCE
    #     dist = configDist(data['configurations'][from_node]["q"],
    #                       data['configurations'][to_node]["q"],
    #                       data['jointspeeds'])
    #     return int(100000*dist)
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # SEARCH PARAMETERS
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # GREEDY_DESCENT, GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 30
    search_parameters.log_search = False
    routing.CloseModelWithParameters(search_parameters)
    initial_solution = routing.ReadAssignmentFromRoutes([initSol], True) # outputs some warning
    print("    solving it")
    solution = routing.SolveFromAssignmentWithParameters(initial_solution, search_parameters)
    print("SOLUTION : ")
    print(solution)
    if solution:
        # print_solution(manager, routing, solution)
        print('Objective : ', solution.ObjectiveValue())
        route = get_route(solution, routing, manager)
    return data, route


### UPDATE COST MATRIX

def nodeIsInCluster(node, clusters):
    clusIdx = 0
    nbClus = len(clusters)
    while clusIdx<nbClus and node not in clusters[clusIdx]:
        clusIdx+=1
    if clusIdx==nbClus:
        print("node not found")
    else:
        return clusIdx

def computeCostWithBase(q1, h1, q2, h2, armPlanner, basePlanner, graph, ps):
    q1r = q1[:4]+robot.q0[4:]
    res1, q1r, _ = graph.generateTargetConfig('move_base', robot.q0, q1r)
    if res1:
        q2r = q2[:4]+robot.q0[4:]
        res2, q2r, _ = graph.generateTargetConfig('move_base', robot.q0, q2r)
        if res2:
            try:
                p1 = wd(armPlanner.computePath(q1,[q1r]))
                p2 = wd(basePlanner.computePath(q1r,[q2r]))
                p3 = wd(armPlanner.computePath(q2r,[q2]))
            except:
                raise Exception("collision during path planning")
            else:
                ps.client.basic.problem.addPath(p1)
                ps.client.basic.problem.addPath(p2)
                ps.client.basic.problem.addPath(p3)
                armCost = ps.pathLength(0)
                armCost+=ps.pathLength(2)
                baseCost = ps.pathLength(1)
                ps.erasePath(2)
                ps.erasePath(1)
                ps.erasePath(0)
                return baseCost, armCost

def computeArmOnlyCost(q1, h1, q2, h2, armPlanner, ps):
    try:
        p1 = wd(armPlanner.computePath(q1,[q2]))
    except:
        raise Exception("collision during path planning")
    else:
        ps.client.basic.problem.addPath(p1)
        armCost = ps.pathLength(0)
        ps.erasePath(0)
    return armCost

def updateGTSP(data, sol, clusters, configs, armPlanner, basePlanner, graph, ps):
    # RETRIEVE GTSP ROUTE
    print("retrieve gtsp sol from tsp one")
    gtspSol = list()
    clustersOrder = list()
    idx = 0
    clusId = nodeIsInCluster(sol[idx], clusters)
    solValid = True
    while solValid and len(gtspSol)<len(clusters):
        if sol[idx+len(clusters[clusId])-1] in clusters[clusId]:
            gtspSol.append(sol[idx])
            clustersOrder.append(clusId)
            idx = idx+len(clusters[clusId])
            clusId = nodeIsInCluster(sol[idx], clusters)
        else:
            print("UNVALID SOLUTION")
            solValid = False
            return
    if solValid:
        gtspSol.append(gtspSol[0])
        clustersOrder.append(clustersOrder[0])
    # COMPUTE COSTS
    print("compute costs")
    solCosts = list()
    for i in range(len(gtspSol)-1):
        q1 = configs[gtspSol[i]]["q"]
        h1 = configs[gtspSol[i]]["hole"]
        q2 = configs[gtspSol[i+1]]["q"]
        h2 = configs[gtspSol[i+1]]["hole"]
        if baseMoves(q1, q2):
            print("base moves")
            baseCost, armCost = computeCostWithBase(q1, h1, q2, h2,
                                                    armPlanner, basePlanner, graph, ps)
            solCosts.append(baseCost+armCost)
        else:
            print("only arm")
            armCost = computeArmOnlyCost(q1, h1, q2, h2, armPlanner, ps)
            solCosts.append(armCost)
    # UPDATE COST MATRIX
    print("update cost matrix")
    for i in range(len(gtspSol)-1):
        c1 = clustersOrder[i]
        n1_index = clusters[c1].index(gtspSol[i])
        n1 = clusters[c1][n1_index-1]
        n2 = gtspSol[i+1]
        print("old/new ratio :  ", data['distance_matrix'][n1][n2]/int(1000000*solCosts[i]))
        data['distance_matrix'][n1][n2] = int(1000000*solCosts[i])
