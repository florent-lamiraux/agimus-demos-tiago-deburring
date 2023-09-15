# BSD 2-Clause License

# Copyright (c) 2023, Hannes Van Overloop
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



#import ortools
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

### DATA MODEL FOR THE SOLVER
def create_solver_instance(distances):
    """
    \param distances two dimensionnal array containing the distance matrix
    \retval data dict with all the data needed for the solver
    """
    data = dict()
    data['distance_matrix'] = distances
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

### SOLUTION PRINTER
def print_solution(manager, routing, solution):
    """
    \param manager
    \param routing
    \param solution list with the order of visit of the nodes
    prints solution and the associated cost
    """
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

### SOLUTION SAVER
def get_route(solution, routing, manager):
    """
    \param solution list with the order of visit of the nodes
    \param routing
    \param manager
    \retval route list of the nodes in the order they are visited in the solution
    get vehicle routes from a solution and store them in an array
    """
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    index = routing.Start(0)
    route = [manager.IndexToNode(index)]
    while not routing.IsEnd(index): 
        index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
    return route


### CALLS TO THE SOLVER

def firstGTSPround(distances):
    """
    \param distances two dimensionnal array containing the distance matrix
    \retval data dict with all the data needed for the solver
    \retval route the route obtained by the solver (OR-Tools data structure)
    """
    print("    creating model")
    # ROUTING MODEL FOR THE SOLVER
    data = dict()
    data['distance_matrix'] = distances
    data['num_vehicles'] = 1
    data['depot'] = 0
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    # DISTANCE CALLBACK
    def distance_callback(from_index, to_index):
        """
        \retval the distance between the two nodes
        convert from routing variable Index to distance matrix NodeIndex
        """
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # SEARCH PARAMETERS
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)
    # possible heuristics : CHRISTOFIDES, PATH_CHEAPEST_ARC, PARALLEL_CHEAPEST_INSERTION
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # possible heuristics : GREEDY_DESCENT, GUIDED_LOCAL_SEARCH
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
    """
    \param data dict with all the data needed for the solver
    \param initsol list with an initial solution
    \retval data dict with all the data needed for the solver
    \retval route the route obtained by the solver (OR-Tools data structure)
    """
    # ROUTING MODEL FOR THE SOLVER
    print("    creating model")
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    # DISTANCE CALLBACK
    def distance_callback(from_index, to_index):
        """
        \retval the distance between the two nodes
        convert from routing variable Index to distance matrix NodeIndex
        """
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # SEARCH PARAMETERS
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    # possible heuristics : GREEDY_DESCENT, GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 10
    search_parameters.log_search = False
    routing.CloseModelWithParameters(search_parameters)
    initial_solution = routing.ReadAssignmentFromRoutes([initSol], True) # outputs some warning
    # SOLVE
    print("    solving it")
    solution = routing.SolveFromAssignmentWithParameters(initial_solution, search_parameters)
    print("SOLUTION : ")
    print(solution)
    if solution:
        # print_solution(manager, routing, solution)
        print('Objective : ', solution.ObjectiveValue())
        route = get_route(solution, routing, manager)
    return data, route


### RETRIEVE THE RTSP SOLUTION

def solFromFile(filePath):
    """
    \param filePath string with the location of the file to read the TSP solution
    \retval nbVertices int corresponding to the number of nodes in the TSP solution
    \retval sol list containing the TSP solution
    """
    f = open(filePath, 'r')
    content = f.readlines()
    f.close()
    sol = list()
    nbVertices = int(content[0])
    for line in content[1:]:
        for i in line.split(' ')[:-1]:
            sol.append(int(i))
    sol.append(0)
    return nbVertices, sol

def LKHsolFromFile(filePath):
    """
    \param filePath string with the location of the file to read the LKH solution
    \retval nbVertices int corresponding to the number of nodes in the TSP solution
    \retval sol list containing the TSP solution
    """
    f = open(filePath, 'r')
    content = f.readlines()
    f.close()
    sol = list()
    nbVertices = int(content[4].split(' ')[-1])
    for line in content[6:-1]:
        sol.append(int(line[:-1])-1)
    sol.append(0)
    return nbVertices, sol

def clustersFromFile(filePath):
    """
    \param filePath string with the location of the file to read the node/configuration clusters
    \retval clusters list of lists with the indices of the nodes in each cluster of the GTSP
    """
    f = open(filePath, 'r')
    content = f.readlines()
    f.close()
    clusters = list()
    for line in content:
        c = list()
        for i in line.split(' '):
            c.append(int(i))
        clusters.append(c)
    return clusters

def configsFromFile(filePath):
    """
    \param filePath string with the location of the file to read the configurations
    \retval configs list of lists with the configurations corresponding to the nodes of the GTSP
    """
    f = open(filePath, 'r')
    content = f.readlines()
    f.close()
    configs = list()
    for line in content:
        c = list()
        for j in line.split(' '):
            c.append(float(j))
        configs.append(c)
    return configs

def getGTSPsolFromOrtoolsSol(sol, clusters, configs, resPath):
    """
    \param sol list starting and ending at the depot (0) with the solution of the gtsp instance
    \param clusters list of lists with the node clusters of the gtsp instance
    \param configs list of lists with the configuration associated to every node
    \param resPath string with the location to write the solution at
    \retval gtspSol ordered list of the nodes to effectively visit in the RTSP solution
    \retval clustersOrder list with the order of visit of the clusters ie. tasks
    """
    # INITIALISATION
    print("Retrieve gtsp sol from tsp one")
    gtspSol = list()
    clustersOrder = list()
    idx = 1
    clusId = nodeIsInCluster(sol[idx], clusters)
    solValid = True
    solComplete = False
    # RETRIEVE GTSP ROUTE
    while not solComplete and solValid and len(gtspSol)<len(clusters):
        if sol[idx+len(clusters[clusId])-1] in clusters[clusId]:
            gtspSol.append(sol[idx])
            clustersOrder.append(clusId)
            idx = idx+len(clusters[clusId])
            if sol[idx]!=0:
                clusId = nodeIsInCluster(sol[idx], clusters)
            else:
                print("sol complete")
                solComplete = True
        else:
            print("UNVALID SOLUTION", idx)
            solValid = False
            return
    if solValid and solComplete:
        print("sol valid and complete")
    # WRITE IT TO A FILE
    f = open(resPath, "w")
    for i in range(len(gtspSol)):
        f.write(str(clustersOrder[i])+"\n")
        line = "["
        for v in configs[gtspSol[i]]['q']:
            line+=str(v)
            line+=", "
        f.write(line+"]\n")
    f.close()
    return gtspSol, clustersOrder

def getGTSPsolFromConcordeSol(nbVertices, sol, clusters, configs, resPath):
    """
    \param nbVertices int corresponding to the number of vertices in sol
    \param sol list starting and ending at the depot (0) with the solution of the tsp instance
    \param clusters list of lists with the indices of the nodes in each cluster of the GTSP
    \param configs list of lists with the configuration corresponding to the nodes of the GTSP
    \param resPath string with the location to write the solution at
    \retval gtspSol ordered list of the nodes to effectively visit in the RTSP solution
    \retval clustersOrder list with the order of visit the clusters ie. tasks
    """
    # CHECKING AND REMOVING i+n nodes
    nbConfigs = nbVertices//2
    solValidForTransfo = True
    doneRemoving = False
    if sol[1]>nbConfigs:
        whereToLook = -1
        idx = 2
    else:
        whereToLook = 1
        idx = 1
    print("order : ", whereToLook)
    while solValidForTransfo and (not doneRemoving):
        if sol[idx+whereToLook]==sol[idx]+nbConfigs:
            print(idx, sol[idx], sol[idx+whereToLook])
            sol.pop(idx+whereToLook)
            idx+=1
            doneRemoving = (whereToLook==1 and idx>nbConfigs) or (whereToLook==-1 and idx>nbConfigs+1)
        else:
            print(idx, sol[idx], sol[idx+whereToLook])
            solValidForTransfo = False
    if not doneRemoving:
        print("unvalid for symmetry transformation")
        return
    else:
        print("valid solution for symmetry transformation")
    # return
    # INITIALISE GTSP SOLUTION RETRIEVAL
    gtspSol = list()
    clustersOrder = list()
    idx = 1
    clusId = nodeIsInCluster(sol[idx], clusters)
    solValid = True
    solComplete = False
    # RETRIEVE GTSP ROUTE
    while (not solComplete) and solValid and len(gtspSol)<len(clusters):
        if sol[idx+len(clusters[clusId])-1] in clusters[clusId]:
            gtspSol.append(sol[idx])
            clustersOrder.append(clusId)
            idx = idx+len(clusters[clusId])
            if sol[idx]!=0:
                clusId = nodeIsInCluster(sol[idx], clusters)
            else:
                print("sol complete")
                solComplete = True
        else:
            print("UNVALID SOLUTION", idx)
            solValid = False
            return
    if solValid and solComplete:
        print("sol valid and complete")
    # WRITE IT TO A FILE
    f = open(resPath, "w")
    f.write("nodes :\n")
    for node in gtspSol:
        f.write(str(node)+' ')
    f.write("\nclusters :\n")
    for c in clustersOrder:
        f.write(str(c)+' ')
    # for i in range(len(gtspSol)):
    #     f.write(str(clustersOrder[i])+"\n")
    #     line = "["
    #     for v in configs[gtspSol[i]]['q']:
    #         line+=str(v)
    #         line+=", "
    #     f.write(line+"]\n")
    f.close()
    return gtspSol, clustersOrder


### COMPUTATION OF EXACT COSTS

def nodeIsInCluster(node, clusters):
    clusIdx = 0
    nbClus = len(clusters)
    while clusIdx<nbClus and node not in clusters[clusIdx]:
        clusIdx+=1
    if clusIdx==nbClus:
        print("node not found")
    else:
        return clusIdx

def computeCostWithBase(q1, q2, armPlanner, basePlanner, graph, ps):
    """
    \param q1 list with the configuration to start at
    \param q2 list with the configuration to go to
    \param armPlanner InStatePlanner instance
    \param basePlanner InStatePlanner instance
    \param graph ConstraintGraph instance
    \param ps ProblemSolver instance
    \retval baseCost base movement time to go from q1 to q2
    \retval armCost arm movement time to go from q1 to q2
    """
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

def computeArmOnlyCost(q1, q2, armPlanner, ps):
    """
    \param q1 list with the configuration to start at
    \param q2 list with the configuration to go to
    \param armPlanner InStatePlanner instance
    \param ps ProblemSolver instance
    \retval armCost arm movement time to go from q1 to q2
    """
    try:
        p1 = wd(armPlanner.computePath(q1,[q2]))
    except:
        raise Exception("collision during path planning")
    else:
        ps.client.basic.problem.addPath(p1)
        armCost = ps.pathLength(0)
        ps.erasePath(0)
    return armCost
