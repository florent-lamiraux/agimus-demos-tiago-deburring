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

from CORBA import Any, TC_long, TC_float
from hpp.corbaserver import wrap_delete as wd
from hpp.corbaserver.manipulation import createContext, ProblemSolver, ConstraintGraph, Rule, Constraints, loadServerPlugin
from hpp.gepetto.manipulation import ViewerFactory
from hpp_idl.hpp import Error as HppError
from hpp.corbaserver.task_sequencing import Client as SolverClient
import pinocchio
from agimus_demos import InStatePlanner
import sys, argparse, numpy as np, time
import operator
from copy import deepcopy
from math import fabs

from robot import Robot
from constraints import *
from resolution import *
from gtsp import *
from display import *
from configurations import *

# PARSE ARGUMENTS
defaultContext = "corbaserver"
p = argparse.ArgumentParser (description=
                             'Initialize demo of Pyrene manipulating a box')
p.add_argument ('--context', type=str, metavar='context',
                default=defaultContext,
                help="identifier of ProblemSolver instance")
p.add_argument ('--n-random-handles', type=int, default=None,
                help="Generate a random model with N handles.")
args = p.parse_args ()
if args.context != defaultContext:
    createContext (args.context)
isSimulation = args.context == "simulation"

# DEFINE CLASSES FOR THE PARTS
class Driller:
    urdfFilename = "package://gerard_bauzil/urdf/driller_with_qr_drill.urdf"
    srdfFilename = "package://gerard_bauzil/srdf/driller.srdf"
    rootJointType = "freeflyer"

class PartP72:
    urdfFilename = "package://agimus_demos/urdf/P72-with-table.urdf"
    srdfFilename = "package://agimus_demos/srdf/P72.srdf"
    rootJointType = "freeflyer"

class Airfoil:
    urdfFilename = "package://agimus_demos/tiago/deburring/urdf/airfoil.urdf"
    srdfFilename = "package://agimus_demos/tiago/deburring/srdf/airfoil.srdf"
    rootJointType = "freeflyer"

# THE ROBOT
robot = Robot('./tiago.urdf', args) # CREATE THE ROBOT
robot.setNeutralPosition() # SET ITS NEUTRAL POSITION

# PROBLEM SOLVER
print("Loading model")
ps = ProblemSolver(robot)
ps.loadPlugin("manipulation-spline-gradient-based.so")

# LOAD THE ROBOT INTO THE GUI
vf = ViewerFactory(ps)
vf.loadRobotModel (Driller, "driller")
# vf.loadRobotModel (PartP72, "part")
vf.loadRobotModel (Airfoil, "part")

robot.readSRDF() # LOAD SRDF DATA AND SET SOME JOINT BOUNDS
robot.disableCollisions() # DISABLE COLLISIONS
robot.setStartingPosition() # SETTING STARTING POSITION



### GENERATE VIRTUAL HANDLES

# RETRIEVE REAL HANDLES
r = robot.rankInConfiguration['part/root_joint']
partPose = Transform(robot.q0[r:r+7])
all_handles = ps.getAvailable('handle')
part_handles = list(filter(lambda x: x.startswith("part/"), all_handles))
holeCoordsInPart = robot.getHandlesCoords(part_handles)
# f = open("/home/hvanoverlo/devel/instanceData/quaternions.txt", "w")
# for c in holeCoordsInPart:
#     for q in c[3:-1]:
#         f.write(str(q)+" ")
#     f.write(str(c[-1])+"\n")
# f.close()

# CLUSTER THEM
iso_c = 5 # expected nb of clusters
iso_nc = 5 # initial nb of clusters
iso_tn = 1 # min nb of points per cluster
iso_te = .05 # max variance
iso_tc = .05 # min distance between centroids
iso_nt = 2 # max merges per iteration
iso_ns = 500 # max iterations
iso_k = 1. # influence of the angle on the task distance
loadServerPlugin("corbaserver", "task-sequencing.so")
s = SolverClient()
print("Clustering tasks")
clusters = s.tools.isoData(holeCoordsInPart, len(holeCoordsInPart), len(holeCoordsInPart[0]), iso_c, iso_nc, iso_tn, iso_te, iso_tc, iso_nt, iso_ns, iso_k)

# GET COORDINATES OF VIRTUAL HANDLES
virtualHandlesOriginInPart, virtualHandlesOriginInWorld = \
    retrieveVirtualHandles(clusters, partPose)

# COORDINATE SYSTEMS (part, world)
# clustering : part
# projection : world
# configuration generation : part

# DISPLAY VIRTUAL HANDLES
try:
    v = vf.createViewer()
except:
    print("Did not find viewer")

redRGB = [1,0,0,1]
greenRGB = [0,1,0,1]
displayHandlesOrigins(v, virtualHandlesOriginInWorld, redRGB, "virt_")

# PROJECT VIRTUAL HANDLES ON THE MESH
def RotationMatrixFromAxis(direction):
    # computation of the rotation matrix with first column corresponding to direction
    # obtained with the Gram-Schmidt orthonormalisation process (3D)
    # original set of vectors
    x = direction/np.linalg.norm(direction)
    y = np.array([0,1,0])
    z = np.array([0,0,1])
    # make it orthonormal
    col1 = x
    if fabs(np.dot(col1,y))<fabs(np.dot(col1,z)): # taking the most orthogonal first
        col2 = y - np.dot(col1,y)*col1
        col2/=np.linalg.norm(col2)
        col3 = np.cross(col1,col2)
        col3/=np.linalg.norm(col3)
    else:
        col2 = z - np.dot(col1,z)*col1
        col2/=np.linalg.norm(col2)
        col3 = np.cross(col1,col2)
        col3/=np.linalg.norm(col3)
    # return the rotation matrix
    eps = 1e-6
    assert(fabs(np.dot(col1, col2))<eps)
    assert(fabs(np.dot(col2, col3))<eps)
    assert(fabs(np.dot(col3, col1))<eps)
    assert(np.dot(col1, col1)-1<eps)
    assert(np.dot(col2, col2)-1<eps)
    assert(np.dot(col3, col3)-1<eps)
    R = np.column_stack( (col1,col2,col3) )
    return R


print("Projecting virtual handles")
virtualHandlesOnSurfaceInWorld = list()
q0 = [robot.q0[i] for i in range(len(robot.q0))]
for vhIdx in range(len(virtualHandlesOriginInWorld)):
    dist, closest = s.tools.distanceToMesh(q0, virtualHandlesOriginInWorld[vhIdx])
    direction = np.array(virtualHandlesOriginInWorld[vhIdx]) - np.array(closest)
    rotationMatrix = RotationMatrixFromAxis(direction)
    direction = pinocchio.Quaternion(rotationMatrix)
    virtualHandlesOnSurfaceInWorld.append(closest+list(direction.coeffs()))

# ADD PROJECTIONS TO THE MODEL
virtualHandlesOnSurfaceInPart = fromWorldToPart(virtualHandlesOnSurfaceInWorld, partPose)
robot.addVirtualHandles(virtualHandlesOnSurfaceInPart)
displayHandlesFrame(v, virtualHandlesOnSurfaceInWorld, greenRGB, "proj_")



### SETTING A FEW VARIABLES AND PARAMETERS
print("Setting some parameters")

# SETTING JOINT BOUNDS
robot.defineVariousJointBounds()

# HPP PARAMETERS
ps.selectPathValidation("Graph-Discretized", 0.05)
#ps.selectPathValidation("Graph-Dichotomy", 0)
#ps.selectPathValidation("Graph-Progressive", 0.02)
ps.selectPathProjector("Progressive", 0.2)
ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
ps.setParameter("SimpleTimeParameterization/safety", 0.25)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)
ps.setParameter("ManipulationPlanner/extendStep", 0.7)
ps.setParameter("SteeringMethod/Carlike/turningRadius", 0.05)



### DEFINING CONSTRAINTS
ljs, lock_arm, lock_head, look_at_gripper, tool_gripper = createConstraints(ps, robot)


### BUILDING THE CONSTRAINT GRAPH

# ADDING VIRTUAL HANDLES
print("Adding virtual handles")
virtual_handles = []
for hole in range(len(clusters)):
    part_handles.append("part/virtual_{}".format(str(hole)))
    all_handles.append("part/virtual_{}".format(str(hole)))
    virtual_handles.append("part/virtual_{}".format(str(hole)))

# CONSTRAINT GRAPH FACTORY
print("Building graph")
graph = ConsGraphFactory(robot, ps, all_handles, part_handles,
                         ljs, lock_arm, lock_head, look_at_gripper, tool_gripper)
# INITIALIZATION
cproblem = ps.hppcorba.problem.getProblem()
cgraph = cproblem.getConstraintGraph()
graph.initialize()
# a little test
res, robot.q0, err = graph.generateTargetConfig('move_base', robot.q0, robot.q0)
assert(res)
# GRAPH VALIDATION
ConsGraphValidation(ps, cgraph)



### PROBLEM RESOLUTION

# FOV FILTER
res, q, err = graph.applyNodeConstraints(robot.free, robot.q0)
assert res
robot.setCurrentConfig(q)
oMh, oMd = robot.hppcorba.robot.getJointsPosition(q, ["tiago/hand_tool_link", "driller/base_link"])

# INSTATEPLANNER
armPlanner = createArmPlanner(ps, graph, robot)
basePlanner = createBasePlanner(ps, graph, robot)



### CONFIGURATION GENERATION
print("Shooting configurations on virtual handles")
configsOnVirtual = list()
nbConfigsPerVHandle = 2 # 10
for handle in virtual_handles:
    shootPregraspConfigs(robot, graph, handle, robot.q0, nbConfigsPerVHandle, configsOnVirtual)
idOfVirtualHandle = 0
for config in configsOnVirtual:
    config["name"] = "qOnVirtual_"+str(idOfVirtualHandle)
    idOfVirtualHandle+=1


### CHECK REACHABILITY OF REAL HANDLES
print("Checking reachability")
real_handles = list(filter(lambda x : not x.startswith("part/virtual"), part_handles))
reachesHandle = reachesHandles(robot, graph, real_handles, configsOnVirtual)

# STRUCTURING THE DATA
print("Structuring the data")
configHandles, handleConfigs, configClusters, nbConfig = \
    structureHandleAndConfigData(real_handles, configsOnVirtual, reachesHandle)


### MANAGING UNREACHED TASKS
# get handles that are not reached yet
unreached_handles = list()
for h in range(len(configClusters)):
    if configClusters[h]==[]:
        unreached_handles.append(real_handles[h])
print("***", len(unreached_handles), " tasks not attained***")
if len(unreached_handles)>0:
    # cluster them (shrink cluster parameters)
    print("Clustering unattained tasks")
    unreachedHolesCoordsInPart = robot.getHandlesCoords(unreached_handles)
    iso_te/=2 # max variance
    iso_tc/=2 # min distance between centroids
    clustersOnUnreached = s.tools.isoData(unreachedHolesCoordsInPart,
                                          len(unreachedHolesCoordsInPart),
                                          len(unreachedHolesCoordsInPart[0]),
                                          iso_c, iso_nc, iso_tn, iso_te, iso_tc,
                                          iso_nt, iso_ns, iso_k)
    # retrieve virtual handles
    virtualHandlesBisOriginInPart, virtualHandlesBisOriginInWorld = \
        retrieveVirtualHandles(clustersOnUnreached, partPose)
    virtual_handlesBis = []
    for hole in range(len(clusters), len(clusters)+len(clustersOnUnreached)):
        part_handles.append("part/virtual_{}".format(str(hole)))
        all_handles.append("part/virtual_{}".format(str(hole)))
        virtual_handlesBis.append("part/virtual_{}".format(str(hole)))
    # display the extra virtual handles
    fadedRedRGB = [1,0,0,.5]
    fadedGreenRGB = [0,1,0,.5]
    displayHandlesOrigins(v, virtualHandlesBisOriginInWorld, fadedRedRGB, "virt_bis_")
    # project extra virtual handles on the mesh
    print("Projecting extra virtual handles")
    virtualHandlesBisOnSurfaceInWorld = list()
    for vhIdx in range(len(virtualHandlesBisOriginInWorld)):
        dist, closest = s.tools.distanceToMesh(q0, virtualHandlesBisOriginInWorld[vhIdx])
        direction = np.array(virtualHandlesBisOriginInWorld[vhIdx]) - np.array(closest)
        rotationMatrix = RotationMatrixFromAxis(direction)
        direction = pinocchio.Quaternion(rotationMatrix)
        virtualHandlesBisOnSurfaceInWorld.append(closest+list(direction.coeffs()))
    # add extra handles to the model
    virtualHandlesBisOnSurfaceInPart = fromWorldToPart(virtualHandlesBisOnSurfaceInWorld, partPose)
    robot.addExtraVirtualHandles(len(clusters), virtualHandlesBisOnSurfaceInPart)
    # display extra projected virtual handles
    displayHandlesFrame(v, virtualHandlesBisOnSurfaceInWorld, fadedGreenRGB, "proj_bis_")
    # add edges and nodes to the constraint graph
    print("Adding extra virtual handles to constraint graph")
    addExtraHandles(graph, len(clusters), len(clustersOnUnreached))
    # shoot configurations
    print("Shooting configurations on extra virtual handles")
    configsOnVirtualBis = list()
    for handle in virtual_handlesBis:
        shootPregraspConfigs(robot, graph, handle, robot.q0, nbConfigsPerVHandle, configsOnVirtualBis)
    for config in configsOnVirtualBis:
        config["name"] = "qOnVirtual_"+str(idOfVirtualHandle)
        idOfVirtualHandle+=1
    # reachability from new base configurations
    print("Checking reachability from extra base configurations")
    reachesHandle_bis = reachesHandles(robot, graph, real_handles, configsOnVirtualBis)
    # adding the obtained extra full configurations to those already generated
    print("Structuring the extra data")
    nbConfig = addExtraData(nbConfig, configHandles, handleConfigs, configClusters,
                 real_handles, configsOnVirtualBis, reachesHandle_bis)

    
### LAST RECOURSE FOR STILL UNREACHED TASKS
# get handles that are not reached yet
unreached_handles_bis = list()
for h in range(len(configClusters)):
    if configClusters[h]==[]:
        unreached_handles_bis.append(real_handles[h])
print("***", len(unreached_handles_bis), " tasks still not attained***")
print("Shooting configurations on still unreached handles")
configsOnStillUnreached = list()
for handle in unreached_handles_bis:
    data = shootPregraspConfig(robot, graph, str(handle), robot.q0)
    data["name"] = str(handle)
    configsOnStillUnreached.append(data)
# check if the new configurations reach other handles
print("Checking reachability from new configurations")
reachesHandle_ter = reachesHandles(robot, graph, real_handles, configsOnStillUnreached)
# adding the obtained configurations to those already generated
print("Structuring the extra data")
nbConfig = addExtraData(nbConfig, configHandles, handleConfigs, configClusters,
                 real_handles, configsOnStillUnreached, reachesHandle_ter)

# getting the data to compute the distance matrix
clustersForDistances = deepcopy(configClusters)
maxClusterSizeIdx = np.argmax([len(c) for c in clustersForDistances])
maxClusterSize = len(clustersForDistances[maxClusterSizeIdx])
for c in clustersForDistances:
    while len(c)<maxClusterSize:
        c.append(-1)


print("Storing clusters and configurations")
f = open("./instanceData/clustersbis.txt", "w")
for task in configClusters:
    for config in task[:-1]:
        f.write(str(config)+" ")
    f.write(str(task[-1])+"\n")
f.close()
f = open("./instanceData/configurationsbis.txt", "w")
for config in configHandles:
    for val in config['q'][:-1]:
        f.write(str(val)+" ")
    f.write(str(config['q'][-1])+"\n")
f.close()



### GTSP

import yaml
with open("./instanceData/jointVelocities.yaml", "r") as stream:
    try:
        jointSpeedsYAML = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)
jointSpeeds = list()
for elt in jointSpeedsYAML:
    jointSpeeds.append(float(jointSpeedsYAML[elt]))

print("Computing the distance matrix")
armJoints = list(filter(lambda s:s.startswith("tiago/arm"), robot.jointNames))
armIndices = list(map(lambda j:robot.rankInConfiguration[j], armJoints))
armIndices.append(robot.rankInConfiguration['tiago/torso_lift_joint'])
s.tools.setRobotArmIndices(int(np.min(armIndices)),int(np.max(armIndices)-np.min(armIndices)+1))
configs = [c['q'] for c in configHandles]
distances = s.tools.computeDistances(configs, clustersForDistances, jointSpeeds, robot.q0)

# print("Storing distances")
# f = open("/home/hvanoverlo/devel/instanceData/distances.txt", "w")
# for row in distances:
#     for col in row[:-1]:
#         f.write(str(col)+" ")
#     f.write(str(row[-1])+"\n")
# f.close()

# ROUTING MODEL
# Nota : the gtsp is solved here (either from an initial solution, or not)
# The executed code is in gtsp.py
# print("solving the GTSP with no initial solution")
# gtspData1, firstSol = firstGTSPround(distances)
# print("solving the GTSP from an initial solution")
# gtspData0 = create_solver_instance(distances)
# sol0 = list()
# for c in configClusters:
#     sol0+=c
# gtspData1, firstSol = GTSPiteration(gtspData0, sol0)
# print("second run of the solver)
# gtspData2, secondSol = GTSPiteration(gtspData1, firstSol)

### MANIPULATE SOLUTION

# gtspSol, clustersOrder = getGTSPsolFromOrtoolsSol(secondSol, configClusters, configHandles, "/home/hvanoverlo/devel/instanceData/P72rtspSol.txt")
# gtspSol, clustersOrder = getGTSPsolFromOrtoolsSol(secondSol, configClusters, configHandles, "/home/hvanoverlo/devel/instanceData/AirfoilRTSPsol.txt")

# nbVertices, concordeSol = LKHsolFromFile("/home/hvanoverlo/Téléchargements/matrixbis-lkh.sol")
# clusters = clustersFromFile("/home/hvanoverlo/devel/instanceData/clustersbis.txt")
# configurations = configsFromFile("/home/hvanoverlo/devel/instanceData/configurationsbis.txt")
# gtspSol, clustersOrder = getGTSPsolFromConcordeSol(nbVertices, concordeSol, clusters, configurations, "/home/hvanoverlo/devel/instanceData/concordeSolbis.txt")




# USELESS ???
# \param fullConfigs list of the configurations to get the base part of
# \param restConfig rest configuration of the robot
# \rval baseConfigs list of the configurations where the arm is set to its retracted position
def getBaseConfigs(fullConfigs: configListType, restConfig: configType) -> configListType:
    baseConfigs = [restConfig.copy()]*len(fullConfigs)
    for q in baseConfigs:
        for i in range(4):
            q[i] = fullConfigs[i]
    return baseConfigs


# generating a path starting from q0 and pregrasping every hole
# print("directly generating a pregrasp config and going to it")
def generatePathToHandle(handle, paths, initRest, initPregrasp):
    res, q2, err = graph.generateTargetConfig('driller/drill_tip > '+handle+' | 0-0_01', initPregrasp, initPregrasp)
    if res:
        try:
            p1 = wd(armPlanner.computePath(initPregrasp,[q2]))
        except:
            res = False
            # raise Exception("collision during path planning")
        else:
            print("collision-free paths found")
            paths.append(p1)
            return initRest, q2
    else:
        tries = 0
        while res is False and tries<100:
            try:
                tries+=1
                if tries%10==0:
                    print("attempt ", tries)
                q2 = robot.shootRandomConfig()
                res, q2, err = graph.generateTargetConfig("driller/drill_tip pregrasps "+handle, initRest, q2)
                if res:
                    # get the base configuration of the pregrasp configuration
                    q1 = initRest[:]
                    for i in range(4):
                        q1[i] = q2[i]
                    res, q1, err = graph.generateTargetConfig('move_base', initRest, q1)
                    # ps.addConfigToRoadmap(q1)
                    try:
                        p1 = wd(armPlanner.computePath(initPregrasp,[initRest]))
                        p2 = wd(basePlanner.computePath(initRest,[q1]))
                        p3 = wd(armPlanner.computePath(q1,[q2]))
                    except:
                        res = False
                        raise Exception("collision during path planning")
                    else:
                        print("collision-free paths found")
            except:
                res = False
                pass
            else:
                print("path generation successfull")
                paths.append(p1)
                paths.append(p2)
                paths.append(p3)
                return q1, q2

def generateFirstPath(paths, handle, initRest):
    res = False
    tries = 0
    while res is False and tries<100:
        try:
            tries+=1
            if tries%10==0:
                print("attempt ", tries)
            q2 = robot.shootRandomConfig()
            res, q2, err = graph.generateTargetConfig("driller/drill_tip pregrasps "+handle, initRest, q2)
            if res:
                # get the base configuration of the pregrasp configuration
                q1 = initRest[:]
                for i in range(4):
                    q1[i] = q2[i]
                res, q1, err = graph.generateTargetConfig('move_base', initRest, q1)
                # ps.addConfigToRoadmap(q1)
                try:
                    p1 = wd(basePlanner.computePath(initRest,[q1]))
                    p2 = wd(armPlanner.computePath(q1,[q2]))
                    p3 = wd(armPlanner.computePath(q2,[q1]))
                except:
                    res = False
                    raise Exception("collision during path planning")
                else:
                    print("collision-free paths found")
        except:
            res = False
            pass
        else:
            print("path generation successfull")
    paths.append(p1)
    paths.append(p2)
    paths.append(p3)
    return q1, q2


# ### PARSE JOINT SPEEDS
# from xml.dom import minidom
# import yaml
# # names of the joints
# robotJoints = robot.jointNames
# robotJoints = robotJoints[:-2]
# for j in range (len(robotJoints)):
#     robotJoints[j] = robotJoints[j][6:]
# # parse the urdf
# tiago = minidom.parse("tiago.urdf")
# data = tiago.documentElement
# # get the max velocities
# maxVelocities = {}
# for joint in data.getElementsByTagName("joint"):
#     if joint.attributes['name'].value in robotJoints:
#         limits = joint.getElementsByTagName('limit')
#         limits = limits[0] if len(limits) > 0 else None
#         if limits is not None and limits.hasAttribute('velocity'):
#             vmax = limits.getAttribute('velocity')
#             maxVelocities.update({joint.attributes['name'].value : vmax})
# # put them in a yaml
# with open('/home/hvanoverlo/devel/jointVelocities.yaml', 'w') as file:
#     documents = yaml.dump(maxVelocities, file)



# ### SOME MORE STUFF
# print(graph.displayEdgeConstraints(name of edge))
# pour recuperer l'erreur sur chaque composante entre q et la feuille souhaitee
# graph.getConfigErrorForEdgeLeaf(edgeId, leafConfig, q)

# InStatePlanner is at devel/hpp/src/agimus-demos/src/agimus_demos
# ccs = basePlanner.croadmap.getConnectedComponents()
