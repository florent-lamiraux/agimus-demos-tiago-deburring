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

import hpp_idl
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraphFactory, ConstraintGraph, Rule, Constraints


def lockJoint(robot, ps, jname, cname=None, constantRhs=True):
    if cname is None:
        cname = jname
    s = robot.rankInConfiguration[jname]
    e = s+robot.getJointConfigSize(jname)
    ps.createLockedJoint(cname, jname, robot.q0[s:e])
    ps.setConstantRightHandSide(cname, constantRhs)
    return cname

# \param ps ProblemSolver instance,
# \param handle name of the handle: should be "part/handle_i" where i is an
#        integer,
# \param graph constraint graph
def addAlignmentConstrainttoEdge(ps, graph, allHandles, handle, tool_gripper):
    #recover id of handle
    handleId = allHandles.index(handle)
    J1, gripperPose = ps.robot.getGripperPositionInJoint(tool_gripper)
    J2, handlePose = ps.robot.getHandlePositionInJoint(handle)
    T1 = Transform(gripperPose)
    T2 = Transform(handlePose)
    constraintName = handle + '/alignment'
    ps.client.basic.problem.createTransformationConstraint2\
        (constraintName, J1, J2, T1.toTuple(), T2.toTuple(),
         [False, True, True, False, False, False])
    # Set constraint
    edgeName = tool_gripper + ' > ' + handle + ' | 0-0_12'
    graph.addConstraints(edge = edgeName, constraints = \
                         Constraints(numConstraints=[constraintName]))
    edgeName = tool_gripper + ' < ' + handle + ' | 0-0:1-{}_21'.format(handleId)
    graph.addConstraints(edge = edgeName, constraints = \
                         Constraints(numConstraints=[constraintName]))

def createConstraints(ps, robot):
    # joint list
    ljs = list()
    ps.createLockedJoint("tiago_base", "tiago/root_joint", [0,0,1,0])
    lockJoint(robot, ps, "part/root_joint", "lock_part", constantRhs=False)
    c_lock_part = ps.hppcorba.problem.getConstraint("lock_part")
    # fill joint list
    for n in robot.jointNames:
        if n.startswith('tiago/gripper_') or n.startswith('tiago/hand_'):
            ljs.append(lockJoint(robot, ps, n))
    # ARM
    lock_arm = [ lockJoint(robot, ps, n) for n in robot.jointNames
                 if n.startswith("tiago/arm") or n.startswith('tiago/torso')]
    # HEAD
    lock_head = [ lockJoint(robot, ps, n) for n in robot.jointNames
                  if n.startswith("tiago/head")]
    # "LOOK AT GRIPPER"
    # for (X,Y,Z) the position of the gripper in the camera frame, (X, Y) = 0 and Z >= 0
    tool_gripper = "driller/drill_tip"
    ps.createPositionConstraint("look_at_gripper", "tiago/xtion_rgb_optical_frame", tool_gripper,
                                (0,0,0), (0,0,0), (True,True,True))
    look_at_gripper = ps.hppcorba.problem.getConstraint("look_at_gripper")
    look_at_gripper.setComparisonType([hpp_idl.hpp.EqualToZero,hpp_idl.hpp.EqualToZero,hpp_idl.hpp.Superior])
    return ljs, lock_arm, lock_head, look_at_gripper, tool_gripper

def ConsGraphFactory(robot, ps, allHandles, partHandles,
                     ljs, lock_arm, lock_head, look_at_gripper, tool_gripper):
    graph = ConstraintGraph(robot, 'graph')
    factory = ConstraintGraphFactory(graph)
    factory.setGrippers([ "tiago/gripper", "driller/drill_tip", ])
    # OBJECTS
    factory.setObjects([ "driller", "part", ], [ [ "driller/handle", ], partHandles, ], [ [], [] ])
    # RULES
    factory.setRules([
        # Forbid driller to grasp itself.
    Rule([ "driller/drill_tip", ], [ "driller/handle", ], False),
        # Tiago always holds the gripper.
    Rule([ "tiago/gripper", ], [ "", ], False), Rule([ "tiago/gripper", ], [ "part/.*", ], False), 
        # Allow to associate drill_tip with part holes only.
    Rule([ "tiago/gripper", "driller/drill_tip", ], [ "driller/handle", ".*", ], True), ])
    factory.generate()
    # FREE NODE
    graph.addConstraints(graph=True, constraints=Constraints(numConstraints=ljs))
    # other nodes
    for n in graph.nodes.keys():
        if n == robot.free: continue
        graph.addConstraints(node=n,
                             constraints=Constraints(numConstraints=["look_at_gripper"]))
    for e in graph.edges.keys():
        graph.addConstraints(edge=e,
                             constraints=Constraints(numConstraints=["tiago_base"]))
    for handle in partHandles: # alignment
        addAlignmentConstrainttoEdge(ps, graph, allHandles, handle, tool_gripper)
        # extra edge for configuration generation
        
        graph.createEdge(nodeFrom="tiago/gripper grasps driller/handle",
                         nodeTo="driller/drill_tip > " + handle + " | 0-0_pregrasp",
                         name="driller/drill_tip pregrasps "+handle,
                         weight=-1,
                         isInNode="tiago/gripper grasps driller/handle") # create the edge
        graph.addConstraints(edge="driller/drill_tip pregrasps "+handle,
                             constraints=Constraints(numConstraints=["part/root_joint"])) # fix the table
    # HOME NODE
    graph.createNode('home', priority=1000)
    graph.createEdge('home', 'home', 'move_base')
    graph.createEdge('home',  robot.free , 'start_arm', isInNode="home")
    graph.createEdge( robot.free , 'home', 'end_arm', isInNode=robot.free)
    graph.addConstraints(node="home",
                         constraints=Constraints(numConstraints=lock_arm+lock_head + ['tiago/gripper grasps driller/handle', ]))
    graph.addConstraints(edge="end_arm",
                         constraints=Constraints(numConstraints=["tiago_base", "lock_part"]))
    graph.addConstraints(edge="move_base",
                         constraints=Constraints(numConstraints=["lock_part"]))
    graph.addConstraints(edge="start_arm",
                         constraints=Constraints(numConstraints=['tiago/gripper grasps driller/handle', "lock_part"]))
    return graph

def ConsGraphValidation(ps, cgraph):
    graphValidation = ps.client.manipulation.problem.createGraphValidation()
    graphValidation.validate(cgraph)
    if graphValidation.hasErrors():
        print(graphValidation.str())
        print("Graph has infeasibilities")
        sys.exit(1)
    elif graphValidation.hasWarnings():
        print(graphValidation.str())
        print("Graph has only warnings")
    else:
        print("Graph *seems* valid.")
