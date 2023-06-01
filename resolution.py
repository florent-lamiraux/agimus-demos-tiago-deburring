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

import security_margins
from os import getcwd, path
from CORBA import Any, TC_long, TC_float
from hpp.corbaserver import wrap_delete as wd
from hpp.corbaserver.manipulation import createContext, ProblemSolver, ConstraintGraph, Rule, Constraints, loadServerPlugin
from agimus_demos import InStatePlanner


def createArmPlanner(ps, graph, robot):
    armPlanner = InStatePlanner (ps, graph)
    armPlanner.setEdge(robot.loop_free)
    #armPlanner.optimizerTypes = [ "SplineGradientBased_bezier3", ]
    armPlanner.optimizerTypes = [ ]
    armPlanner.cproblem.setParameter("SimpleTimeParameterization/safety",
                                     Any(TC_float, 0.25))
    armPlanner.cproblem.setParameter("SimpleTimeParameterization/order",
                                     Any(TC_long, 2))
    armPlanner.cproblem.setParameter("SimpleTimeParameterization/maxAcceleration",
                                     Any(TC_float, 1.0))
    armPlanner.maxIterPathPlanning = 600
    armPlanner.timeOutPathPlanning = 10.
    # Set collision margin between mobile base and the rest because the collision model is not correct.
    bodies = ("tiago/torso_fixed_link_0", "tiago/base_link_0")
    cfgVal = armPlanner.cproblem.getConfigValidations()
    pathVal = armPlanner.cproblem.getPathValidation()
    for _, la, lb, _, _ in zip(*robot.distancesToCollision()):
        if la in bodies or lb in bodies:
            cfgVal.setSecurityMarginBetweenBodies(la, lb, 0.07)
            pathVal.setSecurityMarginBetweenBodies(la, lb, 0.07)
    del cfgVal
    del pathVal
    return armPlanner

def createBasePlanner(ps, graph, robot):
    InStatePlanner.pathProjectorType = None
    InStatePlanner.parameters['kPRM*/numberOfNodes'] = Any(TC_long, 500)
    basePlanner = InStatePlanner (ps, graph)
    basePlanner.plannerType = "kPRM*"
    basePlanner.maxIterPathPlanning = 100000
    basePlanner.optimizerTypes = list()
    basePlanner.setEdge("move_base")
    basePlanner.setReedsAndSheppSteeringMethod()
    # security margins
    smBase = security_margins.SecurityMargins(robot.jointNames)
    margin = 0.1
    for i in [ 0, smBase.jid("part/root_joint") ]:
        smBase.margins[i,:] = margin
        smBase.margins[:,i] = margin
    basePlanner.cproblem.setSecurityMargins(smBase.margins.tolist())
    # empty roadmap
    basePlanner.createEmptyRoadmap()
    return basePlanner

def getMobileBaseRoadmap(basePlanner):
    roadmap_file = getcwd() + "/roadmap-hpp.bin"
    if path.exists(roadmap_file):
        print("Reading mobile base roadmap", roadmap_file)
        basePlanner.readRoadmap(roadmap_file)
    else:
        print("Building mobile base roadmap")
        try:
            basePlanner.cproblem.setParameter('kPRM*/numberOfNodes', Any(TC_long,300))
            basePlanner.buildRoadmap(q0)
            #sm.margins[:,:] = 0.
            #basePlanner.cproblem.setSecurityMargins(sm.margins.tolist())
        except HppError as e:
            print(e)
        print("Writing mobile base roadmap", roadmap_file)
        basePlanner.writeRoadmap(roadmap_file)
