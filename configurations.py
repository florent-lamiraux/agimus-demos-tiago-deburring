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

from typing import List, Dict
configType = Dict[str, List[float]]
configListType = List[configType]

def shootPregraspConfig(robot, graph, handle: str, restConfig: List[float]) -> configType:
    """
    \param robot Robot instance (c.f. robot.py)
    \param graph ConstraintGraph instance
    \param handle string with the name of the handle ("part/handle_i" or "part/virtual_i")
    \param restConfig list with the rest configuration of the robot
    \retval dict with q the configuration pregrasping handle
    gets a collision-free pregrasp configuration for the given handle
    """
    res = False
    tries = 0
    while (not res) and (tries<20):
        try: # get a config
            tries+=1
            if tries%10==0:
                print("attempt ", tries)
            q = robot.shootRandomConfig()
            res, q, err = graph.generateTargetConfig("driller/drill_tip pregrasps "+handle, restConfig, q)
            if (res) and (robot.isConfigValid(q)[0] is False): # check it is collision-free
                res = False
        except Exception as exc:
            print(exc)
            res = False
            pass
    return {"name": handle+"_pregrasp", "config": q}

def shootPregraspConfigs(robot, graph, handle: str, restConfig: List[float], nbConfigs: int, configList: configListType) -> None:
    """
    \param robot Robot instance (c.f. robot.py)
    \param graph ConstraintGraph instance
    \param handle string with the name of the handle ("part/handle_i" or "part/virtual_i")
    \param restConfig list with the rest configuration of the robot
    \param nbConfigs int of the nb of pregrasp configurations to generate
    \param configList list containing the dicts with the configurations pregrasping the handle
    gets nbConfigs collision-free pregrasp configurations for the given handle
    stores them in configList
    """
    for i in range(nbConfigs):
        configList.append(shootPregraspConfig(robot, graph, handle, restConfig))

def configReachesHandle(robot, graph, handle: str, q: List[float], qName: str, reachMatrix) -> None:
    """
    \param robot Robot instance (c.f. robot.py)
    \param graph ConstraintGraph instance
    \param handle string with the name of the handle ("part/handle_i" or "part/virtual_i")
    \param q list with the configuration of the robot
    \param qName string of the name of the configuration
    \param reachMatrix dict on the handles with for every base config the full config reaching the handle, or None
    checks if base configuration q allows to reach handle
    stores the result in reachMatrix
    """
    try:
        res, qh, err = graph.generateTargetConfig("driller/drill_tip > "+handle+" | 0-0_01", q, q)
        if res and robot.isConfigValid(qh)[0]:
            reachMatrix[handle][qName] = qh
        else:
            reachMatrix[handle][qName] = None
    except:
        reachMatrix[handle][qName] = None

def structureHandleAndConfigData(real_handles, configsOnVirtual, reachesHandle):
    """
    \param real_handles list of the real handles
    \param configsOnVirtual list of the configurations pregrasping virtual handles
    \param reachesHandle reach matrix
    \retval configHandles telling what handle a configuration reaches
    \retval handleConfigs telling what configurations reach a handle
    \retval configClusters list of lists of the nodes of each cluster of the GTSP
    \retval nbConfig total number of configurations generated on real handles
    """
    configHandles, handleConfigs, configClusters = list(), dict(), list()
    nbConfig = 0
    for h in real_handles:
        handleConfigs[h] = list()
        for c in configsOnVirtual:
            if reachesHandle[h][c["name"]] is not None:
                nbConfig+=1
                configHandles.append({"hole":h, "q":reachesHandle[h][c["name"]]})
                handleConfigs[h].append(nbConfig)
        configClusters.append(handleConfigs[h])
    return configHandles, handleConfigs, configClusters, nbConfig

def addExtraData(nbConfig, configHandles, handleConfigs, configClusters,
                 real_handles, configsOnVirtual, reachesHandle):
    """
    \param nbConfig total number of configurations already generated on real handles
    \param configHandles telling what handle a configuration reaches
    \param handleConfigs telling what configurations reach a handle
    \param configClusters list of lists of the nodes of each cluster of the GTSP
    \param real_handles list of the real handles
    \param configsOnVirtual list of the configurations pregrasping virtual handles
    \param reachesHandle reach matrix
    \retval nbConfig total number of configurations generated on real handles
    further fills configHandles,handleConfigs,configClusters with extra configurations on real handles
    """
    for h in range(len(real_handles)):
        handle = real_handles[h]
        for c in configsOnVirtual:
            if reachesHandle[handle][c["name"]] is not None:
                nbConfig+=1
                configHandles.append({"hole":handle, "q":reachesHandle[handle][c["name"]]})
                handleConfigs[handle].append(nbConfig)
        configClusters[h] = handleConfigs[handle]
    return nbConfig

def reachesHandles(robot, graph, real_handles, configsOnVirtual):
    """
    \param robot Robot instance (c.f. robot.py)
    \param graph ConstraintGraph instance
    \param real_handles list of all real handles
    \param configsOnVirtual list of base configurations on virtual handles
    \retval reachMatrix matrix telling for each real handle, which base configuration from configsOnVirtual reaches it 
    """
    reachMatrix = dict()
    for handle in real_handles:
        reachMatrix[handle] = {}
        for c in configsOnVirtual:
            configReachesHandle(robot, graph, handle, c["config"], c["name"], reachMatrix)
    return reachMatrix
