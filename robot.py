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

from hpp.corbaserver import shrinkJointRange
from hpp.corbaserver.manipulation import Robot as Parent
from hpp.corbaserver.manipulation import loadServerPlugin, newProblem, CorbaClient

class Robot(Parent):
    urdfString = ""
    srdfString = ""
    def __init__(self, urdfFile, userArgs):
        # urdf of the parts
        with open(urdfFile, 'r') as f:
            for line in f:
                self.urdfString += line
        # create corba client
        loadServerPlugin (userArgs.context, "manipulation-corba.so")
        newProblem()
        self.client = CorbaClient(context=userArgs.context)
        self.client.basic._tools.deleteAllServants()
        # create robot
        super().__init__("robot", "tiago", rootJointType="planar", client=self.client)
        # free node
        self.free = "tiago/gripper grasps driller/handle"
        self.loop_free = 'Loop | 0-0'
        pass
    def setNeutralPosition(self):
        crobot = self.hppcorba.problem.getProblem().robot()
        self.qneutral = crobot.neutralConfiguration()
        self.qneutral[self.rankInConfiguration['tiago/hand_thumb_abd_joint']] = 1.5707
        self.qneutral[self.rankInConfiguration['tiago/hand_index_abd_joint']]  = 0.35
        self.qneutral[self.rankInConfiguration['tiago/hand_middle_abd_joint']] = -0.1
        self.qneutral[self.rankInConfiguration['tiago/hand_ring_abd_joint']]   = -0.2
        self.qneutral[self.rankInConfiguration['tiago/hand_little_abd_joint']] = -0.35
        removedJoints = [
            'tiago/caster_back_left_1_joint',
            'tiago/caster_back_left_2_joint',
            'tiago/caster_back_right_1_joint',
            'tiago/caster_back_right_2_joint',
            'tiago/caster_front_left_1_joint',
            'tiago/caster_front_left_2_joint',
            'tiago/caster_front_right_1_joint',
            'tiago/caster_front_right_2_joint',
            'tiago/suspension_left_joint',
            'tiago/wheel_left_joint',
            'tiago/suspension_right_joint',
            'tiago/wheel_right_joint',

            # Comment this 3 joints otherwise sot is not happy
            #'tiago/hand_index_joint',
            #'tiago/hand_mrl_joint',
            #'tiago/hand_thumb_joint',

            'tiago/hand_index_abd_joint',
            'tiago/hand_index_virtual_1_joint',
            'tiago/hand_index_flex_1_joint',
            'tiago/hand_index_virtual_2_joint',
            'tiago/hand_index_flex_2_joint',
            'tiago/hand_index_virtual_3_joint',
            'tiago/hand_index_flex_3_joint',
            'tiago/hand_little_abd_joint',
            'tiago/hand_little_virtual_1_joint',
            'tiago/hand_little_flex_1_joint',
            'tiago/hand_little_virtual_2_joint',
            'tiago/hand_little_flex_2_joint',
            'tiago/hand_little_virtual_3_joint',
            'tiago/hand_little_flex_3_joint',
            'tiago/hand_middle_abd_joint',
            'tiago/hand_middle_virtual_1_joint',
            'tiago/hand_middle_flex_1_joint',
            'tiago/hand_middle_virtual_2_joint',
            'tiago/hand_middle_flex_2_joint',
            'tiago/hand_middle_virtual_3_joint',
            'tiago/hand_middle_flex_3_joint',
            'tiago/hand_ring_abd_joint',
            'tiago/hand_ring_virtual_1_joint',
            'tiago/hand_ring_flex_1_joint',
            'tiago/hand_ring_virtual_2_joint',
            'tiago/hand_ring_flex_2_joint',
            'tiago/hand_ring_virtual_3_joint',
            'tiago/hand_ring_flex_3_joint',
            'tiago/hand_thumb_abd_joint',
            'tiago/hand_thumb_virtual_1_joint',
            'tiago/hand_thumb_flex_1_joint',
            'tiago/hand_thumb_virtual_2_joint',
            'tiago/hand_thumb_flex_2_joint',
        ]
        crobot.removeJoints(removedJoints, self.qneutral)
        del crobot
    def readSRDF(self):
        self.insertRobotSRDFModel("tiago", "package://tiago_data/srdf/tiago.srdf")
        self.insertRobotSRDFModel("tiago", "package://tiago_data/srdf/pal_hey5_gripper.srdf")
        self.setJointBounds('tiago/root_joint', [-3, 3, -3, 3])
        self.insertRobotSRDFModel("driller", "package://gerard_bauzil/srdf/qr_drill.srdf")
        self.setJointBounds('driller/root_joint', [-3, 3, -3, 3, 0, 2])
        self.setJointBounds('part/root_joint', [-2, 2, -2, 2, -2, 2])
    def disableCollisions(self):
        srdf_disable_collisions_fmt = """  <disable_collisions link1="{}" link2="{}" reason=""/>\n"""
        # Disable collision between tiago/hand_safety_box_0 and driller
        srdf_disable_collisions = """<robot>"""
        srdf_disable_collisions += srdf_disable_collisions_fmt.format("tiago/hand_safety_box", "driller/base_link")
        srdf_disable_collisions += srdf_disable_collisions_fmt.format("tiago/arm_6_link", "tiago/hand_safety_box")
        srdf_disable_collisions += srdf_disable_collisions_fmt.format("tiago/arm_6_link", "tiago/wrist_ft_tool_link")
        # Disable autocollisions
        linka, linkb, enabled = self.hppcorba.robot.autocollisionPairs()
        for la, lb, en in zip(linka, linkb, enabled):
            if not en: continue
            disable = False
            # Disable collision between caster wheels and anything else
            if not disable:
                disable = ('caster' in la or 'caster' in lb)
            # Disable collision between tiago/hand (except hand_safety_box_0) and all other tiago links
            if not disable:
                hand_vs_other = False
                for l in [la, lb]:
                    if l.startswith("tiago/hand_") and l != "tiago/hand_safety_box_0":
                        disable = True
                        break
            # Disable collision between driller/* and tiago/arm_[1234567]_link
            if not disable:
                for l0, l1 in [ (la, lb), (lb, la) ]:
                    if l0.startswith("driller/") and l1.startswith("tiago/arm_") and l1.endswith("_link_0") and l1[10] in "1234567":
                        disable = True
                        break
            if disable:
                srdf_disable_collisions += srdf_disable_collisions_fmt.format(la[:la.rfind('_')], lb[:lb.rfind('_')])
        srdf_disable_collisions += "</robot>"
        self.client.manipulation.robot.insertRobotSRDFModelFromString("", srdf_disable_collisions)
    def getHandlesCoords(self, handlesNames):
        return [self.getHandlePositionInJoint(handle)[1] for handle in handlesNames]
    def addVirtualHandles(self, virtualHandlesCoords):
        for i in range(len(virtualHandlesCoords)):
            self.client.manipulation.robot.addHandle('part/base_link', "part/virtual_"+str(i),
                                                     virtualHandlesCoords[i], 0.05,
                                                     [True,True,True,False,True,True])
    def addExtraVirtualHandles(self, idxOffset, virtualHandlesCoords):
        for i in range(len(virtualHandlesCoords)):
            self.client.manipulation.robot.addHandle('part/base_link', "part/virtual_"+str(i+idxOffset),
                                                     virtualHandlesCoords[i], 0.05,
                                                     [True,True,True,False,True,True])
    def defineVariousJointBounds(self):
        self.joint_bounds = {}
        jointsToShrink = list(filter(lambda jn:jn.startswith("tiago/torso") or
                             jn.startswith("tiago/arm") or jn.startswith("tiago/head"),
                            self.jointNames))
        self.joint_bounds["default"] = [ (jn, self.getJointBounds(jn)) for jn in self.jointNames ]
        shrinkJointRange(self, jointsToShrink, 0.6)
        self.joint_bounds["grasp-generation"] = [ (jn, self.getJointBounds(jn)) for jn in self.jointNames ]
        for jn, bound in self.joint_bounds["default"]:
            self.setJointBounds(jn, bound)
        shrinkJointRange(self, jointsToShrink, 0.95)
        self.joint_bounds["planning"] = [ (jn, self.getJointBounds(jn)) for jn in self.jointNames ]
    def setStartingPosition(self):
        self.q0 = self.getCurrentConfig()
        self.q0[:4] = [0, -0.9, 0, 1]
        #self.q0[self.rankInConfiguration['tiago/torso_lift_joint']] = 0.15
        self.q0[self.rankInConfiguration['tiago/torso_lift_joint']] = 0.34
        self.q0[self.rankInConfiguration['tiago/arm_1_joint']] = 0.10
        self.q0[self.rankInConfiguration['tiago/arm_2_joint']] = -1.47
        self.q0[self.rankInConfiguration['tiago/arm_3_joint']] = -0.16
        self.q0[self.rankInConfiguration['tiago/arm_4_joint']] = 1.87
        self.q0[self.rankInConfiguration['tiago/arm_5_joint']] = -1.57
        self.q0[self.rankInConfiguration['tiago/arm_6_joint']] = 1.3
        self.q0[self.rankInConfiguration['tiago/arm_7_joint']] = 0.00
        self.q0[self.rankInConfiguration['part/root_joint']:] = [0,0.3,0.8,0,0,1,0]
