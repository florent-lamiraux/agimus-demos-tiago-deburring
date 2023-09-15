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

from hpp import Transform
import pinocchio

def displayHandle(v, name, coords, rgba):
    """
    \param v viewer from the ViewerFactory
    \param name string with the name of the handle to display
    \param coords list with the coordinates of the handle to display
    \param rgba list with the RGBA of the origin of the displayed handle
    nota : coords should be expressed in the coordinate system of the world (not the part)
    displays the origin of the handle in the gepetto-gui
    """
    v.client.gui.addSphere(name, .01 , rgba)
    v.client.gui.addToGroup(name, "robot")
    v.client.gui.applyConfiguration(name, coords)

def displayXYZ(v, name, coords, rgba):
    """
    \param v viewer from the ViewerFactory
    \param name string with the name of the handle to display
    \param coords list with the coordinates of the handle to display
    \param rgba list with the RGBA of the origin of the displayed handle
    nota : coords should be expressed in the coordinate system of the world (not the part)
    displays the handle in the gepetto-gui
    """
    v.client.gui.addXYZaxis(name, rgba, .01 , 0.005)
    v.client.gui.addToGroup(name, "robot")
    v.client.gui.applyConfiguration(name, coords)

def displayHandlesOrigins(v, handles, color, namePrefix):
    """
    \param v viewer from the ViewerFactory
    \param handles list with the coordinates of the origins of the handles to dislpay
    \param color list with the RGBA of the origins
    \param namePrefix string with the prefix of the name of the handles to display
    nota : coords should be expressed in the coordinate system of the world (not the part)
    displays the handles origins in the gepetto-gui and names them "namePrefixi"
    """
    for idx in range(len(handles)):
        name = namePrefix+str(idx)
        displayHandle(v, name, handles[idx]+[0,0,0,1], color)
    v.client.gui.refresh()

def displayHandlesFrame(v, handles, color, namePrefix):
    """
    \param v viewer from the ViewerFactory
    \param handles list with the coordinates of the origins of the handles to dislpay
    \param color list with the RGBA of the origins
    \param namePrefix string with the prefix of the name of the handles to display
    nota : coords should be expressed in the coordinate system of the world (not the part)
    displays the handles in the gepetto-gui and names them "namePrefixi"
    """
    for idx in range(len(handles)):
        name = namePrefix+str(idx)
        displayXYZ(v, name, handles[idx], color)
    v.client.gui.refresh()


def retrieveVirtualHandles(clusters, partPose):
    """
    \param clusters list of lists with the nodes of each cluster of the GTSP
    \param partPose of type Transform, to change coordinate system from world to piece
    \retval originInPart list of lists with the coordinates of the origins of the handles in the part coord syst
    \retval originInWorld list of lists with the coordinates of the origins of the handles in the world coord syst
    """
    originInPart, originInWorld = list(), list()
    for c in clusters:
        normQ = pinocchio.Quaternion(c.centroid[3], c.centroid[4], c.centroid[5], c.centroid[6]).normalized()
        direction = [normQ[0], normQ[1], normQ[2], normQ[3]]
        originInPart.append([c.centroid[0], c.centroid[1], c.centroid[2]])
        originInWorld.append(list((partPose*Transform(originInPart[-1]+direction)).toTuple())[:3])
    return originInPart, originInWorld

def fromWorldToPart(coordsInWorld, partPose):
    """
    \param coordsInWorld list of the coordinates in the coord syst of the world 
    \param partPose of type Transform, to change coordinate system from world to piece
    \retval coordsInPart list of the coordinates in the coord syst of the part
    """
    coordsInPart = list()
    for vhw in coordsInWorld:
        coordsInPart.append((partPose.inverse()*Transform(vhw)).toTuple())
    return coordsInPart
