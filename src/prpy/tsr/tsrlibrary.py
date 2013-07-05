#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy

from prpy.tsr import *
from prpy.tsr.rodrigues import *
from prpy.tsr.tsr import *
def GetCylinderTSR(radius, height, manip, T0_w = numpy.eye(4), Tw_e = numpy.eye(4), lateral_tolerance = 0.02):
    """
    Generates a tsr that is appropriate for grasping a cylinder.
        
    @param radius - The radius of the cylinder
    @param height - The height along the cylinder for performing the grasp
    @param manip - The manipulator to use for the grasp
    @param T0_w - A transform from the cylinder frame to the world frame
    @param lateral_tolerance - The tolerance for the height at which to grab the cylinder
    """

    with manip.parent:
        manip.SetActive()
        manipidx= manip.parent.GetActiveManipulatorIndex()

    Bw = numpy.array([[  .0,    .0],
                      [  .0,    .0],
                      [-lateral_tolerance,lateral_tolerance],
                      [  .0,    .0],   
                      [  .0,    .0],   
                      [ -numpy.pi,    numpy.pi]])
    Tw_ee = Tw_e.copy()
    Tw_ee[:3,:3] = numpy.dot(Tw_ee[:3,:3], rodrigues([numpy.pi/2, 0, 0]))
    Tw_ee[:3,3] = [0, radius, height/2]
    cylinderTSR = TSR(T0_w=T0_w, Tw_e=Tw_ee, Bw=Bw, manip=manipidx)

    return cylinderTSR


def GetTSRChainsForObjectGrab(obj, manip,
                              T0_w = numpy.eye(4),
                              Tw_e = numpy.eye(4),
                              start_tsr = True,
                              goal_tsr = False):
    """
    Returns a list of tsr chains that desribe valid grasps
    for the object.
    
    @param obj - The object to be grasped.
    @param manip - The manipulator to perform the grasping
    @param T0_w - The transform from world to object frame
    @param Tw_e - The initial transform from end effector to object
    @param start_tsr - A flag indicating the tsr should be sampled
    for the start of the trajectory
    @param goal_tsr - A flag indicating the tsr should be sampled 
    for the end of the trajectory
    """

    # First grab the manipulator index, this is needed in the tsr specification
    with manip.parent.GetEnv():
        with manip.parent:
            manip.SetActive()
            manipidx = manip.parent.GetActiveManipulatorIndex()

    # This defines the offset from the end effector frame to the palm(ish)
    ee_offset = 0.14


    # We will use the bounding box to attempt to infer the shape of the object
    with manip.parent.GetEnv():
        with obj:
            # Assumption: the coordinate frame of the object is axis aligned
            identity_transform = numpy.eye(4)
            obj.SetTransform(identity_transform)
            obj_bb = obj.ComputeAABB()

    
    # first check if we have a cylinder
    # Assumption: Anything with approximately matching x and y dimensions is a cylinder
    if numpy.abs(obj_bb.extents()[0] - obj_bb.extents()[1]) < 0.001:
        
        # We have a cylinder
        radius = obj_bb.extents()[0] + ee_offset
        height = obj_bb.pos()[2]  # grasp in the middle
        cylinderTSR = GetCylinderTSR(radius = radius,
                                     height = height,
                                     manip = manip,
                                     T0_w = T0_w,
                                     Tw_e = Tw_e)

        cylinderTSRChain = TSRChain(sample_start = start_tsr,
                                    sample_goal = goal_tsr,
                                    TSR = cylinderTSR)
        return [cylinderTSRChain]

    else:

        # We have a box. Define a TSR for each side of the box. 
        #  Also, for each side define a TSR for two opposing end-effector orientations

        # This parameter defines the largest an edge can be for it to be graspable
        #   if one side is larger than this dimension, we won't add the associated
        #   TSRs to the chain list.
        max_spread = 0.2

        chains = []

        if 2.0*obj_bb.extents()[1] < max_spread:
            # negative y
            Bw = numpy.array([[  .0,    .0],
                              [  .0,    .0],
                              [-0.02, 0.02],
                              [  .0,    .0],   
                              [  .0,    .0],   
                              [  .0,    .0]])

            Tw_ee = Tw_e.copy()
            Tw_ee[:3,:3] = numpy.dot(Tw_ee[:3,:3], rodrigues([-numpy.pi/2, 0, 0]))
            Tw_ee[:3,3] += [0, -obj_bb.extents()[1] - ee_offset, 0.]
            boxTSR1 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
            chains.append(TSRChain(sample_start=start_tsr, sample_goal=goal_tsr, TSR=boxTSR1))

            Bw = numpy.array([[  .0,    .0],
                              [  .0,   .0],
                              [-0.02, 0.02],
                              [  .0,    .0],   
                              [  .0,    .0],   
                              [  .0,    .0]])

            Tw_ee = Tw_e.copy()
            Tw_ee[:3,:3] = numpy.dot(Tw_ee[:3,:3],numpy.dot(rodrigues([-numpy.pi/2, 0, 0]), rodrigues([0, 0, numpy.pi])))
            Tw_ee[:3,3] += [0, -obj_bb.extents()[1] - ee_offset, 0.]
            boxTSR2 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
            chains.append(TSRChain(sample_start=start_tsr, sample_goal=goal_tsr, TSR=boxTSR2))


            # positive y
            Bw = numpy.array([[  .0,    .0],
                              [  .0,    .0],
                              [-0.02, 0.02],
                              [  .0,    .0],   
                              [  .0,    .0],   
                              [  .0,    .0]])
            Tw_ee = Tw_e.copy()
            Tw_ee[:3,:3] = numpy.dot(Tw_ee[:3,:3], rodrigues([numpy.pi/2, 0, 0]))
            Tw_ee[:3, 3] += [0, obj_bb.extents()[1] + ee_offset, 0.]
            boxTSR3 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
            chains.append(TSRChain(sample_start=start_tsr, sample_goal=goal_tsr, TSR=boxTSR3))

            Bw = numpy.array([[  .0,    .0],
                              [  .0,    .0],
                              [-0.02, 0.02],
                              [  .0,    .0],   
                              [  .0,    .0],   
                              [  .0,    .0]])
            Tw_ee = Tw_e.copy()
            Tw_ee[:3,:3] = numpy.dot(Tw_ee[:3,:3],numpy.dot(rodrigues([0, numpy.pi/2, 0]), rodrigues([0, 0, numpy.pi/2])))
            Tw_ee[:3, 3] += [-obj_bb.extents()[0] - ee_offset, 0., 0.]
            boxTSR7 = TSR(T0_w=T0_w,Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
            chains.append(TSRChain(sample_start=start_tsr, sample_goal=goal_tsr, TSR=boxTSR7))

            Bw = numpy.array([[  .0,    .0],
                              [  .0,    .0],
                              [-0.02,  0.02],
                              [  .0,    .0],   
                              [  .0,    .0],   
                              [  .0,    .0]])  
            Tw_ee = Tw_e.copy()
            Tw_ee[:3,:3] = numpy.dot(Tw_ee[:3,:3],numpy.dot(rodrigues([0, numpy.pi/2, 0]), rodrigues([0, 0, -numpy.pi/2])))
            Tw_ee[:3, 3] += [-obj_bb.extents()[0] - ee_offset, 0., 0.]
            boxTSR8 = TSR(T0_w=T0_w, Tw_e=Tw_ee, Bw=Bw, manip=manipidx)
            chains.append(TSRChain(sample_start=start_tsr, sample_goal=goal_tsr, TSR=boxTSR8))


        return chains

def GetNoTiltTSRChain(manip, axis = [0., 0., 1.]):
    """
    Returns the no tilting tsr string for the end-effector. No-tilting
    here means that the end-effector is allowed to rotate only along the
    end-effecttor axis that is closest to the given axis (world +z by default).
    it.  
    
    @param manip - The manipulator
    @param axis - The axis to hold steady
    """
    
    # Grab the tranform for the ee on this manip
    T0_w = manip.GetEndEffectorTransform()
    
    # Get the index - this is necessary for the tsr definition
    maninumpy.pindex = manip.GetActiveManipulatorIndex()
    

    # Find which axis of end-effector is aligned with the +z in world
    projections = []
    for idx in range(3):
        projections.append(numpy.abs(numpy.dot(T0_w[:3,idx], numpy.array(axis))))
        

    axis_idx = max( (v,i) for i,v in enumerate(projections) )[1]

    Tw_e = numpy.eye(4)
    Bw = numpy.array([-100., 100.,
                      -100., 100.,
                      -100., 100.,
                         0.,   0.,
                         0.,   0.,
                         0.,   0.])
    Bw[(3+axis_idx)*2.] = -numpy.numpy.pi
    Bw[(3+axis_idx)*2. + 1] = numpy.numpy.pi
    
    constraintTSR = TSR(T0_w=T0_w, Tw_e=Tw_e, Bw=Bw, manip=manipidx)
    constraintTSRChain = TSRChain(constrain=True, TSR=constraintTSR)

    return constraintTSRChain
