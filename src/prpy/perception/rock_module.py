#!/usr/bin/env python

# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
# Authors: Jennifer King <jeking@cs.cmu.edu>
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
import logging
from base import PerceptionModule, PerceptionMethod

logger = logging.getLogger(__name__)

class RockModule(PerceptionModule):

    def __init__(self, kinbody_path, detection_frame, destination_frame):
        super(RockModule, self).__init__()
        self.kinbody_path = kinbody_path
        self.detection_frame = detection_frame
        self.destination_frame = destination_frame

    def __str__(self):
        return 'RockModule'

    def _GetDetector(self, env, kinbody_path, detection_frame, destination_frame):
        """
        @param env The OpenRAVE environment
        @param kinbody_path The path to use to map kinbodies to poses 
           (if None defaults to the path used to initialize the module)
        @param detection_frame The camera frame
           (if None defaults to the frame used to initialize the module)
        @param destination_frame The frame to place detected objects into
           (if None defaults to the frame used to intialize the module)
        """

        if kinbody_path is None:
            kinbody_path = self.kinbody_path

        if detection_frame is None:
            detection_frame = self.detection_frame
            
        if destination_frame is None:
            destination_frame = self.destination_frame

        from rock import RockDetector
        rock_detector = RockDetector(
            env,
            kinbody_path,
            detection_frame=detection_frame,
            world_frame=destination_frame)
        return rock_detector

    @PerceptionMethod
    def DetectObject(self, robot, obj_name,
                     kinbody_path=None,
                     detection_frame=None,
                     destination_frame=None):
        """
        @param obj_name The name of the object to search for
        @param kinbody_path The path to use to map kinbodies to poses 
           (if None defaults to the path used to initialize the module)
        @param detection_frame The camera frame
           (if None defaults to the frame used to initialize the module)
        @param destination_frame The frame to place detected objects into
           (if None defaults to the frame used to intialize the module)
        @return A list of detected kinbodies
        """
        # Initial the detector
        rock_detector = self._GetDetector(robot.GetEnv(),
                                          kinbody_path,
                                          detection_frame,
                                          destination_frame)
    
        # Use the detector to find object poses
        poses = rock_detector.SearchTabletop(obj_name)
    
        # Use the poses to generate new kinbodies
        kinbodies = rock_detector.KinBodiesFromPoses(obj_names, poses)

        return kinbodies

    @PerceptionMethod
    def DetectObjectNearPose(self, robot, obj_name, pose, 
                             search_extents=None,
                             kinbody_path=None,
                             detection_frame=None,
                             destination_frame=None):
        """
        Searches near a given pose for any objects
        @param obj_name The name of the object to search for
        @param pose The pose to search near
        @param search_extents = [x,y,z]-extents of search area
           (in pose frame)
        @param kinbody_path The path to use to map kinbodies to poses 
           (if None defaults to the path used to initialize the module)
        @param detection_frame The camera frame
           (if None defaults to the frame used to initialize the module)
        @param destination_frame The frame to place detected objects into
           (if None defaults to the frame used to intialize the module)
        @return A list of detected kinbodies
        """
        if search_extents is None: 
            search_extents=[-0.1, -0.1, 0.]

        # Initial the detector
        rock_detector = self._GetDetector(robot.GetEnv(),
                                     kinbody_path,
                                     detection_frame,
                                     destination_frame)

        # Use the detector to find object poses
        poses = rock_detector.SearchBBox(
            obj_name, pose, 
            -0.5*numpy.array(search_extents),
             0.5*numpy.array(search_extents),
            rock_detector.UprightOrientations())

        # Use the poses to generate new kinbodies
        kinbodies = rock_detector.KinBodiesFromPoses(obj_names, poses)
        
        return kinbodies

