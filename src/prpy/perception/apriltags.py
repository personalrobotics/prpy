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
logger.setLevel(logging.INFO)

class ApriltagsModule(PerceptionModule):
    
    def __init__(self, marker_topic, marker_data_path, kinbody_path,
                 detection_frame, destination_frame):
        """
        This initializes an April Tags detector.
        Args:
            marker_topic (string): The ROS topic to read markers from. Typically the output topic for April Tags
            marker_data_path(string): The json file where the association between tag and object is stored
            kinbody_path: The path to the folder where kinbodies are stored
            detection_frame: The TF frame of the camera
            destination_frame: The desired world TF frame
        """

        super(ApriltagsModule, self).__init__()
        
        self.marker_topic = marker_topic
        self.marker_data_path = marker_data_path
        self.kinbody_path = kinbody_path
        self.detection_frame = detection_frame
        self.destination_frame = destination_frame
        
        
    def __str__(self):
        return 'Apriltags'

    def DetectObjects(self, env, object_names, **kw_args):
        """
        This hack detects only the objects in object_names. Updates existing
        objects, but only adds objects in the object_names list.
        """
        added_kinbodies, updated_kinbodies = self.DetectObjects(robot, **kw_args);
        detected = [];
        for body in added_kinbodies:
            if not (body.GetName() in object_names):
                env.RemoveKinbody(body);
            else:
                detected.append(body);
        return detected;
                
    def DetectObject(self, env, object_name, **kw_args):
        """
        Detects a single named object.
        """
        return (self._DetectObjects( env, object_names=[object_name], **kw_args))[0][0];


    def _DetectObjects(self, env, **kw_args):
        """
        Use the apriltags service to detect objects and add them to the
        environment
        """
        try:
            # Allow caller to override any of the initial parameters
            # loaded into the module
            if 'marker_data_path' in kw_args:
                marker_data_path = kw_args['marker_data_path']
            else:
                marker_data_path = self.marker_data_path
            
            if 'kinbody_path' in kw_args:
                kinbody_path = kw_args['kinbody_path']
            else:
                kinbody_path = self.kinbody_path
                
            if 'marker_topic' in kw_args:
                marker_topic = kw_args['marker_topic']
            else:
                marker_topic = self.marker_topic

            if 'detection_frame' in kw_args:
                detection_frame = kw_args['detection_frame']
            else:
                detection_frame = self.detection_frame

            if 'destination_frame' in kw_args:
                destination_frame = kw_args['destination_frame']
            else:
                destination_frame = self.destination_frame

            # TODO: Creating detector is not instant...might want
            #  to just do this once in the constructor
            import kinbody_detector.kinbody_detector as kd
            detector = kd.KinBodyDetector(env,
                                          marker_data_path,
                                          kinbody_path,
                                          marker_topic,
                                          detection_frame,
                                          destination_frame)

            logger.warn('Waiting to detect objects...')
            return detector.Update()
        except Exception, e:
            logger.error('Detecton failed update: %s' % str(e))
            raise

    @PerceptionMethod
    def DetectObjects(self, robot, **kw_args):
        """
        Overriden method for detection_frame
        """
        return self._DetectObjects(robot.GetEnv(),**kwargs)
                                          
