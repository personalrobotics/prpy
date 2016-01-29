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
import prpy
import rospy
import os
import tf
import json
import numpy
from visualization_msgs.msg import MarkerArray, MarkerArray
from tf.transformations import quaternion_matrix
from base import PerceptionModule, PerceptionMethod

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class ApriltagsModule(PerceptionModule):
    
    def __init__(self, marker_topic, marker_data_path, kinbody_path,
                 detection_frame='head/kinect2_rgb_optical_frame', 
                 destination_frame='map',
                 reference_link=None):
        """
        This initializes an April Tags detector.
        
        @param marker_topic The ROS topic to read markers from. Typically the output topic for April Tags
        @param marker_data_path The json file where the association between tag and object is stored
        @param kinbody_path The path to the folder where kinbodies are stored
        @param detection_frame The TF frame of the camera
        @param destination_frame The desired world TF frame
        """

        super(ApriltagsModule, self).__init__()
        
        try:
            rospy.init_node('apriltag_detector', anonymous=True)
        except rospy.exception.ROSException:
            pass

        self.marker_topic = marker_topic
        self.marker_data_path = marker_data_path
        self.kinbody_path = kinbody_path
        self.detection_frame = detection_frame
        self.destination_frame = destination_frame
        self.generated_bodies = []

        self.reference_link = reference_link
        self.listener = tf.TransformListener()
        self.ReloadKinbodyData()
        
        
    def __str__(self):
        return self.__class__.__name__

    def ReloadKinbodyData(self):
        """
        Load the kinbody data (tag-object relation) from the json file. 
        """
        with open(self.marker_data_path, 'r') as f:
            self.marker_data = json.load(f)

    def Update(self, timeout=10):
        """
        This updates the kinbodies after the detection. 
        @param env: The current OpenRAVE environment
        @return The list of newly added and updated kinbodies associated with the detected apriltags
        """
        marker_message = rospy.wait_for_message(self.marker_topic, 
                                                MarkerArray,
                                                timeout=timeout)
        env = self.env
        added_kinbodies = []
        updated_kinbodies = []
        for marker in marker_message.markers:
            if marker.ns in self.marker_data:
                kinbody_file, kinbody_offset = self.marker_data[marker.ns]
                kinbody_offset = numpy.array(kinbody_offset)
                marker_pose = numpy.array(quaternion_matrix([
                        marker.pose.orientation.x,
                        marker.pose.orientation.y,
                        marker.pose.orientation.z,
                        marker.pose.orientation.w
                    ]))
                marker_pose[0,3] = marker.pose.position.x
                marker_pose[1,3] = marker.pose.position.y
                marker_pose[2,3] = marker.pose.position.z
                try:
                    self.listener.waitForTransform(
                        self.detection_frame,
                        self.destination_frame,
                        rospy.Time(),
                        rospy.Duration(timeout))
                
                    frame_trans, frame_rot = self.listener.lookupTransform(
                        self.destination_frame,
                        self.detection_frame,
                        rospy.Time(0))

                except Exception, e:
                    logger.error("Can't retrive head tf tree please check head connection: %s" % str(e))
                    raise

                frame_offset = numpy.matrix(quaternion_matrix(frame_rot))
                frame_offset[0,3] = frame_trans[0]
                frame_offset[1,3] = frame_trans[1]
                frame_offset[2,3] = frame_trans[2]

                kinbody_pose = numpy.array(numpy.dot(numpy.dot(frame_offset, marker_pose),
                                            kinbody_offset))

                final_kb_pose = kinbody_pose

                #Transform w.r.t reference link if link present
                if self.reference_link is not None:
                    ref_link_pose = self.reference_link.GetTransform()
                    final_kb_pose = numpy.dot(ref_link_pose, kinbody_pose)

                kinbody_name = kinbody_file.replace('.kinbody.xml', '')
                kinbody_name = kinbody_name + str(marker.id)

                if env.GetKinBody(kinbody_name) is None:
                    new_body = prpy.rave.add_object(
                        env,
                        kinbody_name,
                        os.path.join(self.kinbody_path, kinbody_file))
                    added_kinbodies.append(new_body)
                    self.generated_bodies.append(new_body)

                body = env.GetKinBody(kinbody_name)
                body.SetTransform(final_kb_pose)
                updated_kinbodies.append(body)

        return added_kinbodies, updated_kinbodies

    @PerceptionMethod
    def DetectObjects(self, robot, **kw_args):
        """
        Overriden method for detection_frame
        """
        try:
            self.env = robot.GetEnv();
            added_kinbodies, updated_kinbodies = self.Update(**kw_args)
        except Exception, e:
            logger.error('Detecton failed update: %s' % str(e))
            raise

        return added_kinbodies + updated_kinbodies
                                          
    @PerceptionMethod
    def DetectObject(self, robot, object_name, **kw_args):
        """
        Detects a single named object.
        """
        try:
            self.env = robot.GetEnv();
            added_bodies, updated_bodies = self.Update( **kw_args)
        except Exception, e:
            logger.error('Detecton failed update: %s' % str(e))
            raise

        return_obj = None
        for obj in added_bodies:
            if obj.GetName() != object_name:
                env.Remove(obj)
            else:
                return_obj = obj
        for obj in updated_bodies:
            if obj.GetName() == object_name:
                return_obj = obj
            # TODO: Otherwise we need to put it back

        if return_obj is not None:
            return return_obj

        from prpy.perception.base import PerceptionException
        raise PerceptionException('Failed to detect object %s', object_name)
