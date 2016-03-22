#!/usr/bin/env python

# Copyright (c) 2016, Carnegie Mellon University
# All rights reserved.
# Authors: Gilwoo Lee <gilwool@andrew.cmu.edu>

# Uses optimization to joint estimate pose of table and
# relative transform of camera with respect to table.


import prpy
import prpy.rave
import rospy
import os
import tf
import json
import numpy
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import (quaternion_matrix,
                                euler_from_quaternion,
                                euler_matrix)
from openravepy.misc import DrawAxes

import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class CameraTableJointPoseEstimator(object):
    """ 
    Assuming table height is known and is lying flat on the ground, 
    estimate the pose of table as well as the frame offset
    between destimation frame and detection frame (camera frame).
    """

    def __init__(self, env, marker_data_path,
                 kinbody_directory,
                 marker_topic,
                 detection_frame='head/kinect2_rgb_optical_frame',
                 destination_frame='map',
                 reference_link=None):

        # Initialize a new ros node if one has not already been created
        try:
            rospy.init_node('frame_offset_estimator', anonymous=True)
        except rospy.exceptions.ROSException:
            pass
        
        self.env = env
        self.marker_data_path = marker_data_path
        self.kinbody_directory = kinbody_directory
        self.marker_topic = marker_topic
        self.detection_frame = detection_frame
        self.destination_frame = destination_frame
        self.generated_bodies = []
        self.reference_link = reference_link
        self.listener = tf.TransformListener()
        
        self.ReloadKinbodyData()

        # Camera Projection Matrix
        self.P = numpy.matrix([529.2945040622658, 0.0, 466.96044871160075, 0.0,
                               0.0, 531.2834529497384, 273.2593671723483, 0.0,
                               0.0, 0.0, 1.0, 0.0])

        self.P = self.P.reshape([3, 4])

    def ReloadKinbodyData(self):
        with open(self.marker_data_path, 'r') as f:
            self.marker_data = json.load(f)
            
    def se2_to_kinbody_pose(self, se2_pose, height):
        """ Takes [theta, x, y].
        Returns 4x4 object transform.
        """
        r = se2_pose[0]
        x = se2_pose[1]
        y = se2_pose[2]

        obj_transform = numpy.identity(4)
        obj_transform[:3, :3] = numpy.array(
                                    [[numpy.cos(r), -numpy.sin(r), 0.],
                                     [numpy.sin(r),  numpy.cos(r), 0.],
                                     [0., 0., 1.]])
        obj_transform[:, 3] = numpy.array([x, y, height, 1])
        obj_transform = numpy.matrix(obj_transform)
        return obj_transform


    def detect_and_optimize(self, timeout=10):
        """
        Detect table and optimize pose of table and camera
        """
        marker_message = rospy.wait_for_message(self.marker_topic,
                                                MarkerArray,
                                                timeout=timeout)

        added_kinbodies = []
        updated_kinbodies = []
        
        for marker in marker_message.markers:
            if marker.ns in self.marker_data:
                kinbody_file, kinbody_offset = self.marker_data[marker.ns]

                if "table" not in kinbody_file:
                    continue

                kinbody_offset = numpy.array(kinbody_offset)
                marker_pose = numpy.array(quaternion_matrix([
                        marker.pose.orientation.x,
                        marker.pose.orientation.y,
                        marker.pose.orientation.z,
                        marker.pose.orientation.w]))
                marker_pose[0, 3] = marker.pose.position.x
                marker_pose[1, 3] = marker.pose.position.y
                marker_pose[2, 3] = marker.pose.position.z
                
                self.listener.waitForTransform(
                        self.detection_frame,
                        self.destination_frame,
                        rospy.Time(),
                        rospy.Duration(timeout))
                frame_trans, frame_rot = self.listener.lookupTransform(
                        self.destination_frame,
                        self.detection_frame,
                        rospy.Time(0))
                frame_offset = numpy.matrix(quaternion_matrix(frame_rot))
                frame_offset[:,3] = frame_trans[0:3]
                
                table_pose = numpy.array(numpy.dot(numpy.dot(frame_offset,
                                                             marker_pose),
                                                   kinbody_offset))

                if self.reference_link is not None:
                    ref_link_pose = self.reference_link.GetTransform()
                    final_kb_pose = numpy.dot(ref_link_pose, table_pose)
                    DrawAxes(self.env, final_kb_pose)

                from table_clearing.perception_utils import (get_table_height,
                                                             PerceptionException)

                # Optimize table and cam pose
                try:
                    table_height = get_table_height(self.env)
                    table_pose_new, frame_offset_new = self.optimize(
                                                                kinbody_offset,
                                                                frame_trans,
                                                                frame_rot,
                                                                marker_pose,
                                                                table_pose,
                                                                table_height)

                    logger.info("Optimized table and cam pose", kinbody_file)

                    if table_pose_new is not None:
                        table_pose = table_pose_new 
                    if frame_offset is not None:
                        frame_offset = frame_offset_new

                except PerceptionException as e:
                    logger.warn(str(e))
                except ValueError as e:
                    logger.warn(str(e))

              
                # Transform w.r.t reference link if link present
                if self.reference_link is not None:
                    ref_link_pose = self.reference_link.GetTransform()
                    final_kb_pose = numpy.dot(ref_link_pose,table_pose)
                    DrawAxes(self.env, final_kb_pose)
                
                kinbody_name = kinbody_file.replace('.kinbody.xml', '')
                kinbody_name = kinbody_name + str(marker.id)
                
                # load the object if it does not exist
                if self.env.GetKinBody(kinbody_name) is None:
                    new_body = prpy.rave.add_object(
                            self.env,
                            kinbody_name,
                            os.path.join(self.kinbody_directory, kinbody_file))
                    added_kinbodies.append(new_body)
                    self.generated_bodies.append(new_body)
                
                body = self.env.GetKinBody(kinbody_name)
                body.SetTransform(table_pose)
                updated_kinbodies.append(body)
        
        return table_pose, frame_offset


    def optimize(self, kinbody_offset, cam_translation, cam_quaternion,
                 marker_pose, table_init_pose, table_height):
        """
        Returns optimized table_pose and frame_offset.
        @raise ValueError when optimization fails
        """

        from scipy.optimize import minimize

        def cost(x):
            """ 
            x[0:3] = table_se2(theta, x, y)
            x[3:6] = camera translation
            x[6:9] = camera rotation (euler)
            """
            
            frame_rot = x[6:9]
            frame_trans = x[3:6]
            frame_offset = numpy.matrix(euler_matrix(frame_rot))
            frame_offset[:,3] = frame_trans[0:3]

            observed_projection = self.project_marker_to_im_screen(marker_pose)
            
            import numpy.linalg as la
            obj_transform = self.se2_to_kinbody_pose(table_se2, table_height)
            expected_marker_pose = numpy.dot(numpy.dot(la.inv(frame_offset),
                                                       obj_transform),
                                             la.inv(kinbody_offset))
            expected_marker_position = expected_marker_pose[:, 3]
            expected_projection = numpy.array(numpy.dot(P, 
                                                        expected_marker_position)
                                              .transpose())[0]

            for i in range(3):
                expected_projection[i] /= expected_projection[2]

            return la.norm(observed_projection - expected_projection)


        x = numpy.zero([9, 1])
        # x[0:3] is for table se2 pose
        x[0] = numpy.arctan2(table_init_pose[1, 0], table_init_pose[0, 0])
        x[1] = table_init_pose[0, 3]
        x[2] = table_init_pose[1, 3]
        # x[3:9] is for camera frame offset
        x[3:6] = cam_translation
        x[6:9] = euler_from_quaternion(cam_quaternion)

        res = minimize(cost, x0, constraints=None, method='SLSQP',
                       options={'disp': True})

        table_se2 = res.x[0:2]
        
        frame_rot = res.x[6:9]
        frame_trans = res.x[3:6]
        
        frame_offset = numpy.matrix(euler_matrix(frame_rot))
        frame_offset[:,3] = frame_trans[0:3]

        table_se3 = self.se2_to_kinbody_pose(table_se2, table_height)
        
        return table_se3, frame_offset