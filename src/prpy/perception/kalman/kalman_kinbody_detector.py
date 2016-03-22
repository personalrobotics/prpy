import prpy
import prpy.rave
import rospy
import os
import tf
import json
import numpy
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import quaternion_matrix

import logging
# from base import PerceptionModule, PerceptionMethod

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class DetectorException(Exception):
    pass


class KinBodyDetector(object):
    def __init__(self,
                 env,
                 marker_data_path,
                 kinbody_directory,
                 marker_topic,
                 detection_frame='head/kinect2_rgb_optical_frame',
                 destination_frame='map',
                 reference_link=None, 
                 frame_offset=None): # TODO: use frame offset
        
        # Initialize a new ros node if one has not already been created
        try:
            rospy.init_node('kinbody_detector', anonymous=True)
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

        # Camera Matrix
        self.P = numpy.matrix([529.2945040622658,
                               0.0,
                               466.96044871160075,
                               0.0, 0.0,
                               531.2834529497384,
                               273.2593671723483,
                               0.0, 0.0, 0.0, 1.0, 0.0])
        self.P = self.P.reshape([3, 4])

    def convert_to_kinbody_pose(self, se2_pose, height):
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

    def project_marker_to_im_screen(self, marker_pose):
        """ Returns (u,v,1) on image screen """ 
        projection = numpy.dot(self.P, marker_pose[:, 3])[0]
        projection = numpy.array(projection)[0]

        for i in range(3):
            projection[i] /= projection[2]
        return projection

    def optimize(self, kinbody_offset, frame_offset,
                 marker_pose, kinbody_init_pose, table_height):
        """
        Returns optimized kinbody_pose and (theta,x,y).
        Returns None, None if fails to optimize.
        """
        from scipy.optimize import minimize

        h = table_height

        def cost(x):
            """ x = theta(r), tx, ty,"""
            obj_transform = self.convert_to_kinbody_pose(x, h)

            projection = self.project_marker_to_im_screen(marker_pose)
            import numpy.linalg as la
            expected_marker_pose = numpy.dot(numpy.dot(la.inv(frame_offset),
                                                       obj_transform),
                                             la.inv(kinbody_offset))
            expected_marker_position = expected_marker_pose[:, 3]
            expected_projection = numpy.dot(self.P,
                                            expected_marker_position)
            expected_projection = numpy.array(expected_projection.transpose())[0]

            for i in range(3):
                expected_projection[i] /= expected_projection[2]

            return la.norm(projection - expected_projection)

        x0 = numpy.zeros([3, 1])
        x0[0] = numpy.arctan2(kinbody_init_pose[1, 0], kinbody_init_pose[0, 0])
        x0[1] = kinbody_init_pose[0, 3]
        x0[2] = kinbody_init_pose[1, 3]


        try:
            res = minimize(cost, x0, method='SLSQP',
                           options={'disp': True})
        except ValueError:
            return None, None

        r = res.x[0]
        tx = res.x[1]
        ty = res.x[2]

        obj_transform = self.convert_to_kinbody_pose((r, tx, ty), h)

        return obj_transform, (r, tx, ty)

    
    def update_kinbody_pose(self, obj_transform, reference_link):
        final_kb_pose = obj_transform
        if self.reference_link is not None:
            ref_link_pose = self.reference_link.GetTransform()
            final_kb_pose = numpy.dot(ref_link_pose,
                                      final_kb_pose)
        return final_kb_pose

    def ReloadKinbodyData(self):
        with open(self.marker_data_path, 'r') as f:
            self.marker_data = json.load(f)
    
    def Update(self, timeout=10):
        marker_message = rospy.wait_for_message(self.marker_topic,
                                                MarkerArray,
                                                timeout=timeout)
        # current image
        # from sensor_msgs import Image
        # image = rospy.wait_for_message('camera/rgb/image_raw', #TODO: topic name
        #                                 Image,
        #                                 timeout=timeout)

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
                frame_offset[0,3] = frame_trans[0]
                frame_offset[1,3] = frame_trans[1]
                frame_offset[2,3] = frame_trans[2]
                
                kinbody_pose = numpy.array(numpy.dot(numpy.dot(frame_offset,
                                                               marker_pose),
                                                     kinbody_offset))

                # Draw initial pose estimate
                from openravepy.misc import DrawAxes
                if self.reference_link is not None:
                    ref_link_pose = self.reference_link.GetTransform()
                    final_kb_pose = numpy.dot(ref_link_pose, kinbody_pose)
                    DrawAxes(self.env, final_kb_pose)

                from table_clearing.perception_utils import (get_table_height,
                                                             PerceptionException)
                if "table" not in kinbody_file:
                    # try: 
                    #     # Optimize for non-table objects to get initial guess
                    #     h = get_table_height(self.env)
                    #     # obj_transform, se2_pose = self.optimize(kinbody_offset,
                    #     #                                         frame_offset,
                    #     #                                         marker_pose,
                    #     #                                         kinbody_pose,
                    #     #                                         h)

                    #     logger.info("Optimizing: %s", kinbody_file)
                    #     if obj_transform is not None:
                    #         kinbody_pose = obj_transform
                    # except PerceptionException:
                    #     h = 0.5  # Known table height
                    #     obj_transform, se2_pose = self.optimize(kinbody_offset,
                    #                                             frame_offset,
                    #                                             marker_pose,
                    #                                             kinbody_pose,
                    #                                             h)

                    #     if obj_transform is not None:
                    #         kinbody_pose = obj_transform

                    h = get_table_height(self.env)

                    se2_pose = numpy.zeros(3)
                    se2_pose[0] = numpy.arctan2(kinbody_pose[1, 0], kinbody_pose[0, 0])
                    se2_pose[1] = kinbody_pose[0, 3]
                    se2_pose[2] = kinbody_pose[1, 3]

                    logger.info("SE2 pose initial %s", str(se2_pose))
                    # Get mu = (theta, x, y) from current kinbody pose estimate.
                    mu = numpy.matrix(se2_pose).transpose()
                    cov = numpy.matrix(numpy.identity(3))

                    # Get marker position z=(u,v,1) on image screen
                    projection = self.project_marker_to_im_screen(marker_pose)
                    z = numpy.matrix(projection[0:2]).transpose()

                    # Update (x,y,theta) by UKF
                    logger.info("Run UKF update for %s", kinbody_file)
                    import ukf_update 
                    from ukf import get_sigma_points
                    sig_points, w1, w2 = get_sigma_points(mu, cov)
                    # sig_points_projection = ukf_update.measure(sig_points, self.P,
                    #                                 frame_offset,
                    #                                 kinbody_offset,
                    #                                 height=h)
                    #TODO: draw sigma_points_on_screen on 2D image 

                    # Convert sigma points to poses and draw on screen
                    axes = []
                    # for i in range(sig_points.shape[1]):
                    #     kin_pose = self.convert_to_kinbody_pose(sig_points[:, i], h)
                    #     if self.reference_link is not None:
                    #         ref_link_pose = self.reference_link.GetTransform()
                    #         kin_pose = numpy.dot(ref_link_pose, kin_pose)
                    #     axes.append(DrawAxes(self.env, kin_pose))
                    #     raw_input(str(sig_points[:,i]))

                    raw_input('Drew old kin_poses, proceed to draw new ones')

                    mu_new, cov_new = ukf_update.update(mu, cov, self.P,
                                             frame_offset,
                                             kinbody_offset,
                                             z,
                                             height=h)

                    sig_points_new, w1, w2 = get_sigma_points(mu_new, cov_new)
                    # sig_points_new_projection = ukf_update.measure(sig_points_new,
                    #                                     self.P,
                    #                                     frame_offset,
                    #                                     kinbody_offset,
                    #                                     height=h)
                    # TODO: draw new sigma_points on 2D image 
                    
                    # Convert sigma points to poses and draw on screen

                    for i in range(sig_points_new.shape[1]):
                        kin_pose = self.convert_to_kinbody_pose(sig_points_new[:, i], h)
                        if self.reference_link is not None:
                            ref_link_pose = self.reference_link.GetTransform()
                            kin_pose = numpy.dot(ref_link_pose, kin_pose)
                            DrawAxes(self.env, kin_pose)
                        axes.append(DrawAxes(self.env, kin_pose))
                        raw_input(str(sig_points[:,i]))

                    raw_input('Drew new kin_poses after UKF update')



                    logger.info("mu old: %s", str(mu.transpose()))
                    logger.info("mu new: %s", str(mu_new.transpose()))
                    logger.info("cov old: \n %s", str(cov))
                    logger.info("cov new: \n %s", str(cov_new))

                    kinbody_pose = self.convert_to_kinbody_pose(mu_new, h)
                
                final_kb_pose = numpy.array(kinbody_pose)

                logger.info("final_kb_pose \n %s", str(final_kb_pose))

                # Transform w.r.t reference link if link present
                if self.reference_link is not None:
                    ref_link_pose = self.reference_link.GetTransform()
                    final_kb_pose = numpy.dot(ref_link_pose,kinbody_pose)
                
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
                body.SetTransform(numpy.array(final_kb_pose))
                updated_kinbodies.append(body)
        
        return added_kinbodies, updated_kinbodies
