import prpy
import prpy.rave
import rospy
import os
import tf
import json
import numpy
from visualization_msgs.msg import MarkerArray
from tf.transformations import quaternion_matrix


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
                 reference_link=None):

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

        self.P = numpy.matrix([529.2945040622658,
                               0.0,
                               466.96044871160075,
                               0.0, 0.0,
                               531.2834529497384,
                               273.2593671723483,
                               0.0, 0.0, 0.0, 1.0, 0.0])
        self.P = self.P.reshape([3, 4])

    def ReloadKinbodyData(self):
        with open(self.marker_data_path, 'r') as f:
            self.marker_data = json.load(f)

    def optimize(self, kinbody_offset, frame_offset,
                 marker_pose, kinbody_init_pose, table_height):
        from scipy.optimize import minimize

        h = table_height
        print "Optimizer called"

        def cost(x):
            """ x = r, tx, ty,"""
            r = x[0]

            obj_transform = numpy.ones((4, 4))
            obj_transform[:3, :3] = numpy.array(
                                        [[numpy.cos(r), -numpy.sin(r), 0.],
                                         [numpy.sin(r),  numpy.cos(r), 0.],
                                         [0., 0., 1.]])

            obj_transform[:, 3] = numpy.array([x[1], x[2], h, 1])
            obj_transform = numpy.matrix(obj_transform)

            import numpy.linalg as la

            projection = numpy.dot(self.P, marker_pose[:, 3])[0]
            projection = numpy.array(projection)[0]
            print projection

            for i in range(3):
                projection[i] /= projection[2]

            expected_marker_pose = numpy.dot(numpy.dot(la.inv(frame_offset),
                                                       obj_transform),
                                             la.inv(kinbody_offset))
            print "expected_marker_pose", expected_marker_pose
            expected_marker_position = expected_marker_pose[:, 3]
            expected_projection = numpy.dot(self.P,
                                            expected_marker_position)
            expected_projection = numpy.array(expected_projection.transpose())[0]
            print expected_projection

            for i in range(3):
                expected_projection[i] /= expected_projection[2]

            print "projection\n", projection
            print "exepected\n", expected_projection

            return la.norm(projection - expected_projection)

        # make world pose constrained in z -height & rotation
        cons = ({'type': 'ineq',
                 'fun': lambda x: x[3]})  # s>0

        x0 = numpy.zeros([4, 1])
        x0[0] = numpy.arctan2(kinbody_init_pose[1, 0], kinbody_init_pose[0, 0])
        x0[1] = kinbody_init_pose[0, 3]
        x0[2] = kinbody_init_pose[1, 3]
        import random
        x0[3] = random.random()


        try:
            res = minimize(cost, x0, constraints=cons, method='SLSQP',
                       options={'disp': True})
            print res
        except ValueError as e:
            print "Exception", e
            return marker_pose, None

        r = res.x[0]
        tx = res.x[1]
        ty = res.x[2]
        s = res.x[3]

        obj_transform = numpy.ones((4, 4))
        obj_transform[:3, :3] = numpy.array(
                                    [[numpy.cos(r), -numpy.sin(r), 0.],
                                     [numpy.sin(r),  numpy.cos(r), 0.],
                                     [0., 0., 1.]])

        obj_transform[:, 3] = numpy.array([tx, ty, h, 1])
        obj_transform[3, :] = numpy.array([0, 0, 0, 1])

        return None, obj_transform

    def Update(self, timeout=10):
        marker_message = rospy.wait_for_message(self.marker_topic,
                                                MarkerArray,
                                                timeout=timeout)
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
                frame_offset[0, 3] = frame_trans[0]
                frame_offset[1, 3] = frame_trans[1]
                frame_offset[2, 3] = frame_trans[2]

                kinbody_pose = numpy.array(numpy.dot(numpy.dot(frame_offset,
                                                               marker_pose),
                                                     kinbody_offset))
                
                from openravepy.misc import DrawAxes
                
                if self.reference_link is not None:
                    ref_link_pose = self.reference_link.GetTransform()
                    final_kb_pose = numpy.dot(ref_link_pose, kinbody_pose)

                    h1= DrawAxes(self.env, final_kb_pose)



                from table_clearing.perception_utils import (get_table_height,
                                                             PerceptionException)

                if "table" not in kinbody_file:
                    try:
                        h = get_table_height(self.env)
                        a, obj_transform = self.optimize(kinbody_offset,
                                                         frame_offset,
                                                         marker_pose,
                                                         kinbody_pose,
                                                         h)
                        print "Optimizing", kinbody_file
                        if obj_transform is not None:
                            kinbody_pose = obj_transform
                            if self.reference_link is not None:
                                ref_link_pose = self.reference_link.GetTransform()
                                final_kb_pose = numpy.dot(ref_link_pose, kinbody_pose)
                                h2 = DrawAxes(self.env, final_kb_pose)

                    except PerceptionException as e:
                        h = 0.5
                        a, obj_transform = self.optimize(kinbody_offset,
                                                         frame_offset,
                                                         marker_pose, h)

                        print kinbody_file
                        if obj_transform is not None:
                            print "Optimized kinbody pose based on h=0.5"
                            kinbody_pose = obj_transform

                final_kb_pose = kinbody_pose

                # Transform w.r.t reference link if link present
                if self.reference_link is not None:
                    ref_link_pose = self.reference_link.GetTransform()
                    final_kb_pose = numpy.dot(ref_link_pose, kinbody_pose)
                    
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
                body.SetTransform(final_kb_pose)
                updated_kinbodies.append(body)

        
        return added_kinbodies, updated_kinbodies