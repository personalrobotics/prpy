import math
import numpy
import os.path

from base import PerceptionModule, PerceptionMethod


class SimtrackModule(PerceptionModule):

    def __init__(self, kinbody_path, detection_frame, 
                 destination_frame=None, reference_link=None,
                 service_namespace=None):
        """
        This initializes a simtrack detector.
        
        @param kinbody_path The path to the folder where kinbodies are stored
        @param detection_frame The TF frame of the camera
        @param destination_frame The tf frame that the kinbody should be transformed to
        @param reference_link The OpenRAVE link that corresponds to the tf destination_frame
        @param service_namespace The namespace for the simtrack service (default: /simtrack)
        """
        import rospy
        import tf
        import tf.transformations as transformations
        # Initialize a new ros node if one has not already been created
        try:
            rospy.init_node('simtrack_detector', anonymous=True)
        except rospy.exceptions.ROSException:
            pass
            
        #For getting transforms in world frame
        if detection_frame is not None and world_frame is not None:
            self.listener = tf.TransformListener()
        else:
            self.listener = None
                
        if service_namespace is None:
            service_namespace='/simtrack'
        self.service_namespace = service_namespace

        self.detection_frame = detection_frame
        if destination_frame is None:
            destination_frame='/map'
        self.destination_frame = destination_frame
        self.reference_link = reference_link
            
        self.kinbody_path = kinbody_path

        # A map of known objects that can be queries
        self.kinbody_to_query_map = {'fuze_bottle': 'fuze_bottle_visual',
                                     'pop_tarts': 'pop_tarts_visual'}
        self.query_to_kinbody_map = {'fuze_bottle_visual': 'fuze_bottle',
                                     'pop_tarts_visual': 'pop_tarts'}

    @staticmethod
    def _MsgToPose(msg):
        """
        Parse the ROS message to a 4x4 pose format
        @param msg The ros message containing a pose
        @return A 4x4 transformation matrix containing the pose
        as read from the message
        """
        import tf.transformations as transformations
        #Get translation and rotation (from Euler angles)
        pose = transformations.quaternion_matrix(numpy.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]))
    
        pose[0,3] = msg.pose.position.x
        pose[1,3] = msg.pose.position.y
        pose[2,3] = msg.pose.position.z
        
	return pose

    def _GetDetections(self, obj_names):
        import simtrack_msgs.srv
        """
        Calls the service to get a detection of a particular object.
        @param obj_name The name of the object to detect
        @return A 4x4 transformation matrix describing the pose of the object
        in world frame, None if the object is not detected
        """
        #Call detection service for a particular object
        detect_simtrack = rospy.ServiceProxy(self.service_namespace+'/detect_objects',
                                         simtrack_msgs.srv.DetectObjects)
        
        detect_resp = detect_simtrack(obj_names, 5.0)

        
        detections = []

        for i in xrange(0, len(detect_resp.detected_models)) :
            obj_name = detect_resp.detected_models[i];
            obj_pose = detect_resp.detected_poses[i];
            obj_pose_tf = self._MsgToPose(obj_pose);
            detections.append((obj_name, obj_pose_tf));

        return detections

    @PerceptionMethod
    def DetectObject(self, robot, obj_name, **kw_args):
        """
        Detect a single object
        @param robot The OpenRAVE robot
        @param obj_name The name of the object to detect
        @returns A kinbody placed in the detected location
        @throws PerceptionException if the object is not detected
        """

        from prpy.perception.base import PerceptionException

        if obj_name not in self.kinbody_to_query_map:
            raise PerceptionException('The simtrack module cannot detect object %s', obj_name)

        query_name = self.kinbody_to_query_map[obj_name]
        obj_poses = self._GetDetections(query_name)
        if len(obj_poses is 0):
            raise PerceptionException('Failed to detect object %s', obj_name)
        
        obj_pose = None

        for (name, pose) in obj_poses:
            if (name is query_name):
                obj_pose = pose
                break;

        if (obj_pose is None):
            raise PerceptionException('Failed to detect object %s', obj_name)  

        env = robot.GetEnv()
        if env.GetKinBody(obj_name) is None:
            from prpy.rave import add_object
            kinbody_file = '%s.kinbody.xml' % obj_name
            new_body = add_object(
                env,
                obj_name,
                os.path.join(self.kinbody_path, kinbody_file))
            
        body = env.GetKinBody(obj_name)

        # Apply a transform to the kinbody to put it in the 
        #  desired location in OpenRAVE
        from kinbody_helper import transform_to_or
        transform_to_or(kinbody=body,
                        detection_frame=self.detection_frame,
                        destination_frame=self.destination_frame,
                        reference_link=self.reference_link)

        return body

    @PerceptionMethod 
    def DetectObjects(self, robot, **kw_args):
        """
        Overriden method for detection_frame
        """
        from prpy.perception.base import PerceptionException
        from kinbody_helper import transform_to_or

        env = robot.GetEnv()
        # Detecting empty list will detect all possible objects
        detections = self._GetDetections([])

        for (obj_name, obj_pose) in detections:
            if (obj_name not in self.query_to_kinbody_map):
                continue

            kinbody_name = self.query_to_kinbody_map[obj_name]
            if env.GetKinBody(kinbody_name) is None:
                from prpy.rave import add_object
                kinbody_file = '%s.kinbody.xml' % kinbody_name
                new_body = add_object(
                    env,
                    kinbody_name,
                    os.path.join(self.kinbody_path, kinbody_file))


            body = env.GetKinBody(kinbody_name)
            transform_to_or(kinbody=body,
                            detection_frame=self.detection_frame,
                            destination_frame=self.destination_frame,
                            reference_link=self.reference_link)


