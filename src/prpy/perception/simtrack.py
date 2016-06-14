import math
import numpy
import os.path

from base import PerceptionModule, PerceptionMethod
from tracking import TrackingModule, TrackingMethod

class SimtrackModule(PerceptionModule, TrackingModule):

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
        
        # A dictionary of subscribers to topic - one for each body
        self.track_sub = {}

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
        """
        Calls the service to get a detection of a particular object.
        @param obj_name The name of the object to detect
        @return A 4x4 transformation matrix describing the pose of the object
        in world frame, None if the object is not detected
        """
        import simtrack_msgs.srv, rospy

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
        obj_poses = self._GetDetections([query_name])
        if len(obj_poses) == 0:
            raise PerceptionException('Failed to detect object %s', obj_name)
        
        obj_pose = None

        print query_name
        for (name, pose) in obj_poses:
            print  name
            if name == query_name:
                obj_pose = pose
                break;

        if (obj_pose is None):
            raise PerceptionException('Failed to detect object %s', obj_name)  
            
        if numpy.array_equal(obj_pose, numpy.eye(4)):
            raise PerceptionException('Object %s detected but failed to compute pose', obj_name)

        env = robot.GetEnv()
        if env.GetKinBody(obj_name) is None:
            from prpy.rave import add_object
            kinbody_file = '%s.kinbody.xml' % obj_name
            new_body = add_object(
                env,
                obj_name,
                os.path.join(self.kinbody_path, kinbody_file))
            
        print 'Object pose: ', obj_pose
        body = env.GetKinBody(obj_name)

        # Apply a transform to the kinbody to put it in the 
        #  desired location in OpenRAVE
        from kinbody_helper import transform_to_or
        transform_to_or(kinbody=body,
                        detection_frame=self.detection_frame,
                        destination_frame=self.destination_frame,
                        reference_link=self.reference_link,
                        pose=obj_pose)

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
                            reference_link=self.reference_link,
                            pose=obj_pose)

    def _UpdatePose(self, msg, kwargs):
        """
        Update the pose of an object
        """
        pose_tf = self._MsgToPose(msg)

        robot = kwargs['robot']
        obj_name = kwargs['obj_name']
        detection_in_destination = kwargs['detection_in_destination']

        env = robot.GetEnv()
        with env:
             obj = env.GetKinBody(obj_name)
        if obj is None:
            from prpy.rave import add_object
            kinbody_file = '%s.kinbody.xml' % obj_name
            new_body = add_object(
                env,
                obj_name,
                os.path.join(self.kinbody_path, kinbody_file))

        with env:
             obj = env.GetKinBody(obj_name)
                                              
        from kinbody_helper import transform_to_or
        if detection_in_destination is not None:
            transform_to_or(kinbody=obj,
                            detection_in_destination=detection_in_destination,
                            reference_link=self.reference_link,
                            pose=pose_tf)
        else:
            transform_to_or(kinbody=obj,
                            detection_frame=self.detection_frame,
                            destination_frame=self.destination_frame,
                            reference_link=self.reference_link,
                            pose=pose_tf)

    @TrackingMethod
    def StartTrackObject(self, robot, obj_name, cache_transform=True, **kw_args):
        """
        Subscribe to the pose array for an object in order to track
        @param robot The OpenRAVE robot
        @param obj_name The name of the object to track
        @param cache_transform If true, lookup the transform from detection_frame
          to destination_frame once, cache it and use it for the duration of tracking.
          This speeds up tracking by removing the need to perform a tf lookup
          in the callback.
        """
        import rospy
        from geometry_msgs.msg import PoseStamped
        from prpy.perception.base import PerceptionException

        if obj_name not in self.kinbody_to_query_map:
            raise PerceptionException('The simtrack module cannot track object %s' & obj_name)
        query_name = self.kinbody_to_query_map[obj_name]
        pose_topic = self.service_namespace + '/' + query_name

        if obj_name in self.track_sub and self.track_sub[obj_name] is not None:
            raise PerceptionException('The object %s is already being tracked' % obj_name)

        #  These speeds up tracking by removing the need to do a tf lookup
        detection_in_destination=None
        if cache_transform:
            import numpy, tf, rospy
            listener = tf.TransformListener()
            
            listener.waitForTransform(
                self.detection_frame, self.destination_frame,
                rospy.Time(),
                rospy.Duration(10))
            
            frame_trans, frame_rot = listener.lookupTransform(
                self.destination_frame, self.detection_frame,
                rospy.Time(0))
        
            from tf.transformations import quaternion_matrix
            detection_in_destination = numpy.array(numpy.matrix(quaternion_matrix(frame_rot)))
            detection_in_destination[:3,3] = frame_trans

        self.track_sub[obj_name] = rospy.Subscriber(pose_topic, 
                                                    PoseStamped, 
                                                    callback=self._UpdatePose,
                                                    callback_args={
                                                        'robot':robot, 
                                                        'obj_name':obj_name,
                                                        'detection_in_destination': detection_in_destination},
                                                    queue_size=1
        )

    @TrackingMethod
    def StopTrackObject(self, robot, obj_name, **kw_args):
        """
        Unsubscribe from the pose array to end tracking
        @param robot The OpenRAVE robot
        @param obj_name The name of the object to stop tracking
        """
        if obj_name in self.track_sub and self.track_sub[obj_name] is not None:
            self.track_sub[obj_name].unregister()            
            self.track_sub[obj_name] = None
