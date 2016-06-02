from base import TrackingModule, TrackingMethod

class SimTracker(TrackingModule):

    def __init__(self,env,detection_frame, destination_frame = '/map',
        service_namespace='/simtrack'):

        self.env = env
        self.service_namespace = service_namespace

        # A map of known objects that can be queries
        self.kinbody_to_query_map = {'fuze_bottle': 'fuze_bottle_visual',
                                     'pop_tarts': 'pop_tarts_visual'}
        self.query_to_kinbody_map = {'fuze_bottle_visual': 'fuze_bottle',
                                     'pop_tarts_visual': 'pop_tarts'}

        # A dictionary of subscribers to topic - one for each body
        self.track_sub = {}

        self.detection_frame = detection_frame
        self.destination_frame = destination_frame


    @TrackingMethod
    def StartTrackObject(self, robot, obj_name, cache_transform=True):
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
            raise TrackingException('The simtrack tracker cannot track object %s' & obj_name)

        query_name = self.kinbody_to_query_map[obj_name]
        pose_topic = self.service_namespace + '/' + query_name

        if obj_name in self.track_sub and self.track_sub[obj_name] is not None:
            raise TrackingException('The object %s is already being tracked' % obj_name)


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