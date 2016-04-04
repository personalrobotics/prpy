import rospy
import vncc_msgs.srv
import vncc_msgs.msg
import math
import tf
import tf.transformations as transformations
import numpy
import os.path

from base import PerceptionModule, PerceptionMethod


class VnccModule(PerceptionModule):

    def __init__(self, kinbody_path, detection_frame, world_frame,
                 service_namespace=None):
        """
        This initializes a VNCC detector.
        
        @param kinbody_path The path to the folder where kinbodies are stored
        @param detection_frame The TF frame of the camera
        @param world_frame The desired world TF frame
        @param service_namespace The namespace for the VNCC service (default: /vncc)
        """

        # Initialize a new ros node if one has not already been created
        try:
            rospy.init_node('vncc_detector', anonymous=True)
        except rospy.exceptions.ROSException:
            pass
            
        #For getting transforms in world frame
        if detection_frame is not None and world_frame is not None:
            self.listener = tf.TransformListener()
        else:
            self.listener = None
                
        if service_namespace is None:
            service_namespace='/vncc'

        self.detection_frame = detection_frame
        self.world_frame = world_frame
        self.service_namespace = service_namespace
            
        self.kinbody_path = kinbody_path

        # A map of known objects that can be queries
        self.kinbody_to_query_map = {'plastic_plate': 'plate',
                                     'plastic_bowl': 'bowl'}

    @staticmethod
    def _MsgToPose(msg):
        """
        Parse the ROS message to a 4x4 pose format
        @param msg The ros message containing a pose
        @return A 4x4 transformation matrix containing the pose
        as read from the message
        """
        #Get translation and rotation (from Euler angles)
        pose = transformations.euler_matrix(msg.roll*0.0,msg.pitch*0.0,msg.yaw*0.0) 
    
        pose[0,3] = msg.pt.x
        pose[1,3] = msg.pt.y
        pose[2,3] = msg.pt.z
        
	return pose

    def _LocalToWorld(self,pose):
        """
        Transform a pose from local frame to world frame
        @param pose The 4x4 transformation matrix containing the pose to transform
        @return The 4x4 transformation matrix describing the pose in world frame
        """
        #Get pose w.r.t world frame
        self.listener.waitForTransform(self.world_frame,self.detection_frame,
                                       rospy.Time(),rospy.Duration(10))
        t, r = self.listener.lookupTransform(self.world_frame,self.detection_frame,
                                             rospy.Time(0))
        
        #Get relative transform between frames
        offset_to_world = numpy.matrix(transformations.quaternion_matrix(r))
        offset_to_world[0,3] = t[0]
        offset_to_world[1,3] = t[1]
        offset_to_world[2,3] = t[2]
        
        #Compose with pose to get pose in world frame
        result = numpy.array(numpy.dot(offset_to_world, pose))
        
        return result
        

    def _GetDetection(self, obj_name):
        """
        Calls the service to get a detection of a particular object.
        @param obj_name The name of the object to detect
        @return A 4x4 transformation matrix describing the pose of the object
        in world frame, None if the object is not detected
        """
        #Call detection service for a particular object
        detect_vncc = rospy.ServiceProxy(self.service_namespace+'/get_vncc_detections',
                                         vncc_msgs.srv.GetDetections)
        
        detect_resp = detect_vncc(object_name=obj_name)
        
        if detect_resp.ok == False:
            return None
            
        #Assumes one instance of object
        result = self._MsgToPose(detect_resp.detections[0])
        if (self.detection_frame is not None and self.world_frame is not None):
            result = self._LocalToWorld(result)
        result[:3,:3] = numpy.eye(3)
        return result

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
            raise PerceptionException(
                'The VNCC module cannot detect object {:s}'.format(obj_name))

        query_name = self.kinbody_to_query_map[obj_name]
        obj_pose = self._GetDetection(query_name)
        if obj_pose is None:
            raise PerceptionException(
                'Failed to detect object {:s}'.format(obj_name))
            
        env = robot.GetEnv()
        if env.GetKinBody(obj_name) is None:
            from prpy.rave import add_object
            kinbody_file = '{:s}.kinbody.xml'.format(obj_name)
            new_body = add_object(
                env,
                obj_name,
                os.path.join(self.kinbody_path, kinbody_file))
            
        body = env.GetKinBody(obj_name)
        body.SetTransform(obj_pose)
        return body
