import rospy
import vncc_msgs.srv
import vncc_msgs.msg
import math
import tf
import tf.transformations as transformations
import numpy
import os.path


def MsgToPose(msg):

	#Parse the ROS message to a 4x4 pose format

	#Get translation and rotation (from Euler angles)
	pose = transformations.euler_matrix(msg.roll*0.0,msg.pitch*0.0,msg.yaw*0.0) 

        pose[0,3] = msg.pt.x
        pose[1,3] = msg.pt.y
        pose[2,3] = msg.pt.z
    
	return pose


class VnccDetector(object):

	def __init__(self, env, kinbody_directory,
                     service_namespace=None,
                     detection_frame=None,world_frame=None):


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
            
            self.env = env
            self.kinbody_directory = kinbody_directory
            self.kinbody_to_query_map = {'plastic_plate': 'plate',
                                         'plastic_bowl': 'bowl'}

	def LocalToWorld(self,pose):
		#Get pose w.r.t world frame
		self.listener.waitForTransform(self.world_frame,self.detection_frame,
                                       rospy.Time(),rospy.Duration(10))
		t, r = self.listener.lookupTransform(self.world_frame,self.detection_frame,rospy.Time(0))

		#Get relative transform between frames
		offset_to_world = numpy.matrix(transformations.quaternion_matrix(r))
		offset_to_world[0,3] = t[0]
                offset_to_world[1,3] = t[1]
                offset_to_world[2,3] = t[2]

		#Compose with pose to get pose in world frame
		result = numpy.array(numpy.dot(offset_to_world, pose))

		return result


	def GetDetection(self,obj_name):

		#Call detection service for a particular object
		detect_vncc = rospy.ServiceProxy(self.service_namespace+'/get_vncc_detections',
                                                 vncc_msgs.srv.GetDetections)

		detect_resp = detect_vncc(object_name=obj_name)

		if detect_resp.ok == False:
			return None
		#Assumes one instance of object
                result = MsgToPose(detect_resp.detections[0])
		if (self.detection_frame is not None and self.world_frame is not None):
		   result = self.LocalToWorld(result)

		return result

        def DetectObject(self, obj_name):

            if obj_name not in self.kinbody_to_query_map:
                from prpy.perception.base import PerceptionException
                raise PerceptionException('Unknown object')

            query_name = self.kinbody_to_query_map[obj_name]
            obj_pose = self.GetDetection(query_name)

            if self.env.GetKinBody(obj_name) is None:
                from prpy.rave import add_object
                kinbody_file = os.path.join('objects', '%s.kinbody.xml' % obj_name)
                new_body = add_object(
                    self.env,
                    obj_name,
                    os.path.join(self.kinbody_directory, kinbody_file))

            body = self.env.GetKinBody(obj_name)
            body.SetTransform(obj_pose)
            return body
