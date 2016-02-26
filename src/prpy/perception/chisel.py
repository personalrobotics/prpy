import rospy
import chisel_msgs.srv
import chisel_msgs.msg
import math
import tf
import tf.transformations as transformations
import numpy
import os.path

'''
Typical way of running

from prpy.perception.chisel import ChiselModule
from offscreen_render import ros_camera_sim
robot.DetectObjects()
table = env.GetKinBody('table127')
glass = env.GetKinBody('plastic_glass124')
camera = ros_camera_sim.RosCameraSim(env)
camera.start('/head/kinect2/qhd')
camera.add_body(table)
camera.add_body(glass)

Nodelet manager screen
rosrun nodelet nodelet manager __name:=nodelet_manager
Depth converter screen
rosrun nodelet nodelet load depth_image_proc/convert_metric nodelet_manager image_raw:=/head/kinect2/qhd/image_depth_rect image:=/head/kinect2/qhd/image_depth_rect_float
In herb2, source herb2_chisel_ws/devel/setup.bash
Depth mask screen
roslaunch depth_mask launch_depth_mask.launch (args are correct)

Chisel screen
roslaunch chisel_ros launch_kinect_local.launch

IN PYTHON
mesh_client = openravepy.RaveCreateSensorSystem(env, 'mesh_marker')
chisel_det = ChiselModule(mesh_client,'Chisel')

robot.DetectObjects()
#reset chisel mesh
mesh_client.SendCommand('GetMesh Chisel/full_mesh')
'''

class ChiselModule():

	def __init__(self, env, mesh_client=None, service_namespace=None,
                     reference_link=None):

		try:
			rospy.init_node('chisel_detector',anonymous=True)
		except rospy.exceptions.ROSException:
			pass

		if service_namespace is None:
			service_namespace = 'Chisel'

                if mesh_client is None:
                        import openravepy
                        mesh_client = openravepy.RaveCreateSensorSystem(env, 'mesh_marker')

		self.mesh_client = mesh_client
		self.serv_ns = service_namespace

                from offscreen_render import ros_camera_sim
                self.camera = ros_camera_sim.RosCameraSim(env)
                self.camera.start('/head/kinect2/qhd')
                self.camera_bodies = []

                self.detect_chisel_refresh=None
                self.reference_link=reference_link

                import tf
                self.listener = tf.TransformListener()

	def DetectObject(self, robot, ignored_bodies=None, **kw_args):
    
		env = robot.GetEnv()
		already_in_env = False

                if ignored_bodies is None:
                        ignored_bodies = []

                for b in ignored_bodies:
                        if b not in self.camera_bodies:
                                self.camera.add_body(b)
                                self.camera_bodies.append(b)

		#Check if kinbody in env
		for body in env.GetBodies():
			if body.GetName() == 'Chisel/full_mesh':
				already_in_env = True
				break

		#Remove kinbody if present
		if already_in_env:
			chisel_kb = env.GetKinBody('Chisel/full_mesh')
			env.RemoveKinBody(chisel_kb)

		#Reset Chisel
                if self.detect_chisel_refresh is None:
                        srv_nm = self.serv_ns+'/Reset'
                        rospy.wait_for_service(srv_nm)
                        detect_chisel_refresh = rospy.ServiceProxy(srv_nm,
                                                                   chisel_msgs.srv.ResetService)
                detect_chisel_refresh()

		#Get Kinbody and load into env
		self.mesh_client.SendCommand('GetMesh Chisel/full_mesh')

                self.listener.waitForTransform(
                       '/map',
                       '/herb_base',
                        rospy.Time(),
                        rospy.Duration(10))
                frame_trans, frame_rot = self.listener.lookupTransform(
                        '/herb_base', '/map',
                        rospy.Time(0))
                from tf.transformations import quaternion_matrix
                offset = numpy.array(numpy.matrix(quaternion_matrix(frame_rot)))
                offset[0,3] = frame_trans[0]
                offset[1,3] = frame_trans[1]
                offset[2,3] = frame_trans[2]

                chisel_mesh = env.GetKinBody('Chisel/full_mesh')
                if chisel_mesh is not None and self.reference_link is not None:
                        print 'Moving mesh'
                        t = numpy.dot(offset, chisel_mesh.GetTransform())
                        ref_link_pose = self.reference_link.GetTransform()
                        t = numpy.dot(ref_link_pose, t)
                        chisel_mesh.SetTransform(t)
                        
                return chisel_mesh
