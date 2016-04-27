import logging
from base import PerceptionModule, PerceptionMethod

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class ChiselModule(PerceptionModule):
    
    def __init__(self, env, mesh_client=None, service_namespace=None,
               destination_frame=None, reference_link=None):
        """
        @param env The OpenRAVE environment
        @param mesh_client An instance of the Chisel sensor system (if None, one is created)
        @param service_namespace The namespace to use for Chisel service calls (default: Chisel)
        @param destination_frame The tf frame the detected chisel kinbody should be transformed to
        (default map)
        @param reference_link A link on an OpenRAVE kinbody that corresponds to the tf frame
        specified by the destination_frame parameter. If not provided it is assumed that the 
        transform from the destination_frame tf frame to the OpenRAVE environment is the identity
        """
        import rospy

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

        # Create a ROS camera used to generate a synthetic point cloud
        #  to mask known kinbodies from the chisel mesh
        from offscreen_render import ros_camera_sim
        self.camera = ros_camera_sim.RosCameraSim(env)
        self.camera.start('/head/kinect2/qhd')
        self.camera_bodies = []

        self.reset_detection_srv = None
        if destination_frame is None:
            destination_frame = "/map"
        self.destination_frame = destination_frame
        self.reference_link = reference_link

    @PerceptionMethod
    def DetectObject(self, robot, ignored_bodies=None, **kw_args):
        """
        @param robot Required by the PerceptionMethod interface
        @param ignored_bodies A list of known kinbodies to be masked out
        of the chisel mesh
        """
        env = robot.GetEnv()
        
        if ignored_bodies is None:
            ignored_bodies = []

        # Remove any previous bodies in the camera
        for b in self.camera_bodies:
            self.camera.remove_body(b)
        self.camera_bodies = []

        # Add any ignored bodies to the camera
        for b in ignored_bodies:
            if b not in self.camera_bodies:
                self.camera.add_body(b)
                self.camera_bodies.append(b)
                
        #Check if chisel kinbody in env
        already_in_env = False
        for body in env.GetBodies():
            if body.GetName() == 'Chisel/full_mesh':
                already_in_env = True
                break

        #Remove chisel kinbody if present
        if already_in_env:
            chisel_kb = env.GetKinBody('Chisel/full_mesh')
            env.Remove(chisel_kb)
                        
        #Reset Chisel
        if self.reset_detection_srv is None:
            import rospy, chisel_msgs.srv
            srv_nm = self.serv_ns+'/Reset'
            rospy.wait_for_service(srv_nm)
            self.reset_detection_srv = rospy.ServiceProxy(srv_nm,
                                                      chisel_msgs.srv.ResetService)
        self.reset_detection_srv()

        #Get Kinbody and load into env
        self.mesh_client.SendCommand('GetMesh Chisel/full_mesh')
        chisel_mesh = env.GetKinBody('Chisel/full_mesh')
        
        if chisel_mesh is not None and self.reference_link is not None:
            # Apply a transform to the chisel kinbody to put it in the correct
            # location in the OpenRAVE environment
            from kinbody_helper import transform_to_or
            transform_to_or(kinbody=chisel_mesh,
                            detection_frame="/map",
                            destination_frame=self.destination_frame,
                            reference_link=self.reference_link)
                        
        return chisel_mesh
