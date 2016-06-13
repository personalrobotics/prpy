import logging
import openravepy
from base import PerceptionModule, PerceptionMethod

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class ChiselModule(PerceptionModule):
    
    def __init__(self, env, service_namespace='Chisel', mesh_name = 'Chisel/full_mesh',
                detection_frame='/herb_base', destination_frame='/map', reference_link=None):
        """
        The perception module implementation for CHISEL. It converts all unmodelled clutter from the point 
        cloud to an OpenRAVE Kinbody - after masking out specified known objects already in the Environment.

        @param env The OpenRAVE environment
        @param service_namespace The namespace to use for Chisel service calls (default: Chisel)
        @param mesh_name The name of the topic on which the Chisel marker is published.
        @param detection_frame The frame in which the Chisel mesh is detected. (default: map)
        @param destination_frame The tf frame the detected chisel kinbody should be transformed to (default: map)
        @param reference_link A link on an OpenRAVE kinbody that corresponds to the tf frame
        specified by the destination_frame parameter. If not provided it is assumed that the 
        transform from the destination_frame tf frame to the OpenRAVE environment is the identity
        """
        import rospy

        self.mesh_client = openravepy.RaveCreateSensorSystem(env, 'mesh_marker')

        if self.mesh_client is None:
            raise RuntimeError('Could not create mesh client')

        self.serv_ns = service_namespace
        self.mesh_name = mesh_name

        # Create a ROS camera used to generate a synthetic point cloud
        #  to mask known kinbodies from the chisel mesh
        from offscreen_render import ros_camera_sim
        self.camera = ros_camera_sim.RosCameraSim(env)
        self.camera.start('/head/kinect2/qhd')
        self.camera_bodies = []

        self.reset_detection_srv = None
        self.detection_frame = detection_frame
        self.destination_frame = destination_frame
        self.reference_link = reference_link

    @PerceptionMethod
    def DetectObject(self, robot, ignored_bodies=None, timeout=5, **kw_args):
        """
        Obtain the KinBody corresponding to the Chisel Mesh after masking out
        any existing kinbodies if desired (as specified in ignored_bodies).

        @param robot Required by the PerceptionMethod interface
        @param ignored_bodies A list of known kinbodies to be masked out
        @param timeout The timeout for which to wait for the Chisel Mesh
        of the chisel mesh
        """
        env = robot.GetEnv()
        
        if ignored_bodies is None:
            ignored_bodies = []

        #Check if chisel kinbody in env
        chisel_kb = env.GetKinBody(self.mesh_name)
        if chisel_kb is not None:
            env.RemoveKinBody(chisel_kb)

        with env:
            try:
                from kinbody_helper import transform_from_or
                import openravepy
                # Add any ignored bodies to the camera
                # We need to transform all these bodies from their pose
                #   in OpenRAVE to the equivalent pose in TF so that
                #   chisel can perform the appropriate masking
                #   Then we will transform them all back after the server
                #   performs the detection
                for b in ignored_bodies:
                    if b not in self.camera_bodies:
                        poses = self.SamplePoses(b.GetTransform())
                        orig_name = b.GetName()

                        for idx, pose in enumerate(poses):
                            bcloned = openravepy.RaveCreateKinBody(env, b.GetXMLId())
                            bcloned.Clone(b, 0)
                            
                            bcloned.SetName('%s_%d' % (orig_name, idx))
                            env.Add(bcloned)
                            transform_from_or(kinbody=bcloned,
                                              pose=pose,
                                              detection_frame='/map',
                                              destination_frame=self.destination_frame,
                                              reference_link=self.reference_link)
                            self.camera.add_body(bcloned)
                            self.camera_bodies.append(bcloned)

                #Reset Chisel
                import rospy, chisel_msgs.srv
                srv_nm = self.serv_ns+'/Reset'
                rospy.wait_for_service(srv_nm,timeout)
                reset_detection_srv = rospy.ServiceProxy(srv_nm,
                                                              chisel_msgs.srv.ResetService)
                reset_detection_srv()

                #Get Kinbody and load into env
                self.mesh_client.SendCommand('GetMesh '+self.mesh_name)
                chisel_mesh = env.GetKinBody(self.mesh_name)

            finally:
                
                # Remove any previous bodies in the camera
                for b in self.camera_bodies:
                    self.camera.remove_body_now(b)
                    env.Remove(b)
                
        if chisel_mesh is not None and self.reference_link is not None:
            # Apply a transform to the chisel kinbody to put it in the correct
            # location in the OpenRAVE environment
            from kinbody_helper import transform_to_or
            transform_to_or(kinbody=chisel_mesh,
                            detection_frame=self.detection_frame,
                            destination_frame=self.destination_frame,
                            reference_link=self.reference_link)
                        
        return chisel_mesh

    def SamplePoses(self, orig_transform, radius=0.03):
        """
        Sample poses around the sphere of the given radius
        Sampled poses maintain orientation but have different translations
        @param orig_transform The original pose
        @param radius The radius of the sphere to sample from
        """
        import copy, numpy
        retvals = []
        for i in [-1., 0., 1.]:
            for j in [-1., 0., 1.]:
                for k in [-1., 0., 1.]:
                    v = numpy.array([i, j, k])
                    if numpy.linalg.norm(v) > 0:
                        v = radius * v / numpy.linalg.norm(v)
                    new_transform = copy.deepcopy(orig_transform)
                    new_transform[:3,3] = orig_transform[:3,3] + v
                    retvals.append(new_transform)
        return retvals
