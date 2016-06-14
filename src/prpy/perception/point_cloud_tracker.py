from tracking import TrackingModule, TrackingMethod
import openravepy

class PointCloudTracker(TrackingModule):

    def __init__(self,env,depth_info='/head/kinect2/qhd/camera_info',pcl_topic='/head/kinect2/qhd/points'):
        """
        Initializes an OpenRAVE plugin that tracks the point-cloud
        corresponding to some provided KinBody
        @param env The OpenRAVE robot
        @depth_info The camera information of depth topic 
        @pcl_topic 
        """

        self.env = env
        self.depth_info = depth_info
        self.pcl_topic = pcl_topic
        self.tracked_objects = list()

        #Create tracker module
        self.tracker = openravepy.RaveCreateModule(env, "object_tracker")

        #Initialize tracker 
        tracker_init_message = 
        self.tracker.SendCommand('Initialize '+self.depth_info+' '+self.pcl_topic)

    @TrackingMethod
    def StartTrackObject(self,obj_name,n_iters=300):
        """
        Begin tracking the OpenRAVE object - blocks and updated kinbody pose
        @param obj_name The name of the kinbody to track
        @n_iters The number of tracking iterations it is blocked for
        """
        from prpy.perception.base import PerceptionException
        if obj_name in self.tracked_objects:
            raise TrackingException('The object %s is already being tracked' % obj_name)
        
        self.tracked_objects.append(obj_name)
        self.tracker.SendCommand("Track "+obj_name+' '+`n_iters`)

    @TrackingMethod
    def StopTrackObject(self,obj_name):
        """
        Stop tracking given object. Currently nothing because PCL tracker stops after iterations
        @param obj_name The name of the object to stop tracking
        """

        if obj_name not in self.tracked_objects:
            print '{0} is not being tracked!'.format(obj_name)
        else:
            self.tracked_objects.remove(obj_name)
