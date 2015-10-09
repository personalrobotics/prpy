import prpy,os,numpy,sys
import tabletop_perception_tools.msg
from tabletop_perception_tools.srv import *
import rospy
import tf
import geometry_msgs.msg
from tf.transformations import quaternion_matrix,euler_from_matrix,euler_matrix

from base import PerceptionModule, PerceptionMethod

import logging
logger = logging.getLogger(__name__)

table_z_offset = -0.02

class BlockDetector(PerceptionModule):

    def __init__(self,point_cloud_topic,detection_frame,destination_frame):
        """
        Initializes a Block Detector

        @param point_cloud_topic the name of the point-cloud to read from
        @param detection_frame the TF frame of the camera
        @param destination_frame the TF frame of block_in_world
        """

        super(PerceptionModule, self).__init__()

        self.point_cloud_topic = point_cloud_topic
        self.detection_frame = detection_frame
        self.destination_frame = destination_frame
        self.listener = tf.TransformListener()
                
    def __str__(self):
        return 'BlockDetector'


    def find_blocks(self,service_name="tools_server/find_blocks", 
                     cloud_topic="/head/kinect2/qhd/points",
                     segment_planes=True, 
                     num_planes=1, 
                     plane_distance=0.015, 
                     segment_depth=True, 
                     min_depth=0.5, 
                     max_depth=1.5, 
                     cluster_tolerance=0.005, 
                     min_cluster_size=50, 
                     max_cluster_size=300,
                     segment_box=True,
                     box_min = [-0.3, -0.05, 0.6],
                     box_max = [0.3, 0.5, 1.5]):
        """
        @param service_name: name of the ROS service for tabletop_perception_tools
        @param cloud_topic: name of the ROS topic for the colored PointCloud2
        @param segment_planes: discard the largest num_plane planes
        @param num_planes: number of planes to discard, if segment_planes is True
        @param plane_distance: minimum distance from largest plane for points to be accepted
        @param min_depth: minimum depth from the camera
        @param max_depth: minimum depth from the camera
        @param cluster_tolerance: maximum distance between any two points in a cluster
        @param min_cluster_size: minimum number of points in a cluster
        @param max_cluster_size: maximum number of points in a cluster
        @param segment_box: flag to discard points outside of (box_min, box_max)
        @param box_min: minimum coordinsates of search area in camera frame
        @param box_max: maximum coordinsates of search area in camera frame
        """

        logging.info("waiting for service...")
        rospy.wait_for_service(service_name);
        logging.info("Calling service...")
        try:
            box_min_pt = geometry_msgs.msg.Point();
            box_min_pt.x = box_min[0];
            box_min_pt.y = box_min[1];
            box_min_pt.z = box_min[2];
            box_max_pt = geometry_msgs.msg.Point();
            box_max_pt.x = box_max[0];
            box_max_pt.y = box_max[1];
            box_max_pt.z = box_max[2];

            service = rospy.ServiceProxy(service_name, FindBlocks);
            response = service(cloud_topic, segment_planes, num_planes, plane_distance, segment_depth, min_depth, max_depth, cluster_tolerance, min_cluster_size, max_cluster_size, segment_box, box_min_pt, box_max_pt);
            return response.blocks;
        except rospy.ServiceException, e:
            logging.error("Service call failed: %s", str(e))
            return []


    @PerceptionMethod
    def DetectBlocks(self, robot, table, blocks=None,timeout=10, **kw_args):
        """
        Place blocks on the table
        """
        if blocks is not None and len(blocks) == 0:
            # Add all blocks
        
            env = robot.GetEnv()

            # Get the pr-ordata package path
            import prpy.util
            block_path = os.path.join('objects', 'block.kinbody.xml')
            
            detected_blocks = self.find_blocks(cloud_topic=self.point_cloud_topic)
        
            # Place blocks on the table
            from prpy.util import ComputeEnabledAABB
            with prpy.rave.Disabled(table, padding_only=True):
                table_aabb = ComputeEnabledAABB(table)
                z = table_aabb.pos()[2] + table_aabb.extents()[2] + table_z_offset #OFFSET SET AT TOP

            for b in detected_blocks:

                block = env.ReadKinBodyXMLFile(block_path)

                for link in block.GetLinks():
                    for geometry in link.GetGeometries():
                        geometry.SetDiffuseColor(numpy.array([b.avg_color.r,b.avg_color.g,b.avg_color.b,b.avg_color.a]))

                block_pose = numpy.array(quaternion_matrix([
                        b.pose.orientation.x,
                        b.pose.orientation.y,
                        b.pose.orientation.z,
                        b.pose.orientation.w]))
                block_pose[0,3] = b.pose.position.x
                block_pose[1,3] = b.pose.position.y
                block_pose[2,3] = b.pose.position.z

                self.listener.waitForTransform(
                        self.detection_frame,
                        self.destination_frame,
                        rospy.Time(0),
                        rospy.Duration(timeout))

                frame_trans, frame_rot = self.listener.lookupTransform(
                        self.destination_frame,
                        self.detection_frame,
                        rospy.Time(0))

                frame_offset = numpy.matrix(quaternion_matrix(frame_rot))
                frame_offset[0,3] = frame_trans[0]
                frame_offset[1,3] = frame_trans[1]
                frame_offset[2,3] = frame_trans[2]

                block_in_world = numpy.array(numpy.dot(frame_offset,block_pose))
              
                
                #Snap block to table
                
                block_in_world[2,3] = z

                #To snap blocks to upright on table
                
                block_matrix = block_in_world[0:3,0:3]
                ax, ay, az = euler_from_matrix(block_matrix)
                ax = 0
                ay = 0
                block_in_world_corrected = euler_matrix(ax,ay,az)
                block_in_world[0:3,0:3] = block_in_world_corrected[0:3,0:3]
                

                #Set block name - should we change?
                block.SetTransform(block_in_world)
                valid = False
                while not valid:
                    rand_name = 'block' + `int(numpy.random.randint(1,10000))`
                    valid = env.GetKinBody(rand_name) is None
                
                block.SetName(rand_name)
                env.Add(block)
                blocks.append(block)

        return blocks
