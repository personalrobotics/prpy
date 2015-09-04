import prpy,os,numpy,sys
import tabletop_perception_tools.msg
from tabletop_perception_tools.srv import *
import rospy
import tf
import geometry_msgs.msg
from tf.transformations import quaternion_matrix

from base import PerceptionModule, PerceptionMethod

class BlockDetector(PerceptionModule):

    def __init__(self,point_cloud_topic,detection_frame,destination_frame):

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
                     box_min = [-0.5, 0.1, 0.5],
                     box_max = [0.5, 0.5, 1.5]):
	    print "waiting for service..."
	    rospy.wait_for_service(service_name);
	    print "Calling service..."
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
	        print "Service call failed: %s"%e


    @PerceptionMethod
    def DetectObjects(self, robot, **kw_args):
        """
        Detect the table and the bin
        """
        env = robot.GetEnv()

        # Get the pr-ordata package path
        data_dir = prpy.util.FindCatkinResource('pr_ordata', 'data')

        # Place the table in the environment
        table_path = os.path.join(data_dir, 'furniture', 'table.kinbody.xml')
        table = env.ReadKinBodyXMLFile(table_path)
        env.Add(table)
        
        table_in_robot = numpy.array([[0., 0., 1., 1.025],
                                      [1., 0., 0., 0.],
                                      [0., 1., 0., 0.],
                                      [0., 0., 0., 1.]])
        table_in_world = numpy.dot(robot.GetTransform(), table_in_robot)
        table.SetTransform(table_in_world)
        table_aabb = table.ComputeAABB()
        table_height = table_aabb.pos()[2] + table_aabb.extents()[2] 

        # TODO: Add a bin to the edge of the table for placing the blocks into
        tray_path = os.path.join(data_dir, 'objects', 'wicker_tray.kinbody.xml')
        tray = env.ReadKinBodyXMLFile(tray_path)
        tray_aabb = tray.ComputeAABB()
        env.Add(tray)

        xpose = table
        tray_in_table = numpy.array([[1., 0., 0., -0.7607], # right edge - -.99575 + .235
                                     [0., 0., 1., table_height + 0.01],
                                     [0.,-1., 0., 0.],
                                     [0., 0., 0., 1.]])
        tray_in_world = numpy.dot(table.GetTransform(), tray_in_table)
        tray.SetTransform(tray_in_world)

    @PerceptionMethod
    def DetectBlocks(self, robot, table, blocks=[], **kw_args):
        """
        Place blocks on the table
        """
        if len(blocks) == 0:
            # Add all blocks
        
            env = robot.GetEnv()

            # Get the pr-ordata package path
            data_dir = prpy.util.FindCatkinResource('pr_ordata', 'data')
            block_path = os.path.join(data_dir, 'objects', 'block.kinbody.xml')
                    
            # Place blocks in a pattern on the table
            table_aabb = table.ComputeAABB()
            table_height = table_aabb.pos()[2] + table_aabb.extents()[2]
            table_corner = numpy.eye(4)
            table_corner[:3,3] = [table_aabb.pos()[0] - table_aabb.extents()[0],
                                  table_aabb.pos()[1] - table_aabb.extents()[1],
                                  table_height]
            
            detected_blocks = self.find_blocks(cloud_topic=self.point_cloud_topic)


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
                        rospy.Time(),
                        rospy.Duration(5))

                frame_trans, frame_rot = self.listener.lookupTransform(
                        self.destination_frame,
                        self.detection_frame,
                        rospy.Time(0))

                frame_offset = numpy.matrix(quaternion_matrix(frame_rot))
                frame_offset[0,3] = frame_trans[0]
                frame_offset[1,3] = frame_trans[1]
                frame_offset[2,3] = frame_trans[2]

                block_in_world = numpy.array(numpy.dot(frame_offset,block_pose))

                '''
                block_pose = numpy.eye(4)
                block_pose[:2,3] = b['pose']
                block_in_world = numpy.dot(table_corner, block_pose)
                block_in_world[2,3] = table_height + 0.01
                '''
                block.SetTransform(block_in_world)
                rand_name = int(numpy.random.randint(1,10000))
                block.SetName('block'+`rand_name`)
                env.Add(block)
            blocks.append(block)

        return blocks