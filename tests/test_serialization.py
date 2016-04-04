from __future__ import print_function
import openravepy, os, subprocess, unittest

# Add the models included with OpenRAVE to the OPENRAVE_DATA path.
# These may not be available if the user manually set the OPENRAVE_DATA
# environmental variable, e.g. through openrave_catkin.
try:
    share_path = \
          subprocess.check_output(['openrave-config', '--share-dir']).strip()
    os.environ['OPENRAVE_DATA'] = os.path.join(share_path, 'data')
except subprocess.CalledProcessError as e:
    print('error: Failed using "openrave-config" to find the default'
          ' OPENRAVE_DATA path. Loading assets may fail.',
          file=sys.stderr)

# Initialize OpenRAVE.
openravepy.RaveInitialize(True)
openravepy.misc.InitOpenRAVELogging()
openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)

class SerializationTests(object):#unittest.TestCase):
    """
    Tests of the prpy serialization functionality
    """
    def setUp(self):
        self.env = openravepy.Environment()
        self.env.Load('wamtest1.env.xml')
        self.robot = self.env.GetRobot('BarrettWAM')
        self.manipulator = self.robot.GetManipulator('arm')

        # Remove the whiteboard, it has multiple unnamed links 
        wnb = self.env.GetKinBody('whiteboard')
        self.env.Remove(wnb)
        

    def test_RobotSerialization(self):
        from prpy.serialization import serialize, serialize_environment
        from prpy.serialization import deserialize, deserialize_environment
        env_dict = serialize_environment(self.env)
        robot_dict = serialize(self.robot)

        env_new = deserialize_environment(env_dict)
        robot_new = deserialize(env_new, robot_dict)

        self.assertEqual(self.robot.GetKinematicsGeometryHash(),
                         robot_new.GetKinematicsGeometryHash())
        self.assertEqual(self.robot.GetRobotStructureHash(),
                         robot_new.GetRobotStructureHash())
        
        for manip1 in self.robot.GetManipulators():
            manip2 = robot_new.GetManipulator(manip1.GetName())
            self.assertEqual(manip1.GetKinematicsStructureHash(),
                             manip2.GetKinematicsStructureHash())

        self.assertEqual([j.GetName() for j in self.robot.GetJoints()],
                         [j.GetName() for j in robot_new.GetJoints()])

        
