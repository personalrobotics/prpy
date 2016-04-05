from __future__ import print_function
import numpy, openravepy, os, subprocess, unittest

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

class SerializationTests(unittest.TestCase):
    """
    Tests of the prpy serialization functionality
    """
    def setUp(self):
        # import herbpy
        # self.env, self.robot = herbpy.initialize(sim=True)
        self.env = openravepy.Environment()
        self.env.Load('wamtest1.env.xml')
        self.robot = self.env.GetRobot('BarrettWAM')
        self.manipulator = self.robot.GetManipulator('arm')
        
        # Remove the whiteboard, it has multiple unnamed links 
        wnb = self.env.GetKinBody('whiteboard')
        self.env.Remove(wnb)

        # Remove ketchup - it has linked iv files
        for b in self.env.GetBodies():
            if 'ketchup' in b.GetURI():
                self.env.Remove(b)
        
        # Random dof values
        lower, upper = self.robot.GetDOFLimits()
        import random
        dof_values = [l + random.random()*(u-l) for l,u in zip(lower,upper)]
        self.robot.SetDOFValues(dof_values)
        self.robot.SetActiveDOFs(range(2,7))

    def test_RobotSerialization(self):
        from prpy.serialization import serialize, serialize_environment
        from prpy.serialization import ReturnTransformQuaternionStateSaver
        from prpy.serialization import deserialize, deserialize_environment
        from prpy.serialization_database import SerializationDatabase
        db = SerializationDatabase('/tmp')

        with ReturnTransformQuaternionStateSaver(True):
            env_serialized = serialize_environment(self.env, database=db)
            robot_serialized = serialize(self.robot, database=db)
            
            env_new = deserialize_environment(env_serialized, database=db)
            robot_new = deserialize(env_new, robot_serialized, database=db)

        # Joints
        self.assertItemsEqual([j.GetName() for j in self.robot.GetJoints()],
                         [j.GetName() for j in robot_new.GetJoints()])

        # Links
        self.assertItemsEqual([l.GetName() for l in self.robot.GetLinks()],
                         [l.GetName() for l in robot_new.GetLinks()])

        #self.assertEqual(self.robot.GetKinematicsGeometryHash(),
        #                 robot_new.GetKinematicsGeometryHash())
        #self.assertEqual(self.robot.GetRobotStructureHash(),
        #                 robot_new.GetRobotStructureHash())

        # Manipulator hashes
        for manip1 in self.robot.GetManipulators():
            manip2 = robot_new.GetManipulator(manip1.GetName())
            self.assertEqual(manip1.GetKinematicsStructureHash(),
                             manip2.GetKinematicsStructureHash())
            
        # Robot pose
        self.assertTrue(numpy.allclose(self.robot.GetTransform(),
                                       robot_new.GetTransform()))

        # DOF values
        old_vals = self.robot.GetDOFValues()
        new_vals = robot_new.GetDOFValues()
        self.assertEqual(len(old_vals), len(new_vals))
        for idx in range(len(old_vals)):
            self.assertAlmostEqual(old_vals[idx], new_vals[idx])

        # Active DOF
        self.assertEqual(self.robot.GetActiveDOF(), robot_new.GetActiveDOF())
        old_vals = self.robot.GetActiveDOFIndices()
        new_vals = robot_new.GetActiveDOFIndices()
        self.assertEqual(len(old_vals), len(new_vals))
        for idx in range(len(old_vals)):
            self.assertAlmostEqual(old_vals[idx], new_vals[idx])

        # Bodies
        for b in self.env.GetBodies():
            new_body = env_new.GetKinBody(b.GetName());
            self.assertIsNotNone(new_body)
            #self.assertEqual(b.GetKinematicsGeometryHash(),
            #                 new_body.GetKinematicsGeometryHash())
            self.assertTrue(numpy.allclose(b.GetTransform(), new_body.GetTransform()))
            

if __name__ == '__main__':
    unittest.main()
