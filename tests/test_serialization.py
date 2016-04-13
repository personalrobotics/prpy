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
        
        import IPython; IPython.embed()

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
        dof_values = [0]*self.robot.GetDOF()
        self.robot.SetDOFValues(dof_values)
        self.robot.SetActiveDOFs(range(2,7))

    def compare_joints(self, j1, j2):
        self.assertEqual(j1.GetFirstAttached().GetName(),
                         j2.GetFirstAttached().GetName())
        self.assertEqual(j1.GetSecondAttached().GetName(),
                         j2.GetSecondAttached().GetName())
        self.assertTrue(numpy.allclose(j1.GetFirstAttached().GetTransform(),
                                       j2.GetFirstAttached().GetTransform()))
        self.assertTrue(numpy.allclose(j1.GetSecondAttached().GetTransform(),
                                       j2.GetSecondAttached().GetTransform()))
        self.assertTrue(numpy.allclose(j1.GetInternalHierarchyLeftTransform(),
                                       j2.GetInternalHierarchyLeftTransform()))
        if not numpy.allclose(j1.GetInternalHierarchyRightTransform(),
                                        j2.GetInternalHierarchyRightTransform()):
        #     #print(j1.GetInfo()._vaxes)
        #     #print(j2.GetInfo()._vaxes)
        #     print(j1.GetName())
        #     print(j2.GetName())
        #     print(j1.GetDOF())
        #     print(j2.GetDOF())
        #     print('%s -> %s' % (j1.GetInternalHierarchyRightTransform(), j2.GetInternalHierarchyRightTransform()))
             import IPython; IPython.embed()
        # self.assertTrue(numpy.allclose(j1.GetInternalHierarchyRightTransform(),
        #                                j2.GetInternalHierarchyRightTransform()))

        for aidx in range(j1.GetDOF()):
            self.assertAlmostEqual(j1.GetAxis()[aidx],
                                   j2.GetAxis()[aidx])
            self.assertEqual(j1.GetMimicEquation(aidx),
                             j2.GetMimicEquation(aidx))

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

        SO = openravepy.SerializationOptions

        v1, df = self.robot.GetLinkTransformations(True)
        _, df2 = robot_new.GetLinkTransformations(True)
        robot_new.SetLinkTransformations(v1)
        print(df)
        print(df2)

        # Joints
        self.assertItemsEqual([j.GetName() for j in self.robot.GetJoints()],
                         [j.GetName() for j in robot_new.GetJoints()])
        old_joints = self.robot.GetJoints()
        new_joints = robot_new.GetJoints()
        for idx in range(len(old_joints)):
            self.compare_joints(old_joints[idx], new_joints[idx])

        # Passive joints
        self.assertItemsEqual([j.GetName() for j in self.robot.GetPassiveJoints()],
                         [j.GetName() for j in robot_new.GetPassiveJoints()])
        old_joints = self.robot.GetPassiveJoints()
        new_joints = robot_new.GetPassiveJoints()
        for idx in range(len(old_joints)):
            self.compare_joints(old_joints[idx], new_joints[idx])

        # Links
        self.assertItemsEqual([l.GetName() for l in self.robot.GetLinks()],
                         [l.GetName() for l in robot_new.GetLinks()])
        old_links = self.robot.GetLinks()
        new_links = robot_new.GetLinks()
        for idx in range(len(old_links)):
            glist1 = old_links[idx].GetGeometries()
            glist2 = new_links[idx].GetGeometries()
            self.assertEqual(len(glist1), len(glist2))
            
            for gidx in range(len(glist1)):
                g1 = glist1[gidx]
                g2 = glist2[gidx]
                self.assertTrue(numpy.allclose(g1.GetInfo()._t, g2.GetInfo()._t))
                self.assertEqual(g1.GetInfo()._type, g2.GetInfo()._type)
                self.assertTrue(numpy.allclose(g1.GetInfo()._vRenderScale, g2.GetInfo()._vRenderScale))
                
                if g1.GetInfo()._type == openravepy.GeometryType.Trimesh:
                    m1 = g1.GetInfo()._meshcollision
                    m2 = g2.GetInfo()._meshcollision
                    self.assertEqual(len(m1.indices), len(m2.indices))
                    self.assertEqual(len(m1.vertices), len(m2.vertices))
                    
                    for midx in range(len(m1.indices)):
                        self.assertTrue(numpy.allclose(m1.indices[midx], m2.indices[midx]))
                    for midx in range(len(m1.vertices)):
                        self.assertTrue(numpy.allclose(m1.vertices[midx], m2.vertices[midx]))
                else:
                    self.assertTrue(numpy.allclose(g1.GetInfo()._vGeomData, g2.GetInfo()._vGeomData))

        s1 = self.robot.serialize(SO.Geometry)
        s2 = robot_new.serialize(SO.Geometry)
        self.assertEqual(s1, s2)

#        s1 = self.robot.serialize(SO.Kinematics)
#        s2 = robot_new.serialize(SO.Kinematics)
#        self.assertEqual(s1, s2)

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
