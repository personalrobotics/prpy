from __future__ import print_function
import openravepy
import unittest

import os # environ, path
import subprocess
import sys # stderr

import numpy 

from prpy.clone import Clone

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


class Tests(unittest.TestCase):
    """
    Various unit tests for grasping and cloning
    """

    def setUp(self):
        self.env = openravepy.Environment()
        self.env.Load('wamtest1.env.xml')
        self.robot = self.env.GetRobot('BarrettWAM')
        self.manipulator = self.robot.GetManipulator('arm')

        self.robot.Enable(True)

        # Set all 7 DOF of the WAM arm to active
        with self.env:
            self.robot.SetActiveDOFs(self.manipulator.GetArmIndices())
            self.robot.SetActiveManipulator(self.manipulator)
            self.active_dof_indices = self.robot.GetActiveDOFIndices()

        # Get the resolution (in radians) for the 7 joints
        # [0.0043, 0.0087, 0.0087, 0.0174, 0.0193, 0.0282, 0.0282]
        self.dof_resolutions = \
               self.robot.GetDOFResolutions()[self.active_dof_indices]


        #add the box object we will grab
        self.to_grab_body = openravepy.RaveCreateKinBody(self.env, '')
        self.to_grab_body.SetName('box')
        self.to_grab_body.InitFromBoxes(numpy.array([[0., 0., 0., 0.1, 0.1, 0.1]]), False)
        self.env.Add(self.to_grab_body)

        self.to_grab_body.Enable(True)
        T = numpy.eye(4)
        T[2,3] = 20. #move far to not be in collision
        self.to_grab_body.SetTransform(T)


    def tearDown(self):
        self.env.Destroy()

    # clone the environment, and test to make sure everything is the same
    def check_cloned_attributes_collisions(self):
        with Clone(self.robot.GetEnv()) as cloned_env:
            cloned_robot = cloned_env.Cloned(self.robot)
        
            #self collision is identical
            self.assertEqual(self.robot.CheckSelfCollision(), cloned_robot.CheckSelfCollision())
            
            #same number of grasped items
            self.assertEqual(len(self.robot.GetGrabbedInfo()), len(cloned_robot.GetGrabbedInfo()) )

            #grabbed info is set properly
            for grab_info in self.robot.GetGrabbedInfo():
              
              found_equal_info = False
              for cloned_grab_info in cloned_robot.GetGrabbedInfo():
                  this_info_equal = (grab_info._grabbedname == cloned_grab_info._grabbedname)
                  this_info_equal = this_info_equal & (grab_info._robotlinkname == cloned_grab_info._robotlinkname)
                  this_info_equal = this_info_equal & (grab_info._setRobotLinksToIgnore == cloned_grab_info._setRobotLinksToIgnore)
                  this_info_equal = this_info_equal & (numpy.linalg.norm(grab_info._trelative - cloned_grab_info._trelative) < 1e-4) 

                  if this_info_equal:
                      found_equal_info = True
                      break
                  
              #assert that there exists a grabbed info with equal properties
              self.assertTrue(found_equal_info)
              
        
    def test_cloning_start_config(self):
        self.check_cloned_attributes_collisions()

    def test_cloning_grasped_object_nocollision(self):
        self.assertFalse(self.env.CheckCollision(self.robot))

        self.robot.Grab(self.to_grab_body)
        self.check_cloned_attributes_collisions()

    def test_cloning_grasped_object_collision(self):
        self.to_grab_body.SetTransform(self.manipulator.GetEndEffectorTransform())
        self.assertTrue(self.env.CheckCollision(self.robot))

        self.robot.Grab(self.to_grab_body)
        self.check_cloned_attributes_collisions()


    def test_cloning_grasped_object_nocollision_grablinks(self):
        self.assertFalse(self.env.CheckCollision(self.robot))
        link = self.robot.GetLinks()[7]
        self.robot.Grab(self.to_grab_body, grablink=link, linkstoignore=[8, 9, 10])
        self.check_cloned_attributes_collisions()

    def test_cloning_grasped_object_collision_grablinks(self):
        self.to_grab_body.SetTransform(self.manipulator.GetEndEffectorTransform())
        self.assertTrue(self.env.CheckCollision(self.robot))
        link = self.robot.GetLinks()[7]
        self.robot.Grab(self.to_grab_body, grablink=link, linkstoignore=[8, 9, 10])
        self.check_cloned_attributes_collisions()







if __name__ == '__main__':
    unittest.main()
