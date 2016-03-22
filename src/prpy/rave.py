#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This code originated in prrave and was written by Chris Dellin.
## @package libherb.orv OpenRAVE environment manipulation. This needs to be merged into the rest of libherb.

import logging
import numpy
import os
import openravepy

# Load a module into an environment
# modcmd is passed to the module's main method
# returns True on success, False on failure
def load_module(env, modname, modcmd=''):
   m = openravepy.RaveCreateModule(env, modname)
   if not m:
      return None
   env.AddModule(m, modcmd)
   return m

# Get a module loaded in an environment by name
def get_module(env, mod_name):
   for m in env.GetModules():
      if m.GetXMLId() == mod_name:
         return m
   return None

# send a module command which saves a trajectory to disk,
# and then deserialize it
def send_for_traj(mod=None, modfunc=None, filename_arg='filename', **kwargs):
   if modfunc is None:
      raise ValueError('modfunc must be passed!')
   if filename_arg not in kwargs:
      kwargs[filename_arg] = os.tmpnam()
   if mod is None:
      modfunc(**kwargs) # this could raise an exception
   else:
      modfunc(mod, **kwargs) # this could raise an exception
   traj_string = open(kwargs[filename_arg],'r').read()
   if mod is None:
      if hasattr(modfunc,'__self__'):
         e = modfunc.__self__.GetEnv()
      else:
         e = modfunc.im_self.GetEnv() # for pre-Python2.6
   else:
      e = mod.GetEnv()
   return openravepy.RaveCreateTrajectory(e,'').deserialize(traj_string)

# Set the transparency of a robot
def set_transparency(robot, transparency):
   for link in robot.GetLinks():
      for geom in link.GetGeometries():
         geom.SetTransparency(transparency)

# add an object to the environment
def add_object(env, object_name, object_xml_path, object_pose = None):
   '''
   Add an object (kinbody) to the OpenRAVE environment.
   @param env The openrave environment object
   @param object_name The name to be given to the object when it is added to the environment
   @param object_xml_path The xml path of the object to add
   @param object_pose - The initial transform wrt world frame of the object when added to the environment. If None, the object is placed at the origin.
   @exception Exception Could not load the object xml file.
   '''

   if object_pose is None:
      object_pose = numpy.eye(4)
      
   with env:
      body = env.ReadKinBodyXMLFile(object_xml_path)
      if body is None:
         raise Exception('Could not load object from \'%s\'.' % object_xml_path)
      else:
         body.SetName(object_name)
         body.SetTransform(object_pose)
         env.AddKinBody(body)
         return body

   return None



# render a trajectory in the environment
def render_trajectory(env, manip, traj,obj_path):
   '''
   Renders a trajectory in the specified OpenRAVE environment
   @param env The openrave environment object
   @param traj The trajectory to render
   '''

   sphere_obj_path = obj_path+'/objects/smallsphere.kinbody.xml'
   numdofs = len(manip.GetArmIndices())

   with env:
      numwpts = traj.GetNumWaypoints()

      # iterate the waypoints in the trajectory
      for wid in range(0,numwpts):
         wpt = traj.GetWaypoint(wid)

         # grab the end effector pose for the waypoint
         arm_config = wpt[:numdofs]
         
         # put the arm in that pose
         manip.SetActive()
         manip.GetRobot().SetActiveDOFValues(arm_config)

         # grab the end effector pose
         ee_pose = manip.GetEndEffectorTransform()

         add_object(env, 'traj_sphere'+str(wid), sphere_obj_path, ee_pose)

# Clear out any rendered trajectories
def clear_rendered_trajectories(env):
   '''
   Clears all trajectories that have been rendered via the render_trajectory method
   @param env The OpenRAVE environment object
   '''

   for b in env.GetBodies():
      if b.GetName().startswith('traj_sphere'):
         env.Remove(b)

def load_trajectory(env, path, xmlid=''):
    traj = openravepy.RaveCreateTrajectory(env, xmlid)

    with open(path, 'rb') as traj_file:
        traj_xml = traj_file.read()
        traj.deserialize(traj_xml)

    return traj

def save_trajectory(traj, path):
    traj_xml = traj.serialize(0)

    with open(path, 'wb') as traj_file:
        traj_file.write(traj_xml)

def fix_trajectory(traj):
    """Remove duplicate waypoints that are introduced during smoothing.
    """
    cspec = openravepy.ConfigurationSpecification()
    cspec.AddDeltaTimeGroup()

    iwaypoint = 1
    num_removed = 0
    while iwaypoint < traj.GetNumWaypoints():
        waypoint = traj.GetWaypoint(iwaypoint, cspec)
        delta_time = cspec.ExtractDeltaTime(waypoint)

        if delta_time == 0.0:
            traj.Remove(iwaypoint, iwaypoint + 1)
            num_removed += 1
        else:
            iwaypoint += 1

    return num_removed

# This class allows for disabling kinbodies
class AllDisabled:
   def __init__(self, env, bodies, padding_only=False):
       '''
       @param env is the *Openrave* environment.
       @param bodies is a list of openrave kinbodies.
       '''
       self.bodies = bodies
       self.env = env
       self.padding_only = padding_only

   def __enter__(self):
       with self.env:
           for body in self.bodies:
              if self.padding_only:
                 for link in body.GetLinks():
                    if link.GetName().startswith('padding_'):
                       logging.debug('Disabling link: %s',link.GetName())
                       link.Enable(False)
              else:
                 logging.debug('Disabling body: %s',body.GetName())
                 body.Enable(False)

   def __exit__(self, exc_type, exc_val, exc_tb):
       with self.env:
           for body in self.bodies:

              if self.padding_only:
                 for link in body.GetLinks():
                    if link.GetName().startswith('padding_'):
                       logging.debug('Enabling link: %s',link.GetName())
                       link.Enable(True)
              else:
                 logging.debug('Enabling body: %s',body.GetName())
                 body.Enable(True)

       return False

class Disabled(AllDisabled):
   def __init__(self, body, padding_only=False):      
      AllDisabled.__init__(self, 
                           env = body.GetEnv(), 
                           bodies = [body], 
                           padding_only=padding_only)

def disable_padding(body, enable=False):
    for link in body.GetLinks():
        if link.GetName().startswith('padding_'):
            link.Enable(enable)
