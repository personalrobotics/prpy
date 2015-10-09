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
def render_trajectory(env, manip, traj):
   '''
   Renders a trajectory in the specified OpenRAVE environment
   @param env The openrave environment object
   @param traj The trajectory to render
   '''

   sphere_obj_path = 'objects/misc/smallsphere.kinbody.xml'
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
         manip.parent.SetActiveDOFValues(arm_config)

         # grab the end effector pose
         ee_pose = manip.GetEndEffectorTransform()

         # render a sphere here
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


def add_box(env, name, x_len=0.1, y_len=0.1, z_len=0.1):
    """Add simple box kinbody to OpenRAVE environment"""
    with env:
       box = openravepy.RaveCreateKinBody(env, '')
       success = box.InitFromBoxes(numpy.array([[0,0,0,x_len,y_len,z_len]]))
       box.SetName(name)
       env.Add(box)
       return box


def resize_box(env, body, x=None, y=None, z=None):
    """Resize a simple (single-link) box kinbody"""
    with env:
       env.Remove(body)
       orig_transform = body.GetTransform()
       geom_data = body.GetLinks()[0].GetGeometries()[0].GetInfo()._vGeomData
       box = numpy.array([[0,0,0,geom_data[0], geom_data[1], geom_data[2]]])
       if x or x == 0.0:
          box[0,3] = x
       if y or y == 0.0:
          box[0,4] = y
       if z or z == 0.0:
          box[0,5] = z
       success = body.InitFromBoxes(box)
       body.SetTransform(orig_transform)
       env.Add(body)


def _interactive_resize_box(stdscr, env, body):
   import curses
   RESIZE_SCALE = 0.01

   def print_size(l,w,h):
      stdscr.addstr('\rlength: {}, width: {}, height: {}'.format(l, w, h))

   stdscr.clear()
   stdscr.addstr("'b': increase length, 'b': decrease length\n")
   stdscr.addstr("'h': increase width, 'l': decrease width\n")
   stdscr.addstr("'k': increase height, 'j': decrease height\n")
   stdscr.addstr("'q': quit\n")
   stdscr.addstr("(vi-style commands)\n")
   stdscr.refresh()
   while 1:
      geom_data = body.GetLinks()[0].GetGeometries()[0].GetInfo()._vGeomData
      l = geom_data[0]
      w = geom_data[1]
      h = geom_data[2]

      c = stdscr.getch()
      # length
      if c == ord('b'):
         new_l = l + RESIZE_SCALE
         resize_box(env, body, x=new_l)
         print_size(new_l, w, h)
      elif c == ord('n'):
         new_l = l - RESIZE_SCALE
         if new_l > 0.0:
            resize_box(env, body, x=new_l)
            print_size(new_l, w, h)
      # width
      elif c == ord('h'):
         new_w = w + RESIZE_SCALE
         resize_box(env, body, y=new_w)
         print_size(l, new_w, h)
      elif c == ord('l'):
         new_w = w - RESIZE_SCALE
         if new_w > 0.0:
            resize_box(env, body, y=new_w)
            print_size(l, new_w, h)
      # height
      elif c == ord('k'):
         new_h = h + RESIZE_SCALE
         resize_box(env, body, z=new_h)
         print_size(l, w, new_h)
      elif c == ord('j'):
         new_h = h - RESIZE_SCALE
         if new_h > 0.0:
            resize_box(env, body, z=new_h)
            print_size(l, w, new_h)
      elif c == ord('q'):
         break


def add_cylinder(env, name, radius=0.03, height=0.12):
    """Add simple cylinder kinbody to OpenRAVE environment"""
    with env:
       geom = openravepy.KinBody.Link.GeometryInfo()
       geom._type = openravepy.GeometryType.Cylinder
       geom._vGeomData = [radius, height]
       geom._vDiffuseColor = [1,0,0]
       link = openravepy.KinBody.LinkInfo()
       link._vgeometryinfos = [geom]
       link._name = 'link0'
       cyl = openravepy.RaveCreateKinBody(env, '')
       success = cyl.Init([link], [])
       cyl.SetName(name)
       env.Add(cyl)
       return cyl


def resize_cylinder(env, body, radius=None, height=None):
    """Resize simple (single-link) cylinder kinbody"""
    with env:
       env.Remove(body)
       geom = body.GetLinks()[0].GetGeometries()[0].GetInfo()
       if radius or radius == 0.0:
          geom._vGeomData[0] = radius
       if height or height == 0.0:
          geom._vGeomData[1] = height
       link = body.GetLinks()[0].GetInfo()
       link._vgeometryinfos = [geom]
       success = body.Init([link],[])
       env.Add(body)


def _interactive_resize_cylinder(stdscr, env, body):
   import curses
   RESIZE_SCALE = 0.01

   def print_size(h,r):
         stdscr.addstr('\rheight: {}, radius: {}'.format(h, r))

   stdscr.clear()
   stdscr.addstr("'i': increase height, 'j': decrease height\n")
   stdscr.addstr("'h': increase radius, 'l': decrease radius\n")
   stdscr.addstr("'q': quit\n")
   stdscr.addstr("(vi-style commands)\n")
   stdscr.refresh()
   while 1:
      geom_data = body.GetLinks()[0].GetGeometries()[0].GetInfo()._vGeomData
      r = geom_data[0]
      h = geom_data[1]
      c = stdscr.getch()
      if c == ord('k'):
         new_h = h + RESIZE_SCALE
         resize_cylinder(env, body, height=new_h)
         print_size(new_h, r)
      if c == ord('j'):
         new_h = h - RESIZE_SCALE
         if new_h > 0.0:
            resize_cylinder(env, body, height=new_h)
            print_size(new_h, r)
      if c == ord('h'):
         new_r = r + RESIZE_SCALE
         resize_cylinder(env, body, radius=new_r)
         print_size(h, new_r)
      if c == ord('l'):
         new_r = r - RESIZE_SCALE
         if new_r > 0.0:
            resize_cylinder(env, body, radius=new_r)
            print_size(h, new_r)
      elif c == ord('q'):
         break


def add_sphere(env, name, radius=0.03):
    """Add simple sphere kinbody to OpenRAVE environment"""
    with env:
       geom = openravepy.KinBody.Link.GeometryInfo()
       geom._type = openravepy.GeometryType.Sphere
       geom._vGeomData = [radius]
       geom._vDiffuseColor = [1,0,0]
       link = openravepy.KinBody.LinkInfo()
       link._vgeometryinfos = [geom]
       link._name = 'link0'
       sphere = openravepy.RaveCreateKinBody(env, '')
       success = sphere.Init([link], [])
       sphere.SetName(name)
       env.Add(sphere)
       return sphere


def resize_sphere(env, body, radius=None):
    """Resize simple (single-link) sphere kinbody"""
    with env:
       env.Remove(body)
       geom = body.GetLinks()[0].GetGeometries()[0].GetInfo()
       if radius or radius == 0.0:
          geom._vGeomData[0] = radius
          link = body.GetLinks()[0].GetInfo()
          link._vgeometryinfos = [geom]
          success = body.Init([link],[])
          env.Add(body)


def _interactive_resize_sphere(stdscr, env, body):
   import curses
   RESIZE_SCALE = 0.01
   stdscr.clear()
   stdscr.addstr("'k': increase radius, 'j': decrease radius, 'q': quit\n")
   stdscr.addstr("(vi-style commands)\n")
   stdscr.refresh()
   while 1:
      r = body.GetLinks()[0].GetGeometries()[0].GetInfo()._vGeomData[0]
      c = stdscr.getch()
      if c == ord('k'):
         new_r = r + RESIZE_SCALE
         resize_sphere(env, body, radius=new_r)
         stdscr.addstr('\rradius: {}'.format(new_r))
      if c == ord('j'):
         new_r = r - RESIZE_SCALE
         if new_r > 0.0:
            resize_sphere(env, body, radius=new_r)
            stdscr.addstr('\rradius: {}'.format(new_r))
      elif c == ord('q'):
         break


def resize_primitive_interactive(env, body):
   """Live, interactive primitive object resizing

   NOTE: cannot be used in a script because it creates a curses UI
   that requires user input
   """
   import curses.wrapper
   body_type = body.GetLinks()[0].GetGeometries()[0].GetType()
   if body_type is openravepy.GeometryType.Sphere:
      curses.wrapper(_interactive_resize_sphere, env, body)
   if body_type is openravepy.GeometryType.Cylinder:
      curses.wrapper(_interactive_resize_cylinder, env, body)
   if body_type is openravepy.GeometryType.Box:
      curses.wrapper(_interactive_resize_box, env, body)
   else:
      print "Unknown body type. Unable to resize"
