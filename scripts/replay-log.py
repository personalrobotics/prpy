#!/usr/bin/python

# This script allows the planning requests produced by the Logged
# metaplanner (using PrPy's serialization facilities) to be replayed.
# Downstream users may need to add some additional features
# (e.g. initializing robot-specific helpers such as HerbPy)
# in their log replay facilities so that planners can make use of them.

# Copyright (c) 2016, Carnegie Mellon University
# All rights reserved.
# Authors: Chris Dellin <cdellin@gmail.com>
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

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import argparse
import atexit
import math

import numpy
import openravepy
import yaml

import prpy.planning
import prpy.serialization

parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logfile', required=True)
parser.add_argument('--planner', default='cbirrt', help='cbirrt OMPL_RRTConnect birrt')
parser.add_argument('--viewer', '-v', type=str, default='interactivemarker', help='The viewer to attach (none for no viewer)')
args = parser.parse_args()

# start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

planner = None
if args.planner == 'cbirrt':
   planner = prpy.planning.CBiRRTPlanner()
if args.planner == 'OMPL_RRTConnect':
   planner = prpy.planning.ompl.OMPLPlanner('RRTConnect')
if args.planner == 'birrt':
   planner = prpy.planning.openrave.OpenRAVEPlanner('birrt')

# read log file
yamldict = yaml.safe_load(open(args.logfile))

# deserialize environment
prpy.serialization.deserialize_environment(yamldict['environment'], env=env)

# load planning request
try:
   method = getattr(planner, yamldict['request']['method'])
except AttributeError:
   raise RuntimeError('That planner does not support planning method {}!'.format(yamldict['request']['method']))
method_args = []
for method_arg in yamldict['request']['args']:
   method_args.append(prpy.serialization.deserialize(env, method_arg))
method_kwargs = {}
for key,value in yamldict['request']['kw_args'].items():
   method_kwargs[key] = prpy.serialization.deserialize(env, value)

# attempt to retrieve robot
if 'robot' in method_kwargs:
   robot = method_kwargs['robot']
elif len(method_args) > 0:
   robot = method_args[0]
else:
   raise RuntimeError('could not retrieve robot!')

# load ik solver for robot in case it's needed
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,
   iktype=openravepy.IkParameterizationType.Transform6D)
if not ikmodel.load():
   ikmodel.autogenerate()

# call planning method itself ...
print('calling planning method ...')
traj = method(*method_args, **method_kwargs)

# retime/view resulting trajectory
if args.viewer != 'none':
   env.SetViewer(args.viewer)
openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot)
try:
   while traj is not None:
      raw_input('Press [Enter] to run the trajectory, [Ctrl]+[C] to quit ...')
      with env:
         robot.GetController().SetPath(traj)
except KeyboardInterrupt:
   print()
