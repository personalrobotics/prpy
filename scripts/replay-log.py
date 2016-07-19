#!/usr/bin/python
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
import json

import prpy.planning
import prpy.serialization



parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logfile', required=True)
parser.add_argument('--planner', default='cbirrt', help='cbirrt OMPL_RRTConnect birrt')
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
if args.planner == 'snap':
   planner = prpy.planning.SnapPlanner()
if args.planner == 'vectorfield':
   planner = prpy.planning.VectorFieldPlanner()

# read log file
yamldict = yaml.safe_load(open(args.logfile))

# deserialize environment
prpy.serialization.deserialize_environment(yamldict['environment'], env=env)

#Set collision checker
stubchecker = openravepy.RaveCreateCollisionChecker(env,'stub_checker')
env.SetCollisionChecker(stubchecker)

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

# Log the collision checks

check_info = stubchecker.SendCommand('GetLogInfo')
check_info_dict = json.loads(check_info)

if 'collision_log' in yamldict.keys():
   yamldict['collision_log'][args.planner] = check_info_dict
else:
   yamldict['collision_log'] = {args.planner : check_info_dict}

with open(args.logfile,'w') as f_out:
   yaml.safe_dump(yamldict,f_out)

stubchecker.SendCommand('Reset')