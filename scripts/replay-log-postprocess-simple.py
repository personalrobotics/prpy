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
from or_trajopt import TrajoptPlanner
import prpy.planning
import prpy.serialization
from prpy.planning import (
   FirstSupported,
   Sequence,
   PlanningError,
   UnsupportedPlanningError,
   TSRPlanner,
   NamedPlanner,
   CBiRRTPlanner,
   CHOMPPlanner,
   GreedyIKPlanner,
   IKPlanner,
   SBPLPlanner,
   SnapPlanner,
   VectorFieldPlanner
)

import os.path
import re


def get_filename(logfile_name, planner, method, output_dir):

   # dump to file    
   save_path = os.path.abspath(output_dir)
   method_name = yamldict['request']['method']
   logfile_name = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',args.logfile))[0]
   filename = "postprocess_" + logfile_name + "_" + method + "_" + planner + ".yaml"
   completePath = os.path.join(save_path, filename)
   return completePath


parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logfile', required=True)
parser.add_argument('--viewer', '-v', type=str, default='interactivemarker', help='The viewer to attach (none for no viewer)')
parser.add_argument('--outputdir', default="postprocess", type=str, help='Output directory.')
parser.add_argument('--overwrite', action='store_true')

args = parser.parse_args()

# start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

# List of planners supported 
planner_list = ['cbirrt', 'OMPL_RRTConnect', 'snap', 'chomp', 'vectorfield', 'greedy-ik', 'trajopt']

# read log file
yamldict = yaml.safe_load(open(args.logfile))

if not yamldict['result']['ok']:
   raise ValueError('Cannot postprocess unsuccessful trajecotry')

planner_name = yamldict['result']['planner_used']
planner_name = filter(lambda x: x.lower() in planner_name.lower(), planner_list)[0]

# if planner_name == 'snap' or yamldict['result']['planner_used'] == 'snap':
#    raise ValueError("Postprocessing has no effect on {}".format(planner_name))

actual_planner = None
if planner_name == 'cbirrt':
   actual_planner = CBiRRTPlanner()
elif planner_name == 'OMPL_RRTConnect':
   actual_planner = prpy.planning.ompl.OMPLPlanner('RRTConnect')
elif planner_name == "snap":
   actual_planner = SnapPlanner()
elif planner_name == 'chomp':
   actual_planner = CHOMPPlanner()
elif planner_name == 'vectorfield':
   actual_planner = VectorFieldPlanner()
elif planner_name == 'greedy-ik':
   actual_planner = GreedyIKPlanner()
elif planner_name == 'trajopt':
   actual_planner = TrajoptPlanner()
else:
   raise ValueError("Unrecognized planner")


# Wrapps planner with for PlanToTSR and PlanToNamedConfiguration
planner = FirstSupported(Sequence(actual_planner,
                                  TSRPlanner(delegate_planner=actual_planner)),
                                  # Special purpose meta-planner.
                         NamedPlanner(delegate_planner=actual_planner)
                        )

# Setup environment
robot = None

# Use herbpy if herb. 
for body in yamldict['environment']['bodies']:
   if body['name'] == 'herb':
      import herbpy 
      env, robot = herbpy.initialize(sim=True)
      print ("Retreieving herb")

# deserialize environment
if robot: 
   prpy.serialization.deserialize_environment(yamldict['environment'], env=env, reuse_bodies=[robot])
else:
   prpy.serialization.deserialize_environment(yamldict['environment'], env=env)
   robots = prpy.serialization.deserialize_robots(yamldict['environment'], env)
   robot = robots[0]

# load planning request
try:
   method = getattr(planner, yamldict['request']['method'])
except (AttributeError, UnsupportedPlanningError) as e:
   print ('{} does not support planning method {}!'.format(planner, yamldict['request']['method']))
   raise e

method_args = []
for method_arg in yamldict['request']['args']:
   method_args.append(prpy.serialization.deserialize(env, method_arg))
method_kwargs = {}
for key,value in yamldict['request']['kw_args'].items():
   method_kwargs[key] = prpy.serialization.deserialize(env, value)

# remove robot and use properly deserialized robot 
if robot in method_args:
   method_args.remove(robot)
if 'robot' in method_kwargs:
   method_kwargs.pop('robot')


# 'ranker' is not properly serialized, so this key causes error.
if 'ranker' in method_kwargs:
   method_kwargs.pop('ranker')

# load ik solver for robot in case it's needed
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,
   iktype=openravepy.IkParameterizationType.Transform6D)
if not ikmodel.load():
   ikmodel.autogenerate()


# call planning method itself ...
print('calling planning method {} ...'.format(actual_planner))
from prpy.util import Timer, SetTrajectoryTags
from prpy.planning.base import Tags

error_msg = None
traj = None


# get filename to save the data
logfile_name = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',args.logfile))[0]
method_name = yamldict['request']['method']
filename = get_filename(args.logfile, str(actual_planner), method_name, args.outputdir)

# Quit if file already exists 
if os.path.isfile(filename) and not args.overwrite:
   import time
   import math
   stamp = time.time()
   struct = time.localtime(stamp)

   filename = filename.split('.yaml')[0] + '.{}.{:06d}.yaml'.format(
            time.strftime('%Y%m%d-%H%M%S', struct),
            int(1.0e6*(stamp-math.floor(stamp))))

elif os.path.isfile(filename):
   print (filename, " already exists. Use --overwrite to re-prostprocess.")
   import sys
   sys.exit(0)

print ("Log will be saved at "+filename)


# Get vanila trajectory
traj = None
try:
   with Timer() as timer:
      traj = method(robot, *method_args, **method_kwargs)
   planning_time = timer.get_duration()
   SetTrajectoryTags(traj, {Tags.PLAN_TIME: timer.get_duration()}, append=True)
except PlanningError as e: 
   raise
except AttributeError:
   print ('{} does not support planning method {}!'.format(actual_planner, yamldict['request']['method']))

if not traj:
   print ("Failed to generate trajectory.")
   raise RuntimeError("Failed to generate trajectory.")

with Timer() as timer:
    traj = robot.PostProcessPath(traj, timelimit=5.)
SetTrajectoryTags(traj, {Tags.POSTPROCESS_TIME: timer.get_duration()}, append=True)
postprocessing_time = timer.get_duration()

# Dump result to file
reqdict = {}
resdict = {}

reqdict['method'] = yamldict['request']['method']
reqdict['planner_name'] = str(actual_planner)
if 'action' in yamldict['request'].keys():
   reqdict['action'] = yamldict['request']['action']

resdict['ok'] = True
resdict['planning_time'] = planning_time
resdict['postprocessing_time'] = postprocessing_time 

yamldict_res = {}
yamldict_res['environment'] = yamldict['environment']
yamldict_res['request'] = reqdict
yamldict_res['result'] = resdict

with open(filename,'w') as fp:
   yaml.safe_dump(yamldict_res, fp)
   print ('\n{} written\n'.format(filename))


# retime/view resulting trajectory
# if args.viewer != 'none':
#    env.SetViewer(args.viewer)
# if traj is not None: 
#    openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot)

# try:
#    response = raw_input('Press [Enter] to continue to the next planner, [Ctrl]+[C] to quit ...')
#    while traj is not None:
#       with env:
#          robot.GetController().SetPath(traj)
# except KeyboardInterrupt:
#    print ("Exiting ...")
      

