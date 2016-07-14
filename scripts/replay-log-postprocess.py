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
parser.add_argument('--planner', default=None, help=('cbirrt OMPL_RRTConnect snap chomp vectorfield' 
                                                          'greedy-ik trajopt'))
parser.add_argument('--viewer', '-v', type=str, default='interactivemarker', help='The viewer to attach (none for no viewer)')
parser.add_argument('--outputdir', default="postprocess", type=str, help='Output directory.')
parser.add_argument('--overwrite', action='store_true')
parser.add_argument('--max-trials', default=1, type=int, 
   help='Number of max trials until getting raw trajectory. Set this only for randomized planners')
parser.add_argument('--num-postprocess-trials', default=1, type=int, help='Number of postprocess runs per setting')

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


# setup planner
if args.planner:
   planner_name = args.planner
else:
   planner_name = yamldict['result']['planner_used']
   planner_name = filter(lambda x: x.lower() in planner_name.lower(), planner_list)[0]

if planner_name == 'snap' or yamldict['result']['planner_used'] == 'snap':
   raise ValueError("Postprocessing has no effect on {}".format(planner_name))

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

# # attempt to retrieve robot
# if 'robot' in method_kwargs:
#    robot = method_kwargs['robot']
# elif len(method_args) > 0:
#    robot = method_args[0]
# else:
#    raise RuntimeError('could not retrieve robot!')

# remove robot and use properly deserialized robot 
if robot in method_args:
   method_args.remove(robot)
if 'robot' in method_kwargs:
   method_kwargs.pop('robot')

# method_args.append(robot)

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
# Allow max_trials > 1 only for randomized planners
if "PlanToTSR".lower() in method_name.lower() or 'rrt' in planner_name.lower():
   max_trials = args.max_trials
else:
   max_trials = 1


# Get vanila trajectory
traj = None
for i in range(max_trials):
   try:
      with Timer() as timer:
         traj = method(robot, *method_args, **method_kwargs)
      planning_time = timer.get_duration()
      SetTrajectoryTags(traj, {Tags.PLAN_TIME: timer.get_duration()}, append=True)
      break
   except PlanningError as e: 
      continue
   except AttributeError:
      print ('{} does not support planning method {}!'.format(actual_planner, yamldict['request']['method']))
      break
   # except:
   #    break

if not traj:
   print ("Failed to generate trajectory.")
   raise RuntimeError("Failed to generate trajectory.")

# Get defaultTags
from prpy.util import GetTrajectoryTags
tags = GetTrajectoryTags(traj)

constrained = tags.get(Tags.CONSTRAINED, False)
if constrained:
   # with open(filename,'w') as fp:
   #    yaml.safe_dump('Constrained, not doing any shortcut', fp)
   #    print ('\n{} written\n'.format(filename))

   raise RuntimeError("Constrained trajectory")

print ("Post processing...")

from or_parabolicsmoother.prpy.retimer import HauserParabolicSmoother

# Postprocessing options
# blend_options = {}
# blend_options['iterations'] = [1, 2, 3, 4, 5]
# blend_options['radius'] = [0.5]
timelimits = numpy.linspace(0.2,5.0,25).tolist()
# timelimits = [2.0]

print ("Timelimits: ", timelimits)

smoothers = []
# post-process with different blending options
# for t in timelimits:
#    for do_shortcut in [False]:
#       for num_iter in blend_options['iterations']:
#          for radius in blend_options['radius']:
#             smoother = HauserParabolicSmoother(do_blend=True,
#                                                blend_iterations=num_iter,
#                                                blend_radius=radius,
#                                                do_shortcut=do_shortcut,
#                                                timelimit=t)
#             properties = {}
#             properties['request'] = {}
#             properties['request']['do_shortcut'] = do_shortcut
#             properties['request']['do_blend'] = True 
#             properties['request']['blend_iterations']= num_iter
#             properties['request']['blend_radius'] = radius
#             properties['request']['timelimit'] = t
#             properties['result'] = {'postprocessing_time':[],
#                                     'postprocessed_traj_execution_time':[],
#                                     'postprocessed_traj':[]}
#             smoothers.append((smoother, properties))

# no smoothing
smoother = HauserParabolicSmoother(do_blend=False,
                                   do_shortcut=False,
                                   timelimit=0)
properties = {}
properties['request'] = {}
properties['result'] = {'postprocessing_time':[],
                        'postprocessed_traj_execution_time':[],
                        'postprocessed_traj':[]}
properties['request']['timelimit'] = 0
smoothers.append((smoother, properties))


# # with shortcut only 
for t in timelimits:
   smoother = HauserParabolicSmoother(do_blend=False,
                                      do_shortcut=True,
                                      timelimit=t)
   properties = {}
   properties['request'] = {}
   properties['result'] = {'postprocessing_time':[],
                           'postprocessed_traj_execution_time':[],
                           'postprocessed_traj':[]}
   properties['request']['do_shortcut'] = True
   properties['request']['timelimit'] = t
   smoothers.append((smoother, properties))


if args.viewer != 'none':
   env.SetViewer(args.viewer)
   
# Run postproocessing per smoothing option
for (smoother, properties) in smoothers:

   for trial in range(args.num_postprocess_trials):

      # Post-process
      from prpy.util import CopyTrajectory
      path_copy = CopyTrajectory(traj, env=env)

      import prpy.base
      postprocess_env = prpy.base.Robot._postprocess_envs[
               openravepy.RaveGetEnvironmentId(robot.GetEnv())]

      from prpy.clone import Clone, Cloned

      with Clone(robot.GetEnv(),
                clone_env=postprocess_env) as cloned_env:
         cloned_robot = cloned_env.Cloned(robot)
         with Timer() as timer:
            postprocessed_traj = smoother.RetimeTrajectory(cloned_robot, path_copy)

         postprocessing_time = timer.get_duration()
         SetTrajectoryTags(postprocessed_traj, {Tags.POSTPROCESS_TIME: postprocessing_time}, append=True)
         SetTrajectoryTags(postprocessed_traj, {Tags.EXECUTION_TIME: postprocessed_traj.GetDuration()}, append=True)

      # Save the result
      properties['result']['postprocessing_time'].append(postprocessing_time)
      properties['result']['postprocessed_traj_execution_time'].append(postprocessed_traj.GetDuration())
      # properties['result']['postprocessed_traj'].append(postprocessed_traj.serialize())

   # with prpy.viz.RenderTrajectory(robot, postprocessed_traj, color=[0.4,0,0,1], linewidth=20):
   #    raw_input("Enter")


# Dump result to file
reqdict = {}
resdict = {}

reqdict['method'] = yamldict['request']['method']
reqdict['planner_name'] = str(actual_planner)
if 'action' in yamldict['request'].keys():
   reqdict['action'] = yamldict['request']['action']

resdict['ok'] = True
resdict['planning_time'] = planning_time
resdict['postprocessing_results'] = map(lambda (smoother, result): result, smoothers)

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
      

