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


def get_filename(logfile_name, planner, method, trial, seed=None):

   # dump to file    
   save_path = os.path.abspath('replay')
   method_name = yamldict['request']['method']
   logfile_name = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',args.logfile))[0]
   filename = "replay_" + logfile_name + "_" + method + "_" + planner
   if seed:
      filename = filename + "_seed_" + str(seed) 
   if trial > 1:
      filename = filename + "_trial_" + str(j+1) 
   
   filename = filename + ".yaml"
   completePath = os.path.join(save_path, filename)
   return completePath



parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logfile', required=True)
parser.add_argument('--planner', default='cbirrt', help=('cbirrt OMPL_RRTConnect birrt snap chomp vectorfield' 
                                                        'greedy-ik trajopt try-all'))
parser.add_argument('--viewer', '-v', type=str, default='interactivemarker', help='The viewer to attach (none for no viewer)')
parser.add_argument('--log', default='true', help='Log this replay.')
parser.add_argument('--seed', type=int, default=None, help='Seed for randomized planners.')
parser.add_argument('--num-trials', type=int, default=1, help='Number of trials per planner. Use only for cbirrt')
args = parser.parse_args()

# start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

planner_list = args.planner.split(' ')

planners = []
for pl in planner_list: 
   if pl == 'cbirrt':
      planners.append(CBiRRTPlanner())
   elif pl == 'OMPL_RRTConnect':
      planners.append(prpy.planning.ompl.OMPLPlanner('RRTConnect'))
   elif pl == 'birrt':
      planners.append(prpy.planning.openrave.OpenRAVEPlanner('birrt'))
   elif pl == "snap":
      planners.append(SnapPlanner())
   elif pl == 'chomp':
      planners.append(CHOMPPlanner())
   elif pl == 'vectorfield':
      planners.append(VectorFieldPlanner())
   elif pl == 'greedy-ik':
      planners.append(GreedyIKPlanner())
   elif pl == 'trajopt':
      planners.append(TrajoptPlanner())
   elif pl == 'try-all':
      planners = [
              CBiRRTPlanner(),
              prpy.planning.ompl.OMPLPlanner('RRTConnect'),
              SnapPlanner(),
              CHOMPPlanner(),
              VectorFieldPlanner(),
              GreedyIKPlanner(),
              TrajoptPlanner()
              ]
      break
   else:
      raise ValueError("Unrecognized planner")

logger = True if args.log == 'true' else False 

# read log file
yamldict = yaml.safe_load(open(args.logfile))

# deserialize environment
prpy.serialization.deserialize_environment(yamldict['environment'], env=env)
robots = prpy.serialization.deserialize_robots(yamldict['environment'], env)

for actual_planner in planners:

   planner = FirstSupported(
      Sequence(actual_planner,
               TSRPlanner(delegate_planner=actual_planner)),
      # Special purpose meta-planner.
      NamedPlanner(delegate_planner=actual_planner)
      )

   # load planning request
   try:
      method = getattr(planner, yamldict['request']['method'])
   except (AttributeError, UnsupportedPlanningError) as e:
      print ('{} does not support planning method {}!'.format(planner, yamldict['request']['method']))
      continue


   method_args = []
   for method_arg in yamldict['request']['args']:
      method_args.append(prpy.serialization.deserialize(env, method_arg))
   method_kwargs = {}
   for key,value in yamldict['request']['kw_args'].items():
      method_kwargs[key] = prpy.serialization.deserialize(env, value)

   if args.seed:
      method_kwargs['seed'] = args.seed

   # attempt to retrieve robot
   if 'robot' in method_kwargs:
      robot = method_kwargs['robot']
   elif len(method_args) > 0:
      robot = method_args[0]
   else:
      raise RuntimeError('could not retrieve robot!')

   robot = robots[0]
   # remove robot and use properly deserialized robot 
   if robot in method_args:
      method_args.remove(robot)
   if 'robot' in method_kwargs:
      method_kwargs.pop('robot')
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
   no_attribute = False
   logfile_name = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',args.logfile))[0]
   method_name = yamldict['request']['method']

   for j in range(args.num_trials):
      # filename to save the data
      filename = get_filename(logfile_name, str(actual_planner), method_name, j, args.seed)

      import os.path 
      if os.path.isfile(filename):
         print ("\n\nSkipping ", filename,'\n\n')
         continue

      try:
         with Timer() as timer:
            traj = method(robot, *method_args, **method_kwargs)
         planning_time = timer.get_duration()
         SetTrajectoryTags(traj, {Tags.PLAN_TIME: timer.get_duration()}, append=True)
         print ("Timer: ", timer.get_duration())
      except PlanningError as e: 
         error_msg = str(e)
         print (error_msg)
      except AttributeError:
         no_attribute = True
         print ('{} does not support planning method {}!'.format(actual_planner, yamldict['request']['method']))
         break;
      except:
         import sys
         error_msg = sys.exc_info()[0]
         print (error_msg)

      if logger and not no_attribute: 
         reqdict = {}
         resdict = {}
         reqdict['method'] = yamldict['request']['method']
         if args.seed:
            reqdict['seed'] = method_kwargs['seed']
         reqdict['planner_name'] = str(planner)
         resdict['ok'] = True if traj else False
         if traj: 
            resdict['planning_time'] = planning_time
            resdict['traj'] = traj.serialize()
         if error_msg:
            resdict['error'] = str(error_msg)
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
      

