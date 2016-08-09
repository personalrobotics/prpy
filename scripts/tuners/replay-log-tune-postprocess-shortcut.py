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
import prpy.planning
import prpy.serialization
from prpy.planning import (
    FirstSupported,
    Sequence,
    PlanningError,
    UnsupportedPlanningError,

)
from prpy.clone import Clone


import os.path
import re
from os import listdir
from os.path import isfile, join
import time 

def get_filename(logfile_name):

    method_name = 'PlanToTSR'
    planner = 'RRTConnect'
    logfile_name = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',logfile_name))[0]
    print (logfile_name)
    filename = "postprocess_" + logfile_name + ".yaml"
    return filename


parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logfile', required=True)

args = parser.parse_args()


from prpy.collision import (
    BakedRobotCollisionChecker,
)
from openravepy import RaveCreateCollisionChecker

robot_collision_checker = BakedRobotCollisionChecker
planner = prpy.planning.ompl.OMPLPlanner('RRTConnect', robot_collision_checker=robot_collision_checker)

logfile = args.logfile
yamldict = yaml.safe_load(open(logfile))
method_name = yamldict['request']['method']
if method_name != 'PlanToTSR':
    import sys
    print ("Not PlanToTSR")
    sys.exit(0)


# deserialize environment
import herbpy
env, robot = herbpy.initialize(sim=True)
cc = openravepy.RaveCreateCollisionChecker(env, 'fcl')
assert cc is not None
env.SetCollisionChecker(openravepy.RaveCreateCollisionChecker(env, 'fcl'))
env.GetCollisionChecker().SetDescription('fcl')

# load planning request
try:
    method = getattr(planner, yamldict['request']['method'])
except (AttributeError, UnsupportedPlanningError) as e:
    print ('{} does not support planning method {}!'.format(planner, yamldict['request']['method']))
    import sys
    sys.exit(0)



logfile = args.logfile

print ("Reading " + logfile)

# read log file
yamldict = yaml.safe_load(open(logfile))

# deserialize environment
prpy.serialization.deserialize_environment(yamldict['environment'], env=env, reuse_bodies=[robot])

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
if 'ranker' in method_kwargs:
    method_kwargs.pop('ranker')

# load ik solver for robot in case it's needed
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,
    iktype=openravepy.IkParameterizationType.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()

# call planning method itself ...
print('calling planning method {} ...'.format(planner))
from prpy.util import Timer, SetTrajectoryTags
from prpy.planning.base import Tags

error_msg = None

filename = get_filename(logfile)

# Get vanila trajectory
traj = None
for i in xrange(3):
    try:
        planning_time = None 
        with Clone(env, lock=True) as planning_env:
            planning_robot = planning_env.GetRobots()[0]
            start_time = time.time()
            traj = method(planning_robot, *method_args, **method_kwargs)  
            planning_time = time.time() - start_time
        break
    except PlanningError as e: 
        continue
    except AttributeError as e:
        print ('{} does not support planning method {}!'.format(planner, yamldict['request']['method']))
        break

if not traj:
    with open(filename) as f: 
        f.write("ABORTED: Failed to generate trajectory")
        f.write(str(e))
    import sys 
    sys.exit(0)

# Get defaultTags
from prpy.util import GetTrajectoryTags
tags = GetTrajectoryTags(traj)

constrained = tags.get(Tags.CONSTRAINED, False)
if constrained:
    print ("Skipping constrained trajectory")
    with open(filename) as f: 
        f.write("ABORTED: Skipping constrained trajectory")
    import sys 
    sys.exit(0)

print ("Post processing...")

from or_parabolicsmoother.prpy.retimer import HauserParabolicSmoother

# Postprocessing options
timelimits = [0.25, 0.5, 0.75, 1, 2]

smoothers = []
# no smoothing
smoother = HauserParabolicSmoother(do_blend=True,
                                   do_shortcut=False,
                                   blend_radius=0.4,
                                   blend_iterations=1)
properties = {}
properties['request'] = {'do_blend':True, 'do_shortcut':False, 'blend_radius':0.4, 'blend_iterations':1}
properties['result'] = dict()
smoothers.append((smoother, properties))


# # with shortcut only 
for t in timelimits:
    smoother = HauserParabolicSmoother(do_blend=True,
                                       do_shortcut=True,
                                       timelimit=t,
                                       blend_radius=0.4,
                                       blend_iterations=1)
    properties = {}
    properties['request'] = {'do_shortcut': True, 'timelimit':t, 'blend_radius':0.4, 'blend_iterations':1, 'do_blend':True }
    properties['result'] = dict() 
    smoothers.append((smoother, properties))


# Run postproocessing per smoothing option
for (smoother, properties) in smoothers:
    
    from prpy.util import CopyTrajectory
    path_copy = CopyTrajectory(traj, env=env)

    import prpy.base
    postprocess_env = prpy.base.Robot._postprocess_envs[
                openravepy.RaveGetEnvironmentId(robot.GetEnv())]

    from prpy.clone import Clone, Cloned
    with Clone(robot.GetEnv(), clone_env=postprocess_env, lock=True) as cloned_env:
        cloned_robot = cloned_env.Cloned(robot)
        with Timer() as timer:
            postprocessed_traj = smoother.RetimeTrajectory(cloned_robot, path_copy)

        postprocessing_time = timer.get_duration()
        
    # Save the result
    properties['result']['postprocessing_time'] = (postprocessing_time)
    properties['result']['postprocessed_traj_execution_time'] = (postprocessed_traj.GetDuration())
        

# Dump result to file
reqdict = {}
resdict = {}

reqdict['method'] = yamldict['request']['method']
reqdict['planner_name'] = str(planner)
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

