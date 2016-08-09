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
#import prpy_lemur.lemur
#import prpy_lemur.roadmaps

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
    SnapPlanner,
    VectorFieldPlanner
)

import time 


import os.path
from os.path import join, isfile
from os import listdir
import re

def get_filename(logfile, planner, method):

    logfile = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',logfile))[0]
    savedir = os.path.join(planner)
    if not os.path.exists(savedir):
        os.makedirs(savedir)

    filename = '_'.join(str(x) for x in ['replay', logfile, method, planner])
    filename += '.yaml'

    completePath = os.path.join(savedir, filename)
    return completePath


parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logfile', required=True)
parser.add_argument('--planner', required=True, type=str, choices=['snap', 'trajopt'])

args = parser.parse_args()



from prpy.collision import (
    BakedRobotCollisionChecker,
)
from openravepy import RaveCreateCollisionChecker

robot_collision_checker = BakedRobotCollisionChecker

if args.planner == "snap":
    actual_planner = SnapPlanner(robot_collision_checker=robot_collision_checker)
elif args.planner == 'trajopt':
    actual_planner = TrajoptPlanner(robot_collision_checker=robot_collision_checker)
else:
    raise ValueError("Unrecognized planner")


logfile = args.logfile
print ("Reading ", logfile)
start_logfile_at = time.time()

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

planner = TSRPlanner( 
                robot_collision_checker=robot_collision_checker,
                delegate_planner=actual_planner)

# load planning request
try:
    method = getattr(planner, yamldict['request']['method'])
except (AttributeError, UnsupportedPlanningError) as e:
    print ('{} does not support planning method {}!'.format(planner, yamldict['request']['method']))
    import sys
    sys.exit(0)

num_trials = 3 # if 'plantotsr' in str(method_name).lower() else 3

# load ik solver for robot in case it's needed
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,
    iktype=openravepy.IkParameterizationType.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()

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


# deserialize environment
prpy.serialization.deserialize_environment(yamldict['environment'], env=env, reuse_bodies=[robot])

from prpy.clone import Clone


reqdict = {}
resdict = {'ok':[], 'planning_time':[], 'planner_used':[], 'error':[]}
reqdict['collisionchecker'] = env.GetCollisionChecker().GetDescription()
reqdict['args'] = yamldict['request']['args']
reqdict['kw_args'] = yamldict['request']['kw_args']
reqdict['method'] = yamldict['request']['method'] 
reqdict['planner_name'] = getattr(actual_planner, 'name', str(planner))


planning_times = dict()
success = dict()

for icandidates in [1, 12, 25]:
    
    method_kwargs['num_candidates'] = icandidates

    # call planning method itself ...
    print('calling planner {} for {} ...'.format(planner, method_name))
    print ('icandidate: {}'.format(icandidates))
    from prpy.util import Timer, SetTrajectoryTags
    from prpy.planning.base import Tags

    planning_times[icandidates] = []
    success[icandidates] = []
    for k in xrange(3):
        error_msg = None
        traj = None
        try:
            with Clone(env, lock=True) as planning_env:
                robot = planning_env.GetRobots()[0]
                start_time = time.time()
                traj = method(robot, *method_args, **method_kwargs)    
                planning_time = time.time() - start_time 
        except (UnsupportedPlanningError, AttributeError) as e: 
            print (e)
            import sys
            sys.exit(0)
        except PlanningError as e: 
            error_msg = str(e)
            print (error_msg)
            planning_time = time.time() - start_time 

        planning_times[icandidates].append(planning_time)
        ok = True if traj else False
        success[icandidates].append(ok)

    with open('tsr-tuning-completed.log','a') as fp:
        loginfo = ' '.join([str(x) for x in [logfile,icandidates, planning_times[icandidates], success[icandidates]]])
        fp.write(loginfo+"\n")

resdict['ok'] = success
resdict['planning_times'] = planning_times
    
yamldict_res = {}
yamldict_res['environment'] = yamldict['environment']
yamldict_res['request'] = reqdict
yamldict_res['result'] = resdict

filename = get_filename(logfile, str(actual_planner), method_name)
with open(filename,'w') as fp:
    yaml.safe_dump(yamldict_res, fp)
    print ('\n{} written\n'.format(filename))


