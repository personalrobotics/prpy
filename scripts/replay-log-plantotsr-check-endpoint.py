#!/usr/bin/python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import argparse
import atexit
import math

import time 
    
import numpy
import openravepy
import yaml
from or_trajopt import TrajoptPlanner
import prpy.planning
import prpy.serialization
from prpy.planning import (
    TSRPlanner,
    SnapPlanner,
    PlanningError
)
from os import listdir
from os.path import isfile, join
import re
from or_trajopt import TrajoptPlanner

# This script tests PlanToTSR on TSRPlanner with different delegate planners

def get_filename(logfile_name, planner_name):

    # dump to file    
    import os.path
    save_path = os.path.abspath('.')
    logfile_name = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',logfile_name))[0]
    filename = "replay_"+planner_name+"_tsr_plantotsr_" + logfile_name
    filename = filename + ".yaml"
    completePath = os.path.join(save_path, filename)
    return completePath

parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logdir', required=True)
parser.add_argument('--planner', required=True)
args = parser.parse_args()

# start openrave
import herbpy
env, robot = herbpy.initialize(sim=True)

if args.planner == 'snap':
    planner = TSRPlanner(delegate_planner=SnapPlanner())
elif args.planner == 'trajopt':
    planner = TSRPlanner(delegate_planner=TrajoptPlanner())
else:
    raise ValueError("Unrecognized planner.")

logdir = args.logdir

files = [join(logdir, f) for f in listdir(logdir) if isfile(join(logdir, f)) and f.split('.')[-1] == 'yaml']

success = 0
failure = 0
failure_distance = []

for f in files[200:]:

    print (f)
    # read log file
    yamldict = yaml.safe_load(open(f))

    if yamldict['request']['method'] != 'PlanToTSR':
        print ("Skipping {}, has {}.".format(f, yamldict['request']['method']))
        continue

    # deserialize environment
    if robot: 
       prpy.serialization.deserialize_environment(yamldict['environment'], env=env, reuse_bodies=[robot])
    else:
       prpy.serialization.deserialize_environment(yamldict['environment'], env=env)
       robots = prpy.serialization.deserialize_robots(yamldict['environment'], env)
       robot = robots[0]


    method = getattr(planner, yamldict['request']['method'])
    method_args = []
    for method_arg in yamldict['request']['args']:
        method_args.append(prpy.serialization.deserialize(env, method_arg))
    method_kwargs = {}
    for key,value in yamldict['request']['kw_args'].items():
        method_kwargs[key] = prpy.serialization.deserialize(env, value)

    # # attempt to retrieve robot
    # if 'robot' in method_kwargs:
    #     robot = method_kwargs['robot']
    # elif len(method_args) > 0:
    #     robot = method_args[0]
    # else:
    #     raise RuntimeError('could not retrieve robot!')

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
    traj = None
    
    method_kwargs['num_attempts'] = 3
    try:
        traj = method(robot, *method_args, **method_kwargs)
    except PlanningError as e: 
	continue
    
    #check if end point of trajectory satisfy tsr
    if traj:
        in_tsr = False
        robot.GetActiveManipulator().SetDOFValues(traj.GetWaypoint(-1))
        print (len(method_args[0]), " arguments") 
        for tsr in  method_args[0]:
            if not isinstance(tsr, prpy.tsr.tsr.TSRChain):
                print ('tsr not tsr')
                import IPython; IPython.embed()
            if not tsr.sample_goal:
                continue
            if (tsr.contains(robot.GetActiveManipulator().hand.GetTransform())):
                in_tsr = True
                break; 
        if in_tsr:
            success += 1
        else:
            failure += 1
            import IPython; IPython.embed()
            distance = (tsr.distance(robot.GetActiveManipulator().hand.GetTransform())[0])
            failure_distance.append(distance)
            print (f + " failing, distance ", str(distance))
    print (success, failure)
    print (failure_distance)   
print ("Success ", success)
print ("Failure ", failure)

import IPython; IPython.embed()

