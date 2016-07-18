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
    CBiRRTPlanner,
    PlanningError
)
from os import listdir
from os.path import isfile, join
import re

# This script tests PlanToTSR on CBiRRT with different parameters

def get_filename(logfile_name):

    # dump to file    
    import os.path
    save_path = os.path.abspath('.')
    logfile_name = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',logfile_name))[0]
    filename = "replay_cbirrt_" + logfile_name
    filename = filename + ".yaml"
    completePath = os.path.join(save_path, filename)
    return completePath

parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logdir', required=True)
args = parser.parse_args()

# start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

planner = CBiRRTPlanner()


logdir = args.logdir

files = [join(logdir, f) for f in listdir(logdir) if isfile(join(logdir, f)) and f.split('.')[-1] == 'yaml']

for f in files:
    print (f)
    # read log file
    yamldict = yaml.safe_load(open(f))

    if yamldict['request']['method'] != 'PlanToTSR':
        print ("Skipping {}, has {}.".format(f, yamldict['request']['method']))
        continue


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
    method_kwargs['timelimit'] = 60
    psamples = numpy.linspace(0.1, 0.5, 5).tolist()
    print (psamples )

    results = {'psamples':psamples,
               'planning_time':[],
               'ok':[]}

    for psample in psamples:
        method_kwargs['psample'] = psample
        start_time = time.time()
        success = False
        try:
            traj = method(robot, *method_args, **method_kwargs)
            success = True
        except PlanningError as e: 
            pass
        results['planning_time'].append(time.time() - start_time )
        results['ok'].append(success)



    reqdict = {}
    reqdict['method'] = yamldict['request']['method']
    reqdict['planner_name'] = str(planner)
    yamldict_res = {}
    yamldict_res['environment'] = yamldict['environment']
    yamldict_res['request'] = reqdict
    yamldict_res['result'] = results

    filename = get_filename(f)
    with open(filename,'w') as fp:
        yaml.safe_dump(yamldict_res, fp)
        print ('\n{} written\n'.format(filename))


