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
import prpy_lemur.lemur
import prpy_lemur.roadmaps

from prpy.planning import (
    FirstSupported,
    Sequence,
    PlanningError,
    UnsupportedPlanningError,
    TSRPlanner,
    CBiRRTPlanner,
    SnapPlanner,
)

import time 
import os.path
from os.path import join, isfile
from os import listdir
import re

def get_filename(logfile, planner, method, outputdir):

    logfile = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',logfile))[0]
    savedir = os.path.join(outputdir, method, logfile, planner)
    if not os.path.exists(savedir):
        os.makedirs(savedir)

    filename = '_'.join(str(x) for x in ['replay', logfile, method, planner])
    filename += '.yaml'

    completePath = os.path.join(savedir, filename)
    return completePath


parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logfile', required=True)
parser.add_argument('--planner', required=True, help='SnapTrajopt CBiRRT')
parser.add_argument('--outdir', default='', type=str, help='Save log to outdir')
args = parser.parse_args()

snap = SnapPlanner()
trajopt = TrajoptPlanner()

if args.planner == 'SnapTrajopt':
    planner = TSRPlanner(delegate_planner=Sequence(snap, trajopt))
    setattr(planner, 'name', 'SnapTrajopt')
else:
    planner = CBiRRTPlanner()


logfile = args.logfile
print ("Reading ", logfile)

yamldict = yaml.safe_load(open(logfile))
method_name = yamldict['request']['method']
if method_name.lower() != 'plantotsr':
    print ("Skipping", method_name)

# deserialize environment
import herbpy
env, robot = herbpy.initialize(sim=True)

method = getattr(planner, yamldict['request']['method'])

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
from prpy.util import Timer, SetTrajectoryTags, GetTrajectoryTags
from prpy.planning.base import Tags

error_msg = None
traj = None

num_trials = 5 

filename = get_filename(logfile, getattr(planner, 'name', str(planner)), method_name, args.outdir)
reqdict = dict()
resdict = {'planner_used':[],
           'ok':[],
           'planning_time':[]}

reqdict['method'] = yamldict['request']['method']
reqdict['planner_name'] = str(planner)

for j in range(num_trials):

    start_time = time.time()
    try:
        traj = method(robot, *method_args, **method_kwargs)    
    except PlanningError as e: 
        error_msg = str(e)
        print (error_msg)
    except AttributeError as e:
        pass
    finally:
        planning_time = time.time() - start_time
    tags = GetTrajectoryTags(traj)
    resdict['planner_used'].append(None if not traj else tags[Tags.PLANNER])
    resdict['ok'].append(True if traj else False)
    resdict['planning_time'].append(planning_time)

    ok = True if traj else False
    planner_name = getattr(planner, 'name', str(planner))
    with open('replay-completed-'+planner_name+'.log', 'a') as fp:
        trial_info = ' '.join(str(x) for x in [logfile, planner_name, method_name, 'trial', j, ok])
        trial_info += ' ' + str(planning_time)
        fp.write(trial_info+"\n")

yamldict_res = {}
yamldict_res['environment'] = yamldict['environment']
yamldict_res['request'] = reqdict
yamldict_res['result'] = resdict


with open(filename,'w') as fp:
    yaml.safe_dump(yamldict_res, fp)
    print ('\n{} written\n'.format(filename))



