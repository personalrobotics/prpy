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

def get_filename(logfile, planner, method, outputdir):

    logfile = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',logfile))[0]
    savedir = os.path.join(outputdir, method, logfile, planner)
    if not os.path.exists(savedir):
        os.makedirs(savedir)

    filename = '_'.join(str(x) for x in ['replay', logfile, method, planner])

    completePath = os.path.join(savedir, filename)
    return completePath


parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logfile', required=True)
parser.add_argument('--outdir', default='', type=str, help='Save log to outdir')
args = parser.parse_args()


snap = SnapPlanner()
trajopt = TrajoptPlanner()
planner = TSRPlanner(delegate_planner=Sequence(snap, trajopt))

timelimits = [0.1,0.3,0.5,0.7,1.0,1.3,1.5,1.7,1.9]
num_attempts = [1,3,6,9,11,15]


# read log file
logfile = args.logfile 
print ("Reading ", logfile)

yamldict = yaml.safe_load(open(logfile))
method_name = yamldict['request']['method']
if method_name.lower() != 'PlanToTSR'.lower():
    import sys
    sys.exit(0)

# load planning request
try:
    method = getattr(planner, yamldict['request']['method'])
except (AttributeError, UnsupportedPlanningError) as e:
    print ('{} does not support planning method {}!'.format(planner, yamldict['request']['method']))
    import sys
    sys.exit(0)

# deserialize environment
import herbpy
env, robot = herbpy.initialize(sim=True)
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

planning_times = dict()
success = dict()

for iattempt in num_attempts:
    for t in timelimits:
        method_kwargs['tsr_timeout'] = t
        method_kwargs['num_attempts'] = iattempt

        # call planning method itself ...
        print('calling planner {} for {} ...'.format(planner, method_name))
        from prpy.util import Timer, SetTrajectoryTags
        from prpy.planning.base import Tags

        planning_times[(iattempt, t)] = []
        success[(iattempt,t)] = []
        for k in xrange(5):
            error_msg = None
            traj = None
            start_time = time.time()
            try:
                traj = method(robot, *method_args, **method_kwargs)    
            except PlanningError as e: 
                error_msg = str(e)
                print (error_msg)
            finally:
                planning_time = time.time() - start_time

            planning_times[(iattempt, t)].append(planning_time)
            ok = True if traj else False
            success[(iattempt,t)].append(ok)

        with open('tsr-tuning-completed.log','a') as fp:
            loginfo = ' '.join([str(x) for x in [logfile, 'SnapTrajopt', 'TSR', iattempt, t, 
                                                 sum(planning_times[(iattempt,t)]), success[(iattempt,t)]]])
            fp.write(loginfo+"\n")

reqdict = {}
resdict = {}
reqdict['method'] = yamldict['request']['method'] 
reqdict['planner_name'] = str(planner)
resdict['ok'] = success
resdict['planning_times'] = planning_times
    
yamldict_res = {}
yamldict_res['environment'] = yamldict['environment']
yamldict_res['request'] = reqdict
yamldict_res['result'] = resdict

filename = get_filename(logfile, 'SnapTrajopt', method_name, args.outdir)

import pickle
pickle.dump(yamldict_res, open(filename+'.pkl','w'))

for key, val in resdict['ok'].iteritems():
    resdict[str(key)] = val
    resdict.remove(key)
for key, val in resdict['planning_times'].iteritems():
    resdict[str(key)] = val
    resdict.remove(key)

yamldict_res['result'] = resdict
with open(filename+'.pkl','w') as fp:
    yaml.safe_dump(yamldict_res, fp)
    print ('\n{} written\n'.format(filename))



