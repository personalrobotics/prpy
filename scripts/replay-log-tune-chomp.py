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

    filename = '_'.join(str(x) for x in ['replay', logfile, method, planner,'.yaml'])

    completePath = os.path.join(savedir, filename)
    return completePath

def runSnap(yamldict):
    snap = SnapPlanner()
    snapPlanner = FirstSupported(
            Sequence(TSRPlanner(delegate_planner=snap)),
                    NamedPlanner(delegate_planner=snap))
    print ("Running snap")
    # load planning request
    try:
        method = getattr(snapPlanner, yamldict['request']['method'])
    except (AttributeError, UnsupportedPlanningError) as e:
        print ('{} does not support planning method {}!'.format(snapPlanner, yamldict['request']['method']))
        return False

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
    
    method_kwargs['seed'] = 17

    try:
        traj = method(robot, *method_args, **method_kwargs)
        print ("Snap successful")
        return True
    except PlanningError as e: 
        error_msg = str(e)
        print (error_msg)
        return False



parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logdir', required=True)
parser.add_argument('--outdir', default='', type=str, help='Save log to outdir')
args = parser.parse_args()

chomp = CHOMPPlanner()
planner = FirstSupported(
            Sequence(TSRPlanner(delegate_planner=chomp)),
                    NamedPlanner(delegate_planner=chomp))


preferred_numbers = [10, 16, 25, 40, 63, 100]

logdir = args.logdir
logfiles = []
for dirpath, dirnames, filenames in os.walk(logdir):
    subdirs = [os.path.join(dirpath, dirname) for dirname in dirnames]
    for subdir in subdirs:
        print (subdir)
        logfiles.extend([join(subdir, f) for f in listdir(subdir) if isfile(join(subdir, f)) and f.endswith('.yaml')])

logfiles.extend([join(logdir, f) for f in listdir(logdir) if isfile(join(logdir, f)) and f.endswith('.yaml')])

# read log file
for logfile in logfiles:

    print ("Reading ", logfile)
    start_logfile_at = time.time()

    yamldict = yaml.safe_load(open(logfile))
    method_name = yamldict['request']['method']
    if method_name.lower() == 'PlanToEndEffectorOffset'.lower():
        print ("Skipping ", method_name)
        continue

    if runSnap(yamldict):
        print ("Snap succeeded, skipping")
        with open('snap-succeeded.log', 'a') as fp:
            fp.write(logfile +"\n")
        continue

    # load planning request
    try:
        method = getattr(planner, yamldict['request']['method'])
    except (AttributeError, UnsupportedPlanningError) as e:
        print ('{} does not support planning method {}!'.format(planner, yamldict['request']['method']))
        continue

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

    for lambda_ in preferred_numbers:
        for n_iter in preferred_numbers:
    
            method_kwargs['lambda_'] = lambda_
            method_kwargs['n_iter'] = n_iter
            method_kwargs['seed'] = 17

            # call planning method itself ...
            print('calling planner {} for {} ...'.format(planner, method_name))
            from prpy.util import Timer, SetTrajectoryTags
            from prpy.planning.base import Tags

            print ('lambda: ', lambda_, 'n_iter', n_iter)
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

            planning_times[str((lambda_, n_iter))] = planning_time
            ok = True if traj else False
            success[str((lambda_, n_iter))] = ok

            with open('replay-chomp-completed.log', 'a') as fp:
                loginfo = ' '.join([str(x) for x in [logfile, 'CHOMP', method_name, lambda_, n_iter, planning_time, ok]])
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

        filename = get_filename(logfile, 'chomp', method_name, args.outdir)

        with open(filename,'w') as fp:
            yaml.safe_dump(yamldict_res, fp)
            print ('\n{} written\n'.format(filename))

    logfile_duration = time.time() - start_logfile_at
    with open('replay-chomp-completed.log', 'a') as fp:
        fp.write(logfile+" "  + str(logfile_duration) +"\n")

    print ("Took", logfile_duration, "s to finish ", logfile )