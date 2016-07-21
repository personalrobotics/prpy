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

def get_filename(logfile, planner, method, outputdir, trial, seed=None):

    logfile = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',logfile))[0]
    savedir = os.path.join(outputdir, method, logfile, planner)
    if not os.path.exists(savedir):
        os.makedirs(savedir)

    filename = '_'.join(str(x) for x in ['replay', logfile, method, planner, 'trial', trial])
    if seed is not None:
        filename += '_seed_'+str(seed)
    filename += '.yaml'

    completePath = os.path.join(savedir, filename)
    return completePath


parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logdir', required=True)
parser.add_argument('--planner', required=True, help=('cbirrt OMPL_RRTConnect snap chomp vectorfield' 
                                                        ' greedy-ik trajopt lemur cachedlemur'))
parser.add_argument('--outdir', default='', type=str, help='Save log to outdir')
args = parser.parse_args()

planner_list = args.planner.split(' ')
randomized = [x.lower() for x in ['cbirrt', 'OMPL RRTConnect', 'CachedLemur', 'Lemur']]

planners = []
for pl in planner_list: 
    if pl == 'cbirrt':
        planners.append(CBiRRTPlanner())
    elif pl == 'OMPL_RRTConnect':
        planners.append(prpy.planning.ompl.OMPLPlanner('RRTConnect'))
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
    elif pl == 'CachedLemur':
        for i in range(10):
            planner = prpy_lemur.lemur.LEMURPlanner(
                roadmap=prpy_lemur.roadmaps.CachedHaltonOffDens(
                    is_cache_required=True, num_per_batch=10000,
                    gamma_factor=1.0, scaling='loglog_n', seed=i))
            setattr(planner, 'seed', i)
            setattr(planner, 'name', 'CachedLemur')
            planners.append(planner)

    elif pl == 'Lemur':
        for i in range(10):
            planner = prpy_lemur.lemur.LEMURPlanner(
                roadmap=prpy_lemur.roadmaps.HaltonOffDens(
                    num_per_batch=10000, gamma_factor=1.0, scaling='loglog_n', seed=i))
            setattr(planner, 'seed', i)
            setattr(planner, 'name', 'Lemur')
            planners.append(planner)
    else:
      raise ValueError("Unrecognized planner")

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
    if method_name.lower() == 'plantotsr':
        print ("Skipping PlanToTSR...")
        continue

    for actual_planner in planners:

        planner = FirstSupported(
          Sequence(actual_planner,
                    TSRPlanner(delegate_planner=actual_planner)),
                    NamedPlanner(delegate_planner=actual_planner))

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

        # call planning method itself ...
        print('calling planning method {} ...'.format(actual_planner))
        from prpy.util import Timer, SetTrajectoryTags
        from prpy.planning.base import Tags
        
        error_msg = None
        traj = None

        num_trials = 5 if str(actual_planner).lower() in randomized else 1
        for j in range(num_trials):
            print (actual_planner, 'trial ', j, ' seed ',  getattr(actual_planner, 'seed', None))

            filename = get_filename(logfile, getattr(actual_planner, 'name', str(actual_planner)), method_name, 
                                    args.outdir, j, getattr(actual_planner, 'seed', None))

            start_time = time.time()
            try:
                traj = method(robot, *method_args, **method_kwargs)    
            except PlanningError as e: 
                error_msg = str(e)
                print (error_msg)
            except AttributeError as e:
                break;
            # except:
            #     import sys
            #     error_msg = sys.exc_info()[0]
            #     print (error_msg)
            finally:
                planning_time = time.time() - start_time
                # SetTrajectoryTags(traj, {Tags.PLAN_TIME: planning_time}, append=True)

            
            reqdict = {}
            resdict = {}
            reqdict['method'] = yamldict['request']['method'] 
            reqdict['seed'] = getattr(actual_planner, 'seed', None)
            reqdict['planner_name'] = str(planner)
            resdict['ok'] = True if traj else False
            resdict['planning_time'] = planning_time
            if traj: 
                resdict['traj'] = traj.serialize()
            if error_msg:
                resdict['error'] = str(error_msg)
                
            yamldict_res = {}
            yamldict_res['environment'] = yamldict['environment']
            yamldict_res['request'] = reqdict
            yamldict_res['result'] = resdict

            ok = True if traj else False

            with open(filename,'w') as fp:
                yaml.safe_dump(yamldict_res, fp)
                print ('\n{} written\n'.format(filename))

            print ("Planning time: ", planning_time)

            with open('replay-completed-'+str(actual_planner)+'.log', 'a') as fp:
                trial_info = ' '.join(str(x) for x in [logfile, str(actual_planner), method_name, 'trial', j, ok])
                if getattr(actual_planner, 'seed', None) is not None:
                    trial_info += ' seed ' + str(getattr(actual_planner, 'seed', None))
                trial_info += ' ' + str(planning_time)
                fp.write(trial_info+"\n")

    logfile_duration = time.time() - start_logfile_at
    # with open('replay-completed.log', 'a') as fp:
    #     fp.write(logfile+" "  + str(logfile_duration) +"\n")

    print ("Took", logfile_duration, "s to finish ", logfile )

    