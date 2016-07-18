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
from os import listdir
from os.path import isfile, join


def get_filename(logfile_name, planner, method, output_dir):

    # dump to file    
    if not os.path.isdir(os.path.abspath(output_dir)):
        os.mkdir(output_dir)

    output_dir = os.path.abspath(output_dir+'/'+planner)
    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)

    method_name = yamldict['request']['method']
    logfile_name = filter(lambda x: 'log-' in x, re.split(r'[/]|\.yaml',logfile_name))[0]
    filename = "postprocess_" + logfile_name + "_" + method + "_" + planner + ".yaml"
    completePath = os.path.join(output_dir, filename)
    return completePath


parser = argparse.ArgumentParser(description='replay planning request log file')
parser.add_argument('--logdir', required=True)
parser.add_argument('--planner', default=None, help=('cbirrt OMPL_RRTConnect chomp vectorfield' 'greedy-ik trajopt'))
parser.add_argument('--viewer', '-v', type=str, default='interactivemarker', help='The viewer to attach (none for no viewer)')
parser.add_argument('--outputdir', default="postprocess", type=str, help='Output directory.')
parser.add_argument('--overwrite', action='store_true')
parser.add_argument('--force', action='store_true', help='Run even if original planning query failed.')
parser.add_argument('--max-trials', default=1, type=int, 
    help='Number of max trials until getting raw trajectory. Set this only for randomized planners')
parser.add_argument('--num-postprocess-trials', default=1, type=int, help='Number of postprocess runs per setting')

args = parser.parse_args()

# start openrave
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
atexit.register(openravepy.RaveDestroy)
env = openravepy.Environment()
atexit.register(env.Destroy)

# This code only handles herb
import herbpy
env, robot = herbpy.initialize(sim=True)

# List of planners supported 
planner_list = {'cbirrt': CBiRRTPlanner(),
                'OMPL_RRTConnect':prpy.planning.ompl.OMPLPlanner('RRTConnect'),
                'chomp':CHOMPPlanner(),
                'vectorfield':VectorFieldPlanner(),
                'greedy-ik':GreedyIKPlanner(),
                'trajopt':TrajoptPlanner()}


logdir = args.logdir
logfiles = []
for dirpath, dirnames, filenames in os.walk(logdir):
    subdirs = [os.path.join(dirpath, dirname) for dirname in dirnames]
    for subdir in subdirs:
        print (subdir)
        logfiles.extend([join(subdir, f) for f in listdir(subdir) if isfile(join(subdir, f)) and f.endswith('.yaml')])

logfiles.extend([join(logdir, f) for f in listdir(logdir) if isfile(join(logdir, f)) and f.endswith('.yaml')])

# files = [join(logdir, f) for f in listdir(logdir) if isfile(join(logdir, f)) and f.split('.')[-1] == 'yaml']

for f in logfiles:
    print ("Reading " + f)

    # read log file
    yamldict = yaml.safe_load(open(f))
    if not yamldict['result']['ok'] and not args.force:
        print ("Planning query failed in {}; aborting planning. Use --force to force planning.".format(f))
        continue

    
    # setup planner
    if args.planner:
        planner_name = args.planner
    else:
        planner_name = yamldict['result']['planner_used']
        if planner_name.lower().startswith('snap') or yamldict['result']['planner_used'] == 'snap':
            print ("Skipping {}; Postprocessing has no effect on {}.".format(f, planner_name))
            continue
        planner_name = filter(lambda x: x.lower() in planner_name.lower(), planner_list)[0]


    actual_planner = planner_list[planner_name]

    # Wrapps planner with for PlanToTSR and PlanToNamedConfiguration
    planner = FirstSupported(Sequence(actual_planner,
                                      TSRPlanner(delegate_planner=actual_planner)),
                                      NamedPlanner(delegate_planner=actual_planner)
                                     )

    # deserialize environment
    prpy.serialization.deserialize_environment(yamldict['environment'], env=env, reuse_bodies=[robot])

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
    method_name = yamldict['request']['method']
    filename = get_filename(f, str(actual_planner), method_name, args.outputdir)

    # Quit if file already exists 
    if os.path.isfile(filename) and not args.overwrite:
        print (filename, " already exists. Use --overwrite to re-prostprocess. Skipping...")
        continue

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

    if not traj:
        print ("Failed to generate trajectory.")
        continue

    # Get defaultTags
    from prpy.util import GetTrajectoryTags
    tags = GetTrajectoryTags(traj)

    constrained = tags.get(Tags.CONSTRAINED, False)
    if constrained:
        print ("Skipping constrained trajectory")
        continue

    print ("Post processing...")

    from or_parabolicsmoother.prpy.retimer import HauserParabolicSmoother

    # Postprocessing options
    # blend_options = {}
    # blend_options['iterations'] = [1, 2, 3, 4, 5]
    # blend_options['radius'] = [0.5]
    timelimits = numpy.arange(0.1,5.3,0.5).tolist()
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
    properties['request'] = {'timelimit':0}
    properties['result'] = dict()
    smoothers.append((smoother, properties))


    # # with shortcut only 
    for t in timelimits:
        smoother = HauserParabolicSmoother(do_blend=False,
                                           do_shortcut=True,
                                           timelimit=t)
        properties = {}
        properties['request'] = {'do_shortcut': True, 'timelimit':t}
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
        with Clone(robot.GetEnv(), clone_env=postprocess_env) as cloned_env:
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

