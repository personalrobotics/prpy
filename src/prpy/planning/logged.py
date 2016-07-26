#!/usr/bin/env python

# Copyright (c) 2016, Carnegie Mellon University
# All rights reserved.
# Authors: Chris Dellin <cdellin@gmail.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import division
import collections
import math
import time

import openravepy
import numpy
import yaml

import prpy.planning.base
import prpy.serialization

def serialize_or_errorstr(value):
    try:
        return prpy.serialization.serialize(value)
    except prpy.exceptions.UnsupportedTypeSerializationException as ex:
        return 'Error: {}'.format(str(ex))

class LoggedPlanner(prpy.planning.base.MetaPlanner):
    """Planner wrapper which serializes planning requets and responses.

    LoggedPlanner delegates all planning calls to the sub-planner
    passed to it on construction.  It does not affect the input to
    or output from this planner.

    On every request, LoggedPlanner serializes the environment,
    planning method request arguments, and the returned trajectory.
    It creates a timestamped yaml file in the current directory:
     - log-20160101-010203.567890.yaml
    """

    def __init__(self, planner):
        super(LoggedPlanner, self).__init__()
        self.planner = planner
    
    def __str__(self):
        return 'Logged({0:s})'.format(self.planner)
    
    def has_planning_method(self, method_name):
        return self.planner.has_planning_method(method_name)
    
    def get_planning_method_names(self):
        return self.planner.get_planning_method_names()
    
    def get_planners(self, method_name):
        return [self.planner]
    
    def plan(self, method, args, kw_args):
        
        # get log file name
        stamp = time.time()
        struct = time.localtime(stamp)
        fn = 'log-{}.{:06d}.yaml'.format(
            time.strftime('%Y%m%d-%H%M%S', struct),
            int(1.0e6*(stamp-math.floor(stamp))))
        
        # retrieve enviroment
        if 'robot' in kw_args:
            env = kw_args['robot'].GetEnv()
        elif len(args) > 0:
            env = args[0].GetEnv()
        else:
            raise RuntimeError('could not retrieve env from planning request!')
        
        # serialize environment
        envdict = prpy.serialization.serialize_environment(env, uri_only=True)
        
        # serialize planning request
        reqdict = {}
        reqdict['method'] = method # string
        reqdict['args'] = []
        for arg in args:
            reqdict['args'].append(serialize_or_errorstr(arg))
        reqdict['kw_args'] = {}
        for key,value in kw_args.items():
            # if key serialization fails, it thros UnsupportedTypeSerializationException
            key = prpy.serialization.serialize(key)
            reqdict['kw_args'][key] = serialize_or_errorstr(value)
        
        # get ready to serialize result
        resdict = {}
        
        try:
            plan_fn = getattr(self.planner, method)
            traj = plan_fn(*args, **kw_args)
            resdict['ok'] = True
            resdict['traj_first'] = list(map(float,traj.GetWaypoint(0)))
            resdict['traj_last'] = list(map(float,traj.GetWaypoint(traj.GetNumWaypoints()-1)))
            resdict['traj'] = traj.serialize(0)
            return traj
            
        except prpy.planning.PlanningError as ex:
            resdict['ok'] = False
            resdict['exception'] = str(ex)
            raise
        
        except:
            resdict['ok'] = False
            resdict['exception'] = 'unknown exception type'
            raise
        
        finally:
            # create yaml dictionary to be serialized
            yamldict = {}
            yamldict['environment'] = envdict
            yamldict['request'] = reqdict
            yamldict['result'] = resdict
            fp = open(fn,'w')
            yaml.safe_dump(yamldict, fp)
            fp.close()
        
        
