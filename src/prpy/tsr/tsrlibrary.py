# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Jennifer King <jeking@cs.cmu.edu>
#          Michael Koval <mkoval@cs.cmu.edu>
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

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import collections, functools, logging, numpy, os.path

logger = logging.getLogger(__name__)


class TSRFactory(object):
    
    def __init__(self, robot_name, obj_name, action_name):
        logger.debug('Loading %s, %s, %s' % (robot_name, obj_name, action_name))
        self.robot_name = robot_name
        self.obj_name = obj_name
        self.action_name = action_name

    def __call__(self, func):
        TSRLibrary.add_factory(func, self.robot_name, self.obj_name, self.action_name)

        #functools.wrap
        from functools import wraps
        @wraps(func)
        def wrapped_func(robot, kinbody, *args, **kw_args):
            return func( robot, kinbody, *args, **kw_args)
        return wrapped_func


class TSRLibrary(object):
    all_factories = collections.defaultdict(lambda: collections.defaultdict(dict))
    generic_kinbody_key = "_*"  # Something that is unlikely to be an actual kinbody name
    
    def __init__(self, robot, robot_name=None):
        """
        Create a TSRLibrary for a robot.
        @param robot the robot to store TSRs for
        @param robot_name optional robot name, inferred from robot by default
        """
        self.robot = robot

        if robot_name is not None:
            self.robot_name = robot_name
        else:
            self.robot_name = self.get_object_type(robot)
            logger.debug('Inferred robot name "%s" for TSRLibrary.', self.robot_name)

    def __call__(self, kinbody, action_name, *args, **kw_args):
        """
        Return a list of TSRChains to perform an action on an object with this
        robot. Raises KeyError if no matching TSRFactory exists.
        @param robot the robot to run the tsr on 
        @param kinbody the KinBody to act on
        @param action_name the name of the action
        @return list of TSRChains
        """
        kinbody_name = kw_args.get('kinbody_name', None)
        if kinbody_name is None and kinbody is not None:
            kinbody_name = self.get_object_type(kinbody)
            logger.debug('Inferred KinBody name "%s" for TSR.', kinbody_name)

        f = None
        try:
            f = self.all_factories[self.robot_name][kinbody_name][action_name]
        except KeyError, ignored:
            pass

        if f is None:
            try:
                f = self.all_factories[self.robot_name][self.generic_kinbody_key][action_name]
            except KeyError:
                raise KeyError('There is no TSR factory registered for action "{:s}"'
                               ' with robot "{:s}" and object "{:s}".'.format(
                        action_name, self.robot_name, kinbody_name))

        return f(self.robot, kinbody=kinbody, *args, **kw_args)

    def load_yaml(self, yaml_file):
        """
        Load a set of simple TSRFactory's from a YAML file. Each TSRFactory
        contains exactly one TSRChain.
        @param yaml_file path to the input YAML file
        """
        import yaml
        from prpy.tsr.tsr import TSR, TSRChain

        with open(yaml_file, 'r') as f:
            yaml_data = yaml.load(f)

        for chain in yaml_data:
            try:
                robot_name = chain['robot']
                kinbody_name = chain['kinbody']
                action_name = chain['action']

                sample_start = False
                if 'sample_start' in chain:
                    sample_start = bool(chain['sample_start'])
                    
                sample_goal = False
                if 'sample_goal' in chain:
                    sample_goal = bool(chain['sample_goal'])
                        
                constrain = False
                if 'constrain' in chain:
                    constrain = bool(chain['constrain'])


                @TSRFactory(robot_name, kinbody_name, action_name)
                def func(robot, obj):
                    manip_idx = robot.GetActiveManipulatorIndex()

                    all_tsrs = []
                    for tsr in chain['TSRs']:
                        T0_w = obj.GetTransform()
                        Tw_e = numpy.array(tsr['Tw_e'])
                        Bw = numpy.array(tsr['Bw'])
                    
                        yaml_tsr = TSR(T0_w = T0_w, 
                                       Tw_e = Tw_e, 
                                       Bw = Bw, 
                                       manip = manip_idx)
                        all_tsrs.append(yaml_tsr)

                    yaml_chain = TSRChain(sample_start=sample_start,
                                          sample_goal = sample_goal, 
                                          constrain = constrain, 
                                          TSRs = all_tsrs)

                    return [yaml_chain]

            except Exception, e:
                logger.error('Failed to load TSRChain: %s - (Chain: %s)' % (str(e), chain))
                raise IOError('Failed to load TSRChain: %s - (Chain: %s)' % (str(e), chain))
        

    @classmethod
    def add_factory(cls, func, robot_name, object_name, action_name):
        """
        Register a TSR factory function for a particular robot, object, and
        action. The function must take a robot and a KinBody and return a list
        of TSRChains. Optionaly, it may take arbitrary positional and keyword
        arguments. This method is used internally by the TSRFactory decorator.
        @param func function that returns a list of TSRChains
        @param robot_name name of the robot
        @param object_name name of the object
        @param action_name name of the action
        """
        logger.debug('Adding TSRLibrary factory for robot "%s", object "%s", action "%s".',
            robot_name, object_name, action_name)

        if object_name is None:
            object_name = cls.generic_kinbody_key

        if action_name in cls.all_factories[robot_name][object_name]:
            logger.warning('Overwriting duplicate TSR factory for action "%s"'
                           ' with robot "%s" and object "%s"',
                action_name, robot_name, object_name)

        cls.all_factories[robot_name][object_name][action_name] = func

    @staticmethod
    def get_object_type(body):
        """
        Infer the name of a KinBody by inspecting its GetXMLFilename.
        @param body KinBody or Robot object
        @return object name
        """
        path = body.GetXMLFilename()
        filename = os.path.basename(path)
        name, _, _ = filename.partition('.') # remove extension

        if name:
            return name
        else:
            raise ValueError('Failed inferring object name from path: {:s}'.format(path))
