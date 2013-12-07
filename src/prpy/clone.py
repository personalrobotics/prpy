#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
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

import openravepy, threading

class CloneException(Exception):
    pass

class ClonedEnvironment(openravepy.Environment):
    local = threading.local()

    def __init__(self, parent_env, destroy_on_exit):
        self.__class__.get_envs().append(self)
        self.clone_parent = parent_env
        self.destroy_on_exit = destroy_on_exit

    def __enter__(self):
        pass

    def __exit__(self, *args):
        if self.destroy_on_exit:
            self.Destroy()
        else:
            self.__class__.get_envs().pop()

    def Destroy(self):
        self.__class__.get_envs().pop()
        openravepy.Environment.Destroy(self)

    @classmethod
    def get_env(cls):
        environments = cls.get_envs()
        if not environments:
            raise CloneException('Cloned environments may only be used in a Clone context.')
        return environments[-1]

    @classmethod
    def get_envs(cls):
        if not hasattr(cls.local, 'environments'):
            cls.local.environments = list()
        return cls.local.environments

def Clone(env, clone_env=None, destroy=None, options=openravepy.CloningOptions.Bodies):
    # Default to destroying environments we create and not destroying
    # environments that were passed in as an argument.
    if destroy is None:
        destroy = clone_env is None

    if clone_env is None:
        clone_env = openravepy.Environment()

    clone_env.Clone(env, options)
    clone_env.__class__ = ClonedEnvironment
    clone_env.__init__(env, destroy)
    return clone_env

def Cloned(*instances):
    clone_env = ClonedEnvironment.get_env()
    clone_instances = list()

    for instance in instances:
        if isinstance(instance, openravepy.Robot):
            clone_instance = clone_env.GetRobot(instance.GetName())
        elif isinstance(instance, openravepy.KinBody):
            clone_instance = clone_env.GetKinBody(instance.GetName())
        elif isinstance(instance, openravepy.KinBody.Link):
            clone_instance = Cloned(instance.GetParent()).GetLink(instance.GetName())
        elif isinstance(instance, openravepy.Robot.Manipulator):
            clone_instance = Cloned(instance.GetRobot()).GetManipulator(instance.GetName())
        else:
            raise CloneException('Unable to clone object of type {0:s}.'.format(type(instance)))

        if clone_instance is None:
            raise CloneException('{0:s} is not in the cloned environment.'.format(instance))

        clone_instance.clone_parent = instance
        clone_instances.append(clone_instance)

    if len(instances) == 1:
        return clone_instances[0]
    else:
        return clone_instances
