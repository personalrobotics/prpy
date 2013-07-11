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

import numpy

class ConfigurationLibrary(object):
    """
    Library of named configurations. Configurations are specified in terms of
    *groups* of DOF indices. Groups must be added with *add_group* before a
    configuration may be added.
    """
    def __init__(self):
        self._indices = set()
        self._groups = dict()
        self._configs = dict()

    """
    Load named configurations from disk. The root of the YAML file should be of
the following format:

        configurations:
            configuration1:
                group1: [ 0, 0, ..dof_values... ]
                group2: [ ... ]
                ...
            configuration2:
                ...
            ...

    :param path: path to an input YAML file
    """
    def load_yaml(self, path):
        with open(path, 'rb') as config_file:
            import yaml
            config_yaml = yaml.load(config_file)

        for name, groups in config_yaml['configurations'].items():
            self.add_configuration(name, **groups)

    """
    :param name: group name
    :param dof_indices: DOF indices
    """
    def add_group(self, name, dof_indices):
        duplicate_indices = self._indices.intersection(dof_indices)

        if name in self._groups:
            raise Exception('There is already a group named %s.' % name)
        elif duplicate_indices:
            sorted_duplicates = sorted(list(duplicate_indices))
            raise Exception('Indices [ %s ] are already bound to names.'\
                            % ', '.join(map(str, sorted_duplicates)))

        self._groups[name] = numpy.array(dof_indices).copy()
        self._indices |= set(dof_indices)

    """
    :param name: configuration name
    :param **kw_args: an array of joint values for each group
    """
    def add_configuration(self, name, **kw_args):
        all_indices = list()
        all_values = list()

        for group_name, dof_values in kw_args.items():
            if group_name not in self._groups:
                raise Exception('There is no group named %s.' % group_name)

            group_indices = self._groups[group_name]
            if len(group_indices) != len(dof_values):
                raise Exception('Expected %d values; got %d.'\
                                % (len(group_indices), len(dof_values)))

            all_indices.extend(group_indices)
            all_values.extend(dof_values)

        self._configs[name] = (all_indices, all_values)
        return all_indices, all_values

    """
    :return (dof_indices, dof_values): DOF indices and corresponding DOF values
    """
    def get_configuration(self, name):
        if name in self._configs:
            return self._configs[name]
        else:
            raise Exception('There is no configuration named %s.' % name)
