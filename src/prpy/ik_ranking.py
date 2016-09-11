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


def NoRanking(robot, ik_solutions):
    """
    Return IK solutions with an arbitrary ranking.
    """
    return numpy.ones(ik_solutions.shape[0])


def JointLimitAvoidance(robot, ik_solutions):
    """
    Score IK solutions by their distance from joint limits. This is implemented
    with a quadratic loss function that measures distance from limits.
    """
    with robot.GetEnv():
        lower_limits, upper_limits = robot.GetActiveDOFLimits()

    lower_distance = ik_solutions - lower_limits
    upper_distance = upper_limits - ik_solutions
    distance = numpy.minimum(lower_distance, upper_distance)
    return -numpy.sum(distance**2, axis=1)


class NominalConfiguration(object):
    def __init__(self, q_nominal, max_deviation=2*numpy.pi):
        """
        Score IK solutions by their distance from a nominal configuration.
        @param q_nominal nominal configuration
        @param max_deviation specify a maximum allowable per-joint deviation
                             from the nominal configuration, default is 2*PI
        """
        self.q_nominal = q_nominal
        self.max_deviation = max_deviation

    def __call__(self, robot, ik_solutions):
        assert ik_solutions.shape[1] == self.q_nominal.shape[0]
        L_2 = numpy.linalg.norm(ik_solutions - self.q_nominal,
                                axis=1, ord=2)

        # Ignore IK solutions that are more than the specified distance away.
        # (This means a closer solution must exist!)
        if self.max_deviation is not None:
            L_inf = numpy.linalg.norm(ik_solutions - self.q_nominal,
                                      axis=1, ord=numpy.inf)
            L_2[L_inf > self.max_deviation] = numpy.inf

        return L_2



class MultipleNominalConfigurations(object):
    def __init__(self, q_nominal_list, max_deviation=2*numpy.pi):
        """
        Score IK solutions by their summed distance to multiple configurations
        @param q_nominal_list list of configurations to compare distance to
        @param max_deviation specify a maximum allowable per-joint deviation
                             from the nominal configuration, default is 2*PI
        """
        self.all_scorers = [NominalConfiguration(q_nominal, max_deviation) for q_nominal in q_nominal_list]
        self.max_deviation = max_deviation

    def __call__(self, robot, ik_solutions):
        return sum([scorer(robot, ik_solutions) for scorer in self.all_scorers])

