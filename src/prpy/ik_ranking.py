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
    return numpy.sum(distance**2, axis=1)

class NominalConfiguration(object):
    def __init__(self, q_nominal):
        """
        Score IK solutions by their distance from a nominal configuration.
        @param q_nominal nominal configuration
        """
        self.q_nominal = q_nominal

    def __call__(self, robot, ik_solutions):
        assert ik_solutions.shape[1] == self.q_nominal.shape[0]
        return numpy.linalg.norm(ik_solutions - self.q_nominal, axis=0)
