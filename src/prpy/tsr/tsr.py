# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
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

# @package libherb.tsr Utilities for TSRs and TSR chains.

import openravepy
import numpy
import numpy.random
import kin

NANBW = numpy.ones(6)*float('nan')

"""
Functions for Serializing TSRs and TSR Chains

SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw)

Input:
manipindex (int): the 0-indexed index of the robot's manipulator
bodyandlink (str): body and link which is used as the 0 frame. Format
                   'body_name link_name'. To use world frame, specify 'NULL'
T0_w (double 4x4): transform matrix of the TSR's reference frame relative to
                   the 0 frame
Tw_e (double 4x4): transform matrix of the TSR's offset frame relative to the
                   w frame
Bw (double 1x12): bounds in x y z roll pitch yaw.
                  Format: [x_min, x_max, y_min, y_max ...]

Output:
outstring (str): string to use for SerializeTSRChain function

SerializeTSRChain(bSampleFromChain,
                  bConstrainToChain,
                  numTSRs,
                  allTSRstring,
                  mimicbodyname,
                  mimicbodyjoints)

Input:
bSampleStartFromChain (0/1): 1: Use this chain for sampling start configs
                             0: Ignore for sampling starts
bSampleGoalFromChain (0/1): 1: Use this chain for sampling goal configs
                            0: Ignore for sampling goals
bConstrainToChain (0/1): 1: Use this chain for constraining configs
                         0: Ignore for constraining
numTSRs (int): Number of TSRs in this chain (must be > 0)
allTSRstring (str): string of concetenated TSRs generated using SerializeTSR.
                    Should be like [TSRstring 1 ' ' TSRstring2 ...]
mimicbodyname (str): name of associated mimicbody for this chain
                     (NULL if none associated)
mimicbodyjoints (int [1xn]): 0-indexed indices of the mimicbody's joints that
                             are mimiced (MUST BE INCREASING AND CONSECUTIVE)

Output:
outstring (str): string to include in call to cbirrt planner
"""


def SerializeTransform12Col(tm, format='%.5f'):
    return ' '.join([(format % (i,)) for i in tm[0:3, :].T.reshape(12)])


def SerializeArray(a, format='%.5f'):
    return ' '.join([(format % (i,)) for i in a.reshape(-1)])


class TSR(object):  # force new-style class

    def __init__(self, T0_w=None, Tw_e=None, Bw=None,
                 manip=None, bodyandlink='NULL'):
        if T0_w is None:
            T0_w = numpy.eye(4)
        if Tw_e is None:
            Tw_e = numpy.eye(4)
        if Bw is None:
            Bw = numpy.zeros((6, 2))
        self.T0_w = T0_w
        self.Tw_e = Tw_e
        self.Bw = Bw
        if manip is None:
            self.manipindex = -1
        elif type(manip) == openravepy.openravepy_int.Robot:
            self.manipindex = manip.GetActiveManipulatorIndex()
        else:
            self.manipindex = manip
        self.bodyandlink = bodyandlink

    def to_transform(self, vals):
        """
        Converts a [x y z roll pitch yaw] into an
        end-effector transform.

        @param  vals [x y z roll pitch yaw]
        @return trans 4x4 transform
        """
        if len(vals) != 6:
            raise ValueError('vals must be of length 6')
        xyzypr = [vals[0], vals[1], vals[2],
                  vals[5], vals[4], vals[3]]
        Tw = kin.pose_to_H(kin.pose_from_xyzypr(xyzypr))
        trans = numpy.dot(numpy.dot(self.T0_w, Tw), self.Tw_e)
        return trans

    def to_Bwvals(self, trans):
        """
        Converts an end-effector transform to Bw values
        @param  trans  4x4 transform
        @return Bwvals 6x1 vector of Bw values
        """
        Tw = numpy.dot(numpy.dot(numpy.linalg.inv(self.T0_w), trans),
                       numpy.linalg.inv(self.Tw_e))
        pose = kin.pose_from_H(Tw)
        ypr = kin.quat_to_ypr(pose[3:7])
        Bwvals = [pose[0], pose[1], pose[2],
                  ypr[2], ypr[1], ypr[0]]
        return Bwvals

    def in_tsr(self, trans):
        """
        Checks if a transform is within a TSR
        @param  trans 4x4 transform
        @return       True if inside and False if not
        """
        EPSILON = 0.001
        Bwvals = self.to_Bwvals(trans)
        check = [((x >= self.Bw[i, 0]) and (x <= self.Bw[i, 1]))
                 or (self.Bw[i, 1] - self.Bw[i, 0] < EPSILON)
                 for i, x in enumerate(Bwvals)]
        return all(check)

    def sample(self, vals=NANBW):
        """
        Samples from Bw to generate an end-effector transform.
        Can specify some Bw values optionally.

        @param vals   (optional) a 6-vector of Bw with float('nan') for
                      dimensions to sample uniformly.
        @return       4x4 transform
        """
        if len(vals) != 6:
            raise ValueError('vals must be of length 6')
        if any([(x < self.Bw[i, 0]) or (x > self.Bw[i, 1])
                and not numpy.isnan(x) for i, x in enumerate(vals)]):
            raise ValueError('specified vals must be within bounds')

        Bwvals = [self.Bw[i, 0] + (self.Bw[i, 1] - self.Bw[i, 0]) *
                  numpy.random.random_sample()
                  if numpy.isnan(val) else val for i, val in enumerate(vals)]

        return self.to_transform(Bwvals)

    def serialize(self):
        return '%d %s %s %s %s' % (self.manipindex, self.bodyandlink,
                                   SerializeTransform12Col(self.T0_w),
                                   SerializeTransform12Col(self.Tw_e),
                                   SerializeArray(self.Bw))

    def serialize_dict(self):
        return {
            'T0_w': self.T0_w.tolist(),
            'Tw_e': self.Tw_e.tolist(),
            'Bw': self.Bw.tolist(),
            'manipindex': int(self.manipindex),
            'bodyandlink': str(self.bodyandlink),
        }

    @staticmethod
    def deserialize_dict(x):
        return TSR(
            T0_w=numpy.array(x['T0_w']),
            Tw_e=numpy.array(x['Tw_e']),
            Bw=numpy.array(x['Bw']),
            manip=numpy.array(x['manipindex']),
            bodyandlink=numpy.array(x['bodyandlink'])
        )


class TSRChain(object):

    def __init__(self, sample_start=False, sample_goal=False, constrain=False,
                 TSR=None, TSRs=None,
                 mimicbodyname='NULL', mimicbodyjoints=None):
        """
        A TSR chain is a combination of TSRs representing a motion constraint.

        TSR chains compose multiple TSRs and the conditions under which they
        must hold.  This class provides support for start, goal, and/or
        trajectory-wide constraints.  They can be constructed from one or more
        TSRs which must be applied together.

        @param sample_start apply constraint to start configuration sampling
        @param sample_goal apply constraint to goal configuration sampling
        @param constrain apply constraint over the whole trajectory
        @param TSR a single TSR to use in this TSR chain
        @param TSRs a list of TSRs to use in this TSR chain
        @param mimicbodyname name of associated mimicbody for this chain
        @param mimicbodyjoints 0-indexed indices of the mimicbody's joints that
                               are mimiced (MUST BE INCREASING AND CONSECUTIVE)
        """
        self.sample_start = sample_start
        self.sample_goal = sample_goal
        self.constrain = constrain
        self.mimicbodyname = mimicbodyname
        if mimicbodyjoints is None:
            self.mimicbodyjoints = []
        else:
            self.mimicbodyjoints = mimicbodyjoints
        self.TSRs = []
        if TSR is not None:
            self.append(TSR)
        if TSRs is not None:
            for tsr in TSRs:
                self.append(tsr)

    def append(self, tsr):
        self.TSRs.append(tsr)

    def serialize(self):
        allTSRstring = ' '.join([tsr.serialize() for tsr in self.TSRs])
        numTSRs = len(self.TSRs)
        outstring = '%d %d %d' % (int(self.sample_start),
                                  int(self.sample_goal),
                                  int(self.constrain))
        outstring += ' %d %s' % (numTSRs, allTSRstring)
        outstring += ' ' + self.mimicbodyname
        if len(self.mimicbodyjoints) > 0:
            outstring += ' %d %s' % (len(self.mimicbodyjoints),
                                     SerializeArray(self.mimicbodyjoints))
        return outstring

    def serialize_dict(self):
        return {
            'sample_goal': self.sample_goal,
            'sample_start': self.sample_start,
            'constrain': self.constrain,
            'mimicbodyname': self.mimicbodyname,
            'mimicbodyjoints': self.mimicbodyjoints,
            'tsrs': [tsr.serialize_dict() for tsr in self.TSRs],
        }

    @staticmethod
    def deserialize_dict(x):
        return TSRChain(
            sample_start=x['sample_start'],
            sample_goal=x['sample_goal'],
            constrain=x['constrain'],
            TSRs=[TSR.deserialize_dict(tsr) for tsr in x['tsrs']],
            mimicbodyname=x['mimicbodyname'],
            mimicbodyjoints=x['mimicbodyjoints'],
        )

    def sample(self, vals=None):
        """
        Samples from the Bw chain to generate an end-effector transform.
        Can specify some Bw values optionally.

        @param vals   (optional) a 6-vector of Bw with float('nan') for
                      dimensions to sample uniformly.
        @return T0_w  4x4 transform
        """

        if len(self.TSRs) == 0:
            return None
        if vals is None:
            vals = NANBW*len(self.TSRs)

        T0_w = self.TSRs[0].T0_w
        for idx in range(len(self.TSRs)):
            tsr_current = self.TSRs[idx]
            tsr_current.T0_w = T0_w
            T0_w = tsr_current.sample(vals[idx])

        return T0_w
