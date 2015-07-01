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

import openravepy
import numpy
import numpy.random
import kin

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

    def sample(self, vals=None):
        Bwdims = 0
        for i in range(6):
            if self.Bw[i, 0] != self.Bw[i, 1]:
                Bwdims += 1

        if vals is None:
            Bwvals = self.Bw[:, 0] + (self.Bw[:, 1] - self.Bw[:, 0]) \
                * numpy.random.random_sample(6)
        elif len(vals) == Bwdims:
            Bwvals = numpy.zeros(6)
            vals_i = 0
            for i in range(6):
                if self.Bw[i, 0] != self.Bw[i, 1]:
                    Bwvals[i] = vals[vals_i]
                    vals_i += 1
                else:
                    Bwvals[i] = self.Bw[i, 0]
        elif len(vals) == 6:
            Bwvals = vals
        else:
            raise ValueError('vals must be of length %d or 6!' % Bwdims)
        # print 'Bwvals[5]:', Bwvals[5]
        xyzypr = [Bwvals[0], Bwvals[1], Bwvals[2],
                  Bwvals[5], Bwvals[4], Bwvals[3]]
        Tw = kin.pose_to_H(kin.pose_from_xyzypr(xyzypr))

        trans = numpy.dot(numpy.dot(self.T0_w, Tw), self.Tw_e)
        return trans

    def serialize(self):
        return '%d %s %s %s %s' % (self.manipindex, self.bodyandlink,
                                   SerializeTransform12Col(self.T0_w),
                                   SerializeTransform12Col(self.Tw_e),
                                   SerializeArray(self.Bw))

    def to_dict(self):
        """ Convert this TSR to a python dict. """
        return {
            'T0_w': self.T0_w.tolist(),
            'Tw_e': self.Tw_e.tolist(),
            'Bw': self.Bw.tolist(),
            'manipindex': int(self.manipindex),
            'bodyandlink': str(self.bodyandlink),
        }

    @staticmethod
    def from_dict(x):
        """ Construct a TSR from a python dict. """
        return TSR(
            T0_w=numpy.array(x['T0_w']),
            Tw_e=numpy.array(x['Tw_e']),
            Bw=numpy.array(x['Bw']),
            manip=numpy.array(x.get('manipindex', -1)),
            bodyandlink=numpy.array(x.get('bodyandlink', 'NULL'))
        )

    def to_json(self):
        """ Convert this TSR to a JSON string. """
        import json
        return json.dumps(self.to_dict())

    @staticmethod
    def from_json(x, *args, **kw_args):
        """
        Construct a TSR from a JSON string.

        This method internally forwards all arguments to `json.loads`.
        """
        import json
        x_dict = json.loads(x, *args, **kw_args)
        return TSR.from_dict(x_dict)

    def to_yaml(self):
        """ Convert this TSR to a YAML string. """
        import yaml
        return yaml.dumps(self.to_dict())

    @staticmethod
    def from_yaml(x, *args, **kw_args):
        """
        Construct a TSR from a YAML string.

        This method internally forwards all arguments to `yaml.safe_load`.
        """
        import yaml
        x_dict = yaml.safe_load(x, *args, **kw_args)
        return TSR.from_dict(x_dict)


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

    def to_dict(self):
        """ Construct a TSR chain from a python dict. """
        return {
            'sample_goal': self.sample_goal,
            'sample_start': self.sample_start,
            'constrain': self.constrain,
            'mimicbodyname': self.mimicbodyname,
            'mimicbodyjoints': self.mimicbodyjoints,
            'tsrs': [tsr.to_dict() for tsr in self.TSRs],
        }

    @staticmethod
    def from_dict(x):
        """ Construct a TSR chain from a python dict. """
        return TSRChain(
            sample_start=x['sample_start'],
            sample_goal=x['sample_goal'],
            constrain=x['constrain'],
            TSRs=[TSR.from_dict(tsr) for tsr in x['tsrs']],
            mimicbodyname=x['mimicbodyname'],
            mimicbodyjoints=x['mimicbodyjoints'],
        )

    def to_json(self):
        """ Convert this TSR chain to a JSON string. """
        import json
        return json.dumps(self.to_dict())

    @staticmethod
    def from_json(x, *args, **kw_args):
        """
        Construct a TSR chain from a JSON string.

        This method internally forwards all arguments to `json.loads`.
        """
        import json
        x_dict = json.loads(x, *args, **kw_args)
        return TSR.from_dict(x_dict)

    def to_yaml(self):
        """ Convert this TSR chain to a YAML string. """
        import yaml
        return yaml.dumps(self.to_dict())

    @staticmethod
    def from_yaml(x, *args, **kw_args):
        """
        Construct a TSR chain from a YAML string.

        This method internally forwards all arguments to `yaml.safe_load`.
        """
        import yaml
        x_dict = yaml.safe_load(x, *args, **kw_args)
        return TSR.from_dict(x_dict)

    def sample(self):
        if len(self.TSRs) == 0:
            return None

        T0_w = self.TSRs[0].T0_w
        for idx in range(len(self.TSRs)):
            tsr_current = self.TSRs[idx]
            tsr_current.T0_w = T0_w
            T0_w = tsr_current.sample()

        return T0_w
