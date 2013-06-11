#!/usr/bin/env python
# -*- coding: utf-8 -*-

## @package libherb.tsr Utilities for TSRs and TSR chains.


import openravepy
import numpy
import numpy.random
import kin
#import copy

'''Functions for Serializing TSRs and TSR Chains

SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw)

Input:
manipindex (int): the 0-indexed index of the robot's manipulator
bodyandlink (str): body and link which is used as the 0 frame. Format 'body_name link_name'. To use world frame, specify 'NULL'
T0_w (double 4x4): transform matrix of the TSR's reference frame relative to the 0 frame
Tw_e (double 4x4): transform matrix of the TSR's offset frame relative the w frame
Bw (double 1x12): bounds in x y z roll pitch yaw. Format: [x_min x_max y_min y_max...]

Output:
outstring (str): string to use for SerializeTSRChain function


SerializeTSRChain(bSampleFromChain,bConstrainToChain,numTSRs,allTSRstring,mimicbodyname,mimicbodyjoints)

Input:
bSampleStartFromChain (0/1): 1: Use this chain for sampling start configurations   0:Ignore for sampling starts
bSampleGoalFromChain (0/1): 1: Use this chain for sampling goal configurations   0:Ignore for sampling goals
bConstrainToChain (0/1): 1: Use this chain for constraining configurations   0:Ignore for constraining
numTSRs (int): Number of TSRs in this chain (must be > 0)
allTSRstring (str): string of concetenated TSRs generated using SerializeTSR. Should be like [TSRstring 1 ' ' TSRstring2 ...]
mimicbodyname (str): name of associated mimicbody for this chain (NULL if none associated)
mimicbodyjoints (int [1xn]): 0-indexed indices of the mimicbody's joints that are mimiced (MUST BE INCREASING AND CONSECUTIVE [FOR NOW])

Output:
outstring (str): string to include in call to cbirrt planner'''


def SerializeTransform12Col(tm,format='%.5f'):
   return ' '.join([(format%(i,)) for i in tm[0:3,:].T.reshape(12)])

def SerializeArray(a,format='%.5f'):
   return ' '.join([(format%(i,)) for i in a.reshape(-1)])

class TSR(object): # force new-style class
   def __init__(self, T0_w=None, Tw_e=None, Bw=None, manip=None, bodyandlink='NULL'):
      if T0_w is None: T0_w = numpy.eye(4)
      if Tw_e is None: Tw_e = numpy.eye(4)
      if Bw   is None: Bw   = numpy.zeros((6,2))
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
         if self.Bw[i,0] != self.Bw[i,1]:
            Bwdims += 1

      if vals is None:
         Bwvals = self.Bw[:,0] + (self.Bw[:,1]-self.Bw[:,0]) * numpy.random.random_sample(6)
      elif len(vals) == Bwdims:
         Bwvals = numpy.zeros(6)
         vals_i = 0
         for i in range(6):
            if self.Bw[i,0] != self.Bw[i,1]:
               Bwvals[i] = vals[vals_i]
               vals_i += 1
            else:
               Bwvals[i] = self.Bw[i,0]
      elif len(vals) == 6:
         Bwvals = vals
      else:
         raise ValueError('vals must be of length %d or 6!' % Bwdims)
      print 'Bwvals[5]:', Bwvals[5]
      xyzypr = [Bwvals[0], Bwvals[1], Bwvals[2], Bwvals[5], Bwvals[4], Bwvals[3]]
      Tw = kin.pose_to_H(kin.pose_from_xyzypr(xyzypr))
      trans = numpy.dot(numpy.dot(self.T0_w,Tw), self.Tw_e)
      return trans
      
   def serialize(self):
      return '%d %s %s %s %s'%(self.manipindex, self.bodyandlink, SerializeTransform12Col(self.T0_w), SerializeTransform12Col(self.Tw_e), SerializeArray(self.Bw))

class TSRChain(object): # force new-style class
   def __init__(self, sample_start=False, sample_goal=False, constrain=False, TSR=None, TSRs=None, mimicbodyname='NULL', mimicbodyjoints=None):
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
      outstring = '%d %d %d'%(int(self.sample_start), int(self.sample_goal), int(self.constrain))
      outstring += ' %d %s' % (numTSRs, allTSRstring)
      outstring += ' ' + self.mimicbodyname
      if len(self.mimicbodyjoints) > 0:
         outstring += ' %d %s'%(len(self.mimicbodyjoints),SerializeArray(self.mimicbodyjoints))
      return outstring


