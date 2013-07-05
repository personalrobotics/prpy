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

#!/usr/bin/env python
# -*- coding: utf-8 -*-

## @package libherb.kin Helper functions for creating and converting transforms and rotations and all their representations.


import numpy

# Python implementation of functions from libcd

# quat = numpy.array([qx,qy,qz,qw]) # 4 element quaternion list with qw last
# H = numpy.eye(4) #4x4 transformation matrix
# R = numpy.eye(3) #3x3 rotation matrix
# pose = [tx,ty,tz, qx,qy,qz,qw] # 7 element pose list with 3 element translation first followed by 4 element quaternion with qw last

# Ideas:
#   TODO: rewrite the quat functions to match the OpenRAVE quat format ([qw,qx,qy,qz]).
#   TODO: rewrite the pose functions to match the OpenRAVE pose format ([qw,qx,qy,qz,tx,ty,tz]).

def pose_normalize(pose):
    nm = numpy.linalg.norm(pose[3:7])
    pose[3:7] /= nm

def R_to_quat(R):

    q = numpy.zeros(4)

    t = 1 + R[0,0] + R[1,1] + R[2,2]
    if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        imax = 0
    elif R[1,1] > R[2,2]:
        imax = 1
    else:
        imax = 2

    if t > 0.000001:
        r = numpy.sqrt(t)
        s = 0.5 / r
        q[0] = (R[2,1]-R[1,2])*s # x
        q[1] = (R[0,2]-R[2,0])*s # y
        q[2] = (R[1,0]-R[0,1])*s # z
        q[3] = 0.5 * r # w
    elif imax == 0: # Rxx largest
        r = numpy.sqrt(1 + R[0,0] - R[1,1] - R[2,2])
        s = 0.5 / r
        q[0] = 0.5 * r # x
        q[1] = (R[0,1]+R[1,0])*s # y
        q[2] = (R[0,2]+R[2,0])*s # z
        q[3] = (R[2,1]-R[1,2])*s # w
    elif imax == 1: # Ryy largest
        r = numpy.sqrt(1 - R[0,0] + R[1,1] - R[2,2])
        s = 0.5 / r
        q[0] = (R[1,0]+R[0,1])*s # x
        q[1] = 0.5 * r # y
        q[2] = (R[1,2]+R[2,1])*s # z ???
        q[3] = (R[0,2]-R[2,0])*s # w
    else: # Rzz largest
        r = numpy.sqrt(1 - R[0,0] - R[1,1] + R[2,2])
        s = 0.5 / r
        q[0] = (R[2,0]+R[0,2])*s # x
        q[1] = (R[2,1]+R[1,2])*s # y
        q[2] = 0.5 * r # z
        q[3] = (R[1,0]-R[0,1])*s # w
    return q


def R_from_quat(quat):
   R = numpy.zeros((3,3))
   xx = quat[0] * quat[0]
   xy = quat[0] * quat[1]
   xz = quat[0] * quat[2]
   xw = quat[0] * quat[3]
   yy = quat[1] * quat[1]
   yz = quat[1] * quat[2]
   yw = quat[1] * quat[3]
   zz = quat[2] * quat[2]
   zw = quat[2] * quat[3]
   R[0,0] = 1 - 2 * (yy + zz)
   R[0,1] = 2 * (xy - zw)
   R[0,2] = 2 * (xz + yw)
   R[1,0] = 2 * (xy + zw)
   R[1,1] = 1 - 2 * (xx + zz)
   R[1,2] = 2 * (yz - xw)
   R[2,0] = 2 * (xz - yw)
   R[2,1] = 2 * (yz + xw)
   R[2,2] = 1 - 2 * (xx + yy)
   return R


def pose_to_H(pose):
   H = numpy.eye(4)
   H[0:3,0:3] = R_from_quat(pose[3:7])
   H[0:3,3] = pose[0:3]
   return H

def pose_from_H(H):
   pose = numpy.zeros(7)
   pose[0:3] = H[0:3,3]
   pose[3:7] = R_to_quat(H[0:3,0:3])
   return pose


def quat_to_ypr(quat):
   ypr = numpy.zeros(3)
   qx = quat[0]
   qy = quat[1]
   qz = quat[2]
   qw = quat[3]   
   sinp2 = qw*qy-qz*qx
   if sinp2 > 0.49999:
      ypr[0] = -2.0*numpy.arctan2(qx,qw)
      ypr[1] = 0.5*numpy.pi
      ypr[2] = 0.0
   elif sinp2 < -0.49999:
      ypr[0] = 2.0*numpy.arctan2(qx,qw)
      ypr[1] = -0.5*numpy.pi
      ypr[2] = 0.0
   else:
      ypr[0] = numpy.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
      ypr[1] = numpy.arcsin(2*sinp2)
      ypr[2] = numpy.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
   return ypr


def quat_from_ypr(ypr):
   quat = numpy.zeros(4)
   cy2 = numpy.cos(0.5*ypr[0])
   sy2 = numpy.sin(0.5*ypr[0])
   cp2 = numpy.cos(0.5*ypr[1])
   sp2 = numpy.sin(0.5*ypr[1])
   cr2 = numpy.cos(0.5*ypr[2])
   sr2 = numpy.sin(0.5*ypr[2])
   quat[0] = -sy2*sp2*cr2 + cy2*cp2*sr2 # qx
   quat[1] =  cy2*sp2*cr2 + sy2*cp2*sr2 # qy
   quat[2] = -cy2*sp2*sr2 + sy2*cp2*cr2 # qz
   quat[3] =  sy2*sp2*sr2 + cy2*cp2*cr2 # qw
   return quat


def pose_from_xyzypr(xyzypr):
   pose = numpy.zeros(7)
   cy2 = numpy.cos(0.5*xyzypr[3])
   sy2 = numpy.sin(0.5*xyzypr[3])
   cp2 = numpy.cos(0.5*xyzypr[4])
   sp2 = numpy.sin(0.5*xyzypr[4])
   cr2 = numpy.cos(0.5*xyzypr[5])
   sr2 = numpy.sin(0.5*xyzypr[5])
   pose[0] = xyzypr[0]
   pose[1] = xyzypr[1]
   pose[2] = xyzypr[2]
   pose[3] = -sy2*sp2*cr2 + cy2*cp2*sr2 # qx
   pose[4] =  cy2*sp2*cr2 + sy2*cp2*sr2 # qy
   pose[5] = -cy2*sp2*sr2 + sy2*cp2*cr2 # qz
   pose[6] =  sy2*sp2*sr2 + cy2*cp2*cr2 # qw
   return pose

def pose_to_xyzypr(pose):
   xyzypr = numpy.zeros(6)
   xyzypr[0] = pose[0]
   xyzypr[1] = pose[1]
   xyzypr[2] = pose[2]
   qx = pose[3]
   qy = pose[4]
   qz = pose[5]
   qw = pose[6]
   sinp2 = qw*qy-qz*qx
   if sinp2 > 0.49999:
      xyzypr[3] = -2.0*numpy.arctan2(qx,qw)
      xyzypr[4] = 0.5*numpy.pi
      xyzypr[5] = 0.0
   elif sinp2 < -0.49999:
      xyzypr[3] = 2.0*numpy.arctan2(qx,qw)
      xyzypr[4] = -0.5*numpy.pi
      xyzypr[5] = 0.0
   else:
      xyzypr[3] = numpy.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
      xyzypr[4] = numpy.arcsin(2*sinp2)
      xyzypr[5] = numpy.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
   return xyzypr


def H_from_op_diff(pos_from, pos_to_diff):
    '''
    Produce a transform H rooted at location pos_from
    with Z axis pointed in direction pos_to_diff
    Taken from libcds kin.c
    2011-08-01 cdellin
    '''
    H = numpy.eye(4)
    # Set d
    H[0,3] = pos_from[0]
    H[1,3] = pos_from[1]
    H[2,3] = pos_from[2]
    # Define Z axis in direction of arrow */
    zlen = numpy.sqrt(numpy.dot(pos_to_diff,pos_to_diff))
    H[0,2] = pos_to_diff[0]/zlen
    H[1,2] = pos_to_diff[1]/zlen
    H[2,2] = pos_to_diff[2]/zlen
    # Define other axes
    if abs(H[0,2]) > 0.9:
        # Z is too close to e1, but sufficiently far from e2
        # cross e2 with Z to get X (and normalize)
        vlen = numpy.sqrt(H[2,2]*H[2,2] + H[0,2]*H[0,2])
        H[0][0] = H[2,2] / vlen
        H[1][0] = 0.0
        H[2][0] = -H[0,2] / vlen
        # Then Y = Z x X
        H[0,1] = H[1,2] * H[2,0] - H[2,2] * H[1,0]
        H[1,1] = H[2,2] * H[0,0] - H[0,2] * H[2,0]
        H[2,1] = H[0,2] * H[1,0] - H[1,2] * H[0,0]
    else:
        # Z is sufficiently far from e1;
        # cross Z with e1 to get Y (and normalize)
        vlen = numpy.sqrt(H[2,2]*H[2,2] + H[1,2]*H[1,2])
        H[0,1] = 0.0
        H[1,1] = H[2,2] / vlen
        H[2,1] = -H[1,2] / vlen
        # Then X = Y x Z
        H[0,0] = H[1,1] * H[2,2] - H[2,1] * H[1,2]
        H[1,0] = H[2,1] * H[0,2] - H[0,1] * H[2,2]
        H[2,0] = H[0,1] * H[1,2] - H[1,1] * H[0,2]
    return H


def invert_H(H):
    '''
    Invert transform H
    '''
    R = H[0:3,0:3]
    d = H[0:3,3]
    Hinv = numpy.eye(4)
    Hinv[0:3,0:3] = R.T
    Hinv[0:3,3] = -numpy.dot(R.T, d)
    return Hinv


def xyzt_to_H(xyzt):
    '''
    Convert [x,y,z,theta] to 4x4 transform H
    theta is rotation about z-axis
    '''
    ypr = [xyzt[3],0.0,0.0]
    quat = quat_from_ypr(ypr)
    pose = [xyzt[0],xyzt[1],xyzt[2],quat[0],quat[1],quat[2],quat[3]]
    H = pose_to_H(pose)
    return H

def xyzypr_to_H(xyzypr):
    '''
    Convert [x,y,z,yaw,pitch,roll] to 4x4 transform H
    '''
    quat = quat_from_ypr(xyzypr[3:6])
    pose = [xyzypr[0],xyzypr[1],xyzypr[2],quat[0],quat[1],quat[2],quat[3]]
    H = pose_to_H(pose)
    return H

def quat_to_axisangle(quat):
   a2 = numpy.arccos(quat[3]);
   angle = 2.0*a2;
   sina2inv = 1.0/numpy.sin(a2);
   axis = numpy.zeros(3)
   axis[0] = sina2inv * quat[0];
   axis[1] = sina2inv * quat[1];
   axis[2] = sina2inv * quat[2];
   return (axis, angle)



def transform_comparison(H1, H2):
    '''
    Compare two 4x4 transforms H1 and H2. 
    Return the differnce in position and rotation.
    '''
    T_difference = numpy.dot( invert_H(H1), H2 )
    quat_difference = R_to_quat(T_difference[0:3,0:3]) #[x,y,z,w]
    rotation_difference = numpy.abs(2.0* numpy.arccos(quat_difference[3])) # 2*acos(qw)
    position_difference = numpy.sqrt( numpy.dot( numpy.array(T_difference[0:3,3]), numpy.array(T_difference[0:3,3]) ) )
    return position_difference, rotation_difference
