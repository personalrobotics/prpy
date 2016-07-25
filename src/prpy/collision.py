# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Chris Dellin <cdellin@gmail.com>
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

import openravepy
from prpy.planning.exceptions import CollisionPlanningError
from prpy.planning.exceptions import SelfCollisionPlanningError


class SimpleRobotCollisionChecker:

    def __init__(self, robot):
        self.robot = robot
        self.env = robot.GetEnv()

    def CheckCollision(self):
        report = openravepy.CollisionReport()
        if self.env.CheckCollision(self.robot, report=report):
            raise CollisionPlanningError.FromReport(report)
        elif self.robot.CheckSelfCollision(report=report):
            raise SelfCollisionPlanningError.FromReport(report)


class BakedRobotCollisionChecker:

    def __init__(self, robot):
        self.robot = robot
        self.env = robot.GetEnv()
        self.checker = self.env.GetCollisionChecker()
        kb_type = self.checker.SendCommand('BakeGetType')
        self.checker.SendCommand('BakeBegin')
        self.env.CheckCollision(self.robot)
        self.robot.CheckSelfCollision()
        self.baked = openravepy.RaveCreateKinBody(self.env, kb_type)
        self.checker.SendCommand('BakeEnd')

    def CheckCollision(self):
        report = openravepy.CollisionReport()
        if self.checker.CheckSelfCollision(self.baked, report):
            raise CollisionPlanningError.FromReport(report)
