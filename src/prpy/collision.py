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
from openravepy import (
    CollisionOptions,
    CollisionOptionsStateSaver,
    CollisionReport,
    RaveCreateKinBody,
    openrave_exception,
  )
from prpy.exceptions import PrPyException
from prpy.planning.exceptions import (
    CollisionPlanningError,
    SelfCollisionPlanningError,
)


class SimpleRobotCollisionCheckerContextManager(object):
    """RobotCollisionChecker which uses the standard OpenRAVE interface.

    This RobotCollisionChecker is instantiated with a robot,
    and when the CheckCollision() method is called, it attempts
    an Env followed by a Self collision check (with shortcutting)
    through the standard OpenRAVE interface.  If a collision is
    found, either CollisionPlanningError or SelfCollisionPlanningError
    is raised.
    """

    def __init__(self, robot, collision_options):
        self.robot = robot
        self.env = robot.GetEnv()

        self.checker = self.env.GetCollisionChecker()
        if self.checker is None:
            raise PrPyException('No collision checker found on environment')

        self.collision_saver = CollisionOptionsStateSaver(
            self.checker, collision_options)

    @property
    def collision_options(self):
        return self.collision_saver.newoptions

    def __enter__(self):
        self.collision_saver.__enter__()
        return self

    def __exit__(self, type, value, traceback):
        self.collision_saver.__exit__(type, value, traceback)

    def CheckCollision(self, report=None):
        if self.env.CheckCollision(self.robot, report=report):
            return True
        elif self.robot.CheckSelfCollision(report=report):
            return True
        return False

    def VerifyCollisionFree(self):
        report = CollisionReport()
        if self.env.CheckCollision(self.robot, report=report):
            raise CollisionPlanningError.FromReport(report)
        elif self.robot.CheckSelfCollision(report=report):
            raise SelfCollisionPlanningError.FromReport(report)


class BakedRobotCollisionCheckerContextManager(object):
    """RobotCollisionChecker which uses a baked collision interface.

    When this RobotCollisionChecker is instantiated with a robot,
    it interfaces with a collision checker which supports the baking
    interface first implemented in or_fcl.  This takes time on
    initialization to pre-allocate and optimize the underlying
    datastructures, in order to speed up the subsequent
    CheckCollision() calls (e.g. in an inner planner loop).

    Since underlying implementations do not currently return which
    links caused a collision, this checker always returns a basic
    CollisionPlanningError on collision.
    """

    def __init__(self, robot, collision_options=CollisionOptions.ActiveDOFs):
        self.robot = robot
        self.env = robot.GetEnv()

        self.checker = self.env.GetCollisionChecker()
        if self.checker is None:
            raise PrPyException('No collision checker found on environment')

        self.baked_kinbody = None
        self.collision_saver = CollisionOptionsStateSaver(
            self.checker, collision_options)

    def __enter__(self):
        self.collision_saver.__enter__()
        return self

    def __exit__(self, type, value, traceback):
        self.collision_saver.__exit__(type, value, traceback)

    def __enter__(self):
        if self.baked_kinbody is not None:
            raise PrPyException(
                'Another baked KinBody is available. Did you call __enter__'
                ' twice or forget to call __exit__?')

        try:
            kb_type = self.checker.SendCommand('BakeGetType')
        except openrave_exception:
            raise PrPyException('Collision checker does not support baking')

        # TODO: How should we handle exceptions that are thrown below?
        self.collision_saver.__enter__(self)

        # This "bakes" the following Env and Self checks.
        # (after the bake, the composite check is stored in self.baked_kinbody)
        self.checker.SendCommand('BakeBegin')

        self.env.CheckCollision(self.robot)
        self.robot.CheckSelfCollision()

        self.baked_kinbody = RaveCreateKinBody(self.env, kb_type)
        if self.baked_kinbody is None:
            raise PrPyException('Failed to create baked KinBody.')

        self.checker.SendCommand('BakeEnd')

        return self

    def __exit__(self, type, value, traceback):
        if self.baked_kinbody is None:
            raise PrPyException(
                'No baked KinBody is available. Did you call __exit__'
                ' without calling __enter__ first?')

        del self.baked_kinbody
        self.baked_kinbody = None

        self.collision_saver.__exit__(type, value, traceback)

    def CheckCollision(self, report=None):
        if self.baked_kinbody is None:
            raise PrPyException(
                'No baked KinBody is available. Did you call __enter__?')

        # The baked check is performed by checking self collision on baked
        return self.checker.CheckSelfCollision(self.baked_kinbody, report)

    def VerifyCollisionFree(self):
        report = CollisionReport()
        if self.CheckCollision(report=report):
            raise CollisionPlanningError.FromReport(report)


class SimpleRobotCollisionChecker(object):
    def __init__(self, collision_options=CollisionOptions.ActiveDOFs):
        self.collision_options = collision_options

    def __call__(self, robot):
        return SimpleRobotCollisionCheckerContextManager(
            robot, self.collision_options)


class BakedRobotCollisionChecker(object):
    def __init__(self, collision_options=CollisionOptions.ActiveDOFs):
        self.collision_options = collision_options

    def __call__(self, robot):
        return BakedRobotCollisionCheckerContextManager(
            robot, self.collision_options)


DefaultRobotCollisionChecker = SimpleRobotCollisionChecker()
