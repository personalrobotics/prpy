#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Pras Velagapudi <pkv@cs.cmu.edu>
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
import logging
import time
import itertools
import numpy
import openravepy
from base import (BasePlanner, ClonedPlanningMethod, PlanningError,
                  UnsupportedPlanningError)
from ..collision import SimpleRobotCollisionChecker
from openravepy import (
    CollisionOptions,
    CollisionOptionsStateSaver,
    IkFilterOptions,
    IkParameterization,
    IkParameterizationType
)

logger = logging.getLogger(__name__)


# Source: http://stackoverflow.com/a/8991553/111426
def grouper(n, iterable):
    it = iter(iterable)
    while True:
       chunk = tuple(itertools.islice(it, n))
       if not chunk:
           return
       yield chunk



class TSRPlanner(BasePlanner):
    def __init__(self, delegate_planner=None, robot_collision_checker=SimpleRobotCollisionChecker):
        super(TSRPlanner, self).__init__()
        self.delegate_planner = delegate_planner
        self.robot_collision_checker = robot_collision_checker

    def __str__(self):
        if self.delegate_planner is not None:
            return 'TSRPlanner({:s})'.format(str(self.delegate_planner))
        else:
            return 'TSRPlanner'

    # TODO: Temporarily disabled based on Pras' feedback.
    """
    @ClonedPlanningMethod
    def PlanToIK(self, robot, goal_pose, **kw_args):
        from ..tsr import TSR, TSRChain

        tsr = TSR(T0_w=goal_pose, Tw_e=numpy.eye(4), Bw=numpy.zeros((6,2)))
        tsr_chain = TSRChain(sample_goal=True, TSR=tsr)

        return self.PlanToTSR(robot, [tsr_chain], **kw_args)
    """

    @ClonedPlanningMethod
    def PlanToTSR(self, robot, tsrchains, tsr_timeout=0.5,
                  num_attempts=3, chunk_size=1, num_candidates=50, ranker=None,
                  max_deviation=2 * numpy.pi, **kw_args):
        """
        Plan to a desired TSR set using a-priori goal sampling.  This planner
        samples a fixed number of goals from the specified TSRs up-front, then
        uses another planner's PlanToConfiguration (chunk_size = 1) or
        PlanToConfigurations (chunk_size > 1) to plan to the resulting affine
        transformations.

        This planner will return failure if the provided TSR chains require
        any constraint other than goal sampling.

        @param robot the robot whose active manipulator will be used
        @param tsrchains a list of TSR chains that define a goal set
        @param num_attempts the maximum number of planning attempts to make
        @param num_candidates the number of candidate IK solutions to rank
        @param chunk_size the number of sampled goals to use per planning call
        @param tsr_timeout the maximum time to spend sampling goal TSR chains
        @param ranker an IK ranking function to use over the IK solutions
        @param max_deviation the maximum per-joint deviation from current pose
                             that can be considered a valid sample.
        @return traj a trajectory that satisfies the specified TSR chains
        """
        # Delegate to robot.planner by default.
        delegate_planner = self.delegate_planner or robot.planner

        # Plan using the active manipulator.
        manipulator = robot.GetActiveManipulator()

        # Distance from current configuration is default ranking.
        if ranker is None:
            from ..ik_ranking import NominalConfiguration
            ranker = NominalConfiguration(manipulator.GetArmDOFValues(),
                                          max_deviation=max_deviation)

        # Test for tsrchains that cannot be handled.
        for tsrchain in tsrchains:
            if tsrchain.sample_start or tsrchain.constrain:
                raise UnsupportedPlanningError(
                    'Cannot handle start or trajectory-wide TSR constraints.')
        tsrchains = [t for t in tsrchains if t.sample_goal]

        def compute_ik_solutions(tsrchain):
            pose = tsrchain.sample()
            ik_param = IkParameterization(pose,
                IkParameterizationType.Transform6D)
            ik_solutions = manipulator.FindIKSolutions(
                ik_param, IkFilterOptions.IgnoreSelfCollisions,
                ikreturn=False, releasegil=True)

            statistics['num_tsr_samples'] += 1
            statistics['num_ik_solutions'] += ik_solutions.shape[0]

            return ik_solutions

        # Instantiate a robot checker
        with CollisionOptionsStateSaver(
                self.env.GetCollisionChecker(), CollisionOptions.ActiveDOFs):
            robot_checker = self.robot_collision_checker(robot)

        def is_configuration_valid(ik_solution):
            p = openravepy.KinBody.SaveParameters
            with robot.CreateRobotStateSaver(p.LinkTransformation):
                robot.SetActiveDOFValues(ik_solution)
                return not robot_checker.CheckCollision()

        def is_time_available(*args):
            # time_start and time_expired are defined below.
            return time.time() - time_start + time_expired < tsr_timeout

        time_expired = 0.
        statistics = {
            'num_tsr_samples': 0,
            'num_ik_solutions': 0
        }

        # We assume the active manipulator's DOFs are active when computing IK,
        # calling the delegate planners, and collision checking with the
        # ActiveDOFs option set.
        robot.SetActiveDOFs(manipulator.GetArmIndices())

        configuration_generator = itertools.chain.from_iterable(
            itertools.ifilter(
                lambda configurations: configurations.shape[0] > 0,
                itertools.imap(compute_ik_solutions,
                    itertools.takewhile(
                        is_time_available,
                        itertools.cycle(tsrchains)))))

        for iattempt in xrange(num_attempts):
            configurations_chunk = []
            time_start = time.time()

            # Set ActiveDOFs for IK collision checking. We intentionally
            # restore the original collision checking options before calling
            # the planner to give it a pristine environment.
            with CollisionOptionsStateSaver(
                    self.env.GetCollisionChecker(), CollisionOptions.ActiveDOFs):
                while is_time_available() and len(configurations_chunk) < chunk_size:
                    # Generate num_candidates candidates and rank them using the
                    # user-supplied IK ranker.
                    candidates = list(itertools.islice(configuration_generator, num_candidates))
                    if not candidates:
                        break

                    candidates_scores = ranker(robot, numpy.array(candidates))
                    candidates_scored = zip(candidates_scores, candidates)
                    candidates_scored.sort(key=lambda (score, _): score)
                    candidates_ranked = [q for _, q in candidates_scored]

                    # Select valid IK solutions from the chunk.
                    candidates_valid = itertools.islice(
                        itertools.ifilter(
                            is_configuration_valid,
                            itertools.takewhile(
                                is_time_available,
                                candidates_ranked)),
                        chunk_size - len(configurations_chunk))
                    configurations_chunk.extend(candidates_valid)

            time_expired += time.time() - time_start

            if len(configurations_chunk) == 0:
                raise PlanningError(
                    'Reached TSR sampling timelimit on attempt {:d} of {:d}: Failed'
                    ' to generate any collision free IK solutions after attempting'
                    ' {:d} TSR samples with {:d} candidate IK solutions.'.format(
                        iattempt + 1, num_attempts,
                        statistics['num_tsr_samples'], statistics['num_ik_solutions']))
            elif len(configurations_chunk) < chunk_size:
                logger.warning(
                    'Reached TSR sampling timelimit on attempt %d of %d: got %d'
                    ' of %d IK solutions.',
                    iattempt + 1, num_attempts,
                    len(configurations_chunk), chunk_size)

            try:
                logger.info('Planning attempt %d of %d to a set of %d IK solution(s).',
                    iattempt + 1, num_attempts, len(configurations_chunk))

                if chunk_size == 1:
                    traj = delegate_planner.PlanToConfiguration(
                        robot, configurations_chunk[0], **kw_args)
                else:
                    traj = delegate_planner.PlanToConfigurations(
                        robot, configurations_chunk, **kw_args)

                return traj
            except PlanningError as e:
                logger.warning('Planning attempt %d of %d failed: %s',
                    iattempt + 1, num_attempts, e)

        raise PlanningError('Failed to find a solution in {:d} attempts.'.format(
            iattempt + 1))
