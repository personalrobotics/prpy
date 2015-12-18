
import openravepy 
from prpy.planning.base import PlanningError
import numpy

class IKSampler(object): 

    def __init__(self, env, robot, manipulator): 
        self.env = env
        self.robot = robot 
        self.manipulator = manipulator 

    def FindIKSamples(self, method, num_samples=5, collision_check=True, **kw_args): 

        if method == 'PlanToEndEffectorPose':
            goal_pose = kw_args['goal_pose']
            return self.FindIKSamplesForEndEffectorPose(goal_pose, num_samples, collision_check) 

        elif method == 'PlanToEndEffectorOffset':
            direction = kw_args['direction']
            distance = kw_args['distance']
            return self.FindIKSamplesForEndEffectorOffset(direction, distance, num_samples, collision_check)

        elif method == 'PlanToConfiguration':
            active_dof_indices = kw_args['active_dof_indices']
            return self.FindIKSamplesForConfiguration(active_dof_indices, goal_config, collision_check)

        elif method == 'PlanToTSR':
            raise Exception('Call FindIKSamplesForTSR directly.')

        else: 
            raise Exception('Cannot find method %s.' % method)


    def FindIKSamplesForEndEffectorPose(self, goal_pose, 
                                        num_samples=5, collision_check=True):
        """ Returns IK solutions satisfying EndEffector pose constraint
        @param num_samples Maximum number of samples it would return 
        @throws PlanningError if no solution can be found 
        """

        with self.env: 
            ikp = openravepy.IkParameterizationType
            ikfo = openravepy.IkFilterOptions

            if collision_check: 
                check = ikfo.CheckEnvCollisions
            else: 
                check = ikfo.IgnoreSelfCollisions

            # Find an IK solution. 
            ik_param = openravepy.IkParameterization(goal_pose, ikp.Transform6D)
            ik_solutions = self.manipulator.FindIKSolutions(ik_param, check,
                                                       ikreturn=False, 
                                                       releasegil=True)
            if len(ik_solutions) == 0: 
                raise PlanningError('Cannot find solution.')

            return ik_solutions[0:min(len(ik_solutions), num_samples)]


    def FindIKSamplesForEndEffectorOffset(self, direction, distance,
                                          num_samples=5, collision_check=True): 
        # It sets goal pose to be current+distance*direction, but shouldn't we 
        # move until we hit something? ... 

        if distance < 0:
            raise ValueError('Distance must be non-negative.')
        elif numpy.linalg.norm(direction) == 0:
            raise ValueError('Direction must be non-zero')

        # Normalize the direction vector.
        direction = numpy.array(direction, dtype='float')
        direction /= numpy.linalg.norm(direction)

        start_pose = self.manipulator.GetEndEffectorTransform()
        goal_pose = numpy.copy(start_pose)
        goal_pose[0:3, 3] += distance*direction

        return self.FindIKSamplesForEndEffectorPose(goal_pose, num_samples, collision_check)


    def FindIKSamplesForConfiguration(self, active_indices, goal_config, collision_check=True): 
        """ Returns goal_config is there is no collision """ 

        p = openravepy.KinBody.SaveParameters 
        robot = self.robot 
        env = self.env 

        with env: 
            with robot.CreateRobotStateSaver(p.LinkTransformation):
                robot.SetActiveDOFs(active_indices)
                robot.SetActiveDOFValues(goal_config)

                if collision_check:
                    if env.CheckCollision(robot) or robot.CheckSelfCollision(): 
                        raise PlanningError('Goal config in collision')

        return [goal_config]


    def FindIKSamplesForTSR(self, obj, tsrchains, num_samples=5, collision_check=True, **kw_args): 
        # Mostly from prpy.planning.TSRPlanner

        import itertools
        import time

        robot = self.robot 
        manipulator = self.manipulator 
        env = self.env 

        with robot.CreateRobotStateSaver(
                Robot.SaveParameters.ActiveDOF
              | Robot.SaveParameters.ActiveManipulator):

            from prpy.ik_ranking import NominalConfiguration
            ranker = NominalConfiguration(manipulator.GetArmDOFValues())

            # Test for tsrchains that cannot be handled.
            for tsrchain in tsrchains:
                if tsrchain.sample_start or tsrchain.constrain:
                    raise UnsupportedPlanningError(
                        'Cannot handle start or trajectory-wide TSR constraints.')
            tsrchains = [t for t in tsrchains if t.sample_goal]

            # Create an iterator that cycles through each TSR chain.
            tsr_cycler = itertools.cycle(tsrchains)

            # Create an iterator that cycles TSR chains until the timelimit.
            tsr_timeout=2.0
            tsr_timelimit = time.time() + tsr_timeout
            tsr_sampler = itertools.takewhile(
                lambda v: time.time() < tsr_timelimit, tsr_cycler)

            # Sample a list of TSR poses and collate valid IK solutions.
            from openravepy import (IkFilterOptions,
                                    IkParameterization,
                                    IkParameterizationType)
            ik_solutions = []
            for tsrchain in tsr_sampler:
                ik_param = IkParameterization(
                    tsrchain.sample(), IkParameterizationType.Transform6D)
                ik_solution = manipulator.FindIKSolutions(
                    ik_param, IkFilterOptions.CheckEnvCollisions,
                    ikreturn=False, releasegil=True
                )
                if ik_solution.shape[0] > 0:
                    ik_solutions.append(ik_solution)

            if len(ik_solutions) == 0:
                raise PlanningError('No collision-free IK solutions at goal TSRs.')

            # Sort the IK solutions in ascending order by the costs returned by the
            # ranker. Lower cost solutions are better and infinite cost solutions
            # are assumed to be infeasible.
            ik_solutions = numpy.vstack(ik_solutions)
            scores = ranker(robot, ik_solutions)
            valid_idxs = ~numpy.isposinf(scores)
            valid_scores = scores[valid_idxs]
            valid_solutions = ik_solutions[valid_idxs, :]
            ranked_indices = numpy.argsort(valid_scores)
            ranked_ik_solutions = valid_solutions[ranked_indices, :]

            return ranked_ik_solutions[0:min(len(ranked_ik_solutions), num_samples)]
