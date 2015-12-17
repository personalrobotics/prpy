import openravepy 
from prpy.planning.base import PlanningError
import numpy

class Jumper(object): 

    def JumpToEnd(self, env, robot, method, collision_check=True, **kw_args):
        """ 
        """
        active_dof_indices = kw_args['active_dof_indices']
        assert (active_dof_indices is not None)

        if method == 'PlanToEndEffectorPose':
            goal_pose = kw_args['goal_pose']
            self.JumpToEndEffectorPose(env, robot, 
                                       active_dof_indices,
                                       goal_pose, 
                                       collision_check) 
        elif method == 'PlanToEndEffectorOffset':
            direction = kw_args['direction']
            distance = kw_args['distance']
            self.JumpToEndEffectorOffset(env, robot,
                                         active_dof_indices,
                                         direction, distance, 
                                         collision_check)
        elif method == 'PlanToConfiguration':
            goal_config = kw_args['goal']
            self.JumpToConfiguration(env, robot, active_dof_indices,
                                     goal_config, collision_check)
        elif method == 'PlanToTSR':
            raise Exception('Should call PlanToConfiguration with specific config') 

        else: 
            raise Exception('Cannot find method %s.' % method)


    def JumpToEndEffectorPose(self, env, robot, 
                              active_dof_indices, goal_pose, 
                              collision_check=True):
        with env: 
            ikp = openravepy.IkParameterizationType
            ikfo = openravepy.IkFilterOptions

            if collision_check: 
                check = ikfo.CheckEnvCollisions
            else: 
                check = ikfo.IgnoreSelfCollisions

            # Find an IK solution. 
            manipulator = robot.GetActiveManipulator()
            ik_param = openravepy.IkParameterization(goal_pose, ikp.Transform6D)
            ik_solution = manipulator.FindIKSolution(ik_param, check,
                                                     ikreturn=False, 
                                                     releasegil=True)

            if ik_solution is None: 
                raise PlanningError('Cannot find solution.')
            
            robot.SetActiveDOFs(active_dof_indices)
            robot.SetActiveDOFValues(ik_solution)


    def JumpToEndEffectorOffset(self, env, robot, active_dof_indices,
                                direction, distance,
                                collision_check=True): 
        if distance < 0:
            raise ValueError('Distance must be non-negative.')
        elif numpy.linalg.norm(direction) == 0:
            raise ValueError('Direction must be non-zero')

        # Normalize the direction vector.
        direction = numpy.array(direction, dtype='float')
        direction /= numpy.linalg.norm(direction)

        manipulator = robot.GetActiveManipulator()
        start_pose = manipulator.GetEndEffectorTransform()
        goal_pose = numpy.copy(start_pose)
        goal_pose[0:3, 3] += distance*direction

        self.JumpToEndEffectorPose(env, robot, 
                                   active_dof_indices, 
                                   goal_pose, collision_check)


    def JumpToConfiguration(self, env, robot, active_dof_indices, 
                            goal_config, collision_check=True):
        with env: 
            robot.SetActiveDOFs(active_dof_indices)
            robot.SetActiveDOFValues(goal_config)

            if collision_check:
                if env.CheckCollision(robot) or robot.CheckSelfCollision(): 
                    raise PlanningError('Goal pose in collision')