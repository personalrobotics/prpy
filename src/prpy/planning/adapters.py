import numpy
from openravepy import Environment
from .base import (
    LockedPlanningMethod,
    Planner,
)
from ..kin import (
    H_from_op_diff,
    invert_H,
)
from tsr.tsr import (
    TSR,
    TSRChain,
)
from ..util import GetManipulatorIndex


class PlanToEndEffectorOffsetTSRAdapter(Planner):
    epsilon = 1e-3


    def __init__(self, delegate_planner):
        """ Plan PlanToEndEffectorOffset using PlanToTSR on a delegate planner.

        @param delegate_planner planner that supports goal and constraint TSRs
        """
        super(PlanToEndEffectorOffsetTSRAdapter, self).__init__()

        self.delegate_planner = delegate_planner

    @LockedPlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance, **kwargs):
        """ Plan to a desired end-effector offset with fixed orientation.

        This function converts a PlanToEndEffectorOffset query into: (1) a goal
        TSR at "distance" away from the start pose along "direction" and (2) a
        constraint TSR that keeps the end-effector's position constrained to
        that line segment with fixed orientation. Those TSRs are forwarded to
        PlanToTSR on the delegate planner.

        @param robot 
        @param direction unit vector in the world frame
        @param distance distance to move, in meters
        @param **kwargs arguments forwarded to the delegate planner
        @return output trajectory
        """
        chains = self.CreateTSRChains(robot, direction, distance)
        return self.delegate_planner.PlanToTSR(robot, chains, **kwargs)


    @classmethod
    def CreateTSRChains(cls, robot, direction, distance):
        direction = numpy.array(direction, dtype=float)

        if direction.shape != (3,):
            raise ValueError('Direction must be a three-dimensional vector.')

        if not (distance >= 0):
            raise ValueError('Distance must be non-negative; got {:f}.'.format(
                             distance))


        manip, manip_index = GetManipulatorIndex(robot)
        H_world_ee = manip.GetEndEffectorTransform()
        direction = direction / numpy.linalg.norm(direction)

        # 'object frame w' is at ee, z pointed along direction to move
        H_world_w = H_from_op_diff(H_world_ee[0:3, 3], direction)
        H_w_ee = numpy.dot(invert_H(H_world_w), H_world_ee)
        Hw_end = numpy.eye(4)
        Hw_end[2, 3] = distance

        goal_chain = TSRChain(sample_goal=True, TSR=TSR(
            T0_w=numpy.dot(H_world_w, Hw_end),
            Tw_e=H_w_ee,
            Bw=numpy.zeros((6, 2)),
            manipindex=manip_index))
        constraint_chain = TSRChain(constrain=True, TSR=TSR(
            T0_w=H_world_w,
            Tw_e=H_w_ee,
            Bw=numpy.array([
                [-cls.epsilon, cls.epsilon],
                [-cls.epsilon, cls.epsilon],
                [min(0.0, distance), max(0.0, distance)],
                [-cls.epsilon, cls.epsilon],
                [-cls.epsilon, cls.epsilon],
                [-cls.epsilon, cls.epsilon]]),
            manipindex=manip_index))

        return [goal_chain, constraint_chain]
