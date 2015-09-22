from methods import RetimeTrajectoryTest, SmoothTrajectoryTest
from planning_helpers import BasePlannerTest
from prpy.planning.retimer import HauserParabolicSmoother
from unittest import TestCase

# HauserParabolicSmoother should just time the trajectory, without changing the
# geometric path, if do_blend and do_shortcut are False.
class HauserParabolicSmootherRetimingTests(BasePlannerTest,
                                           RetimeTrajectoryTest,
                                           TestCase):
    planner_factory = lambda _: HauserParabolicSmoother(
        do_blend=False, do_shortcut=False)


# Both blending and shortcutting should reduce the duration of the trajectory.
class HauserParabolicSmootherSmoothingTests(BasePlannerTest,
                                            SmoothTrajectoryTest,
                                            TestCase):
    planner_factory = lambda _: HauserParabolicSmoother(
        do_blend=False, do_shortcut=True)


class HauserParabolicSmootherBlendingTests(BasePlannerTest,
                                           SmoothTrajectoryTest,
                                           TestCase):
    planner_factory = lambda _: HauserParabolicSmoother(
        do_blend=True, do_shortcut=False)
