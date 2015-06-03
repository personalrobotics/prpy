#!/usr/bin/env python
import unittest
from methods import *
from planning_helpers import BasePlannerTest

class CBiRRTPlannerTest(BasePlannerTest,
                        PlanToConfigurationTest,
                        PlanToEndEffectorPoseTest,
                        unittest.TestCase):
    from prpy.planning.cbirrt import CBiRRTPlanner
    planner_factory = CBiRRTPlanner


class OMPLPlannerTests(BasePlannerTest,
                       PlanToConfigurationTest,
                       unittest.TestCase):
    from prpy.planning.ompl import OMPLPlanner
    planner_factory = OMPLPlanner


class OMPLSimplifierTests(BasePlannerTest,
                          ShortcutPathTest,
                          unittest.TestCase):
    from prpy.planning.ompl import OMPLSimplifier
    planner_factory = OMPLSimplifier


class ParabolicRetimerTests(BasePlannerTest,
                            RetimeTrajectoryTest,
                            unittest.TestCase):
    from prpy.planning.retimer import ParabolicRetimer
    planner_factory = ParabolicRetimer


class MacSmootherTests(BasePlannerTest,
                       SmoothTrajectoryTest,
                       unittest.TestCase):
    from prpy.planning.mac_smoother import MacSmoother
    planner_factory = MacSmoother


if __name__ == '__main__':
    import openravepy

    openravepy.RaveInitialize(True)
    openravepy.misc.InitOpenRAVELogging()
    openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)

    unittest.main()
