from methods import (
    PlanToConfigurationTest,
    PlanToConfigurationStraightLineTest,
    PlanToConfigurationCompleteTest,
    PlanToEndEffectorPoseTest,
    PlanToEndEffectorOffsetTest,
)
from planning_helpers import BasePlannerTest
from prpy.planning.cbirrt import CBiRRTPlanner
from unittest import TestCase


class CBiRRTPlannerTest(BasePlannerTest,
                        PlanToConfigurationTest,
                        PlanToConfigurationStraightLineTest,
                        PlanToConfigurationCompleteTest,
                        PlanToEndEffectorPoseTest,
                        PlanToEndEffectorOffsetTest,
                        TestCase):
    planner_factory = CBiRRTPlanner
