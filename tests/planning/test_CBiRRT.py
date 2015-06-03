from methods import (
    PlanToConfigurationTest,
    PlanToEndEffectorPoseTest
)
from planning_helpers import BasePlannerTest
from prpy.planning.cbirrt import CBiRRTPlanner
from unittest import TestCase


class CBiRRTPlannerTest(BasePlannerTest,
                        PlanToConfigurationTest,
                        PlanToEndEffectorPoseTest,
                        TestCase):
    planner_factory = CBiRRTPlanner
