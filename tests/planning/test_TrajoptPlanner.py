from methods import (
    PlanToConfigurationTest,
    PlanToConfigurationStraightLineTest,
    PlanToEndEffectorPoseTest,
)
from methods.PlanToConfiguration import PlanToConfigurationTestCollisionTest
from planning_helpers import BasePlannerTest
from prpy.planning.snap import SnapPlanner
from unittest import TestCase
from or_trajopt import TrajoptPlanner

class TrajoptPlannerTest(BasePlannerTest,
                         PlanToConfigurationTest,
                         PlanToConfigurationStraightLineTest,
                         PlanToEndEffectorPoseTest,
                         TestCase):
    planner_factory = TrajoptPlanner

    def setUp(self):
        super(TrajoptPlannerTest, self).setUp()
