from methods import (
    PlanToConfigurationTest,
    PlanToConfigurationStraightLineTest,
)
from methods.PlanToConfiguration import PlanToConfigurationTestCollisionTest
from planning_helpers import BasePlannerTest
from prpy.planning.snap import SnapPlanner
from unittest import TestCase


class SnapPlannerTest(BasePlannerTest,
                      PlanToConfigurationTest,
                      PlanToConfigurationStraightLineTest,
                      PlanToConfigurationTestCollisionTest,
                      TestCase):
    planner_factory = SnapPlanner

    def setUp(self):
        super(SnapPlannerTest, self).setUp()

