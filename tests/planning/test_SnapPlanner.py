from methods import (
    PlanToConfigurationTest,
    PlanToConfigurationStraightLineTest,
)
from planning_helpers import BasePlannerTest
from prpy.planning.snap import SnapPlanner
from unittest import TestCase


class SnapPlannerTest(BasePlannerTest,
                      PlanToConfigurationTest,
                      PlanToConfigurationStraightLineTest,
                      TestCase):
    planner_factory = SnapPlanner

    def setUp(self):
        super(SnapPlannerTest, self).setUp()

