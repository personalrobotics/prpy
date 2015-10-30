from methods import (
    PlanToConfigurationTest,
    PlanToConfigurationStraightLineTest,
    PlanToEndEffectorPoseTest,
    PlanToEndEffectorOffsetTest
)
from methods.PlanToConfiguration import PlanToConfigurationTestCollisionTest
from methods.PlanToEndEffectorOffset import PlanToEndEffectorOffsetCollisionTest
from planning_helpers import BasePlannerTest
from unittest import TestCase
from or_trajopt import TrajoptPlanner


class TrajoptPlannerTest(BasePlannerTest,
                        PlanToConfigurationTest,
                        PlanToEndEffectorPoseTest,
                        PlanToEndEffectorOffsetTest,
                        PlanToEndEffectorOffsetCollisionTest,
                        PlanToConfigurationTestCollisionTest,
                        TestCase):
    planner_factory = TrajoptPlanner

    def setUp(self):
        super(TrajoptPlannerTest, self).setUp()

