from methods import (
    PlanToConfigurationTest,
    PlanToConfigurationStraightLineTest,
    PlanToConfigurationCompleteTest,
)
from planning_helpers import BasePlannerTest
from prpy.planning.ompl import OMPLPlanner
from unittest import TestCase


class OMPLPlannerTests(BasePlannerTest,
                       PlanToConfigurationTest,
                       PlanToConfigurationStraightLineTest,
                       PlanToConfigurationCompleteTest,
                       TestCase):
    planner_factory = OMPLPlanner
