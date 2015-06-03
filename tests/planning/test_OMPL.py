from methods import PlanToConfigurationTest
from planning_helpers import BasePlannerTest
from prpy.planning.ompl import OMPLPlanner
from unittest import TestCase


class OMPLPlannerTests(BasePlannerTest,
                       PlanToConfigurationTest,
                       TestCase):
    planner_factory = OMPLPlanner
