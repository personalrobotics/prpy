from methods import PlanToEndEffectorOffsetTest
from planning_helpers import BasePlannerTest
from prpy.planning.workspace import GreedyIKPlanner
from unittest import TestCase


class GreedyIKPlannerTest(BasePlannerTest,
                             PlanToEndEffectorOffsetTest,
                             TestCase):
    planner_factory = GreedyIKPlanner 
