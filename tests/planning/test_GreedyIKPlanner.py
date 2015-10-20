from methods import PlanToEndEffectorOffsetTest
from methods.PlanToEndEffectorOffset import PlanToEndEffectorOffsetCollisionTest
from planning_helpers import BasePlannerTest
from prpy.planning.workspace import GreedyIKPlanner
from unittest import TestCase


class GreedyIKPlannerTest(BasePlannerTest,
                             PlanToEndEffectorOffsetTest,
                             PlanToEndEffectorOffsetCollisionTest,
                             TestCase):
    planner_factory = GreedyIKPlanner 
