from methods import PlanToEndEffectorOffsetTest
from planning_helpers import BasePlannerTest
from prpy.planning.vectorfield import VectorFieldPlanner
from unittest import TestCase


class VectorFieldPlannerTest(BasePlannerTest,
                             PlanToEndEffectorOffsetTest,
                             TestCase):
    planner_factory = VectorFieldPlanner
