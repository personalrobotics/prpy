from methods import PlanToEndEffectorOffsetTest
from methods.PlanToEndEffectorOffset import PlanToEndEffectorOffsetCollisionTest
from planning_helpers import BasePlannerTest
from prpy.planning.vectorfield import VectorFieldPlanner
from unittest import TestCase


class VectorFieldPlannerTest(BasePlannerTest,
                             PlanToEndEffectorOffsetTest,
                             PlanToEndEffectorOffsetCollisionTest,
                             TestCase):
    planner_factory = VectorFieldPlanner
