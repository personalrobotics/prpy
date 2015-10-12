from methods import RetimeTrajectoryTest
from planning_helpers import BasePlannerTest
from prpy.planning.retimer import ParabolicRetimer
from unittest import TestCase

class ParabolicRetimerTests(BasePlannerTest,
                            RetimeTrajectoryTest,
                            TestCase):
    planner_factory = ParabolicRetimer
