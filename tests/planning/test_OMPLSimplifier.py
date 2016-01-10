from methods import ShortcutPathTest
from planning_helpers import BasePlannerTest
from prpy.planning.ompl import OMPLSimplifier
from unittest import TestCase


class OMPLSimplifierTests(BasePlannerTest,
                          ShortcutPathTest,
                          TestCase):
    planner_factory = OMPLSimplifier
