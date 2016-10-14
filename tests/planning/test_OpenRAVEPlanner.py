#!/usr/bin/env python
from methods import (
    PlanToConfigurationTest,
    PlanToConfigurationStraightLineTest
)
from planning_helpers import BasePlannerTest
from methods.PlanToConfiguration import PlanToConfigurationTestCollisionTest
from prpy.planning.openrave import OpenRAVEPlanner
from unittest import TestCase


class OpenRAVEPlannerTest(BasePlannerTest,
                          PlanToConfigurationTest,
                          TestCase):
    planner_factory = OpenRAVEPlanner
