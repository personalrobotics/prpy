#!/usr/bin/env python
from methods import (
    PlanToConfigurationTest,
)
from planning_helpers import BasePlannerTest
from prpy.planning.openrave import OpenRAVEPlanner
from unittest import TestCase


class OpenRAVEPlannerTest(BasePlannerTest,
                          PlanToConfigurationTest,
                          TestCase):
    planner_factory = OpenRAVEPlanner
