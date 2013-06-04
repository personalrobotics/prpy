import numpy, openravepy, planning
import herbpy

env, robot = herbpy.initialize(sim=True)
robot.right_arm.SetActive()
robot.planner.PlanToConfiguration(robot, numpy.zeros(7))
#aggregate_planner.PlanToConfiguration(robot, numpy.zeros(7))
