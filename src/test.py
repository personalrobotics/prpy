import numpy, openravepy, planning
import herbpy

env, robot = herbpy.initialize(sim=True)
robot.right_arm.SetActive()

chomp = planning.CHOMPPlanner(env)
#chomp.ComputeDistanceField(robot)

aggregate_planner = planning.Sequence(chomp, chomp)
aggregate_planner.PlanToConfiguration(robot, numpy.zeros(7))
#aggregate_planner.PlanToConfiguration(robot, numpy.zeros(7))
