import numpy

def NoRanking(robot, ik_solutions):
    return numpy.ones(ik_solutions.shape[0])

def JointLimitAvoidance(robot, ik_solutions):
    with robot.GetEnv():
        lower_limits, upper_limits = robot.GetActiveDOFLimits()

    lower_distance = ik_solutions - lower_limits
    upper_distance = upper_limits - ik_solutions
    distance = numpy.minimum(lower_distance, upper_distance)
    return numpy.sum(distance**2, axis=1)
