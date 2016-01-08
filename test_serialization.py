#!/usr/bin/env python
from __future__ import print_function
import herbpy
import openravepy
import prpy.serialization
import logging

env1, robot1 = herbpy.initialize(sim=True)

logging.getLogger('prpy.deserialization').setLevel(logging.DEBUG)
logging.getLogger('prpy.serialization').setLevel(logging.DEBUG)

# Serialize
env1_serialized = prpy.serialization.serialize_environment(env1)

# Deserialize
env2 = prpy.serialization.deserialize_environment(env1_serialized)
robot2 = env2.GetRobot(robot1.GetName())

print('robot.GetKinematicsGeometryHash()',
    robot1.GetKinematicsGeometryHash(), '?=',
    robot2.GetKinematicsGeometryHash())
print('robot.GetRobotStructureHash()',
    robot1.GetRobotStructureHash(), '?=',
    robot2.GetRobotStructureHash())

for manip1 in robot1.GetManipulators():
    manip2 = robot2.GetManipulator(manip1.GetName())
    print('robot.GetManipulator("{:s}").GetKinematicsStructureHash()'.format(
        manip1.GetName()),
        manip1.GetKinematicsStructureHash(), '?=',
        manip2.GetKinematicsStructureHash())

