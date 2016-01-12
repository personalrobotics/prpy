#!/usr/bin/env python
from __future__ import print_function
import herbpy
import numpy
import openravepy
import json
import prpy.serialization
import logging

def ok(b):
  return 'OK' if b else 'FAILED'

env1, robot1 = herbpy.initialize(sim=True)

logging.getLogger('prpy.deserialization').setLevel(logging.DEBUG)
logging.getLogger('prpy.serialization').setLevel(logging.DEBUG)

with prpy.serialization.ReturnTransformQuaternionStateSaver(True):
    # Serialize
    env1_serialized = prpy.serialization.serialize_environment(env1)

    # Deserialize
    env2 = prpy.serialization.deserialize_environment(env1_serialized)
    env2_serialized = prpy.serialization.serialize_environment(env2)
    robot2 = env2.GetRobot(robot1.GetName())

numpy.set_printoptions(precision=3)

with open('env1.json', 'wb') as file1:
    json.dump(env1_serialized, file1, sort_keys=True, indent=2)
    print('robot1_serialized', robot1.GetDOFValues())

with open('env2.json', 'wb') as file2:
    json.dump(env2_serialized, file2, sort_keys=True, indent=2)
    print('robot2_serialized', robot2.GetDOFValues())

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

"""
print('robot.GetJoints()\n', 
    [joint.GetName() for joint in robot1.GetJoints()],
    '?=\n',
    [joint.GetName() for joint in robot2.GetJoints()])
print('  ', ok(
    [joint.GetName() for joint in robot1.GetJoints()]
    == [joint.GetName() for joint in robot2.GetJoints()]))

print('robot.GetLinks()\n', 
    [link.GetName() for link in robot1.GetLinks()],
    '?=\n',
    [link.GetName() for link in robot2.GetLinks()])
print('  ', ok(
    [link.GetName() for link in robot1.GetLinks()]
    == [link.GetName() for link in robot2.GetLinks()]))

SO = openravepy.SerializationOptions.Kinematics
print('robot.serialize(SO.Kinematics)', ok(
    robot1.serialize(SO.Kinematics)
    == robot2.serialize(SO.Kinematics)))
print('robot.serialize(SO.Geometry)', ok(
    robot1.serialize(SO.Geometry)
    == robot2.serialize(SO.Geometry)))

with open('kinematics1.log', 'wb') as f:
    f.write('\n'.join(robot1.serialize(SO.Kinematics).split()))

with open('kinematics2.log', 'wb') as f:
    f.write('\n'.join(robot2.serialize(SO.Kinematics).split()))
"""
