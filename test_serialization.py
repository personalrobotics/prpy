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

def expand(s):
  return '\n'.join(s.split())

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

    # Deserialize
    env3 = prpy.serialization.deserialize_environment(env2_serialized)
    env3_serialized = prpy.serialization.serialize_environment(env3)
    robot3 = env3.GetRobot(robot1.GetName())

numpy.set_printoptions(precision=3)

with open('env1.json', 'wb') as file1:
    json.dump(env1_serialized, file1, sort_keys=True, indent=2)
    print('robot1_serialized', robot1.GetDOFValues())

with open('env2.json', 'wb') as file2:
    json.dump(env2_serialized, file2, sort_keys=True, indent=2)
    print('robot2_serialized', robot2.GetDOFValues())

with open('env3.json', 'wb') as file3:
    json.dump(env3_serialized, file3, sort_keys=True, indent=2)
    print('robot3_serialized', robot3.GetDOFValues())

print('BEFORE robot.GetKinematicsGeometryHash()\n',
    robot1.GetKinematicsGeometryHash(), '\n',
    robot2.GetKinematicsGeometryHash())

env2.Remove(robot2)
env2.Add(robot2)

print('robot.GetKinematicsGeometryHash()\n',
    robot1.GetKinematicsGeometryHash(), '\n',
    robot2.GetKinematicsGeometryHash())
print('robot.GetRobotStructureHash()\n',
    robot1.GetRobotStructureHash(), '\n',
    robot2.GetRobotStructureHash())

for manip1 in robot1.GetManipulators():
    manip2 = robot2.GetManipulator(manip1.GetName())
    print('robot.GetManipulator("{:s}").GetKinematicsStructureHash()\n'.format(
        manip1.GetName()),
        manip1.GetKinematicsStructureHash(), '\n',
        manip2.GetKinematicsStructureHash())

print('robot.GetJoints()', ok(
    [joint.GetName() for joint in robot1.GetJoints()]
    == [joint.GetName() for joint in robot2.GetJoints()]))

print('robot.GetLinks()', ok(
    [link.GetName() for link in robot1.GetLinks()]
    == [link.GetName() for link in robot2.GetLinks()]))

SO = openravepy.SerializationOptions
print('robot.serialize(SO.Kinematics)', ok(
    robot1.serialize(SO.Kinematics)
    == robot2.serialize(SO.Kinematics)))
print('robot.serialize(SO.Geometry)', ok(
    robot1.serialize(SO.Geometry)
    == robot2.serialize(SO.Geometry)))
print('robot.serialize(SO.Kinematics | SO.Geometry)', ok(
    robot1.serialize(SO.Kinematics | SO.Geometry)
    == robot2.serialize(SO.Kinematics | SO.Geometry)))

with open('kinematics1.log', 'wb') as f:
    f.write(expand(robot1.serialize(SO.Kinematics)))

with open('kinematics2.log', 'wb') as f:
    f.write(expand(robot2.serialize(SO.Kinematics)))

print('env1.ID =', openravepy.RaveGetEnvironmentId(env1))
print('env2.ID =', openravepy.RaveGetEnvironmentId(env2))
#env1.SetViewer('rviz')
