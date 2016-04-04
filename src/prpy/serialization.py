import json
import logging
import numpy
import openravepy
from numpy import array, ndarray
from .exceptions import (
    UnsupportedTypeSerializationException,
    UnsupportedTypeDeserializationException,
)
from .tsr import (
    TSR,
    TSRChain,
)
from openravepy import (
    Environment,
    KinBody,
    GeometryType,
    RaveCreateKinBody,
    RaveCreateRobot,
    RaveCreateTrajectory,
    Robot,
    Trajectory,
)

TYPE_KEY = '__type__'

serialization_logger = logging.getLogger('prpy.serialization')
deserialization_logger = logging.getLogger('prpy.deserialization')


class ReturnTransformQuaternionStateSaver(object):
    def __init__(self, value):
        self.desired_value = value

    def __enter__(self):
        self.original_value = openravepy.options.returnTransformQuaternion
        openravepy.options.returnTransformQuaternion = self.desired_value

    def __exit__(self, exc_type, exc_value, traceback):
        openravepy.options.returnTransformQuaternion = self.original_value


# Serialization.
def serialize(obj, database=None):
    """
    Serialize an object
    @param obj The object to serialize
    @param database A SerializationDatabase object. If not None,
    all filenames will be serialized through this database. Otherwise,
    full filepaths will be saved into the serialized object
    """
    NoneType = type(None)

    if isinstance(obj, (int, float, basestring, NoneType)):
        return obj
    elif isinstance(obj, (list, tuple)):
        return [ serialize(x, database) for x in obj ]
    elif isinstance(obj, dict):
        obj = {serialize(k, database): serialize(v, database)
                 for k, v in obj.iteritems()}
        obj[TYPE_KEY] = dict.__name__
        return obj
    elif isinstance(obj, ndarray):
        return {
            TYPE_KEY: ndarray.__name__,
            'data': serialize(obj.tolist(), database)
        }
    elif isinstance(obj, Environment):
        return {
            TYPE_KEY: Environment.__name__,
            'data': serialize_environment(obj, database)
        }
    elif isinstance(obj, KinBody):
        return {
            TYPE_KEY: KinBody.__name__,
            'name': obj.GetName()
        }
    elif isinstance(obj, Robot):
        return {
            TYPE_KEY: Robot.__name__,
            'name': obj.GetName()
        }
    elif isinstance(obj, KinBody.Link):
        return {
            TYPE_KEY: KinBody.Link.__name__,
            'name': obj.GetName(),
            'parent_name': obj.GetParent().GetName()
        }
    elif isinstance(obj, KinBody.Joint):
        return {
            TYPE_KEY: KinBody.Joint.__name__,
            'name': obj.GetName(),
            'parent_name': obj.GetParent().GetName()
        }
    elif isinstance(obj, Robot.Manipulator):
        return {
            TYPE_KEY: KinBody.Manipulator.__name__,
            'name': obj.GetName(),
            'parent_name': obj.GetParent().GetName()
        }
    elif isinstance(obj, Trajectory):
        return {
            TYPE_KEY: Trajectory.__name__,
            'data': obj.serialize(0)
        }
    elif isinstance(obj, TSR):
        return {
            TYPE_KEY: TSR.__name__,
            'data': obj.to_dict()
        }
    elif isinstance(obj, TSRChain):
        return {
            TYPE_KEY: TSRChain.__name__,
            'data': obj.to_dict()
        }
    else:
        raise UnsupportedTypeSerializationException(obj)

def serialize_environment(env, database=None):
    """
    Serialize all bodies in the environment.
    @param env The OpenRAVE environment
    @param database A SerializationDatabase object. If not None,
    all filenames will be serialized through this database. Otherwise,
    full filepaths will be saved into the serialized object
    """
    return {
        'bodies': [ serialize_kinbody(body, database) for body in env.GetBodies() ],
    }

def serialize_environment_file(env, path, writer=None, database=None):
    """
    Serialize the environment and save the resulting serialization
    to the given file
    @param env The OpenRAVE environment
    @param path The path to save the serialized environment to
    @param writer The writer to use to save the serialized environment to file 
    (default: json.dump)
    @param database A SerializationDatabase object. If not None,
    all filenames will be serialized through this database. Otherwise,
    full filepaths will be saved into the serialized object
    """
    if writer is None:
        writer = json.dump

    data = serialize_environment(env, database)

    if path is not None:
        with open(path, 'wb') as output_file:
            writer(data, output_file)
            serialization_logger.debug('Wrote environment to "%s".', path)

    return data

def serialize_kinbody(body, database):
    """
    Serialize an object
    @param obj The object to serialize
    @param database A SerializationDatabase object. If not None,
    all filenames will be serialized through this database. Otherwise,
    full filepaths will be saved into the serialized object
    """
    LinkTransformation = openravepy.KinBody.SaveParameters.LinkTransformation

    with body.CreateKinBodyStateSaver(LinkTransformation):
        body.SetDOFValues([0] * body.GetDOF(), numpy.arange(body.GetDOF()),
            openravepy.KinBody.CheckLimitsAction.Nothing)

        all_joints = []
        all_joints.extend(body.GetJoints())
        all_joints.extend(body.GetPassiveJoints())
        
        data = {
            'is_robot': body.IsRobot(),
            'name': body.GetName(),
            'uri': serialize_path(body.GetURI(), database),
            'links': [serialize_link(l, database) for l in body.GetLinks()],
            'joints': [serialize_joint(j, database) for j in all_joints]
        }

        if body.IsRobot():
            data.update(serialize_robot(body, database))

    data['kinbody_state'] = serialize_kinbody_state(body)

    return data

def serialize_robot(robot, database):
    return {
        'manipulators': [serialize_manipulator(m, database) for m in robot.GetManipulators()],
        'robot_state': serialize_robot_state(robot),
    }

def serialize_kinbody_state(body):
    data = {
        name: get_fn(body)
        for name, (get_fn, _) in KINBODY_STATE_MAP.iteritems()
    }

    link_transforms, dof_branches = body.GetLinkTransformations(True)
    data.update({
        'link_transforms': map(serialize_transform, link_transforms),
        'dof_branches': dof_branches.tolist(),
    })

    return data

def serialize_robot_state(body):
    data = {
        name: get_fn(body)
        for name, (get_fn, _) in ROBOT_STATE_MAP.iteritems()
    }
    data['grabbed_bodies'] = map(serialize_grabbed_info, body.GetGrabbedInfo())
    return data

def serialize_link(link, database):
    data = { 'info': serialize_link_info(link.UpdateAndGetInfo(), database) }

    # Bodies loaded from ".kinbody.xml" do not have GeometryInfo's listed in
    # their LinkInfo class. We manually read them from GetGeometries().
    # TODO: This may not correctly preserve non-active geometry groups.
    data['info']['_vgeometryinfos'] = [
        serialize_geometry_info(geometry.GetInfo(), database) \
        for geometry in link.GetGeometries()
    ]
    return data

def serialize_joint(joint, database):
    # joint.UpdateInfo() applies an incorrect offset to the joint. This seems
    # to be a bug - DO NOT call it.
    return { 'info': serialize_joint_info(joint.GetInfo(), database) }

def serialize_manipulator(manipulator, database):
    return { 'info': serialize_manipulator_info(manipulator.GetInfo(), database) }

def serialize_with_map(obj, database, attribute_map):
    return {
        key: serialize_fn(getattr(obj, key), database)
        for key, (serialize_fn, _) in attribute_map.iteritems()
    }

def serialize_link_info(link_info, database):
    return serialize_with_map(link_info, database, LINK_INFO_MAP)
    
def serialize_joint_info(joint_info, database):
    return serialize_with_map(joint_info, database, JOINT_INFO_MAP)

def serialize_manipulator_info(manip_info, database):
    return serialize_with_map(manip_info, database, MANIPULATOR_INFO_MAP)

def serialize_geometry_info(geom_info, database):
    return serialize_with_map(geom_info, database, GEOMETRY_INFO_MAP)

def serialize_grabbed_info(grabbed_info, database):
    return serialize_with_map(grabbed_info, database, GRABBED_INFO_MAP)

def serialize_transform(t):
    with ReturnTransformQuaternionStateSaver(True):
        return t.tolist()

def serialize_path(path, database):
    import os, openravepy
    path = path if os.path.exists(path) else openravepy.RaveFindLocalFile(path)
    if database and os.path.exists(path):
        return database.save(path)
    return path

def deserialize_path(path, database):
    if database:
        try:
            return database.get_path(path)
        except IOError:
            return path
    return path

# Deserialization.
def _deserialize_internal(env, data, data_type, database):
    if data_type == dict.__name__:
        return {
            deserialize(env, k, database): deserialize(env, v, database)
            for k, v in data.iteritems()
            if k != TYPE_KEY
        }
    elif data_type == ndarray.__name__:
        return array(data['data'])
    elif data_type in [ KinBody.__name__, Robot.__name__ ]:
        body = env.GetKinBody(data['name'])
        if body is None:
            raise ValueError('There is no body with name "{:s}".'.format(
                data['name']))

        return body
    elif data_type == KinBody.Link.__name__:
        body = env.GetKinBody(data['parent_name'])
        if body is None:
            raise ValueError('There is no body with name "{:s}".'.format(
                data['parent_name']))

        link = body.GetLink(data['name'])
        if link is None:
            raise ValueError('Body "{:s}" has no link named "{:s}".'.format(
                data['parent_name'], data['name']))

        return link
    elif data_type == KinBody.Joint.__name__:
        body = env.GetKinBody(data['parent_name'])
        if body is None:
            raise ValueError('There is no body with name "{:s}".'.format(
                data['parent_name']))

        joint = body.GetJoint(data['name'])
        if joint is None:
            raise ValueError('Body "{:s}" has no joint named "{:s}".'.format(
                data['parent_name'], data['name']))

        return joint
    elif data_type == Robot.Manipulator.__name__:
        body = env.GetKinBody(data['parent_name'])
        if body is None:
            raise ValueError('There is no robot with name "{:s}".'.format(
                data['parent_name']))
        elif not body.IsRobot():
            raise ValueError('Body "{:s}" is not a robot.'.format(
                data['parent_name']))

        manip = body.GetJoint(data['name'])
        if manip is None:
            raise ValueError('Robot "{:s}" has no manipulator named "{:s}".'.format(
                data['parent_name'], data['name']))

        return manip
    elif data_type == Trajectory.__name__:
        traj = RaveCreateTrajectory(env, '')
        traj.deserialize(data['data'])
        return traj
    elif data_type == TSR.__name__:
        return TSR.from_dict(data['data'])
    elif data_type == TSRChain.__name__:
        return TSRChain.from_dict(data['data'])
    else:
        raise UnsupportedTypeDeserializationException(data_type)

def deserialize(env, data, database=None):
    if isinstance(data, unicode):
        return data.encode()
    elif isinstance(data, list):
        return [ deserialize(env, x, database) for x in data ]
    elif isinstance(data, dict):
        return _deserialize_internal(env, data, data.get(TYPE_KEY), database)
    else:
        return data

def deserialize_environment(data, env=None, purge=False, reuse_bodies=None,
                            database=None):
    if env is None:
        env = Environment()

    if reuse_bodies is None:
        reuse_bodies_dict = dict()
        reuse_bodies_set = set()
    else:
        reuse_bodies_dict = { body.GetName(): body for body in reuse_bodies }
        reuse_bodies_set = set(reuse_bodies)

    # Release anything that's grabbed.
    for body in reuse_bodies_set:
        body.ReleaseAllGrabbed()

    # Remove any extra bodies from the environment.
    for body in env.GetBodies():
        if body not in reuse_bodies_set:
            deserialization_logger.debug('Purging body "%s".', body.GetName())
            env.Remove(body)

    # Deserialize the kinematic structure.
    deserialized_bodies = []
    for body_data in data['bodies']:
        body = reuse_bodies_dict.get(body_data['name'], None)
        if body is None:
            body = deserialize_kinbody(env, body_data, state=False,
                database=database)

        deserialization_logger.debug('Deserialized body "%s".', body.GetName())
        deserialized_bodies.append((body, body_data))

    # Restore state. We do this in a second pass to insure that any bodies that
    # are grabbed already exist.
    for body, body_data in deserialized_bodies:
        deserialize_kinbody_state(body, body_data['kinbody_state'], database)

        if body.IsRobot():
            deserialize_robot_state(body, body_data['robot_state'], database)

    return env

def deserialize_kinbody(env, data, name=None, anonymous=False, state=True,
                        database=None):
    deserialization_logger.debug('Deserializing %s "%s".',
        'Robot' if data['is_robot'] else 'KinBody',
        data['name']
    )

    link_infos = [
        deserialize_link_info(link_data['info'], database)
        for link_data in data['links']
    ]
    joint_infos = [
        deserialize_joint_info(joint_data['info'], database)
        for joint_data in data['joints']
    ]

    if data['is_robot']:
        # TODO: Also load sensors.
        manipulator_infos = [
            deserialize_manipulator_info(manipulator_data['info'], database)
            for manipulator_data in data['manipulators']
        ]
        sensor_infos = []

        kinbody = RaveCreateRobot(env, '')
        kinbody.Init(
            link_infos, joint_infos,
            manipulator_infos, sensor_infos,
            data['uri']
        )
    else:
        kinbody = RaveCreateKinBody(env, '')
        kinbody.Init(link_infos, joint_infos, data['uri'])

    kinbody.SetName(name or data['name'])
    env.Add(kinbody, anonymous)

    if state:
        deserialize_kinbody_state(kinbody, data['kinbody_state'], database)
        if kinbody.IsRobot():
            deserialize_robot_state(kinbody, data['robot_state'], database)

    return kinbody

def deserialize_kinbody_state(body, data, database):
    deserialization_logger.debug('Deserializing "%s" KinBody state.',
        body.GetName())

    for key, (_, set_fn) in KINBODY_STATE_MAP.iteritems():
        try:
            print key
            set_fn(body, data[key])
        except Exception as e:
            deserialization_logger.error(
                'Failed deserializing KinBody "%s" state "%s": %s',
                body.GetName(), key, e.message
            )
            raise

    body.SetLinkTransformations(
        [deserialize_transform(datum)
         for datum in data['link_transforms']],
        data['dof_branches']
    )

def deserialize_robot_state(body, data, database):
    deserialization_logger.debug('Deserializing "%s" Robot state.',
        body.GetName())

    for key, (_, set_fn) in ROBOT_STATE_MAP.iteritems():
        set_fn(body, data[key])

    env = body.GetEnv()

    for grabbed_info_dict in data['grabbed_bodies']:
        grabbed_info = deserialize_grabbed_info(grabbed_info_dict, database)

        robot_link = body.GetLink(grabbed_info._robotlinkname)
        robot_links_to_ignore = grabbed_info._setRobotLinksToIgnore

        grabbed_body = env.GetKinBody(grabbed_info._grabbedname)
        grabbed_pose = numpy.dot(robot_link.GetTransform(),
                                 grabbed_info._trelative)
        grabbed_body.SetTransform(grabbed_pose)

        body.Grab(grabbed_body, robot_link, robot_links_to_ignore)

def deserialize_with_map(obj, data, database, attribute_map):
    for key, (_, deserialize_fn) in attribute_map.iteritems():
        setattr(obj, key, deserialize_fn(data[key], database))

    return obj

def deserialize_link_info(data, database):
    return deserialize_with_map(KinBody.LinkInfo(), data, database,
            LINK_INFO_MAP)
    
def deserialize_joint_info(data, database):
    return deserialize_with_map(KinBody.JointInfo(), data, database,
            JOINT_INFO_MAP)

def deserialize_manipulator_info(data, database):
    return deserialize_with_map(Robot.ManipulatorInfo(), data, database,
            MANIPULATOR_INFO_MAP)

def deserialize_geometry_info(data, database):
    geom_info = deserialize_with_map(
        KinBody.GeometryInfo(), data, database, GEOMETRY_INFO_MAP)

    # OpenRAVE only has a ReadTrimeshURI method on Environment. We create a
    # static, dummy environment (mesh_environment) just to load meshes.
    if geom_info._filenamecollision:
        geom_info._meshcollision = mesh_environment.ReadTrimeshURI(
            geom_info._filenamecollision)

    return geom_info

def deserialize_grabbed_info(data, database):
    return deserialize_with_map(Robot.GrabbedInfo(), data, database,
            GRABBED_INFO_MAP)

def deserialize_transform(data):
    return numpy.array(data)

# Schema.
mesh_environment = Environment()
identity = lambda x: x
str_identity = (
    lambda x, _: x,
    lambda x, _: x.encode()
)
both_identity = (
    lambda x, _: x,
    lambda x, _: x
)
numpy_identity = (
    lambda x, _: x.tolist(),
    lambda x, _: numpy.array(x)
)
transform_identity = (
    lambda x, _: serialize_transform(x),
    lambda x, _: deserialize_transform(x)
)

# TODO: Change this to use the database.
path_identity = (
    lambda x, db: serialize_path(x, db),
    lambda x, db: deserialize_path(x, db)    
)

KINBODY_STATE_MAP = {
    'description': (
        lambda x: x.GetDescription(),
        lambda x, value: x.SetDescription(value),
    ),
    'link_enable_states': (
        lambda x: x.GetLinkEnableStates().tolist(),
        lambda x, value: x.SetLinkEnableStates(value)
    ),
    'link_velocities': (
        lambda x: x.GetLinkVelocities().tolist(),
        lambda x, value: x.SetLinkVelocities(value),
    ),
    'transform': (
        lambda x: serialize_transform(x.GetTransform()),
        lambda x, value: x.SetTransform(deserialize_transform(value)),
    ),
    'dof_weights': (
        lambda x: x.GetDOFWeights().tolist(),
        lambda x, value: x.SetDOFWeights(value),
    ),
    'dof_resolutions': (
        lambda x: x.GetDOFResolutions().tolist(),
        lambda x, value: x.SetDOFResolutions(value),
    ),
    'dof_position_limits': (
        lambda x: [ limits.tolist() for limits in x.GetDOFLimits() ],
        lambda x, (lower, upper): x.SetDOFLimits(lower, upper),
    ),
    'dof_velocity_limits': (
        lambda x: x.GetDOFVelocityLimits().tolist(),
        lambda x, value: x.SetDOFVelocityLimits(value),
    ),
    'dof_acceleration_limits': (
        lambda x: x.GetDOFAccelerationLimits().tolist(),
        lambda x, value: x.SetDOFAccelerationLimits(value),
    ),
    'dof_torque_limits': (
        lambda x: x.GetDOFTorqueLimits().tolist(),
        lambda x, value: x.SetDOFTorqueLimits(value),
    ),
    'dof_positions': (
        lambda x: x.GetDOFValues().tolist(),
        lambda x, value: x.SetDOFValues(value)
    ),
    # TODO: What about link accelerations and geometry groups?
}
ROBOT_STATE_MAP = {
    # TODO: Does this preserve affine DOFs?
    'active_dof_indices': (
        lambda x: x.GetActiveDOFIndices().tolist(),
        lambda x, value: x.SetActiveDOFs(value)
    ),
    'active_manipulator': (
        lambda x: x.GetActiveManipulator().GetName() if x.GetActiveManipulator() is not None else '',
        lambda x, value: x.SetActiveManipulator(value)
    ),
}
LINK_INFO_MAP = {
    '_bIsEnabled': both_identity,
    '_bStatic': both_identity,
    '_mapFloatParameters': both_identity,
    '_mapIntParameters': both_identity,
    '_mapStringParameters': both_identity, # TODO
    '_mass': both_identity,
    '_name': str_identity,
    '_t': transform_identity,
    '_tMassFrame': transform_identity,
    '_vForcedAdjacentLinks': both_identity,
    '_vgeometryinfos': (
        lambda x, db: [serialize_geometry_info(gi, db) for gi in x],
        lambda x, db: [deserialize_geometry_info(gi, db) for gi in x],
    ),
    '_vinertiamoments': numpy_identity,
}
JOINT_INFO_MAP = {
    '_bIsActive': both_identity,
    '_bIsCircular': both_identity,
    '_linkname0':  str_identity,
    '_linkname1': str_identity,
    '_mapFloatParameters': both_identity,
    '_mapIntParameters': both_identity,
    '_mapStringParameters': both_identity, # TODO
    '_name': str_identity,
    '_type': (
        lambda x, _: x.name,
        lambda x, _: KinBody.JointType.names[x]
    ),
    '_vanchor': numpy_identity,
    '_vaxes': (
        lambda x, _: [ xi.tolist() for xi in x ],
        lambda x, _: map(numpy.array, x)
    ),
    '_vcurrentvalues': numpy_identity,
    '_vhardmaxvel': numpy_identity,
    '_vlowerlimit': numpy_identity,
    '_vmaxaccel': numpy_identity,
    '_vmaxinertia': numpy_identity,
    '_vmaxtorque': numpy_identity,
    '_vmaxvel': numpy_identity,
    '_vmimic': both_identity,
    '_voffsets': numpy_identity,
    '_vresolution': numpy_identity,
    '_vupperlimit': numpy_identity,
    '_vweights': numpy_identity,
}
GEOMETRY_INFO_MAP = {
    '_bModifiable': both_identity,
    '_bVisible': both_identity,
    '_fTransparency': both_identity,
    '_filenamecollision': path_identity,
    '_filenamerender': path_identity,
    '_t': transform_identity,
    '_type': (
        lambda x, _: x.name,
        lambda x, _: GeometryType.names[x]
    ),
    '_vAmbientColor': numpy_identity,
    '_vCollisionScale': numpy_identity,
    '_vDiffuseColor': numpy_identity,
    '_vGeomData': numpy_identity,
    '_vRenderScale': numpy_identity,
    # TODO: What are these?
    #'_mapExtraGeometries': None
    #15 is not JSON serializable
    #'_trajfollow': None,
}
MANIPULATOR_INFO_MAP = {
    '_name': str_identity,
    '_sBaseLinkName': str_identity,
    '_sEffectorLinkName': str_identity,
    '_sIkSolverXMLId': str_identity,
    '_tLocalTool': transform_identity,
    '_vChuckingDirection': numpy_identity,
    '_vClosingDirection': numpy_identity,
    '_vGripperJointNames': both_identity, # TODO
    '_vdirection': numpy_identity,
}
GRABBED_INFO_MAP = {
    '_grabbedname': str_identity,
    '_robotlinkname': str_identity,
    '_setRobotLinksToIgnore': both_identity, # TODO
    '_trelative': transform_identity,
}
