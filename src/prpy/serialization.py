import numpy
import openravepy

# Serialization.
def serialize_environment(env):
    return {
        'bodies': map(serialize_kinbody, env.GetBodies())
    }

def serialize_kinbody(body):
    all_joints = []
    all_joints.extend(body.GetJoints())
    all_joints.extend(body.GetPassiveJoints())
    all_joints.sort(key=lambda x: x.GetJointIndex())

    return {
        'name': body.GetName(),
        'uri': body.GetXMLFilename(),
        'links': map(serialize_link, body.GetLinks()),
        'joints': map(serialize_joint, all_joints),
        'state': serialize_kinbody_state(body),
    }

def serialize_kinbody_state(body):
    return {
        name: get_fn(body)
        for name, (get_fn, _) in KINBODY_STATE_MAP.iteritems()
    }

def serialize_link(link):
    return {
        'info': serialize_link_info(link.GetInfo())
    }

def serialize_joint(joint):
    return {
        'info': serialize_joint_info(joint.GetInfo())
    }

def serialize_with_map(obj, attribute_map):
    return {
        key: serialize_fn(getattr(obj, key))
        for key, (serialize_fn, _) in attribute_map.iteritems()
    }

def serialize_link_info(link_info):
    return serialize_with_map(link_info, LINK_INFO_MAP)
    
def serialize_joint_info(joint_info):
    return serialize_with_map(joint_info, JOINT_INFO_MAP)

def serialize_manipulator_info(manip_info):
    return serialize_with_map(manip_info, MANIPULATOR_INFO_MAP)

def serialize_geometry_info(geom_info):
    return serialize_with_map(geom_info, GEOMETRY_INFO_MAP)

def serialize_transform(t):
    from openravepy import quatFromRotationMatrix

    return {
        'position': list(t[0:3, 3]),
        'orientation': list(quatFromRotationMatrix(t[0:3, 0:3])),
    }

# Deserialization.
def deserialize_kinbody(env, data, name=None, anonymous=False):
    from openravepy import RaveCreateKinBody

    kinbody = RaveCreateKinBody(env, '')
    kinbody.Init(
        linkinfos=[
            deserialize_link_info(link_data['info']) \
            for link_data in data['links']
        ],
        jointinfos=[
            deserialize_joint_info(joint_data['info']) \
            for joint_data in data['joints']
        ],
        uri=data['uri'],
    )

    kinbody.SetName(name or data['name'])
    env.Add(kinbody, anonymous)

    deserialize_kinbody_state(kinbody, data['state'])

    return kinbody

def deserialize_kinbody_state(body, data):
    for key, (_, set_fn) in KINBODY_STATE_MAP.iteritems():
        set_fn(body, data[key])

def deserialize_with_map(obj, data, attribute_map):
    for key, (_, deserialize_fn) in attribute_map.iteritems():
        setattr(obj, key, deserialize_fn(data[key]))

    return obj

def deserialize_link_info(data):
    from openravepy import KinBody

    return deserialize_with_map(KinBody.LinkInfo(), data, LINK_INFO_MAP)
    
def deserialize_joint_info(data):
    from openravepy import KinBody

    return deserialize_with_map(KinBody.JointInfo(), data, JOINT_INFO_MAP)

def deserialize_manipulator_info(data):
    from openravepy import Robot

    return deserialize_with_map(Robot.ManipulatorInfo(), data, MANIPULATOR_INFO_MAP)

def deserialize_geometry_info(data):
    from openravepy import KinBody 

    geom_info = deserialize_with_map(
        KinBody.GeometryInfo(), data, GEOMETRY_INFO_MAP)

    # OpenRAVE only has a ReadTrimeshURI method on Environment. We create a
    # static, dummy environment (mesh_environment) just to load meshes.
    if geom_info._filenamecollision:
        geom_info._meshcollision = mesh_environment.ReadTrimeshURI(
            geom_info._filenamecollision)

    return geom_info

def deserialize_transform(data):
    from openravepy import matrixFromQuat

    t = matrixFromQuat(data['orientation'])
    t[0:3, 3] = data['position']
    return t

# Schema.
mesh_environment = openravepy.Environment()
identity = lambda x: x
both_identity = (identity, identity)

KINBODY_STATE_MAP = {
    'description': (
        lambda x: x.GetDescription(),
        lambda x, value: x.SetDescription(value),
    ),
    'link_enable_states': (
        lambda x: list(x.GetLinkEnableStates()),
        lambda x, value: x.SetLinkEnableStates(value)
    ),
    'link_transforms': (
        lambda x: map(serialize_transform, x.GetLinkTransformations()),
        lambda x, value: x.SetLinkTransformations(
            map(deserialize_transform, value)
        ),
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
        lambda x: list(x.GetDOFWeights()),
        lambda x, value: x.SetDOFWeights(value),
    ),
    'dof_resolutions': (
        lambda x: list(x.GetDOFResolutions()),
        lambda x, value: x.SetDOFResolutions(value),
    ),
    'dof_position_limits': (
        lambda x: [ limits.tolist() for limits in x.GetDOFLimits() ],
        lambda x, (lower, upper): x.SetDOFLimits(lower, upper),
    ),
    'dof_velocity_limits': (
        lambda x: list(x.GetDOFVelocityLimits()),
        lambda x, value: x.SetDOFVelocityLimits(value),
    ),
    'dof_acceleration_limits': (
        lambda x: list(x.GetDOFAccelerationLimits()),
        lambda x, value: x.SetDOFAccelerationLimits(value),
    ),
    'dof_torque_limits': (
        lambda x: list(x.GetDOFTorqueLimits()),
        lambda x, value: x.SetDOFTorqueLimits(value),
    ),
    # TODO: What about link accelerations and geometry groups?
}
LINK_INFO_MAP = {
    '_bIsEnabled': both_identity,
    '_bStatic': both_identity,
    '_mapFloatParameters': both_identity,
    '_mapIntParameters': both_identity,
    '_mapStringParameters': both_identity,
    '_mass': both_identity,
    '_name': both_identity,
    '_t': (serialize_transform, deserialize_transform),
    '_tMassFrame': (serialize_transform, deserialize_transform),
    '_vForcedAdjacentLinks': both_identity,
    '_vgeometryinfos': (
        lambda x: map(serialize_geometry_info, x),
        lambda x: map(deserialize_geometry_info, x),
    ),
    '_vinertiamoments': (list, numpy.array),
}
JOINT_INFO_MAP = {
    '_bIsActive': both_identity,
    '_bIsCircular': both_identity,
    '_linkname0': both_identity,
    '_linkname1': both_identity,
    '_mapFloatParameters': both_identity,
    '_mapIntParameters': both_identity,
    '_mapStringParameters': both_identity,
    '_name': both_identity,
    '_type': (
        lambda x: x.name,
        lambda x: openravepy.KinBody.JointType.names[x]
    ),
    '_vanchor': (list, numpy.array),
    '_vaxes': (
        lambda x: map(list, x),
        lambda x: map(numpy.array, x)
    ),
    '_vcurrentvalues': (list, numpy.array),
    '_vhardmaxvel': (list, numpy.array),
    '_vlowerlimit': (list, numpy.array),
    '_vmaxaccel': (list, numpy.array),
    '_vmaxinertia': (list, numpy.array),
    '_vmaxtorque': (list, numpy.array),
    '_vmaxvel': (list, numpy.array),
    '_vmimic': both_identity,
    '_voffsets': (list, numpy.array),
    '_vresolution': (list, numpy.array),
    '_vupperlimit': (list, numpy.array),
    '_vweights': (list, numpy.array),
}
GEOMETRY_INFO_MAP = {
    '_bModifiable': both_identity,
    '_bVisible': both_identity,
    '_fTransparency': both_identity,
    '_filenamecollision': both_identity,
    '_filenamerender': both_identity,
    '_t': (serialize_transform, deserialize_transform),
    '_type': (
        lambda x: x.name,
        lambda x: openravepy.GeometryType.names[x]
    ),
    '_vAmbientColor': (list, numpy.array),
    '_vCollisionScale': both_identity,
    '_vDiffuseColor': (list, numpy.array),
    '_vGeomData': (list, numpy.array),
    '_vRenderScale': (list, numpy.array),
    # TODO: What are these?
    #'_mapExtraGeometries': None
    #'_trajfollow': None,
}
MANIPULATOR_INFO_MAP = {
    '_name': both_identity,
    '_sBaseLinkName': both_identity,
    '_sEffectorLinkName': both_identity,
    '_sIkSolverXMLId': both_identity,
    '_tLocalTool': (serialize_transform, deserialize_transform),
    '_vChuckingDirection': (list, numpy.array),
    '_vClosingDirection': (list, numpy.array),
    '_vGripperJointNames': (list, numpy.array),
    '_vdirection': (list, numpy.array),
}
