#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
# Authors: Siddhartha Srinivasa <siddh@cs.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import logging
import math
import numpy
import openravepy
import scipy.misc
import scipy.optimize
import threading
import time


logger = logging.getLogger(__name__)


def create_sensor(env, args, anonymous=True):
    sensor = openravepy.RaveCreateSensor(env, args)
    if sensor is None:
        raise Exception("Creating '%s' sensor failed." % args.split()[0])

    env.Add(sensor, anonymous)
    return sensor


def CreatePlannerParametersString(options, params=None,
                                  remove_postprocessing=True):
    """ Creates an OpenRAVE parameter XML string.

    OpenRAVE planners have an InitPlan function that either take an instance of
    the PlannerParameters() struct or the serialized XML version of this
    struct. Unfortunately, it is not possible to override several default
    options in the Python API. This function takes a seed PlannerParameters
    struct and a dictionary of key-value pairs to override. It returns XML that
    can be passed directly to InitPlan.

    @param options: dictionary of key-value pairs
    @type  options: {str: str}
    @param params: input struct (defaults to the defaults in OpenRAVE)
    @type  params: openravepy.Planner.PlannerParameters
    @return planner parameters string XML
    @rtype  str
    """
    import lxml.etree
    import openravepy
    from copy import deepcopy

    options = deepcopy(options)
    if remove_postprocessing:
        options['_postprocessing'] = None

    if params is None:
        params = openravepy.Planner.PlannerParameters()

    params_xml = lxml.etree.fromstring(params.__repr__().split('"""')[1])

    for key, value in options.iteritems():
        element = params_xml.find(key)

        # Remove a value if "value" is None.
        if value is None:
            if element is not None:
                params_xml.remove(element)
        # Add (or overwrite) and existing value.
        else:
            if element is None:
                element = lxml.etree.SubElement(params_xml, key)

            element.text = str(value)

    if remove_postprocessing:
        params_xml.append(
            lxml.etree.fromstring("""
                <_postprocessing planner="">
                    <_nmaxiterations>20</_nmaxiterations>
                    <_postprocessing planner="parabolicsmoother">
                        <_nmaxiterations>100</_nmaxiterations>
                    </_postprocessing>
                </_postprocessing>
            """)
        )

    return lxml.etree.tostring(params_xml)


def HasGroup(cspec, group_name):
    try:
        cspec.GetGroupFromName(group_name)
        return True
    except openravepy.openrave_exception:
        return False


def HasAffineDOFs(cspec):
    return (HasGroup(cspec, 'affine_transform') or
            HasGroup(cspec, 'affine_velocities') or
            HasGroup(cspec, 'affine_accelerations'))


def HasJointDOFs(cspec):
    return (HasGroup(cspec, 'joint_values') or
            HasGroup(cspec, 'joint_velocities') or
            HasGroup(cspec, 'joint_accelerations') or
            HasGroup(cspec, 'joint_torques'))


def GetTrajectoryIndices(traj):
    try:
        cspec = traj.GetConfigurationSpecification()
        joint_values_group = cspec.GetGroupFromName('joint_values')
        return numpy.array([int(index) for index in
                            joint_values_group.name.split()[2:]])
    except openravepy.openrave_exception:
        return list()


def WaitForControllers(controllers, timeout=None, rate=20):
    running_controllers = set(controllers)
    start_time = time.time()
    timestep = 1.0 / rate

    while running_controllers:
        # Check for a timeout.
        now_time = time.time()
        if timeout is not None and now_time - start_time > timeout:
            return False

        # Check if the trajectory is done.
        done_controllers = set()
        for controller in running_controllers:
            if controller.IsDone():
                done_controllers.add(controller)

        running_controllers -= done_controllers
        time.sleep(timestep)

    return True


def SetCameraFromXML(viewer, xml):
    if isinstance(viewer, openravepy.Environment):
        viewer = viewer.GetViewer()

    import lxml.etree
    from StringIO import StringIO
    padded_xml = '<bogus>{0:s}</bogus>'.format(xml)
    camera_xml = lxml.etree.parse(StringIO(padded_xml))

    translation_raw = camera_xml.find('//camtrans').text
    axis_raw = camera_xml.find('//camrotationaxis').text
    focal_raw = camera_xml.find('//camfocal').text

    translation = numpy.loadtxt(StringIO(translation_raw))
    axis_angle = numpy.loadtxt(StringIO(axis_raw))
    axis_angle = axis_angle[3] * axis_angle[0:3] * (numpy.pi / 180)
    focal = float(focal_raw)

    transform = openravepy.matrixFromAxisAngle(axis_angle)
    transform[0:3, 3] = translation
    viewer.SetCamera(transform, focal)


def TakeSnapshot(viewer, path=None, show_figures=True,
                 width=1920, height=1080, fx=640, fy=640):
    if isinstance(viewer, openravepy.Environment):
        viewer = viewer.GetViewer()

    viewer.SendCommand('SetFiguresInCamera {0:d}'.format(show_figures))
    image = viewer.GetCameraImage(width, height, viewer.GetCameraTransform(),
                                  [fx, fy, width / 2, height / 2])

    if path is not None:
        scipy.misc.imsave(path, image)

    return image


def ComputeAinv(N, dof):
    dt = 1.0 / (N - 1)
    K = numpy.mat(numpy.zeros((N - 1, N - 1)))
    for i in range(1, N - 1):
        K[i, i] = 1 / dt
        K[i, i - 1] = -1 / dt
    K[0, 0] = 1 / dt
    A = K.transpose() * K
    invA_small = numpy.linalg.inv(A)

    # Tensorize.
    invA = numpy.mat(numpy.zeros([(N) * dof, (N) * dof]))
    for i in range(1, N):
        for j in range(1, N):
            for k in range(dof):
                invA[i * dof + k, j * dof + k] = invA_small[i - 1, j - 1]
    return invA


def NormalizeVector(vec):
    """
    Normalize a vector.
    This is faster than doing: vec/numpy.linalg.norm(vec)

    @param numpy.array vec: A 1-dimensional vector.
    @returns numpy.array result: A vector of the same size, where the
                                 L2 norm of the elements equals 1.
    """
    numpy.seterr(divide='ignore', invalid='ignore')
    magnitude = numpy.sqrt(vec.dot(vec))
    vec2 = (vec / magnitude)
    return numpy.nan_to_num(vec2)  # convert NaN to zero


def MatrixToTraj(traj_matrix, cs, dof, robot):
    env = robot.GetEnv()
    traj = openravepy.RaveCreateTrajectory(env, '')
    traj.Init(cs)
    for i in range(numpy.size(traj_matrix) / dof):
        tp = traj_matrix[range(i * dof, i * dof + dof)]
        tp = numpy.array(tp.transpose())[0]
        traj.Insert(i, tp)
    openravepy.planningutils.RetimeActiveDOFTrajectory(
        traj, robot, False, 0.2, 0.2, "LinearTrajectoryRetimer", "")
    return traj


def TrajToMatrix(traj, dof):
    traj_matrix = numpy.zeros(dof * (traj.GetNumWaypoints()))
    traj_matrix = numpy.mat(traj_matrix).transpose()
    for i in range(traj.GetNumWaypoints()):
        d = traj.GetWaypoint(i)[range(dof)]
        d = numpy.mat(d).transpose()
        traj_matrix[range(i * dof, (i + 1) * dof)] = d
    return traj_matrix


def AdaptTrajectory(traj, new_start, new_goal, robot):
    """
    Adapt an existing trajectory to move between a new start and goal.

    The trajectory's configuration specification must contain exactly one group
    called "joint_values". Note that this does NOT collision check the warped
    trajectory.

    @param traj input trajectory
    @param new_start new starting configuration
    @param new_goal new goal configuration
    @param robot
    @return adapted trajectory
    """
    # TODO: check joint limits
    # TODO: support arbitrary trajectory types
    # TODO: collision check the warped trajectory
    # TODO: this should not require a robot as a parameter
    cs = traj.GetConfigurationSpecification()
    dof = cs.GetDOF()

    start = traj.GetWaypoint(0)
    start = start[range(dof)]
    goal = traj.GetWaypoint(traj.GetNumWaypoints() - 1)
    goal = goal[range(dof)]

    traj_matrix = TrajToMatrix(traj, dof)

    # Translate trajectory to match start point.
    diff_start = new_start - start
    diff_start = numpy.mat(diff_start).transpose()
    translated_traj = numpy.zeros(dof * (traj.GetNumWaypoints()))
    translated_traj = numpy.mat(translated_traj).transpose()
    for i in range(traj.GetNumWaypoints()):
        translated_traj[range((i - 1) * dof, i * dof)] = \
            traj_matrix[range((i - 1) * dof, i * dof)] - diff_start

    # Apply correction to reach goal point.
    new_traj_matrix = translated_traj
    N = traj.GetNumWaypoints()
    goal_translated = new_traj_matrix[range((N - 1) * dof, (N) * dof)]
    Ainv = ComputeAinv(N, dof)
    goal_diff = numpy.mat(new_goal).transpose() - goal_translated
    traj_diff = numpy.zeros(dof * (N))
    traj_diff = numpy.mat(traj_diff).transpose()
    traj_diff[range((N - 1) * dof, (N) * dof)] = goal_diff
    new_traj_matrix += Ainv * traj_diff / Ainv[N * dof - 1, N * dof - 1]

    new_traj = MatrixToTraj(new_traj_matrix, cs, dof, robot)
    return new_traj


def CopyTrajectory(traj, env=None):
    """
    Create a new copy of a trajectory using its Clone() operator.

    @param traj input trajectory
    @param env optional environment used to initialize a trajectory
    @return copy of the trajectory
    """
    copy_traj = openravepy.RaveCreateTrajectory(env or traj.GetEnv(),
                                                traj.GetXMLId())
    copy_traj.Clone(traj, 0)
    copy_traj.SetDescription(traj.GetDescription())
    return copy_traj


def GetTrajectoryTags(traj):
    """
    Read key/value pairs from a trajectory.

    The metadata is can be set by SetTrajectoryTags; see that function for
    details. If no metadata is set, this function returns an empty dictionary.

    @param traj input trajectory
    @return dictionary of string key/value pairs
    """
    import json

    description = traj.GetDescription()

    if description == '':
        return dict()
    else:
        try:
            return json.loads(description)
        except ValueError as e:
            logger.warning('Failed reading tags from trajectory: %s',
                           e.message)
            return dict()


def SetTrajectoryTags(traj, tags, append=False):
    """
    Tag a trajectory with a dictionary of key/value pairs.

    If append = True, then the dictionary of tags is added to any existing tags
    on the trajectory. Otherwise, all existing tags will be replaced. This
    metadata can be accessed by GetTrajectoryTags. Currently, the metadata is
    stored as JSON in the trajectory's description.

    @param traj input trajectory
    @param append if true, retain existing tags on the trajectory
    """
    import json

    if append:
        all_tags = GetTrajectoryTags(traj)
        all_tags.update(tags)
    else:
        all_tags = tags

    traj.SetDescription(json.dumps(all_tags))


def SimplifyTrajectory(traj, robot):
    """
    Re-interpolate trajectory as minimal set of linear segments.

    This function attempts to extract linear segments from the given
    trajectory by iteratively finding extrema waypoints until a set of
    linear segments until all of the original trajectory waypoints are
    within the robot's joint resolutions of the interpolated segments.

    Currently, only untimed trajectories are supported!

    @param robot the robot that should be used for the interpolation
    @param traj input trajectory that will be simplified
    @returns output trajectory of timed linear segments
    """
    from scipy import interpolate

    if traj.GetDuration() != 0.0:
        raise ValueError("Cannot handle timed trajectories yet!")

    if traj.GetNumWaypoints() < 2:
        return traj

    cspec = traj.GetConfigurationSpecification()
    dofs = robot.GetActiveDOFIndices()
    idxs = range(traj.GetNumWaypoints())
    joints = [robot.GetJointFromDOFIndex(d) for d in dofs]

    times = numpy.array(
        idxs if not traj.GetDuration() else
        numpy.cumsum([cspec.ExtractDeltaTime(traj.GetWaypoint(i),
                                             robot, dofs) for i in idxs]))
    values = numpy.array(
        [cspec.ExtractJointValues(traj.GetWaypoint(i), robot, dofs)
         for i in idxs])
    resolutions = numpy.array([j.GetResolution(0) for j in joints])

    # Start with an extrema set of the first to the last waypoint.
    mask = numpy.zeros(times.shape, dtype=bool)
    mask[[0, -1]] = True

    for _ in idxs:
        # Create new interpolation set from current extrema.
        f = interpolate.interp1d(times[mask], values[mask, :],
                                 axis=0, kind='linear')
        errors = numpy.abs(f(times) - values)

        # TODO: Can this be a single call?
        # Find the extrema in the remaining waypoints.
        max_err_idx = numpy.argmax(errors, axis=0)
        max_err_vals = numpy.max(errors, axis=0)

        # Add any extrema that deviated more than joint resolution.
        max_err_idx = max_err_idx[max_err_vals > resolutions]
        mask[max_err_idx] = True

        # If none deviated more than joint resolution, the set is complete.
        if len(max_err_idx) < 0:
            break

    # Return a new reduced trajectory.
    import openravepy
    reduced_traj = openravepy.RaveCreateTrajectory(traj.GetEnv(),
                                                   traj.GetXMLId())
    reduced_traj.Init(cspec)
    for (new_idx, old_idx) in enumerate(mask.nonzero()[0]):
        reduced_traj.Insert(new_idx, traj.GetWaypoint(old_idx))
    return reduced_traj


class Recorder(object):
    MPEG = 13

    def __init__(self, env, filename, width=1920, height=1080, codec=MPEG):
        self.env = env
        self.filename = filename
        self.width = width
        self.height = height
        self.codec = codec

        self.module = openravepy.RaveCreateModule(env, 'ViewerRecorder')
        env.Add(self.module)

    def __enter__(self):
        self.Start()

    def __exit__(self, type, value, traceback):
        self.Stop()

    def start(self):
        cmd = ('Start {width:d} {height:d} 30 '
               'codec {codec:d} timing realtime filename {filename:s}\n'
               'viewer {viewer:s}'
               .format(width=self.width, height=self.height,
                       codec=self.codec, filename=self.filename,
                       viewer=self.env.GetViewer().GetName()))
        self.module.SendCommand(cmd)

    def stop(self):
        self.module.SendCommand('Stop')


class AlignmentToken(object):
    def __init__(self, env, child_frame, extents,
                 pose=None, period=0.05, parent_frame='world'):
        self.child_frame = child_frame
        self.parent_frame = parent_frame
        self.period = period

        with env:
            self.body = openravepy.RaveCreateKinBody(env, '')
            aabbs = numpy.concatenate(([0., 0., 0.], extents)).reshape((1, 6))
            self.body.InitFromBoxes(aabbs, True)
            self.body.SetName('frame:' + child_frame)

            if pose is not None:
                self.body.SetTransform(pose)

            env.Add(self.body, True)

        import tf
        self.broadcaster = tf.TransformBroadcaster()

        self.update()

    def update(self):
        import rospy

        with self.body.GetEnv():
            or_pose = self.body.GetTransform()
            or_quaternion = openravepy.quatFromRotationMatrix(or_pose)

        position = tuple(or_pose[0:3, 3])
        orientation = (or_quaternion[1], or_quaternion[2],
                       or_quaternion[3], or_quaternion[0])
        self.broadcaster.sendTransform(position, orientation, rospy.Time.now(),
                                       self.child_frame, self.parent_frame)

        self.timer = threading.Timer(self.period, self.update)
        self.timer.daemon = True
        self.timer.start()

    def destroy(self):
        self.body.GetEnv().Remove(self.body)
        self.body = None


class Timer(object):
    def __init__(self, message=None):
        self.message = message
        self.start = 0

    def __enter__(self):
        if self.message is not None:
            logging.info('%s started execution.', self.message)

        self.start = time.time()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

        if self.message is not None:
            logging.info('%s executed in %.5f seconds.',
                         self.message, self.get_duration())

    def stop(self):
        self.end = time.time()

    def get_duration(self):
        return self.end - self.start


class Watchdog(object):
    """
    Calls specified function after duration, unless reset/stopped beforehand

    @param timeout_duration how long to wait before calling handler
    @param handler function to call after timeout_duration
    @param args for handler
    @param kwargs for handler
    """
    def __init__(self, timeout_duration, handler, args=(), kwargs={}):
        self.timeout_duration = timeout_duration
        self.handler = handler
        self.handler_args = args
        self.handler_kwargs = kwargs

        self.thread_checking_time = threading.Thread(
            target=self._check_timer_loop)
        self.timer_thread_lock = threading.Lock()
        self.start_time = time.time()
        self.canceled = False

        self.thread_checking_time.start()

    def reset(self):
        """
        Resets the timer.

        Causes the handler function to be called after the next timeout
        duration is reached. Also restarts the timer thread if it has existed.
        """
        with self.timer_thread_lock:
            if self.canceled or not self.thread_checking_time.is_alive():
                self.thread_checking_time = threading.Thread(
                    target=self._check_timer_loop)
                self.thread_checking_time.start()

            self.start_time = time.time()
            self.canceled = False

    def stop(self):
        """
        Stop the watchdog, so it will not call handler
        """
        with self.timer_thread_lock:
            self.canceled = True

    def _check_timer_loop(self):
        """
        Internal function for timer thread to loop

        If elapsed time has passed, calls the handler function
        Exists if watchdog was canceled, or handler was called
        """
        while True:
            with self.timer_thread_lock:
                if self.canceled:
                    break
                elapsed_time = time.time() - self.start_time
            if elapsed_time > self.timeout_duration:
                self.handler(*self.handler_args, **self.handler_kwargs)
                with self.timer_thread_lock:
                    self.canceled = True
                break
            else:
                time.sleep(self.timeout_duration - elapsed_time)


def quadraticPlusJointLimitObjective(dq, J, dx, q, q_min, q_max,
                                     delta_joint_penalty=5e-1,
                                     lambda_dqdist=0.01,
                                     *args):
    """
    Quadratic and joint-limit-avoidance objective for SciPy's optimization.

    @param dq joint velocity
    @param J Jacobian
    @param dx desired twist
    @param q current joint values
    @param q_min lower joint limit
    @param q_max upper joint limit
    @param delta_joint_penalty distance from limit with penality
    @param lamdbda_dqdist weighting for joint limit penalty
    """
    # Compute quadratic distance part.
    objective, gradient = quadraticObjective(dq, J, dx)

    # Add penalty for joint limit avoidance.
    qdiff_lower = delta_joint_penalty - (q - q_min)
    qdiff_upper = delta_joint_penalty - (q_max - q)

    dq_target = [diff_lower if diff_lower > 0. else
                 (-diff_upper if diff_upper > 0. else 0.)
                 for diff_lower, diff_upper in zip(qdiff_lower, qdiff_upper)]

    objective += lambda_dqdist * 0.5 * sum(numpy.square(dq - dq_target))
    gradient += lambda_dqdist * (dq - dq_target)

    return objective, gradient


def quadraticObjective(dq, J, dx, *args):
    """
    Quadratic objective function for SciPy's optimization.
    @param dq joint velocity
    @param J Jacobian
    @param dx desired twist
    @return objective the objective function
    @return gradient the analytical gradient of the objective
    """
    error = (numpy.dot(J, dq) - dx)
    objective = 0.5 * numpy.dot(numpy.transpose(error), error)
    gradient = numpy.dot(numpy.transpose(J), error)
    return objective, gradient


def ComputeJointVelocityFromTwist(
        robot, twist, objective=quadraticObjective, dq_init=None,
        joint_limit_tolerance=3e-2, joint_velocity_limits=None):
    """
    Computes the optimal joint velocity given a twist by formulating
    the problem as a quadratic optimization with box constraints and
    using SciPy's L-BFGS-B solver.
    @params robot the robot
    @params twist the desired twist in se(3)
            with float('NaN') for dimensions we don't care about
    @params objective optional objective function to optimize
            defaults to quadraticObjective
    @params dq_init optional initial guess for optimal joint velocity
            defaults to robot.GetActiveDOFVelocities()
    @params joint_velocity_limits override the robot's joint velocity limit;
            defaults to robot.GetActiveDOFMaxVel()
    @params joint_limit_tolerance if less then this distance to joint
            limit, velocity is bounded in that direction to 0
    @return dq_opt optimal joint velocity
    @return twist_opt actual achieved twist
            can be different from desired twist due to constraints
    """
    manip = robot.GetActiveManipulator()
    robot.SetActiveDOFs(manip.GetArmIndices())

    if joint_velocity_limits is None:
        joint_velocity_limits = robot.GetActiveDOFMaxVel()
    elif isinstance(joint_velocity_limits, float):
        joint_velocity_limits = numpy.array(
            [numpy.PINF] * robot.GetActiveDOF())

    if len(joint_velocity_limits) != robot.GetActiveDOF():
        raise ValueError(
            'Joint velocity limits has incorrect length:'
            ' Expected {:d}, got {:d}.'.format(
                robot.GetActiveDOF(), len(joint_velocity_limits)))
    elif (joint_velocity_limits <= 0.).any():
        raise ValueError('One or more joint velocity limit is not positive.')

    jacobian_spatial = manip.CalculateJacobian()
    jacobian_angular = manip.CalculateAngularVelocityJacobian()
    jacobian = numpy.vstack((jacobian_spatial, jacobian_angular))

    rows = [i for i, x in enumerate(twist) if math.isnan(x) is False]
    twist_active = twist[rows]
    jacobian_active = jacobian[rows, :]

    bounds = numpy.column_stack(
        (-joint_velocity_limits, joint_velocity_limits))

    # Check for joint limits
    q_curr = robot.GetActiveDOFValues()
    q_min, q_max = robot.GetActiveDOFLimits()
    dq_bounds = [
        (0., max) if q_curr[i] <= q_min[i] + joint_limit_tolerance else
        (min, 0.) if q_curr[i] >= q_max[i] - joint_limit_tolerance else
        (min, max) for i, (min, max) in enumerate(bounds)
    ]

    if dq_init is None:
        dq_init = robot.GetActiveDOFVelocities()

    opt = scipy.optimize.fmin_l_bfgs_b(
        objective, dq_init, fprime=None,
        args=(jacobian_active, twist_active, q_curr, q_min, q_max),
        bounds=dq_bounds, approx_grad=False
    )

    dq_opt = opt[0]
    twist_opt = numpy.dot(jacobian, dq_opt)

    return dq_opt, twist_opt


def GeodesicTwist(t1, t2):
    """
    Computes the twist in global coordinates that corresponds
    to the gradient of the geodesic distance between two transforms.

    @param t1 current transform
    @param t2 goal transform
    @return twist in se(3)
    """
    trel = numpy.dot(numpy.linalg.inv(t1), t2)
    trans = numpy.dot(t1[0:3, 0:3], trel[0:3, 3])
    omega = numpy.dot(t1[0:3, 0:3],
                      openravepy.axisAngleFromRotationMatrix(
                      trel[0:3, 0:3]))
    return numpy.hstack((trans, omega))


def GeodesicError(t1, t2):
    """
    Computes the error in global coordinates between two transforms.

    @param t1 current transform
    @param t2 goal transform
    @return a 4-vector of [dx, dy, dz, solid angle]
    """
    trel = numpy.dot(numpy.linalg.inv(t1), t2)
    trans = numpy.dot(t1[0:3, 0:3], trel[0:3, 3])
    omega = openravepy.axisAngleFromRotationMatrix(trel[0:3, 0:3])
    angle = numpy.linalg.norm(omega)
    return numpy.hstack((trans, angle))


def AngleBetweenQuaternions(quat1, quat2):
    """
    Compute the angle between two quaternions.
    From 0 to 2pi.
    """
    theta = numpy.arccos(2.0 * (quat1.dot(quat2))**2 - 1.0)
    return theta


def AngleBetweenRotations(rot1, rot2):
    """
    Compute the angle between two 3x3 rotation matrices.
    From 0 to 2pi.
    """
    quat1 = openravepy.quatFromRotationMatrix(rot1)
    quat2 = openravepy.quatFromRotationMatrix(rot2)
    return AngleBetweenQuaternions(quat1, quat2)


def GeodesicDistance(t1, t2, r=1.0):
    """
    Computes the geodesic distance between two transforms

    @param t1 current transform
    @param t2 goal transform
    @param r in units of meters/radians converts radians to meters
    """
    error = GeodesicError(t1, t2)
    error[3] = r * error[3]
    return numpy.linalg.norm(error)


def GetGeodesicDistanceBetweenTransforms(T0, T1, r=1.0):
    """
    Wrapper, to match GetGeodesicDistanceBetweenQuaternions()

    Calculate the geodesic distance between two transforms, being
    gd = norm( relative translation + r * axis-angle error )

    @param t1 current transform
    @param t2 goal transform
    @param r in units of meters/radians converts radians to meters
    """
    return GeodesicDistance(T0, T1, r)


def GetEuclideanDistanceBetweenPoints(p0, p1):
    """
    Calculate the Euclidean distance (L2 norm) between two vectors.
    """
    sum = 0.0
    for i in xrange(len(p0)):
        sum = sum + (p0[i] - p1[i]) * (p0[i] - p1[i])
    return numpy.sqrt(sum)


def GetEuclideanDistanceBetweenTransforms(T0, T1):
    """
    Calculate the Euclidean distance between the translational
    component of two 4x4 transforms.
    (also called L2 or Pythagorean distance)
    """
    p0 = T0[0:3, 3]  # Get the x,y,z translation from the 4x4 matrix
    p1 = T1[0:3, 3]
    return GetEuclideanDistanceBetweenPoints(p0, p1)


def GetMinDistanceBetweenTransformAndWorkspaceTraj(T, traj, dt=0.01):
    """
    Find the location on a workspace trajectory which is closest
    to the specified transform.

    @param numpy.matrix T: A 4x4 transformation matrix.
    @param openravepy.Trajectory traj: A timed workspace trajectory.
    @param float dt: Resolution at which to sample along the trajectory.

    @return (float,float) (min_dist, t_loc, T_loc) The minimum distance,
                                         the time value along the timed
                                         trajectory, and the transform.
    """
    if not IsTimedTrajectory(traj):
        raise ValueError("Trajectory must have timing information.")

    if not IsTrajectoryTypeIkParameterizationTransform6D(traj):
        raise ValueError("Trajectory is not a workspace trajectory, it "
                         "must have configuration specification of "
                         "openravepy.IkParameterizationType.Transform6D")

    def _GetError(t):
        T_curr = openravepy.matrixFromPose(traj.Sample(t)[0:7])
        error = GetEuclideanDistanceBetweenTransforms(T, T_curr)
        return error

    min_dist = numpy.inf
    t_loc = 0.0
    T_loc = None

    # Iterate over the trajectory
    t = 0.0
    duration = traj.GetDuration()
    while t < duration:
        error = _GetError(t)
        if error < min_dist:
            min_dist = error
            t_loc = t
        t = t + dt
    # Also check the end-point
    error = _GetError(duration)
    if error < min_dist:
        min_dist = error
        t_loc = t

    T_loc = openravepy.matrixFromPose(traj.Sample(t_loc)[0:7])

    return (min_dist, t_loc, T_loc)


def FindCatkinResource(package, relative_path):
    """
    Find a Catkin resource in the share directory or
    the package source directory. Raises IOError
    if resource is not found.

    @param relative_path Path relative to share or package source directory
    @param package The package to search in
    @return Absolute path to resource
    """
    from catkin.find_in_workspaces import find_in_workspaces

    paths = find_in_workspaces(project=package, search_dirs=['share'],
                               path=relative_path, first_match_only=True)

    if paths and len(paths) == 1:
        return paths[0]
    else:
        raise IOError('Loading resource "{:s}" failed.'.format(
                      relative_path))


def IsAtTrajectoryWaypoint(robot, trajectory, waypoint_idx):
    """
    Check if robot is at a particular waypoint in a trajectory.

    This function examines the current DOF values of the specified
    robot and compares these values to the first waypoint of the
    specified trajectory. If the DOF values specified in the trajectory
    differ by less than the DOF resolution of the specified joint/axis
    then it will return True. Otherwise, it returns False.

    NOTE: This is used in ExecuteTrajectory(),
                            IsAtTrajectoryStart(), and
                              IsAtTrajectoryEnd()

    @param robot: The robot whose active DOFs will be checked.
    @param trajectory: The trajectory containing the waypoint
                       to be checked.
    @returns: True The robot is at the desired position.
              False One or more joints differ by DOF resolution.
    """
    if trajectory.GetNumWaypoints() == 0:
        raise ValueError('Trajectory has 0 waypoints!')

    cspec = trajectory.GetConfigurationSpecification()
    needs_base = HasAffineDOFs(cspec)
    needs_joints = HasJointDOFs(cspec)

    if needs_base and needs_joints:
        raise ValueError('Trajectories with affine and joint DOFs are '
                         'not supported')

    if trajectory.GetEnv() != robot.GetEnv():
        raise ValueError('The environment attached to the trajectory '
                         'does not match the environment attached to '
                         'the robot in IsAtTrajectoryStart().')
    if needs_base:
        rtf = robot.GetTransform()
        doft = (openravepy.DOFAffine.X |
                openravepy.DOFAffine.Y |
                openravepy.DOFAffine.RotationAxis)
        curr_pose = openravepy.RaveGetAffineDOFValuesFromTransform(rtf, doft)
        start_transform = numpy.eye(4)
        waypoint = trajectory.GetWaypoint(0)
        start_t = cspec.ExtractTransform(start_transform, waypoint, robot)
        traj_start = openravepy.RaveGetAffineDOFValuesFromTransform(
            start_t, doft)

        # Compare translation distance
        trans_delta_value = abs(curr_pose[:2] - traj_start[:2])
        trans_resolution = robot.GetAffineTranslationResolution()[:2]
        if trans_delta_value[0] > trans_resolution[0] or \
           trans_delta_value[1] > trans_resolution[1]:
            return False

        # Compare rotation distance
        rot_delta_value = abs(wrap_to_interval(curr_pose[2] - traj_start[2]))
        rot_res = robot.GetAffineRotationAxisResolution()[2]  # Rot about z?
        if rot_delta_value > rot_res:
            return False

    else:
        # Get joint indices used in the trajectory,
        # and the joint positions at this waypoint
        waypoint = trajectory.GetWaypoint(waypoint_idx)
        dof_indices, _ = cspec.ExtractUsedIndices(robot)
        goal_config = cspec.ExtractJointValues(waypoint, robot, dof_indices)

        # Return false if any joint deviates too much
        return IsAtConfiguration(robot, goal_config, dof_indices)

    return True


def IsAtTrajectoryStart(robot, trajectory):
    """
    Check if robot is at the configuration specified by
    the FIRST waypoint in a trajectory.
    """
    waypoint_idx = 0
    return IsAtTrajectoryWaypoint(robot, trajectory, waypoint_idx)


def IsAtTrajectoryEnd(robot, trajectory):
    """
    Check if robot is at the configuration specified by
    the LAST waypoint in a trajectory.
    """
    waypoint_idx = trajectory.GetNumWaypoints() - 1
    return IsAtTrajectoryWaypoint(robot, trajectory, waypoint_idx)


def IsAtConfiguration(robot, goal_config, dof_indices=None):
    """
    Check if robot's joints have reached a desired configuration.

    If the DOF indices are not specified, the robot's active DOF
    will be used.

    @param robot The robot object.
    @param goal_config The desired configuration, an array of joint
                       positions.
    @param dof_indices The joint index numbers.

    @return boolean Returns True if joints are at goal position,
                    within DOF resolution.
    """

    # If DOF indices not specified, use the active DOF by default
    if dof_indices is None:
        dof_indices = robot.GetActiveDOFIndices()

    # Get current position of joints
    with robot.GetEnv():
        joint_values = robot.GetDOFValues(dof_indices)
        dof_resolutions = robot.GetDOFResolutions(dof_indices)

    # If any joint is not at the goal position, return False
    for i in xrange(0, len(goal_config)):
        # Get the axis index for this joint, which is 0
        # for revolute joints or 0-2 for spherical joints.
        joint = robot.GetJointFromDOFIndex(dof_indices[i])
        axis_idx = dof_indices[i] - joint.GetDOFIndex()
        # Use OpenRAVE method to check the configuration
        # difference value1-value2 for axis i,
        # taking into account joint limits and wrapping
        # of continuous joints.
        delta_value = abs(joint.SubtractValue(joint_values[i], goal_config[i],
                                              axis_idx))
        if delta_value > dof_resolutions[i]:
            return False

    # If all joints match the goal, return True
    return True


def IsTimedTrajectory(trajectory):
    """
    Returns True if the trajectory is timed.

    This function checks whether a trajectory has a valid `deltatime` group,
    indicating that it is a timed trajectory.

    @param trajectory: an OpenRAVE trajectory
    @returns: True if the trajectory is timed, False otherwise
    """
    cspec = trajectory.GetConfigurationSpecification()
    empty_waypoint = numpy.zeros(cspec.GetDOF())
    return cspec.ExtractDeltaTime(empty_waypoint) is not None


def IsJointSpaceTrajectory(traj):
    """
    Check if trajectory is a joint space trajectory.

    @param openravepy.Trajectory traj: A path or trajectory.
    @return bool result: Returns True or False.
    """
    try:
        cspec = traj.GetConfigurationSpecification()
        if cspec.GetGroupFromName("joint_values"):
            return True
    except openravepy.openrave_exception:
        pass
    return False


def IsWorkspaceTrajectory(traj):
    """
    Check if trajectory is a workspace trajectory.

    @param openravepy.Trajectory traj: A path or trajectory.
    @return bool result: Returns True or False.
    """
    return IsTrajectoryTypeIkParameterizationTransform6D(traj)


def IsTrajectoryTypeIkParameterization(traj):
    """
    Check if trajectory has a configuration specification
    of type IkParameterization:
      Transform6d
      Rotation3D
      Translation3D
      Direction3D
      Ray4D
      Lookat3D
      TranslationDirection5D
      TranslationXY2D
      TranslationXYOrientation3D
      TranslationLocalGlobal6D
      TranslationXAxisAngle4D
      TranslationYAxisAngle4D
      TranslationZAxisAngle4D
      TranslationXAxisAngleZNorm4D
      TranslationYAxisAngleXNorm4D
      TranslationZAxisAngleYNorm4D

    @param openravepy.Trajectory traj: A path or trajectory.
    @return bool result: Returns True or False.
    """
    try:
        cspec = traj.GetConfigurationSpecification()
        if cspec.GetGroupFromName("ikparam_values"):
            return True
    except openravepy.openrave_exception:
        pass
    return False


def IsTrajectoryTypeIkParameterizationTransform6D(traj):
    """
    Check if trajectory has a configuration specification
    of type IkParameterization.Transform6D

    @param openravepy.Trajectory traj: A path or trajectory.
    @return bool result: Returns True or False.
    """
    try:
        IKP_type = openravepy.IkParameterizationType.Transform6D
        # The IKP type must be passed as a number
        group_name = "ikparam_values {0}".format(int(IKP_type))
        if traj.GetConfigurationSpecification().GetGroupFromName(group_name):
            return True
    except openravepy.openrave_exception:
        pass
    return False


def IsTrajectoryTypeIkParameterizationTranslationDirection5D(traj):
    """
    Check if trajectory has a configuration specification
    of type IkParameterization.TranslationDirection5D

    @param openravepy.Trajectory traj: A path or trajectory.
    @return bool result: Returns True or False.
    """
    try:
        IKP_type = openravepy.IkParameterizationType.TranslationDirection5D
        group_name = "ikparam_values {0}".format(int(IKP_type))
        if traj.GetConfigurationSpecification().GetGroupFromName(group_name):
            return True
    except openravepy.openrave_exception:
        pass
    return False


def ComputeEnabledAABB(kinbody):
    """
    Returns the AABB of the enabled links of a KinBody.

    @param kinbody: an OpenRAVE KinBody
    @returns: AABB of the enabled links of the KinBody
    """
    from numpy import NINF, PINF
    from openravepy import AABB

    min_corner = numpy.array([PINF] * 3)
    max_corner = numpy.array([NINF] * 3)

    for link in kinbody.GetLinks():
        if link.IsEnabled():
            link_aabb = link.ComputeAABB()
            center = link_aabb.pos()
            half_extents = link_aabb.extents()
            min_corner = numpy.minimum(center - half_extents, min_corner)
            max_corner = numpy.maximum(center + half_extents, max_corner)

    center = (min_corner + max_corner) / 2.
    half_extents = (max_corner - min_corner) / 2.
    return AABB(center, half_extents)


def UntimeTrajectory(trajectory, env=None):
    """
    Returns an untimed copy of the provided trajectory.

    This function strips the DeltaTime group from a timed trajectory to create
    an untimed trajectory.

    @param trajectory: an OpenRAVE trajectory
    @returns: an untimed copy of the provided trajectory.
    """
    cspec = trajectory.GetConfigurationSpecification()
    cspec.RemoveGroups('deltatime', True)
    waypoints = trajectory.GetWaypoints(0, trajectory.GetNumWaypoints(), cspec)

    path = openravepy.RaveCreateTrajectory(env or trajectory.GetEnv(),
                                           trajectory.GetXMLId())
    path.Init(cspec)
    path.Insert(0, waypoints)
    return path


def ComputeUnitTiming(robot, traj, env=None):
    """
    Compute the unit velocity timing of a path or trajectory.

    @param robot: robot whose DOFs should be considered
    @param traj: path or trajectory
    @param env: environment to create the output trajectory in; defaults to the
                same environment as the input trajectory
    @returns: trajectory with unit velocity timing
    """
    from openravepy import RaveCreateTrajectory

    if env is None:
        env = traj.GetEnv()

    old_cspec = traj.GetConfigurationSpecification()
    dof_indices, _ = old_cspec.ExtractUsedIndices(robot)

    with robot.CreateRobotStateSaver():
        robot.SetActiveDOFs(dof_indices)
        new_cspec = robot.GetActiveConfigurationSpecification('linear')
    new_cspec.AddDeltaTimeGroup()

    new_traj = RaveCreateTrajectory(env, '')
    new_traj.Init(new_cspec)

    dof_values_prev = None
    for i in range(traj.GetNumWaypoints()):
        old_waypoint = traj.GetWaypoint(i)
        dof_values = old_cspec.ExtractJointValues(
            old_waypoint, robot, dof_indices)

        if i == 0:
            deltatime = 0.
        else:
            deltatime = numpy.linalg.norm(dof_values - dof_values_prev)

        dof_values_prev = dof_values

        new_waypoint = numpy.zeros(new_cspec.GetDOF())

        new_cspec.InsertJointValues(new_waypoint, dof_values,
                                    robot, dof_indices, 0)
        new_cspec.InsertDeltaTime(new_waypoint, deltatime)
        new_traj.Insert(i, new_waypoint)

    return new_traj


def ComputeGeodesicUnitTiming(traj, env=None, alpha=1.0):
    """
    Compute the geodesic unit velocity timing of a workspace path or
    trajectory, also called a path length parameterization.

    The path length is calculated as the sum of all segment lengths,
    where each segment length = norm( delta_translation^2 +
                                           alpha^2*delta_orientation^2 )

    Note: Currently only linear velocity interpolation is supported,
          however OpenRAVE does allow you to specify quadratic
          interpolation.

    @param traj: Workspace path or trajectory
    @param env: Environment to create the output trajectory in, defaults
                to the same environment as the input trajectory.
    @param alpha: Weighting for delta orientation.
    @returns: A workspace trajectory with unit velocity timing.
    """
    if not IsTrajectoryTypeIkParameterizationTransform6D(traj):
        raise ValueError("Trajectory is not a workspace trajectory, it "
                         "must have configuration specification of "
                         "openravepy.IkParameterizationType.Transform6D")

    num_waypoints = traj.GetNumWaypoints()
    if num_waypoints <= 1:
        raise ValueError("Trajectory needs more than 1 waypoint.")

    if env is None:
        env = traj.GetEnv()

    ikparam = openravepy.IkParameterization
    ikparam_type = openravepy.IkParameterizationType

    # Create a new workspace trajectory with the same spec
    # as the old one
    new_traj = openravepy.RaveCreateTrajectory(env, '')
    new_cspec = ikparam.GetConfigurationSpecificationFromType(
        ikparam_type.Transform6D, 'linear')
    new_cspec.AddDeltaTimeGroup()
    new_traj.Init(new_cspec)

    # Get the current pose of the end effector
    # Note: OpenRAVE pose is [qx,qy,qz,qw, tx,ty,tz]
    #       The 8th value is velocity.
    P_ee_prev = traj.GetWaypoint(0)[range(7)]

    for i in range(num_waypoints):
        P_ee = traj.GetWaypoint(i)[range(7)]  # Get 7 pose values

        # Compute the translation delta
        p0 = P_ee_prev[4:7]  # Get the x,y,z translation
        p1 = P_ee[4:7]
        delta_translation = numpy.sqrt(numpy.sum((p0 - p1)**2))

        # Compute the orientation delta
        q0 = P_ee_prev[0:4]  # Get qx,qy,qz,qw rotation
        q1 = P_ee[0:4]
        delta_angle = AngleBetweenQuaternions(q0, q1)

        dist = numpy.sqrt(delta_translation**2 + (alpha**2) * (delta_angle**2))
        if i == 0:
            deltatime = 0.0
        else:
            deltatime = dist

        P_ee_prev = P_ee

        # Insert a new waypoint (1x7 pose, velocity)
        values = numpy.append(P_ee, [deltatime])
        new_traj.Insert(i, values)

    return new_traj


def CheckJointLimits(robot, q):
    """
    Check if a configuration is within a robot's joint position limits.

    If outside limits, this procedure throws an exception
    of type JointLimitError.

    @param openravepy.robot robot: The robot.
    @param list             q:     List or array of joint positions.
    """
    from prpy.planning.exceptions import JointLimitError

    q_limit_min, q_limit_max = robot.GetActiveDOFLimits()
    active_dof_indices = robot.GetActiveDOFIndices()

    if len(q) != len(active_dof_indices):
        raise ValueError('The number of joints in the configuration q '
                         'is not equal to the number of active DOF.')

    lower_position_violations = (q < q_limit_min)
    if lower_position_violations.any():
        index = lower_position_violations.nonzero()[0][0]
        raise JointLimitError(
            robot,
            dof_index=active_dof_indices[index],
            dof_value=q[index],
            dof_limit=q_limit_min[index],
            description='position')

    upper_position_violations = (q > q_limit_max)
    if upper_position_violations.any():
        index = upper_position_violations.nonzero()[0][0]
        raise JointLimitError(
            robot,
            dof_index=active_dof_indices[index],
            dof_value=q[index],
            dof_limit=q_limit_max[index],
            description='position')


def GetForwardKinematics(robot, q, manipulator=None, frame=None):
    """
    Get the forward kinematics for a specific joint configuration,
    relative to the OpenRAVE world frame.

    @param openravepy.robot robot: The robot object.
    @param list             q:     List or array of joint positions.
    @param manipulator
    @param string frame: Get the end effector transform relative to a
                         specific frame e.g. '/right/wam_base'
    @returns T_ee: The pose of the end effector (or last link in the
                   serial chain) as a 4x4 matrix.
    """
    if manipulator is None:
        manipulator = robot.GetActiveManipulator()

    T_ee = None

    # Save the robot state
    sp = openravepy.Robot.SaveParameters
    robot_saver = robot.CreateRobotStateSaver(sp.LinkTransformation)

    with robot_saver:
        robot.SetActiveDOFValues(q)
        T_ee = manipulator.GetEndEffectorTransform()
    # Robot state is restored

    if frame is not None:
        link = robot.GetLink(frame)
        if link is None:
            raise ValueError('Failed to get link \'{:s}\''.format(frame))

        T_ref_frame = link.GetTransform()
        T_ee = numpy.dot(numpy.linalg.inv(T_ref_frame), T_ee)

    return T_ee


def ConvertIntToBinaryString(x, reverse=False):
    """
    Convert an integer to a binary string.

    Optionally reverse the output string, which is
    required for producing a Van Der Corput sequence.

    @param int  x:       The number to be converted.
    @param bool reverse: If True, the output string will be reversed.

    @returns string: A binary number as a string.
    """
    if type(x) != int:
        raise ValueError('Input number must be an integer')

    if reverse:
        return ''.join(reversed(bin(x)[2:]))

    return ''.join(bin(x)[2:])


def VanDerCorputSequence(lower=0.0, upper=1.0, include_endpoints=True):
    """
    Generate the binary Van der Corput sequence, where each value
    is a dyadic fraction re-scaled to the desired range.

    For example, on the interval [0,1], the first 5 values of
    the Van der Corput sequence are:
    [0.0, 1.0, 0.5, 0.5, 0.75]

    @param float lower: The first value of the range of the sequence.
    @param float upper: The last value of the range of the sequence.

    @param bool include_endpoints: If True, the output sequence will
                                   include the value 'lower' and the
                                   value 'upper'.
                                   If False, these endpoint values
                                   will not be returned.

    @returns generator: A sequence of float values.
    """
    from itertools import count, chain

    if include_endpoints is True:
        endpoints = (0.0, 1.0)
    else:
        endpoints = []

    # Get a sequence of reversed binary numbers:
    # '1', '01', '11', '001', '101', '011', '111', '0001', ....
    #
    # Note: count(1) is a generator, starting at 1, making steps of 1.
    reverse_binary_seq = (ConvertIntToBinaryString(x, True) for x in count(1))

    # From the reversed binary sequence, generate the Van der Corput
    # sequence, for which:  0.0 < x < 1.0  (the end-points are excluded)
    # 0.5, 0.25, 0.75, 0.125, 0.625, 0.375, 0.875, 0.0625, ....
    #
    # Note: int(x,2) converts the binary string (base 2) to an integer.
    raw_seq = (float(int(x, 2)) / (2**len(x)) for x in reverse_binary_seq)

    # Scale the Van der Corput sequence across the desired range
    # and optionally add the end-points.
    scale = float(upper - lower)
    return (scale * val + lower for val in chain(endpoints, raw_seq))


def SampleTimeGenerator(start, end, step=1):
    """
    Generate a linear sequence of values from start to end, with
    specified step size. Works with int or float values.

    The end value is also returned if it's more than half the
    distance from the previously returned value.

    For example, on the interval [0.0,5.0], the sequence is:
    [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]

    @param float start: The start value of the sequence.
    @param float end:   The last value of the sequence.
    @param float step:  The step-size between values.

    @returns generator: A sequence of float values.
    """
    if end <= start:
        raise ValueError("The 'end' value must be greater than "
                         "the 'start' value.")
    if not (step > 0):
        raise ValueError("The 'step' value must be positive.")
    t = start
    prev_t = 0.0
    while t <= numpy.floor(end):
        yield t
        prev_t = t
        t = t + step
    if (end - float(prev_t)) > (step / 2.0):
        yield float(end)


def VanDerCorputSampleGenerator(start, end, step=2):
    """
    This wraps VanDerCorputSequence() in a way that's useful for
    collision-checking.

    Generates a sequence of values from start to end, with specified
    step size, using an approximate binary Van der Corput sequence.

    The start and end values will always be checked first.

    For example, on the interval [0.0, 13.7], the sequence is:
    [0.0, 13.7, 12.0, 6.0, 4.0, 8.0, 2.0, 10.0]

    @param float start: The start value of the sequence.
    @param float end:   The last value of the sequence.
    @param float step:  The step-size between values.

    @returns generator: A sequence of float values.
    """
    # 'start' and 'end' must be positive because
    # itertools.islice() only accepts a positive integer
    if end <= start:
        raise ValueError("The 'end' value must be greater than "
                         "the 'start' value.")
    if not (step > 0):
        raise ValueError("The 'step' value must be positive.")

    # Construct the points at which checks must occur to cover this range.
    check_bins = numpy.arange(start, end, step)
    is_checked = [False] * len(check_bins)

    # Always return the start and end points first.
    is_checked[0] = True
    yield start
    yield end

    # Return a collision-checking sequence that eventually covers the range.
    vdc = VanDerCorputSequence(lower=start, upper=end, include_endpoints=False)
    for s in vdc:
        if numpy.all(is_checked):
            return

        idx = numpy.digitize((s,), check_bins, right=True)
        if is_checked[idx]:
            continue

        is_checked[idx] = True
        yield float(check_bins[idx])


def GetCollisionCheckPts(robot, traj, include_start=True, start_time=0.,
                         first_step=None, epsilon=1e-6):
    """
    Generate a list of (time, configuration) pairs to collision check.

    If every generated configuration is collision free, then the trajectory is
    guaranteed to be collision free up to DOF resolution. This function only
    operates on timed trajectories. If you want to use this function on a path,
    then consider using the util.ComputeUnitTiming function to compute its
    arclength parameterization.

    @param trajectory: timed trajectory
    @returns generator of (time, configuration) pairs
    """

    # TODO: This enters an infinite loop if start_time is non-zero.

    if not IsTimedTrajectory(traj):
        raise ValueError(
            'Trajectory must be timed. If you want to use this function on a'
            ' path, then consider using util.ComputeUnitTiming to compute its'
            ' arclength parameterization.')

    cspec = traj.GetConfigurationSpecification()
    dof_indices, _ = cspec.ExtractUsedIndices(robot)
    q_resolutions = robot.GetDOFResolutions()[dof_indices]
    duration = traj.GetDuration()

    if not (0. <= start_time < duration + epsilon):
        raise ValueError(
            'Start time {:.6f} is out of range [0, {:.6f}].'.format(
                start_time, duration))

    start_time = min(start_time, duration)

    if first_step is None:
        first_step = duration - start_time

    if not (0. < first_step <= duration - start_time):
        raise ValueError(
            'First step {:.6f} is out of range (0, {:.6f}]'.format(
                first_step, duration - start_time))

    # Bisection method. Start at the begining of the trajectory and initialize
    # the stepsize to the end of the trajectory.
    t_prev = start_time
    q_prev = cspec.ExtractJointValues(
        traj.GetWaypoint(t_prev), robot, dof_indices)
    dt = first_step

    # Always collision check the first point.
    if include_start:
        yield t_prev, q_prev

    while t_prev < duration - epsilon:
        t_curr = t_prev + dt
        q_curr = cspec.ExtractJointValues(
            traj.Sample(t_curr), robot, dof_indices)

        # Step violated dof resolution. Halve the step size and continue.
        if (numpy.abs(q_curr - q_prev) > q_resolutions).any():
            dt = dt / 2.
        # Yield this configuration. Double the step size and continue.
        else:
            yield t_curr, q_curr

            q_prev = q_curr
            t_prev = min(t_curr, duration)
            dt = 2. * dt


def GetLinearCollisionCheckPts(robot, traj, norm_order=2, sampling_func=None):
    """
    For a piece-wise linear trajectory, generate a list
    of configuration pairs that need to be collision checked.

    This will step along the trajectory from start to end
    at a resolution that satisifies the specified error metric.

    @param openravepy.Robot      robot: The robot.
    @param openravepy.Trajectory traj:  The trajectory for which we need
                                        to generate sample points.
    @param int      norm_order: 1  ==>  The L1 norm
                                2  ==>  The L2 norm
                                inf  ==>  The L_infinity norm
    @param generator sampling_func A function that returns a sequence of
                                   sample times.
                                   e.g. SampleTimeGenerator()
                                         or
                                        VanDerCorputSampleGenerator()

    @returns generator: A tuple (t,q) of float values, being the sample
                        time and joint configuration.
    """

    # If trajectory is already timed, strip the deltatime values
    if IsTimedTrajectory(traj):
        traj = UntimeTrajectory(traj)

    traj_cspec = traj.GetConfigurationSpecification()

    # Make sure trajectory is linear in joint space
    try:
        # OpenRAVE trajectory type can be 'linear', 'quadratic', or
        # other values including 'cubic', 'quadric' or 'quintic'
        interp_type = traj_cspec.GetGroupFromName('joint_values').interpolation
    except openravepy.openrave_exception:
        raise ValueError('Trajectory does not have a joint_values group')
    if interp_type != 'linear':
        raise ValueError('Trajectory must be linear in joint space')

    dof_indices, _ = traj_cspec.ExtractUsedIndices(robot)

    # If trajectory only has 1 waypoint then we only need to
    # do 1 collision check.
    num_waypoints = traj.GetNumWaypoints()
    if num_waypoints == 1:
        t = 0.0
        waypoint = traj.GetWaypoint(0)
        q = traj_cspec.ExtractJointValues(waypoint, robot, dof_indices)
        yield t, q
        return

    env = robot.GetEnv()

    # Create a temporary trajectory that we will use
    # for sampling the points to collision check,
    # because there is no method to modify the 'deltatime'
    # values of waypoints in an OpenRAVE trajectory.
    temp_traj_cspec = traj.GetConfigurationSpecification()
    temp_traj_cspec.AddDeltaTimeGroup()
    temp_traj = openravepy.RaveCreateTrajectory(env, '')
    temp_traj.Init(temp_traj_cspec)

    # Set timing of first waypoint in temporary trajectory to t=0
    waypoint = traj.GetWaypoint(0, temp_traj_cspec)
    q0 = traj_cspec.ExtractJointValues(waypoint, robot, dof_indices)
    delta_time = 0.0
    temp_traj_cspec.InsertDeltaTime(waypoint, delta_time)
    temp_traj.Insert(0, waypoint)

    # Get the resolution (in radians) for each joint
    q_resolutions = robot.GetDOFResolutions()[dof_indices]

    # Iterate over each segment in the trajectory and set
    # the timing of each waypoint in the temporary trajectory
    # so that taking steps of t=1 will be within a required error norm.
    for i in range(1, num_waypoints):
        # We already have the first waypoint (q0) of this segment,
        # so get the joint values for the second waypoint.
        waypoint = traj.GetWaypoint(i)
        q1 = traj_cspec.ExtractJointValues(waypoint, robot, dof_indices)
        dq = numpy.abs(q1 - q0)

        # Get the number of steps (as a float) required for
        # each joint at DOF resolution
        num_steps = dq / q_resolutions

        # Calculate the norm:
        #
        # norm_order = 1  ==>  The L1 norm
        # Which is like a diamond shape in configuration space
        # and equivalent to: L1_norm=sum(num_steps)
        #
        # norm_order = 2  ==>  The L2 norm
        # Which is like an ellipse in configuration space
        # and equivalent to: L2_norm=numpy.linalg.norm(num_steps)
        #
        # norm_order = inf  ==>  The L_infinity norm
        # Which is like a box shape in configuration space
        # and equivalent to: L_inf_norm=numpy.max(num_steps)
        norm = numpy.linalg.norm(num_steps, ord=norm_order)

        # Set timing of this waypoint
        waypoint = traj.GetWaypoint(i, temp_traj_cspec)
        delta_time = norm
        temp_traj_cspec.InsertDeltaTime(waypoint, delta_time)
        temp_traj.Insert(i, waypoint)

        # The last waypoint becomes the first in the next segment
        q0 = q1

    traj_duration = temp_traj.GetDuration()

    # Sample the trajectory using the specified sample generator
    seq = None
    if sampling_func is None:
        # (default) Linear sequence, from start to end
        seq = SampleTimeGenerator(0, traj_duration, step=2)
    else:
        seq = sampling_func(0, traj_duration, step=2)

    # Sample the trajectory in time
    # and return time value and joint positions
    for t in seq:
        sample = temp_traj.Sample(t)
        q = temp_traj_cspec.ExtractJointValues(sample, robot, dof_indices)
        yield t, q


def IsInCollision(traj, robot, selfcoll_only=False):
    report = openravepy.CollisionReport()

    # Get trajectory length.
    NN = traj.GetNumWaypoints()
    ii = 0
    total_dist = 0.0
    for ii in range(NN - 1):
        point1 = traj.GetWaypoint(ii)
        point2 = traj.GetWaypoint(ii + 1)
        dist = 0.0
        total_dof = robot.GetActiveDOF()
        for jj in range(total_dof):
            dist += pow(point1[jj] - point2[jj], 2)
        total_dist += numpy.sqrt(dist)

    step_dist = 0.04

    if traj.GetDuration() < 0.001:
        openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot)
    total_time = traj.GetDuration()
    step_time = total_time * step_dist / total_dist

    for t in numpy.arange(0.0, total_time, step_time):
        point = traj.Sample(t)
        collision = False
        with robot.GetEnv():
            robot.SetActiveDOFValues(point)
            if robot.CheckSelfCollision(report):
                collision = True
            if not collision:
                if ((not selfcoll_only) and
                        robot.GetEnv().CheckCollision(robot, report)):
                    collision = True
        if collision:
            return True

    return False


OPENRAVE_JOINT_DERIVATIVES = {
    0: "joint_values",
    1: "joint_velocities",
    2: "joint_accelerations",
    3: "joint_jerks",
    4: "joint_snaps",
    5: "joint_crackles",
    6: "joint_pops"
}


def GetJointDerivativeGroup(cspec, derivative):
    """
    Helper function to extract a joint derivative group from a trajectory.

    We use a manual mapping of joint derivatives to string values because the
    OpenRAVE source code internally hard codes these constants anyway:
    https://github.com/rdiankov/openrave/blob/master/src/libopenrave/configurationspecification.cpp#L983

    @param cspec a trajectory configurationspecification
    @param derivative the desired joint position derivative
                      (e.g. 0 = positions, 1 = velocities, ...)
    @return a ConfigurationSpecification Group for the derivative
            or None if it does not exist in the specification.
    """
    try:
        return cspec.GetGroupFromName(OPENRAVE_JOINT_DERIVATIVES[derivative])
    except KeyError:
        return None
    except openravepy.openrave_exception:
        return None


def JointStatesFromTraj(robot, traj, times, derivatives=[0, 1, 2]):
    """
    Helper function to extract the joint position, velocity and acceleration
    from an OpenRAVE trajectory.

    @param robot The OpenRAVE robot
    @param traj An OpenRAVE trajectory
    @param times List of times in seconds
    @param derivatives list of desired derivatives defaults to [0, 1, 2]
    @return List of list of derivatives at specified times.
            Inserts 'None' for unavailable or undesired fields
            The i-th element is the derivatives[i]-th derivative
            of position of size |times| x |derivatives|
    """
    if not IsTimedTrajectory(traj):
        raise ValueError("Joint states can only be interpolated"
                         " on a timed trajectory.")

    duration = traj.GetDuration()
    times = numpy.array(times)
    if any(times > duration):
        raise ValueError('Input times {0:} exceed duration {1:.2f}'
                         .format(times, duration))

    cspec = traj.GetConfigurationSpecification()
    num_dofs = robot.GetDOF()
    dof_indices = range(num_dofs)

    pva_list = []
    for t in times:
        pva = [None] * len(derivatives)
        trajdata = traj.Sample(t)
        for i, deriv in enumerate(derivatives):
            pva[i] = cspec.ExtractJointValues(trajdata, robot,
                                              dof_indices, deriv)
        pva_list.append(pva)

    return pva_list


def JointStateFromTraj(robot, traj, time, derivatives=[0, 1, 2]):
    """
    Helper function to extract the joint position, velocity and acceleration
    from an OpenRAVE trajectory.

    @param robot The OpenRAVE robot
    @param traj An OpenRAVE trajectory
    @param time time in seconds
    @param derivatives list of desired derivatives defaults to [0, 1, 2]
    @return List of list of derivatives at specified times.
            Inserts 'None' for unavailable or undesired fields
            The i-th element is the derivatives[i]-th derivative
            of position of size |times| x |derivatives|
    """
    return JointStatesFromTraj(robot, traj, (time,), derivatives)[0]


def BodyPointsStatesFromJointStates(bodypoints,
                                    jointstates,
                                    derivatives=[0, 1, 2]):
    """
    Computes the derivatives body points given jointstates.
    Currently only supports derivatives up to 2.

    @param bodypoints List of bodypoints where each bodypoint
                      is a list comprising of:
                      (1) the OpenRAVE link the bodypoint is on
                      (2) position of the body point in the link frame
    @param jointstates List of list of joint derivatives.
                       Unavailable fields are input as 'None'
    @param derivatives list of desired derivatives defaults to [0, 1, 2]
    @return bodypoint_list List of list of derivatives at specified times.
                           Inserts 'None' for unavailable or undesired fields
                           The i-th element is the derivatives[i]-th derivative
                           of position of size |times| x |derivatives|
    """

    # Convert derivatives to numpy array
    derivatives = numpy.array(derivatives)
    maxd = max(derivatives)
    numd = len(derivatives)

    if any(derivatives > 2):
        raise ValueError("Can only support derivatives up to 2.")

    # Assume everything belongs to the same robot and env
    robot = bodypoints[0][0].manipulator.GetRobot()
    env = robot.GetEnv()

    bpstate_list = []

    with env:
        with robot:
            for js in jointstates:
                # Make all unavailable and undesired derivatives None
                q, qd, qdd = [js[x] if x < len(js) and x <= maxd
                              else None for x in range(3)]
                if q is not None:
                    robot.SetDOFValues(q)
                else:
                    bpstate_list.append([[[None] * numd] * len(bodypoints)])
                    continue
                for bp in bodypoints:
                    bp_state = [None] * numd
                    link, local_pos = bp
                    link_index = link.GetIndex()
                    link_transform = link.GetTransform()
                    world_pos = (numpy.dot(link_transform[0:3, 0:3],
                                           local_pos) +
                                 link_transform[0:3, 3])
                    bp_state[0] = world_pos
                    if qd is not None:
                        Jpos = robot.CalculateJacobian(link_index, world_pos)
                        Jang = robot.CalculateAngularVelocityJacobian(
                            link_index)
                        vpos = numpy.dot(Jpos, qd)
                        vang = numpy.dot(Jang, qd)
                        bp_state[1] = numpy.hstack((vpos, vang))
                    else:
                        continue
                    if qdd is not None:
                        Hpos = robot.ComputeHessianTranslation(
                            link_index, world_pos)
                        Hang = robot.ComputeHessianAxisAngle(link_index)
                        apos = (numpy.dot(Jpos, qdd) +
                                numpy.dot(qd, numpy.dot(Hpos, qd)))
                        aang = (numpy.dot(Jang, qdd) +
                                numpy.dot(qd, numpy.dot(Hang, qd)))
                        bp_state[2] = numpy.hstack((apos, aang))
                bpstate_list.append(bp_state)
    return bpstate_list


def BodyPointsStatesFromJointState(bodypoints, jointstate,
                                   derivatives=[0, 1, 2]):
    """
    Computes the pos, vel, acc of body points given the
    pos, vel, acc of jointstates.

    @param bodypoints List of bodypoints where each bodypoint
                      is a list comprising of:
                      (1) the OpenRAVE link the bodypoint is on
                      (2) position of the body point in the link frame
    @param jointstate List of joint position,
                      velocity and acceleration.
                      Unavailable fields are input as 'None'
    @param derivatives list of desired derivatives defaults to [0, 1, 2]
    @return bodypoint_list List of list of derivatives at specified times.
                           Inserts 'None' for unavailable or undesired fields
                           The i-th element is the derivatives[i]-th derivative
                           of position of size |times| x |derivatives|
    """
    return BodyPointsStatesFromJointStates(bodypoints, (jointstate,),
                                           derivatives)[0]


def BodyPointsStatesFromTraj(bodypoints, traj, times, derivatives=[0, 1, 2]):
    """
    Computes the pos, vel, acc of body points from a joint space trajectory
    at specified times.

    @param bodypoints List of bodypoints where each bodypoint
                      is a list comprising of:
                      (1) the OpenRAVE link the bodypoint is on
                      (2) position of the body point in the link frame
    @param traj An OpenRAVE trajectory
    @param time List of times in seconds
    @param derivatives list of desired derivatives defaults to [0, 1, 2]
    @return bodypoint_list List of list of derivatives at specified times.
                           Inserts 'None' for unavailable or undesired fields
                           The i-th element is the derivatives[i]-th derivative
                           of position of size |times| x |derivatives|
    """
    # Assume everything belongs to the same robot
    robot = bodypoints[0][0].manipulator.GetRobot()
    jointstates = JointStatesFromTraj(robot, traj, times,
                                      range(max(derivatives)))

    return BodyPointsStatesFromJointStates(bodypoints, jointstates,
                                           derivatives)


def BodyPointsStateFromTraj(bodypoints, traj, time, derivatives=[0, 1, 2]):
    """
    Computes the pos, vel, acc of body points from a joint space trajectory
    at a specified time.

    @param bodypoints List of bodypoints where each bodypoint
                      is a list comprising of:
                      (1) the OpenRAVE link the bodypoint is on
                      (2) position of the body point in the link frame
    @param traj An OpenRAVE trajectory
    @param derivatives list of desired derivatives defaults to [0, 1, 2]
    @return bodypoint_list List of list of derivatives at specified times.
                           Inserts 'None' for unavailable or undesired fields
                           The i-th element is the derivatives[i]-th derivative
                           of position of size |times| x |derivatives|
    """
    return BodyPointsStatesFromTraj(bodypoints, traj, (time,), derivatives)[0]


def wrap_to_interval(angles, lower=-numpy.pi):
    """
    Wraps an angle into a semi-closed interval of width 2*pi.

    By default, this interval is `[-pi, pi)`.  However, the lower bound of the
    interval can be specified to wrap to the interval `[lower, lower + 2*pi)`.
    If `lower` is an array the same length as angles, the bounds will be
    applied element-wise to each angle in `angles`.

    See: http://stackoverflow.com/a/32266181

    @param angles an angle or 1D array of angles to wrap
    @type  angles float or numpy.array
    @param lower optional lower bound on wrapping interval
    @type  lower float or numpy.array
    """
    return (angles - lower) % (2 * numpy.pi) + lower


def GetManipulatorIndex(robot, manip=None):
    """
    Takes a robot and returns the active manipulator and its index.

    @param robot The OpenRAVE robot
    @param manip The robot manipulator
    @return (manip, manip_idx) The manipulator and its index
    """

    with robot.GetEnv():
        if manip is None:
            manip = robot.GetActiveManipulator()

        with robot.CreateRobotStateSaver(
                robot.SaveParameters.ActiveManipulator):
            robot.SetActiveManipulator(manip)
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    return (manip, manip_idx)


def GetPointFrom(focus):
    """
    Given a kinbody, array or transform, returns the xyz location.

    @param focus The area to be referred to
    """
    # Pointing at a kinbody
    if isinstance(focus, openravepy.KinBody):
        with focus.GetEnv():
            focus_trans = focus.GetTransform()
        coord = list(focus_trans[0:3, 3])

    # Pointing at a kinbody link
    elif isinstance(focus, openravepy.KinBody.Link):
        with focus.GetParent().GetEnv():
            focus_trans = focus.GetTransform()
        coord = list(focus_trans[0:3, 3])

    # Pointing at a point in space as numpy array
    elif (isinstance(focus, numpy.ndarray) and
          (focus.ndim == 1) and (len(focus) == 3)):
        coord = list(focus)

    # Pointing at point in space as 4x4 transform
    elif isinstance(focus, numpy.ndarray) and (focus.shape == (4, 4)):
        coord = list(focus[0:3, 3])

    # Pointing at a point in space as list or tuple
    elif (isinstance(focus, (tuple, list)) and len(focus) == 3):
        coord = focus

    else:
        raise ValueError('Focus of the point is an unknown object')

    return coord
