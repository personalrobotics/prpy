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

import logging, numpy, openravepy, scipy.misc, time, threading, math
import scipy.optimize

def create_sensor(env, args, anonymous=True):
    sensor = openravepy.RaveCreateSensor(env, args)
    if sensor is None:
        raise Exception("Creating '%s' sensor failed." % args.split()[0])

    env.Add(sensor, anonymous)
    return sensor

def HasAffineDOFs(cspec):
    def has_group(cspec, group_name):
        try:
            cspec.GetGroupFromName(group_name)
            return True
        except openravepy.openrave_exception:
            return False

    return (has_group(cspec, 'affine_transform')
         or has_group(cspec, 'affine_velocities')
         or has_group(cspec, 'affine_accelerations'))

def GetTrajectoryIndices(traj):
    try:
        joint_values_group = traj.GetConfigurationSpecification().GetGroupFromName('joint_values')
        return numpy.array([ int(index) for index in joint_values_group.name.split()[2:] ])
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

def TakeSnapshot(viewer, path=None, show_figures=True, width=1920, height=1080, fx=640, fy=640):
    if isinstance(viewer, openravepy.Environment):
        viewer = viewer.GetViewer()

    viewer.SendCommand('SetFiguresInCamera {0:d}'.format(show_figures))
    image = viewer.GetCameraImage(width, height, viewer.GetCameraTransform(),
                                  [ fx, fy, width/2, height/2 ])

    if path is not None:
        scipy.misc.imsave(path, image)

    return image

def ComputeAinv(N,dof):
    dt = 1.0/(N-1)
    K=numpy.mat(numpy.zeros((N-1,N-1)))
    for i in range(1,N-1):
        K[i,i]=1/dt;
        K[i,i-1]=-1/dt;
    K[0,0]=1/dt;
    A=K.transpose()*K
    invA_small=numpy.linalg.inv(A)
    
    #tensorize
    invA = numpy.mat(numpy.zeros([(N)*dof,(N)*dof]))
    for i in range(1,N):
        for j in range(1,N):
            for k in range(dof):
                invA[i*dof+k,j*dof+k] = invA_small[i-1,j-1]
    return invA
    
def MatrixToTraj(traj_matrix,cs,dof,robot):
    env = robot.GetEnv()
    traj = openravepy.RaveCreateTrajectory(env,'')
    traj.Init(cs)
    for i in range(numpy.size(traj_matrix)/dof):
        tp = traj_matrix[range(i*dof,i*dof+dof)]
        tp = numpy.array(tp.transpose())[0]
        traj.Insert(i,tp)
    openravepy.planningutils.RetimeActiveDOFTrajectory(traj,robot,False,0.2,0.2,"LinearTrajectoryRetimer","")
    return traj
    
def TrajToMatrix(traj,dof):
    traj_matrix = numpy.zeros(dof*(traj.GetNumWaypoints()))
    traj_matrix = numpy.mat(traj_matrix).transpose()
    for i in range(traj.GetNumWaypoints()):
        d = traj.GetWaypoint(i)[range(dof)]
        d = numpy.mat(d).transpose()
        traj_matrix[range(i*dof,(i+1)*dof)]=d
    return traj_matrix

def AdaptTrajectory(traj, new_start, new_goal, robot):
    """
    Adapt an existing trajectory to move between a new start and goal. The
    trajectory's configuration specification must contain exactly one group
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
    goal = traj.GetWaypoint(traj.GetNumWaypoints()-1)
    goal = goal[range(dof)]

    traj_matrix = TrajToMatrix(traj,dof)

    #translate traj to match start
    diff_start = new_start - start
    diff_start = numpy.mat(diff_start).transpose()
    translated_traj = numpy.zeros(dof*(traj.GetNumWaypoints()))
    translated_traj = numpy.mat(translated_traj).transpose()
    for i in range(traj.GetNumWaypoints()):
        translated_traj[range((i-1)*dof,i*dof)] = traj_matrix[range((i-1)*dof,i*dof)]-diff_start

    #apply goal correction
    new_traj_matrix = translated_traj
    N = traj.GetNumWaypoints()
    goal_translated = new_traj_matrix[range((N-1)*dof,(N)*dof)]
    Ainv = ComputeAinv(N,dof)
    goal_diff = numpy.mat(new_goal).transpose() - goal_translated
    traj_diff = numpy.zeros(dof*(N))
    traj_diff = numpy.mat(traj_diff).transpose()
    traj_diff[range((N-1)*dof,(N)*dof)] = goal_diff
    new_traj_matrix = new_traj_matrix+Ainv*traj_diff/Ainv[N*dof-1,N*dof-1]
    
    new_traj = MatrixToTraj(new_traj_matrix,cs,dof,robot)
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
    return copy_traj


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

def IsInCollision(traj, robot, selfcoll_only=False):
    report = openravepy.CollisionReport()

    #get trajectory length
    NN = traj.GetNumWaypoints()
    ii = 0
    total_dist = 0.0
    for ii in range(NN-1):
        point1 = traj.GetWaypoint(ii)
        point2 = traj.GetWaypoint(ii+1)
        dist = 0.0
        total_dof = robot.GetActiveDOF()
        for jj in range(total_dof):
            dist += pow(point1[jj]-point2[jj],2)
        total_dist += numpy.sqrt(dist)
    step_dist = 0.04
    if traj.GetDuration()<0.001:
        openravepy.planningutils.RetimeActiveDOFTrajectory(traj,robot)
    total_time = traj.GetDuration()
    step_time = total_time*step_dist/total_dist

    #check
    for time in numpy.arange(0.0,total_time,step_time):
        point = traj.Sample(time)
        collision = False
        with robot.GetEnv():
            robot.SetActiveDOFValues(point)
            if robot.CheckSelfCollision(report):
                collision = True
            if not collision:
                if  (not selfcoll_only) and robot.GetEnv().CheckCollision(robot,report):
                    collision = True
        if collision:
            return True        
                
    return False    
            


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
        cmd = 'Start {width:d} {height:d} 30 codec {codec:d} timing realtime filename {filename:s}'\
              '\nviewer {viewer:s}'.format(
            width = self.width,
            height = self.height,
            codec = self.codec,
            filename = self.filename,
            viewer = self.env.GetViewer().GetName()
        )
        self.module.SendCommand(cmd)

    def stop(self):
        self.module.SendCommand('Stop')

class AlignmentToken(object):
    def __init__(self, env, child_frame, extents, pose=None, period=0.05, parent_frame='world'):
        self.child_frame = child_frame
        self.parent_frame = parent_frame
        self.period = period

        with env:
            self.body = openravepy.RaveCreateKinBody(env, '')
            aabbs = numpy.concatenate(([ 0., 0., 0. ], extents)).reshape((1, 6))
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
        orientation = (or_quaternion[1], or_quaternion[2], or_quaternion[3], or_quaternion[0])
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
                self.message, self.get_duration()
            )

    def stop(self):
        self.end = time.time()

    def get_duration(self):
        return self.end - self.start


def quadraticObjective(dq, *args):
    '''
    Quadratic objective function for SciPy's optimization.
    @param dq joint velocity
    @param args[0]Jacobian
    @param args[1] desired twist
    @return objective the objective function
    @return gradient the analytical gradient of the objective
    '''
    J = args[0]
    dx = args[1]
    error = (numpy.dot(J, dq) - dx)
    objective = 0.5*numpy.dot(numpy.transpose(error), error)
    gradient = numpy.dot(numpy.transpose(J), error)
    return objective, gradient


def ComputeJointVelocityFromTwist(robot, twist,
                                  objective=quadraticObjective,
                                  dq_init=None):
    '''
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
    @return dq_opt optimal joint velocity
    @return twist_opt actual achieved twist
            can be different from desired twist due to constraints
    '''
    manip = robot.GetActiveManipulator()
    robot.SetActiveDOFs(manip.GetArmIndices())

    jacobian_spatial = manip.CalculateJacobian()
    jacobian_angular = manip.CalculateAngularVelocityJacobian()
    jacobian = numpy.vstack((jacobian_spatial, jacobian_angular))

    rows = [i for i, x in enumerate(twist) if math.isnan(x) is False]
    twist_active = twist[rows]
    jacobian_active = jacobian[rows, :]

    bounds = [(-x, x) for x in robot.GetActiveDOFMaxVel()]
    # Check for joint limits
    q_curr = robot.GetActiveDOFValues()
    q_min, q_max = robot.GetActiveDOFLimits()
    dq_bounds = [(0, max) if (numpy.isclose(q_curr[i], q_min[i])) else
                 (min, 0) if (numpy.isclose(q_curr[i], q_max[i])) else
                 (min, max) for i, (min, max) in enumerate(bounds)]

    if dq_init is None:
        dq_init = robot.GetActiveDOFVelocities()

    opt = scipy.optimize.fmin_l_bfgs_b(objective, dq_init, fprime=None,
                                       args=(jacobian_active, twist_active),
                                       bounds=dq_bounds, approx_grad=False)

    dq_opt = opt[0]
    twist_opt = numpy.dot(jacobian, dq_opt)

    return dq_opt, twist_opt


def GeodesicTwist(t1, t2):
    '''
    Computes the twist in global coordinates that corresponds
    to the gradient of the geodesic distance between two transforms.

    @param t1 current transform
    @param t2 goal transform
    @return twist in se(3)
    '''
    trel = numpy.dot(numpy.linalg.inv(t1), t2)
    trans = numpy.dot(t1[0:3, 0:3], trel[0:3, 3])
    omega = numpy.dot(t1[0:3, 0:3],
                      openravepy.axisAngleFromRotationMatrix(
                        trel[0:3, 0:3]))
    return numpy.hstack((trans, omega))


def GeodesicError(t1, t2):
    '''
    Computes the error in global coordinates between two transforms

    @param t1 current transform
    @param t2 goal transform
    @return a 4-vector of [dx, dy, dz, solid angle]
    '''
    trel = numpy.dot(numpy.linalg.inv(t1), t2)
    trans = numpy.dot(t1[0:3, 0:3], trel[0:3, 3])
    omega = openravepy.axisAngleFromRotationMatrix(trel[0:3, 0:3])
    angle = numpy.linalg.norm(omega)
    return numpy.hstack((trans, angle))


def GeodesicDistance(t1, t2, r=1.0):
    '''
    Computes the geodesic distance between two transforms

    @param t1 current transform
    @param t2 goal transform
    @param r in units of meters/radians converts radians to meters
    '''
    error = GeodesicError(t1, t2)
    error[3] = r*error[3]
    return numpy.linalg.norm(error)
