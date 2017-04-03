#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
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
import collections
import functools, logging, openravepy, numpy
from .. import bind, named_config, exceptions, util
from ..clone import Clone, Cloned
from tsr.tsrlibrary import TSRLibrary
from ..planning.base import Sequence, Tags
from ..planning.ompl import OMPLSimplifier
from ..planning.retimer import OpenRAVEAffineRetimer, ParabolicRetimer
from ..planning.mac_smoother import MacSmoother
from ..util import SetTrajectoryTags, GetManipulatorIndex

logger = logging.getLogger(__name__)


class Robot(openravepy.Robot):

    _postprocess_envs = collections.defaultdict(openravepy.Environment)
    """
    Mapping from robot environments to plan postprocessing environments.
    """

    def __init__(self, robot_name=None):
        self.actions = None
        self.planner = None
        self.detector = None
        self.robot_name = robot_name

        try:
            self.tsrlibrary = TSRLibrary(self, manipindex=GetManipulatorIndex(self), robot_name=robot_name)
        except ValueError as e:
            self.tsrlibrary = None
            logger.warning('Failed creating TSRLibrary for robot "%s": %s',
                self.GetName(), e.message
            )

        self.controllers = list()
        self.manipulators = list()
        self.configurations = named_config.ConfigurationLibrary()
        self.multicontroller = openravepy.RaveCreateMultiController(self.GetEnv(), '')
        self.SetController(self.multicontroller)

        # Standard, commonly-used OpenRAVE plugins.
        self.base_manipulation = openravepy.interfaces.BaseManipulation(self)
        self.task_manipulation = openravepy.interfaces.TaskManipulation(self)

        # Path post-processing for execution. This includes simplification of
        # the geometric path, retiming a path into a trajectory, and smoothing
        # (joint simplificaiton and retiming).
        self.simplifier = None
        self.retimer = ParabolicRetimer()
        self.smoother = self.retimer
        self.affine_retimer = OpenRAVEAffineRetimer()

    def __dir__(self):
        # We have to manually perform a lookup in InstanceDeduplicator because
        # __methods__ bypass __getattribute__.
        self = bind.InstanceDeduplicator.get_canonical(self)

        # Add planning and action methods to the tab-completion list.
        method_names = set(self.__dict__.keys())

        if hasattr(self, 'planner') and self.planner is not None:
            method_names.update(self.planner.get_planning_method_names())
        if hasattr(self, 'actions') and self.actions is not None:
            method_names.update(self.actions.get_actions())
        if hasattr(self, 'detector') and self.detector is not None:
            method_names.update(self.detector.get_perception_method_names())

        return list(method_names)

    def __getattr__(self, name):
        # We have to manually perform a lookup in InstanceDeduplicator because
        # __methods__ bypass __getattribute__.
        canonical = bind.InstanceDeduplicator.get_canonical(self)

        # For special properties, we do NOT want to recursively look them up.
        # If they are not found on the canonical instance, stop looking.
        if canonical == self:
            if name in ['planner', 'actions', 'detector']:
                raise AttributeError('{0:s} is missing method "{1:s}".'
                                     .format(repr(canonical), name))

        # For other methods, search the special properties for meta-methods.
        if (name != 'planner' and
                hasattr(canonical, 'planner') and
                canonical.planner is not None and
                canonical.planner.has_planning_method(name)):

            delegate_method = getattr(canonical.planner, name)

            @functools.wraps(delegate_method)
            def wrapper_method(*args, **kw_args):
                return canonical._PlanWrapper(delegate_method, args, kw_args)

            return wrapper_method
        elif (name != 'actions' and
                hasattr(canonical, 'actions') and
                canonical.actions is not None and
                canonical.actions.has_action(name)):

            delegate_method = canonical.actions.get_action(name)

            @functools.wraps(delegate_method)
            def wrapper_method(*args, **kw_args):
                return delegate_method(canonical, *args, **kw_args)
            return wrapper_method
        elif (name != 'detector' and
                hasattr(canonical, 'detector') and
                canonical.detector is not None and
                canonical.detector.has_perception_method(name)):

            delegate_method = getattr(canonical.detector, name)

            @functools.wraps(delegate_method)
            def wrapper_method(*args, **kw_args):
                return delegate_method(canonical, *args, **kw_args)
            return wrapper_method

        raise AttributeError('{0:s} is missing method "{1:s}".'
                             .format(repr(canonical), name))

    def CloneBindings(self, parent):
        self.planner = parent.planner

        # TODO: This is a bit of a mess. We need to clean this up when we
        # finish the smoothing refactor.
        self.simplifier = parent.simplifier
        self.retimer = parent.retimer
        self.smoother = parent.smoother

        self.robot_name = parent.robot_name
        self.tsrlibrary = parent.tsrlibrary.clone(self)
        self.configurations = parent.configurations

        self.controllers = []
        self.SetController(None)

        self.manipulators = [Cloned(manipulator, into=self.GetEnv())
                             for manipulator in parent.manipulators]

        # TODO: Do we really need this in cloned environments?
        self.base_manipulation = openravepy.interfaces.BaseManipulation(self)
        self.task_manipulation = openravepy.interfaces.TaskManipulation(self)


    def AttachController(self, name, args, dof_indices, affine_dofs, simulated):
        """
        Create and attach a controller to a subset of this robot's DOFs. If
        simulated is False, a controller is created using 'args' and is attached
        to the multicontroller. In simulation mode an IdealController is
        created instead. Regardless of the simulation mode, the multicontroller
        must be finalized before use.

        @param name user-readable name used to identify this controller
        @param args real controller arguments
        @param dof_indices controlled joint DOFs
        @param affine_dofs controleld affine DOFs
        @param simulated simulation mode
        @returns created controller
        """
        if simulated:
            args = 'IdealController'

        delegate_controller = openravepy.RaveCreateController(self.GetEnv(), args)
        if delegate_controller is None:
            type_name = args.split()[0]
            message = 'Creating controller {0:s} of type {1:s} failed.'.format(name, type_name)
            raise openravepy.openrave_exception(message)

        self.multicontroller.AttachController(delegate_controller, dof_indices, affine_dofs)

        return delegate_controller

    def GetTrajectoryManipulators(self, traj):
        """
        Extract the manipulators that are active in a trajectory. A manipulator
        is considered active if joint values are specified for one or more of its
        controlled DOFs.
        @param traj input trajectory
        @returns list of active manipulators
        """
        traj_indices = set(util.GetTrajectoryIndices(traj))

        active_manipulators = []
        for manipulator in self.GetManipulators():
            manipulator_indices = set(manipulator.GetArmIndices())
            if traj_indices & manipulator_indices:
                active_manipulators.append(manipulator)

        return active_manipulators

    def PostProcessPath(self, path,
                        constrained=None, smooth=None, default_timelimit=None,
                        shortcut_options=None, smoothing_options=None,
                        retiming_options=None, affine_retiming_options=None,
                        **kwargs):
        """ Post-process a geometric path to prepare it for execution.

        This method post-processes a geometric path by (optionally) optimizing
        it and timing it. Four different post-processing pipelines are used:

        1. For base trajectories (i..e affine DOFs), we time the trajectory
           using self.affine_retimer. This function does not currently support
           trajectories that contain both regular and affine DOFs.
        2. For constrained trajectories, we do not modify the geometric path
           and retime the path to be time-optimal via self.retimer. This
           trajectory must stop at every waypoint. The only exception is for...
        3. For smooth trajectories, we attempt to fit a time-optimal smooth
           curve through the waypoints (not implemented). If this curve is
           not collision free, then we fall back on...
        4. By default, we run a smoother that jointly times and smooths the
           path via self.smoother. This algorithm can change the geometric path
           to optimize runtime.

        The behavior in (2) and (3) can be forced by passing constrained=True
        or smooth=True. By default, the case is inferred by the tag(s) attached
        to the trajectory: (1) is triggered by the CONSTRAINED tag and (2) is
        triggered by the SMOOTH tag.

        Options an be passed to each post-processing routine using the
        affine_retiming_options, shortcut_options, smoothing_options, and
        retiming_options **kwargs dictionaries. If no "timelimit" is specified
        in any of these dictionaries, it defaults to default_timelimit seconds.

        @param path un-timed OpenRAVE trajectory
        @param constrained the path is constrained; do not change it
        @param smooth the path is smooth; attempt to execute it directly
        @param default_timelimit timelimit for all operations, if not set
        @param shortcut_options kwargs to self.simplifier
        @param smoothing_options kwargs to self.smoother
        @param retiming_options kwargs to self.retimer
        @param affine_retiming_options kwargs to self.affine_retimer
        @return trajectory ready for execution
        """
        from ..planning.base import Tags
        from ..util import GetTrajectoryTags, CopyTrajectory
        from openravepy import DOFAffine

        # Default parameters.
        if shortcut_options is None:
            shortcut_options = dict()
        if smoothing_options is None:
            smoothing_options = dict()
        if retiming_options is None:
            retiming_options = dict()
        if affine_retiming_options is None:
            affine_retimer_options = dict()

        if default_timelimit is not None:
            shortcut_options.setdefault('timelimit', default_timelimit)
            smoothing_options.setdefault('timelimit', default_timelimit)
            retiming_options.setdefault('timelimit', default_timelimit)
            affine_retimer_options.setdefault('timelimit', default_timelimit)
            warnings.warn('The "default_timelimit" argument is deprecated.',
                DeprecationWarning)

        # Read default parameters from the trajectory's tags.
        tags = GetTrajectoryTags(path)

        if constrained is None:
            constrained = tags.get(Tags.CONSTRAINED, False)
            logger.debug('Detected "%s" tag on trajectory: Setting'
                         ' constrained = True.', Tags.CONSTRAINED)

        if smooth is None:
            smooth = tags.get(Tags.SMOOTH, False)
            logger.debug('Detected "%s" tag on trajectory: Setting smooth'
                         ' = True', Tags.SMOOTH)

        # Since we don't want to endlessly create postprocessing environments,
        # we maintain a map that uniquely associates each OpenRAVE environment
        # with a given postprocessing environment.  This way, if we re-clone
        # into a previously used environment, we will not create a new one.
        postprocess_env = Robot._postprocess_envs[
            openravepy.RaveGetEnvironmentId(self.GetEnv())]

        with Clone(self.GetEnv(),
                   clone_env=postprocess_env) as cloned_env:
            cloned_robot = cloned_env.Cloned(self)

            # Planners only operate on the active DOFs. We'll set any DOFs
            # in the trajectory as active.
            env = path.GetEnv()
            cspec = path.GetConfigurationSpecification()

            used_bodies = cspec.ExtractUsedBodies(env)
            if self not in used_bodies:
                raise ValueError(
                    'Robot "{:s}" is not in the trajectory.'.format(
                        self.GetName()))

            # Extract active DOFs from teh trajectory and set them as active.
            dof_indices, _ = cspec.ExtractUsedIndices(self)

            if util.HasAffineDOFs(cspec):
                affine_dofs = (DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)

                # Bug in OpenRAVE ExtractUsedIndices function makes 
                # dof_indices = affine_dofs. Temporary workaround for that bug.
                dof_indices = []
                logger.warning(
                    'Trajectory contains affine DOFs. Any regular DOFs'
                    ' will be ignored.'
                )
            else:
                affine_dofs = 0

            cloned_robot.SetActiveDOFs(dof_indices, affine_dofs)
            logger.debug(
                'Setting robot "%s" DOFs %s (affine? %d) as active for'
                ' post-processing.',
                cloned_robot.GetName(), list(dof_indices), affine_dofs
            )

            if len(dof_indices) and affine_dofs:
                raise ValueError(
                    'Trajectory contains both affine and regular DOFs.')
            # Special case for timing affine-only trajectories.
            elif affine_dofs:
                traj = self.affine_retimer.RetimeTrajectory(
                    cloned_robot, path, **affine_retimer_options)
            else:
                # The trajectory is constrained. Retime it without changing the
                # geometric path.
                if constrained or smooth:
                    logger.debug('Retiming a smooth or constrained path.')
                    traj = self.retimer.RetimeTrajectory(
                        cloned_robot, path, **retiming_options)
                # The trajectory is not constrained, so we can shortcut it
                # before execution.
                else:
                    if self.simplifier is not None:
                        logger.debug('Shortcutting an unconstrained path.')
                        shortcut_path = self.simplifier.ShortcutPath(
                            cloned_robot, path, **shortcut_options)
                    else:
                        logger.debug('Skipping shortcutting; no simplifier'
                                     ' available.')
                        shortcut_path = path

                    logger.debug('Smoothing an unconstrained path.')
                    traj = self.smoother.RetimeTrajectory(
                        cloned_robot, shortcut_path, **smoothing_options)

        # Copy the trajectory into the output environment.
        output_traj = CopyTrajectory(traj, env=self.GetEnv())
        return output_traj

    def ExecutePath(self, path, **kwargs):
        """ Post-process and execute an un-timed path.

        This method calls PostProcessPath, then passes the result to
        ExecuteTrajectory. Any extra **kwargs are forwarded to both of these
        methods. This function returns the timed trajectory that was executed
        on the robot.

        @param path OpenRAVE trajectory representing an un-timed path
        @param **kwargs forwarded to PostProcessPath and ExecuteTrajectory
        @return timed trajectory executed on the robot
        """
        from ..util import Timer

        logger.info('Post-processing path with %d waypoints.', path.GetNumWaypoints())

        with Timer() as timer:
            traj = self.PostProcessPath(path, **kwargs)
        SetTrajectoryTags(traj, {Tags.POSTPROCESS_TIME: timer.get_duration()}, append=True)

        logger.info('Post-processing took %.3f seconds and produced a path'
                    ' with %d waypoints and a duration of %.3f seconds.',
                    timer.get_duration(),
                    traj.GetNumWaypoints(),
                    traj.GetDuration())

        with Timer() as timer:
            exec_traj = self.ExecuteTrajectory(traj, **kwargs)
        SetTrajectoryTags(exec_traj, {Tags.EXECUTION_TIME: timer.get_duration()}, append=True)
        return exec_traj

    def ExecuteTrajectory(self, traj, timeout=None, period=0.01, **kwargs):
        """ Executes a time trajectory on the robot.

        This function directly executes a timed OpenRAVE trajectory on the
        robot. If you have a geometric path, such as those returned by a
        geometric motion planner, you should first time the path using
        PostProcessPath. Alternatively, you could use the ExecutePath helper
        function to time and execute the path in one function call.

        If timeout = None (the default), this function does not return until
        execution has finished. Termination occurs if the trajectory is
        successfully executed or if a fault occurs (in this case, an exception
        will be raised). If timeout is a float (including timeout = 0), this
        function will return None once the timeout has ellapsed, even if the
        trajectory is still being executed.

        NOTE: We suggest that you either use timeout=None. If trajectory
        execution times out, there is no way to tell whether execution was
        successful or not. Other values of timeout are only supported for
        legacy reasons.

        This function returns the trajectory that was actually executed on the
        robot, including controller error. If this is not available, the input
        trajectory will be returned instead.

        @param traj timed OpenRAVE trajectory to be executed
        @param timeout maximum time to wait for execution to finish
        @param period poll rate, in seconds, for checking trajectory status
        @return trajectory executed on the robot
        """
        # Don't execute trajectories that don't have at least one waypoint.
        if traj.GetNumWaypoints() <= 0:
            raise ValueError('Trajectory must contain at least one waypoint.')

        # Check if this trajectory contains both affine and joint DOFs
        cspec = traj.GetConfigurationSpecification()
        needs_base = util.HasAffineDOFs(cspec)
        needs_joints = util.HasJointDOFs(cspec)
        if needs_base and needs_joints:
            raise ValueError('Trajectories with affine and joint DOFs are not supported')

        # Check that the current configuration of the robot matches the
        # initial configuration specified by the trajectory.
        if not util.IsAtTrajectoryStart(self, traj):
            raise exceptions.TrajectoryNotExecutable(
                'Trajectory started from different configuration than robot.')

        # If there was only one waypoint, at this point we are done!
        if traj.GetNumWaypoints() == 1:
            return traj

        # Verify that the trajectory is timed by checking whether the first
        # waypoint has a valid deltatime value.
        if not util.IsTimedTrajectory(traj):
            raise ValueError('Trajectory cannot be executed, it is not timed.')

        # Verify that the trajectory has non-zero duration.
        if traj.GetDuration() <= 0.0:
            import warnings
            warnings.warn('Executing zero-length trajectory. Please update the'
                          ' function that produced this trajectory to return a'
                          ' single-waypoint trajectory.', FutureWarning)

        self.GetController().SetPath(traj)

        active_manipulators = self.GetTrajectoryManipulators(traj)
        active_controllers = [
            active_manipulator.controller
            for active_manipulator in active_manipulators
            if hasattr(active_manipulator, 'controller')
        ]

        if needs_base:
            if (hasattr(self, 'base') and hasattr(self.base, 'controller')
                    and self.base.controller is not None):
                active_controllers.append(self.base.controller)
            else:
                logger.warning(
                    'Trajectory includes the base, but no base controller is'
                    ' available. Is self.base.controller set?')

        util.WaitForControllers(active_controllers, timeout=timeout)
        return traj

    def ViolatesVelocityLimits(self, traj):
        """
        Checks a trajectory for velocity limit violations
        @param traj input trajectory
        """
        # Get the limits that pertain to this trajectory
        all_velocity_limits = self.GetDOFVelocityLimits()
        traj_indices = util.GetTrajectoryIndices(traj)
        velocity_limits = [all_velocity_limits[idx] for idx in traj_indices]

        # Get the velocity group from the configuration specification so
        #  that we know the offset and number of dofs
        config_spec = traj.GetConfigurationSpecification()
        num_waypoints = traj.GetNumWaypoints()

        # Check for the velocity group
        has_velocity_group = True
        try:
            config_spec.GetGroupFromName('joint_velocities')
        except openravepy.openrave_exception:
            logging.warn('Trajectory does not have joint velocities defined')
            has_velocity_group = False

        # Now check all the waypoints
        for idx in range(0, num_waypoints):

            wpt = traj.GetWaypoint(idx)

            if has_velocity_group:
                # First check the velocities defined for the waypoint
                velocities = config_spec.ExtractJointValues(wpt, self, traj_indices, 1)
                for vidx in range(len(velocities)):
                    if (velocities[vidx] > velocity_limits[vidx]):
                        logging.warn('Velocity for waypoint %d joint %d violates limits (value: %0.3f, limit: %0.3f)' % 
                                     (idx, vidx, velocities[vidx], velocity_limits[vidx]))
                        return True

            # Now check the velocities calculated by differencing positions
            dt = config_spec.ExtractDeltaTime(wpt)
            values = config_spec.ExtractJointValues(wpt, self, traj_indices, 0)

            if idx > 0:
                diff_velocities = numpy.fabs(values - prev_values)/(dt - prev_dt)
                for vidx in range(len(diff_velocities)):
                    if (diff_velocities[vidx] > velocity_limits[vidx]):
                        logging.warn('Velocity for waypoint %d joint %d violates limits (value: %0.3f, limit: %0.3f)' % 
                                     (idx, vidx, diff_velocities[vidx], velocity_limits[vidx]))
                        return True

            # Set current to previous
            prev_dt = dt
            prev_values = values

        return False

    def _PlanWrapper(self, planning_method, args, kw_args):
        config_spec = self.GetActiveConfigurationSpecification('linear')

        # Call the planner.
        from ..util import Timer
        with Timer() as timer:
            result = planning_method(self, *args, **kw_args)
        SetTrajectoryTags(result, {Tags.PLAN_TIME: timer.get_duration()}, append=True)

        # Strip inactive DOFs from the trajectory.
        openravepy.planningutils.ConvertTrajectorySpecification(
            result, config_spec)

        # Optionally execute the trajectory.
        if kw_args.get('execute', False):
            result = self.ExecutePath(result, **kw_args)

        return result
