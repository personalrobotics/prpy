import logging, numpy, openravepy, os, tempfile
import prrave.kin
from base import BasePlanner, PlanningError, UnsupportedPlanningError, PlanningMethod

class CBiRRTPlanner(BasePlanner):
    def __init__(self):
        self.env = openravepy.Environment()
        try:
            self.problem = openravepy.RaveCreateProblem(self.env, 'CBiRRT')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create CBiRRT module.')

    def __str__(self):
        return 'CBiRRT'

    def Plan(self, robot, smoothingitrs=None, timelimit=None, allowlimadj=0,
             start_config=None, extra_args=None, **kw_args):
        # Take a snapshot of the source environment for planning.
        self.env.LoadProblem(self.problem, robot.GetName())

        args = [ 'RunCBiRRT' ]
        if extra_args is not None:
            args += extra_args
        if smoothingitrs is not None:
            args += [ 'smoothingitrs', str(smoothingitrs) ]
        if timelimit is not None:
            args += [ 'timelimit', str(timelimit) ]
        if allowlimadj is not None:
            args += [ 'allowlimadj', str(int(allowlimadj)) ]
        if start_config is not None:
            args += [ 'jointstarts', str(len(start_config)), ' '.join([ str(x) for x in start_config ]) ]

        # FIXME: Why can't we write to anything other than cmovetraj.txt or
        # /tmp/cmovetraj.txt with CBiRRT?
        traj_path = '/tmp/cmovetraj.txt'
        args += [ 'filename', traj_path ]
        args_str = ' '.join(args)

        response = self.problem.SendCommand(args_str, True)
        if int(response) != 1:
            raise PlanningError('Unknown error.')

        with open(traj_path, 'rb') as traj_file:
            traj_xml = traj_file.read()
            traj = openravepy.RaveCreateTrajectory(self.env, '')
            traj.deserialize(traj_xml)

        return traj

    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        goal_array = numpy.array(goal)

        if len(goal_array) != robot.GetActiveDOF():
            logging.error('Incorrect number of DOFs in goal configuration; expected {0:d}, got {1:d}'.format(
                          robot.GetActiveDOF(), len(goal_array)))
            raise PlanningError('Incorrect number of DOFs in goal configuration.')

        extra_args = list()
        extra_args += [ 'jointgoals',  str(len(goal_array)), ' '.join([ str(x) for x in goal_array ]) ]
        return self.Plan(robot, extra_args=extra_args, **kw_args)

    @PlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, psample=0.1, **kw_args):
        manipulator_index = robot.GetActiveManipulatorIndex()
        goal_tsr = prpy.tsr.TSR(T0_w=goal_pose, manip=manipulator_index)
        tsr_chain = prpy.tsr.TSRChain(sample_goal=True, TSR=goal_tsr)

        extra_args  = [ 'TSRChain', tsr_chain.serialize() ]
        extra_args += [ 'psample', str(psample) ]
        return self.Plan(robot, extra_args=extra_args, **kw_args)

    @PlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance,
                                timelimit=5.0, smoothingitrs=250, **kw_args):
        with robot:
            manip = robot.GetActiveManipulator()
            H_world_ee = manip.GetEndEffectorTransform()

            # 'object frame w' is at ee, z pointed along direction to move
            H_world_w = prrave.kin.H_from_op_diff(H_world_ee[0:3,3], direction)
            H_w_ee = numpy.dot(prrave.kin.invert_H(H_world_w), H_world_ee)
        
            # Serialize TSR string (goal)
            Hw_end = numpy.eye(4)
            Hw_end[2,3] = distance

            goaltsr = prpy.tsr.TSR(T0_w = numpy.dot(H_world_w,Hw_end), 
                                     Tw_e = H_w_ee, 
                                     Bw = numpy.zeros((6,2)), 
                                     manip = robot.GetActiveManipulatorIndex())
            goal_tsr_chain = prpy.tsr.TSRChain(sample_goal = True,
                                                 TSRs = [goaltsr])

            # Serialize TSR string (whole-trajectory constraint)
            Bw = numpy.zeros((6,2))
            epsilon = 0.001
            Bw = numpy.array([[-epsilon,            epsilon],
                              [-epsilon,            epsilon],
                              [min(0.0, distance),  max(0.0, distance)],
                              [-epsilon,            epsilon],
                              [-epsilon,            epsilon],
                              [-epsilon,            epsilon]])

            trajtsr = prpy.tsr.TSR(T0_w = H_world_w, 
                                   Tw_e = H_w_ee, 
                                   Bw = Bw, 
                                   manip = robot.GetActiveManipulatorIndex())
            traj_tsr_chain = prpy.tsr.TSRChain(constrain=True, TSRs=[ trajtsr ])
        
        extra_args = ['psample', '0.1']
        extra_args += [ 'TSRChain', goal_tsr_chain.serialize() ]
        extra_args += [ 'TSRChain', traj_tsr_chain.serialize() ]
        return self.Plan(robot, allowlimadj=True, timelimit=timelimit,
                         smoothingitrs=smoothingitrs,
                         extra_args=extra_args, **kw_args)

    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, **kw_args):
        extra_args = list()
        extra_args += ['psample', '0.1']
        for chain in tsrchains:
            extra_args += [ 'TSRChain', chain.serialize() ]
            
        return self.Plan(robot, extra_args=extra_args, **kw_args)
