import logging, numpy, openravepy, os, tempfile
import prrave.kin, prrave.tsr
from base import BasePlanner, PlanningError, UnsupportedPlanningError, PlanningMethod

class OMPLPlanner(BasePlanner):
    def __init__(self, algorithm='RRTConnect'):
        self.env = openravepy.Environment()
        self.algorithm = algorithm
        try:
            self.planner = openravepy.RaveCreatePlanner(self.env, 'OMPL')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create OMPL module.')

    def __str__(self):
        return 'OMPL {0:s}'.format(self.algorithm)

    @PlanningMethod
    def PlanToConfiguration(self, robot, goal, **kw_args):
        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetGoalConfig(goal)

        traj = openravepy.RaveCreateTrajectory(self.env, '')

        with self.env:
            try:
                self.planner.InitPlan(robot, params)
                status = self.planner.PlanPath(traj, releasegil=True)
            except Exception as e:
                raise PlanningError('Planning failed with error: {0:s}'.format(e))

        from openravepy import PlannerStatus
        if status not in [ PlannerStatus.HasSolution, PlannerStatus.InterruptedWithSolution ]:
            raise PlanningError('Planner returned with status {0:s}.'.format(str(status)))

        return traj
