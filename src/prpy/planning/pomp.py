import logging
import numpy
import openravepy
import os
from .. import util
from ..collision import (
    BakedRobotCollisionCheckerFactory,
    DefaultRobotCollisionCheckerFactory,
    SimpleRobotCollisionCheckerFactory,
)
from base import BasePlanner, PlanningError, ClonedPlanningMethod
from IPython import embed

class POMPPlanner(BasePlanner):
        
    def __init__(self,robot_checker_factory=None):

        super(POMPPlanner,self).__init__()

        from catkin.find_in_workspaces import find_in_workspaces
        package_name = 'prpy'
        directory = 'data/roadmaps' # To obtain stored herb roadmaps
        objects_path = find_in_workspaces(
            search_dirs=['share'],
            project=package_name,
            path=directory,
            first_match_only = True)
        if len(objects_path) == 0:
            print('Can\'t find directory %s/%s' % (package_name, directory))
            sys.exit()
        else:
            objects_path = objects_path[0]
            roadmap_file = os.path.join(objects_path,'herb_halton_10000_2.graphml') # A good default roadmap
            self.reqd_model_params = {'prior':0.5,'prior_weight':20,'support':2.0,'knn':15}
            self.reqd_roadmap_params = {'roadmap_type':'fromfile', 'filename':roadmap_file,'gen_roadmap':1}

        if robot_checker_factory is None:
            robot_checker_factory = DefaultRobotCollisionCheckerFactory

        self.robot_checker_factory = robot_checker_factory
        if isinstance(robot_checker_factory, SimpleRobotCollisionCheckerFactory):
            self._is_baked = False
        elif isinstance(robot_checker_factory, BakedRobotCollisionCheckerFactory):
            self._is_baked = True
        else:
            raise NotImplementedError(
                'POMP only supports Simple and BakedRobotCollisionChecker.')

        self.planner = openravepy.RaveCreatePlanner(self.env,'OMPL_POMP')
        if self.planner is None:
            raise prpy.planning.base.UnsupportedPlanningError('Unable to create POMP planner')

    def __str__(self):
        return 'POMP'

    def xml_from_params(self,**kw_args):
        xml = list()
        for k,v in kw_args.items():
            xml.append('<{k}>{v}</{k}>'.format(k=k,v=v)) 

        # Add default model params if not in args
        for k,v in self.reqd_model_params.items():
            if k not in kw_args.keys():
                xml.append('<{k}>{v}</{k}>'.format(k=k,v=v))

        # Add default roadmap params if not in args
        ordered_roadmap_param_keys = ['roadmap_type','filename','gen_roadmap']
        for k in ordered_roadmap_param_keys:
            v = self.reqd_roadmap_params[k]
            xml.append('<{k}>{v}</{k}>'.format(k=k,v=v))
        print xml
        return xml

    @ClonedPlanningMethod
    def PlanToConfiguration(self,robot,goal_config,**kw_args):

        orparams = openravepy.Planner.PlannerParameters()
        orparams.SetRobotActiveJoints(robot)
        orparams.SetGoalConfig(goal_config)
        xml = self.xml_from_params(**kw_args)
        orparams.SetExtraParameters('\n'.join(xml))
        self.planner.InitPlan(robot, orparams)

        traj = openravepy.RaveCreateTrajectory(self.env, '')
        status = self.planner.PlanPath(traj)
        if status == openravepy.PlannerStatus.HasSolution:
            return traj
        else:
            raise prpy.planning.base.PlanningError('POMP status: {}'.format(status))
