import functools, logging, os.path

logger = logging.getLogger('tsr')

class TSRFactory(object):
    
    def __init__(self, robot_name, obj_name, action_name):
        logger.debug('Loading %s, %s, %s' % (robot_name, obj_name, action_name))
        self.robot_name = robot_name
        self.obj_name = obj_name
        self.action_name = action_name

    def __call__(self, func):
        TSRLibrary.add_factory(func, self.robot_name, self.obj_name, self.action_name)

        #functools.wrap
        from functools import wraps
        @wraps(func)
        def wrapped_func(robot, kinbody, *args, **kw_args):
            return func( robot, kinbody, *args, **kw_args)
        return wrapped_func


class TSRLibrary(object):
    all_factories = {}
    
    def __init__(self, robot, robot_name=None):
        """
        Create a TSRFactory for a robot.
        @param robot the robot to store TSRs for
        @param robot_name optional robot name, inferred from robot by default
        """
        self.robot = robot

        if self.robot_name is not None:
            self.robot_name = robot_name
        else:
            self.robot_name = self.get_object_type(self.robot_name)
            logger.debug('Inferred robot name "%s" for TSRLibrary.', self.robot_name)

    def __call__(self, kinbody, action_name, *args, **kw_args):
        """
        Calls the appropriate factory. Raise key error if no matching factory exists
        @param robot The robot to run the tsr on 
        @param kinbody The kinbody to act on
        @param action_name The name of the action
        @return list of TSRChains
        """
        kinbody_name = kw_args.get('kinbody_name', None)
        if kinbody_name is None:
            kinbody_name = self.get_object_type(kinbody)
            logger.debug('Inferred KinBody name "%s" for TSR.', kinbody_name)

        f = self.all_factories[self.robot_name][kinbody_name][action_name]
        return f(self.robot, kinbody, *args, **kw_args)

    @classmethod
    def add_factory(cls, func, robot_name, obj_name, action_name):
        logger.debug('Adding factory for %s, %s, %s' % (robot_name, obj_name, action_name))
        if robot_name not in cls.all_factories:
            cls.all_factories[robot_name] = {}
        if obj_name not in cls.all_factories[robot_name]:
            cls.all_factories[robot_name][obj_name] = {}
        cls.all_factories[robot_name][obj_name][action_name] = func

    @staticmethod
    def get_object_type(body):
        path = body.GetXMLFilename()
        filename = os.path.basename(path)
        name, _ = os.path.splitext(filename)

        if name:
            return name
        else:
            raise ValueError('Failed inferring object name from path: {:s}'.format(path))
