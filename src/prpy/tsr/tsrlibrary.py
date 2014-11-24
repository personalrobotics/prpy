import collections, functools, logging, os.path

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
    all_factories = collections.defaultdict(lambda: collections.defaultdict(dict))
    
    def __init__(self, robot, robot_name=None):
        """
        Create a TSRFactory for a robot.
        @param robot the robot to store TSRs for
        @param robot_name optional robot name, inferred from robot by default
        """
        self.robot = robot

        if robot_name is not None:
            self.robot_name = robot_name
        else:
            self.robot_name = self.get_object_type(robot)
            logger.debug('Inferred robot name "%s" for TSRLibrary.', self.robot_name)

    def __call__(self, kinbody, action_name, *args, **kw_args):
        """
        Creates a TSR to perform an action on an object with this robot. Raises
        KeyError if no matching factory exists.
        @param robot The robot to run the tsr on 
        @param kinbody The kinbody to act on
        @param action_name The name of the action
        @return list of TSRChains
        """
        kinbody_name = kw_args.get('kinbody_name', None)
        if kinbody_name is None:
            kinbody_name = self.get_object_type(kinbody)
            logger.debug('Inferred KinBody name "%s" for TSR.', kinbody_name)

        try:
            f = self.all_factories[self.robot_name][kinbody_name][action_name]
        except KeyError:
            raise KeyError('There is no TSR factory registered for action "{:s}"'
                           ' with robot "{:s}" and object "{:s}".'.format(
                           action_name, self.robot_name, kinbody_name))

        return f(self.robot, kinbody, *args, **kw_args)

    @classmethod
    def add_factory(cls, func, robot_name, object_name, action_name):
        """
        Register a TSR factory to perform an action on an object with this robot.
        @param robot_name name of the robot
        @param object_name name of the object
        @param action_name name of the action
        """
        logger.debug('Adding TSRLibrary factory for robot "%s", object "%s", action "%s".',
            robot_name, object_name, action_name)

        if action_name in cls.all_factories[robot_name][object_name]:
            logger.warning('Overwriting duplicate TSR factory for action "%s"'
                           ' with robot "%s" and object "%s"',
                action_name, robot_name, object_name)

        cls.all_factories[robot_name][object_name][action_name] = func

    @staticmethod
    def get_object_type(body):
        """
        Infer the name of a KinBody by inspecting its GetXMLFilename.
        @param body KinBody or Robot object
        @return object string
        """
        path = body.GetXMLFilename()
        filename = os.path.basename(path)
        name, _ = os.path.splitext(filename) # remove xml
        name, _ = os.path.splitext(name) # remove kinbody, robot, etc

        if name:
            return name
        else:
            raise ValueError('Failed inferring object name from path: {:s}'.format(path))
