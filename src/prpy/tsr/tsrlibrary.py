import functools, logging

logger = logging.getLogger('tsr')

class TSRFactory(object):
    
    def __init__(self, robot_name, obj_name, action_name):
        print 'Loading %s, %s, %s' % (robot_name, obj_name, action_name)
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
    
    def __init__(self, robot):
        self.robot = robot

    def __call__(self, kinbody, action_name, *args, **kw_args):
        '''
        Calls the appropriate factory. Raise key error if no matching factory exists
        @param robot The robot to run the tsr on 
        @param kinbody The kinbody to act on
        @param action_name The name of the action
        '''
        robot_name = str(self.robot.GetName()) # TODO: replace with Mike's xmlid parsing
        kinbody_name = str(kinbody.GetName()) #TODO: replace with Mike's xmlid parsing
        f = self.all_factories[robot_name][kinbody_name][action_name]
        return f(self.robot, kinbody, *args, **kw_args)


    @classmethod
    def add_factory(cls, func, robot_name, obj_name, action_name):
        print 'Adding factory for %s, %s, %s' % (robot_name, obj_name, action_name)
        if robot_name not in cls.all_factories:
            cls.all_factories[robot_name] = {}
        if obj_name not in cls.all_factories[robot_name]:
            cls.all_factories[robot_name][obj_name] = {}
        cls.all_factories[robot_name][obj_name][action_name] = func
