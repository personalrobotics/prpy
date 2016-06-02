#!/usr/bin/env python

import functools

class TrackingException(Exception):
    pass

class TrackingMethod(object):

    def __init__(self, func):
        self.func = func

    def __call__(self,instance,robot,*args,**kw_args):
        return self.func(instance, robot, *args, **kw_args)

    def __get__(self, instance, instancetype):
        wrapper = functools.partial(self.__call__, instance)
        functools.update_wrapper(wrapper, self.func)
        wrapper.is_tracking_method = True
        return wrapper

class TrackingModule(object):
    def has_tracking_method(self, method_name):
        """
        Check if this module has the desired TrackingMethod
        """
        if hasattr(self, method_name):
            method = getattr(self, method_name)
            if hasattr(method, 'is_tracking_method'):
                return method.is_tracking_method
            else:
                return False
        else:
            return False

    def get_tracking_method_names(self):
        """
        @return A list of all the TrackingMethod functions
           defined for this module
        """
        return filter(lambda method_name: self.has_tracking_method(method_name), dir(self))

