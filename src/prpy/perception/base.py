#!/usr/bin/env python

import functools

class PerceptionMethod(object):
    
    def __init__(self, func):
        self.func = func

    def __call__(self, instance, robot, *args, **kw_args):
        return self.func(instance, robot, *args, **kw_args)
            
    def __get__(self, instance, instancetype):
        # Bind the self reference and use update_wrapper to propagate the
        # function's metadata (e.g. name and docstring).
        wrapper = functools.partial(self.__call__, instance)
        functools.update_wrapper(wrapper, self.func)
        wrapper.is_perception_method = True
        return wrapper
        
class PerceptionModule(object):
    def has_perception_method(self, method_name):
        """
        Check if this module has the desired PerceptionMethod
        """
        if hasattr(self, method_name):
            method = getattr(self, method_name)
            if hasattr(method, 'is_perception_method'):
                return method.is_perception_method
            else:
                return False
        else:
            return False

    def get_perception_method_names(self):
        """
        @return A list of all the PerceptionMethod functions
           defined for this module
        """
        return filter(lambda method_name: self.has_perception_method(method_name), dir(self))

