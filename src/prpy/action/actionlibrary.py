#!/usr/bin/env python

class ActionError(Exception):
    pass

class ActionMethod(object):

    def __init__(self, func):
        """
        Register this action with the action library
        """
        self.func = func
        ActionLibrary.add_action(func)

    def __call__(self, instance, robot, *args, **kw_args):
        self.func(instance, robot, *args, **kw_args)

class ActionLibrary(object):
    actions = []

    def has_action(self, name):
        all_actions = self.get_actions()
        return name in all_actions

    def get_action(self, name):
        for a in self.actions:
            if a.__name__ == name:
                return a
        return None

    def get_actions(self):
        """
        Return all the action names registered
        """
        return [action.__name__ for action in self.actions]

    @classmethod
    def add_action(cls, func):
        """
        @param func The action method to register
        """
        cls.actions.append(func)
    
