#!/usr/bin/env python

import functools
        
from base import PerceptionModule, PerceptionMethod

class MetaModule(PerceptionModule)
    def __init__(self, modules):
        super(PerceptionModule, self).__init__()
        self.modules = modules;
    
    def __str__(self):
        return 'MetaModule'
        
class RunAll(MetaModule)
    def __init__(self, modules):
        super(MetaModule, self).__init__(modules)
    
    def __str__(self):
        return 'RunAll'
    
    @PerceptionMethod
    def DetectObjects(self, robot, **kw_args):
        for module in modules:
            module.DetectObjects(robot, kw_args);
