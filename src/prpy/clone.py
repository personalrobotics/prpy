import openravepy, threading

class CloneException(Exception):
    pass

class ClonedEnvironment(openravepy.Environment):
    local = threading.local()

    def __enter__(self):
        self.__class__.get_envs().append(self)

    def __exit__(self, *args):
        self.__class__.get_envs().pop()
        if self.destroy_on_exit:
            self.Destroy()

    @classmethod
    def get_env(cls):
        environments = cls.get_envs()
        if not environments:
            raise CloneException('Cloned environments may only be used in a Clone context.')
        return environments[-1]

    @classmethod
    def get_envs(cls):
        if not hasattr(cls.local, 'environments'):
            cls.local.environments = list()
        return cls.local.environments

def Clone(env, destroy=True):
    clone_env = openravepy.Environment()
    clone_env.Clone(env, openravepy.CloningOptions.Bodies)
    clone_env.__class__ = ClonedEnvironment
    clone_env.destroy_on_exit = destroy
    clone_env.parent = env
    return clone_env

def Cloned(*instances):
    clone_env = CloneImpl.get_env()
    clone_instances = list()

    for instance in instances:
        if isinstance(instance, openravepy.Robot):
            clone_instance = clone_env.GetRobot(instance.GetName())
        elif isinstance(instance, openravepy.KinBody):
            clone_instance = clone_env.GetKinBody(instance.GetName())
        elif isinstance(instance, openravepy.KinBody.Link):
            clone_instance = Cloned(instance.GetParent()).GetLink(instance.GetName())
        else:
            raise CloneException('Unable to clone object of type {0:s}.'.format(type(instance)))

        if clone_instance is None:
            raise CloneException('{0:s} is not in the cloned environment.'.format(instance))

        clone_instance.parent = instance
        clone_instances.append(clone_instance)

    if len(instances) == 1:
        return instances[0]
    else:
        return clone_instances
