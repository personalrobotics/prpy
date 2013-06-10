class InstanceDeduplicator:
    instances = dict()

    @staticmethod
    def intercept(self, name):
        # Return the canonical reference stored in this class.
        try:
            true_instance = object.__getattribute__(self, '_true_instance')
        except AttributeError:
            # Retrieve the canonical instance from the global dictionary if it is
            # not already set. This should only occur once, after which the
            # _true_instance field is populated.
            if self in InstanceDeduplicator.instances:
                true_instance = InstanceDeduplicator.instances[self]
                self._true_instance = true_instance
            # There is no canonical instance associated with the object.
            else:
                true_instance = self

        return object.__getattribute__(true_instance, name)

    @classmethod
    def add_canonical(cls, instance):
        cls.instances[instance] = instance
        instance.__class__.__getattribute__ = cls.intercept

def bind_subclass(instance, subclass, *args, **kw_args):
    # Deduplicate the classes associated with any of the objects that we're
    # modifying at runtime. This is necessary for the modifications to be
    # available on instances returned by C++.
    InstanceDeduplicator.add_canonical(instance)
    instance.__class__ = subclass
    instance.__init__(*args, **kw_args)
