from clone import CloneException

class NotCloneableException(CloneException):
    pass

class InstanceDeduplicator:
    # FIXME: Canonical instances leak memory. They always have a pointer in the
    # instances dictionary.
    instances = dict()

    @staticmethod
    def intercept(self, name):
        canonical_instance = InstanceDeduplicator.get_canonical(self)

        # This object has no canonical instance. However, it may be the clone
        # of an object that does.
        if canonical_instance is None:
            # Build a list of the instance's clone parents in ascending order
            # of depth in the clone tree; i.e. the first element is the root.
            parents = [ self ]
            while True:
                try:
                    parent = object.__getattribute__(parents[0], 'clone_parent')
                    parents.insert(0, parent)
                except AttributeError:
                    break

            # Clone each child from its parent if the parent is has a canonical
            # instance and the child does not.
            canonical_parent = None
            for child in parents:
                canonical_child = InstanceDeduplicator.get_canonical(child)

                # Clone this child from the canonical parent. It will become a
                # new canonical instance.
                if canonical_child is None and canonical_parent is not None:
                    # First, change the class of the child to that of the
                    # canonical parent. This exposes the clone method.
                    canonical_class = object.__getattribute__(canonical_parent, '__class__')
                    child.__class__ = canonical_class

                    # Next, invoke the clone method. It is invoked on the child
                    # (destination) and take the parent (source) as an argument.
                    try:
                        clone_method = object.__getattribute__(child, 'CloneBindings')
                    except AttributeError:
                        raise NotCloneableException('Object {0:s} does not have a CloneBindings method.'.format(child))

                    # Register the child as a canonical instance.
                    InstanceDeduplicator.instances[child] = child
                    canonical_child = child
                    print 'Bound Clone(', canonical_parent, ' ) to', child 

                    # Finally invoke the user-provided clone method.
                    canonical_class.__getattribute__ = object.__getattribute__
                    clone_method(canonical_parent)
                    canonical_class.__getattribute__ = InstanceDeduplicator.intercept

                canonical_parent = canonical_child

            # Update our canonical instance in case we were able to clone a
            # parent with a canonical instance.
            canonical_instance = InstanceDeduplicator.get_canonical(self)

        if canonical_instance is None:
            canonical_instance = self
        return object.__getattribute__(canonical_instance, name)

    @staticmethod
    def get_canonical(self):
        canonical_instance = None
        try:
            canonical_instance = object.__getattribute__(self, '_true_instance')
        except AttributeError:
            if self in InstanceDeduplicator.instances:
                canonical_instance = InstanceDeduplicator.instances[self]
                self._true_instance = canonical_instance

        return canonical_instance

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
