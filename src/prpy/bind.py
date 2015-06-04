#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import logging
from clone import CloneException

logger = logging.getLogger(__name__)


class NotCloneableException(CloneException):
    pass


class InstanceDeduplicator(object):
    USERDATA_PREFIX = 0xDEADBEEF
    USERDATA_CHILDREN = '__children__'
    USERDATA_DESTRUCTOR = '__destructor__'
    USERDATA_CANONICAL = 'canonical_instance'
    ATTRIBUTE_CANONICAL = '_canonical_instance'
    ATTRIBUTE_IS_CANONICAL = '_is_canonical_instance'
    KINBODY_TYPE, LINK_TYPE, JOINT_TYPE, MANIPULATOR_TYPE = range(4)

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
                    InstanceDeduplicator.add_canonical(child)
                    canonical_child = child

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

        try:
            return object.__getattribute__(canonical_instance, name)
        except AttributeError:
            # We have to manually call __getattr__ in case it is overriden in
            # the canonical_instance's class or one of its superclasses. This
            # is not handled by the recursive call to __getattribute__ because
            # we explicitly invoke it on object.
            if hasattr(canonical_instance, '__getattr__'):
                return canonical_instance.__getattr__(name)
            else:
                raise

    @classmethod
    def get_canonical(cls, instance):
        # Try looking for a cached value on the object.
        try:
            canonical_instance = object.__getattribute__(instance, cls.ATTRIBUTE_CANONICAL)
        # If it's not available, fall back on doing the full lookup not
        # available, fall back on doing the full lookup
        except AttributeError:
            try:
                is_canonical_instance = object.__getattribute__(instance, cls.ATTRIBUTE_IS_CANONICAL)
                canonical_instance = instance
            except AttributeError:
                userdata_getter, userdata_setter = cls.get_storage_methods(instance)
                try:
                    canonical_instance = userdata_getter(cls.USERDATA_CANONICAL)

                    # ...and cache the value for future queries.
                    if canonical_instance is instance:
                        object.__setattr__(instance, cls.ATTRIBUTE_IS_CANONICAL, True)
                    else:
                        object.__setattr__(instance, cls.ATTRIBUTE_CANONICAL, canonical_instance)
                except KeyError:
                    canonical_instance = None

        return canonical_instance

    @classmethod
    def add_canonical(cls, instance):
        _, userdata_setter = cls.get_storage_methods(instance)
        userdata_setter(cls.USERDATA_CANONICAL, instance)
        instance.__class__.__getattribute__ = cls.intercept

    @classmethod
    def get_environment_id(cls, target, recurse=False):
        def call(self, name, *args, **kw_args):
            return object.__getattribute__(self, name)(*args, **kw_args)

        target_class = object.__getattribute__(target, '__class__')

        import openravepy
        if issubclass(target_class, openravepy.KinBody):
            env = call(target, 'GetEnv')
            key = (call(target, 'GetName'), )
            owner = target
            target_type = cls.KINBODY_TYPE
        elif issubclass(target_class, openravepy.KinBody.Link):
            env, owner, kinbody_key = cls.get_environment_id(
                    call(target, 'GetParent'), recurse=True)
            key = kinbody_key + (call(target, 'GetIndex'), )
            target_type = cls.LINK_TYPE
        elif issubclass(target_class, openravepy.KinBody.Joint):
            env, owner, kinbody_key = cls.get_environment_id(
                    call(target, 'GetParent'), recurse=True)
            key = kinbody_key + (call(target, 'GetJointIndex'), )
            target_type = cls.JOINT_TYPE
        elif issubclass(target_class, openravepy.Robot.Manipulator):
            env, owner, kinbody_key = cls.get_environment_id(
                    call(target, 'GetRobot'), recurse=True)
            key = kinbody_key + (call(target, 'GetName'), )
            target_type = cls.MANIPULATOR_TYPE
        else:
            raise ValueError('Unsupported object type.')

        # Append a unique number to minimize the chance of accidentally
        # colliding with other UserData. Also append a type identifier
        # to prevent cross-type conflicts.
        if not recurse:
            key = (cls.USERDATA_PREFIX, target_type) + key

        return env, owner, key

    # Cleanup the UserData owned by a KinBody when it is removed from
    # the environment.
    @staticmethod
    def cleanup_callback(owner, flag):
        if flag == 0: # removed
            # Clear any attributes that the user might have bound to
            # the object. This is necessary to clear cycles.
            canonical_instance = InstanceDeduplicator.get_canonical(owner)
            if canonical_instance is not None:
                canonical_dict = object.__getattribute__(canonical_instance, '__dict__')
                canonical_dict.clear()


            # Remove any storage (e.g. canonical_instance) bound to
            # this object.
            children, canonical_instance = InstanceDeduplicator.get_bound_children(owner)
            InstanceDeduplicator.remove_storage(owner)

            # Remove circular references that pass through Boost.Python.
            # A normal iterator can't be used here because clear_referrers
            # will delete the child from children causing elements to be
            # skipped
            while children:
                child = children.pop()
                logger.debug(child)
                clear_referrers(child)
                # NOTE: this is also an acceptable body for the loop :)
                # clear_referrers(children[0])
            logger.debug(owner)
            if canonical_instance != None:
                clear_referrers(canonical_instance)

    @classmethod
    def get_storage_methods(cls, target):
        env, owner, target_key = cls.get_environment_id(target)
        _, _, owner_key = cls.get_environment_id(owner)
        user_data = env.GetUserData()

        # Lazily create a dict to store the canonical instance map if it
        # does not already exist.
        if user_data is None:
            user_data = dict()
            env.SetUserData(user_data)

            if hasattr(env, 'RegisterBodyCallback'):
                handle = env.RegisterBodyCallback(InstanceDeduplicator.cleanup_callback)
                user_data[cls.USERDATA_DESTRUCTOR] = handle
            else:
                logger.warning(
                    'Your version of OpenRAVE does not supply Python bindings'
                    ' for Environment.RegisterBodyCallback. PrPy will leak'
                    ' unless you call prpy.bind.InstanceDeduplicator.remove_storage'
                    ' after removing an annotated KinBody from the environment.'
                )

        elif not isinstance(user_data, dict):
            raise ValueError('Detected unknown UserData.')

        # Create getter/setter methods for accessing the data.
        def getter(key):
            target_dict = user_data.get(target_key, dict())
            return target_dict[key]

        def setter(key, value):
            if key == cls.USERDATA_CHILDREN:
                raise KeyError(
                    "The key '{:s}' is reserved for internal use.".format(
                        cls.USERDATA_CHILDREN)
                )

            target_dict = user_data.setdefault(target_key, dict())
            target_dict[key] = value

            # Log this entry so we can easily clean it up later. Note that we
            # add the owner itself to this list to simplify the cleanup code.
            owner_dict = user_data.setdefault(owner_key, dict())
            owner_children = owner_dict.setdefault(cls.USERDATA_CHILDREN, set())
            owner_children.add(owner_key)
            owner_children.add(target_key)

        return getter, setter

    @classmethod
    def get_bound_children(cls, parent):
        # Lookup the key for all bound children.
        env, _, parent_key = cls.get_environment_id(parent)
        user_data = env.GetUserData()
        try:
            child_keys = user_data[parent_key][cls.USERDATA_CHILDREN]
        except KeyError:
            # There are no bound children.
            return [], None

        # Resolve the key to an instance.
        children = []
        canonical_instance = None
        for child_key in child_keys:
            canonical_child = user_data[child_key].get(cls.USERDATA_CANONICAL, None)
            if canonical_child != parent:
                children.append(canonical_child)
            else:
                canonical_instance = canonical_child
        return children, canonical_instance

    @classmethod
    def remove_storage(cls, kinbody):
        env, owner, owner_key = cls.get_environment_id(kinbody)
        if kinbody != owner:
            raise ValueError('Object does not own any storage.')

        user_data = kinbody.GetEnv().GetUserData()
        if user_data is not None:
            owner_dict = user_data.get(owner_key, dict())
            children_set = owner_dict.get(cls.USERDATA_CHILDREN, set())

            for child_key in children_set:
                user_data.pop(child_key)


def clear_referrers(obj, debug=False):
    import gc

    # TODO: We should do a topographical sort on these references.

    for referrer in gc.get_referrers(obj):
        #TODO print if deleting from user defined referrer
        logger.debug('Clearing referrer "%s" to object "%s".', referrer, obj)

        # Handle standard Python objects.
        if hasattr(referrer, '__dict__'):
            for field,value in referrer.__dict__.items():
                if value is obj:
                    del referrer.__dict__[field]

        # Remove references from built-in collections.
        if isinstance(referrer, dict):
            for field, value in referrer.items():
                if value is obj:
                    del referrer[field]
        elif isinstance(referrer, list) or isinstance(referrer, set):
            referrer.remove(obj)
        # tuple and frozenset are immutable, so we remove the whole object.
        elif isinstance(referrer, tuple) or isinstance(referrer, frozenset):
            clear_referrers(referrer, debug=debug)

        if debug:
            import pprint
            pp = pprint.PrettyPrinter(indent=4)
            pp.pprint(referrer)

def print_referrers(obj, dbg=False):
    import gc
    for referrer in gc.get_referrers(obj):
        if isinstance(referrer, dict):
            for field, value in referrer.items():
                if value is obj:
                    print referrer, field
        elif hasattr(referrer, '__dict__'):
            for field,value in referrer.__dict__.items():
                if value is obj:
                    print referrer, field
        elif hasattr(referrer, 'remove'):
            print referrer
        elif type(referrer) == tuple:
            clear_referrers(referrer, dbg=True)
        else:
            #import pprint
            #pp = pprint.PrettyPrinter(indent=4)
            #pp.pprint(referrer)
            pass

def bind_subclass(instance, subclass, *args, **kw_args):
    # Deduplicate the classes associated with any of the objects that we're
    # modifying at runtime. This is necessary for the modifications to be
    # available on instances returned by C++.
    InstanceDeduplicator.add_canonical(instance)
    instance.__class__ = subclass
    instance.__init__(*args, **kw_args)
