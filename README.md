# PrPy

[![Build Status](https://travis-ci.org/personalrobotics/prpy.svg?branch=master)](https://travis-ci.org/personalrobotics/prpy)

PrPy is a Python library used by the Personal Robotics Laboratory at Carnegie
Mellon University. This library includes robot-agnostic utilities that make
it easier to use OpenRAVE in Python scripts. This includes a high-level
planning pipeline, helper functions, and visualization tools.


## Planning Pipeline

There are a large array of motion planners that have complementary strengths
and weaknesses. PrPy provides a *planning pipeline* in the `prpy.planning`
namespace that makes it easy plan with multiple planners in parallel on a
single problem. Additionally, the planning pipeline takes advantage of the
dynamic nature of Python to mix-and-match planners with heterogeneous
capabilities.

Every planner used in the PrPy planning pipeline extends the
`prpy.planning.base.Planner` class. Typically, a planner will extend one of
two subclasses:

1. `prpy.planning.base.BasePlanner`: implements or wraps a motion planner
2. `prpy.planning.base.MetaPlanner`: combines the output of multiple motion
   planners, each of which is a `BasePlanner` or another `MetaPlanner`

Each planner has one or more *planning methods*, annotated with either the
`@LockedPlanningMethod` or `@ClonedPlanningMethod` decorator, that look like
ordinary functions. Using these decorators makes other PrPy components
aware that these methods exist and follow a particular specification that
allows them to be composed with other PrPy objects automatically.  For
example, `MetaPlanner`s will report that they can perform planning methods
that their child motion planners have enumerated via `@PlanningMethod`
decorators.

`@PlanningMethod` decorators also make sure that calls to planning code are
executed in a thread-safe manner.  In the case of `@LockedPlanningMethod`,
this is enforced by locking the calling environment until the planning method
has completed. In the case of `@ClonedPlanningMethod`, this is enforced by
cloning the calling environment, and calling the wrapped method with references
to the cloned environment.  The result of the method is then copied back to the
calling environment.  `@ClonedPlanningMethod`s can be used to run multiple
planners in parallel and to parallelize planning and execution.

In general, **locked** planning methods are used for calls that will terminate
extremely quickly, while **cloned** planning methods are used for calls that
might take a significant amount of time.

For example, the following code will use OMPL to plan `robot`'s active DOFs
from their current values to to the `goal_config` configuration:

    planner = OMPLPlanner('RRTConnect')
    output_path = planner.PlanToConfiguration(robot, goal_config)

As this is a `@ClonedPlanningMethod`, `robot.GetEnv()` is cloned into the
the `planner.env` planning environment. Planning occurs within this cloned
environment. Finally, the output path is cloned back into `robot.GetEnv()`
and is returned by the planner.

See the following sub-sections for more information about the built-in planners
provided with PrPy, information about writing your own planner, and several
more complex usage examples.


### Built-In Planners

PrPy provides wrappers for several existing planning libraries:

- `CBiRRTPlanner`: [Constrained Bi-directional
   Rapidly-Exploring Random Tree (CBiRRT)](http://www.ri.cmu.edu/publication_view.html?pub_id=6309), requires the [CoMPs suite](https://github.com/personalrobotics/comps)
- `CHOMPPlanner`: [Covariant Hamiltonian Optimization for Motion Planning (CHOMP)](https://www.ri.cmu.edu/publication_view.html?pub_id=7421), requires [or_cdchomp](https://github.com/personalrobotics/or_cdchomp.git)
- `OMPLPlanner`: wrapper for randomized planners implemented in the [Open Motion Planning Library](http://ompl.kavrakilab.org), requires [or_ompl](https://github.com/personalrobotics/or_ompl)
- `OpenRAVEPlanner`: wrapper for OpenRAVE planners that implement the [`PlannerBase` interface](http://openrave.org/docs/latest_stable/coreapihtml/arch_planner.html)
- `SBPLPlanner`: wrapper for the [Search-Based Planning Library (SBPL)](https://github.com/sbpl/sbpl), requires [or_sbpl](https://github.com/personalrobotics/or_sbpl)

Additionally, PrPy provides several simple planners of its own:

- `MKPlanner`: Jacobian pseudo-inverse controller for executing straight-line
  workspace trajectories
- `SnapPlanner`: attempts to execute a straight-line joint-space trajectory to
  the goal
- `GreedyIKPlanner`: follows a workspace path by greedily picking IK solutions
- `VectorFieldPlanner`: follows any custom cspace vector field until a custom termination

Finally, PrPy provides several meta-planners for combining the above
planners:

- `Sequence`: sequentially queries a list of planners and returns the
  result of the first planner in the list that succeeds.
- `Ranked`: queries a list of planners in parallel and returns the
  solution first planner in the list that returns success
- `IKPlanner`: plan to an end-effector pose by sequentially planning to
  a list of ranked IK solutions
- `NamedPlanner`: plan to a named configuration associated with the robot

See the Python docstrings in the above classes for more information.


### Common Planning Methods
  
There is no formal list of `@*PlanningMethod`s or their arguments. However, we
have found these methods to be useful:

- `PlanToConfiguration(robot, goal_config)`: plan the robot's active DOFs from
  the robot's current configuration to the `goal_config` configuration.
- `PlanToConfigurations(robot, goal_configs)`: plan the robot's active DOFs from
  the robot's current configuration to any of the elements in the `goal_configs`
  list.
- `PlanToEndEffectorPose(robot, goal_pose)`: plan the robot's active
  manipulator's end-effector to `goal_pose`.
- `PlanToEndEffectorOffset(robot, direction, min_distance, max_distance)`: plan
  the robot's active manipulator in a straight line in the direction specified
  by the `direction` unit vector for [ `min_distance`, `max_distance` ] meters.
- `PlanToTSR(robot, tsrchains)`: plan with the start, goal, and/or constraints
  specified by a list of TSR chains.
- `PlanToBasePose(robot, goal_pose)`: plan with the robot's affine DOFs in the
  plane to a desired base pose.

Most planners that implement these methods accept a `timelimit` parameter, in
seconds, for which to plan before raising a `PlanningError`. Additionally, many
of these methods accept planner-specific keyword arguments.


### Writing a Custom Planner

Implementing a custom planner requires extending the `BasePlanner` class and
decorating one or more methods with the `@LockedPlanningMethod` or
`@ClonedPlanningMethod` decorator.

Extending the `BasePlanner` class allows PrPy to identify your planner as a
base planner class, as opposed to a meta-planner. The `@PlanningMethod`
decorators handle environment cloning or locking and allows meta-planners to
query the list of planning methods that the planner supports (e.g. to generate
docstrings).

Each instance of a `BasePlanner`-derived class constructs a planning
environment `self.env`.  This environment is uniquely associated with each
instance of the planner and is what will be used in `@ClonedPlanningMethod`
calls. Since this environment is persistent and unique, it can also be used
as a place to cache data or pre-load plugins for planners that have heavyweight
initialization steps.  However, because of this, each planning instance can
only execute one `@ClonedPlanningMethod` at a time.  It can still execute
arbitrary `@LockedPlanningMethod` calls, as long as they are referring to
robots in different environments.

Please obey the following guidelines:

- Assume that the planning environment is locked during the entire call.
- Subclass constructors **must** call `BasePlanner.__init__`.
- Each `@PlanningMethod` **must** accept the first argument `robot`, which is a
  robot in the environment it should be using to perform planning.
- Each `@PlanningMethod` **must** accept `**kwargs` to ignore arguments that
  are not supported by the planner.
- Each `@PlanningMethod` **must** return a `Trajectory` which was created in
  the same environment as the robot it was passed (e.g. `robot.GetEnv()`).
- When possible, use one of the defacto-standard `@*PlanningMethod` names listed
  below.
- Raise a `PlanningError` to indicate an expected, but fatal, error (e.g.
  timeout with no collision-free path).
- When applicable, raise a context-specific subclass of `PlanningError` to
  indicate the nature of the error (e.g. `StartCollisionPlanningError`)
- Raise a standard Python exception, e.g. `ValueError`, to indicate an
  unexpected error has occurred (e.g. argument out of range).
- Raise a `UnsupportedPlanningError` to indicate that the planning operation is
  fundamentally not supported (e.g. constraint type is not implemented).


### Examples

Trajectory optimizers, like CHOMP, typically produce higher quality paths than
randomized planners. However, these algorithms are not probabilistically
complete and can get stuck in local minima. You can mitigate this by using the
`Sequence` planner to first call CHOMP, then fall back on RRT-Connect:

    planner = Sequence(CHOMPPlanner(), OMPLPlanner('RRTConnect'))
    path = planner.PlanToConfiguration(robot, goal)

Unfortunately, this means that RRT-Connect does not start planning until CHOMP
has already failed to find a solution. Instead of using `Sequence`, we can use
the `Ranked` meta-planner to plan with both planners in parallel. Just as
before, the meta-planner will immediately return CHOMP's solution if it returns
success. However, RRT-Connect will have a head-start if CHOMP fails:

    planner = Ranked(CHOMPPlanner(), OMPLPlanner('RRTConnect'))
    path = planner.PlanToConfiguration(robot, goal)`

In other cases, a meta-planner can be used to combine the disparate
capabilities of multiple planenrs. For example, SBPL is currently the only
planner included with PrPy that supports planning for affine DOFs. We can use a
meta-planner to combine OMPL's support for `PlanToConfiguration` with SBPL's
support for `PlanToBasePose`:

    planner = Sequence(OMPLPlanner('RRTConnect'), SBPLPlanner())
    path1 = planner.PlanToConfiguration(robot, goal)
    path2 = planner.PlanToBasePose(robot, goal_pose)

## Perception Pipeline

Recently, support has been added for a few perception routines. The general structure is intended
to mirror that of the planning pipeline, but it is somewhat less encapsulated than 
planning, from the user's perspective.

There is a `prpy.perception.base.PerceptionModule` class which is extended by every perception
routine. Every routine has some common methods for perception, which are annotated with 
`@PerceptionMethod`. Here is an example call (should happen in a typical herbpy console):

    from prpy.perception.apriltags import ApriltagsModule

    adetector = ApriltagsModule(marker_topic='/apriltags_kinect2/marker_array',
                               marker_data_path=FindCatkinResource('pr_ordata','data/objects/tag_data.json'),
                               kinbody_path=FindCatkinResource('pr_ordata','data/objects'),
                               destination_frame='/map',
                               detection_frame='/head/kinect2_rgb_optical_frame')
    detected_objects = adetector.DetectObjects(robot)

IMPORTANT - Most of these methods require some underlying CPP server to be running, before calls can be
made to the PrPy detector. 

### Perception Modules

Currently, the following perception routines are supported:

- `AprilTags`
- `VNCC`: Vectorized Normalized Cross Correlation
- `SimTrack`
- `BlockDetector`
- `ROCK`: Robust Object Constellation and Kinematic Pose

### Underlying Servers

- `AprilTags`: Started via `apriltags.launch` in [herb_launch](https://github.com/personalrobotics/herb_launch). Publishes to `/apriltags_kinect2/detections` and `/apriltags_kinect2/marker_array`.
- `VNCC`: Have [vncc_msgs](https://github.com/personalrobotics/vncc_msgs) and [vncc](https://github.com/personalrobotics/vncc) in your workspace. Run `roslaunch vncc vncc_estimator.launch`. This provides the `/vncc/get_vncc_detections` service.
- `SimTrack` - See Caveats section below
- `BlockDetector` - Have [tabletop_perception_tools](https://github.com/personalrobotics/tabletop_perception_tools) in your workspace. Run `rosrun tabletop_perception_tools tools_server`. This provides the `/tools_server/find_blocks` service.
- `ROCK` - To be updated later.


### Common Perception Methods

At this point, two methods are common to all perception routines. However, some 
routine-specific knowledge may be required to make them work. This is particularly reflected
in the constructor for the perception module.

- `DetectObjects(self, robot, **kw_args)`: This runs the perception method for all
objects that the particular routine knows about. Typically, this information is specified
either as a config file (in the case of AprilTags) or in the constructor of the respective
module.
- `DetectObject(self,robot,obj_name)`: This runs the perception routine to detect a particular object,
based on the known names in the database.

The return type for both is typically one or more OpenRAVE kinbodies, with the correct
transformation relative to the current environment, if the input `tf`s have been 
correctly provided.


### Caveats

As mentioned above, running the perception routines require a bit of routine-specific knowledge,
because of differences in the way some of them operate. Some of those caveats, for each routine
are mentioned here.

- `AprilTags`: This method involves detection of visual fiducial markers. There is a database that maps
april tag IDs to the objects to which they are attached, along with the relative transform
of the tag with respect to the object kinbody, in `pr_ordata/data/objects/tag_data.json`.
- `VNCC`: This is a single-query method and so currently does not support `DetectObjects`, but just
`DetectObject`, where the object names are obtained from the map in the module's constructor.
- `SimTrack`: See https://github.com/personalrobotics/simtrack for more details. You will need the `personalrobotics` fork. This can track/detect any kind of textured object stored as an `.obj` file. The perception module only calls the detector, but the tracker can also be integrated pretty easily. It supports `DetectObjects`, and requires the simtrack `multi_rigid_node` to be running on the robot to work. Inside the module, there is a map of `simtrack` objects to kinbodies.
- `BlockDetector`: This is specifically for detecting blocks on a table in front of the camera. Therefore,
it only has a `DetectBlocks` method.
- `ROCK`: This is still under development and so does not exactly conform to the underlying API.

## Environment Cloning

Cloning environments is critical to enable planning with multiple planners in
parallel and parallelizing planning and execution. PrPy provides two utilities
to simplify environment cloning in OpenRAVE: the `Clone` context manager and
the `Cloned` helper function.


### Clone Context Manager

PrPy adds a `prpy.clone.Clone` context manager to manage temporary environment
clones; e.g. those used during planning. This context manager clones an
environment when entering the `with`-block and destroys the environment when
exiting the block. This code is careful to lock the source and destination
environments during cloning correctly to avoid introducing a race condition.

In the simplest case, the `Clone` context manager creates an internal,
temporary environment that is not re-used between calls:

    with Clone(env) as cloned_env:
        robot = cloned_env.GetRobot('herb')
        # ...

The same context manager can be used to clone into an existing environment. In
this case, the same target environment can be used by multiple calls. This
allows OpenRAVE to re-use the environments resources (e.g. collision
tri-meshes) and can dramatically improve performance:

    clone_env = openravepy.Environment()

    with Clone(env, clone_env=clone_env):
        robot = cloned_env.GetRobot('herb')
        # ...

Often times, the cloned environment must be immediately locked to perform
additional setup. This introduces a potential race condition between `Clone`
releasing the lock and the code inside the `with`-block acquiring the lock. To
avoid this, use the `lock` argument to enter the `with`-block without releasing
the lock:

    with Clone(env, lock=True) as cloned_env:
        robot = cloned_env.GetRobot('herb')
        # ...

In this case, the cloned environment will be automatically unlocked when
exiting the `with`-statement. This may be undesirable if you need to explicitly
unlock the environment inside the `with`-statement. In this case, you may pass
the `unlock=False` flag. In this case, you **must** explicitly unlock the
environment inside the `with`-statement:

    with Clone(env, lock=True, unlock=False) as cloned_env:
        robot = cloned_env.GetRobot('herb')
        env.Unlock()
        # ...


### Cloned Helper Function

It is frequently necessary to find an object in a cloned environment that
refers to a particular object in the parent environment. This code frequently
looks like this:

    with Clone(env) as cloned_env:
        cloned_robot = cloned_env.GetRobot(robot.GetName())
        # ...

The `prpy.clone.Cloned` helper function handles this name resolution for most
OpenRAVE data types (including `Robot`, `KinBody`, `Link`, and `Manipulator`).
This function accepts an arbitrary number of input parameters---of the
supported types---and returns the corresponding objects in `Clone`d
environment. For example, the above code can be re-written as:

    with Clone(env) as cloned_env:
        cloned_robot = Cloned(robot)
        # ...

If multiple `Clone` context managers are nested, the `Cloned` function returns
the corresponding object in the inner-most block:

    with Clone(env) as cloned_env1:
        cloned_robot1 = Cloned(robot) # from cloned_env1
        # ...

        with Clone(env) as cloned_env2:
            cloned_robot2 = Cloned(robot) # from cloned_env2
            # ...

        # ...
        cloned_robot3 = Cloned(robot) # from cloned_env1

The `Cloned` function only works if it is called from the same thread in which
the `Clone` context manager was created. If this is not the case, you can still
use the `Cloned` helper function by explicitly passing an environment:

    with Clone(env) as cloned_env:
        def fn(body, e):
            cloned_robot = Cloned(body, clone_env=e)
            # ...

        thread = Thread(target=fn, args=(body, cloned_env))
        thread.start()
        thread.join()

Finally, as a convenience, the `Cloned` function can be used to simultaneously
resolve multiple objects in one statement:

    with Clone(env) as cloned_env:
        cloned_robot, cloned_body = Cloned(robot, body)
        # ...


## Concurrent Execution

PrPy has native support for [futures](http://en.wikipedia.org/wiki/Futures_and_promises) and
[coroutines](http://en.wikipedia.org/wiki/Coroutine) to simplify concurrent programming. A
_future_ encapsulates the execution of a long-running task. We use the concurrency primitives
provided by the [`trollius` module](http://trollius.readthedocs.org/using.html),
which is a Python 2 backport of the [`asyncio` module](https://docs.python.org/3/library/asyncio.html)
from Python 3.

We can use these primitives to parallelize planning and execution:

    @coroutine
    def do_plan(robot):
        # Plan to goal1 and start executing the trajectory.
        path1 = yield From(robot.PlanToEndEffectorPose(goal1, execute=False))
        exec1_future = robot.ExecutePath(path1)

        # Plan from goal1 to goal2.
        robot.SetDOFValues(GetLastWaypoint(path1))
        path2 = yield From(robot.PlanToEndEffectorPope(goal2, execute=False))

        # Wait for path1 to finish executing, then execute path2.
        exec1 = yield From(exec1_future)
        exec2 = yield From(robot.ExecutePath(path2))

        raise Return(path1, path2)

    loop = trollius.get_event_loop()
    path = loop.run_until_complete(do_plan(robot))

## Method Binding

Finally, PrPy offers helper functions for binding custom methods on (i.e.
[monkey patching](http://en.wikipedia.org/wiki/Monkey_patch)) OpenRAVE data
types, including: `KinBody`, `Robot`, `Link`, and `Joint`.

This may appear trivial to accomplish using `setattr`. However, this is
actually quite challenging to implement because OpenRAVE's Python bindings for
these classes are automatically generated as [Boost.Python
bindings](http://www.boost.org/doc/libs/release/libs/python/) that are managed by a
`shared_ptr`. Each instance of a `shared_ptr` returned by C++ is wrapped in a
separate Python object. As a result, the following code does not work as
expected:

    robot = env.GetRobot('herb')
    setattr(robot, 'foo', 'bar')
    robot_ref = env.GetRobot('herb')
    robot_ref.foo # raises AttributeError

PrPy provides the `prpy.bind.InstanceDeduplicator` class to work around this
issue. This class takes advantage of the user data attached to an OpenRAVE
environment to de-duplicate multiple Python `shared_ptr` instances that
reference same object. This is implemented by overriding `__getattribute__` and
`__setattribute__` to defer all attribute queries to a single *canonical
instance* of the object.

### Canonical Instances

An object is flagged for de-duplication using the
`InstanceDeduplicator.add_canonical` function. The above example can be
modified to work as follows:

    robot = env.GetRobot('herb')
    InstanceDeduplicator.add_canonical(robot)

    setattr(robot, 'foo', 'bar')
    robot_ref = env.GetRobot('herb')
    robot_ref.foo # returns 'bar'


### Subclass Binding

Frequently we wish to extend an OpenRAVE object with several tightly coupled
attributes, properties, and methods. This can be achieved by creating subclass
of appropriate OpenRAVE data type (e.g. `Robot`) and dynamically changing that
instance's `__class__` at runtime. PrPy provides a `prpy.bind.bind_subclass`
helper function that calls `add_canonical`, changes `__class__`, and calls the
subclass' `__init__` function.

This functionality is most frequently used with the generic PrPy subclasses
provided in the `prpy.base` module. For example, the following code adds the
capabilities of `prpy.base.Robot` to an existing robot:

    robot = env.GetRobot('herb')
    bind_subclass(robot, prpy.base.Robot)

See the docstrings on the classes defined in `prpy.base` for more information.


### Cloning Bound Subclasses

OpenRAVE is not aware of methods, attributes, or properties that are added to
objects in Python; e.g. using `add_canonical`. As a result, these attributes
are not duplicated when the OpenRAVE environment is cloned. PrPy provides the
limited capability to clone these attributes when: (1) the class was extended
using the `bind_subclass` function and (2) the clone was created using the PrPy
`Clone` function. If these two conditions hold, PrPy will call the
`CloneBindings()` function on your custom subclass the first time the cloned
object is referenced.

See the classes in `prpy.base` for example implementations of `CloneBindings`.

## Dependencies
To run prpy, you will need to have installed the pacakge enum34. To do that, go to a local directory and run 
``sudo pip install enum34``

## License

PrPy is licensed under a BSD license. See `LICENSE` for more information.


## Contributors

PrPy is developed by the
[Personal Robotics Lab](https://personalrobotics.ri.cmu.edu) in the
[Robotics Institute](https://www.ri.cmu.edu) at
[Carnegie Mellon University](http://www.cmu.edu). This library was originally
developed by [Michael Koval](https://github.com/mkoval), with some code copied
from the earlier `prrave` library developed by [Chris Dellin](https://github.com/cdellin).

This is a non-exhaustive list of contributors:

- [Chris Dellin](https://github.com/cdellin)
- [Evan Shapiro](https://github.com/es92)
- [Jen King](https://github.com/jeking)
- [Michael Koval](https://github.com/mkoval)
- [Pras Velagapudi](https://github.com/psigen)
- [Shervin Javdani](https://github.com/sjavdani)
- [Sidd Srinivasa](https://github.com/siddhss5)
- [Stefanos Nikolaidis](https://github.com/Stefanos19)
- [Shushman Choudhury](https://github.com/Shushman)
