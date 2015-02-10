# PrPy

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

Each planner has one or more *planning methods*, annotated with the
`@PlanningMethod` decorator, that look like ordinary functions. However, unlike
an ordinary function, calling a planning method clones the robot's environment
into a *planning environment* associated with the planner. Planning occurs in
the cloned environment to allow PrPy to run multiple planners in parallel and
to paralellize planning and execution.

For example, the following code will use OMPL to plan `robot`'s active DOFs
from their current values to to the `goal_config` configuration:

    planner = OMPLPlanner()
    output_path = planner.PlanToConfiguration(robot, goal_config)

First, `robot.GetEnv()` is cloned into the the `planner.env` planning
environment. Next, planning occurs in the cloned environment. Finally, the
output path is cloned back into `robot.GetEnv()` and is returned by the
planner.

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

Finally, PrPy provides several meta-planners for combining the above
planners:

- `Sequence`: sequentially queries a list of planners and returns the
  result of the first planner in the list that succeeds.
- `Ranked`: queries a list of planners in parallel and returns the
  solution first planner in the list that returns success
- `IKPlanner`: plan to an end-effector pose by sequentially planning to
  a list of ranked IK solutions
- `NamedPlanner`: plan to a named configuration associated with the robot

See the Python docstrings the above classes for more information.


### Common Planning Methods
  
There is no formal list of `@PlanningMethod`s or their arguments. However, we
have found these methods to be useful:

- `PlanToConfiguration(robot, goal_config)`: plan the robot's active DOFs from
  the robot's current configuration to the `goal_config` configuration.
- `PlanToConfigurations(robot, goal_configs)`: plan the robot's active DOFs from
  the robot's current configuration to one of the elements in the `goal_config`
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
decorating one or more methods with the `@PlanningMethod` decorator. Extending
the `BasePlanner` class constructs the planning environment `self.env` and
allows PrPy to identify your planner as a base planner class, as opposed to a
meta-planner. The `@PlanningMethod` decorator handles environment cloning and
allows meta-planners to query the list of planning methods that the planner
supports (e.g. to generate docstrings).

Please obey the following guidelines:

- Assume that the cloned environment is locked during the entire call.
- Subclass constructor **must** call `BasePlanner.__init__`.
- A `@PlanningMethod` **must not** call another `@PlanningMethod`.
- Each `@PlanningMethod` **must** accept the first argument `robot`, which is a
  robot in the cloned environment.
- Each `@PlanningMethod` **must** accept `**kwargs` to ignore arguments that
  are not supported by the planner.
- Each `@PlanningMethod` **must** return a `Trajectory` which was created in
  the cloned environment.
- When possible, use one of the defacto-standard `@PlanningMethod` names listed
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


## Environment Cloning

Cloning environments is critical to enable planning with multiple planners in
parallel and parallelizing planning and execution. PrPy provides two utilities
to simplify environment cloning in OpenRAVE: the `Clone` context manager and
the the `Cloned` helper function

### Clone Context Manager

PrPy adds a `Clone` context manager to manage temporary environment clones;
e.g. those used during planning. This context manager clones an environment
when entering the `with`-block and destroys the environment when exiting the
block. This code is careful to lock the source and destination environments
during cloning correctly to avoid introducing a race condition.

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

The `Clone` helper function handles this name resolution for most OpenRAVE data
types (including `Robot`, `KinBody`, `Link`, and `Manipulator`). This function
accepts an arbitrary number of input parameters---of the supported types---and
returns the corresponding objects in `Clone`d environment. For example, the
above code can be re-written as:

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

Finally, as a convenience, the `Cloned` function can be used to simultaneously
resolve multiple objects in one statement:

    with Clone(env) as cloned_env:
        cloned_robot, cloned_body = Cloned(robot, body)
        # ...


## Method Binding


