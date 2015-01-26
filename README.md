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
   planners, each of which is a `BasePlanner`

Each planner has one or more *planning methods*, annotated with the
`@PlanningMethod` decorator, that like ordinary functions. However, unlike an
ordinary function, calling a planning method causes the current OpenRAVE
environment to be cloned. All planning occurs in the internal, cloned
environment.


### Built-In Planners
PrPy provides wrappers for several existing planning libraries:

- `planning.cbirrt.CBiRRTPlanner`: [Constrained Bi-directional
   Rapidly-Exploring Random Tree (CBiRRT)](http://www.ri.cmu.edu/publication_view.html?pub_id=6309), requires the [CoMPs suite](https://github.com/personalrobotics/comps)
- `planning.chomp.CHOMPlanner`: [Covariant Hamiltonian Optimization for Motion Planning (CHOMP)](https://www.ri.cmu.edu/publication_view.html?pub_id=7421), requires [or_cdchomp](https://github.com/personalrobotics/or_cdchomp.git)
- `planning.ompl.OMPLPlanner`: wrapper for randomized planners implemented in the [Open Motion Planning Library](http://ompl.kavrakilab.org), requires [or_ompl](https://github.com/personalrobotics/or_ompl)
- `planning.openrave.OpenRAVEPlanner`: wrapper for OpenRAVE planners that implement the [`PlannerBase` interface](http://openrave.org/docs/latest_stable/coreapihtml/arch_planner.html)
- `planning.sbpl.SBPLPlanner`: wrapper for the [Search-Based Planning Library (SBPL)](https://github.com/sbpl/sbpl), requires [or_sbpl](https://github.com/personalrobotics/or_sbpl)

Additionally, PrPy provides several simple planners of its own:

- `planning.mk.MKPlanner`: Jacobian pseudo-inverse controller for executing straight-line workspace trajectories
- `planning.snap.SnapPlanner`: attempts to execute a straight-line joint-space trajectory to the goal

Finally, PrPy provides several meta-planners for combining the above
planners:

- `prpy.base.Sequence`: sequentially queries a list of planners and returns the
  result of the first planner in the list that succeeds.
- `prpy.base.Ranked`: queries a list of planners in parallel and returns the
  solution first planner in the list that returns success
- `prpy.ik.IKPlanner`: plan to an end-effector pose by sequentially planning to
  a list of ranked IK solutions
- `prpy.named.NamedPlanner`: plan to a named configuration associated with the robot

See the Python docstrings the above classes for more information.;


### Planning Methods

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

## Base Planners
