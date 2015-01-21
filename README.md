PrPy
====
PrPy is a Python library used by the Personal Robotics Laboratory at Carnegie
Mellon University. This library includes robot-agnostic utilities that make
it easier to use OpenRAVE in Python scripts. This includes a high-level
planning pipeline, helper functions, and visualization tools.


Planning Pipeline
-----------------
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


Built-In Planners
~~~~~~~~~~~~~~~~~
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

- `prpy.base.Sequence`
- `prpy.base.Ranked`
- `prpy.ik.IKPlanner`
- `prpy.named.NamedPlanner`


Base Planners
~~~~~~~~~~~~~

