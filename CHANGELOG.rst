^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prpy
^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Adding planner and planning_method and trajectory tag constants
* Removing smooth tag from SBPL trajectory
* Adding helper function for finding catkin resources
* Fixing bug in name of returned variable from Rotate and Forward
* Simplified logic in PostProcessPath.
* Removing need for ExecuteBasePath. Instead base planning now uses ExecutePath.
* Removing unecessary logging
* Various fixes/enhancements: 1. Base planners no longer add non-PlanningMethod functions as attributes to robot, 2. Removed double call to SimplifyTrajectory in retimer.py, 3. Changed default smoother to HauserParabolicSmoother, 4. Changed default simplifier to None
* Fixing format error when raising value error. Fixing logic error in handling defer flag.
* Restructured defer fixes to raise exception.
  Instead of printing a warning, this restructures the `defer` argument
  checking to raise an exception if an invalid value has been provided.
* Print a warning if defer is not a boolean.
* Print a warning if GetTrajectoryTags is not JSON.
* Mico Refactor
* Changed defer checks to use explicit `is True`.
  Using `if defer is True:` for checks instead of `if defer:` catches a
  lot of weird errors that can occur if the positional args to any of the
  reflected planning-method functions are shifted by one.
  The previous check would return a Future if an extra argument got
  passed, which concealed exceptions indicating that the arguments made
  no sense, and would be passed to subsequent code until something
  actually tried to query a Trajectory method on the Future.
* Changed GetTrajectoryTags() to EAFP-style.
  Instead of using an if-check, GetTrajectoryTags() now just tries
  JSON deserialization and catches a ValueError. This is more robust as
  it also catches situations where the deserialization fails due to the
  trajectory description being invalid or whitespace, but not None.
* added kwargs to ExecuteTrajectory and PostProcessPath
* Switched to emprical acceleration limits.
* CBiRRT and OpenRAVERetimer now use CO_ActiveOnly
* increased the accelearation limtis
* Clear UserData in prpy.Clone (fixes `#111 <https://github.com/personalrobotics/prpy/issues/111>`_ and `#114 <https://github.com/personalrobotics/prpy/issues/114>`_)
* Convert CBiRRT "direction" to a NumPy array.
* Removed references to numpy.isclose (`#63 <https://github.com/personalrobotics/prpy/issues/63>`_).
* Added `releasegil` flags to every FindIKSolution(s) call in prpy.
* Released GIL during TSR Planner.
  This prevents unnecessary hangs during planning when using python
  threads.  I see no cases where this would not be necessary.
* Contributors: ADA Demo, Jennifer King, Michael Koval, Pras, Pras Velagapudi, Rachel Holladay, Stefanos Nikolaidis

0.5.1 (2015-04-15)
------------------
* Merge branch 'feature/MICORefactor' of github.com:personalrobotics/prpy into feature/MICORefactor
* Fixed ParabolicSmoother bug (thanks @rdiankov)
* added code to cleanup ik solver, changed acceleration to 1.5
* Added some hacks for ParabolicSmoother.
* More retiming fixes.
* Added a few useful log messages.
* Cleaned up wrappers for OpenRAVE retimers.
* Fixed Open/Close/CloseTight functions on MicoHand.
* Set acceleration limits by default.
* Convert CBiRRT "direction" to a NumPy array.
* Merge branch 'master' into feature/MICORefactor
  Conflicts:
  src/prpy/base/robot.py
* Merge pull request `#95 <https://github.com/personalrobotics/prpy/issues/95>`_ from personalrobotics/feature/SmoothingRefactor2
  Trajectory timing/smoothing refactor 2.0.
* Merge pull request `#108 <https://github.com/personalrobotics/prpy/issues/108>`_ from personalrobotics/bugfix/issue99
  Fixed two bugs in vectorfield planner.
* Made robot.simplifier optional.
* Load an IdealController in simulation.
* Fixed two bugs in planner
  Fixed two bugs:
  1. Missing `abs`
  2. Changed default `dt_multiplier` to 1.01 so that `numsteps` floors to 1 by default.
* Fixed weird superclass issue.
* Removed multi-controller references from Mico.
* More MicoHand cleanup.
* Started removing BH-specific code from MicoHand
* Removed MICORobot, since it does nothing.
* Load or_nlopt_ik by default.
* PEP-8 fixes.
* Removed more dead code from Mico.
* Rearranged Mico file.
* Removed PlanToNamedConfiguration from Mico.
* Removed OWD-specific code from the Mico.
* Documented ExecutePath and ExecuteTrajectory.
* Simplified PostProcessPath with defer=True.
* Rough PostProcessPath function.
* Contributors: Michael Koval, Siddhartha Srinivasa, Stefanos Nikolaidis

0.5.0 (2015-04-07)
------------------
* Fixed the OMPL planner creation test.
* Modified CBiRRT to output linear interpolation.
* Fixed __getattr__ and __dir__ on Manipulator (`#89 <https://github.com/personalrobotics/prpy/issues/89>`_)
* Fixed infinite recursion in `#89 <https://github.com/personalrobotics/prpy/issues/89>`_
  robot.planner or robot.actions not being defined caused infinite
  recursion in __getattr__. This patch explicitly checks for those
  attributes before querying them.
* Added robot_name pass-through argument.
* Various fixes: Added logic to catch openrave excpetion and reraise as planning exception in CHOMP. Added PlanToConfiguration to BiRRT. Changed SetTrajectoryTags to util.SetTrajectoryTags in vectorfield planner.
* Feature/action library
* Changed RenderPose to RenderPoses. Made RenderTSRChains call RenderPoses. Added render flag to RenderTSRChains, RenderPoses and RenderVector so that they can be used optionally.
* Adding RenderPose function to allow rendering an axis from a with block
* for servo simulation, sleep time takes into account how much time already was spend on computation
* Merge pull request `#81 <https://github.com/personalrobotics/prpy/issues/81>`_ from personalrobotics/feature/PlanningRefactor
  Added new MethodMask and FirstSupported meta-planners
* Disabled PlanToIK on TSRPlanner.
* Renamed new meta-planners.
  - Only to MethodMask
  - Fallback to FirstSupported
* made default quadraticObjective, changed to allow you to specify arguments for joint limit avoidance
* Tag trajectories with information necessary to control smoothing.
* Moved common tags into an Enum.
* Switched from XML to JSON to trajectory tagging.
* Added python-enum dependency.
* Added PlanToIK to TSRPlanner.
* Added new MetaPlanners and refactored planning.
  - Added the Fallback meta-planner. This meta-planner operates on a list
  of planners and calls the first planner in the list that supports the
  desired planning method.
  - Added the Only meta-planner. This meta-planner operates on a single
  planner by only allowing access to a subset of its planning methods.
  - Added support for explicitly passing a delegate planner to:
  - IKPlanner
  - NamedPlanner
  - TSRPlanner
  - Modified TSRPlanner to raise an UnsupportedPlanningError when it
  receives unsupported TSRs. This is necessary to trigger the fallback
  behavior implemented in the Fallback meta-planner.
* feature added to avoid joint limit with ComputeJointVelocityFromTwist
* Cleaned up CloneBindings functions
  - Reference the TSRLibrary from the parent environment.
  - Reference the NamedConfigurations from the parent environment.
  - Don't load ServoSimulatored in cloned environments.
  - Don't load any controllers in cloned environments.
  - Avoid calling __init__ to prevent future nasty surprises.
  - NOTE: This fixes a memory leak caught by Pras.
* Merge pull request `#76 <https://github.com/personalrobotics/prpy/issues/76>`_ from personalrobotics/feature/vector_field_planner_timestepping
  Added variable time steps for vector field planner
* Hide IK log spam when cloning environments.
* Tag trajectories with constrained and optimized
* More CHOMP module refactoring.
* Cleaned up CHOPM file.
* Added variable time steps for vector field planner
* Tag trajectories with planner and planning method.
* Renaming robot.actionlibrary to robot.actions
* Adding logic to explicitely clear handles arrays in visualization helper functions
* Fixing logic that adds actions as methods on robot. Adding logic to add actions as methods on manipulator. Updating visualization of TSR lists to have parameter for axis length. Removing reference to push_grasp from prpy/action init.
* Adding logic to expose actions as methods on robot
* Initial action library implementation
* Contributors: Jennifer King, Michael Koval, Shervin Javdani, Siddhartha Srinivasa

0.4.0 (2015-03-30)
------------------
* Planning with vector fields.
* Documentation update
* Go as fast as possible!
* Fixed status logic bug
* Added caching
* Added exception handling for min distance
* More code refactoring and testing of end effector offset
* First pass at plan to end effector offset
* Added termination function
* Trajectory execution refactor
* Modify OptimizeTrajectory in chomp to catch generic exceptions and raise them as PlanningError
* Adding support for execution of base trajectories
* Fixing two typos in cbirrt that cause failures
* Changing parabolic smoother to use HauserParabolicSmoother by default
* Adding logic to clone the environment eshen simplifying and smoothing a path. This allows us to set the dofs in the trajectory as active.
* Refactored vectorfield planner to input function pointer
* Implemented defer=True on ExecuteTrajectory.
* Eat kwargs in OMPLSimplifier.
* Added defer=True support to ExecutePath.
* Fixed typo in vectorfield planner
* Fixed bug when getting DOF resolutions
* Added a few cleanups for syntax and simplicity.
* First pass at vector field planner to end effector transform
* Cleaned up optimized joint velocity computation
* Added gradient for objective function
* Implemented and tested ComputeJointVelocityFromTwist in util
* Added workspace planner to prpy.planning __init__.py
* Fixed a number of bugs related to workspace planner.
  This commit addresses several major bugs unmasked by the workspace planner.
  1) Fixed a bug in cloning an environment into itself
  (needed for recursive `@PlanningMethod`s)
  2) Fixed a bug in incorrect formatting of RetimeTrajectory error messages.
  3) Fixed numerous small issues in the workspace planner:
  a) Returning a 1-waypoint trajectory when started in-contact with an object.
  b) Fixed max_distance calculation error from missing `numpy.copy()`
  c) Simplified some of the workspace planning logic.
* Changed Clone() to lock by default.
  This emulates the functions of `with env:` more closely,
  which is useful because the call `with Clone(env):` looks
  extremely similar.
* Added workspace planner to prpy.planning init.py
  This just adds the new workspace planner to __init__.py so it can be imported from `prpy.planning`.
* Bugfixes for SimplifyTrajectory and NominalConfiguration.
  - SimplifyTrajectory has been modified to gracefully return if passed a trajectory with only one waypoint.
  - NominalConfiguration optionally takes a maximum allowable DOF range, which allows robots with fully redundant configurations (i.e. multiple rotation joints) to ignore IK configurations for which a closer solution must exist.
* Changed default chunksize of tsr_planner to be 1.
* Added PlanToEndEffectorOffset method. Untested.
* Added fix to make ik_ranking default to ignoring multirotation IK solutions.
* Added fix for SimplifyTrajectory to handle 1-waypoint trajectories.
* Added patch for correctly cloning grabbed objects.
  Due to a bug in OpenRAVE, cloned grabbed objects may have incorrect
  adjacency properties, causing them to not be evaluated correctly
  for self collisions (with the robot).  This bugfix forces cloned
  environments to regrab all objects, which resets these incorrect links.
* Added PlanToEndEffectorPose method that creates a geodesic workspace trajectory from start to goal and sends it off to PlanWorkspacePath
* Added default 1 rotation offset to nominal configuration.
* Fixed missing and child-referencing constructors in CloneBindings.
* Changed Cloned(clone_env=...) to Cloned(into=...).  Also added docs.
* Enabled syntax highlighting.
* Added a new subsection.
* Added InstanceDeduplicator examples.
* Improved the planning README (thanks @cdellin).
* First pass at greedy IK planner
* Added numerous bugfixes for cloning and deferred planning.
  * Deferred planning now consistently returns trollius.futures.Future
  * Fixed bug in robot PlanWrapper that caused deferred planning to terminate early.
  * Cloned() references are now explicitly passed their clone environment.
  * .Cloned() helper method added to environments created by Clone(env)
  * Existing clone references consolidated to minimize Cloned() lookups.
* Stripped WAMRobot to the bare basics.
* Fixed indexing bug in IK ranking function.
* Generalized the nominalconfiguration ranker to accept angle bounds.
* Adjusted default chunk size for tsr sampler and removed unused param.
* Added multirotation filtering to nominal configuration IK ranker.
* Added a MacSmoother test.
* Simplify the trajectory in MacSmoother.
* Made the Timer log message optional.
* Fixed the ParabolicSmoother wrapper class.
* Call SimplifyTrajectory before an OpenRAVE retimer.
* Fixed argument names in robot.SimplifyPath.
* Modified _PlanWrapper to set linear interpolation.
* Added MacSmoother to wrap or_pr_spline.
* Update README.md
* More planner documentation.
* Switched fallback retimer from linear to parabolic.
* Added env lock to get active manipulator and DOF values at start.
* Fixed incorrect swapping between Arm DOF Indices and Robot DOF Indices.
* Implemented TsrPlanner as standalone from IkPlanner.
* Added explicit chunk size parameter.
* Added restructured IK and TSR planners that can do multiple goals.
* Wrapped OpenRAVE retimers in the planning pipeline.
* Added SimplifyPath tests.
* Added SimplifyPath method using OMPL.
* Fixed NamedPlanner in cloned environments.
* Added PlanToEndEffectorPose tests.
* Added more PlanToConfiguration tests.
* Strip extraneous groups from the CBiRRT output.
* Added basic planning unit tests.
* Disabled smoothing in OMPL.
* Disabled smoothing in CBiRRT.
* Contributors: Jennifer King, Michael Koval, Pras, Pras Velagapudi, Siddhartha Srinivasa, Stefanos Nikolaidis

0.3.1 (2015-02-10)
------------------
* Added fix for error caused by clone_env being set to None.
* Contributors: Michael Koval, Pras


0.3.0 (2015-02-06)
------------------
* Adds the ability to pass a defer=True to PlanningMethods and ExecuteTrajectory.
* Fixed detection of missing CBiRRT module.
* Contributors: Michael Koval, Pras Velagapudi

0.2.0 (2015-01-29)
------------------
* Adding `kw_args` to CHOMP's `OptimizeTrajectory` so execute flag doesn't cause error.
* Disabling `PlanToTSR` in CHOMP due to inconsistent behavior.
* Added linear path segment simplification.
* Changed the metaplanners to only catch `PlanningError`s instead of all Exceptions.
* Planning to goal sets with OMPL.
* Made `base.BarrettHand` compatable with the Hydro HERB model.
* Added `RobotStateSaver` to set active manipulator DOFs before IK planning.
* PEP8/lint fixes.
* Removed type(list) check in `planning.openrave` (this check is too strict).
* Fixed `NominalConfiguration`: norm was computed on wrong axis.
* Bugfixes for SnapPlanner.
* Set the default `range` for OMPL RRT-Connect.
* Expose OpenRAVE's builtin planners as prpy Planners.
* Changed `ValueError` to `TypeError` for wrong goals type
* Some error checking for input goals
* Removed robot-specific imports from PrPy.
* Added several unit tests.
* Fixed DOF values in `CHOMPDistanceFieldManager`.
* Improved `SnapPlanner` docstrings.
* `SnapPlanner` checks the straight-line trajectory
  Switched to new or_ompl plugin architecture.
* Added `OpenHand` and `CloseHandTight` functions
* Use DOF resolution for snapping (`#16 <https://github.com/personalrobotics/prpy/issues/16>`_ and `#17 <https://github.com/personalrobotics/prpy/issues/17>`_).
* Check collisions in `SnapPlanner` (fix for `#18 <https://github.com/personalrobotics/prpy/issues/18>`_).
* Added `RetimeTrajectory` function that fall backs on linear smoothing.
* Added documentation for TSR library.
* Improved docstring for `ompl.PlanToTSR`
* Adding `PlanToTSR` method
* Contributors: Jennifer King, Michael Koval, Pras Velagapudi, Stefanos Nikolaidis, Siddhartha Srinivasa

0.1.0 (2014-12-11)
------------------
* Fixed tab completion on MobileBase.
* Added pitcher TSRs.
* Added proper license information.
* Added `TSRLibrary` class.
* Added CHOMP `DistanceFieldManager` class.
* Added `CopyTrajectory` helper function.
* Added `PlanToConfigurations` planning function.
* Added `OptimizeTrajectory` planning function to CHOMP.
* Fixed a major memory leak in environment cloning (`#9 <https://github.com/personalrobotics/prpy/issues/9>`)
* Fixed MICO hand controller.
* Registered Python unit tests with Catkin.
* Contributors: Evan Shapiro, Jennifer King, Michael Koval, Pras Velagapudi, Stefanos Nikolaidis

0.0.1 (2014-09-08)
------------------
* Changes to allow for passing planner options.
* Fixed the TF token with simtime.
* Made dependency_manager a noop in Catkin.
* Helper tool for aligning TF frames.
* Added save_trajectory helper function.
* Added load_trajectory function.
* Merge branch 'master' of github.com:personalrobotics/prpy
* Fixed a prpy.bind memory leak with cloning.
* Merge pull request `#3 <https://github.com/personalrobotics/prpy/issues/3>`_ from personalrobotics/patch/switchToCatkinCheckForSetChuckingDirection
  Only call SetChuckingDirection on the new HERB model.
* fixed fuerte check for SetChuckingDirection
* Merge pull request `#2 <https://github.com/personalrobotics/prpy/issues/2>`_ from personalrobotics/feature_fuerte_support
  backwards compatibility for fuerte
* Fixed the Catkin test.
* added back fuerte support
* Re-enabled canonical instance caching.
* Added support for Cloned() again.
* Cleanup memory using the removal callback.
* Switched to UserData for the InstanceDeduplicator.
* Added the new UserData-based storage method.
* Merge branch 'master' of github.com:personalrobotics/prpy
* Added a disable_padding helper function.
* Fixed a major bug in PrPy's OMPL wrapper.
  The OMPL planner was getting called twice, instead of the OMPL simplifier. This
  could cause the planner to return invalid output trajectory.
* Merge branch 'master' of github.com:personalrobotics/prpy
* Added a hack to fix smoothed trajectories.
* Added shortcutting to OMPLPlanner.
* Set closing direction for the BarrettHand.
  This cannot be inferred from the SRDF.
* Fixed controllers.
* Fixed WAM IK by adding precision = 4.
* Upgraded dependency_manager for Catkin.
* added a height paramter for tsr
* Added several missing docstrings.
* move until touch fix to work on sim and real robot
* Fix of CreateAndDiscretizeTSR for boxes
* Adding retime of base trajectories even when not in simulation
* stat
* discretized tsr
* mkplanner only checks collision against active bodies for faster planning
* fixed move until touch error...had to change things back
* Moving location of the writing of the traj file by cbirrt
* fixed move until touch for execution
* Catkin-ized PrPy.
* Fixing parameter passing of return first
* Updating to allow for passing through command line parameters
* changed simulated moveuntiltouch collision checking
* Cleaning up parameter setting. Now just send raw yaml to sbpl planner and do all parsing there.
* changed disable kin body logs -> debug
* added locking to cloning code
* Fixed base planning.
* Removed Fastest.
* Removed unimplemented Fastest planner.
* Cleaned up docstring building.
* Fixed CHOMP failures from terminating the Ranked metaplanners.
* Fixed some typos.
* Added unittests for metaplanners.
* Fixed another reference to is_planning_method.
* Fixed a hilarious bug where accessing a docstring triggered planning.
* Fixed an edge case with planner docstring concatenation.
* Added a helper function for removing the ROS log handler.
* Adding PlanToTSR function to chomp
* Updating recorder to be able to manually start and stop it
* removed printing statement for debug
* hacky fix for move hand straight
* Added some notes to AdaptTrajectory.
* fixed moveuntiltouch for simulation
* Fixed an environment locking issue in OMPLPlanner.
* added mico related sources
* added GetVelocityLimits command
* Cleaning up the way parameters are sent to the sbpl planner
* Adding more informative logging of errors
* Adding function for testing a trajectory for velocity limit violations
* is in collision
* adapttrajectory function
* adapttrajectory function
* Adding error imports
* Expanding action set
* Fixing up planning pipeline to work with base
* adapttrajectory function
* Updates to try to integrate base planner
* ExecuteTrajectory now supports affine DOFs.
* Creating a distance field after planning works.
* Switched Rotate to run a base trajectory.
* Moved trajectory execution from HerbPy.
* Added support for affine DOF trajectories.
* Updating sbpl to call into the base planner
* added sbpl base planner structure
* fixed function signature in mobilebase
* fixed syntax error in mobilebase
* added DriveStraightUntilForce to mobilebase
* Adding mobilebase class for the robot base
* Found the source of the MacTrajectory spam.
* We're now able to plan outside of joint limits.
* Fixing bugs. Moved declaration of collided_with_obj in wam to fix problem when not in simulation. Added ik planner. Removed the PlanToIK function from planning base. Fixed minor distance calculation bug in mk planner. Modified Ranked to not call planners without the method implemented.
* Improved planner docstrings.
* Docstrings are finally working with planning!
* Switched the dispatch mechanism for planning calls.
* Closer to preserving docstrings for planning.
* Added PlanToNamedConfiguration to manipulators.
* fix bug in joint limits and mkplanner for movehandstraight
* Added an IK ranker for a nominal configuration.
* Added documentation to wam functions.
* Modified MoveUntilTouch to accept a maximum distance.
* Added support for a minimum distance in PlanToEndEffectorOffset.
* Added OPENRAVE_DATABASE to dependency_manager.
* Added scipy as a rosdep for prpy (used for saving images out).
* Merging prpy branch changes for door opening back into trunk
* Draft of the MongoDB metadata store.
* simulated move until touch
* Added a <review> tag.
* Added PlanToEndEffectorPose to the snap planner.
* Fixed PlanToEndEffectorPose in GSCHOMP. It seems to be working well.
* Fixed snap planner with bimanual trajectories.
* lowering default chomp iterations
* fixed prpy exceptions
* Updating to use the default openrave multi-controller instead of or_multi_controller
* Fixing error when trying to set hand dof values
* Adding snap planner. Adding mk planner to init file. Fixing RetimeTrajectory and ExecuteTrajectory to ignore trajectories with less than 2 waypoints.
* Removing references to manip.parent in favor of manip.GetRobot()
* Adding missing import of numpy
* Making planning robust to exceptions other than type PlanningError that may occur during planning
* Improvements to the tactile rendering code.
* Merging back changes from Toyota visit
* Fixed an import * warning.
* Added TakeSnapshot.
* Adding ability to visualize trajectories
* Added utility functions from herbpy.
* Adding logic to clone trajectory back to live environment during calls to PlanToNamedConfiguration
* Adding an input to specifiy distance from ee to palm.
* Adding or_multi_controller to dependencies.  Fixing dependency manager.
* Removed circular herbpy reference.
* Added copyright headers.
* Copied rave and kin utilities from prrave.
* Removed prrave.tsr dependency.
* Added the dependency manager.
* Added Recorder and SetCameraFromXML to util.
* Added a wrapper for or_ompl.
* Added IK ranking code.
* Implemented PlanToIK.
* Removed explicit planner type registration.
* Fixing logic errors in checking for successful plans
* Adding PlanToTSR method. Probably want to remove once we fix problems with call functions not defined on all planners.
* Adding robot to PlanToTSR. Passing robot to Plan method.
* Updated PlanWrapper function to properly clone during planning.
* Cleaned up tactile sensor rendering code.
* Merged get_origins() and get_normals().
* More complete cloning implementation.
* Partial support for cloning deduplicated instances.
* import fixes in tsrlibrary
* Fixing broken tsr library
* Moving function to get a no tilt tsr into tsrlibrary
* Moving tsr classes from prrave to prpy. Note: Moved kin.py for now. This should be replaced with parallel calls in openravepy. However, initial testing shows slightly different functionality.  Need to resolve before removing kin.
* Visualize tactile sensors as vectors.
* Refactored to replace a loop with NumPy calls.
* Utility classes for visualizing tactile sensors.
* Added logger utilities.
* Cloning tweaks.
* Copied WAM and BarrettHand functionality from AndyPy.
* Moved clone into the prpy module.
* Utilities for cloning environmetns.
* CHOMP successfully runs in parallel with CBiRRT.
* Automatically run planners in cloned environments.
* Committed pending changes.
* Support for loading named configurations from YAML.
* Utility class for named configurations.
* Bind with a lazily evaluated planner.
* Added the executer wrapper to the planning interface.
* Partial implementation of the new planning pipeline.
* Moved system packages to pr-ros-pkg.
* Created a prpy directory.
* Contributors: Anca Dragan, Andrey Kurenkov, Evan Shapiro, Jennifer King, Jonathan Gammell, Joshua Haustein, Michael Koval, Mike Koval, Prasanna Velagapudi, Shervin Javdani, Tekin Meriçli
