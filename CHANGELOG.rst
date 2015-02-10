^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package prpy
^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
