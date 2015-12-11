# Task Space Regions

This directory contains the python interfaces necessary to specify Task Space Regions (TSRs). For a detailed description of TSRs and their uses, please refer to the 2010 IJRR paper entitled "Task Space Regions: A Framework for Pose-Constrained
Manipulation Planning" by Dmitry Berenson, Siddhartha Srinivasa, and James Kuffner.  A copy of this publication can be downloaded [here](https://www.ri.cmu.edu/pub_files/2011/10/dmitry_ijrr10-1.pdf).

## TSR Overview
A TSR is typically used to defined a constraint on the pose of the end-effector of a manipulator. For example consider a manipulator tasked with grabbing a glass. The end-effector (hand) must be near the glass, and oriented in a way that allows the fingers to grab around the glass when closed. This set of workspace constraints on valid poses of the end-effector can be expressed as a TSR.

A TSR is defined by three components:
* ```T0_w``` - A transform from the world origin to the TSR frame w
* ```Tw_e``` - A transform from the TSR frame w to the end-effector
* ```Bw``` - A 6x2 matrix of bounds in the coordinates of w

The first three rows of Bw bound the allowable translation along the x,y and z axes (in meters).  The last three rows bound the allowable rotation about those axes (in radians), all in w frame.  Note that this asumed Roll-Pitch-Yaw (RPY) Euler angle convention.

### Example: Defining a TSR
Lets return to our previous example of selecting a pose for the end-effector to allow a manipulator to grasp a glass. The following code shows the python commands that allow the TSR to be defined:
```python
ipython> glass = env.GetKinBody('plastic_glass')
ipython> T0_w = glass.GetTransform()  # We use the glass's coordinate frame as the w frame
# Now define Tw_e to represent the pose of the end-effector relative to the glass
ipython> Tw_e =  numpy.array([[ 0., 0., 1., -total_offset], 
                              [1., 0., 0., 0.], 
                              [0., 1., 0., 0.08], # glass height
                              [0., 0., 0., 1.]])  
ipython> Bw = numpy.zeros((6,2))
ipython> Bw[2,:] = [0.0, 0.02]  # Allow a little vertical movement
ipython> Bw[5,:] = [-numpy.pi, numpy.pi]  # Allow any orientation about the z-axis of the glass
ipython> robot.right_arm.SetActive()  # We want to grasp with the right arm
ipython> manip_idx = robot.GetActiveManipulatorIndex()
ipython> grasp_tsr = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
```
### Example: Using a TSR
The following code shows an example of how to use a TSR to find a collision-free configuration for the manipulator that allows for a valid grasp:
```python
ipython> ee_sample = grasp_tsr.sample() # Compute a sample pose of the end-effector
ipython> ik = robot.right_arm.FindIKSolution(ee_sample, openravepy.IkFilterOptions.CheckEnvCollisions)  
```
```ik``` will now contain a configuration for the arm.  This configuration could be given as a goal to a planner to move the robot into place for the grasp:
```python
ipython> robot.right_arm.PlanToConfiguration(ik, execute=True)
```
### Example: Determining if a configuration is within a TSR
In the following code snippet, we show a method for determining whether or not the current pose of the manipulator meets the constraint by using the ```distance``` function defined on the TSR.
```python
ipython> current_ee_pose = robot.right_arm.GetEndEffectorTransform()
ipython> dist_to_tsr = grasp_tsr.distance(current_ee_pose)
ipython> meets_constraint = (dist_to_tsr == 0.0)
```

## TSR Chains
A single TSR, or finite set of TSRs, is sometimes insufficient to capture pose constraints of a given task. To describe more complex constraints, such as closed-chain kinematics, we can use a TSR Chain.  Consider the example of opening a refrigerator door while allowing the manipulator to rotate around the handle. Here, the constraint on the motion of the hand is defined by the product of two constraints.  The first constraint describes valid locations of the handle, which all lie on the arc defined by the position of the handle relative to the door hinge.  The second constraint defines the position of the robot end-effector relative to the handle. Each of these constraints can be defined by a single TSR. In order to specify the full constraint on the hand motion, we link the TSRs in a TSR Chain.  

### Example: Defining a TSR Chain
In the following code snippet, we show how to define a TSR Chain for the example of opening the refrigerator door, allowing the robot's hand to rotate around the door handle.

First we define the TSR that constrains the pose of the handle
```python
ipython> T0_w = hinge_pose  # hinge_pose is a 4x4 matrix defining the pose of the hinge in world frame
# Now define Tw_e as the pose of the handle relative to the hinge
ipython> Tw_e = numpy.eye() # Same orientation as the hinge frame
ipython> Tw_e[0,3] = 0.4 # The handle is offset 40cm from the hinge along the x-axis of the hinge-frame
ipython> Bw = numpy.zeros((6,2)) # Assume the handle is fixed
ipython> fridge = env.GetKinBody('refridgerator')
ipython> fridge.SetActiveManipulator('door')
ipython> door_idx = fridge.GetActiveManipulatorIndex()
ipython> constraint1 = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = door_idx)
```

Next we define the TSR that constraints the pose of the hand relative to the handle
```python
ipython> T0_w = numpy.eye(4) # This will be ignored once we compose the chain
ipython> Tw_e = ee_in_handle # A 4x4 defining the desire pose of the end-effector relative to handle
ipython> Bw = numpy.zeros((6,2))
ipython> Bw(5,:) = [-0.25*numpy.pi, 0.25*numpy.pi]
ipython> robot.right_arm.SetActive() # use the right arm to grab the door
ipython> manip_idx = robot.GetActiveManipulatorIndex()
ipython> constraint2 = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
```

Finally, we compose these into a chain
```python
ipython> tsrchain = prpy.tsr.TSRChain(sample_start=False, sample_goal=False, constrain=True, 
                                      TSRs = [constraint1, constraint2])
```
Similar to the TSRs, we can sample and compute distance to chains using the ```sample``` and ```distance``` functions respectively. The ```sample_start```, ```sample_goal``` and ```constrain``` flags will be explained in the next section. 

## Prpy Planning support for TSRs
Several of the planners in the prpy [planning pipeline](https://github.com/personalrobotics/prpy/tree/master/src/prpy/planning) have some support for using TSRs for defining constriants through the ```PlanToTSR``` method. The method accepts as a list of ```TSRChain``` objects. The ```sample_start```, ```sample_goal``` and ```constrain``` flags on the each ```TSRChain``` indicate to the planner how the chain should be used.

### Example: Planning to a single TSR
Consider the example of grasping a glass. Given our ```grasp_tsr``` we would now like to generate a plan that moves the robot to any configuration such that the end-effector meets the constraint defined by the tsr.  The following code can be used to do this:
```python
ipython> tsrchain = prpy.tsr.TSRChain(sample_goal=True, sample_start=False, constrain=False,
                                       TSR = grasp_tsr)
```
Defining ```sample_goal=True``` tells the planner to apply the constraint only to the last point in the plan. Now we can call the planner:
```python
ipython> traj = robot.PlanToTSR([tsrchain])
```
### Example: Planning from a single TSR
Now imagine we wish to generate a plan that starts from any point in the grasp TSR and plans to a defined configuration, ```config```. The following code can be used to do this:
```python
ipython> tsrchain = prpy.tsr.TSRChain(sample_goal=False, sample_start=True, constrain=False,
                                       TSR = grasp_tsr)
```
Defining ```sample_start=True``` tells the planner to apply the constraint only to the first point in the plan. Now we can call the planner:
```python
ipython> traj = robot.PlanToTSR([tsrchain], jointgoals=[config])
```
### Example: Apply a TSR constraint across a full trajectory
In the refrigerator opening example, the TSR Chain defined a constraint on the motion of the end-effector that should be applied over the whole trajectory.  We defined:
```python
ipython> tsrchain = prpy.tsr.TSRChain(sample_start=False, sample_goal=False, constrain=True, 
                                      TSRs = [constraint1, constraint2])
```
Here ```constrain=True``` tells the planner to apply the constraint to every point in the plan.  Again, we can call the planner:
```python
ipython> traj = robot.PlanToTSR([tsrchain], jointgoals=[config])
```
Here, the caller must be careful to ensure that ```config``` meets the constraint defined by the TSR. 

### Example: Planning to a set of TSRs
Now imagine we had to TSRs, ```grasp1_tsr``` and ```grasp2_tsr``` the each defined a set of valid configurations for grasping.  We can ask the planner to generate a plan to any configuration that meets either the ```grasp1_tsr``` or the ```grasp2_tsr``` constraint in the following way:
```python
ipython> tsrchain1 = prpy.tsr.TSRChain(sample_goal=True, sample_start=False, constrain=False,
                                       TSR = grasp1_tsr)
ipython> tsrchain2 = prpy.tsr.TSRChain(sample_goal=True, sample_start=False, constrain=False,
                                       TSR = grasp2_tsr)
ipython> traj = robot.PlanToTSR([tsrchain1, tsrchain2])
```
## TSR Library
The prpy framework contains the ability to define and cache TSRChains that are commonly used by the robot. These pre-defined TSRChains can be accessed via the ```tsrlibrary``` defined on the robot. The following shows an example for how the TSR Library might be used:
```python
ipython> glass = env.GetKinBody('plastic_glass')
ipython> tsrlist = robot.tsrlibrary(glass, 'grasp')
ipython> traj = robot.PlanToTSR(tsrlist)
```

