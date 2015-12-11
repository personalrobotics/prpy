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
Lets return to our previous example of selecting a pose for the end-effector to allow a manipulator to grasp a bottle. The following code shows the python commands that allow the TSR to be defined:
```python
ipython> bottle = env.GetKinBody('fuze')
ipython> T0_w = bottle.GetTransform()  # We use the bottle's coordinate frame as the w frame
# Now define Tw_e to represent the pose of the end-effector relative to the glass
ipython> Tw_e =  numpy.array([[ 0., 0., 1., -total_offset], 
                              [1., 0., 0., 0.], 
                              [0., 1., 0., 0.08], # glass height
                              [0., 0., 0., 1.]])  
ipython> Bw = numpy.zeros((6,2))
ipython> Bw[2,:] = [0.0, 0.02]  # Allow a little vertical movement
ipython> Bw[5,:] = [-numpy.pi, numpy.pi]  # Allow any orientation about the z-axis of the bottle
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
A single TSR, or finite set of TSRs, is sometimes insufficient to capture pose ocnstraints of a given task. To describe more complex constraints, such as closed-chain kinematics, we can use a TSR Chain.  Consider the example of opening a refrigerator door while allowing the manipulator to rotate around the handle. Here, the constraint on the motion of the hand is defined by the product of two constraints.  The first constraint describes valid locations of the handle, which all lie on the arc defined by the position of the handle relative to the door hinge.  The second constraint defines the position of the robot end-effector relative to the handle. Each of these constraints can be defined by a single TSR. In order to specify the full constraint on the hand motion, we link the TSRs in a TSR Chain.  

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
Similar to the TSRs, we can sample and compute distance to chains using the ```sample``` and ```distance``` functions respectively.

## Prpy Planning support for TSRs
