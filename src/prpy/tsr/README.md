# Task Space Regions

This directory contains the python interfaces necessary to specify Task Space Regions (TSRs). For a detailed description of TSRs and their uses, please refer to the 2010 IJRR paper entitled "Task Space Regions: A Framework for Pose-Constrained
Manipulation Planning" by Dmitry Berenson, Siddhartha Srinivasa, and James Kuffner.  A copy of this publication can be downloaded [here](https://www.ri.cmu.edu/pub_files/2011/10/dmitry_ijrr10-1.pdf).

## TSR Overview
A TSR is typically used to defined a constraint on the pose of the end-effector of a manipulator. For example consider a manipulator tasked with grabbing a bottle. The end-effector (hand) must be near the bottle, and oriented in a way that allows the fingers to grab around the bottle when closed. This set of workspace constraints on valid poses of the end-effector can be expressed as a TSR.

A TSR is defined by three components:
* T0_w - A transform from the world origin to the TSR frame w
* Tw_e - A transform from the TSR frame w to the end-effector
* Bw - A 6x2 matrix of bounds in the coordinates of w

The first three rows of Bw bound the allowable translation along the x,y and z axes (in meters).  The last three rows bound the allowable rotation about those axes (in radians), all in w frame.  Note that this asumed Roll-Pitch-Yaw (RPY) Euler angle convention.

## Example Usage
Lets return to our previous example of selecting a pose for the end-effector to allow a manipulator to grasp a bottle. The following code shows the python commands that allow the TSR to be defined:
```
```
