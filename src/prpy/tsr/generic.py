import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import TSR, TSRChain

def get_manip_idx(robot, manip=None):
    ''' 
    Helper function for getting the manipulator index
    to be used by the TSR
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        import openravepy
        p = openravepy.KinBody.SaveParameters
        with robot.CreateRobotStateSaver(p.ActiveManipulator):
            robot.SetActiveManipulator(manip)
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()
    return manip_idx

def cylinder_grasp(robot, obj, obj_radius, obj_height, 
                   lateral_offset = 0.0, 
                   vertical_tolerance = 0.02,
                   yaw_range = None,
                   manip = None, **kwargs):
    """
    NOTE: This function makes the following assumptions:
      1. The end-effector is oriented such that the z-axis is out of the palm
          and the x-axis should be perpendicular to the object
      2. The object coordinate frame is at the bottom, center of the object
          
    @param robot The robot performing the grasp
    @param obj The object to grasp
    @param obj_radius The radius of the object
    @param obj_height The height of the object
    @param lateral_offset The lateral offset from the edge of the object
      to the end-effector
    @param vertical_tolerance The maximum vertical distance from the vertical center
      of the object that the grasp can be performed
    @param yaw_range Allowable range of yaw around object (default: [-pi, pi])
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    manip_idx = get_manip_idx(robot, manip=manip)

    T0_w = obj.GetTransform()
    total_offset = lateral_offset + obj_radius

    # First hand orientation
    Tw_e_1 = numpy.array([[ 0., 0., 1., -total_offset], 
                        [1., 0., 0., 0.], 
                        [0., 1., 0., obj_height*0.5], 
                        [0., 0., 0., 1.]])

    Bw = numpy.zeros((6,2))
    Bw[2,:] = [-vertical_tolerance, vertical_tolerance]  # Allow a little vertical movement
    if yaw_range is None:
        Bw[5,:] = [-numpy.pi, numpy.pi]  # Allow any orientation
    else:
        Bw[5,:] = yaw_range
    
    grasp_tsr1 = TSR(T0_w = T0_w, Tw_e = Tw_e_1, Bw = Bw, manip = manip_idx)
    grasp_chain1 = TSRChain(sample_start=False, sample_goal = True, 
                            constrain=False, TSR = grasp_tsr1)

    # Flipped hand orientation
    Tw_e_2 = numpy.array([[ 0., 0., 1., -total_offset], 
                          [-1., 0., 0., 0.], 
                          [0.,-1., 0., obj_height*0.5], 
                          [0., 0., 0., 1.]])


    grasp_tsr2 = TSR(T0_w = T0_w, Tw_e = Tw_e_2, Bw = Bw, manip = manip_idx)
    grasp_chain2 = TSRChain(sample_start=False, sample_goal = True, 
                            constrain=False, TSR = grasp_tsr2)

    return [grasp_chain1, grasp_chain2]


def box_grasp(robot, box, length, width, height,
              lateral_offset = 0.0,
              lateral_tolerance = 0.02,
              manip = None, **kwargs):
    """
    This returns a set of 12 TSRs. There are two TSRs for each of the 
    6 faces of the box, one for each orientation of the end-effector.
    @param robot The robot performing the grasp
    @param box The box to grasp
    @param length The length of the box - along its x-axis
    @param width The width of the box - along its y-axis
    @param height The height of the box - along its z-axis
    @param lateral_offset - The offset from the edge of the box to the end-effector
    @param lateral_tolerance - The maximum distance along the edge from
      the center of the edge that the end-effector can be placed and still achieve
      a good grasp
    @param manip The manipulator to perform the grasp, if None the active 
      manipulator on the robot is used
    """
    manip_idx = get_manip_idx(robot, manip=manip)

    T0_w = box.GetTransform()
    
    # Top face
    Tw_e_top1 = numpy.array([[0., 1.,  0., 0.],
                             [1., 0.,  0., 0.],
                             [0., 0., -1., lateral_offset + 0.5*height],
                             [0., 0.,  0., 1.]])
    Bw_top1 = numpy.zeros((6,2))
    Bw_top1[1,:] = [-lateral_tolerance, lateral_tolerance]
    grasp_tsr1 = TSR(T0_w = T0_w, Tw_e = Tw_e_top1, Bw = Bw_top1, 
                     manip = manip_idx)
    grasp_chain1 = TSRChain(sample_start=False, sample_goal=True,
                            constrain=False, TSR=grasp_tsr1)

    # Side face
    Tw_e_side1 = numpy.array([[-1., 0.,  0., 0.],
                              [0., 0.,  1., -lateral_offset - 0.5*width],
                              [0., 1.,  0., 0.5*height],
                              [0., 0.,  0., 1.]])
    Bw_side1 = numpy.zeros((6,2))
    Bw_side1[0,:] = [-lateral_tolerance, lateral_tolerance]
    grasp_tsr2 = TSR(T0_w = T0_w, Tw_e = Tw_e_side1, Bw = Bw_side1, 
                     manip = manip_idx)
    grasp_chain2 = TSRChain(sample_start=False, sample_goal=True,
                            constrain=False, TSR=grasp_tsr2)

    # Side 2 face
    Tw_e_side2 = numpy.array([[-1., 0.,  0., 0.],
                              [0., 0., -1., lateral_offset + 0.5*width],
                              [0.,-1.,  0., 0.5*height],
                              [0., 0.,  0., 1.]])
    Bw_side2 = numpy.zeros((6,2))
    Bw_side2[0,:] = [-lateral_tolerance, lateral_tolerance]
    grasp_tsr3 = TSR(T0_w = T0_w, Tw_e = Tw_e_side2, Bw = Bw_side2, 
                     manip = manip_idx)
    grasp_chain3 = TSRChain(sample_start=False, sample_goal=True,
                            constrain=False, TSR=grasp_tsr3)
    
    #return [grasp_chain1, grasp_chain2, grasp_chain3]
    return [grasp_chain1]

def place_object(robot, obj, pose_tsr_chain, manip=None, 
                     **kwargs):
    '''
    Generates end-effector poses for placing an object.
    This function assumes the object is grasped when called
    
    @param robot The robot grasping the object
    @param bowl The grasped object
    @param pose_tsr_chain The tsr chain for sampling placement poses for the object
    @param manip The manipulator grasping the object, if None the active
       manipulator of the robot is used
    '''
    manip_idx = get_manip_idx(robot, manip=manip)
    if manip is None:
        manip = robot.GetManipulators()[manip_idx]

    ee_in_obj = numpy.dot(numpy.linalg.inv(obj.GetTransform()), 
                          manip.GetEndEffectorTransform())
    Bw = numpy.zeros((6,2)) 
   
    for tsr in pose_tsr_chain.TSRs:
        if tsr.manipindex != manip_idx:
            raise Exception('pose_tsr_chain defined for a different manipulator.')

    grasp_tsr = TSR(Tw_e = ee_in_obj, Bw = Bw, manip = manip_idx)
    all_tsrs = list(pose_tsr_chain.TSRs) + [grasp_tsr]
    place_chain = TSRChain(sample_start = False, sample_goal = True, constrain = False,
                           TSRs = all_tsrs)

    return  [ place_chain ]

def transport_upright(robot, obj, 
                      roll_epsilon=0.2, 
                      pitch_epsilon=0.2, 
                      yaw_epsilon=0.2,
                      manip=None, **kwargs):
    '''
    Generates a trajectory-wide constraint for transporting the object with little roll, pitch or yaw
    Assumes the object has already been grasped and is in the proper
    configuration for transport.

    @param robot The robot grasping the object
    @param obj The grasped object
    @param roll_epsilon The amount to let the object roll during transport (object frame)
    @param pitch_epsilon The amount to let the object pitch during transport (object frame)
    @param yaw_epsilon The amount to let the object yaw during transport (object frame)
    @param manip the manipulator grasping the object, if None the active manipulator 
       of the robot is used
    '''
    manip_idx = get_manip_idx(robot, manip=manip)
    if manip is None:
        manip = robot.GetManipulators()[manip_idx]

    ee_in_obj = numpy.dot(numpy.linalg.inv(obj.GetTransform()), 
                          manip.GetEndEffectorTransform())
    Bw = numpy.array([[-100., 100.], # bounds that cover full reachability of manip
                      [-100., 100.],
                      [-100., 100.],
                      [-roll_epsilon, roll_epsilon],
                      [-pitch_epsilon, pitch_epsilon],
                      [-yaw_epsilon, yaw_epsilon]])
    transport_tsr = TSR(T0_w = obj.GetTransform(),
                        Tw_e = ee_in_obj,
                        Bw = Bw,
                        manip = manip_idx)

    transport_chain = TSRChain(sample_start = False, sample_goal=False, 
                               constrain=True, TSR = transport_tsr)
    
    return [ transport_chain ]
