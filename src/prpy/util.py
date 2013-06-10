import numpy, openravepy, time

def create_sensor(env, args, anonymous=True):
    sensor = openravepy.RaveCreateSensor(env, args)
    if sensor is None:
        raise Exception("Creating '%s' sensor failed." % args.split()[0])

    env.Add(sensor, anonymous)
    return sensor

def GetTrajectoryIndices(traj):
    joint_values_group = traj.GetConfigurationSpecification().GetGroupFromName('joint_values')
    return numpy.array([ int(index) for index in joint_values_group.name.split()[2:] ])

def WaitForControllers(controllers, timeout=None, rate=20):
    running_controllers = set(controllers)
    start_time = time.time()
    timestep = 1.0 / rate

    while running_controllers:
        # Check for a timeout.
        now_time = time.time()
        if timeout is not None and now_time - start_time > timeout:
            return False

        # Check if the trajectory is done.
        done_controllers = set()
        for controller in running_controllers:
            if controller.IsDone():
                done_controllers.add(controller)

        running_controllers -= done_controllers
        time.sleep(timestep)

    return True
