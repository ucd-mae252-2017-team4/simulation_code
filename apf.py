def proxemic_force_function(human_position, robot_position):
    return field_value

def radial_force_function(human_position, robot_position):
    return field_value

def default_goal_force_function(goal, robot):
    return force_value

def default_boundary_force_function(robot,module_size):
    return force_value

def apf_path_planner(robot,goal,humans,module_size=(8.5,4.3),
  human_force_function,
  goal_force_function=default_goal_force_function,
  boundary_force_function=default_boundary_force_function):


    while True:
        force_vector = np.zeros((3,1))

        force_vector += goal_force_function()
        force_vector += boundary_force_function(robot, module_size)
        for human in humans:
            force_vector += human_force_function(human,robot)

        robot.time_step(
            force_vector[0]/robot.m + robot.dx,
            force_vector[1]/robot.m + robot.dy,
            force_vector[2]/robot.izz + robot.dtheta)

        # checks
        is_stuck = np.all(
            (np.allclose(force_vector,np.zeros((3,1))), 
            np.allclose(robot.velocities,np.zeros((3,1))),
            ))
        if is_stuck:
            break






