import numpy as np 

def proxemic_force_function(human_position, robot_position):
    return field_value

def radial_force_function(human_positions, robot_position):
    return force_value

def linear_goal_force_function(robot, goal):
    k = 2**-4 #
    return k*np.array([goal[0] - robot.x, goal[1] - robot.y, 0])

def gaussian_boundary_force_function(robot, module_size):
    dfdx = 0
    dfdy = 0
    A = 1 #Figure out this value; magnitude of distribution
    sigma = 2 #Figure out this value; width of distribution; equal in x and y
    for x,y in [(0,0),module_size]:
        dfdx += (robot.x - x)*(A/sigma**2)*np.exp(-((robot.x-x)**2/(2*sigma**2)))
        dfdy += (robot.y - y)*(A/sigma**2)*np.exp(-((robot.y-y)**2/(2*sigma**2)))

    return np.array([dfdx, dfdy, 0])

def apf_path_planner(robot,goal,humans,
  human_force_function,
  module_size=(28,14),
  boundary_force_function=gaussian_boundary_force_function,
  goal_force_function=linear_goal_force_function):


    while True:
        force_vector = np.zeros((3,))

        force_vector += goal_force_function(robot, goal)
        force_vector += boundary_force_function(robot, module_size)
        # for human in humans:
        #     force_vector += human_force_function(robot, human)
        force_vector += human_force_function(robot, humans)

        # add some damping
        force_vector += -10 * robot.velocities

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






