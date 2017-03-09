import numpy as np 
import parameters
def proxemic_force_function(human_position, robot_position):
    return field_value

def radial_force_function(human_positions, robot_position):
    return force_value

def linear_goal_force_function(robot, goal):
    k = 2**-2 #
    max_factor = 2**4

    rx = robot[-1,parameters.X_POS]
    ry = robot[-1,parameters.Y_POS]

    return np.clip(
        k*np.array([goal[0] - rx, goal[1] - ry, 0]),
        [-max_factor*k,-max_factor*k,-max_factor*k],
        [max_factor*k,max_factor*k,max_factor*k],        
    )

def constant_goal_force_function(robot, goal):
    pass

def gaussian_boundary_force_function(robot, module_size):
    dfdx = 0
    dfdy = 0
    A = 2**-1 #Figure out this value; magnitude of distribution
    sigma = 6*2.54E-2 #Figure out this value; width of distribution; equal in x and y

    rx = robot[-1,parameters.X_POS]
    ry = robot[-1,parameters.Y_POS]

    for x,y in [(0,0),module_size]:
        dfdx += (rx - x)*(A/sigma**2)*np.exp(-((rx-x)**2/(2*sigma**2)))
        dfdy += (ry - y)*(A/sigma**2)*np.exp(-((ry-y)**2/(2*sigma**2)))

    return np.array([dfdx, dfdy, 0])

def apf_path_planner(robot_initial_condition,goals,humans,
  human_force_function,
  module_size=parameters.module_size,
  boundary_force_function=gaussian_boundary_force_function,
  goal_force_function=linear_goal_force_function):

    robot_path = robot_initial_condition.copy()

    for goal in goals:
        while True:
            force_vector = np.zeros((3,))

            force_vector += goal_force_function(robot_path, goal)
            force_vector += boundary_force_function(robot_path, module_size)
            # for human in humans:
            #     force_vector += human_force_function(robot_path, human)
            force_vector += human_force_function(robot_path, humans)

            # add some damping
            force_vector += -20 * robot_path[-1,parameters.VEL_POS]

            new_state = np.zeros((1,6))

            new_state[0,parameters.VEL_POS] = (robot_path[-1,parameters.VEL_POS]
                + force_vector/parameters.robot_inertia_vec )

            new_state[0,parameters.POS_POS] = (robot_path[-1,parameters.POS_POS]
                + new_state[0,parameters.VEL_POS] )

            robot_path = np.vstack((robot_path, new_state))

            # checks
            is_stuck = np.all(
                (np.allclose(force_vector,np.zeros((3,1))), 
                np.allclose(robot_path[-1,parameters.VEL_POS],np.zeros((3,1))),
                ))
            if is_stuck:
                break

    return robot_path







