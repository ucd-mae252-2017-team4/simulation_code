import numpy as np 
import parameters
def proxemic_force_function(human_position, robot_position):
    return field_value

def radial_force_function(human_positions, robot_position):
    return force_value

def linear_goal_force_function(robot, goal):
    k = 2**-2 #
    max_factor = 2**4

    rx = robot[:,parameters.X_POS]
    ry = robot[:,parameters.Y_POS]

    gx = goal[parameters.X_POS]
    gy = goal[parameters.Y_POS]

    fx = np.clip(k*(gx - rx), -max_factor*k, max_factor*k)
    fy = np.clip(k*(gy - ry), -max_factor*k, max_factor*k)

    # print('fx',fx)
    # print('fy',fy)
    # print('fz',np.zeros_like(fx))
    # print('stack',np.stack((fx,fy, np.zeros_like(fx)),axis=1))
    return np.stack((fx,fy, np.zeros_like(fx)),axis=1)

def constant_goal_force_function(robot, goal):
    pass

def gaussian_boundary_force_function(robot, module_size):
    A = 2**-1 #Figure out this value; magnitude of distribution
    sigma = 6*2.54E-2 #Figure out this value; width of distribution; equal in x and y

    rx = robot[:,parameters.X_POS]
    ry = robot[:,parameters.Y_POS]

    dfdx = np.zeros_like(rx)
    dfdy = np.zeros_like(ry)

    for x,y in [(0,0),module_size]:
        dfdx += (rx - x)*(A/sigma**2)*np.exp(-((rx-x)**2/(2*sigma**2)))
        dfdy += (ry - y)*(A/sigma**2)*np.exp(-((ry-y)**2/(2*sigma**2)))

    return np.stack([dfdx, dfdy, np.zeros_like(dfdx)],axis=1)

def apf_path_planner(robot_initial_condition,goals,humans,
  human_force_function,
  module_size=parameters.module_size,
  boundary_force_function=gaussian_boundary_force_function,
  goal_force_function=linear_goal_force_function):

    robot_path = robot_initial_condition.copy()

    for goal in goals:
        while True:
            force_vector = np.zeros((3,))
            # print('froce',force_vector)
            force_vector = force_vector + goal_force_function(robot_path[[-1],:], goal)
            # print('goal',force_vector)
            force_vector = force_vector + boundary_force_function(robot_path[[-1],:], module_size)
            # print('boundary',force_vector)
            # for human in humans:
            #     force_vector += human_force_function(robot_path, human)
            print('goal+boundary',force_vector)
            res = human_force_function(robot_path[[-1],:], humans) 
            # if isinstance(res,tuple):
            #     return res
            force_vector = force_vector + res
            print('human',res)

            # add some damping
            force_vector = force_vector - 20 * robot_path[[-1],parameters.VEL_POS]
            print('dampingy',-20 * robot_path[[-1],parameters.VEL_POS])


            new_state = np.zeros((1,6))

            new_state[0,parameters.VEL_POS] = (robot_path[-1,parameters.VEL_POS]
                + force_vector/parameters.robot_inertia_vec )

            new_state[0,parameters.POS_POS] = (robot_path[-1,parameters.POS_POS]
                + new_state[0,parameters.VEL_POS] )

            robot_path = np.vstack((robot_path, new_state))

            # checks
            # print('force',force_vector)
            # print(np.allclose(force_vector,0))

            # print(np.allclose(robot_path[-1,parameters.VEL_POS],0))
            if np.any(np.isnan(new_state)):
                print("NAN")
                break

            if np.any(np.abs(robot_path[[-2],parameters.XY_POS]) > 2*parameters.module_size):
                print("OUT OF BOUNDS")
                break


            is_stuck = (np.allclose(force_vector,0) and 
                np.allclose(robot_path[-1,parameters.VEL_POS],0))

            if is_stuck:
                break

    return robot_path







