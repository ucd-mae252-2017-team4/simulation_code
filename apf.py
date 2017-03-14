import numpy as np 
import parameters

dt = 2**-2


def proxemic_force_function(human_position, robot_position):
    return field_value

def radial_force_function(human_positions, robot_position):
    return force_value

def linear_goal_force_function(robot, goal, k=2**-3):
    # k = 2**-3 # 2**-3 worked until I made the integration better
    max_f = 2*parameters.robot_length*parameters.goal_cutoff*k

    rx = robot[:,parameters.X_POS]
    ry = robot[:,parameters.Y_POS]

    gx = goal[parameters.X_POS]
    gy = goal[parameters.Y_POS]

    fx = np.clip(k*(gx - rx), -max_f, max_f)
    fy = np.clip(k*(gy - ry), -max_f, max_f)

    # print('fx',fx)
    # print('fy',fy)
    # print('fz',np.zeros_like(fx))
    # print('stack',np.stack((fx,fy, np.zeros_like(fx)),axis=1))
    return np.stack((fx,fy, np.zeros_like(fx)),axis=1)

def constant_goal_force_function(robot, goal):
    pass

def gaussian_boundary_force_function(robot, module_size):
    A = 1 #Figure out this value; magnitude of distribution
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
  human_force_function, goal_k=2**-3, damping_b=5,
  module_size=parameters.module_size,
  boundary_force_function=gaussian_boundary_force_function,
  goal_force_function=linear_goal_force_function):

    # robot_path = robot_initial_condition.copy()
    robot_path = np.zeros((1,9))
    robot_path[0,:6] = robot_initial_condition.copy()

    for goal in goals:
        while True:
            force_vector = np.zeros((1,3))
            # print(human_force_function)
            force_vector += goal_force_function(robot_path[[-1],:], goal, goal_k)
            force_vector += boundary_force_function(robot_path[[-1],:], module_size)
            force_vector += human_force_function(robot_path[[-1],:], humans)
            force_vector += -damping_b*robot_path[[-1],parameters.VEL_POS]

            force_norm = np.linalg.norm(force_vector[:,:-1])
            if force_norm > parameters.robot_max_thrust:
                force_vector *= parameters.robot_max_thrust/force_norm

            new_state = np.zeros((1,9))

            new_state[0,parameters.VEL_POS] = (robot_path[-1,parameters.VEL_POS]
                + force_vector*dt/(parameters.robot_inertia_vec) )

            # new_state[0,parameters.VEL_POS] = force_vector/parameters.robot_inertia_vec

            new_state[0,parameters.POS_POS] = (robot_path[-1,parameters.POS_POS]
                + robot_path[-1,parameters.VEL_POS]*dt ) #new_state[0,parameters.VEL_POS]/dt )

            new_state[0,6:] = force_vector

            robot_path = np.vstack((robot_path, new_state))

            # checks
            # print('force',force_vector)
            # print(np.allclose(force_vector,0))

            # print(np.allclose(robot_path[-1,parameters.VEL_POS],0))
            if np.any(np.isnan(new_state)):
                print("NAN")
                break

            if np.any(((robot_path[[-2],parameters.XY_POS] > 1.5*parameters.module_size),(robot_path[[-2],parameters.XY_POS] < -0.5*parameters.module_size) )):
                print("OUT OF BOUNDS")
                break

            if robot_path.shape[0]*dt > 10E3:
                break


            is_stuck = (np.allclose(force_vector,0) and 
                np.allclose(robot_path[-1,parameters.VEL_POS],0))

            if is_stuck:
                break

    return robot_path







