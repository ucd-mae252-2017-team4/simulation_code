import matplotlib as ml, matplotlib.pyplot as plt
import parameters
import astar_new as astar
import numpy as np

crew_color = 'k'
plt.ion()

def path_to_trajectory(path):

    trajectory = np.zeros((len(path),7))
    for node_idx, node in enumerate(path):
        trajectory[node_idx, parameters.XY_POS] = node[astar.X_I], node[astar.Y_I]
        v = node[astar.V_I]

        #get change in distance
        prev_node = path[node_idx-1]
        dx = node[astar.X_I] - prev_node[astar.X_I]
        dy = node[astar.Y_I] - prev_node[astar.Y_I]

        if dx == 0:
            trajectory[node_idx, parameters.VEL_POS] = 0, v, 0
        elif dy == 0:
            trajectory[node_idx, parameters.VEL_POS] = v, 0, 0
        else:
            trajectory[node_idx, parameters.VEL_POS] = v/np.sqrt(2), v/np.sqrt(2), 0

        ds = np.sqrt(dx**2 + dy**2)
        trajectory[node_idx, 6] = ds/v



    return trajectory


def draw_crew(crew):
    for cx,cy,ct in crew: #crew x, crew y, crew theta
        plt.gca().add_patch(
            plt.Circle(
                (cx,cy),
                radius=parameters.crew_radius,
                ec=crew_color,fill=False
            ))
        plt.gca().add_line(
            plt.Line2D(
                (cx,cx+np.cos(ct)*parameters.crew_radius), 
                (cy,cy+np.sin(ct)*parameters.crew_radius),
                color=crew_color
            ))

def draw_waypoints(mission):
    for wp in mission:
        plt.gca().add_patch(
            plt.Circle(
                wp,
                radius=parameters.robot_length*1.5,
                ec='m',fill=False, lw=3
            ))

# np array columns for robot_path: x,y,theta
def draw_path(robot_path, mission, crew):
    plt.figure()
    plt.plot(robot_path[:,0],robot_path[:,1])
    plt.axis('scaled')

    draw_crew(crew)

    last_x = -100
    last_y = -100
    for idx in range(robot_path.shape[0]):
        x,y = robot_path[idx,0], robot_path[idx,1]
        if (np.linalg.norm(np.array([x-last_x,y-last_y])) >= 2* parameters.robot_length) or (idx == robot_path.shape[0]-1):
            plt.gca().add_patch(plt.Rectangle(
                robot_path[idx,parameters.XY_POS] - parameters.robot_length/2,
                width=parameters.robot_length,
                height=parameters.robot_length,
                angle=robot_path[idx,2],
                ec='b',
                fill=False)) 
            last_x = x
            last_y = y

    draw_waypoints(mission)

    plt.xlim((0,parameters.module_size[0]))
    plt.ylim((0,parameters.module_size[1]))
    plt.tight_layout()
    # plt.show()



