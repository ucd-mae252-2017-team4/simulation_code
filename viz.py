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
        if np.isclose(ds,0) or np.isclose(v,0):
            trajectory[node_idx, 6] = trajectory[node_idx-1, 6]
        else:
            trajectory[node_idx, 6] = trajectory[node_idx-1, 6] + ds/v



    return trajectory


def draw_crew(crew, labels=False):
    for cidx,(cx,cy,ct) in enumerate(crew): #crew x, crew y, crew theta
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
        if labels:
            plt.text(
                cx-2*parameters.crew_radius, 
                cy+ 1.25*parameters.crew_radius,
                'Crew %d' % (cidx+1), multialignment='center'
            )

def draw_waypoints(mission, labels=False):
    for wi,wp in enumerate(mission):
        plt.gca().add_patch(
            plt.Circle(
                wp,
                radius=parameters.robot_length*parameters.goal_cutoff,
                ec='m',fill=False, lw=3
            ))
        if labels:
            if wi < 4:
                plt.text(
                    wp[0]-0.0625, 
                    wp[1]-0.0625,
                    '%d' % (wi+1), multialignment='center', fontsize=16,
                )
            else:
                plt.text(
                    wp[0]+0.0625+0.75*parameters.robot_length*parameters.goal_cutoff, 
                    wp[1]-0.0625-1*parameters.robot_length*parameters.goal_cutoff,
                    '%d' % (wi+1), multialignment='center', fontsize=16,
                )

velocity_bins = [2**-3, 2**-2, 2**-1]
vel_color = 'ryg'

path_colors = {'proxemic':'b','non-proxemic':'g'}
# np array columns for robot_path: x,y,theta
def draw_path(robot_paths, mission, crew, with_vel = False, key=['proxemic','non-proxemic']):
    plt.figure()
    if not isinstance(robot_paths,list):
        robot_paths = [robot_paths]
        do_legends = False
    else: 
        do_legends = True

    for path_idx,robot_path in enumerate(robot_paths):
        plt.plot(robot_path[:,0],robot_path[:,1])

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
                    ec=path_colors[key[path_idx]],
                    fill=False)) 
                last_x = x
                last_y = y
    

        if with_vel:
            robot_v_norm = (robot_path[:,3]**2+robot_path[:,4]**2)**0.5
            for vidx,vel in enumerate(velocity_bins):
                selector = (robot_v_norm >= vel)
                plt.scatter(robot_path[selector,0],robot_path[selector,1], color=vel_color[vidx])

    if False: #do_legends:
        plt.legend(key)

    plt.axis('scaled')
    draw_crew(crew)
    draw_waypoints(mission)

    plt.xlim((0,parameters.module_size[0]))
    plt.ylim((0,parameters.module_size[1]))
    plt.tight_layout()
    # plt.show()

if __name__ == '__main__':
    for crew_idx in range(1,5):
        crew = parameters.select_crew(crew_idx)
        plt.figure()
        plt.axis('scaled')
        plt.gca().add_patch(plt.Rectangle(
            parameters.robot_initial_condition[0,parameters.XY_POS] - parameters.robot_length/2,
            width=parameters.robot_length,
            height=parameters.robot_length,
            angle=0,
            ec='b',
            fill=False)) 
        draw_crew(crew, True)
        plt.xlim((0,parameters.module_size[0]))
        plt.ylim((0,parameters.module_size[1]))

        # plt.xlabel('x, m')
        # plt.ylabel('y, m')
        plt.title('Crew Positions for Crew Configuration %d' % crew_idx)
        plt.tight_layout()
        plt.savefig('crew%d' % crew_idx,bbox_inches='tight')
        plt.close()

    for mission_idx in range(1,4):
        mission = parameters.select_mission(mission_idx)
        plt.figure()
        plt.axis('scaled')
        plt.gca().add_patch(plt.Rectangle(
            parameters.robot_initial_condition[0,parameters.XY_POS] - parameters.robot_length/2,
            width=parameters.robot_length,
            height=parameters.robot_length,
            angle=0,
            ec='b',
            fill=False)) 
        draw_waypoints(mission, True)
        plt.xlim((0,parameters.module_size[0]))
        plt.ylim((0,parameters.module_size[1]))
        plt.tick_params(
            axis='both',          # changes apply to the x-axis
            which='both',      # both major and minor ticks are affected
            bottom='off',      # ticks along the bottom edge are off
            top='off',         # ticks along the top edge are off
            left='off',
            right='off',
            labelleft='off',
            labelbottom='off') # labels along the bottom edge are off
        # plt.xlabel('x, m')
        # plt.ylabel('y, m')
        # plt.title('Waypoint Positions for Mission %d' % mission_idx)
        plt.tight_layout()
        plt.savefig('mission%d' % mission_idx,bbox_inches='tight')
        plt.close()





