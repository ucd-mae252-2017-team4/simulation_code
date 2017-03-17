import parameters, viz, astar_new as astar, time
import numpy as np, matplotlib.pyplot as plt, pandas as pd, time, itertools
from multiprocessing import Pool
from matplotlib.colors import Normalize
from datetime import datetime
import os

# xx and yy are distributed throughout the module
xx,yy = np.meshgrid(
    np.linspace(0,parameters.module_width,51),
    np.linspace(0,parameters.module_height,51)
)

# make place-holder grid for angles, velocities, etc
zz = np.zeros_like(xx)

# put the grids together
grid = np.stack((xx,yy,zz,zz,zz,zz), axis=1)

funcs = {'proxemic': parameters.proxemic_apf_function,
 'nonproxemic': parameters.nonproxemic_apf_function }

n_mission = 3
n_crew = 4

# n_mission = 1
# n_crew = 1

dists_to_bin = [0, 0.15, 0.45, 1.20, 3.60, 100]

idx = 0
astar_path_columns = ['x','y','theta','vx','vy','vtheta']
df_columns = ['type','mission','crew','time','max vel', 'average vel', 'distance', 'max accel', 'boundary collisions', 'crew collisions', 'min crew distance', '%% complete', 'time_to_compute', 'max_node_time'] + \
        ['time in distance bin %d from crew %d' % (dist_idx, crewnum+1) for crewnum in range(4) for dist_idx in range(len(dists_to_bin[1:]))]


path_df_dir = 'astar_data_03_15_1154'

# xx and yy are distributed throughout the module
xx,yy = np.meshgrid(
    np.linspace(0,parameters.module_width,100),
    np.linspace(0,parameters.module_height,100)
)

# make place-holder grid for angles, velocities, etc
zz = np.zeros_like(xx)

# put the grids together
grid = np.stack((xx,yy,zz,zz,zz,zz), axis=1)

def astar_analyze(do_path=True, do_distance=True, do_speed=False,do_contours=False,save_summary=True):
    df = pd.DataFrame(columns=df_columns,)
    for mission_idx in range(1,n_mission+1): #4
        # mission_idx = 3
        mission = parameters.select_mission(mission_idx)

        for crew_idx in range(1,n_crew+1): #5
            # crew_idx = 3
            if (crew_idx == 5):
                if (mission_idx==1):
                    crew = parameters.select_crew(1)
                    crew[:,parameters.Y_POS] = 2.0
                else:
                    continue
            else:
                crew = parameters.select_crew(crew_idx)

            for func_name in funcs:
                idx = 100*mission_idx + 10*crew_idx + int(func_name =='proxemic')

                use_proxemic = func_name =='proxemic'

                path_df = pd.DataFrame.from_csv(os.path.join(path_df_dir, str(idx)+ '.csv'))

                path = path_df[astar_path_columns].values
                
                # func_name = 'proxemic' if use_proxemic else 'nonproxemic'

                tic = time.time()
                # nodelist, max_node_time = astar.astar(mission_idx,crew_idx,func_name=='proxemic')
                toc = time.time()
                # path = viz.path_to_trajectory(nodelist)
                max_node_time = 0
                if do_path:
                    viz.draw_path(path,mission,crew,True)
                    plt.xlabel('x, m')
                    plt.ylabel('y, m')
                    plt.title('Robot Path on Mission %d with Crew %d, A* - %s' % (mission_idx, crew_idx, 'Non-Proxemic' if func_name=='nonproxemic' else 'Proxemic'))
                    plt.savefig('data/astar_path__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
                    plt.close()


                # path_df = pd.DataFrame(data=path[:,:-1], columns=astar_path_columns, index=path[:,-1])
                

                path_df['vn'] = (path_df['vx']**2 + path_df['vy']**2)**0.5
                path_df['an'] = path_df['vn'].diff() #(path_df['vx'].diff()**2 + path_df['vy'].diff()**2)**0.5

                if do_speed:
                    path_df[['vn','an']].plot()
                    plt.tight_layout()
                    plt.savefig('data/astar_velocity_accel__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
                    plt.close()

                count_crew_collisions = 0 
                for crewnum,crewmember in enumerate(crew):
                    path_df['distance from crew %d' % (crewnum+1)] = ((path_df['x']-crewmember[0])**2 + (path_df['y']-crewmember[1])**2)**0.5
                    count_crew_collisions += ((path_df['distance from crew %d' % (crewnum+1)] <= parameters.collision_distance).astype(int).diff()>0).sum()

                count_complete_wp = 0
                for wp in mission:
                    if (((path_df['x']-wp[0])**2 + (path_df['y']-wp[1])**2)**0.5).min().min() <= parameters.goal_cutoff*parameters.robot_length:
                        count_complete_wp += 1

                percent_complete_wp = count_complete_wp/mission.shape[0]

                if do_distance:
                    path_df[path_df.columns[path_df.columns.str.startswith('distance from crew ')]].plot()

                    for line_y in dists_to_bin[1:-1]:
                        plt.plot(path_df.index[[0,-1]],[line_y]*2,'--')

                    plt.xlabel('t, sec')
                    plt.ylabel('distance, m')
                    plt.title('Distance Between Robot and Crew\n for Mission %d, Crew Configuration %d, A* - %s' % (crew_idx, mission_idx, 'Non-Proxemic' if func_name=='nonproxemic' else 'Proxemic'))

                    plt.tight_layout()
                    plt.savefig('data/astar_crew_dist__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
                    plt.close()

                count_boundary_collisions = ((
                    (path_df['x'] <= parameters.robot_length/2) |
                    (path_df['x'] >= parameters.module_width - parameters.robot_length/2) |
                    (path_df['y'] <= parameters.robot_length/2) |
                    (path_df['y'] >= parameters.module_height - parameters.robot_length/2)
                    ).astype(int).diff()>0).sum()

                # time to complete
                # avg vel
                # distance travelled
                # collisions
                # entered waypoint zone %
                # max thrust, max velocity

                row_data = [
                        func_name, # proxemic or not
                        mission_idx,
                        crew_idx,
                        path_df.index[-1], # time
                        path_df['vn'].max(), # max vel
                        path_df['vn'].mean(), # average velocity
                        (path_df['vn']*path_df.index).sum(), # path length 
                        path_df['an'].max(), # maximum accel?
                        count_boundary_collisions, # boundary collisions
                        count_crew_collisions, # crew colissions
                        path_df[path_df.columns[path_df.columns.str.startswith('distance from crew ')]].min().min(), # min crew distance
                        percent_complete_wp,
                        0, #toc-tic, # time to compute
                        max_node_time,
                    ]

                for crewnum,crewmember in enumerate(crew):
                    check_time = 0
                    for dist_idx,dist in enumerate(dists_to_bin[1:]):
                        in_bin_selector = (
                                (path_df['distance from crew %d' % (crewnum+1)] >= dists_to_bin[dist_idx]) & \
                                (path_df['distance from crew %d' % (crewnum+1)] < dist)
                            ).astype(int).diff().abs().astype(bool)

                        for end_pt in [0,-1]:
                            in_bin_selector.iloc[end_pt] = ((path_df['distance from crew %d' % (crewnum+1)].iloc[end_pt] >= dists_to_bin[dist_idx]) and
                                    (path_df['distance from crew %d' % (crewnum+1)].iloc[end_pt] < dist))

                        if (mission_idx, crew_idx, crewnum+1, use_proxemic) in [(1,2,2, True), (2,2,1, True), (2,3,2, True), (3,3,3, False), (3,4,1, True)]:
                            print("\n\n CHECKING ERRORS BIN:",dist,"\n",(
                                (path_df['distance from crew %d' % (crewnum+1)] >= dists_to_bin[dist_idx]) & \
                                (path_df['distance from crew %d' % (crewnum+1)] < dist)
                            ),)

                        row_data += [np.diff(path_df.index[in_bin_selector])[::2].sum()]
                        check_time += row_data[-1]

                    print("mission", mission_idx, "crew cfg",crew_idx, "model", func_name, "crew #", crewnum+1, "time check",path_df.index[-1],check_time, "good" if path_df.index[-1] == check_time else "ERROR! ERROR! ERROR!")

                    # if check_time != path_df.index[-1]:
                    #     return
                
                row_data += [0] * (4 - crew.shape[0])*(len(dists_to_bin)-1)
                df.loc[idx] = row_data

                if do_contours:
                    if use_proxemic:
                        r = parameters.determine_constants(grid,crew)
                        cost = parameters.proxemic_astar_function(grid, crew, r)
                    else:
                        cost = parameters.nonproxemic_astar_function(grid, crew)

                    # plot contours
                    plt.figure()
                    plt.contour(xx,yy,cost,20)
                    plt.axis('scaled')
                    viz.draw_crew(crew)
                    plt.xlim((0,parameters.module_size[0]))
                    plt.ylim((0,parameters.module_size[1]))
                    plt.tight_layout()

                    plt.savefig('data/crew_contour_crew%d_%s.png' % (crew_idx,func_name))
                    plt.close()






    if save_summary:
        df.to_csv( datetime.now().__format__('data/astar_summary_%Y_%m_%d__%H_%M_%S.csv'))

if __name__ == '__main__':
    # astar_analyze(False,False,False, True,save_summary=False)

    ## plot grid
    # xlist = np.arange(-parameters.robot_x0-astar.dx,parameters.module_width-parameters.robot_x0+astar.dx,astar.dx)+parameters.robot_x0
    # ylist = np.arange(-parameters.robot_y0-astar.dx,parameters.module_height--parameters.robot_x0-astar.dx,astar.dy)+parameters.robot_y0

    xlist = np.arange(-parameters.robot_x0+astar.dx/2,parameters.module_width,astar.dx)+parameters.robot_x0
    ylist = np.arange(-parameters.robot_y0,parameters.module_height,astar.dy)+parameters.robot_y0

    print(np.diff(xlist))

    xx,yy = np.meshgrid(xlist,ylist)

    plt.figure()
    plt.scatter(xx,yy,color='g', s=5)
    plt.scatter([parameters.robot_x0], [parameters.robot_y0], color='b')
    plt.axis('scaled')
    plt.gca().add_patch(plt.Rectangle(
                parameters.robot_initial_condition[0,parameters.XY_POS] - parameters.robot_length/2,
                width=parameters.robot_length,
                height=parameters.robot_length,
                angle=0,
                ec='b',
                fill=False)) 

    plt.xlim((0,parameters.module_size[0]))
    plt.ylim((0,parameters.module_size[1]))
    plt.tight_layout()

    plt.savefig('astar_analyzegrid.png')
    plt.close()





