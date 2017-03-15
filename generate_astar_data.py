import parameters, viz, astar_new as astar, time
import numpy as np, matplotlib.pyplot as plt, pandas as pd, time, itertools
from multiprocessing import Pool
from matplotlib.colors import Normalize
from datetime import datetime

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

n_mission = 1#3
n_crew = 1#4

dists_to_bin = [0, 0.15, 0.45, 1.20, 3.60]

idx = 0
astar_path_columns = ['x','y','theta','vx','vy','vtheta']
df_columns = ['type','mission','crew','time','max vel', 'average vel', 'distance', 'max accel', 'boundary collisions', 'crew collisions', 'min crew distance', '%% complete', 'time_to_compute', 'max_node_time'] + \
        ['time in distance bin %d from crew %d' % (dist_idx, crewnum+1) for crewnum in range(4) for dist_idx in range(len(dists_to_bin[1:]))]

def astar_generate(args):
    mission_idx, crew_idx, use_proxemic = args
    
    idx = 100*mission_idx + 10*crew_idx + int(use_proxemic)
    mission = parameters.select_mission(mission_idx)
    crew = parameters.select_crew(crew_idx)
    func_name = 'proxemic' if use_proxemic else 'nonproxemic'

    tic = time.time()
    nodelist, max_node_time = astar.astar(mission_idx,crew_idx,func_name=='proxemic')
    toc = time.time()
    path = viz.path_to_trajectory(nodelist) 

    viz.draw_path(path,mission,crew)
    plt.xlabel('x, m')
    plt.ylabel('y, m')
    plt.title('Robot Path on Mission %d with Crew %d, A* - %s' % (mission_idx, crew_idx, 'Non-Proxemic' if func_name=='nonproxemic' else 'Proxemic'))
    plt.savefig('data/astar_path__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
    plt.close()


    path_df = pd.DataFrame(data=path[:,:-1], columns=astar_path_columns, index=path[:,-1])
    

    path_df['vn'] = (path_df['vx']**2 + path_df['vy']**2)**0.5
    path_df['an'] = path_df['vn'].diff() #(path_df['vx'].diff()**2 + path_df['vy'].diff()**2)**0.5

    path_df[['vn','an']].plot()
    plt.tight_layout()
    plt.savefig('data/astar_velocity_accel__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
    plt.close()

    count_crew_collisions = 0 
    for crewnum,crewmember in enumerate(crew):
        path_df['distance from crew %d' % crewnum] = ((path_df['x']-crewmember[0])**2 + (path_df['y']-crewmember[1])**2)**0.5
        count_crew_collisions += ((path_df['distance from crew %d' % crewnum] <= parameters.collision_distance).astype(int).diff()>0).sum()

    count_complete_wp = 0
    for wp in mission:
        if (((path_df['x']-wp[0])**2 + (path_df['y']-wp[1])**2)**0.5).min().min() <= parameters.goal_cutoff*parameters.robot_length:
            count_complete_wp += 1

    percent_complete_wp = count_complete_wp/mission.shape[0]

    path_df[path_df.columns[path_df.columns.str.startswith('distance from crew ')]].plot()

    for line_y in dists_to_bin:
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

    df = [
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
            toc-tic, # time to compute
            max_node_time,
        ] + [ 
            np.diff(
                path_df.index[
                    (
                        (path_df['distance from crew %d' % (crewnum+1)] >= dists_to_bin[dist_idx]) & (path_df['distance from crew %d' % (crewnum+1)] < dist)
                    ).astype(int).diff().abs().astype(bool)
                ]
            )[1::2].sum() for crewnum,crewmember in enumerate(crew) for dist_idx,dist in enumerate(dists_to_bin[1:])
        ] + [0] * (4 - crew.shape[0])*(len(dists_to_bin)-1)
    path_df.to_csv('data/%d.csv' % idx)

    # out = apf.linear_goal_force_function(grid,mission[0])
    # out += apf.gaussian_boundary_force_function(grid, parameters.module_size)
    # out += funcs[func_name](grid, crew)

    # plt.figure()
    # plt.quiver(xx,yy,out[:,0],out[:,1])
    # plt.axis('scaled')
    # # viz.draw_crew(crew)
    # viz.draw_waypoints(mission[[0]])

    # plt.xlim((0,parameters.module_size[0]))
    # plt.ylim((0,parameters.module_size[1]))
    # plt.tight_layout()
    # plt.savefig('data/quiver_mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
    # plt.close()

    return (df, path_df)

if __name__ == '__main__':
    df = pd.DataFrame(columns=df_columns,)
    missions_to_iter = [1]#,2,3]
    crew_to_iter = [1]#,2,3,4]

    args = itertools.product(missions_to_iter,crew_to_iter,[True, False])
    with Pool(4) as p:
        # paths = [ p.apply(astar_generate, arg) for arg in args ]
        results = p.map(astar_generate, args)

    for idx, result in enumerate(results):
        df.loc[idx] = result[0]
    df.to_csv( datetime.now().__format__('summary_%Y_%m_%d__%H_%M_%S.csv'))