import apf, parameters, viz, astar_new as astar
import numpy as np, matplotlib.pyplot as plt, pandas as pd
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

func_names_list = ['proxemic','nonproxemic']
funcs = {'proxemic': parameters.proxemic_apf_function,
 'nonproxemic': parameters.nonproxemic_apf_function }

n_mission = 3
n_crew = 4

idx = 0
path_columns = ['x','y','theta','vx','vy','vtheta', 'fx','fy', 'tth']

use_k = 1.0
use_b = 5.0

dists_to_bin = [0, 0.15, 0.45, 1.20, 3.60, 100]

def do_generate(do_path=True, do_distance=True, do_speed=False, do_paired_velocity=False):
    df_columns = ['type','mission','crew','time','max vel', 'average vel', 'distance', 'max accel', 'boundary collisions', 'crew collisions', 'min crew distance', '%% complete'] + \
        ['time in distance bin %d from crew %d' % (dist_idx, crewnum+1) for crewnum in range(4) for dist_idx in range(len(dists_to_bin[1:]))]
    idx = 0
    df = pd.DataFrame(columns=df_columns,)
    for mission_idx in range(1,n_mission+1): #4
        # mission_idx = 1
        mission = parameters.select_mission(mission_idx)

        for crew_idx in range(1,n_crew+1): #5
            # crew_idx = 1
            if (crew_idx == 5):
                if (mission_idx==1):
                    crew = parameters.select_crew(1)
                    crew[:,parameters.Y_POS] = 2.0
                else:
                    continue
            else:
                crew = parameters.select_crew(crew_idx)

            paths_df_list = []
            for func_name in func_names_list:
                path = apf.apf_path_planner(
                    parameters.robot_initial_condition,
                    mission,
                    crew,
                    funcs[func_name],
                    goal_k = use_k, damping_b=use_b)
                if do_path:
                    viz.draw_path(path,mission,crew, True)
                    plt.xlabel('x, m')
                    plt.ylabel('y, m')
                    plt.title('Robot Path on Mission %d with Crew %d, APF - %s' % (mission_idx, crew_idx, 'Non-Proxemic' if func_name=='nonproxemic' else 'Proxemic'))
                    # plt.gcf().set_size_inches(3.5,3)
                    plt.tight_layout()
                    plt.savefig('data/path_mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
                    plt.close()

                path_df = pd.DataFrame(data=path, columns=path_columns)
                path_df.index = path_df.index*apf.dt

                path_df['vn'] = (path_df['vx']**2 + path_df['vy']**2)**0.5
                path_df['an'] = (path_df['fx']**2 + path_df['fy']**2)**0.5/parameters.robot_mass

                if do_speed:
                    path_df[['vn','an']].plot()
                    plt.tight_layout()
                    plt.savefig('data/velocity_force__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
                    plt.close()

                count_crew_collisions = 0 
                for crewnum,crewmember in enumerate(crew):
                    path_df['distance from crew %d' % (crewnum+1)] = ((path_df['x']-crewmember[0])**2 + (path_df['y']-crewmember[1])**2)**0.5

                    for dist_idx,dist in enumerate(dists_to_bin[1:]):

                        path_df['time in distance bin %d from crew %d' % (dist_idx, crewnum+1)] = np.diff(path_df.index[((path_df['distance from crew %d' % (crewnum+1)] >= dists_to_bin[dist_idx]) & \
                            (path_df['distance from crew %d' % (crewnum+1)] < dist)).astype(int).diff().abs().astype(bool)])[1::2].sum()



                    count_crew_collisions += ((path_df['distance from crew %d' % (crewnum+1)] <= parameters.collision_distance).astype(int).diff()>0).sum()

                count_complete_wp = 0
                for wp in mission:
                    if (((path_df['x']-wp[0])**2 + (path_df['y']-wp[1])**2)**0.5).min().min() <= parameters.goal_cutoff*parameters.robot_length:
                        count_complete_wp += 1

                percent_complete_wp = count_complete_wp/mission.shape[0]

                if do_distance:
                    path_df[path_df.columns[path_df.columns.str.startswith('distance from crew ')]].plot()
                    # plt.legend(bbox_to_anchor=(0, -0.02, 1, -0.002),legends=['crew 1'])

                    for line_y in dists_to_bin[1:-1]:
                        plt.plot(path_df.index[[0,-1]],[line_y]*2,'--')

                    
                    plt.xlabel('t, sec')
                    plt.ylabel('distance, m')
                    plt.title('Distance Between Robot and Crew\n for Mission %d, Crew Configuration %d, APF - %s' % (crew_idx, mission_idx, 'Non-Proxemic' if func_name=='nonproxemic' else 'Proxemic'))
                    # plt.gcf().set_size_inches(3.5,3)
                    plt.tight_layout()
                    plt.savefig('data/apf_crew_distance__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
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
                        path_df['vn'].sum()*apf.dt, # path length 
                        path_df['an'].max(), # maximum accel?
                        count_boundary_collisions, # boundary collisions
                        count_crew_collisions, # crew colissions
                        path_df[path_df.columns[path_df.columns.str.startswith('distance from crew ')]].min().min(), # min crew distance
                        percent_complete_wp
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
                        row_data += [np.diff(path_df.index[in_bin_selector])[::2].sum()]
                        check_time += row_data[-1]

                    print("check",path_df.index[-1],check_time, "good" if path_df.index[-1] == check_time else "ERROR! ERROR! ERROR!")
                
                row_data += [0] * (4 - crew.shape[0])*(len(dists_to_bin)-1)
                df.loc[idx] = row_data

                paths_df_list.append(path_df)

                
                idx += 1

                # out = apf.linear_goal_force_function(grid,mission[0])
                # out += apf.gaussian_boundary_force_function(grid, parameters.module_size)
                # out += funcs[func_name](grid, crew)

                # norm = (out[:,0]**2 + out[:,1]**2)**0.5
                # norm = norm/norm.max().max()

                # plt.figure()
                # plt.quiver(xx,yy,out[:,0],out[:,1],norm)
                # plt.axis('scaled')
                # # viz.draw_crew(crew)
                # viz.draw_waypoints(mission[[0]])

                # plt.xlim((0,parameters.module_size[0]))
                # plt.ylim((0,parameters.module_size[1]))

                # plt.xlabel('x, m')
                # plt.ylabel('y, m')
                # plt.title('Potential Field for Goal %d Mission %d with Crew %d APF - %s' % (mission_idx, crew_idx, 'Non-Proxemic' if func_name=='nonproxemic' else 'Proxemic'))

                # plt.tight_layout()
                # plt.savefig('data/quiver_mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
                # plt.close()

            colors = 'bg'
            if do_paired_velocity:
                new_df = pd.DataFrame()
                new_df['proxemic'] = paths_df_list[0]['vn']
                new_df['non-proxemic'] = paths_df_list[1]['vn']

                plt.figure()
                plt.plot(new_df['proxemic'], 'b')
                plt.plot(new_df['non-proxemic'], 'g')

                new_df['proxemic goals'] = 0
                new_df['nonproxemic goals'] = 0
                for wp in mission:
                    for func_idx,func_name in enumerate(func_names_list):
                        path_df = paths_df_list[func_idx]
                        selector = (((((path_df['x']-wp[0])**2 + (path_df['y']-wp[1])**2)**0.5) <= parameters.goal_cutoff*parameters.robot_length).astype(int)).diff().astype(bool)
                        for time_idx in path_df.index[selector]:
                            plt.plot([time_idx]*2,[0, path_df['vn'].max()], colors[func_idx]+'--')

                # new_df.plot()
                plt.tight_layout()
                plt.savefig('data/apfpaired_v_plot__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name),bbox_inches='tight')
                plt.close()

                viz.draw_path([paths_df_list[0].values,paths_df_list[1].values], mission, crew)
                plt.tight_layout()
                plt.savefig('data/apfpaired_path__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name),bbox_inches='tight')
                plt.close()



    return df


def compare_damp_amp():
    df_columns = ['type','k_exp','b_val','time','max vel', 'average vel', 'distance', 'max accel', 'boundary collisions', 'crew collisions', 'min crew distance', '%% complete']
    idx = 0
    df = pd.DataFrame(columns=df_columns,)
    mission = parameters.select_mission(1)
    crew = parameters.select_crew(1)
    func_name = 'nonproxemic'
    for k_exp in range(-4,1):
        for b_val in [10, 5, 1, 0]:
            path = apf.apf_path_planner(
                parameters.robot_initial_condition,
                mission,
                crew,
                funcs[func_name], goal_k = 2**k_exp, damping_b=b_val)
            viz.draw_path(path,mission,crew, True)

            path_df = pd.DataFrame(data=path, columns=path_columns)
            path_df.index = path_df.index*apf.dt

            plt.savefig('data/apf_path__amp%d_damp%d.png' % (k_exp,b_val))
            plt.close()

            path_df['vn'] = (path_df['vx']**2 + path_df['vy']**2)**0.5
            path_df['fn'] = (path_df['fx']**2 + path_df['fy']**2)**0.5/parameters.robot_mass

            path_df[['vn','fn']].plot()
            plt.tight_layout()
            plt.savefig('data/velocity_force__amp%d_damp%d.png' % (k_exp,b_val))
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
            plt.tight_layout()
            plt.savefig('data/crew_distance__amp%d_damp%d.png' % (k_exp,b_val))
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

            df.loc[idx] = [
                    func_name, # proxemic or not
                    k_exp,
                    b_val,
                    path_df.index[-1], # time
                    path_df['vn'].max(), # max vel
                    path_df['vn'].mean(), # average velocity
                    path_df['vn'].sum()*apf.dt, # path length 
                    path_df['fn'].max()*apf.dt/parameters.robot_mass, # maximum accel?
                    count_boundary_collisions, # boundary collisions
                    count_crew_collisions, # crew colissions
                    path_df[path_df.columns[path_df.columns.str.startswith('distance from crew ')]].min().min(), # min crew distance
                    percent_complete_wp
                ]

            idx += 1
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
            # plt.savefig('data/quiver__amp%d_damp%d.png' % (k_exp,b_val))
            # plt.close()

    return df

def color_quiver_plt():
    for mission_idx,crew_idx,wp_idx in [(1,1,0)]: # (2,2,1),(2,2,2),(2,3,1),(3,4,3)]:
        mission = parameters.select_mission(mission_idx)
        crew = parameters.select_crew(crew_idx)
        for func_name in funcs:
            out = apf.linear_goal_force_function(grid,mission[wp_idx], use_k)
            out += apf.gaussian_boundary_force_function(grid, parameters.module_size)
            out += funcs[func_name](grid, crew)

            norm = (out[:,0]**2 + out[:,1]**2)**0.5
            norm = norm/norm.max().max()

            plt.figure()
            plt.quiver(xx,yy,out[:,0],out[:,1],norm,cmap=plt.cm.viridis)
            # plt.colorbar()
            plt.colorbar(fraction=0.046, pad=0.04)
            # norm = Normalize()
            # norm.autoscale()
            plt.axis('scaled')
            # viz.draw_crew(crew)
            viz.draw_waypoints(mission[[wp_idx]])

            plt.xlim((0,parameters.module_size[0]))
            plt.ylim((0,parameters.module_size[1]))

            plt.xlabel('x, m')
            plt.ylabel('y, m')
            plt.title('Potential Field for Goal %d Mission %d with Crew %d - %s' % ((wp_idx+1), mission_idx, crew_idx, 'Non-Proxemic' if func_name=='nonproxemic' else 'Proxemic'))
            # plt.gcf().set_size_inches(3.5,3)
            plt.tight_layout()
            # plt.savefig('data/quiver_goal%d_mission%d_crew%d_%s.png' % ((wp_idx+1),mission_idx,crew_idx,func_name))
            # plt.close()


if __name__ == '__main__':
    df = do_generate(False,False,False,do_paired_velocity=True)
    # color_quiver_plt()
    # df.to_csv( datetime.now().__format__('data/apf_summary_%Y_%m_%d__%H_%M_%S.csv'))