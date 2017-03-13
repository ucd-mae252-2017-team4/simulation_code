import apf, parameters, viz
import numpy as np, matplotlib.pyplot as plt, pandas as pd
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

n_mission = 3
n_crew = 4

df_columns = ['type','mission','crew','time','max vel', 'average vel', 'distance', 'max accel', 'boundary collisions', 'crew collisions', 'min crew distance']
idx = 0
path_columns = ['x','y','theta','vx','vy','vtheta', 'fx','fy', 'tth']

def do_generate():
    idx = 0
    df = pd.DataFrame(columns=df_columns,)
    for mission_idx in range(1,n_mission+1): #4
        mission = parameters.select_mission(mission_idx)

        for crew_idx in range(1,n_crew+1): #5
            if (crew_idx == 5):
                if (mission_idx==1):
                    crew = parameters.select_crew(1)
                    crew[:,parameters.Y_POS] = 2.0
                else:
                    continue
            else:
                crew = parameters.select_crew(crew_idx)

            for func_name in funcs:
                path = apf.apf_path_planner(
                    parameters.robot_initial_condition,
                    mission,
                    crew,
                    funcs[func_name])
                viz.draw_path(path,mission,crew)

                path_df = pd.DataFrame(data=path, columns=path_columns)
                path_df.index = path_df.index*apf.dt

                plt.savefig('path_mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
                plt.close()

                path_df['vn'] = (path_df['vx']**2 + path_df['vy']**2)**0.5
                path_df['fn'] = (path_df['fx']**2 + path_df['fy']**2)**0.5/parameters.robot_mass

                path_df[['vn','fn']].plot()
                plt.tight_layout()
                plt.savefig('velocity_force__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
                plt.close()

                count_crew_collisions = 0 
                for crewnum,crewmember in enumerate(crew):
                    path_df['distance from crew %d' % crewnum] = ((path_df['x']-crewmember[0])**2 + (path_df['y']-crewmember[1])**2)**0.5
                    count_crew_collisions += ((path_df['distance from crew %d' % crewnum] <= parameters.collision_distance).astype(int).diff()>0).sum()

                path_df[path_df.columns[path_df.columns.str.startswith('distance from crew ')]].plot()
                plt.tight_layout()
                plt.savefig('crew_distance__mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
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
                        mission_idx,
                        crew_idx,
                        path_df.index[-1], # time
                        path_df['vn'].max(), # max vel
                        path_df['vn'].mean(), # average velocity
                        path_df['vn'].sum()*apf.dt, # path length 
                        path_df['fn'].max()*apf.dt/parameters.robot_mass, # maximum accel?
                        count_boundary_collisions, # boundary collisions
                        count_crew_collisions, # crew colissions
                        path_df[path_df.columns[path_df.columns.str.startswith('distance from crew ')]].min().min(), # min crew distance
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
                # plt.savefig('quiver_mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
                # plt.close()

    return df

if __name__ == '__main__':
    df = do_generate()
    df.to_csv( datetime.now().__format__('summary_%Y_%m_%d__%H_%M_%S.csv'))