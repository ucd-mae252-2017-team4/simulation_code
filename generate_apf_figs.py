import apf, parameters, viz
import numpy as np, matplotlib.pyplot as plt


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

for mission_idx in range(1,4): #4
    mission = parameters.select_mission(mission_idx)

    for crew_idx in range(1,6): #5
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

            plt.savefig('path_mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
            plt.close()

            out = apf.linear_goal_force_function(grid,mission[0])
            out += apf.gaussian_boundary_force_function(grid, parameters.module_size)
            out += funcs[func_name](grid, crew)

            plt.figure()
            plt.quiver(xx,yy,out[:,0],out[:,1])
            plt.axis('scaled')
            # viz.draw_crew(crew)
            viz.draw_waypoints(mission)

            plt.xlim((0,parameters.module_size[0]))
            plt.ylim((0,parameters.module_size[1]))
            plt.tight_layout()
            plt.savefig('quiver_mission%d_crew%d_%s.png' % (mission_idx,crew_idx,func_name))
            plt.close()
