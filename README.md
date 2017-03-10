# simulation code

I'm imagining to collect the data for the actual report, we'll do something like this:

```python
robby = Robot(1,1,0) # set robt initial position + orientation

# make a list of human objects with positions and orientations
human1 = Human(3,4,np.pi/3)
human2 = Human(3,3,-np.pi/3)
case1humans = [human1, human2]

goal_pos = (4,3.5)

trial1path = apf_path_planner(robby,case1humans,human_force_function=proxemic_force_function)
trial2path = astar_path_planner(robby,case1humans,human_cost_function=proxemic_cost_function)

```

where ``trial1path`` and ``trial2path`` are arrays of dimension (T,6). We could then plot/draw the trajectories, determine whether there has been any collisions, etc.



```python
%load_ext autoreload
%autoreload 2

import apf, parameters, viz
# make crew member a little bit below so APF doesn't get stuck
mission = parameters.select_mission(1)
crew = parameters.select_crew(1)
crew[:,parameters.Y_POS] = 2.0
path = apf.apf_path_planner(
	parameters.robot_initial_condition,
	mission,
	crew,
	parameters.nonproxemic_apf_function)
viz.draw_path(path,mission,crew)

# robby gets stuck due to local minima
mission = parameters.select_mission(1)
crew = parameters.select_crew(1)
path = apf.apf_path_planner(
	parameters.robot_initial_condition,
	mission,
	crew,
	parameters.nonproxemic_apf_function)
viz.draw_path(path,mission,crew)



# Mission 3, crew 4  also gets stuck
mission = parameters.select_mission(3)
crew = parameters.select_crew(3)
path = apf.apf_path_planner(
	parameters.robot_initial_condition,
	mission,
	crew,
	parameters.nonproxemic_apf_function)
viz.draw_path(path,mission,crew)

import numpy as np, matplotlib.pyplot as plt
xx,yy = np.meshgrid(
	np.linspace(0,parameters.module_width),
	np.linspace(0,parameters.module_height)
)

zz = np.zeros_like(xx)
grid = np.stack((xx,yy,zz,zz,zz,zz), axis=1)

mission = parameters.select_mission(1)
crew = parameters.select_crew(1)

out = apf.linear_goal_force_function(grid,mission[0])
out += apf.gaussian_boundary_force_function(grid, parameters.module_size)
out += parameters.nonproxemic_apf_function(grid, crew)

plt.figure()
plt.quiver(xx,yy,out[:,0],out[:,1])
plt.axis('scaled')
viz.draw_crew(crew)
viz.draw_waypoints(mission)

plt.xlim((0,parameters.module_size[0]))
plt.ylim((0,parameters.module_size[1]))
plt.tight_layout()


```
# Grid computation
below is an example for grid computations, which may be useful for A*


```python
import apf, parameters, viz
import numpy as np, matplotlib.pyplot as plt

# xx and yy are distributed throughout the module
xx,yy = np.meshgrid(
	np.linspace(0,parameters.module_width,100),
	np.linspace(0,parameters.module_height,100)
)

# make place-holder grid for angles, velocities, etc
zz = np.zeros_like(xx)

# put the grids together
grid = np.stack((xx,yy,zz,zz,zz,zz), axis=1)

# select the crew pattern
crew = parameters.select_crew(4)

# compute the cost
r = parameters.determine_constants(grid,crew)
cost = parameters.proxemic_astar_function(grid, crew, r)

# plot contours
plt.figure()
plt.contour(xx,yy,cost,20)
plt.axis('scaled')
viz.draw_crew(crew)
plt.xlim((0,parameters.module_size[0]))
plt.ylim((0,parameters.module_size[1]))
plt.tight_layout()
```

Below was demo code for debugging

```python
crew = np.hstack(
	tuple(arr.reshape(-1,1) for arr in np.meshgrid(
		*tuple(
			np.hsplit(
				np.outer(np.array([[0.25,0.75]]),parameters.module_size),
			2)
		)
	)) + (np.arange(4).reshape(-1,1)*np.pi/2,)
)
r = parameters.determine_constants(grid,crew)
cost = parameters.proxemic_astar_function(grid,crew, r)
plt.figure()
plt.contour(xx,yy,cost,10)
plt.axis('scaled')
viz.draw_crew(crew)
plt.xlim((0,parameters.module_size[0]))
plt.ylim((0,parameters.module_size[1]))
plt.tight_layout()


# debugging proxemic apf
%load_ext autoreload
%autoreload 2

import apf, parameters, viz
import numpy as np, matplotlib.pyplot as plt

close_pos = np.array([[parameters.module_width*6/16,parameters.module_height/2,0,0,0,0]])
mission = parameters.select_mission(1)
crew = parameters.select_crew(1)
crew[:,parameters.Y_POS] = 2.0
path = apf.apf_path_planner(
	parameters.robot_initial_condition,# close_pos,
	mission,
	crew,
	parameters.proxemic_apf_function)
viz.draw_path(path,mission,crew)

```

