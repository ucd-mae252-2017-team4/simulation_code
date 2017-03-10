# simulation code

Here are some chucnks of example code. 

First, run APF for a modified crew position.

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
```

Generate a quiver plot

```python
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
below is a complete example for grid computations, which may be useful for A*


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

Put a crew member in each quadrant facing in the cardinal directions, plot the cost contour

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

```

