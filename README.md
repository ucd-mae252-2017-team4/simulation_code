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
import simulator, apf, parameters, viz

# robby gets stuck due to local minima
robby = simulator.Robot(12*2.54E-2,7*12*2.54E-2,0) # set robt initial position + orientation
mission = parameters.select_mission(1)
crew = parameters.select_crew(1)
apf.apf_path_planner(robby, mission, crew, parameters.nonproxemic_apf_function)
viz.draw_path(robby.state_trajectory,mission,crew)

# make crew member a little bit below so APF doesn't get stuck
robby = simulator.Robot(12*2.54E-2,7*12*2.54E-2,0) # set robt initial position + orientation
mission = parameters.select_mission(1)
crew = [(4.2672,2.0,3.14159)]
apf.apf_path_planner(robby, mission, crew, parameters.nonproxemic_apf_function)
viz.draw_path(robby.state_trajectory,mission,crew)

# does this also fix it? nope
Y_POS = 1.5
robby = simulator.Robot(12*2.54E-2,Y_POS,0) # set robt initial position + orientation
mission = [(parameters.select_mission(1)[0][1], Y_POS)]
crew = [(4.2672,Y_POS,3.14159)]
apf.apf_path_planner(robby, mission, crew, parameters.nonproxemic_apf_function)
viz.draw_path(robby.state_trajectory,mission,crew)

# Mission 3, crew 4  also gets stuck
robby = simulator.Robot(12*2.54E-2,7*12*2.54E-2,0) # set robt initial position + orientation
mission = parameters.select_mission(3)
crew = parameters.select_crew(4)
apf.apf_path_planner(robby, mission, crew, parameters.nonproxemic_apf_function)
viz.draw_path(robby.state_trajectory,mission,crew)


```

