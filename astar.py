################
## ASTAR CODE ##
################
import numpy as np
import queue as q
import parameters as p
import viz as viz

#NO ACTUAL WORLDMAP IN CODE - IS THIS A PROBLEM?#

''' 
Starting with pseudocode, then expanding from there 
1) constants
2) A*
3) Output chosen path 
'''

#################
##  CONSTANTS  ##
#################
MODULE_WIDTH, MODULE_LENGTH = p.module_width, p.module_height #meters; Destiny Module
ROBOT_WIDTH = p.robot_length #12" in meters
ROBOT_MASS = p.robot_mass

# each path gets added to the queue as (priority,[nodes]), where nodes are (cost,visitedNodes,x,y,v)
# and priority is cost+distance

## node indices ##
COST_I = 0 # this is old cost plus new cost
VISITED_I = 1
X_I = 2
Y_I = 3
V_I = 4






# these all need figuring out based on max thrust and step size
GRID_SIZE = 10 #size of mesh
dx = MODULE_WIDTH/GRID_SIZE
dy = MODULE_LENGTH/GRID_SIZE
dv = 0.5 #max that it can change based on physical robot properties
#velocities that get changed are arange(v-dv:vstep:v+dv)
vmax = 5.0
vstep = 0.1

def distance(start,end): #both (x,y) tuples
	return np.sqrt((start[0]-end[0])**2+(start[1]-end[1])**2)

def getCost(myRobot,crew,useProxemics):
	# 1) convert to robot form
	#MYROBOT INPUT: myRobot = (newx,newy,vel,x,y,v)
	myRobot = np.array(myRobot).reshape((1,-1))
	r = p.determine_astar_constants(myRobot,crew)

	if useProxemics:
		cost =  p.proxemic_astar_function(myRobot,crew,r)
		return cost[0]
	else: 
		cost = p.nonproxemic_astar_function(myRobot,crew)
		return cost[0]


def insideBoundaries(x,y):
	#This checks that we haven't run into a wall
	if x <= MODULE_WIDTH - ROBOT_WIDTH/2:
		if y <= MODULE_LENGTH - ROBOT_WIDTH/2:
			return True
	return False


''' we should change this function if we want to use theta; actually, that might make my life a little easier, 
even if this function will run hellishly slowly without maintaining a world map. We could maintain a world map if 
we wanted to, I suppose, it'd just take some careful indexing '''
'''takes in a node(position&velocity) and returns all of the next options (no revisiting a node in the same path)
so no infinite loops'''
def getAdjacentNodes(currPath,crew,useProxemics):
	nextStepList = []
	lastNode = currPath[-1]
	c = lastNode[COST_I]
	visitedNodes = lastNode[VISITED_I]
	#print(visitedNodes)
	x = lastNode[X_I]
	y = lastNode[Y_I]
	v = lastNode[V_I]
	vels = np.arange(max(v-dv,0),max(v+dv,vmax),vstep) #possible velocities robot could physically change to in this timestep
	#if x,y aren't leading into a boundary
	xs = [x-dx,x,x+dx]
	ys = [y-dy,y,y+dy]
	possibleNodes = []
	for vel in vels: #go through possible next velocities and add if we haven't already visited that location
		for newx in xs:
			for newy in ys:
				if (newx,newy) not in visitedNodes:
					if insideBoundaries(newx,newy):
						robot = (newx,newy,vel,x,y,v)### sort this!
						dc = getCost(robot,crew,useProxemics) 
						nextStepList.append((dc,visitedNodes,newx,newy,vel))
	if len(nextStepList) == 0:
		print("Oh no! You've got nowhere to go!")
	#print(nextStepList)
	return nextStepList

## recursively returns the lowest cost path, base case is when you're starting at the goal
def astarPath(startpoint,goal,paths,crew,useProxemics):

	while not paths.empty():

		currPath = paths.get()[-1] #Take and remove from queue
		#currPath is now a list of nodes

		if np.linalg.norm(startpoint-goal) <= dy*5:
			break# return currPath #Startpoint and goal should both be (x,y) tuples

		currCost = currPath[-1][COST_I] #cost from last visited node
		newPath = currPath.copy()
		for node in getAdjacentNodes(currPath,crew,useProxemics):
			#node is (dc,visitedNodes,newx,newy,vel)

			newCost = currCost+node[COST_I]

			#define priority
			nX = node[X_I]
			nY = node[Y_I]
			priority = newCost + distance((nX,nY),goal)

			#add current node to list of visited nodes
			newVisitedNodes = node[VISITED_I].copy()
			newVisitedNodes.append((nX,nY))

			newPath.append((newCost,newVisitedNodes,nX,nY,node[V_I]))
			paths.put((priority,newPath))
			startpoint = (nX,nY) 
		'''return astarPath(newStartpoint,goal,paths,crew,useProxemics)'''
	return currPath

#mission, crew are indices for configuration specified in parameters.py
#proxemics should be a boolean TRUE/FALSE
def astar(mission,crew,proxemics): 
	cp = p.select_crew(crew)
	waypoints = p.select_mission(mission)
	#endpoint = waypoints[-1] #endpoint is last waypoint
	goal_i = 0
	goal0 = waypoints[0]
	x0 = p.robot_x0
	y0 = p.robot_y0
	startVel = 0
	startpoint = np.array((x0,y0))
	visitedNodes = [(x0,y0)] #list of tuples
	cost0 = 0
	paths = q.PriorityQueue()
	# each path gets added to the queue as (priority,[nodes]), where nodes are (cost,visitedNodes,x,y,v)
	paths.put((0,[(cost0,visitedNodes,x0,y0,startVel)]))
	#paths.get() should return list of tuples
	useProxemics = proxemics 
	fullPath = []
	for goal in waypoints:
		bestPath = astarPath(startpoint,goal,paths,cp,useProxemics)
		startpoint = waypoints[goal_i]
		goal_i += 1
		fullPath.append(bestPath)

	return fullPath #warning this isn't the right format for viz
		

def draw_astar(mission,crew,proxemics):
	path = astar(mission,crew,proxemics)
	cp = p.select_crew(crew)
	waypoints = p.select_mission(mission)
	drawablePath = viz.path_to_trajectory(path)

	viz.draw_path(drawablePath,cp,waypoints)









##    #  #####  #####  #####  #####
# #   #  #   #    #    #      #
#  #  #  #   #    #    ###    #####
#   # #  #   #    #    #          #
#    ##  #####    #    #####  #####

"""

IS THE BEST THING TO DO TO HAVE N POSSIBLE VELOCITIES, 
AND SEARCH THE PATHS FOR EACH ONE???

Alternatives include: 
- only slow down if you've hit a heuristic boundary
- Say, "at this contour, I will be travelling at this speed with this cost, 
and back-update your previous speed costs"

"""

########
## A* ##
########
"""for each start-to-waypoint path: 
	from start point : add adjacent (admissible = no loops) vertices to minimum priority queue with appropriate v.pi

	with priority f(n) = g(n) + h(n) 
	''' priority = cost to get to node + heuristic (sqrt(x^2+y^2)) (distance fx is what was used for COMPANION) '''
	''' let's keep a dict of visited nodes '''
	update visited nodes dict

############
## return ##
############
once endpoint is pulled out of top of min-priority queue:
	check path for collisions with humans
	return list of tuples
"""
