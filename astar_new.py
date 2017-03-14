####################
## ASTAR_NEW CODE ##
####################

''' 
nodes = (cost,visitedNodes,x,y,v)
cost = float/double
visitedNodes = list of tuples (x,y)
x = float/double
y = float/double
v = float/double


single_path = [nodes]
paths = priority queue of single_path(s)
	they will be placed in queue as (priority,single_path)
	and returned from queue as (priority,single_path)

'''




import numpy as np
import queue as q
import parameters as p
import viz as viz
import time

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
ROBOT_RADIUS = np.sqrt(0.5*(ROBOT_WIDTH**2))
a = 1 #distance weight
b = 0.05 #weight of 1/velocity

# each path gets added to the queue as (priority,[nodes]), where nodes are (cost,visitedNodes,x,y,v)
# and priority is cost+distance

## node indices ##
COST_I = 0 # this is old cost plus new cost
VISITED_I = 1
X_I = 2
Y_I = 3
V_I = 4






# these all need figuring out based on max thrust and step size
GRID_SIZE = 50 #size of mesh

dy = MODULE_LENGTH/GRID_SIZE
dx = dy
#dv = 1 #max that it can change based on physical robot properties
#velocities that get changed are arange(v-dv:vstep:v+dv)
V_MAX = 0.5
numVelocities = 5 #number of velocities to test

def distance(start,end): #both (x,y) tuples
	return np.sqrt( (end[0]-start[0])**2 + (end[1]-start[1])**2 )

def getCost(myRobot,crew,useProxemics):
	
	#MYROBOT INPUT: myRobot = (newx,newy,vel,x,y,v)
	myRobot = np.array(myRobot).reshape((1,-1))
	r = p.determine_astar_constants(myRobot,crew)

	robotPos = (myRobot[0][0],myRobot[0][1])
	#
	'''outOfRange = False
	for i in range(len(crew)):
		crewCenter = (crew[i][0],crew[i][1])
		sigma = r[i][3]#3=sigma
		if distance(robotPos,crewCenter) > 3*sigma:
			outOfRange = True
		else: 
			outOfRange = False
		
	if outOfRange:
		return 0'''

	if useProxemics:
		cost =  p.proxemic_astar_function(myRobot,crew,r)
		return cost[0]
	else: 
		cost = p.nonproxemic_astar_function(myRobot,crew)
		return cost[0]


def insideBoundaries(x,y):
	#This checks that we haven't run into a wall
	if ROBOT_RADIUS < x < MODULE_WIDTH - ROBOT_RADIUS:
		if ROBOT_RADIUS < y < MODULE_LENGTH - ROBOT_RADIUS:
			return True
	return False

def noCollisons(x,y,crew):
	if not insideBoundaries(x,y):
		return False
	for i in range(len(crew)):
		crewCenter = (crew[i][0],crew[i][1])
		if distance((x,y),crewCenter) < ROBOT_RADIUS + p.crew_radius:
			return False
	return True


def getVels(v,dx,dy):
	a = p.robot_max_thrust/ROBOT_MASS
	s = np.sqrt(dx**2 + dy**2)

	vMax = min(V_MAX,np.sqrt(v**2 + 2*a*s)) 
	vMin = 0
	if v**2 > 2*a*s:
		vMin = np.sqrt(v**2 - 2*a*s)

	vRange = np.linspace(vMin,vMax,numVelocities)
	if v not in vRange:
		vI = int(np.floor(numVelocities/2))
		vRange = np.insert(vRange,vI,v)

	return vRange



'''
Called by astarPath()
Takes in a path in the form of a list of nodes, identifies the last
one as the end of the path,  and spits back a list of the neighboring 
nodes with uncumulative costs
(dc,visitedNodes,newx,newy,vel) '''
def getAdjacentNodes(currPath,crew,useProxemics):
	#takes in a list of nodes and spits back a list of nodes with uncumulative costs
	#returned nodes are (dc,visitedNodes,newx,newy,vel)
	nextStepList = []

	#get latest values
	lastNode = currPath[-1]
	c = lastNode[COST_I]
	visitedNodes = lastNode[VISITED_I]
	x = lastNode[X_I]
	y = lastNode[Y_I]
	v = lastNode[V_I]

	#define possible velocities robot could physically change to in this timestep
	#vels = np.arange(max(v-dv,0),max(v+dv,vmax),vstep) 

	# check that neighboring nodes are valid (inside the module, haven't already been visited in this path)
	xs = [x-dx,x,x+dx]
	ys = [y-dy,y,y+dy]


	for newx in xs:
		for newy in ys:
			if (newx,newy) not in visitedNodes:
				if noCollisons(newx,newy,crew):
					possibleVels = getVels(v,newx-x,newy-y)
					for vel in possibleVels:
						robot = (newx,newy,vel,x,y,v)
						dc = getCost(robot,crew,useProxemics) 
						## if outside 3*sigma for all possible crew members:
						'''if dc == 0:
							vel = max(possibleVels)
							nextStepList.append((dc,visitedNodes,newx,newy,vel))
							break'''
						if vel > 0:
							nextStepList.append((dc,visitedNodes,newx,newy,vel))
	if len(nextStepList) == 0:
		print("Oh no! You've got nowhere to go!")
	
	return nextStepList






## should return a data structure of the form single_path = [nodes]
def astarPath(goal,paths,crew,useProxemics):

	while not paths.empty():

		currPath = paths.get()[-1] #Take and remove from queue
		#currPath is now a list of nodes
		currX = currPath[-1][X_I] # x value of last node in path
		currY = currPath[-1][Y_I] # y value of last node in path
		currPoint = (currX,currY) # location of current end of path

		if np.linalg.norm(currPoint-goal) <= dy*1.2:
			break# return currPath #currPoint and goal should both be (x,y) tuples
		elif np.linalg.norm(currPoint-goal) < ROBOT_RADIUS + dy:
			break

		currCost = currPath[-1][COST_I] #cost from last visited node (costs stored in nodes are cumulative)
		
		for node in getAdjacentNodes(currPath,crew,useProxemics):
			#that means it takes in a list of nodes and spits back a list of nodes with uncumulative costs
			#node is (dc,visitedNodes,newx,newy,vel)

			#update path cost
			newPath = currPath.copy() #resets newPath to original list of nodes from currPath
			newCost = currCost+node[COST_I]

			#define priority
			nX = node[X_I]
			nY = node[Y_I]
			nV = node[V_I]
			priority = newCost + a*distance((nX,nY),goal) + b/nV

			#add current node to list of visited nodes
			newVisitedNodes = node[VISITED_I].copy() 
			newVisitedNodes.append((nX,nY)) 

			#add node to path, add path to q with defined priority
			newPath.append((newCost,newVisitedNodes,nX,nY,node[V_I]))
			paths.put((priority,newPath))
		
	return currPath




#mission, crew are indices for configuration specified in parameters.py
#proxemics should be a boolean TRUE/FALSE
def astar(mission,crew,proxemics): 
	cp = p.select_crew(crew)
	waypoints = p.select_mission(mission)

	x0 = p.robot_x0
	y0 = p.robot_y0
	v0 = 0
	visitedNodes = [(x0,y0)] #list of tuples
	cost0 = 0
	
	node0 = (cost0,visitedNodes,x0,y0,v0)

	goal_i = 0
	fullPath = []

	for goal in waypoints:
		paths = q.PriorityQueue()
		if goal_i > 0: 
			#print ("goal index =", goal_i)
			#start again at the last node from the last segment
			lastNode = bestPath[-1]
			#print ("lastNode =", (lastNode[X_I],lastNode[Y_I]))
			lastX = lastNode[X_I]
			lastY = lastNode[Y_I]
			newVisitedNodes = [(lastX,lastY)]
			lastVel = lastNode[V_I]
			node0 = (0,newVisitedNodes,lastX,lastY,lastVel)		
		
		path0 = [node0] #should now be a list of nodes
		# each path gets added to the queue as (priority,[nodes]), where nodes are (cost,visitedNodes,x,y,v)
		# if not paths.empty():
		# 	print("Path not empty!")
		paths.put((0,path0))
		#paths.get() should return list of tuples
		useProxemics = proxemics 

		#astarPath(goal,paths,crew,useProxemics)
		bestPath = astarPath(goal,paths,cp,useProxemics) 
		#bestPath should be a single_path = [nodes]
		goal_i += 1
		fullPath.extend(bestPath) 
		#fullpath should be an array of single_paths
	return fullPath
		

def draw_astar(mission,crew,proxemics):
	tic = time.time()
	path = astar(mission,crew,proxemics)
	cp = p.select_crew(crew)
	waypoints = p.select_mission(mission)
	drawablePath = viz.path_to_trajectory(path)
	#print(drawablePath)

	viz.draw_path(drawablePath,waypoints,cp)
	toc = time.time()

	tElapsed = toc - tic
	tmins = np.floor(tElapsed/60)
	tsecs = np.mod(tElapsed,60)

	print ("Took", toc - tic, "seconds")
	print ("or", tmins, "mins and", tsecs, "seconds")









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
