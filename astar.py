################
## ASTAR CODE ##
################
import numpy as np
import queue as q
import simulator.py as simulator
from parameters.py import *

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
MODULE_WIDTH, MODULE_LENGTH = 4.3, 8.5 #meters; Destiny Module
ROBOT_WIDTH = 12.*0.0254 #12" in meters
ROBOT_MASS = 6. # kg

MESH_SIZE = 100
possibleVelocities = np.arange(0) ## decide this

# each path gets added to the queue as (cost,[nodes]), where nodes are (cost,visitedNodes,x,y,v)
## node indices ##
COST_I = 0
VISITED_I = 1
X_I = 2
Y_I = 3
V_I = 4



'''initialize '''
#velocities = np.arange(something)
startVel = 0

# these all need figuring out based on max thrust and step size
dx = 0
dy = 0
dv = 0 #max that it can change based on physical robot properties
#velocities that get changed are arange(v-dv:vstep:v+dv)
vstep = 0.1

def distance(start,end): #both (x,y) tuples
	return np.sqrt((start[0]-end[0])^2+(start[1]-end[1])^2)

def getCost(robot,vel):
	# 1) convert to robot form
	# 2) if useProxemics: 
	   #	 r = determine_constants(robot, cp)
	   #	 return proxemic_astar_function(robot,cp,r)
	   # else: return nonproxemic_astar_function(robot,cp)

def insideBoundaries(x,y):
	#This will eventually check that we haven't run into a wall
	return TRUE


''' we should change this function if we want to use theta; actually, that might make my life a little easier, 
even if this function will run hellishly slowly without maintaining a world map. We could maintain a world map if 
we wanted to, I suppose, it'd just take some careful indexing '''
'''takes in a node(position&velocity) and returns all of the next options (no revisiting a node in the same path)
so no infinite loops'''
def getAdjacentNodes(currPath):
	nextStepList = ()
	lastNode = currPath[-1]
	c = lastNode[COST_I]
	visitedNodes = lastNode[VISITED_I]
	x = lastNode[X_I]
	y = lastNode[Y_I]
	v = lastNode[V_I]
	possibleVels = np.arange(v-dv:vstep:v+dv) #possible velocities robot could physically change to in this timestep
	#if x,y aren't leading into a boundary
	xs = [x-dx,x,x+dx]
	ys = [y-dy,y,y+dy]
	possibleNodes = []
	for vel in possibleVels: #go through possible next velocities and add if we haven't already visited that location
		for newx in x:
			for newy in ys:
				if !visitedNodes.has_key(newx,newy):
					if insideBoundaries(newx,newy):
						robot = (newx,newy,vel,x,y,v)### sort this!
						dc = getCost(robot,vel) + distance(newx,newy,x,y)
						nextStepList.append(dc,newx,newy,vel)
	if len(nextStepList) == 0:
		print("Oh no! You've got nowhere to go!")
	return nextStepList

## recursively returns the lowest cost path, base case is when you're starting at the goal
def astarPath(startpoint,goal,paths):

	#Base case 1 - ran out of paths despite not reaching end
	if paths.empty():
		print("You couldn't reach your goal, sorry! Not sure why.")
		return NULL

	#Base case 2 (desired) - you've reached your goal
	currPath = paths.get() #Take and remove from queue
	if startpoint == goal: return currPath #Startpoint and goal should both be (x,y) tuples

	currCost = currPath[COST_I]
	for node in getAdjacentNodes(currPath):
		newPath = currPath
		newCost = currCost+node[COST_I]
		newVisitedNodes = node[VISITED_I].append(node)
		currPath.append(newCost,newVisitedNodes,node[X_I],node[Y_I],node[V_I])
		paths.put(newCost,currPath)
		newStartpoint = (node[X_I],node[Y_I]) #(x,y)
		return astarPath(newStartpoint,goal,paths)

#mission, crew are indices for configuration specified in parameters.py
#proxemics should be a boolean TRUE/FALSE
def astar(mission,crew,proxemics): 

	cp = select_crew(crew)
	waypoints = select_mission(mission)
	#endpoint = waypoints[-1] #endpoint is last waypoint
	goal_i = 0
	goal0 = waypoint[0]
	cost0 = distance(startpoint,goal0)
	startpoint = (robot_x0,robot_y0)
	paths = q.PriorityQueue()
	paths.put(,(0,robot_x0,robot_y0,startVel))
	useProxemics = proxemics 
	fullPath = []
		for goal in waypoints:
			bestPath = astarPath(startpoint,goal,paths,crew_=cp,mission_=mission)
			startpoint = waypoints[goal_i]
			goal_i += 1
			fullPath.append(bestPath)

	return fullPath #warning this isn't the right format for viz
		








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
