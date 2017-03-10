################
## ASTAR CODE ##
################
import numpy as np
import queue as q
import simulator.py as simulator
from parameters.py import *


''' 
Starting with pseudocode, then expanding from there 
1) generate weighted graph 
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




##################################################################################
##  weighted graph -> 															##
##	     ndarray; vertex coordinates are the indices, stored values are weights ##
##################################################################################

"""weights = ndarray.ones(MODULE_WIDTH*MESH_SIZE,MODULE_LENGTH*MESH_SIZE) # this might break if we change mesh size, make sure they're integers

# add in humans
if !useProxemics:
	# crew = select_crew(crewID)
	for member in crew:
		# use nonproxemic_astar_function
else 
	# do the whole proxemics thing """


'''initialize '''
startpoint = (robot_x0,robot_y0)
#velocities = np.arange(something)
startVel = 0
position = startpoint

paths = q.PriorityQueue()



paths.put(0,(0,robot_x0,robot_y0,startVel))

waypoints = select_mission(1)
endpoint = waypoints[-1] #endpoint is last waypoint

useProxemics = FALSE 

dx = 0
dy = 0
dv = 0 #max that 
vstep = 0.1



def getCost(robot,vel):
	# convert to robot form
	# if useProxemics: 
		# r = determine_constants(robot, cp)
		# return proxemic_astar_function(robot,cp,r)
	# else: return nonproxemic_astar_function(robot,cp)



''' we should change this function if we want to use theta; actually, that might make my life a little easier, 
even if this function will run hellishly slowly without maintaining a world map '''
def getAdjacentNodes(currPath):
	nextStepList = ()
	lastNode = currPath[-1]
	c = lastNode[COST_I]
	visitedNodes = lastNode[VISITED_I]
	x = lastNode[X_I]
	y = lastNode[Y_I]
	v = lastNode[V_I]
	possibleVels = np.arange(v-dv:vstep:v+dv) #possible velocities robot could physically change to in this timestep
	xs = [x-dx,x,x+dx]
	ys = [y-dy,y,y+dy]
	possibleNodes = []
	for vel in possibleVels: #go through possible next velocities and add if we haven't already visited that location
		for newx in x:
			for newy in ys:
				if !visitedNodes.has_key(newx,newy):
					robot = 
					dc = getCost(robot,vel)
					nextStepList.append(dc,newx,newy,vel)
	if len(nextStepList) == 0:
		print("Oh no! You've got nowhere to go!")
	return nextStepList

def astarPath(startpoint,goal):
	currPath = paths.get()
	if startpoint == goal: return currPath

	currCost = currPath[0]
	for node in getAdjacentNodes(currPath):
		newPath = currPath
		newCost = currCost+node[0]
		currPath.append(newCost,node[1],node[2],node[3])
		paths.put(newCost,currPath)
		newStartpoint = (node[1],node[2]) #(x,y)
		return astarPath(newStartpoint,goal)


for goal in waypoints:
	#do the search from startpoint until curr == goal
	bestPath = astarPath(startpoint,goal)
	


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
for each start-to-waypoint path: 
	from start point : add adjacent (admissible = no loops) vertices to minimum priority queue with appropriate v.pi
		'''(store each as list of tuples, add new vertex to end, 
		reference current tuple pair with list[-1] 
		-- assuming memory isn't an issue here, so it's okay if they get big'''
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

