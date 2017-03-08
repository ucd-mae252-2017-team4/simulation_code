################
## ASTAR CODE ##
################
import numpy as np
import queue as * 
import simulator.py as simulator
from parameters.py import *


''' 
Starting with pseudocode, then expanding from there 
1) generate weighted graph 
2) A*
3) Output chosen path as an ordered list of (x,y) tuples. This should allow for appropriate analysis
'''
#################
##  VARIABLES  ##
## in SI units ##
#################
MODULE_WIDTH, MODULE_LENGTH = 4.3, 8.5 #meters; Destiny Module
ROBOT_WIDTH = 12.*0.0254 #12" in meters
ROBOT_MASS = 6. # kg

MESH_SIZE = 100


useProxemics = FALSE

##################################################################################
##  weighted graph -> 															##
##	     ndarray; vertex coordinates are the indices, stored values are weights ##
##################################################################################

weights = ndarray.ones(MODULE_WIDTH*MESH_SIZE,MODULE_LENGTH*MESH_SIZE) # this might break if we change mesh size, make sure they're integers

# add in humans
if !useProxemics:
	# crew = select_crew(crewID)
	for member in crew:
		# use nonproxemic_astar_function
else 
	# do the whole proxemics thing


weights = ndarray initialized with ones
- setup "infinite" weights for wall boundaries
for each human in  humans:
	update appropriate weights 
	''' (NOTE: WEIGHTS NEED DIRECTIONS. 
	Can do this by assigning positive/negative weights 
	always corresponding with left to right, bottom to top 
	OR by making weights tuples) '''

########
## A* ##
########
for each start-to-waypoint path: 
	from start point : add adjacent (admissible = no loops) vertices to minimum priority queue with appropriate v.pi
		'''(store each as list of tuples, add new vertex to end, 
		reference current tuple pair with list[-1] 
		-- assuming memory isn't an issue here, so it's okay if they get big'''
	with priority f(n) = g(n) + h(n) 
	''' priority = cost to get to node + heuristic (Manhattan distance) '''
	''' let's keep a dict of visited nodes '''
	update visited nodes dict

############
## return ##
############
once endpoint is pulled out of top of min-priority queue:
	check path for collisions with humans
	return list of tuples

