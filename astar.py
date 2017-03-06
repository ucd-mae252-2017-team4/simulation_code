################
## ASTAR CODE ##
################
import numpy as np
import queue as * 
import simulator.py as simulator


''' 
Starting with pseudocode, then expanding from there 
1) generate weighted graph 
2) A*
3) Output chosen path as an ordered list of (x,y) tuples. This should allow for appropriate analysis
'''

##################################################################################
##  weighted graph -> 															##
##	     ndarray; vertex coordinates are the indices, stored values are weights ##
##################################################################################
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
