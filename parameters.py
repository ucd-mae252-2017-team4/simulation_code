def select_mission(mission_id): #Defines the waypoints (aka goal(s)) based on the mission ID
#All mission coordinates are given in feet
#All mission coordinates are placed 1 ft from the edges
#Mission coordinates do not contain a starting point, which should be in an initialization step
#(x,y) where x is along the length of the module (28 ft) and y is across the width or diameter (14 ft)
#(0,0) is located in the bottom left corner
	mission1 = [(27,7)] #Mission 1 is a direct task from one end of the module to the other
	mission2 = [(7,1),(14,0),(24,1),(27,7)] #Mission 2 is the waypoints (wp) alternating sides of the module; 
	#wp1 was choosen halfway to the center
	#wp2 is at the center top
	#wp3 is over halfway from the center to end (sharper turn)
	#wp4 is the same as mission1
	mission3 = [(1,1),(27,1),(27,13),(1,13),(0,14)] #Mission 3 circulations through the module on a monitoring task and returns to the start point
#Select the desired mission coordinates
	if mission_id = 1:
		wp = mission1
	else if mission_id = 2:
		wp = mission2
	else if mission_id = 3:
		wp = mission3
	else:
		fprint('Not a valid mission ID! (Valid codes: 1 = direct, 2 = waypoints, 3 = monitoring)')
	end

	return wp

def select_crew(crew_id): #Defines the location and orientation of crew 
#All crew coordinates are given for the center of the profile
#(x,y,theta) where (x,y) are defined same as mission coordinates and theta (deg) is 0 in the direction of the module end and parallel to module midline
	crew1 = [(14,7,180)] #One crew in module center
	crew2 = [(14,11,270),(14,3,90)] #Two crew in module center facing each other
	crew3 = [(11,11,315),(14,11,225),(17,11,204),(12.5,3,90)] #Cluster of three crew and one across module
	crew4 = [(12,12,102),(21,9,44),(22,11,39),(6,9,359)] #Four crew randomly generated from excel
	#Add check for crew4 to make sure crew not on top of each other
#Select the desired crew coordinates
	if crew_id = 1:
		cp = crew1
	else if crew_id = 2:
		cp = crew2
	else if crew_id = 3:
		cp = crew3
	else if crew_id = 4:
		cp = crew4
	else:
		fprint('Not a valid crew ID! (Valid codes: 1 = one crew, 2 = opposing, 3 = cluster, 4 = random)')
	end

	return cp

def proxemic_function(): #Mathematical function to define the potential field and cost 

def nonproxemic_function(): #Use collision avoidance for humans; no proxemics

def crew(): #Defines the size and shape of crew
#Assume profile to be a circle for simplicity
#Diameter of profile will be based on average shoulder width for minimum crew size