import numpy as np
import math

def select_mission(mission_id): #Defines the waypoints (aka goal(s)) based on the mission ID
#All mission coordinates are given in meters
#All mission coordinates are placed 1 ft from the edges
#Mission coordinates do not contain a starting point, which should be in an initialization step
#(x,y) where x is along the length of the module (28 ft) and y is across the width or diameter (14 ft)
#(0,0) is located in the bottom left corner
	mission1 = [(8.2296,2.1336)] #Mission 1 is a direct task from one end of the module to the other (27,7)
	mission2 = [(2.1336,0.3048),(4.2672,0),(7.3152,0.3048),(8.2296,2.1336)] #Mission 2 is the waypoints (wp) alternating sides of the module; 
	#wp1 was choosen halfway to the center (7,1)
	#wp2 is at the center top (14,0)
	#wp3 is over halfway from the center to end (sharper turn) (24,1)
	#wp4 is the same as mission1 (27,7)
	mission3 = [(0.3048,0.3048),(8.2296,0.3048),(8.2296,3.9624),(0.3048,3.9624),(0,4.2672)] 
	#Mission 3 circulations through the module on a monitoring task and returns to the start point (1,1),(27,1),(27,13),(1,13),(0,14)
#Select the desired mission coordinates
	if mission_id == 1:
		wp = mission1
	elif mission_id == 2:
		wp = mission2
	elif mission_id == 3:
		wp = mission3
	else:
		print('Not a valid mission ID! (Valid codes: 1 = direct, 2 = waypoints, 3 = monitoring)')

	return wp

def select_crew(crew_id): #Defines the location and orientation of crew 
#All crew coordinates are given for the center of the profile in meters
#(x,y,theta) where (x,y) are defined same as mission coordinates and theta (deg) is 0 in the direction of the module end and parallel to module midline
	crew1 = [(4.2672,2.1336,3.14159)] #One crew in module center (14,7,180)
	crew2 = [(4.2672,3.3528,4.71239),(4.2672,0.9144,1.5708)] #Two crew in module center facing each other (14,11,270),(14,3,90)
	crew3 = [(3.3528,3.3528,5.49779),(4.2672,3.3528,3.92699),(5.1816,3.3528,3.56047),(3.8100,0.9144,1.5708)] #Cluster of three crew and one across module (11,11,315),(14,11,225),(17,11,204),(12.5,3,90)
	crew4 = [(3.6576,3.6576,1.78024),(6.4008,2.7432,0.767945),(6.7056,3.3528,0.680678),(1.8288,2.7432,6.26573)] #Four crew randomly generated from excel (12,12,102),(21,9,44),(22,11,39),(6,9,359)
#Check for crew4 to make sure crew not on top of each other
	for i in range(len(crew4)):
		for j in range(i+1, len(crew4)):
			d = ((crew4[j][0]-crew4[i][0])**2+(crew4[j][1]-crew4[i][1])**2)**(1/2) #Distance between crew centers
			print(crew4[i])
			print(crew4[j])
			print(d)
			if d <= crew_radius*2:
				print("Crew members overlapping; generate new random positions")
#Select the desired crew coordinates
	if crew_id == 1:
		cp = crew1
	elif crew_id == 2:
		cp = crew2
	elif crew_id == 3:
		cp = crew3
	elif crew_id == 4:
		cp = crew4
	else:
		print('Not a valid crew ID! (Valid codes: 1 = one crew, 2 = opposing, 3 = cluster, 4 = random)')

	return cp

def proxemic_function(): #Mathematical function to define the potential field and cost 
	pass

def nonproxemic_apf_function(robot, cp): #Use collision avoidance for humans; no proxemics; used for APF only
	dfdx = 0
	dfdy = 0
	sigma = (14/12)*0.3048 #Use crew shoulder width (meters); equal in x and y
	A = 1/(2*math.pi*sigma**2)

	for x,y,theta in cp:
		dfdx += -(robot.x - x)*(A/sigma**2)*exp(-((robot.x-x)**2+(robot.y-y)**2)/(2*sigma**2))
		dfdy += -(robot.y - y)*(A/sigma**2)*exp(-((robot.x-x)**2+(robot.y-y)**2)/(2*sigma**2))

	gradient = np.array([dfdx, dfdy, 0])
	
	return gradient

def nonproxemic_astar_function(robot, cp): #Use collision avoidance for humans; no proxemics; used for A* only
	cost = 0
	sigma = (14/12)*0.3048 #Use crew shoulder width (meters); equal in x and y
	A = 1/(2*math.pi*sigma**2)

	for x,y,theta in cp:
		cost += A*exp(-((robot.x-x)**2+(robot.y-y)**2)/(2*sigma**2))
	
	return cost

crew_radius = 0.3048*(14/12)/2 #Determines the space taken up by crew profile
#Shoulder width given in inches and converted to meters; based on average of shoulder width for 95% male and 5% female,
#which fall within height requirements for astronauts; average relationship between height and shoulder width
#Average used because difference was 1-2 inches and there didn't seem to be a strong reason to select max or min
#Assume profile to be a circle for simplicity