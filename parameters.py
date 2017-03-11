import numpy as np
import math

module_width = 28*12*2.54E-2
module_height = 14*12*2.54E-2
module_size = np.array([module_width,module_height])
crew_radius = 0.3048*(14/12)/2 #Determines the space taken up by crew profile
#Shoulder width given in inches and converted to meters; based on average of shoulder width for 95% male and 5% female,
#which fall within height requirements for astronauts; average relationship between height and shoulder width
#Average used because difference was 1-2 inches and there didn't seem to be a strong reason to select max or min
#Assume profile to be a circle for simplicity

robot_length = 12*2.54E-2
robot_mass = 24.5
robot_izz = robot_mass*2*(robot_length**2)/12

robot_inertia_vec = np.array([robot_mass]*2 + [robot_izz])

robot_x0 = 12*2.54E-2 #starting point?
robot_y0 = 7*12*2.54E-2

X_POS = 0
Y_POS = 1
THETA_POS = 2
DX_POS = 3
DY_POS = 4
DTHETA_POS = 5

XY_POS = [X_POS, Y_POS]
POS_POS = [X_POS, Y_POS, THETA_POS]
VEL_POS = [DX_POS, DY_POS, DTHETA_POS]



robot_initial_condition = np.zeros((1,6))
robot_initial_condition[0,X_POS] = robot_x0
robot_initial_condition[0,Y_POS] = robot_y0
robot_initial_condition[0,THETA_POS] = 0
robot_initial_condition[0,DX_POS] = 0
robot_initial_condition[0,DY_POS] = 0
robot_initial_condition[0,DTHETA_POS] = 0

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
		return [(0,0)]
	return np.asarray(wp)

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
			# print(crew4[i])
			# print(crew4[j])
			# print(d)
			if d <= crew_radius*2:
				print("Crew members overlapping; generate new random positions")
				return NULL
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
		return NULL 
	return np.asarray(cp)

def determine_constants(robot, cp):
	rvx = robot[:,DX_POS]
	rvy = robot[:,DY_POS]

	v = (rvx**2 + rvy**2)**(1/2) #Relative velocity between robot and human (stationary)
	sigma_h = 2*v.copy() #From thesis appendix
	sigma_h[sigma_h < 0.5] = 0.5
	sigma_s = (2/3)*sigma_h #From thesis appendix
	sigma_r = (1/2)*sigma_h #From thesis appendix

	rx = robot[:,X_POS]
	ry = robot[:,Y_POS]


	r = []
 	#Determine where the robot is located in relation to front, side, or back of crew
	for i in range(len(cp)):
		x = cp[i][0]  
		y = cp[i][1]
		theta = cp[i][2]

		alpha = np.arctan2((ry-y),(rx-x))
		alpha[alpha < 0 ] += 2*np.pi
		
		sigma = sigma_s.copy()

		s1 = theta + np.pi/2

		if (theta <= math.pi/2):
			s2 = s1+np.pi
			selector = np.all(((s1<alpha),(alpha<s2)),axis=0) 
			sigma[selector] = sigma_r[selector]

			selector = np.any(((s1>alpha),(alpha>s2)),axis=0)
			sigma[selector] = sigma_h[selector]

			# selector = np.any((  np.isclose(s1,alpha),np.isclose(alpha,s2) ),axis=0)
			# sigma[selector] = sigma_s[selector]

		elif  (theta > 3*math.pi/2):
			s1 = theta+np.pi/2 - 2*np.pi
			s2 = s1 + np.pi
			selector = np.all(((s1<alpha),(alpha<s2)),axis=0) 
			sigma[selector] = sigma_r[selector]

			selector = np.any(((s1>alpha),(alpha>s2)),axis=0)
			sigma[selector] = sigma_h[selector]

			# selector = np.any((  np.isclose(s1,alpha),np.isclose(alpha,s2) ),axis=0)
			# sigma[selector] = sigma_s[selector]

		else:
			s2 = s1-np.pi
			selector = np.all(((s2<alpha),(alpha<s1)),axis=0) 
			sigma[selector] = sigma_h[selector]

			selector = np.any(((s2>alpha),(alpha>s1)),axis=0)
			sigma[selector] = sigma_r[selector]

			# selector = np.any((  np.isclose(s1,alpha),np.isclose(alpha,s2) ),axis=0)
			# sigma[selector] = sigma_s[selector]
		
		a = (np.cos(theta)**2)/(2*sigma**2)+(np.sin(theta)**2)/(2*sigma_s**2)
		b = (np.sin(2*theta))/(4*sigma**2)-(np.sin(2*theta))/(4*sigma_s**2)
		c =(np.sin(theta)**2)/(2*sigma**2)+(np.cos(theta)**2)/(2*sigma_s**2)

		ans = (a,b,c)
		r.append(ans)

	return r

def determine_astar_constants(robot, cp):
	# myRobot = np.array((newx,newy,vel,x,y,v))
	# pythagorean thm

	
	v = robot[[0],2] #Relative velocity between robot and human (stationary)
	sigma_h = 2*v.copy() #From thesis appendix
	sigma_h[sigma_h < 0.5] = 0.5
	sigma_s = (2/3)*sigma_h #From thesis appendix
	sigma_r = (1/2)*sigma_h #From thesis appendix

	rx = robot[[0],0]
	ry = robot[[0],1]


	r = []
 	#Determine where the robot is located in relation to front, side, or back of crew
	for i in range(len(cp)):
		x = cp[i][0]  
		y = cp[i][1]
		theta = cp[i][2]

		alpha = np.arctan2((ry-y),(rx-x))
		alpha[alpha < 0 ] += 2*np.pi
		
		sigma = sigma_s.copy()

		s1 = theta + np.pi/2

		if (theta <= math.pi/2):
			s2 = s1+np.pi
			selector = np.all(((s1<alpha),(alpha<s2)),axis=0) 
			sigma[selector] = sigma_r[selector]

			selector = np.any(((s1>alpha),(alpha>s2)),axis=0)
			sigma[selector] = sigma_h[selector]

			# selector = np.any((  np.isclose(s1,alpha),np.isclose(alpha,s2) ),axis=0)
			# sigma[selector] = sigma_s[selector]

		elif  (theta > 3*math.pi/2):
			s1 = theta+np.pi/2 - 2*np.pi
			s2 = s1 + np.pi
			selector = np.all(((s1<alpha),(alpha<s2)),axis=0) 
			sigma[selector] = sigma_r[selector]

			selector = np.any(((s1>alpha),(alpha>s2)),axis=0)
			sigma[selector] = sigma_h[selector]

			# selector = np.any((  np.isclose(s1,alpha),np.isclose(alpha,s2) ),axis=0)
			# sigma[selector] = sigma_s[selector]

		else:
			s2 = s1-np.pi
			selector = np.all(((s2<alpha),(alpha<s1)),axis=0) 
			sigma[selector] = sigma_h[selector]

			selector = np.any(((s2>alpha),(alpha>s1)),axis=0)
			sigma[selector] = sigma_r[selector]

			# selector = np.any((  np.isclose(s1,alpha),np.isclose(alpha,s2) ),axis=0)
			# sigma[selector] = sigma_s[selector]
		
		a = (np.cos(theta)**2)/(2*sigma**2)+(np.sin(theta)**2)/(2*sigma_s**2)
		b = (np.sin(2*theta))/(4*sigma**2)-(np.sin(2*theta))/(4*sigma_s**2)
		c =(np.sin(theta)**2)/(2*sigma**2)+(np.cos(theta)**2)/(2*sigma_s**2)

		ans = (a,b,c)
		r.append(ans)

	return r

def proxemic_apf_function(robot, cp):
	
	A = 1

	r = determine_constants(robot,cp)

	rx = robot[:,X_POS]
	ry = robot[:,Y_POS]

	dfdx = np.zeros_like(rx)
	dfdy = np.zeros_like(ry)

	for i in range(len(cp)):
		x = cp[i][0]  
		y = cp[i][1]
		a = r[i][0]
		b = r[i][1]
		c = r[i][2]

		dfdx += A*(2*a*(rx-x)+2*b*(ry-y))*np.exp(-(a*(rx-x)**2+2*b*(rx-x)*(ry-y)+c*(ry-y)**2))
		dfdy += A*(2*b*(rx-x)+2*c*(ry-y))*np.exp(-(a*(rx-x)**2+2*b*(rx-x)*(ry-y)+c*(ry-y)**2))
	
	gradient = np.stack((dfdx, dfdy, np.zeros_like(rx)),axis=1) #np.array([dfdx, dfdy, 0])
	
	return gradient

def proxemic_astar_function(robot, cp, r): #Mathematical function to define the potential field and cost 
	A = 1

	rx = robot[:,X_POS]
	ry = robot[:,Y_POS]
	cost = np.zeros_like(rx)
	
	for i in range(len(cp)):
		x = cp[i][0]  
		y = cp[i][1]
		a = r[i][0]
		b = r[i][1]
		c = r[i][2]

		cost += A*np.exp(-(a*(rx-x)**2+2*b*(rx-x)*(ry-y)+c*(ry-y)**2))
	
	return cost

def nonproxemic_apf_function(robot, cp): #Use collision avoidance for humans; no proxemics; used for APF only
	sigma = (14/12)*0.3048 #Use crew shoulder width (meters); equal in x and y
	A = 1

	rx = robot[:,X_POS]
	ry = robot[:,Y_POS]

	dfdx = np.zeros_like(rx)
	dfdy = np.zeros_like(ry)

	for x,y,theta in cp: 
		dfdx += (rx - x)*(A/sigma**2)*np.exp(-((rx-x)**2+(ry-y)**2)/(2*sigma**2))
		dfdy += (ry - y)*(A/sigma**2)*np.exp(-((rx-x)**2+(ry-y)**2)/(2*sigma**2))

	gradient = np.stack((dfdx, dfdy, np.zeros_like(rx)),axis=1)
	
	return gradient

def nonproxemic_astar_function(robot, cp): #Use collision avoidance for humans; no proxemics; used for A* only
	cost = 0
	sigma = (14/12)*0.3048 #Use crew shoulder width (meters); equal in x and y
	A = 1

	rx = robot[:,X_POS]
	ry = robot[:,Y_POS]
	cost = np.zeros_like(rx)

	for x,y,theta in cp:
		cost += A*np.exp(-((rx-x)**2+(ry-y)**2)/(2*sigma**2))
	
	return cost
