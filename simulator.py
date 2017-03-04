import numpy as np

''' Is this the robot itself? '''
class SimulatedObject(object):
    X_POS = 0
    Y_POS = 1
    THETA_POS = 2
    DX_POS = 3
    DY_POS = 4
    DTHETA_POS = 5

    def __init__(self, x=0, y=0, theta=0, dx=0, dy=0, dtheta=0,
        m=0, w=0, h=0):
        # self.x, self.y, self.theta = x, y, theta
        # self.dx, self.dy, self.dtheta = dx, dy, dtheta

        self.state_trajectory = np.zeros((1,6))
        self.state_trajectory[0,type(self).X_POS] = x
        self.state_trajectory[0,type(self).Y_POS] = y
        self.state_trajectory[0,type(self).THETA_POS] = theta
        self.state_trajectory[0,type(self).DX_POS] = dx
        self.state_trajectory[0,type(self).DY_POS] = dy
        self.state_trajectory[0,type(self).DTHETA_POS] = dtheta

        self.m = m
        self.w, self.h = w, h
        self.izz = m*(w**2 + h**2)/12

    
    #what is the -1 doing here?
    @property
    def x(self):
        return self.state_trajectory[-1,type(self).X_POS]

    @property
    def y(self):
        return self.state_trajectory[-1,type(self).Y_POS]

    @property
    def theta(self):
        return self.state_trajectory[-1,type(self).THETA_POS]

    @property
    def dx(self):
        return self.state_trajectory[-1,type(self).DX_POS]

    @property
    def dy(self):
        return self.state_trajectory[-1,type(self).DY_POS]

    @property
    def dtheta(self):
        return self.state_trajectory[-1,type(self).DTHETA_POS]

    @property
    def positions(self):
        return self.state_trajectory[-1,
            [type(self).X_POS,type(self).Y_POS,type(self).THETA_POS]
          ]

    @property
    def velocities(self):
        return self.state_trajectory[-1,
            [type(self).DX_POS,type(self).DY_POS,type(self).DTHETA_POS]
          ]

    def time_step(self,dx,dy,dtheta):
        new_state = np.zeros((1,6))

        new_state[0,type(self).DX_POS] = dx
        new_state[0,type(self).DY_POS] = dy
        new_state[0,type(self).DTHETA_POS] = dtheta

        new_state[0,type(self).X_POS] = self.x + dx
        new_state[0,type(self).Y_POS] = self.y + dy
        new_state[0,type(self).THETA_POS] = self.theta + dtheta

        self.state_trajectory = np.vstack((self.state_trajectory, new_state))
        

""" what is going on here? 
I'm assuming m/w/h are mass, width, height? what kind of weird units are these? """
class Robot(SimulatedObject):
    def __init__(self, x, y, theta, dx=0, dy=0, dtheta=0):
        super().__init__(x,y,theta, dx, dy, dtheta, 
            m=24.5, w=12*2.54E-2,h=12*2.54E-2)

"""what is the list of objects supposed to be? Is this a grouping of module/humans/movement requirements?
does this interact with apf? What does it interact with? """
class Simulator(object):
    def __init__(*list_of_objects):
        self.objects = list_of_objects

    ''' is this running through time steps and recording data? '''
    def simulate(n_steps):
        pass