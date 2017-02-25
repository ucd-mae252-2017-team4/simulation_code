import numpy as np

class SimulatedObject(object):
    X_POS = 0
    Y_POS = 1
    THETA_POS = 2
    DX_POS = 3
    DY_POS = 4
    DTHETA_POS = 5

    def __init__(self, x=0, y=0, theta=0, dx=0, dy=0, dtheta=0):
        # self.x, self.y, self.theta = x, y, theta
        # self.dx, self.dy, self.dtheta = dx, dy, dtheta

        self.state_trajectory = np.zeros((1,6))
        self.state_trajectory[0,type(self).X_POS] = x
        self.state_trajectory[0,type(self).Y_POS] = y
        self.state_trajectory[0,type(self).THETA_POS] = theta
        self.state_trajectory[0,type(self).DX_POS] = dx
        self.state_trajectory[0,type(self).DY_POS] = dy
        self.state_trajectory[0,type(self).DTHETA_POS] = dtheta

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

    def time_step(dx,dy,dtheta)
        new_state = np.zeros((1,6))

        new_state[0,type(self).DX_POS] = dx
        new_state[0,type(self).DY_POS] = dy
        new_state[0,type(self).DTHETA_POS] = dtheta

        new_state[0,type(self).X_POS] = self.x + dx
        new_state[0,type(self).Y_POS] = self.y + dy
        new_state[0,type(self).THETA_POS] = self.theta + dtheta

        self.state_trajectory = np.vstack((self.state_trajectory, new_state))
        


class Robot(SimulatedObject):
    def __init__(self, x, y, theta, dx=0, dy=0, dtheta=0):
        super().__init__(x,y,theta, dx, dy, dtheta)

class Simulator(object):
    def __init__(*list_of_objects):
        self.objects = list_of_objects

    def simulate(n_steps):

