from __future__ import division
import numpy as np

class GoalController:
    """Finds linear and angular velocities necessary to drive toward
    a goal pose.
    """

    def __init__(self):
        self.k1 = 1
        self.k2 = 3
        self.kv = 0.7
        self.beta = 0.4
        self.lmbda = 2
        self.goal = {'x': 0., 'y': 0., 't': 0.}
        self.pose = {'x': 0., 'y': 0., 't': 0.}
        self.state = {'r': 0., 't': 0., 'd': 0.}
        self.control = {'do': 0., 'v': 0., 'w': 0.}
        # self.OneDegree = 0.0174533
        # self.linearTolerance = 0.05 # 2.5cm
        # self.angularTolerance = 5/180*pi # 3 degrees

    def get_control_op(self):
        self.polarize()
        self.get_virtual_control()
        # self.get_real_control()
        print(self.control, self.k, self.z)
        return self.control['v'], self.control['w']

    def get_virtual_control(self):
        self.k1_t = self.k1*self.state['t']
        self.control['do'] = np.arctan(-self.k1_t)
        self.z = self.state['d'] - self.control['do']
        self.k = -1/self.state['r']*(
            self.k2*self.z +
            (1 + self.k1/((1 + self.k1_t)**2))*np.sin(self.state['d'])
        )
        self.control['v'] = 3/(1 + self.beta*np.abs(self.k)**self.lmbda)
        self.control['w'] = self.control['v']*self.k
        
    def polarize(self):
        self.diff_vector = np.array([self.goal['x'] - self.pose['x'],
            self.goal['y']- self.pose['y']])
        self.state['r'] = np.linalg.norm(self.diff_vector)
        self.state['t'] = np.arctan2(*(self.diff_vector[::-1]))
        self.state['d'] = self.goal['t'] - self.pose['t']
        if self.state['d'] < -np.pi:
            self.state['d'] = 2*np.pi + self.state['d']
        elif self.state['d'] > np.pi:
            self.state['d']  = self.state['d'] - 2*np.pi

    def set_goal(self, **goal):
        self.goal.update(goal)

    def set_pose(self, **pose):
        self.pose.update(pose)

    def at_goal(self):
        return self.state['r'] < 0.05 and self.state['d'] < np.radians(3)