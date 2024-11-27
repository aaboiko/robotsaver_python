import numpy as np

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.cartesian = np.array([x, y])
        self.theta = theta