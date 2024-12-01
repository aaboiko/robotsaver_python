import numpy as np
import threading
import time
from robot import robot
from obstacle import obstacles_handler
from target import target

R_PERCEPTION = 2

class PathPlanner:
    def __init__(self):
        self.mutex = threading.Lock()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()


    def line_ellipse_intersect(self, x0, theta, xc, A):
        a = np.array([np.cos(theta), np.sin(theta)])
        xb = x0 - xc
        A_sq = a.T @ A @ A
        B_sq = 2 * a.T @ A @ xb
        C_sq = xb.T @ A @ xb - 1
        D = B_sq**2 - 4 * A_sq * C_sq

        return D >= 0


    def run(self):
        while(True):
            robot_pose_xy = np.array([robot.pose.x, robot.pose.y])
            robot_pose_theta = robot.pose.theta
            target_pose_xy = np.array([target.pose.x, target.pose.y])
            obstacle_states = obstacles_handler.get_obstacle_states()


