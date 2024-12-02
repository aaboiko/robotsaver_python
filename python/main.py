from path_planner import path_planner
from obstacle import obstacles_handler
from robot import robot
from target import target
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse

obstacle_states = obstacles_handler.get_obstacle_states()
p_target = np.array([target.pose.x, target.pose.y])
p_robot = np.array([robot.pose.x, robot.pose.y])


def draw_obstacles():
    for state in obstacle_states:
        pose = state["pose"]
        theta = state["theta"]
        a = state["a"]
        b = state["b"]
        ellipse = Ellipse(xy=[pose.x, pose.y], width=2*a, height=2*b, angle=np.rad2deg(theta))
        plt.gca().add_artist(ellipse)


def draw_points(points):
    for p1, p2 in zip(points[:-1], points[1:]):
        x1, y1 = p1
        x2, y2 = p2
        plt.plot([x1, x2], [y1, y2], color='blue')