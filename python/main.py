from path_planner import path_planner
from obstacle import obstacles_handler
from robot import robot
from target import target
import matplotlib.pyplot as plt
import numpy as np
import time
from matplotlib.patches import Ellipse, Rectangle, Circle


obstacle_states = obstacles_handler.get_obstacle_states()
p_target = np.array([target.pose.x, target.pose.y])
p_robot = np.array([robot.pose.x, robot.pose.y])


def draw_target():
    pose = target.pose
    circle = Circle(xy=[pose.x, pose.y], radius=1, color="green")
    plt.gca().add_artist(circle)


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


#robot.set_left_motor_level(70)
#robot.set_right_motor_level(100)
n_iters = 0
timestamp_prev = time.time()
traj = []

while(n_iters < 1000):
    timestamp = time.time()
    dt = timestamp - timestamp_prev
    path_planner.step()
    robot.step(dt)
    timestamp_prev = timestamp
    #print("iter: " + str(n_iters))

    n_iters += 1
    plt.clf()
    plt.xlim((0, 60))
    plt.ylim((0, 60))

    pose = robot.pose
    '''traj.append([pose.x, pose.y])
    for p in traj:
        x, y = p
        plt.scatter(x, y, s=1, color="red")'''

    rect = Rectangle(xy=[pose.x, pose.y], width=robot.width, height=robot.length, rotation_point='center', angle=np.rad2deg(pose.theta), color='red')
    plt.gca().add_artist(rect)
    draw_obstacles()
    draw_target()
    plt.pause(0.01)

plt.show()