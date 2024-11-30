import numpy as np
import json
import threading
import time
from data_handler import data_handler
from pose import Pose

class Robot:
    def __init__(self):
        robot_obj = data_handler.get_robot_obj()

        self.mass = robot_obj["mass"]
        self.length = robot_obj["length"]
        self.width = robot_obj["width"]
        self.inertia = (1/12) * self.mass * (self.length**2 + self.width**2)
        self.k = robot_obj["k"]
        self.motor_bias = robot_obj["motor_bias"]
        self.motor_max_force = robot_obj["motor_max_force"]
        self.control_type = robot_obj["control_type"]

        self.pose = Pose(0, 0, 0)
        self.dpose = Pose(0, 0, 0)
        self.velocity = np.linalg.norm(self.dpose.cartesian)
        self.ddpose = Pose(0, 0, 0)

        self.left_motor_force = 0
        self.right_motor_force = 0
        self.left_motor_level = 0
        self.right_motor_level = 0
        self.external_force = np.zeros(2)

        self.mutex = threading.Lock()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()


    def set_pose(self, pose):
        self.pose = pose


    def set_dpose(self, dpose):
        self.dpose = dpose
        self.velocity = np.linalg.norm(self.dpose.cartesian)


    def set_velocity(self, velocity):
        self.velocity = velocity
        vx = velocity * np.cos(self.pose.theta)
        vy = velocity * np.sin(self.pose.theta)
        self.dpose = Pose(vx, vy, 0)


    def left_motor_apply_force(self, force):
        self.left_motor_force = force
        self.left_motor_level = (self.left_motor_force / self.motor_max_force) * 100


    def right_motor_apply_force(self, force):
        self.left_motor_force = force
        self.right_motor_level = (self.right_motor_force / self.motor_max_force) * 100


    def set_left_motor_level(self, level):
        self.left_motor_level = level
        self.left_motor_force = (level / 100) * self.motor_max_force


    def set_right_motor_level(self, level):
        self.right_motor_level = level
        self.right_motor_force = (level / 100) * self.motor_max_force


    def set_external_force(self, force):
        self.external_force = force


    def run(self):
        print('robot thread started...')

        timestamp_prev = time.time()

        while(True):
            timestamp = time.time()
            dt = timestamp - timestamp_prev

            internal_acceleration_linear = (self.left_motor_force + self.right_motor_force) / (self.k * self.mass)
            internal_acceleration_rotational = self.motor_bias * (self.right_motor_force - self.left_motor_force) / self.inertia
            external_acceleration_vector = self.external_force / self.mass
            internal_acceleration_vector = internal_acceleration_linear * np.array([np.cos(self.pose.theta), np.sin(self.pose.theta)])
            acceleration_vector = internal_acceleration_vector + external_acceleration_vector
            self.ddpose = Pose(acceleration_vector[0], acceleration_vector[1], internal_acceleration_rotational)

            velocity_vector_linear = np.array([self.dpose.x, self.dpose.y]) + acceleration_vector * dt
            dtheta = self.dpose.theta + internal_acceleration_rotational * dt
            self.dpose = Pose(velocity_vector_linear[0], velocity_vector_linear[1], dtheta)
            self.velocity = np.linalg.norm(velocity_vector_linear)

            pose_xy = np.array([self.pose.x, self.pose.y]) + velocity_vector_linear
            theta = self.pose.theta + dtheta * dt
            self.pose = Pose(pose_xy[0], pose_xy[1], theta)

            timestamp_prev = timestamp


robot = Robot()