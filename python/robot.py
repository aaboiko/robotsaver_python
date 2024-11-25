import numpy as np
import json
from data_handler import data_handler

class Robot:
    def __init__(self):
        robot_obj = data_handler.get_robot_obj()

        self.mass = robot_obj["mass"]
        self.length = robot_obj["length"]
        self.width = robot_obj["width"]
        self.inertia = (1/12) * self.mass * (self.length**2 + self.width**2)
        self.motor_bias = robot_obj["motor_bias"]
        self.motor_max_force = robot_obj["motor_max_force"]
        self.control_type = robot_obj["control_type"]