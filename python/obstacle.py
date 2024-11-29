import numpy as np
from data_handler import data_handler
from pose import Pose

class Obstacle:
    def __init__(self, obstacle_obj):
        self.id = obstacle_obj["id"]
        self.type = obstacle_obj["type"]
        self.a = obstacle_obj["a"]
        self.b = obstacle_obj["b"]

        self.A_orth = np.array([
            [1 / (self.a**2), 0],
            [0, 1 / (self.b**2)]
        ])

        pose_obj = obstacle_obj["pose"]
        self.pose = Pose(pose_obj["x"], pose_obj["y"], pose_obj["theta"])

        R = np.array([
            [np.cos(pose_obj["theta"]), -np.sin(pose_obj["theta"])],
            [np.sin(pose_obj["theta"]), np.cos(pose_obj["theta"])]
        ])

        self.A = R @ self.A_orth @ R.T


    def set_pose(self, pose):
        self.pose = pose


class ObstaclesHandler:
    def __init__(self):
        obstacles_obj = data_handler.get_obstacles_obj()
        self.obstacles = []

        for obstacle_obj in obstacles_obj:
            obstacle = Obstacle(obstacle_obj)
            self.obstacles.append(obstacle)


    def get_obstacles(self):
        return self.obstacles
    

obstacles_handler = ObstaclesHandler()