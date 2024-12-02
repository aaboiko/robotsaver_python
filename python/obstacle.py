import numpy as np
import threading
import time
from data_handler import data_handler
from pose import Pose

class Obstacle:
    def __init__(self, obstacle_obj):
        self.id = obstacle_obj["id"]
        self.type = obstacle_obj["type"]
        self.a = obstacle_obj["a"]
        self.b = obstacle_obj["b"]
        self.running = True

        self.A_orth = np.array([
            [1 / (self.a**2), 0],
            [0, 1 / (self.b**2)]
        ])

        pose_obj = obstacle_obj["pose"]
        self.pose = Pose(pose_obj["x"], pose_obj["y"], pose_obj["theta"])
        self.theta = pose_obj["theta"]

        R = np.array([
            [np.cos(pose_obj["theta"]), -np.sin(pose_obj["theta"])],
            [np.sin(pose_obj["theta"]), np.cos(pose_obj["theta"])]
        ])

        self.A = R @ self.A_orth @ R.T

        traj_file = obstacle_obj["traj"]
        self.traj = np.loadtxt(traj_file, delimiter=' ')

        self.mutex = threading.Lock()
        self.thread = threading.Thread(target=self.run)
        #self.thread.start()


    def set_pose(self, pose):
        self.pose = pose


    def stop(self):
        self.running = False


    def run(self):
        print('started thread for obstacle with id = ' + str(self.id))
        n = self.traj.shape[0]
        timestamp_prev = int(time.time() * 1000)
        ptr = 0

        while(self.running):
            timestamp = int(time.time() * 1000)
            dt = timestamp - timestamp_prev

            if dt >= 100:
                x, y, theta = self.traj[ptr,:]
                self.pose = Pose(x, y, theta)

                R = np.array([
                    [np.cos(theta), -np.sin(theta)],
                    [np.sin(theta), np.cos(theta)]
                ])

                self.A = R @ self.A_orth @ R.T

                timestamp_prev = timestamp
                ptr += 1

                if ptr == n:
                    ptr = 0


class ObstaclesHandler:
    def __init__(self):
        obstacles_obj = data_handler.get_obstacles_obj()
        self.obstacles = []

        for obstacle_obj in obstacles_obj:
            obstacle = Obstacle(obstacle_obj)
            self.obstacles.append(obstacle)


    def get_obstacles(self):
        return self.obstacles
    

    def get_obstacle_states(self):
        states = []

        for obstacle in self.obstacles:
            A = obstacle.A
            pose = obstacle.pose
            id = obstacle.id
            type = obstacle.type
            a = obstacle.a
            b = obstacle.b
            theta = obstacle.theta

            obj = {
                "id": id,
                "type": type,
                "A": A,
                "pose": pose,
                "a": a,
                "b": b,
                "theta": theta
            }

            states.append(obj)

        return states
    

    def stop(self):
        for obstacle in self.obstacles:
            obstacle.stop()
    

obstacles_handler = ObstaclesHandler()