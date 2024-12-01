import numpy as np
#import quaternion
from data_handler import data_handler

class Camera:
    def __init__(self, params_obj):
        self.id = params_obj["id"]
        self.cx = params_obj["cx"]
        self.cy = params_obj["cy"]
        self.fx = params_obj["fx"]
        self.fy = params_obj["fy"]
        self.fov = params_obj["fov"]
        self.s = params_obj["s"]

        position_obj = params_obj["position"]
        orientation_obj = params_obj["orientation"]
        euler_obj = orientation_obj["euler"]

        self.position = np.array([position_obj["x"], position_obj["y"], position_obj["z"]])
        self.orientation = np.array([euler_obj["roll"], euler_obj["pitch"], euler_obj["yaw"]])


class CamerasHandler:
    def __init__(self):
        cameras_obj = data_handler.get_cameras_obj()
        self.cameras = []

        for camera_obj in cameras_obj:
            camera = Camera(camera_obj)
            self.cameras.append(camera)


    def get_cameras(self):
        return self.cameras
    

    def get_camera_params(self):
        params = []

        for camera in self.cameras:
            id = camera.id
            fx = camera.fx
            fy = camera.fy
            cx = camera.cx
            cy = camera.cy
            s = camera.s
            position = camera.position
            orientation = camera.orientation

            roll, pitch, yaw = orientation

            K = np.array([
                [fx, s, cx],
                [0, fy, cy],
                [0, 0, 1]
            ])

            R1 = np.array([
                [np.cos(roll), -np.sin(roll), 0],
                [np.sin(roll), np.cos(roll), 0],
                [0, 0, 1]
            ])

            R2 = np.array([
                [1, 0, 0],
                [0, np.cos(pitch), -np.sin(pitch)],
                [0, np.sin(pitch), np.cos(pitch)]
            ])

            R3 = np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1]
            ])

            R = R1 @ R2 @ R3

            obj = {
                "id": id,
                "K": K,
                "R": R,
                "t": position
            }

            params.append(obj)

        return params
    

cameras_handler = CamerasHandler()