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
    

cameras_handler = CamerasHandler()