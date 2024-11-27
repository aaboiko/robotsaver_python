import numpy as np
import quaternion
from data_handler import data_handler

class Camera:
    def __init__(self, params_obj):
        self.position = params_obj["position"]
        self.orientation = params_obj["orientation"]


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