import numpy as np
from data_handler import data_handler
from pose import Pose

class Target:
    def __init__(self):
        target_obj = data_handler.get_target_obj()

        pose_obj = target_obj["pose"]
        self.pose = Pose(pose_obj["x"], pose_obj["y"], pose_obj["theta"])


target = Target()