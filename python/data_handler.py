import json

class DataHandler:
    def __init__(self):
        with open('config.json', 'r') as file:
            json_obj = json.load(file)

        self.robot_obj = json_obj["robot"]
        self.obstacles_obj = json_obj["obstacles"]

        self.cameras_obj = json_obj["cameras"]


    def get_robot_obj(self):
        return self.robot_obj
    

    def get_obstacles_obj(self):
        return self.obstacles_obj
    
    
    def get_cameras_obj(self):
        return self.cameras_obj
    

data_handler = DataHandler()