import json

class DataHandler:
    def __init__(self):
        with open('config.json', 'r') as file:
            json_obj = json.load(file)

        self.robot_obj = json_obj["robot"]
        self.obstacles_obj = json_obj["obstacles"]

    def get_robot_obj(self):
        return self.robot_obj
    

    def get_obstacles_obj(self):
        return self.obstacles_obj
    

data_handler = DataHandler()