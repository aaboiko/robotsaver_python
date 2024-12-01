from robot import robot
from path_planner import PathPlanner
import time

planner = PathPlanner()

while(True):
    if int(time.time() * 1000) % 500 == 0:
        print("main: " + str(robot.get_fuck()))