import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import struct

from ultralytics import YOLO

from controller import Robot, Supervisor
from controller import Camera
from controller import Keyboard
from controller import Motor
from controller import Emitter, Receiver

from controllers.robot.robot import handler

model_name = "yolov8n"
        

def main():
    robot = Supervisor()
    print('robot initiated')
    timestep = int(robot.getBasicTimeStep())
    print('timestep: ' + str(timestep))

    camera = robot.getDevice('camera')
    camera.enable(timestep)

    camera_roll_motor = robot.getDevice("camera roll")
    camera_pitch_motor = robot.getDevice("camera pitch")
    camera_yaw_motor = robot.getDevice("camera yaw")

    print("Start the camera post...\n")

    model = YOLO(model_name)
    emitter = robot.getDevice("emitter")

    keyboard = Keyboard()
    keyboard.enable(timestep)

    camera_yaw = 0
    camera_pitch = 0

    while robot.step(timestep) != -1:
        time = robot.getTime()

        image = camera.getImage()
        image_rgba = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)) 
        image_rgb = cv2.cvtColor(image_rgba, cv2.COLOR_RGBA2RGB)

        results = model(image_rgb)

        data = struct.pack(">f", 1.0)
        emitter.send(data)
        #print('data sent: ' + data)
        handler.inc()

        #camera_roll_motor.setPosition(0)
        #camera_pitch_motor.setPosition(0)

        key = keyboard.getKey()

        if key > 0:
            if key == Keyboard.UP:
                camera_pitch += 0.01
            elif key == Keyboard.DOWN:
                camera_pitch -= 0.01
            elif key == Keyboard.LEFT:
                camera_yaw += 0.01
            elif key == Keyboard.RIGHT:
                camera_yaw -= 0.01

        camera_yaw_motor.setPosition(camera_yaw)
        camera_pitch_motor.setPosition(camera_pitch)

main()