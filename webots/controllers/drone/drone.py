import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import struct
import sys

from ultralytics import YOLO

from controller import Robot, Supervisor
from controller import Camera
from controller import Keyboard
from controller import Motor
from controller import Emitter, Receiver

model_name = "yolov8n"

camera_motor_const = 0.01
robot_motor_const = 0.1

def get_objects_from_image(camera, model):
    image = camera.getImage()
    image_rgba = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)) 
    image_rgb = cv2.cvtColor(image_rgba, cv2.COLOR_RGBA2RGB)

    results = model(image_rgb)

    #data = np.array([1.0, 1.0])
    #np.savetxt('../data_handler/motors.txt', data, delimiter=' ')

    return results
        

def main():
    robot = Supervisor()
    robot_name = robot.getSelf().getField('name').getSFString()
    print('name' + robot_name)
    model = YOLO(model_name)
    
    print('robot initiated')
    timestep = int(robot.getBasicTimeStep())
    print('timestep: ' + str(timestep))

    if robot_name == "camera_post":
        camera = robot.getDevice('camera')
        camera.enable(timestep)

        camera_roll_motor = robot.getDevice("camera roll")
        camera_pitch_motor = robot.getDevice("camera pitch")
        camera_yaw_motor = robot.getDevice("camera yaw")

        print("Start the camera post...\n")

        keyboard = Keyboard()
        keyboard.enable(timestep)

        camera_yaw = 0
        camera_pitch = 0

    if robot_name == "robot_saver":
        imu = robot.getDevice("inertial unit")
        imu.enable(timestep)
        gyro = robot.getDevice("gyro")
        gyro.enable(timestep)

        motor_left = robot.getDevice("robot_left_motor")
        motor_right = robot.getDevice("robot_right_motor")

        motor_left.setPosition(np.inf)
        motor_left.setVelocity(0.0)
        motor_right.setPosition(np.inf)
        motor_right.setVelocity(0.0)

        left_motor_force, right_motor_force = 0, 0

    while robot.step(timestep) != -1:
        time = robot.getTime()

        res = get_objects_from_image(camera, model)

        key = keyboard.getKey()

        if key > 0:
            if key == Keyboard.UP:
                camera_pitch += camera_motor_const
            elif key == Keyboard.DOWN:
                camera_pitch -= camera_motor_const
            elif key == Keyboard.LEFT:
                camera_yaw += camera_motor_const
            elif key == Keyboard.RIGHT:
                camera_yaw -= camera_motor_const

            elif key == ord('W'):
                left_motor_force += robot_motor_const
                right_motor_force += robot_motor_const
            elif key == ord('S'):
                left_motor_force -= robot_motor_const
                right_motor_force -= robot_motor_const
            elif key == ord('A'):
                left_motor_force -= robot_motor_const
                right_motor_force += robot_motor_const
            elif key == ord('D'):
                left_motor_force += robot_motor_const
                right_motor_force -= robot_motor_const

        camera_yaw_motor.setPosition(camera_yaw)
        camera_pitch_motor.setPosition(camera_pitch)

        #data = np.array([left_motor_force, right_motor_force])
        #np.savetxt('../data_handler/motors.txt', data, delimiter=' ')

        print('yaw = ' + str(camera_yaw) + ', pitch = ' + str(camera_pitch) + ', left = ' + str(left_motor_force) + ', right = ' + str(right_motor_force) + ', robot_saver: ' + str(robot_saver))

        motor_left.setVelocity(left_motor_force)
        motor_right.setVelocity(right_motor_force)

main()