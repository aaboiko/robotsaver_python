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

camera_xyz = np.array([0, 0, 10])
#camera_rpy = np.array([0, 0, 0])


def box_to_plane(box, camera, camera_rpy):
    camera_width = camera.getWidth()
    camera_height = camera.getHeight()

    fov_h = camera.getFov()
    fov_v = 2 * np.arctan((fov_h / 2) * (camera_height / camera_width))

    camera_roll, camera_pitch, camera_yaw = camera_rpy
    camera_x, camera_y, camera_z = camera_xyz
    x, y, w, h = box

    pan_offset = (x - 0.5) * fov_h
    tilt_offset = (y - 0.5) * fov_v 

    pan = camera_yaw - pan_offset
    tilt = camera_pitch + tilt_offset

    ax = -np.cos(pan) * np.cos(tilt)
    ay = np.sin(pan) * np.cos(tilt)
    az = -np.sin(tilt)
    
    t = -camera_z / az
    obj_x = ax * t + camera_x
    obj_y = ay * t + camera_y

    obj_pose = np.array([obj_x, obj_y])

    d = np.sqrt((camera_x - obj_x)**2 + (camera_y - obj_y)**2 + camera_z**2)
    a = d * np.tan((w / 2) * fov_h)
    b = (d * np.tan((h / 2) * fov_v)) / np.cos(tilt)

    print('obj_pose: ' + str(obj_pose) + ', camera_yaw: ' + str(camera_yaw) + ', camera_pitch: ' + str(camera_pitch) + ", d = " + str(d) + ', a = ' + str(a) + ', b = ' + str(b) + ', w = ' + str(w) + ', h = ' + str(h))

    return obj_pose


def get_objects_from_image(camera, camera_rpy, model):
    image = camera.getImage()
    camera_width = camera.getWidth()
    camera_height = camera.getHeight()
    
    image_rgba = np.frombuffer(image, np.uint8).reshape((camera_height, camera_width, 4)) 
    image_rgb = cv2.cvtColor(image_rgba, cv2.COLOR_RGBA2RGB)

    results = model(image_rgb)
    classes = results[0].boxes.cls.cpu().numpy()
    boxes = results[0].boxes.xywhn.cpu().numpy()
    obj_pose = np.zeros(2)
    #print('CLASSES: ' + str(classes))
    #print("BOXES: " + str(boxes))
    if len(boxes) > 0:
        obj_pose = box_to_plane(boxes[0], camera, camera_rpy)

    return obj_pose
        

def main():
    robot = Supervisor()
    robot_name = robot.getSelf().getField('name').getSFString()
    
    model = YOLO(model_name)
    
    print('robot initiated')
    timestep = int(robot.getBasicTimeStep())
    print('timestep: ' + str(timestep))

    camera_yaw, camera_pitch = 0, 0
    left_motor_force, right_motor_force = 0, 0

    keyboard = Keyboard()
    keyboard.enable(timestep)

    if robot_name == "camera_post":
        camera = robot.getDevice('camera')
        camera.enable(timestep)

        camera_roll_motor = robot.getDevice("camera roll")
        camera_pitch_motor = robot.getDevice("camera pitch")
        camera_yaw_motor = robot.getDevice("camera yaw")

        print("Start the camera post...\n")


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

    while robot.step(timestep) != -1:
        time = robot.getTime()
        key = keyboard.getKey()

        if robot_name == "camera_post":
            camera_rpy = [0, camera_pitch, camera_yaw]

            res = get_objects_from_image(camera, camera_rpy, model)

            if key > 0:
                if key == Keyboard.UP:
                    camera_pitch += camera_motor_const
                elif key == Keyboard.DOWN:
                    camera_pitch -= camera_motor_const
                elif key == Keyboard.LEFT:
                    camera_yaw += camera_motor_const
                elif key == Keyboard.RIGHT:
                    camera_yaw -= camera_motor_const

            camera_yaw_motor.setPosition(camera_yaw)
            camera_pitch_motor.setPosition(camera_pitch)

        if robot_name == "robot_saver":
            imu_values = imu.getRollPitchYaw()
            roll, pitch, yaw = imu_values
            gyro_values = gyro.getValues()
            roll_velocity, pitch_velocity, yaw_velocity = gyro_values

            if key > 0:
                if key == ord('W'):
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

            motor_left.setVelocity(left_motor_force)
            motor_right.setVelocity(right_motor_force)
            
        #print('yaw = ' + str(camera_yaw) + ', pitch = ' + str(camera_pitch) + ', left = ' + str(left_motor_force) + ', right = ' + str(right_motor_force))

main()