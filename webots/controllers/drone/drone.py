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

robot_motor_max_force = 5.0
robot_k = 0.3
robot_mass = 50
robot_k_obs_avoid = 5.0
robot_k_theta_e = 100

camera_xyz = np.array([0, 0, 10])
#camera_rpy = np.array([0, 0, 0])


def line_ellipse_intersect(x0, vec, xc, a, b, theta):
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    A_orth = np.array([
        [1 / (a**2), 0],
        [0, 1 / (b**2)]
    ])

    A = R @ A_orth @ R.T
    ax = vec / np.linalg.norm(vec)
    xb = x0 - xc
    A_sq = ax.T @ A @ ax
    B_sq = 2 * ax.T @ A @ xb
    C_sq = xb.T @ A @ xb - 1
    D = B_sq**2 - 4 * A_sq * C_sq

    t = (-B_sq - np.sqrt(D)) / (2 * A_sq)
    print('D = ' + str(D) + ', t = ' + str(t))
    return D >= 0 and t >= 0


def obstacle_is_forward(p_robot, robot_theta, obstacle_params):
    vec = np.array([np.cos(robot_theta), np.sin(robot_theta)])
    direct_obs_dist = np.inf
    vec_to_obs = np.zeros(2)
    obs_xy = np.zeros(2)
    n_direct_obs = 0

    for obs_param in obstacle_params:
        x = obs_param["x"]
        y = obs_param["y"]
        theta = obs_param["theta"]
        a = obs_param
        b = obs_param["b"]

        obs_pose_xy = np.array([x, y])

        if line_ellipse_intersect(p_robot, vec, obs_pose_xy, a, b, theta):
            dist = np.linalg.norm(obs_pose_xy - p_robot)
            n_direct_obs += 1

            if dist < direct_obs_dist:
                direct_obs_dist = dist
                vec_to_obs = obs_pose_xy - p_robot
                obs_xy = obs_pose_xy

    has_obstacles = n_direct_obs > 0

    return has_obstacles, obs_xy, vec_to_obs, direct_obs_dist


def robot_step(robot_pose_xy, robot_pose_theta, robot_velocity, target_pose_xy, obstacle_params, left_motor, right_motor):
    acc_brake = robot_motor_max_force / (robot_k * robot_mass)

    has_obstacles, obs_xy, vec_to_obs, obs_dist = obstacle_is_forward(robot_pose_xy, robot_pose_theta, obstacle_params)
    
    if has_obstacles:
        v_ref = np.sqrt(2 * acc_brake * obs_dist)
        v_e = v_ref - robot_velocity
        azimuth_to_obs = np.arctan2(vec_to_obs[1], vec_to_obs[0])
        level = v_e / abs(v_e) * 100

        if robot_pose_theta >= azimuth_to_obs:
            left_motor.setVelocity(0)
            right_motor.setVelocity(level)
        else:
            left_motor.setVelocity(level)
            right_motor.setVelocity(0)
    else:
        vec_to_target = target_pose_xy - robot_pose_xy
        target_dist = np.linalg.norm(vec_to_target)
        azimuth_to_target = np.arctan2(vec_to_target[1], vec_to_target[0])
        theta_e = azimuth_to_target - robot_pose_theta
        v_ref = np.sqrt(2 * acc_brake * target_dist)
        v_e = v_ref - robot_velocity

        level_left = (robot_k_obs_avoid * v_e - robot_k_theta_e * theta_e) / 2
        level_right = (robot_k_obs_avoid * v_e + robot_k_theta_e * theta_e) / 2
        left_motor.setVelocity(level_left)
        right_motor.setVelocity(level_right)


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
    offset = np.array([pan_offset, tilt_offset]) 

    R = np.array([
        [np.cos(camera_roll), -np.sin(camera_roll)],
        [np.sin(camera_roll), np.cos(camera_roll)]
    ])

    offset_rot = R.T @ offset
    pan_offset_rot, tilt_offset_rot = offset_rot

    pan = camera_yaw - pan_offset_rot
    tilt = camera_pitch + tilt_offset_rot

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

    return obj_pose, a, b, pan


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

    camera_roll, camera_yaw, camera_pitch = 0, 0, 0
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
                elif key == ord('Z'):
                    camera_roll -= camera_motor_const
                elif key == ord('X'):
                    camera_roll += camera_motor_const

            camera_yaw_motor.setPosition(camera_yaw)
            camera_pitch_motor.setPosition(camera_pitch)
            camera_roll_motor.setPosition(camera_roll)

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