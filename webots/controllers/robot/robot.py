import numpy as np
import cv2
import math
import struct

from controller import Robot, Supervisor
from controller import Gyro
from controller import InertialUnit
from controller import Motor
from controller import Emitter, Receiver

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

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
        if robot.getTime() > 1.0:
            break

    while robot.step(timestep) != -1:
        time = robot.getTime()

        imu_values = imu.getRollPitchYaw()
        roll, pitch, yaw = imu_values
        gyro_values = gyro.getValues()
        roll_velocity, pitch_velocity, yaw_velocity = gyro_values

        np.savetxt('../data_handler/robot_imu.txt', np.array(imu_values), delimiter=' ')

        motor_torques = np.loadtxt('../data_handler/motors.txt', delimiter=' ')
        
        if motor_torques.shape[0] > 0:
            print('data: ' + str(motor_torques))
            left_motor_torque, right_motor_torque = motor_torques

            motor_left.setVelocity(left_motor_torque)
            motor_right.setVelocity(right_motor_torque)



main()