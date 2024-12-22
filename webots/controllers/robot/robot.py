import numpy as np
import cv2
import math

from controller import Robot, Supervisor
from controller import Gyro
from controller import InertialUnit
from controller import Motor

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    motor = robot.getDevice("robot_left_motor")
    motor.setPosition(np.inf)
    motor.setVelocity(1.0)

    while robot.step(timestep) != -1:
        if robot.getTime() > 1.0:
            break

    while robot.step(timestep) != -1:
        time = robot.getTime()

        imu_values = imu.getRollPitchYaw()
        roll, pitch, yaw = imu_values
        gyro_values = gyro.getValues()
        roll_velocity, pitch_velocity, yaw_velocity = gyro_values

        motor.setVelocity(2.0)


main()