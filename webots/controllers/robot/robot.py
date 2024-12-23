import numpy as np
import cv2
import math
import struct

from controller import Robot, Supervisor
from controller import Gyro
from controller import InertialUnit
from controller import Motor
from controller import Emitter, Receiver

class Handler:
    def __init__(self):
        self.value = 0

    def inc(self):
        self.value += 1

handler = Handler()

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)

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

        motor_left.setVelocity(5.0)
        motor_right.setVelocity(5.0)

        #print('queue length: ' + str(receiver.getQueueLength()))
        print("value: " + str(handler.value))

        '''while receiver.getQueueLength() > 0:
            data = receiver.getFloats()
            print('data received: ' + str(data))
            receiver.nextPacket()'''


main()