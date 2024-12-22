import numpy as np
import cv2
import math
import matplotlib.pyplot as plt

from controller import Robot, Supervisor
from controller import Camera
from controller import Compass
from controller import GPS
from controller import Gyro
from controller import InertialUnit
from controller import Keyboard
from controller import LED
from controller import Motor


k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0

target_altitude = 1.0


def CLAMP(value, low, high):
    if value < low:
        return low
    else:
        if value > high:
            return high
        else:
            return value
        

def get_control_inputs(roll_disturbance, pitch_disturbance, yaw_disturbance, target_altitude, gps_values, gyro_values, imu_values):
    altitude = gps_values[2]
    roll_velocity, pitch_velocity, yaw_velocity = gyro_values
    roll, pitch, yaw = imu_values

    roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance
    pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance
    yaw_input = yaw_disturbance
    clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * clamped_difference_altitude**3

    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input

    return [front_left_motor_input, front_right_motor_input, rear_left_motor_input, rear_right_motor_input]
        

def main():
    robot = Supervisor()
    print('robot initiated')
    timestep = int(robot.getBasicTimeStep())
    print('timestep: ' + str(timestep))

    camera = robot.getDevice('camera')
    camera.enable(timestep)
    front_left_led = robot.getDevice("front left led")
    front_right_led = robot.getDevice("front right led")
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    camera_roll_motor = robot.getDevice("camera roll")
    camera_pitch_motor = robot.getDevice("camera pitch")

    front_left_motor = robot.getDevice("front left propeller")
    front_right_motor = robot.getDevice("front right propeller")
    rear_left_motor = robot.getDevice("rear left propeller")
    rear_right_motor = robot.getDevice("rear right propeller")
    motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

    for motor in motors:
        motor.setPosition(np.inf)
        motor.setVelocity(1.0)

    print("Start the drone...\n")

    while robot.step(timestep) != -1:
        if robot.getTime() > 1.0:
            break

    while robot.step(timestep) != -1:
        time = robot.getTime()

        imu_values = imu.getRollPitchYaw()
        roll, pitch, yaw = imu_values
        gps_values = gps.getValues()
        altitude = gps_values[2]
        gyro_values = gyro.getValues()
        roll_velocity, pitch_velocity, yaw_velocity = gyro_values

        led_state = int(time) % 2
        front_left_led.set(led_state)
        front_right_led.set(1 - led_state)

        camera_roll_motor.setPosition(-0.115 * roll_velocity)
        camera_pitch_motor.setPosition(-0.1 * pitch_velocity)

        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0

        front_left_motor_input, front_right_motor_input, rear_left_motor_input, rear_right_motor_input = get_control_inputs(roll_disturbance, pitch_disturbance, yaw_disturbance, target_altitude, gps_values, gyro_values, imu_values)

        front_left_motor.setVelocity(front_left_motor_input)
        front_right_motor.setVelocity(-front_right_motor_input)
        rear_left_motor.setVelocity(-rear_left_motor_input)
        rear_right_motor.setVelocity(rear_right_motor_input)


main()