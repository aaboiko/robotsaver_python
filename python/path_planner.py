import numpy as np
import threading
import time
from robot import robot
from obstacle import obstacles_handler
from target import target


R_PERCEPTION = 2

class PathPlanner:
    def __init__(self):
        self.running = True
        self.mutex = threading.Lock()
        self.thread = threading.Thread(target=self.run)
        #self.thread.start()


    def line_ellipse_intersect(self, x0, vec, xc, A):
        a = vec / np.linalg.norm(vec)
        xb = x0 - xc
        A_sq = a.T @ A @ a
        B_sq = 2 * a.T @ A @ xb
        C_sq = xb.T @ A @ xb - 1
        D = B_sq**2 - 4 * A_sq * C_sq

        t = (-B_sq - np.sqrt(D)) / (2 * A_sq)
        print('D = ' + str(D) + ', t = ' + str(t))
        return D >= 0 and t >= 0
    

    def obstacle_is_forward(self, p_robot, robot_theta, obstacle_states):
        vec = np.array([np.cos(robot_theta), np.sin(robot_theta)])
        direct_obs_dist = np.inf
        vec_to_obs = np.zeros(2)
        obs_xy = np.zeros(2)
        n_direct_obs = 0

        for obs_state in obstacle_states:
            obs_A = obs_state["A"]
            obs_pose = obs_state["pose"]
            obs_pose_xy = np.array([obs_pose.x, obs_pose.y])

            if self.line_ellipse_intersect(p_robot, vec, obs_pose_xy, obs_A):
                dist = np.linalg.norm(obs_pose_xy - p_robot)
                n_direct_obs += 1

                if dist < direct_obs_dist:
                    direct_obs_dist = dist
                    vec_to_obs = obs_pose_xy - p_robot
                    obs_xy = obs_pose_xy

        has_obstacles = n_direct_obs > 0

        return has_obstacles, obs_xy, vec_to_obs, direct_obs_dist
    

    def next_point(self, p_robot, p_target, obstacle_states):
        vec_to_target = p_target - p_robot
        direct_obs_r = -1
        direct_obs_dist = np.inf
        vec_to_obs = np.zeros(2)
        obs_xy = np.zeros(2)
        n_direct_obs = 0

        for obs_state in obstacle_states:
            obs_r = max(obs_state["a"], obs_state["b"])
            obs_A = obs_state["A"]
            obs_pose = obs_state["pose"]
            obs_pose_xy = np.array([obs_pose.x, obs_pose.y])

            if self.line_ellipse_intersect(p_robot, vec_to_target, obs_pose_xy, obs_A):
                dist = np.linalg.norm(obs_pose_xy - p_robot)
                n_direct_obs += 1

                if dist < direct_obs_dist:
                    direct_obs_dist = dist
                    direct_obs_r = obs_r
                    vec_to_obs = obs_pose_xy - p_robot
                    obs_xy = obs_pose_xy

        if n_direct_obs == 0:
            return p_target
        
        azimuth_to_obs = np.arctan2(vec_to_obs[1], vec_to_obs[0])
        azimuth_to_target = np.arctan2(vec_to_target[1], vec_to_target[0])
        axis_to_obs = vec_to_obs / np.linalg.norm(vec_to_obs)

        if azimuth_to_target >= azimuth_to_obs:
            angle_rot = np.pi / 4
        else:
            angle_rot = -np.pi / 4

        R = np.array([
            [np.cos(angle_rot), -np.sin(angle_rot)],
            [np.sin(angle_rot), np.cos(angle_rot)]
        ])

        p_bypass = obs_xy + 2 * direct_obs_r * R @ axis_to_obs
        print('p_bypass: ' + str(p_bypass))

        return self.next_point(p_robot, p_bypass, obstacle_states)
    

    def stop(self):
        self.running = False
        robot.stop()
        obstacles_handler.stop()


    def step(self):
        robot_pose_xy = np.array([robot.pose.x, robot.pose.y])
        robot_pose_theta = robot.pose.theta
        target_pose_xy = np.array([target.pose.x, target.pose.y])
        obstacle_states = obstacles_handler.get_obstacle_states()
        acc_brake = robot.motor_max_force / (robot.k * robot.mass)

        has_obstacles, obs_xy, vec_to_obs, obs_dist = self.obstacle_is_forward(robot_pose_xy, robot_pose_theta, obstacle_states)
        
        if has_obstacles:
            v_ref = np.sqrt(2 * acc_brake * obs_dist)
            v_e = v_ref - robot.velocity
            azimuth_to_obs = np.arctan2(vec_to_obs[1], vec_to_obs[0])
            level = v_e / abs(v_e) * 100

            if robot_pose_theta >= azimuth_to_obs:
                robot.set_left_motor_level(0)
                robot.set_right_motor_level(level)
            else:
                robot.set_left_motor_level(level)
                robot.set_right_motor_level(0)
        else:
            vec_to_target = target_pose_xy - robot_pose_xy
            target_dist = np.linalg.norm(vec_to_target)
            azimuth_to_target = np.arctan2(vec_to_target[1], vec_to_target[0])
            theta_e = azimuth_to_target - robot_pose_theta
            v_ref = np.sqrt(2 * acc_brake * target_dist)
            v_e = v_ref - robot.velocity

            level_left = (robot.k_obs_avoid * v_e - robot.k_theta_e * theta_e) / 2
            level_right = (robot.k_obs_avoid * v_e + robot.k_theta_e * theta_e) / 2
            robot.set_left_motor_level(level_left)
            robot.set_right_motor_level(level_right)


    def run(self):
        print('path planner started...')

        while(self.running):
            self.step()


path_planner = PathPlanner()