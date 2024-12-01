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
        self.thread.start()


    def line_ellipse_intersect(self, x0, vec, xc, A):
        a = vec / np.linalg.norm(vec)
        xb = x0 - xc
        A_sq = a.T @ A @ a
        B_sq = 2 * a.T @ A @ xb
        C_sq = xb.T @ A @ xb - 1
        D = B_sq**2 - 4 * A_sq * C_sq

        return D >= 0
    

    def next_point(self, p_robot, p_target, obstacle_states):
        vec_to_target = p_target - p_robot
        direct_obs_r = -1
        direct_obs_A = np.zeros((2, 2))
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
                    direct_obs_A = obs_A
                    vec_to_obs = obs_pose_xy - p_robot
                    obs_xy = obs_pose_xy

        if n_direct_obs == 0:
            return p_target
        
        azimuth_to_obs = np.arctan2(vec_to_obs[1], vec_to_obs[0])
        azimuth_to_target = np.arctan2(vec_to_target[1], vec_to_target[0])
        axis_to_obs = vec_to_obs / np.linalg.norm(vec_to_obs)

        if azimuth_to_target >= azimuth_to_obs:
            angle_rot = np.pi / 2
        else:
            angle_rot = -np.pi / 2

        R = np.array([
            [np.cos(angle_rot), -np.sin(angle_rot)],
            [np.sin(angle_rot), np.cos(angle_rot)]
        ])

        p_bypass = obs_xy + obs_r * R @ axis_to_obs

        return self.next_point(p_robot, p_bypass, obstacle_states)
    

    def stop(self):
        self.running = False
        robot.stop()
        obstacles_handler.stop()


    def run(self):
        while(self.running):
            robot_pose_xy = np.array([robot.pose.x, robot.pose.y])
            robot_pose_theta = robot.pose.theta
            target_pose_xy = np.array([target.pose.x, target.pose.y])
            obstacle_states = obstacles_handler.get_obstacle_states()


path_planner = PathPlanner()