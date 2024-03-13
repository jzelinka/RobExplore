#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

#import messages
from messages import *

# changed to make it work
DELTA_DISTANCE = 0.12
C_TURNING_SPEED = 5
C_AVOID_SPEED = 20
ORIENTATION_THRESHOLD = np.pi/16

class HexapodController:
    def __init__(self):
        pass

    def goto(self, goal: Pose, odometry: Odometry, collision):
        """Method to steer the robot towards the goal position given its current 
           odometry and collision status
        Args:
            goal: Pose of the robot goal
            odometry: Perceived odometry of the robot
            collision: bool of the robot collision status
        Returns:
            cmd: Twist steering command
        """
        if goal is None or odometry is None:
            cmd_msg = Twist()
            return cmd_msg

        # zero velocity steering command
        if collision:
            return None
        
        robot_p = np.array([odometry.pose.position.x, odometry.pose.position.y])
        robot_heading = odometry.pose.orientation.to_Euler()[0]
        goal_p = np.array([goal.position.x, goal.position.y])

        goal_relative = goal_p - robot_p

        if np.linalg.norm(goal_relative) < DELTA_DISTANCE:
            print("goal reached")
            return None

        goal_angle = np.arctan2(goal_relative[1], goal_relative[0])
        cmd_msg = Twist()
        if abs(goal_angle - robot_heading) > ORIENTATION_THRESHOLD:
            heading_error = goal_angle - robot_heading
            heading_error = heading_error + 2 * np.pi if heading_error < -np.pi else heading_error - 2 * np.pi if heading_error > np.pi else heading_error
            
            cmd_msg.angular.z = C_TURNING_SPEED * np.sign(heading_error)
        else:
            cmd_msg.linear.x = 1

        return cmd_msg            


    def goto_reactive(self, goal, odometry, collision, laser_scan: LaserScan):
        """Method to steer the robot towards the goal position while avoiding 
           contact with the obstacles given its current odometry, collision 
           status and laser scan data
        Args:
            goal: Pose of the robot goal
            odometry: Perceived odometry of the robot
            collision: bool of the robot collision status
            laser_scan: LaserScan data perceived by the robot
        Returns:
            cmd: Twist steering command
        """
        #zero velocity steering command
        if goal is None or odometry is None or laser_scan is None:
            cmd_msg = Twist()
            return cmd_msg

        # zero velocity steering command
        if collision:
            return None
        
        robot_p = np.array([odometry.pose.position.x, odometry.pose.position.y])
        robot_heading = odometry.pose.orientation.to_Euler()[0]
        goal_p = np.array([goal.position.x, goal.position.y])

        num_measurements = len(laser_scan.distances)
        scan_left = min(x for x in laser_scan.distances[:num_measurements//2] if x < laser_scan.range_max and x > laser_scan.range_min)
        scan_right = min(x for x in laser_scan.distances[num_measurements//2:] if x < laser_scan.range_max and x > laser_scan.range_min)
        repulsive_force = 1/scan_left - 1/scan_right

        goal_relative = goal_p - robot_p
        goal_distance = np.linalg.norm(goal_relative)

        if goal_distance < DELTA_DISTANCE:
            print("goal reached")
            return None

        goal_angle = np.arctan2(goal_relative[1], goal_relative[0])
        cmd_msg = Twist()

        heading_error = goal_angle - robot_heading
        heading_error = heading_error + 2 * np.pi if heading_error < -np.pi else heading_error - 2 * np.pi if heading_error > np.pi else heading_error

        angular_speed_navigation_component = C_TURNING_SPEED * heading_error
        angular_speed_avoidance_component = repulsive_force * C_AVOID_SPEED

        cmd_msg.angular.z = angular_speed_avoidance_component + angular_speed_navigation_component
        cmd_msg.linear.x = 10
        
        return cmd_msg
