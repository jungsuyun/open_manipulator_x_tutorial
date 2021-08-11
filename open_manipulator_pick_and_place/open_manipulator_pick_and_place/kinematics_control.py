#!/usr/bin/env python3

from getkey import getkey

import rclpy
from rclpy.node import Node

from open_manipulator_msgs.srv import SetKinematicsPose


class KinematicsController(Node):
    def __init__(self):
        super().__init__("kinematics_controller")

        self.client = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.request = SetKinematicsPose.Request()

        self.path_time = 2.0

    def send_command(self, goal_position: list, goal_orientation: list):
        self.set_task_space_path_from_present_position_only(goal_position, goal_orientation, self.path_time)

    def set_task_space_path_from_present_position_only(self, goal_position: list, goal_orientation: list, path_time):
        self.request.end_effector_name = 'gripper'

        self.request.kinematics_pose.pose.position.x = goal_position[0]
        self.request.kinematics_pose.pose.position.y = goal_position[1]
        self.request.kinematics_pose.pose.position.z = goal_position[2]

        self.request.kinematics_pose.pose.orientation.x = goal_orientation[0]
        self.request.kinematics_pose.pose.orientation.y = goal_orientation[1]
        self.request.kinematics_pose.pose.orientation.z = goal_orientation[2]
        self.request.kinematics_pose.pose.orientation.w = goal_orientation[3]
        self.request.path_time = path_time
        future = self.client.call_async(self.request)
