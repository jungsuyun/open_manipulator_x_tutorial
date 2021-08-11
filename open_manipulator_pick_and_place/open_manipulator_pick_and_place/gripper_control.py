#!/usr/bin/env python3

import os
from getkey import getkey

import rclpy
from rclpy.node import Node

from open_manipulator_msgs.srv import SetJointPosition

class GripperControl(Node):
    def __init__(self):
        self.future = None
        super().__init__("gripper_control_node")
        self.path_time = 1.0
        self.client = self.create_client(SetJointPosition, 'goal_tool_control')

        self.request = SetJointPosition.Request()

    def send_command(self, command: str):
        if command is 'g' or command is 'G':
            joint_angle = []

            joint_name = ['gripper', ]
            joint_angle.append(0.01)

            self.set_tool_control(joint_angle, joint_name)

        elif command is 'f' or command is 'F':
            joint_angle = []

            joint_name = ['gripper', ]
            joint_angle.append(-0.01)

            self.set_tool_control(joint_angle, joint_name)

    def set_tool_control(self, joint_angle: list, joint_name: list):
        self.request.joint_position.joint_name = joint_name
        self.request.joint_position.position = joint_angle
        self.request.path_time = self.path_time

        future = self.client.call_async(self.request)