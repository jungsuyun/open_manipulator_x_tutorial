#!/usr/bin/env python3

import os
from getkey import getkey

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import SetKinematicsPose

PI = 3.14159265359
NUM_OF_JOINT = 4

class InitAndHome(Node):
    def __init__(self):
        self.future = None
        super().__init__('init_and_home_node')
        self.path_time = 2.0
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')

        self.request = SetJointPosition.Request()
        self.joint_name = []
        self.joint_angle = []

    def set_joint_space_path(self, joint_name: list, joint_angle: list):
        self.request.joint_position.joint_name = joint_name
        self.request.joint_position.position = joint_angle
        self.request.path_time = self.path_time

        self.future = self.client.call_async(self.request)

    def send_command(self, command: str):
        tmp_joint_name = []
        tmp_joint_angle = []
        if command is '1':
            tmp_joint_name.append('joint1')
            tmp_joint_name.append('joint2')
            tmp_joint_name.append('joint3')
            tmp_joint_name.append('joint4')

            tmp_joint_angle.append(0.0)
            tmp_joint_angle.append(0.0)
            tmp_joint_angle.append(0.0)
            tmp_joint_angle.append(0.0)

            self.joint_name = tmp_joint_name
            self.joint_angle = tmp_joint_angle
            self.set_joint_space_path(tmp_joint_name, tmp_joint_angle)
        elif command is '2':
            tmp_joint_name.append('joint1')
            tmp_joint_name.append('joint2')
            tmp_joint_name.append('joint3')
            tmp_joint_name.append('joint4')

            tmp_joint_angle.append(0.0)
            tmp_joint_angle.append(-PI/3)
            tmp_joint_angle.append(PI/9)
            tmp_joint_angle.append(PI*2/9)

            self.joint_name = tmp_joint_name
            self.joint_angle = tmp_joint_angle
            self.set_joint_space_path(self.joint_name, self.joint_angle)
        else:
            print("input : %s" % command)