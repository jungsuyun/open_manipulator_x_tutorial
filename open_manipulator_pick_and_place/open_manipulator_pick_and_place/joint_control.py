#!/usr/bin/env python3

from getkey import getkey

import rclpy
from rclpy.node import Node

from open_manipulator_msgs.srv import SetJointPosition


class JointTeleoperation(Node):
    def __init__(self):
        self.future = None
        super().__init__('init_and_home_node')

        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path_from_present')
        self.request = SetJointPosition.Request()

        self.joint_delta = 0.05
        self.path_time = 0.5

    def send_command(self, command: str):
        if command is 'q' or command is 'Q' :
            print("input : q \tincrease(++) joint 1 angle\n")
            joint_name = []
            joint_angle = []

            joint_name.append("joint1")
            joint_name.append("joint2")
            joint_name.append("joint3")
            joint_name.append("joint4")

            joint_angle.append(self.joint_delta)
            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(0.0)

            self.set_joint_space_path_from_present(joint_name, joint_angle)

        elif command is 'a' or command is 'A' :
            print("input : q \tdecrease(--) joint 1 angle\n")
            joint_name = []
            joint_angle = []

            joint_name.append("joint1")
            joint_name.append("joint2")
            joint_name.append("joint3")
            joint_name.append("joint4")

            joint_angle.append(self.joint_delta * -1)
            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(0.0)

            self.set_joint_space_path_from_present(joint_name, joint_angle)

        elif command is 'w' or command is 'W' :
            print("input : w \tincrease(++) joint 2 angle\n")
            joint_name = []
            joint_angle = []

            joint_name.append("joint1")
            joint_name.append("joint2")
            joint_name.append("joint3")
            joint_name.append("joint4")

            joint_angle.append(0.0)
            joint_angle.append(self.joint_delta)
            joint_angle.append(0.0)
            joint_angle.append(0.0)

            self.set_joint_space_path_from_present(joint_name, joint_angle)

        elif command is 's' or command is 'S' :
            print("input : w \tdecrease(--) joint 2 angle\n")
            joint_name = []
            joint_angle = []

            joint_name.append("joint1")
            joint_name.append("joint2")
            joint_name.append("joint3")
            joint_name.append("joint4")

            joint_angle.append(0.0)
            joint_angle.append(self.joint_delta * -1.0)
            joint_angle.append(0.0)
            joint_angle.append(0.0)

            self.set_joint_space_path_from_present(joint_name, joint_angle)

        elif command is 'e' or command is 'E' :
            print("input : e \tincrease(++) joint 3 angle\n")
            joint_name = []
            joint_angle = []

            joint_name.append("joint1")
            joint_name.append("joint2")
            joint_name.append("joint3")
            joint_name.append("joint4")

            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(self.joint_delta)
            joint_angle.append(0.0)

            self.set_joint_space_path_from_present(joint_name, joint_angle)

        elif command is 'd' or command is 'D' :
            print("input : w \tdecrease(--) joint 3 angle\n")
            joint_name = []
            joint_angle = []

            joint_name.append("joint1")
            joint_name.append("joint2")
            joint_name.append("joint3")
            joint_name.append("joint4")

            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(self.joint_delta * -1.0)
            joint_angle.append(0.0)

            self.set_joint_space_path_from_present(joint_name, joint_angle)

        elif command is 'r' or command is 'R' :
            print("input : r \tincrease(++) joint 4 angle\n")
            joint_name = []
            joint_angle = []

            joint_name.append("joint1")
            joint_name.append("joint2")
            joint_name.append("joint3")
            joint_name.append("joint4")

            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(self.joint_delta)

            self.set_joint_space_path_from_present(joint_name, joint_angle)

        elif command is 'f' or command is 'F' :
            print("input : w \tdecrease(--) joint 4 angle\n")
            joint_name = []
            joint_angle = []

            joint_name.append("joint1")
            joint_name.append("joint2")
            joint_name.append("joint3")
            joint_name.append("joint4")

            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(self.joint_delta * -1.0)

            self.set_joint_space_path_from_present(joint_name, joint_angle)

    def set_joint_space_path_from_present(self, joint_name: list, joint_angle: list):
        self.request.joint_position.joint_name = joint_name
        self.request.joint_position.position = joint_angle
        self.request.path_time = self.path_time

        future = self.client.call_async(self.request)
