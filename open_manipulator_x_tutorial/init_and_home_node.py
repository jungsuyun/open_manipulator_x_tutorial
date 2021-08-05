#!/usr/bin/env python3

import os
from getkey import getkey

import rclpy
from rclpy.node import Node

from open_manipulator_msgs.srv import SetJointPosition

PI = 3.14159265359
NUM_OF_JOINT = 4

class InitAndHome(Node):
    def __init__(self):
        self.future = None
        super().__init__('init_and_home_node')
        self.path_time = 2.0
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')

        self.request = SetJointPosition.Request()

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        command = getkey()
        self.send_command(str(command))

    def set_joint_space_path(self, joint_name: list, joint_angle: list):
        self.request.joint_position.joint_name = joint_name
        self.request.joint_position.position = joint_angle
        self.request.path_time = self.path_time

        future = self.client.call_async(self.request)

    def send_command(self, command: str):
        if command is '1':
            print("input : 1 \t init pose")
            joint_name = []
            joint_angle = []

            joint_name.append('joint1')
            joint_name.append('joint2')
            joint_name.append('joint3')
            joint_name.append('joint4')

            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(0.0)
            self.set_joint_space_path(joint_name, joint_angle)
        elif command is '2':
            print("input : 2 \t home pose")
            joint_name = []
            joint_angle = []

            joint_name.append('joint1')
            joint_name.append('joint2')
            joint_name.append('joint3')
            joint_name.append('joint4')

            joint_angle.append(0.0)
            joint_angle.append(-PI/3)
            joint_angle.append(PI/9)
            joint_angle.append(PI*2/9)

            self.set_joint_space_path(joint_name, joint_angle)
        else:
            print("input : %s" % command)

def main(args=None):
    rclpy.init(args=args)
    init_and_home = InitAndHome()
    while rclpy.ok():
        rclpy.spin_once(init_and_home)

    init_and_home.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
