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
        self.path_time = 2.0
        self.client = self.create_client(SetJointPosition, 'goal_tool_control')

        self.request = SetJointPosition.Request()

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        command = getkey()
        self.send_command(str(command))

    def send_command(self, command: str):
        if command is 'g' or command is 'G':
            print("input : g \topen gripper")
            joint_angle = []
            joint_angle.append(0.01)

            self.set_tool_control(joint_angle)

        elif command is 'f' or command is 'F':
            print("input : f \tclose gripper")
            joint_angle = []
            joint_angle.append(-0.01)

            self.set_tool_control(joint_angle)

    def set_tool_control(self, joint_angle: list):
        print(joint_angle)
        self.request.joint_position.joint_name.append('gripper')
        self.request.joint_position.position = joint_angle

        future = self.client.call_async(self.request)
        while future.done() is not None:
            try:
                response = future.result()
                print(response)
                break
            except Exception as e:
                print("%s" % (e, ))


def main(args=None):
    rclpy.init(args=args)
    gripper_control = GripperControl()
    while rclpy.ok():
        rclpy.spin_once(gripper_control)

    gripper_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()