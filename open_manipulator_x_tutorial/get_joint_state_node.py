#!/usr/bin/env python3

import os
from getkey import getkey

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

NUM_OF_JOINT = 4

class JointSubscriber(Node):
    def __init__(self):
        super().__init__('jointstate_subscriber')
        self.subscription = self.create_subscription(JointState, 'joint_states', self.jointstate_callback, 10)
        self.subscription

        self.jointstate_list = []

    def jointstate_callback(self, msg: JointState):
        os.system('clear')
        for i in range(NUM_OF_JOINT):
            self.jointstate_list.append(msg.position[i])
        self.print_state(self.jointstate_list)

    def print_state(self, joint_state: list):
        for i in range(NUM_OF_JOINT):
            self.get_logger().info("Present Joint %s state : %f" % (str(i+1), self.jointstate_list[i]))
        self.jointstate_list.clear()

def main(args = None):
    rclpy.init(args=args)
    joint_subscriber = JointSubscriber()
    rclpy.spin(joint_subscriber)
    joint_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()