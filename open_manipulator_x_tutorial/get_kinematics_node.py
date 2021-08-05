#!/usr/bin/env python3

import os
from getkey import getkey

import rclpy
from rclpy.node import Node

from open_manipulator_msgs.msg import KinematicsPose

NUM_OF_JOINT = 4

class KinematicsSubscriber(Node):
    def __init__(self):
        super().__init__('kinematics_subscriber')
        self.subscription = self.create_subscription(KinematicsPose, 'kinematics_pose', self.kinematics_callback, 10)
        self.subscription

    def kinematics_callback(self, msg: KinematicsPose):
        os.system('clear')
        kinematics_pose_list = []
        kinematics_pose_list.append(msg.pose.position.x)
        kinematics_pose_list.append(msg.pose.position.y)
        kinematics_pose_list.append(msg.pose.position.z)

        self.print_kinematics(kinematics_pose_list)

    def print_kinematics(self, kinematics_list: list):
        self.get_logger().info("Present Kinematics Position X : %3lf" % kinematics_list[0])
        self.get_logger().info("Present Kinematics Position Y : %3lf" % kinematics_list[1])
        self.get_logger().info("Present Kinematics Position Z : %3lf" % kinematics_list[2])

def main(args = None):
    rclpy.init(args=args)
    kinematics_subscriber = KinematicsSubscriber()
    rclpy.spin(kinematics_subscriber)
    kinematics_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()