#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('hello_ros_subscriber')
        self.subscription = self.create_subscription(String, 'talker', self.talker_callback, 10)
        self.subscription

    def talker_callback(self, msg: String):
        self.get_logger().info('I heard : %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)

    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()