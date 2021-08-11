#!/usr/bin/env python3

from rclpy.node import Node

from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose


class MarkerController(Node):
    def __init__(self):
        super().__init__('ar_marker_subscriber')

        self.target_marker = 0
        self.pose = Pose()
        self.subscription = self.create_subscription(ArucoMarkers, "/aruco_markers", self.marker_callback, 10)

    def marker_callback(self, msg: ArucoMarkers):
        for marker in msg.marker_ids:
            print(marker)
            if marker == self.target_marker:
                self.get_logger().info("Find!! %d" % marker)
                pose_index = msg.marker_ids.index(marker)
                pose = msg.poses[pose_index]
                self.pose = pose
