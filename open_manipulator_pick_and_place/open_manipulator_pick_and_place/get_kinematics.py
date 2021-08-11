import rclpy
from rclpy.node import Node

from open_manipulator_msgs.msg import KinematicsPose

NUM_OF_JOINT = 4


class KinematicsSubscriber(Node):
    def __init__(self):
        super().__init__('kinematics_subscriber')
        self.subscription = self.create_subscription(KinematicsPose, 'kinematics_pose', self.kinematics_callback, 10)
        self.subscription
        self.kinematics_position = []
        self.kinematics_orientation = []

    def kinematics_callback(self, msg: KinematicsPose):
        kinematics_pose_list = []
        kinematics_orientation_list = []
        kinematics_pose_list.append(msg.pose.position.x)
        kinematics_pose_list.append(msg.pose.position.y)
        kinematics_pose_list.append(msg.pose.position.z)

        kinematics_orientation_list.append(msg.pose.orientation.x)
        kinematics_orientation_list.append(msg.pose.orientation.y)
        kinematics_orientation_list.append(msg.pose.orientation.z)
        kinematics_orientation_list.append(msg.pose.orientation.w)

        self.kinematics_position = kinematics_pose_list.copy()
        self.kinematics_orientation = kinematics_orientation_list

    def get_kinematics_pose(self):
        return self.kinematics_position

    def get_kinematics_orientation(self):
        return self.kinematics_orientation
