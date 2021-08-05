#!/usr/bin/env python3

from getkey import getkey

import rclpy
from rclpy.node import Node

from open_manipulator_msgs.srv import SetKinematicsPose

class KinematicsTeleoperation(Node):
    def __init__(self):
        super().__init__("kinematics_teleoperation")

        self.client = self.create_client(SetKinematicsPose, 'goal_task_space_path_from_present_position_only')
        self.request = SetKinematicsPose.Request()

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.delta = 0.01
        self.path_time = 0.5

    def timer_callback(self):
        command = getkey()
        self.send_command(command)

    def send_command(self, command: str):
        if command is 'u' or command is 'U':
            print("input : u \tincrease(++) x axis in task space")
            goal_pose = []
            goal_pose.append(self.delta)
            goal_pose.append(0.0)
            goal_pose.append(0.0)

            self.set_task_space_path_from_present_position_only(goal_pose, self.path_time)

        elif command is 'j' or command is 'J':
            print("input : u \tdecrease(--) x axis in task space")
            goal_pose = []
            goal_pose.append(self.delta * -1)
            goal_pose.append(0.0)
            goal_pose.append(0.0)

            self.set_task_space_path_from_present_position_only(goal_pose, self.path_time)

        elif command is 'i' or command is 'I':
            print("input : i \tincrease(++) y axis in task space")
            goal_pose = []
            goal_pose.append(0.0)
            goal_pose.append(self.delta)
            goal_pose.append(0.0)

            self.set_task_space_path_from_present_position_only(goal_pose, self.path_time)

        elif command is 'k' or command is 'K':
            print("input : u \tdecrease(--) y axis in task space")
            goal_pose = []
            goal_pose.append(0.0)
            goal_pose.append(self.delta * -1)
            goal_pose.append(0.0)

            self.set_task_space_path_from_present_position_only(goal_pose, self.path_time)

        elif command is 'o' or command is 'O':
            print("input : o \tincrease(++) z axis in task space")
            goal_pose = []
            goal_pose.append(0.0)
            goal_pose.append(0.0)
            goal_pose.append(self.delta)

            self.set_task_space_path_from_present_position_only(goal_pose, self.path_time)

        elif command is 'l' or command is 'L':
            print("input : u \tdecrease(--) z axis in task space")
            goal_pose = []
            goal_pose.append(0.0)
            goal_pose.append(0.0)
            goal_pose.append(self.delta * -1)

            self.set_task_space_path_from_present_position_only(goal_pose, self.path_time)

    def set_task_space_path_from_present_position_only(self, goal_pose, path_time):
        self.request.planning_group = "gripper"
        self.request.kinematics_pose.pose.position.x = goal_pose[0]
        self.request.kinematics_pose.pose.position.y = goal_pose[1]
        self.request.kinematics_pose.pose.position.z = goal_pose[2]
        self.request.path_time = path_time

        future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    kinematics_teleoperation = KinematicsTeleoperation()
    rclpy.spin(kinematics_teleoperation)
    kinematics_teleoperation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()