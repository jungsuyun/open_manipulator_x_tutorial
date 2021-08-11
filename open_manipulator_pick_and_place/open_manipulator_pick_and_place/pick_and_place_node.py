#!/usr/bin/env python3

import getch
import time

import rclpy

from open_manipulator_pick_and_place.get_kinematics import KinematicsSubscriber
from open_manipulator_pick_and_place.set_actuator_state import ActuatorController
from open_manipulator_pick_and_place.init_and_home import InitAndHome
from open_manipulator_pick_and_place.kinematics_control import KinematicsController
from open_manipulator_pick_and_place.joint_control import JointTeleoperation
from open_manipulator_pick_and_place.gripper_control import GripperControl

command_usage = """
+-----------------------------------+
| Open Manipulator Pick and Place!! |
+-----------------------------------+
|             명 령 어              |
+-----------------------------------+
|  1 : pick 위치 지정하기           |
|  2 : place 위치 지정하기          |
|  3 : Pick and Place 구동          |
|  q : 종료하기                     |
+-----------------------------------+
"""


def main(args=None):
    rclpy.init(args=args)

    kinematics_subscriber = KinematicsSubscriber()
    actuator_controller = ActuatorController()
    init_and_home = InitAndHome()
    kinematics_controller = KinematicsController()
    joint_controller = JointTeleoperation()
    gripper_controller = GripperControl()

    pick_kinematics_position = []
    pick_kinematics_orientation = []

    up_kinematics_position = []
    up_kinematics_orientation = []

    place_kinematics_position = []
    place_kinematics_orientation = []

    if rclpy.ok():
        while True:
            print(command_usage)
            print("input command : ", end='')
            command = input()
            if command is '1' :
                print("go home")
                init_and_home.send_command('2')
                rclpy.spin_once(init_and_home)
                time.sleep(2)
                print("pick할 위치로 이동해주세요. gripper를 잘 잡아주세요. 위치를 조정했으면 아무키나 눌러주세요.")
                while True:
                    command = getch.getche()
                    if command == '\n':
                        break
                    else:
                        joint_controller.send_command(command)
                        rclpy.spin_once(joint_controller)

                        rclpy.spin_once(kinematics_subscriber)
                        pick_kinematics_position = kinematics_subscriber.get_kinematics_pose()
                        pick_kinematics_orientation = kinematics_subscriber.get_kinematics_orientation()

                        up_kinematics_position = pick_kinematics_position.copy()
                        up_kinematics_orientation = pick_kinematics_orientation.copy()
                        up_kinematics_position[2] = up_kinematics_position[2] + 0.10

                print("pick 위치: ", pick_kinematics_position, pick_kinematics_orientation)

            if command is '2' :
                print("go home")
                init_and_home.send_command('2')
                rclpy.spin_once(init_and_home)
                time.sleep(2)
                print("place 할 위치로 이동해주세요. gripper를 잘 잡아주세요. 위치를 조정했으면 아무키나 눌러주세요.")
                while True:
                    command = getch.getche()
                    if command == '\n':
                        break
                    else:
                        joint_controller.send_command(command)
                        rclpy.spin_once(joint_controller)

                        rclpy.spin_once(kinematics_subscriber)
                        place_kinematics_position = kinematics_subscriber.get_kinematics_pose()
                        place_kinematics_orientation = kinematics_subscriber.get_kinematics_orientation()

                print("place 위치 : ", place_kinematics_position, place_kinematics_orientation)

            elif command is '3':
                if not pick_kinematics_position or not pick_kinematics_orientation:
                    print("pickup 위치 지정이 잘못되었습니다! 다시 해주세요!")
                elif not place_kinematics_position or not place_kinematics_orientation:
                    print("place 위치 지정이 잘못되었습니다! 다시 해주세요!")

                else:
                    actuator_controller.set_request(True)
                    rclpy.spin_once(actuator_controller)
                    time.sleep(2)

                    print("go home")
                    init_and_home.send_command('2')
                    rclpy.spin_once(init_and_home)
                    time.sleep(2)

                    print("mission start!")

                    while rclpy.ok():
                        print("go pick position")
                        kinematics_controller.send_command(pick_kinematics_position, pick_kinematics_orientation)
                        rclpy.spin_once(kinematics_controller)
                        time.sleep(2)

                        print("gripper close")
                        gripper_controller.send_command('f')
                        rclpy.spin_once(gripper_controller)
                        time.sleep(2)

                        print("bring_up")
                        kinematics_controller.send_command(up_kinematics_position, up_kinematics_orientation)
                        rclpy.spin_once(kinematics_controller)
                        time.sleep(2)

                        print("go place position")
                        kinematics_controller.send_command(place_kinematics_position, place_kinematics_orientation)
                        rclpy.spin_once(kinematics_controller)
                        time.sleep(2)

                        print("gripper open")
                        gripper_controller.send_command('g')
                        rclpy.spin_once(gripper_controller)
                        time.sleep(2)

                        print("go home")
                        init_and_home.send_command('2')
                        rclpy.spin_once(init_and_home)
                        time.sleep(3)

            elif command is 'q':
                break

    kinematics_subscriber.destroy_node()
    actuator_controller.destroy_node()
    init_and_home.destroy_node()
    kinematics_controller.destroy_node()
    joint_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()