# open_manipulator_x_tutorial
본 리포지토리는 Open Manipulator X의 ROS2 연습용 리포지토리입니다.

## 개발환경
* [Ubuntu 18.04 LTS](https://ubuntu.com/)
* [ROS2 Dashing](https://docs.ros.org/en/dashing/index.html)
* [Robotis Open Manipulator X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/#opensoftware)
* Python 3.6

## 노드 소개
* __hello_ros_pub__ : Python 기반의 ROS2 Topic Publishing 예제
* __hello_ros_sub__ : Python 기반의 ROS2 Topic Subscribing 예제
* __init_and_home__ : 초기 위치, home 위치로 이동하기
* __gripper_control__ : Gripper 열고 닫기
* __jointstate_subscriber__ : 각 Joint별 각속도 값 읽어오기
* __kinematics_subscriber__ : Gripper를 기준으로 현재 위치를 X/Y/Z축 좌표값으로 읽어오기
* __joint_teleoperation__ : Joint 기반의 이동 명령 내리기
* __kinematics_teleoperation__ : X/Y/Z 좌표값 기반의 이동 명령 내리기

