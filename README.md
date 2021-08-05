# open_manipulator_x_tutorial
본 리포지토리는 Open Manipulator X의 ROS2 연습용 리포지토리입니다.
![openManipulator](img/OpenManipulator_Introduction.jpg)

## __1. 개발환경__
* [Ubuntu 18.04 LTS](https://ubuntu.com/)
* [ROS2 Dashing](https://docs.ros.org/en/dashing/index.html)
* [Robotis Open Manipulator X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/#opensoftware)
* Python 3.6

## __2. 노드 소개__
* [__hello_ros_pub__](https://github.com/jungsuyun/open_manipulator_x_tutorial#4-hello_ros_pub) : Python 기반의 ROS2 Topic Publishing 예제
* __hello_ros_sub__ : Python 기반의 ROS2 Topic Subscribing 예제
* __init_and_home__ : 초기 위치, home 위치로 이동하기
* __gripper_control__ : Gripper 열고 닫기
* __jointstate_subscriber__ : 각 Joint별 각속도 값 읽어오기
* __kinematics_subscriber__ : Gripper를 기준으로 현재 위치를 X/Y/Z축 좌표값으로 읽어오기
* __joint_teleoperation__ : Joint 기반의 이동 명령 내리기
* __kinematics_teleoperation__ : X/Y/Z 좌표값 기반의 이동 명령 내리기

## __3. 개발환경 세팅하기__
### __3.1. Open Manipulator 패키지 설치하기__
가장 먼저 ros dashing의 python 패키지와 rqt 관련 패키지를 설치한다.
```bash
sudo apt install ros-dashing-python* ros-dashing-rqt*
```

다음으로 open mainpulator와 관련된 의존성 패키지들을 다운로드 및 빌드를 수행한다.
```bash
cd ~/colcon_ws/src
git clone -b ros2 https://github.com/ROBOTIS-GIT/DynamixelSDK.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/dynamixel-workbench.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/robotis_manipulator.git  
cd ~/colcon_ws && colcon build --symlink-install
```

### __3.2. USB Latency Timer Setting__
Ubuntu 환경에서 USB의 지연시간은 기본적으로 16ms로 설정되어 있다. 하지만 Open Manipulator X에 설치되어 있는 DYNAMIXEL과 PC의 통신 실시간성을 보장하기 위해 지연시간을 1ms로 설정하는 노드를 실행한다.
```bash
ros2 run open_manipulator_x_controller create_udev_rules
```

### __3.3. Hardware 구성하기__
U2D2와 PC를 Micro USB로 연결한 후 U2D2의 TTL포트와 U2D2 Power Hub Board의 TTL 파트를 연결한다. 그리고 나머지 포트와 Open Manipulator를 연결한다.
![U2D2 연결법](img/OpenManipulator_u2d2_setup2.png)

### __3.4. Python 의존성 패키지 설치하기__
다양한 key값을 받아올 수 있도록 우리는 getkey()라는 함수가 필요하다. 이는 Windows의 C++ 코드에는 구현되어 있지만, Ubuntu Linux에는 구현되어 있지 않다. 이를 위해 우리는 getkey()와 관련된 python 패키지를 설치해준다.
```bash
pip3 install getkey
```

## __4. hello_ros_pub__
본 패키지는 ROS2 기반의 프로그래밍 개발을 위해 가볍게 코드를 구현해보자는 의미에서 만든 패키지이다.
### 4.1. 노드 동작 Process

### 4.2. Source code 설명
가장 먼저 의존성 패키지들을 import 해준다. ROS2 기반의 python 프로그래밍을 위해선 `rclpy` 패키지를 import 해주어야 한다. 또한 `String` 타입의 메시지를 발행하기 위해 `std_msgs/msg/String` 타입을 import 해준다.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```

다음으로 클래스 선언 부분이다. 우리는 SimplePublisher라는 클래스를 새로 생성할 것이고 해당 클래스는 `rclpy.node`를 상속받게된다. 가장 먼저 부모 클래스의 `__init__` 함수를 통해 해당 Node 명을 선언해주고 `talker`라는 topic을 발행할 publisher를 선언해준다. 해당 topic은 `timer_callback` 함수를 통해 0.5초마다 반복 실행 될 것이다.
```python
class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'talker', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
```

`timer_callback` 함수에서는 `String`타입의 메시지를 선언해주고 해당 `msg`의 `data` 부분에 str 값을 입력해주게 된다.
```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello ROS %d' % self.i
    self.publisher.publish(msg)
    self.get_logger().info('Publishing: %s' % msg.data)
    self.i += 1
```

다음으로 메인에서는 `rclpy.init` 을 통해 노드 연결을 준비하고 `rclpy.spin()` 함수를 통해 무한루프 형태로 노드가 동작하도록 구현하였다.
```python
def main(args = None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)

    simple_publisher.destroy_node()
    rclpy.shutdown()
```

### 4.3. 구동하기
```bash
cd ~/colcon_ws && colcon build
ros2 run open_manipulator_x_tutorial hello_ros_pub
```
[구동화면 캡쳐하기]

## __5. hello_ros_sub__

