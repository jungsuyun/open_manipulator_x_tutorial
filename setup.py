from setuptools import setup

package_name = 'open_manipulator_x_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jungsu',
    maintainer_email='yunjs@edgeilab.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_ros_pub = open_manipulator_x_tutorial.hello_ros_publisher:main',
            'hello_ros_sub = open_manipulator_x_tutorial.hello_ros_subscriber:main',
            'init_and_home = open_manipulator_x_tutorial.init_and_home_node:main',
            'gripper_control = open_manipulator_x_tutorial.gripper_control_node:main',
            'jointstate_subscriber = open_manipulator_x_tutorial.get_joint_state_node:main',
            'kinematics_subscriber = open_manipulator_x_tutorial.get_kinematics_node:main',
            'joint_teleoperation = open_manipulator_x_tutorial.joint_teleoperation:main',
            'kinematics_teleoperation = open_manipulator_x_tutorial.kinematics_teleoperation:main',
        ],
    },
)
