from setuptools import setup

package_name = 'open_manipulator_pick_and_place'

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
    maintainer_email='69963988+jungsuyun@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_and_place = open_manipulator_pick_and_place.pick_and_place_node:main',
        ],
    },
)
