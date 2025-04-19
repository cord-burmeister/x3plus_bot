from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'x3plus_wrapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name,'rviz'),glob(os.path.join('rviz','*.rviz*'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'geometry_msgs', 'sensor_msgs'],
    zip_safe=True,
    maintainer='marvin',
    maintainer_email='cord.burmeister@live.de',
    description='Package mapping the driver functionality into the ROS2 world. This package includes launch files, RViz configuration files, parameter files, and the main driver code for the robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              'Mecanum_driver_X3Plus = x3plus_wrapper.Mecanum_driver_X3Plus:main',
        ],
    },
)
