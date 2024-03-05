from setuptools import setup
import os 
from glob import glob

package_name = 'my_robot_streamer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/my_robot_streamer/launch', glob('ros2_ws/src/my_robot_streamer/launch/*.launch.py')),  
        ('share/my_robot_streamer/config', glob('ros2_ws/src/my_robot_streamer/config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikhil',
    maintainer_email='nikhil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "publisher_node=my_robot_streamer.publisher:main",
            "subscriber_node=my_robot_streamer.subscriber:main",
            "ros_mqtt_bridge=my_robot_streamer.bridge_node:main"
        ],
    },
)
