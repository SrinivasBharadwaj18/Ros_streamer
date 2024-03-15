from launch import LaunchDescription
from launch_ros.actions import Node


 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_streamer',
            executable='ros_mqtt_bridge',
            name='ros_mqtt_bridge',
            output='screen',
        ),
        Node(
            package='my_robot_streamer',
            executable='publisher_node',
            name='strem_publisher',
            output='screen',
        ),
    ])
 