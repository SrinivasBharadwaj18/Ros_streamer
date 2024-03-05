from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
 
def generate_launch_description():
    config_path = LaunchConfiguration('/home/nikhil/Desktop/ros2_ws/src/my_robot_streamer/config/ros_mqtt_config.yaml')
    return LaunchDescription([
        Node(
            package='my_robot_streamer',
            executable='ros_mqtt_bridge',
            name='ros_mqtt_bridge',
            output='screen',
            parameters=[{'file_path': config_path}],
        ),
        Node(
            package='my_robot_streamer',
            executable='publisher_node',
            name='strem_publisher',
            output='screen',
        ),
    ])
 