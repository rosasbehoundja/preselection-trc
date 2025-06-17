from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_data_evaluation',
            executable='sensor_publisher',
            name='sensor_publisher'
        ),
        Node(
            package='sensor_data_evaluation',
            executable='sensor_subscriber',
            name='sensor_subscriber'
        ),
    ])
