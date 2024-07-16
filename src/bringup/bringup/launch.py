from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_readings',
            namespace='',
            executable='VideoPublisher',
            name='video_publisher'
        ),
        Node(
            package='data_processing',
            namespace='',
            executable='video_processor',
            name='video_processor'
        )
    ])