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
        ),
        Node(
            package='navigation',
            namespace='',
            executable='pid_controller',
            name='pid_controller'
        ),
        Node(
            package='actuator_output',
            namespace='',
            executable='motor_controller',
            name='motor_controller'
        )
    ])