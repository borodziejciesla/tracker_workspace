from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tracker',
            namespace='tracker',
            executable='tracker_wrapper',
            name='sim'
        ),
        Node(
            package='visualization',
            namespace='visualization',
            executable='tracker_visualization',
            name='sim'
        )
    ])