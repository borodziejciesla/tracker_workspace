import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bag_file_createor'),
        'config',
        'sensor_config.yaml'
        )
    
    return LaunchDescription([
        Node(
            package='bag_file_createor',
            namespace='bag_file_createor',
            executable='bag_file_createor',
            parameters = [config],
            name='bag_file_createor'
        ),
        Node(
            package='radar_preprocessor',
            namespace='radar_preprocessor',
            executable='radar_preprocessor_wrapper',
            name='sim'
        ),
        Node(
            package='tracker',
            namespace='tracker',
            executable='tracker_wrapper',
            name='sim'
        ),
        Node(
            package='visualization',
            namespace='visualization',
            executable='radar_visualization',
            name='radar_visualization'
        ),
        Node(
            package='visualization',
            namespace='visualization',
            executable='tracker_visualization',
            name='tracker_visualization'
        )
    ])