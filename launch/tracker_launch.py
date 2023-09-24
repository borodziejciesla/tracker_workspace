import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sensors_static_tf_broadcaster'),
        'config',
        'sensors_config.yaml'
        )
    
    return LaunchDescription([
        Node(
            package = 'sensors_static_tf_broadcaster',
            name = 'sensors_static_tf_broadcaster',
            executable = 'sensors_static_tf_broadcaster_node'
            #parameters = [config]
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
        ),
         Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('config'), 'config', 'rviz_config.rviz')]
        )
    ])