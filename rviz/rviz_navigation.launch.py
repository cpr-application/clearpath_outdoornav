import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_rviz_config = os.path.join(os.path.dirname(__file__), 'outdoornav.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot ROS namespace',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to the RViz configuration file',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=LaunchConfiguration('namespace'),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        ),
    ])
