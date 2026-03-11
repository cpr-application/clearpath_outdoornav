import os

from clearpath_config.common.utils.yaml import read_yaml
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'


def launch_setup(context, *args, **kwargs):

    # Read YAML
    robot_config_path = LaunchConfiguration('robot_config_path',
                                            default=DEFAULT_ROBOT_CONFIG_PATH).perform(context)
    robot_config = read_yaml(robot_config_path)

    namespace = robot_config['system']['ros2']['namespace']
    namespace_prefix = '' if namespace == '' else '/' + namespace

    declared_arguments = []

    platform = Node(
        package='clearpath_outdoornav_simulation',
        executable='platform_sim',
        name='platform_sim',
        output='screen',
    )

    localization = Node(
        package='clearpath_outdoornav_simulation',
        executable='localization_sim',
        name='localization_sim',
        output='screen',
    )

    return declared_arguments + [  # noqa: RUF005
        platform,
        localization,
    ]


def generate_launch_description():

    return LaunchDescription(
       [OpaqueFunction(function=launch_setup)]
    )
