#!/usr/bin/env python3

import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from horiokart_drivers.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    pkg_name = "horiokart_simulator_client"
    pkg_share = get_package_share_directory(pkg_name)

    # Launch arguments
    launch_argument_creator = LaunchArgumentCreator()

    simulator_group = launch.actions.GroupAction(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory(
                        "ros_tcp_endpoint") + "/launch/endpoint.py"
                ),
            ),
        ]
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            simulator_group,
        ]
    )
