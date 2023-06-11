#!/usr/bin/env python3

import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from horiokart_drivers.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    launch_argument_creator = LaunchArgumentCreator()

    # Launch arguments
    device_name_arg = launch_argument_creator.create(
        "device_name", default="/dev/ttyHoriokart-motordriver")

    pkg_name = "horiokart_drivers"
    pkg_share = get_package_share_directory(pkg_name)

    # Launch action group with ifconditions
    hardware_group = launch.actions.GroupAction(
        [
            Node(
                package=pkg_name,
                executable="motor_driver_node",
                name="motor_driver_node",
                output="screen",
                remappings=[
                    ("cmd_vel", "cmd_vel"),
                ],
                parameters=[{
                    "motor_driver.device_name": device_name_arg.launch_config,
                    "motor_driver.wheel_pitch": 0.358,  # m
                    "motor_driver.max_speed": 0.4,  # m/s
                }],
            ),
        ]
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            hardware_group,
        ]
    )
