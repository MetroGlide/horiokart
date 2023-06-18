#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from horiokart_drivers import launch_argument


def generate_launch_description():
    pkg_name = "horiokart_bringup"
    pkg_share = get_package_share_directory(pkg_name)

    drivers_pkg_name = "horiokart_drivers"
    drivers_pkg_share = get_package_share_directory(drivers_pkg_name)

    description_pkg_name = "horiokart_description"
    description_pkg_share = get_package_share_directory(description_pkg_name)

    # Launch configurations
    launch_argument_creator = launch_argument.LaunchArgumentCreator()

    simulation_arg = launch_argument_creator.create(
        "simulation", default="false")
    drive_arg = launch_argument_creator.create(
        "drive", default="false")

    launch_drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            drivers_pkg_share + "/launch/bringup.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
            "drive": drive_arg.launch_config,
        }.items(),
    )

    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            description_pkg_share + "/launch/bringup.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
        }.items(),
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),

            launch_drivers,
            launch_description,
        ]
    )
