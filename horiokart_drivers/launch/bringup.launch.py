#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from horiokart_drivers import launch_argument


def generate_launch_description():
    launch_argument_creator = launch_argument.LaunchArgumentCreator()

    # Launch configurations
    simulation_arg = launch_argument_creator.create(
        "simulation", default=EnvironmentVariable("SIMULATION")
    )
    drive_arg = launch_argument_creator.create(
        "drive", default="false")

    pkg_name = "horiokart_drivers"
    pkg_share = get_package_share_directory(pkg_name)

    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share + "/launch/bringup_sensors.launch.py"
        ),
        condition=launch.conditions.UnlessCondition(
            simulation_arg.launch_config)
    )

    launch_postprocess = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share + "/launch/bringup_postprocess.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
        }.items(),
    )

    launch_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share + "/launch/bringup_common.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
        }.items(),
    )

    launch_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share + "/launch/bringup_hardware.launch.py"
        ),
        condition=launch.conditions.IfCondition(drive_arg.launch_config)
        and launch.conditions.UnlessCondition(simulation_arg.launch_config)
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            launch_sensors,
            launch_postprocess,
            launch_common,
            launch_drive,
        ]
    )
