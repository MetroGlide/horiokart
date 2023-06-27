#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from horiokart_drivers import launch_argument


def generate_launch_description():
    pkg_name = "horiokart_bringup"
    pkg_share = get_package_share_directory(pkg_name)

    slam_pkg_name = "horiokart_slam"
    slam_pkg_share = get_package_share_directory(slam_pkg_name)

    # Launch configurations
    launch_argument_creator = launch_argument.LaunchArgumentCreator()

    simulation_arg = launch_argument_creator.create(
        "simulation", default=EnvironmentVariable("SIMULATION")
    )
    drive_arg = launch_argument_creator.create(
        "drive", default="false")
    rviz_arg = launch_argument_creator.create(
        "rviz", default=EnvironmentVariable("USE_RVIZ")
    )

    launch_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share + "/launch/common/bringup_common.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
            "drive": drive_arg.launch_config,
        }.items(),
    )

    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            slam_pkg_share + "/launch/bringup_slam_toolbox.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
            "rviz": rviz_arg.launch_config,
        }.items(),
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),

            launch_common,
            launch_slam,
        ]
    )
