#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from horiokart_drivers import launch_argument


def generate_launch_description():
    pkg_name = "horiokart_bringup"
    pkg_share = get_package_share_directory(pkg_name)

    navigation_pkg_name = "horiokart_navigation"
    navigation_pkg_share = get_package_share_directory(navigation_pkg_name)

    # Launch configurations
    launch_argument_creator = launch_argument.LaunchArgumentCreator()

    map_path_arg = launch_argument_creator.create(
        "map_path", default=EnvironmentVariable("LOCALIZATION_MAP_PATH")
    )
    simulation_arg = launch_argument_creator.create(
        "simulation", default=EnvironmentVariable("SIMULATION")
    )
    rviz_arg = launch_argument_creator.create(
        "rviz", default=EnvironmentVariable("USE_RVIZ")
    )

    launch_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share + "/launch/common/bringup_common.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
            "drive": "false",
        }.items(),
    )

    # map_path = PathJoinSubstitution(
    #     ["/root/ros2_data", map_path_arg.launch_config, "map.yaml"])
    map_path = map_path_arg.launch_config
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            navigation_pkg_share + "/launch/bringup.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
            "map": map_path,
            "rviz": rviz_arg.launch_config,
        }.items(),
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),

            launch_common,
            launch_navigation,
        ]
    )
