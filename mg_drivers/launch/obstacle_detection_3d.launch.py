#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    launch_argument_creator = LaunchArgumentCreator()

    use_sim_time_arg = launch_argument_creator.create(
        "use_sim_time", default="false")

    use_sensor_data_qos_arg = launch_argument_creator.create(
        "use_sensor_data_qos", default="false")

    pkg_name = "mg_drivers"
    pkg_share = get_package_share_directory(pkg_name)

    param_file = os.path.join(pkg_share, "params", "obstacle_detection.yaml")

    node = Node(
        package=pkg_name,
        executable="obstacle_detection_3d_node",
        name="obstacle_detection_3d_node",
        output="screen",
        parameters=[
            param_file,
            {"use_sim_time": use_sim_time_arg.launch_config},
            {"use_sensor_data_qos": use_sensor_data_qos_arg.launch_config},
        ],
        remappings=[
            # ("points", "/camera/camera/depth/color/points"),
            ("points", "/rs_d435i/depth/color/points"),
            # Outputs: ~/points_obstacle, ~/cluster_markers
        ],
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            node,
        ]
    )
