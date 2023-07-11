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
    launch_argument_creator = LaunchArgumentCreator()

    # Launch arguments
    use_rs_d435_arg = launch_argument_creator.create(
        "use_rs_d435", default="false")
    use_rs_d435i_arg = launch_argument_creator.create(
        "use_rs_d435i", default="true")

    rs_pkg_name = "realse2_camera"
    rs_pkg_share = get_package_share_directory(rs_pkg_name)

    # Launch action group with ifconditions
    sensors_group = launch.actions.GroupAction(
        [
            # RealSense D435
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        rs_pkg_share + "/launch/rs_launch.py"
                ),
                namespace="rs_d435",
                launch_arguments={
                    "serial_no": "",
                    "align_depth.enable": True,
                    "pointcloud.enable": True,
                }.items(),
                condition=launch.conditions.IfCondition(
                    use_rs_d435_arg.launch_config),
            ),

            # RealSense D435i
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        rs_pkg_share + "/launch/rs_launch.py"
                ),
                namespace="rs_d435i",
                launch_arguments={
                    "serial_no": "",
                    "align_depth.enable": True,
                    "pointcloud.enable": True,
                    "enable_gyro": True,
                    "enable_accel": True,
                    "unite_imu_method": 1,
                }.items(),
                condition=launch.conditions.IfCondition(
                    use_rs_d435i_arg.launch_config),
            ),
        ],
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            sensors_group,
        ]
    )
