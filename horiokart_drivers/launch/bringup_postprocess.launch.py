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
    simulation_arg = launch_argument_creator.create(
        "simulation", default="false")
    use_odom_arg = launch_argument_creator.create(
        "use_odom", default="true")
    use_lidar_arg = launch_argument_creator.create(
        "use_lidar", default="true")
    use_gps_arg = launch_argument_creator.create(
        "use_gps", default="false")
    use_ekf_arg = launch_argument_creator.create(
        "use_ekf", default="false")

    pkg_name = "horiokart_drivers"
    pkg_share = get_package_share_directory(pkg_name)

    sensors_processing_group = launch.actions.GroupAction(
        [
            # Wheel odometry tf broadcaster
            Node(
                package=pkg_name,
                executable="odometry_tf_broadcaster_node.py",
                name="odometry_tf_broadcaster_node",
                output="screen",
                parameters=[{
                    "odom_frame_id": "odom",
                    "child_frame_id": "base_footprint",
                    "use_sim_time": simulation_arg.launch_config,
                }],
                remappings=[("odom", "odom")],
                condition=launch.conditions.IfCondition(
                    use_odom_arg.launch_config),
            ),

            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    pkg_share + "/launch/laser_filters.launch.py"
                ),
                condition=launch.conditions.IfCondition(
                    use_lidar_arg.launch_config),
            ),

            Node(
                package="nmea_navsat_driver",
                executable="nmea_topic_driver",
                name="nmea_topic_driver",
                output="screen",
                parameters=[{
                    "time_ref_source": "gps",
                    "use_sim_time": simulation_arg.launch_config,
                }],
                remappings=[
                    ("nmea_sentence", "gps/nmea_sentence"),
                    ("fix", "gps/fix"),
                ],
                condition=launch.conditions.IfCondition(
                    use_gps_arg.launch_config),
            ),
        ]
    )

    ekf_group = launch.actions.GroupAction(
        [
            Node(
                package="robot_localization",
                executable="ekf_localization_node",
                name="ekf_localization_node",
                output="screen",
                parameters=[
                    {"use_sim_time": simulation_arg.launch_config},
                    pkg_share + "/config/ekf_odom_params.yaml",
                ],
                remappings=[
                    ("odometry/filtered", "ekf_odom"),
                ],
            ),

            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[
                    {"use_sim_time": simulation_arg.launch_config},
                    pkg_share + "/config/ekf_gps_params.yaml",
                ],
                remappings=[
                    ("odometry/filtered", "ekf_odom"),
                    ("gps/fix", "gps/fix"),
                ],
            )
        ],
        condition=launch.conditions.IfCondition(
            use_ekf_arg.launch_config),
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            sensors_processing_group,
            ekf_group,
        ]
    )
