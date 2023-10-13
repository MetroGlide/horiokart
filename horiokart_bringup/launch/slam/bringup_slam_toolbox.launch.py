#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from horiokart_drivers import launch_argument


def generate_launch_description():
    pkg_name = "horiokart_bringup"
    pkg_share = get_package_share_directory(pkg_name)

    drivers_pkg_name = "horiokart_drivers"
    drivers_pkg_share = get_package_share_directory(drivers_pkg_name)

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

    record_bag_arg = launch_argument_creator.create(
        "record_bag", default="false")

    use_ekf_arg = launch_argument_creator.create(
        "use_ekf", default="False")
    ekf_params_file_arg = launch_argument_creator.create(
        "ekf_params_file", default="ekf_slam.yaml")
    ekf_odom_topic_arg = launch_argument_creator.create(
        "ekf_odom_topic", default="ekf_odom")

    use_odom_arg = launch_argument_creator.create(
        "use_odom", default="true")
    use_lidar_arg = launch_argument_creator.create(
        "use_lidar", default="true")
    use_gps_arg = launch_argument_creator.create(
        "use_gps", default="true")

    launch_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share + "/launch/common/bringup_common.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
            "drive": drive_arg.launch_config,
            "use_odom": use_odom_arg.launch_config,
            "use_odom_tf": launch.substitutions.PythonExpression(
                ["not ", use_ekf_arg.launch_config]),
            "use_lidar": use_lidar_arg.launch_config,
            "use_gps": use_gps_arg.launch_config,
        }.items(),
    )

    ekf_group = launch.actions.GroupAction(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_slam_node",
                output="screen",
                parameters=[
                    {"use_sim_time": simulation_arg.launch_config},
                    PathJoinSubstitution(
                        [drivers_pkg_share, "params",
                            ekf_params_file_arg.launch_config]
                    ),
                ],
                remappings=[
                    ("odometry/filtered", ekf_odom_topic_arg.launch_config),
                ],
            ),

        ],
        condition=launch.conditions.IfCondition(
            use_ekf_arg.launch_config),
    )

    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            slam_pkg_share + "/launch/bringup_slam_toolbox.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
            "rviz": rviz_arg.launch_config,
            "record_bag": record_bag_arg.launch_config,
        }.items(),
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),

            launch_common,
            ekf_group,
            launch_slam,
        ]
    )
