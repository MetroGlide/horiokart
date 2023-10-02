#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from horiokart_drivers import launch_argument


def generate_launch_description():
    pkg_name = "horiokart_bringup"
    pkg_share = get_package_share_directory(pkg_name)

    drivers_pkg_name = "horiokart_drivers"
    drivers_pkg_share = get_package_share_directory(drivers_pkg_name)

    navigation_pkg_name = "horiokart_navigation"
    navigation_pkg_share = get_package_share_directory(navigation_pkg_name)

    # Launch configurations
    launch_argument_creator = launch_argument.LaunchArgumentCreator()

    map_path_arg = launch_argument_creator.create(
        "map_path", default=EnvironmentVariable("LOCALIZATION_MAP_PATH")
    )
    planning_map_path_arg = launch_argument_creator.create(
        "planning_map_path", default=EnvironmentVariable("PLANNING_MAP_PATH")
    )
    simulation_arg = launch_argument_creator.create(
        "simulation", default=EnvironmentVariable("SIMULATION")
    )
    rviz_arg = launch_argument_creator.create(
        "rviz", default=EnvironmentVariable("USE_RVIZ")
    )

    use_ekf_arg = launch_argument_creator.create(
        "use_ekf", default="False")
    ekf_params_file_arg = launch_argument_creator.create(
        "ekf_params_file", default="ekf_global.yaml")
    ekf_odom_topic_arg = launch_argument_creator.create(
        "ekf_odom_topic", default="ekf_global_odom")

    use_odom_arg = launch_argument_creator.create(
        "use_odom", default="true")
    use_lidar_arg = launch_argument_creator.create(
        "use_lidar", default="true")
    use_gps_arg = launch_argument_creator.create(
        "use_gps", default="true")

    # Launch descriptions
    launch_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_share + "/launch/common/bringup_common.launch.py"
        ),
        launch_arguments={
            "simulation": simulation_arg.launch_config,
            "drive": "false",
            "use_odom": use_odom_arg.launch_config,
            "use_odom_tf": "true",
            "use_lidar": use_lidar_arg.launch_config,
            "use_gps": use_gps_arg.launch_config,
        }.items(),
    )

    ekf_group = launch.actions.GroupAction(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_global_node",
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
            "planning_map": planning_map_path_arg.launch_config,
            "rviz": rviz_arg.launch_config,
        }.items(),
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),

            launch_common,
            ekf_group,
            launch_navigation,
        ]
    )
