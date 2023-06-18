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
    simulation_arg = launch_argument_creator.create(
        "simulation", default="false")
    use_resource_pub_arg = launch_argument_creator.create(
        "use_resource_pub", default="true")

    pkg_name = "horiokart_drivers"
    pkg_share = get_package_share_directory(pkg_name)

    # Launch action group with ifconditions
    common_group = launch.actions.GroupAction(
        [
            Node(
                package=pkg_name,
                executable="pc_resource_publisher_node.py",
                name="pc_resource_publisher_node",
                output="screen",
                remappings=[
                    ("cpu_usage", "cpu_usage"),
                ],
                parameters=[{
                    "publish_rate": 1.0,
                    "cpu_interval": 0.5,
                    "use_sim_time": simulation_arg.launch_config,
                }],
                condition=launch.conditions.IfCondition(
                    use_resource_pub_arg.launch_config),
            ),
        ]
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            common_group,
        ]
    )
