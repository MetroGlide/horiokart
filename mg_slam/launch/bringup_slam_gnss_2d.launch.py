#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    pkg_dir = get_package_share_directory('mg_slam')

    launch_argument_creator = LaunchArgumentCreator()

    simulation_arg = launch_argument_creator.create(
        'simulation', default=EnvironmentVariable('SIMULATION'))
    launch_rviz_arg = launch_argument_creator.create(
        'rviz', default=EnvironmentVariable('USE_RVIZ'))
    rviz_param_arg = launch_argument_creator.create(
        'rviz_param', default='slam_gnss_2d.rviz')
    params_file_arg = launch_argument_creator.create(
        'params_file',
        default=os.path.join(pkg_dir, 'params', 'slam_gnss_2d.yaml'))

    slam_node = Node(
        package='mg_slam',
        executable='slam_node.py',
        name='slam_gnss_2d_node',
        output='screen',
        parameters=[
            {'use_sim_time': simulation_arg.launch_config},
            params_file_arg.launch_config,
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [pkg_dir, 'rviz', rviz_param_arg.launch_config])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(launch_rviz_arg.launch_config),
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),

            slam_node,
            rviz_node,
        ]
    )
