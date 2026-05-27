#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mg_utils.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    pkg_dir = get_package_share_directory('mg_slam')

    launch_argument_creator = LaunchArgumentCreator()

    bag_path_arg = launch_argument_creator.create(
        'bag_path', default=EnvironmentVariable('ROSBAG_FILE'))
    launch_rviz_arg = launch_argument_creator.create('rviz', default='false')
    rviz_param_arg = launch_argument_creator.create(
        'rviz_param', default='slam_gnss_2d.rviz')
    params_file_arg = launch_argument_creator.create(
        'params_file',
        default=os.path.join(pkg_dir, 'params', 'slam_gnss_2d.yaml'))

    mg_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    'mg_description'), 'launch', 'bringup.launch.py'
            )
        ),
        launch_arguments={'simulation': 'false'}.items(),
    )

    bringup_postprocess_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    'mg_drivers'), 'launch', 'bringup_postprocess.launch.py'
            )
        ),
        launch_arguments={
            'use_lidar': 'false',
            'use_gps': 'false',
            'use_realsense': 'false',
        }.items(),
    )

    offline_node = Node(
        package='mg_slam',
        executable='slam_offline_node.py',
        name='slam_gnss_2d_offline_node',
        output='screen',
        parameters=[
            params_file_arg.launch_config,
            {'bag_path': bag_path_arg.launch_config},
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

            mg_description_launch,
            bringup_postprocess_launch,
            offline_node,
            rviz_node,
        ]
    )
