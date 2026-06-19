#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('mg_slam')

    input_dir_arg = DeclareLaunchArgument(
        'input_dir',
        description='Directory containing input pose_graph.json and gnss_transform.yaml'
    )
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Override path to the original ROS bag'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params', 'slam_gnss_2d.yaml'),
        description='Path to the slam parameter YAML file'
    )
    save_dir_arg = DeclareLaunchArgument(
        'save_dir',
        default_value='',
        description='Directory to save the reoptimized map (can be same as input_dir)'
    )

    reoptimize_node = Node(
        package='mg_slam',
        executable='reoptimize_node.py',
        name='reoptimize_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'input_dir': LaunchConfiguration('input_dir'),
                'bag_path': LaunchConfiguration('bag_path'),
                'save_dir': LaunchConfiguration('save_dir'),
            }
        ]
    )

    return LaunchDescription([
        input_dir_arg,
        bag_path_arg,
        params_file_arg,
        save_dir_arg,
        reoptimize_node
    ])
