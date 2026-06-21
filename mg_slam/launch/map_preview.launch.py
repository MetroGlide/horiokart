#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav_dir = get_package_share_directory('mg_slam')
    
    slam_map_dir_arg = DeclareLaunchArgument(
        'slam_map_dir', 
        description='Directory containing map.yaml and gnss_transform.yaml'
    )
    
    map_yaml_file = PathJoinSubstitution([LaunchConfiguration('slam_map_dir'), 'map.yaml'])
    gnss_yaml_file = PathJoinSubstitution([LaunchConfiguration('slam_map_dir'), 'gnss_transform.yaml'])
    
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
    )
    
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_preview',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    anchor_publisher_cmd = Node(
        package='mg_slam',
        executable='anchor_publisher_node.py',
        name='anchor_publisher',
        output='screen',
        parameters=[{'gnss_transform_file': gnss_yaml_file}]
    )

    pose_graph_json_file = PathJoinSubstitution([LaunchConfiguration('slam_map_dir'), 'pose_graph.json'])

    pose_graph_preview_cmd = Node(
        package='mg_slam',
        executable='pose_graph_preview_node.py',
        name='pose_graph_preview',
        output='screen',
        parameters=[{'pose_graph_file': pose_graph_json_file}]
    )

    return LaunchDescription([
        slam_map_dir_arg,
        map_server_cmd,
        lifecycle_manager_cmd,
        anchor_publisher_cmd,
        pose_graph_preview_cmd,
    ])
