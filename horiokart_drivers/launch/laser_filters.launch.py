#!/usr/bin/env python3

import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from horiokart_drivers.launch_argument import LaunchArgumentCreator


def generate_launch_description():

    launch_argument_creator = LaunchArgumentCreator()

    # Launch arguments
    front_laser_filter_yaml = launch_argument_creator.create(
        "front_laser_filter_yaml", default="front_laser_filter.yaml")
    top_laser_filter_yaml = launch_argument_creator.create(
        "top_laser_filter_yaml", default="top_laser_filter.yaml")

    pkg_name = "horiokart_drivers"
    pkg_share = get_package_share_directory(pkg_name)
    param_dir = "param"

    front_laser_filter_yaml_path = PathJoinSubstitution([
        pkg_share, param_dir, front_laser_filter_yaml.launch_config])
    front_laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="front_laser_filter",
        output="screen",
        parameters=[front_laser_filter_yaml_path],
        remappings=[
            ("scan", "/scan_front_lidar"),
            ("scan_filtered", "/scan_filtered/scan_front_lidar"),
        ],
    )

    top_laser_filter_yaml_path = PathJoinSubstitution([
        pkg_share, param_dir, top_laser_filter_yaml.launch_config])
    top_laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="top_laser_filter",
        output="screen",
        parameters=[top_laser_filter_yaml_path],
        remappings=[
            ("scan", "/scan_top_lidar"),
            ("scan_filtered", "/scan_filtered/scan_top_lidar"),
        ],
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            front_laser_filter_node,
            top_laser_filter_node,
        ]
    )
