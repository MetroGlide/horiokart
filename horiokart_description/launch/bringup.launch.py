#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "horiokart_description"

    simulation_arg = LaunchConfiguration('simulation')
    xacro_file_name = LaunchConfiguration('xacro_file_name')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "urdf", xacro_file_name]
            ),
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': simulation_arg},
        ]
    )

    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": simulation_arg,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'xacro_file_name',
            default_value="horiokart_description.urdf.xacro",
            description='Xacro file to use for URDF'
        ),
        robot_state_publisher_node,
        joint_state_pub_node,
    ])
