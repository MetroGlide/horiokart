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

    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_file_name = LaunchConfiguration('xacro_file_name')


    # xacro_file_path = os.path.join(
    #     get_package_share_directory(package_name),
    #     'urdf',
    #     xacro_file_name)

    xacro_file_path = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'urdf',
        xacro_file_name,
    ])

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
        #{
        # 'robot_description': ParameterValue(
        #     Command(['xacro ', xacro_file_path]), value_type=str
        # )},
        {'robot_description': robot_description_content},
        # {'use_sim_time': use_sim_time},
        ]
    )

    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    joint_state_pub_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
            ),
        DeclareLaunchArgument(
            'xacro_file_name',
            default_value="horiokart_description.urdf.xacro",
            description='Xacro file to use for URDF'
            ),
        robot_state_publisher_node,
        joint_state_pub_node,
        # joint_state_pub_gui_node,
    ])
