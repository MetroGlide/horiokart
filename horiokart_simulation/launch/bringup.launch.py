#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim = get_package_share_directory("horiokart_simulation")

    default_world = os.path.join(pkg_sim, "worlds", "warehouse.sdf")

    world_arg = DeclareLaunchArgument("world", default_value=default_world)
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="horiokart")
    spawn_x_arg = DeclareLaunchArgument("spawn_x", default_value="0.0")
    spawn_y_arg = DeclareLaunchArgument("spawn_y", default_value="0.0")
    spawn_z_arg = DeclareLaunchArgument("spawn_z", default_value="0.05")
    spawn_yaw_arg = DeclareLaunchArgument("spawn_yaw", default_value="0.0")
    headless_arg = DeclareLaunchArgument("headless", default_value="false")

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("horiokart_description"),
            "urdf",
            "horiokart_description.urdf.xacro",
        ]),
        " simulation:=true",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}],
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [
            "-s -r ", LaunchConfiguration("world")]}.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [
            "-r ", LaunchConfiguration("world")]}.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", LaunchConfiguration("robot_name"),
            "-topic", "robot_description",
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
        ],
        output="screen",
    )

    bridge_config = os.path.join(pkg_sim, "config", "bridge.yaml")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
        output="screen",
    )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        headless_arg,
        robot_state_publisher,
        gz_sim_headless,
        gz_sim_gui,
        spawn_robot,
        bridge,
    ])
