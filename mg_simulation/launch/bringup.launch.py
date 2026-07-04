#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from mg_utils.launch_argument import LaunchArgumentCreator
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim = get_package_share_directory("mg_simulation")

    # default_world = os.path.join(pkg_sim, "worlds", "solar_farm.sdf")
    default_world = os.path.join(pkg_sim, "worlds", "warehouse.sdf")

    models_path = os.path.join(pkg_sim, "models")
    set_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_path)

    arg = LaunchArgumentCreator()
    arg.create("world", default=default_world)
    arg.create("robot_name", default="mg")
    arg.create("spawn_x", default="0.0")
    arg.create("spawn_y", default="0.0")
    arg.create("spawn_z", default="0.05")
    arg.create("spawn_yaw", default="0.0")
    arg.create("headless", default="false")
    arg.create(
        "publish_gazebo_tf",
        default=EnvironmentVariable(
            "PUBLISH_GAZEBO_TF", default_value="false"),
    )

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("mg_description"),
            "urdf",
            "mg_description.urdf.xacro",
        ]),
        " simulation:=true",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("publish_gazebo_tf")),
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
            "-string", robot_description,
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
        set_env,
        *arg.get_created_declare_launch_args(),
        robot_state_publisher,
        gz_sim_headless,
        gz_sim_gui,
        spawn_robot,
        bridge,
    ])
