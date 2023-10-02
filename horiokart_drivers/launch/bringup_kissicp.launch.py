#!/usr/bin/env python3

import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from horiokart_drivers.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    launch_argument_creator = LaunchArgumentCreator()

    # Launch arguments
    use_odom_arg = launch_argument_creator.create(
        "use_odom", default="true")

    kissicp_pc_topic = launch_argument_creator.create(
        "pc_topic", default="/cloud_top_lidar")
    kissicp_odom_topic = launch_argument_creator.create(
        "odom_topic", default="/kissicp/odom")
        # "odom_topic", default="/odom")
    kissicp_visualize = launch_argument_creator.create(
        "visualize", default="true")
    kissicp_odom_frame = launch_argument_creator.create(
        "odom_frame", default="odom")
    kissicp_child_frame = launch_argument_creator.create(
        "child_frame", default="base_footprint")
    kissicp_deskew = launch_argument_creator.create(
        "deskew", default="false")
    kissicp_max_range = launch_argument_creator.create(
        "max_range", default="40.0")
    kissicp_min_range = launch_argument_creator.create(
        "min_range", default="5.0")

    kissicp_remove_parcentile_rate = launch_argument_creator.create(
        "remove_parcentile_rate", default="0.15")

    kissicp_publish_odom_tf = launch_argument_creator.create(
        "publish_odom_tf", default="false")
    kissicp_publish_alias_tf = launch_argument_creator.create(
        "publish_alias_tf", default="false")

    pkg_name = "horiokart_drivers"
    pkg_share = get_package_share_directory(pkg_name)

    # Launch action group with ifconditions
    node_group = launch.actions.GroupAction(
        [
            Node(
                package="pointcloud_to_laserscan",
                executable="laserscan_to_pointcloud_node",
                name="laserscan_to_pointcloud_node",
                output="screen",
                parameters=[{
                    "target_frame": "base_footprint",
                    "transform_tolerance": 0.01,
                }],
                remappings=[
                    ("scan_in", "/scan_top_lidar"),
                    ("cloud", "/cloud_top_lidar"),
                ],
            ),
            # launch.actions.IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         get_package_share_directory(
            #             "kiss_icp") + "/launch/odometry.launch.py"
            #     ),
            #     launch_arguments={
            #         "topic": "/cloud_top_lidar",
            #         "visualize": "true",
            #         "odom_frame": "odom",
            #         "child_frame": "base_footprint",
            #         "publish_odom_tf": "true",
            #         "publish_alias_tf": "true",

            #         "deskew": "false",
            #         "max_range": "40.0",
            #         "min_range": "5.0",
            #     }.items(),
            # ),
            Node(
                package="kiss_icp",
                executable="odometry_node",
                name="odometry_node",
                output="screen",
                remappings=[
                    ("pointcloud_topic", kissicp_pc_topic.launch_config),
                    ("odometry", "kissicp/odom_no_covariance"),
                ],
                parameters=[
                    {
                        "odom_frame": kissicp_odom_frame.launch_config,
                        "child_frame": kissicp_child_frame.launch_config,
                        "max_range": kissicp_max_range.launch_config,
                        "min_range": kissicp_min_range.launch_config,
                        "deskew": kissicp_deskew.launch_config,
                        # "max_points_per_voxel": 20,
                        "max_points_per_voxel": 50,
                        "initial_threshold": 2.0,
                        "min_motion_th": 0.1,
                        "publish_odom_tf": kissicp_publish_odom_tf.launch_config,
                        "publish_alias_tf": kissicp_publish_alias_tf.launch_config,
                        "remove_parcentile_rate": kissicp_remove_parcentile_rate.launch_config,
                    }
                ],
            ),
            Node(
                package=pkg_name,
                executable="odom_covariance_override_node.py",
                name="odom_covariance_override_node",
                output="screen",
                parameters=[{
                }],
                remappings=[
                    ("odom", "kissicp/odom_no_covariance"),
                    ("odom/covariance", kissicp_odom_topic.launch_config),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output={"both": "log"},
                arguments=[
                    "-d", launch.substitutions.PathJoinSubstitution([get_package_share_directory("kiss_icp"), "rviz", "kiss_icp_ros2.rviz"])],
                condition=launch.conditions.IfCondition(
                    LaunchConfiguration("visualize")),
            ),
        ],
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            node_group,
        ]
    )
