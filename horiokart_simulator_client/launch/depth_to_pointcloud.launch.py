#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    return LaunchDescription([
        # launch plugin through rclcpp_components container
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('rgb/camera_info', '/rs_d435/color/camera_info'),
                                ('rgb/image_rect_color', '/rs_d435/color/image_raw'),
                                ('depth_registered/image_rect',
                                 '/rs_d435/aligned_depth_to_color/image_raw'),

                                ('points', '/rs_d435/depth_registered/points')]
                ),
            ],
            output='screen',
        ),
    ])
