#!/usr/bin/env python3
"""シナリオテストノード単体起動ランチファイル。

Gazebo / Navigation (waypoint_sequencer 含む) / RViz2 は事前に個別起動済みであることを前提とする。

起動するノード:
  1. scenario_test_node
"""
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    scenario_file = LaunchConfiguration("scenario_file").perform(context)

    scenario_test = Node(
        package="mg_scenario_test",
        executable="scenario_test_node.py",
        parameters=[
            {
                "use_sim_time": True,
                "scenario_file": scenario_file,
            }
        ],
        output="screen",
    )

    return [scenario_test]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario_file", description="Path to scenario YAML"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
