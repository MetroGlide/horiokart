#!/usr/bin/env python3
"""シナリオテスト一括起動ランチファイル。

起動順序:
  1. mg_simulation/bringup.launch.py  (Gazebo Fortress)
  2. mg_bringup/navigation/bringup_navigation.launch.py
  3. waypoint_sequencer_node  (waypoints_file モード時のみ使用)
  4. scenario_test_node
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_waypoints_yaml(goals: list) -> str:
    """goals リスト → waypoints v2.0 YAML を一時ファイルに書き込む。"""
    import math

    waypoints = []
    for i, g in enumerate(goals):
        yaw = float(g.get("yaw", 0.0))
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        waypoints.append(
            {
                "index": i,
                "pose": {
                    "position": {
                        "x": float(g.get("x", 0.0)),
                        "y": float(g.get("y", 0.0)),
                        "z": float(g.get("z", 0.0)),
                    },
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                },
                "on_reached_actions": [],
            }
        )

    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".yaml",
        prefix="mg_scenario_test_wp_",
        delete=False,
    )
    yaml.safe_dump(
        {"version": "2.0", "waypoints": waypoints},
        tmp,
        allow_unicode=True,
        default_flow_style=False,
        sort_keys=False,
    )
    tmp.close()
    return tmp.name


def _launch_setup(context, *args, **kwargs):
    scenario_file = LaunchConfiguration("scenario_file").perform(context)
    headless = LaunchConfiguration("headless").perform(context)

    with open(scenario_file, "r") as f:
        scenario_raw = yaml.safe_load(f)

    world_name = scenario_raw.get("world_name", "warehouse")
    pkg_sim = get_package_share_directory("mg_simulation")
    world_path = os.path.join(pkg_sim, "worlds", f"{world_name}.sdf")

    if "goals" in scenario_raw:
        waypoints_path = _build_waypoints_yaml(
            [g["pose"] for g in scenario_raw["goals"]]
        )
    else:
        waypoints_path = scenario_raw["waypoints_file"]

    pkg_bringup = get_package_share_directory("mg_bringup")
    pkg_waypoint_nav = get_package_share_directory("mg_waypoint_navigation")
    pkg_scenario = get_package_share_directory("mg_scenario_test")

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "headless": headless,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "navigation",
                         "bringup_navigation.launch.py")
        ),
        launch_arguments={
            "simulation": "true",
        }.items(),
    )

    waypoint_sequencer = Node(
        package="mg_waypoint_navigation",
        executable="waypoint_sequencer_node.py",
        parameters=[
            {
                "use_sim_time": True,
                "load_path": waypoints_path,
                "publish_waypoint_status": False,
            }
        ],
        output="screen",
    )

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

    return [gz_launch, nav_launch, waypoint_sequencer, scenario_test]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario_file", description="Path to scenario YAML"),
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
