import os

import launch
import launch_ros
import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import EnvironmentVariable

from mg_utils.launch_argument import LaunchArgumentCreator


def get_map_group_action(index, map_yaml_path, simulation_arg):
    map_server_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        parameters=[{"use_sim_time": simulation_arg,
                     "yaml_filename": map_yaml_path}],
        remappings=[("map", f"map_{index}")],
        output="screen",
        name=f"map_server_{index}",
        namespace="",
    )
    return launch.actions.GroupAction(
        [
            map_server_node,
            EmitEvent(
                event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(
                        map_server_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )
            ),
            launch.actions.RegisterEventHandler(
                launch_ros.event_handlers.OnStateTransition(
                    target_lifecycle_node=map_server_node,
                    start_state="configuring",
                    goal_state="inactive",
                    entities=[
                        launch.actions.LogInfo(
                            msg="transition start : map_server :activating"
                        ),
                        launch.actions.EmitEvent(
                            event=launch_ros.events.lifecycle.ChangeState(
                                lifecycle_node_matcher=launch.events.matches_action(
                                    map_server_node
                                ),
                                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                            )
                        ),
                    ],
                )
            ),
        ]
    )


def generate_launch_description():
    pkg_name = "mg_waypoint_navigation"
    pkg_dir = get_package_share_directory(pkg_name)

    rviz_config_dir = os.path.join(pkg_dir, "rviz", "waypoint_editor.rviz")

    launch_argument_creator = LaunchArgumentCreator()

    simulation_arg = launch_argument_creator.create(
        "simulation", default="false")
    map_dir_arg = os.environ.get("MAP_PATH")
    if not map_dir_arg:
        raise RuntimeError(
            "MAP_PATH environment variable is not set. "
            "Please set MAP_PATH to the directory containing map_list.txt "
            "before launching waypoint_editor.launch.py."
        )
    load_waypoints_yaml_path = launch_argument_creator.create(
        "load_path", default="/root/ros2_data/map/waypoint.yaml"
    )
    save_waypoints_yaml_path = launch_argument_creator.create(
        "save_path", default=EnvironmentVariable("WAYPOINT_PATH")
    )

    map_list_name = "map_list.txt"
    with open(os.path.join(map_dir_arg, map_list_name), "r") as file:
        map_list = [
            line
            for line in (raw.strip() for raw in file.readlines())
            if line and not line.startswith("#")
        ]

    map_server_group = [
        get_map_group_action(
            index,
            os.path.join(map_dir_arg, map_name),
            simulation_arg.launch_config,
        )
        for index, map_name in enumerate(map_list)
    ]

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            *map_server_group,
            Node(
                package=pkg_name,
                executable="waypoint_editor_node.py",
                parameters=[
                    {
                        "use_sim_time": simulation_arg.launch_config,
                        "load_path": load_waypoints_yaml_path.launch_config,
                        "save_path": save_waypoints_yaml_path.launch_config,
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_dir],
                parameters=[{"use_sim_time": simulation_arg.launch_config}],
                output="screen",
            ),
        ]
    )
