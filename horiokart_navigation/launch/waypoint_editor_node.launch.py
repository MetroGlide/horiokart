import os

import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node, LifecycleNode

import lifecycle_msgs.msg

from horiokart_navigation.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    # Getting directories and launch-files
    pkg_name = "horiokart_navigation"
    pkg_dir = get_package_share_directory(pkg_name)

    rviz_config_dir = os.path.join(
        pkg_dir, 'rviz', 'waypoint_editor.rviz')

    # Launch arguments
    launch_argument_creator = LaunchArgumentCreator()

    simulation_arg = launch_argument_creator.create(
        'simulation', default='false')
    map_dir_arg = launch_argument_creator.create(
        'map', default=EnvironmentVariable("PLANNING_MAP_PATH")
    )

    load_waypoints_yaml_path = launch_argument_creator.create(
        'load_path', default=""
    )
    save_waypoints_yaml_path = launch_argument_creator.create(
        'save_path', default=EnvironmentVariable("WAYPOINT_PATH")
    )

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{
                'use_sim_time': simulation_arg.launch_config,
                'yaml_filename': map_dir_arg.launch_config,
        }],
        output='screen',
        name='map_server',
        namespace='',
    )

    map_server_group = launch.actions.GroupAction([
        map_server_node,
        EmitEvent(
            event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(
                    map_server_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,

            )),

        launch.actions.RegisterEventHandler(  # イベントハンドラの登録
            launch_ros.event_handlers.OnStateTransition(  # lifecycle_nodeが状態遷移したときのイベント
                target_lifecycle_node=map_server_node,  # ターゲットノード
                start_state="configuring", goal_state="inactive",  # どの状態からどの状態へ遷移したかを書く
                entities=[
                    launch.actions.LogInfo(
                        msg="transition start : map_server :activating"),
                    launch.actions.EmitEvent(
                        event=launch_ros.events.lifecycle.ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(
                                map_server_node),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                        )),
                ],
            )
        ),
    ])

    return LaunchDescription([
        *launch_argument_creator.get_created_declare_launch_args(),
        map_server_group,

        Node(
            package=pkg_name,
            executable='waypoint_editor_node.py',
            parameters=[{
                'use_sim_time': simulation_arg.launch_config,
                'load_path': load_waypoints_yaml_path.launch_config,
                'save_path': save_waypoints_yaml_path.launch_config,
            }],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': simulation_arg.launch_config}],
            output='screen',
        ),
    ])
