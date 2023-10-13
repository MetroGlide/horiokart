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
    pkg_dir = get_package_share_directory('horiokart_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file_dir = os.path.join(nav2_bringup_dir, 'launch')

    rviz_config_dir = os.path.join(
        pkg_dir, 'rviz', 'rviz.rviz')
    # nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # Launch arguments
    launch_argument_creator = LaunchArgumentCreator()

    simulation_arg = launch_argument_creator.create(
        'simulation', default=EnvironmentVariable('SIMULATION'))
    rviz_arg = launch_argument_creator.create(
        'rviz', default=EnvironmentVariable("USE_RVIZ")
    )
    record_bag_arg = launch_argument_creator.create(
        'record_bag', default="false")

    map_dir_arg = launch_argument_creator.create(
        'map', default=os.path.join(pkg_dir, 'map', 'map.yaml'))
    planning_map_dir_arg = launch_argument_creator.create(
        'planning_map', default=os.path.join(pkg_dir, 'map', 'map.yaml'))

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            pkg_dir, 'params', 'nav2_params.yaml'))

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='planning_map_server',
        namespace='',
        output='screen',
        parameters=[
            {
                'use_sim_time': simulation_arg.launch_config,
                'yaml_filename': planning_map_dir_arg.launch_config,
                # 'topic_name': 'planning_map',
            },
        ],
        remappings=[('/map', '/planning_map')]
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

    costmap_filter_info_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[param_dir],
    )

    costmap_filter_info_group = launch.actions.GroupAction([
        costmap_filter_info_server_node,
        EmitEvent(
            event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(
                    costmap_filter_info_server_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )),
        launch.actions.RegisterEventHandler(  # イベントハンドラの登録
            launch_ros.event_handlers.OnStateTransition(  # lifecycle_nodeが状態遷移したときのイベント
                target_lifecycle_node=costmap_filter_info_server_node,  # ターゲットノード
                start_state="configuring", goal_state="inactive",  # どの状態からどの状態へ遷移したかを書く
                entities=[
                    launch.actions.LogInfo(
                        msg="transition start : costmap_filter_info_server :activating"),
                    launch.actions.EmitEvent(
                        event=launch_ros.events.lifecycle.ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(
                                costmap_filter_info_server_node),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                        )),
                ],
            )
        ),
    ])

    record_bag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'record_bag.launch.py')
        ),
        condition=IfCondition(record_bag_arg.launch_config),
    )

    return LaunchDescription([
        *launch_argument_creator.get_created_declare_launch_args(),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir_arg.launch_config,
                'use_sim_time': simulation_arg.launch_config,
                'params_file': param_dir}.items(),
        ),
        map_server_group,
        costmap_filter_info_group,

        record_bag_launch,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': simulation_arg.launch_config}],
            # output='screen',
            condition=IfCondition(rviz_arg.launch_config),
        ),
    ])
