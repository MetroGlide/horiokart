import os

import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node, LifecycleNode, PushRosNamespace
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml, ReplaceString

import lifecycle_msgs.msg

from horiokart_navigation.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    # Getting directories and launch-files
    pkg_dir = get_package_share_directory('horiokart_navigation')
    pkg_launch_dir = os.path.join(pkg_dir, 'launch')

    rviz_config_dir = os.path.join(
        pkg_dir, 'rviz', 'rviz.rviz')

    # Launch arguments
    launch_argument_creator = LaunchArgumentCreator()

    use_collision_behavior_arg = launch_argument_creator.create(
        'use_collision_behavior', default="true")
    use_waypoints_follower_arg = launch_argument_creator.create(
        'use_waypoints_follower', default="true")

    namespace_arg = launch_argument_creator.create(
        'namespace', default='')
    use_namespace_arg = launch_argument_creator.create(
        'use_namespace', default="false")
    autostart_arg = launch_argument_creator.create(
        'autostart', default="true")
    use_composition_arg = launch_argument_creator.create(
        'use_composition', default="True")
    use_respawn_arg = launch_argument_creator.create(
        'use_respawn', default="false")
    log_level_arg = launch_argument_creator.create(
        'log_level', default="info")

    simulation_arg = launch_argument_creator.create(
        'simulation', default=EnvironmentVariable('SIMULATION'))
    rviz_arg = launch_argument_creator.create(
        'rviz', default=EnvironmentVariable("USE_RVIZ")
    )
    record_bag_arg = launch_argument_creator.create(
        'record_bag', default="false")

    localization_map_yamL_file_arg = launch_argument_creator.create(
        'localization_map', default=os.path.join(pkg_dir, 'map', 'map.yaml'))
    planning_map_yaml_file_arg = launch_argument_creator.create(
        'planning_map', default=os.path.join(pkg_dir, 'map', 'map.yaml'))

    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            pkg_dir, 'params', 'nav2_params.yaml'))

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': simulation_arg.launch_config,
        'yaml_filename': localization_map_yamL_file_arg.launch_config, }

    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace_arg.launch_config)},
        condition=IfCondition(use_namespace_arg.launch_config))

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace_arg.launch_config,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    record_bag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'record_bag.launch.py')
        ),
        condition=IfCondition(record_bag_arg.launch_config),
    )

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace_arg.launch_config),
            namespace=namespace_arg.launch_config),

        # Composer
        Node(
            condition=IfCondition(use_composition_arg.launch_config),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {
                'autostart': autostart_arg.launch_config}],
            arguments=['--ros-args', '--log-level',
                       log_level_arg.launch_config],
            remappings=remappings,
            output='screen'
        ),

        # Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_launch_dir,
                                                       'localization_launch.py')),
            launch_arguments={'namespace': namespace_arg.launch_config,
                              'map': localization_map_yamL_file_arg.launch_config,
                              'use_sim_time': simulation_arg.launch_config,
                              'autostart': autostart_arg.launch_config,
                              'params_file': params_file,
                              'use_composition': use_composition_arg.launch_config,
                              'use_respawn': use_respawn_arg.launch_config,
                              'container_name': 'nav2_container'}.items()
        ),

        # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace_arg.launch_config,
                              'use_sim_time': simulation_arg.launch_config,
                              'autostart': autostart_arg.launch_config,
                              'params_file': params_file,
                              'use_composition': use_composition_arg.launch_config,
                              'use_respawn': use_respawn_arg.launch_config,
                              'planning_map': planning_map_yaml_file_arg.launch_config,
                              'container_name': 'nav2_container'}.items()
        ),

        # Collision Avoidance Node
        Node(
            package='horiokart_navigation',
            executable='collision_behavior_node.py',
            name='collision_behavior_node',
            parameters=[configured_params],
            output='screen',
            condition=IfCondition(use_collision_behavior_arg.launch_config),
        ),

        # Waypoint Follower
        Node(
            package='horiokart_navigation',
            executable='waypoints_follower.py',
            name='waypoints_follower_node',
            parameters=[configured_params],
            output='screen',
            condition=IfCondition(use_waypoints_follower_arg.launch_config),
        ),

    ])

    return LaunchDescription([
        *launch_argument_creator.get_created_declare_launch_args(),
        bringup_cmd_group,

        record_bag_launch,

        # Rviz
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
