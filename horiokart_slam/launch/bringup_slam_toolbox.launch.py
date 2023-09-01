import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from nav2_common.launch import HasNodeParams, RewrittenYaml

from horiokart_slam.launch_argument import LaunchArgumentCreator


def generate_launch_description():

    # Getting directories and launch-files
    pkg_dir = get_package_share_directory('horiokart_slam')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(
        slam_toolbox_dir, 'launch', 'online_sync_launch.py')

    # Launch arguments
    launch_argument_creator = LaunchArgumentCreator()

    slam_param_file_arg = launch_argument_creator.create(
        'slam_toolbox_params_file',
        default=os.path.join(pkg_dir, 'params', 'slam_toolbox.yaml'),
    )
    simulation_arg = launch_argument_creator.create(
        'simulation', default=EnvironmentVariable('SIMULATION'))
    log_level_arg = launch_argument_creator.create(
        'log_level', default='info')

    launch_rviz_arg = launch_argument_creator.create(
        'rviz', default=EnvironmentVariable("USE_RVIZ"))
    rviz_param_arg = launch_argument_creator.create(
        'rviz_param', default='rviz.rviz')

    # Variables
    lifecycle_nodes = ['map_saver']

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': simulation_arg.launch_config}

    configured_params = RewrittenYaml(
        source_file=slam_param_file_arg.launch_config,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Nodes launching commands
    start_map_saver_server_cmd = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=['--ros-args', '--log-level', log_level_arg.launch_config],
        parameters=[{
            'use_sim_time': simulation_arg.launch_config,
            'save_map_timeout': 0.5,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
            'map_subscribe_transient_local': True,
        }])

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level_arg.launch_config],
        parameters=[{'use_sim_time': simulation_arg.launch_config},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}])

    has_slam_toolbox_params = HasNodeParams(source_file=slam_param_file_arg.launch_config,
                                            node_name='slam_toolbox')

    start_slam_toolbox_cmd_with_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': simulation_arg.launch_config,
                          'slam_params_file': slam_param_file_arg.launch_config,
                          'log_level': log_level_arg.launch_config}.items(),
    )

    actual_path_publisher_node = Node(
        package='horiokart_slam',
        executable='actual_path_publisher.py',
        output='screen',
    )

    # Launch rviz2
    rviz_config_file = PathJoinSubstitution(
        [pkg_dir, 'rviz', rviz_param_arg.launch_config])
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(launch_rviz_arg.launch_config),
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),

            # Running Map Saver Server
            start_map_saver_server_cmd,
            start_lifecycle_manager_cmd,

            # Running SLAM Toolbox (Only one of them will be run)
            start_slam_toolbox_cmd_with_params,

            # Running actual_path_publisher.py for debug
            actual_path_publisher_node,

            # Running rviz2
            start_rviz_cmd,
        ]
    )
