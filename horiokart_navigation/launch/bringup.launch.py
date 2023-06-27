import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node

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
    map_dir_arg = launch_argument_creator.create(
        'map', default=os.path.join(pkg_dir, 'map', 'map.yaml'))
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            pkg_dir, 'params', 'nav2_params.yaml'))

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
