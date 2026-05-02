from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('mg_diagnostics'),
        'params',
        'diagnostics.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
        ),
        Node(
            package='mg_diagnostics',
            executable='diagnostics_node.py',
            name='diagnostics_node',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
    ])
