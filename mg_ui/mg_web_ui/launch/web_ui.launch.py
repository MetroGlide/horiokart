from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('dist_dir', default_value=''),

        Node(
            package='mg_web_ui',
            executable='http_server_node.py',
            name='http_server_node',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'dist_dir': LaunchConfiguration('dist_dir'),
            }],
            output='screen',
        ),
    ])
