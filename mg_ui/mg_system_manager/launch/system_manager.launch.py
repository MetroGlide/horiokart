from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'project_dir',
            default_value='/app',
        ),
        Node(
            package='mg_system_manager',
            executable='system_manager_node.py',
            name='system_manager_node',
            parameters=[{
                'project_dir': LaunchConfiguration('project_dir'),
            }],
            output='screen',
        ),
    ])
