from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mg_tui',
            executable='tui_node.py',
            name='mg_tui_node',
            output='screen',
        ),
    ])
