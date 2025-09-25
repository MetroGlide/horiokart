from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    node = Node(
        package='horiokart_obstacle_detector_3d',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen',
        parameters=[
            {'fixed_frame': 'base_link'},
            {'sensor_frame': 'camera_link'},
            {'processing_rate': 15.0},
        ]
    )
    ld.add_action(node)
    return ld
