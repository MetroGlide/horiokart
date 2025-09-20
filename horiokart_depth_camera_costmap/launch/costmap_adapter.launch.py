from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    adapter = Node(
        package='horiokart_depth_camera_costmap',
        executable='costmap_adapter_node',
        name='costmap_adapter',
        output='screen',
        parameters=[
            {'depth_input_topic': '/depth_camera/occupancy_grid'},
            {'master_input_topic': '/master/costmap'},
            {'output_topic': '/merged/costmap'},
            {'conditional_overwrite': True},
            {'overwrite_if_more_lethal': True}
        ]
    )

    ld.add_action(adapter)
    return ld
