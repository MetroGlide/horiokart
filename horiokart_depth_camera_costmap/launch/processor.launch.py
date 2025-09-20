from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    processor = Node(
        package='horiokart_depth_camera_costmap',
        executable='depth_camera_processor_node',
        name='depth_camera_processor_node',
        output='screen',
        parameters=[
            '/root/ros2_ws/src/horiokart/horiokart_depth_camera_costmap/config/depth_camera_costmap.yaml']
    )

    adapter = Node(
        package='horiokart_depth_camera_costmap',
        executable='costmap_adapter_node',
        name='costmap_adapter_node',
        output='screen',
        parameters=[
            '/root/ros2_ws/src/horiokart/horiokart_depth_camera_costmap/config/depth_camera_costmap.yaml']
    )

    ld.add_action(processor)
    ld.add_action(adapter)
    return ld
