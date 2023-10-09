#!/usr/bin/env python3

import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from horiokart_drivers.launch_argument import LaunchArgumentCreator


def generate_launch_description():
    launch_argument_creator = LaunchArgumentCreator()

    # Launch arguments
    use_odom_arg = launch_argument_creator.create(
        "use_odom", default="true")
    odom_port_arg = launch_argument_creator.create(
        "odom_port", default="/dev/ttyHoriokart-odom")

    use_lidar_arg = launch_argument_creator.create(
        "use_lidar", default="true")
    front_rplidar_port_arg = launch_argument_creator.create(
        "front_rplidar_port", default="/dev/ttyHoriokart-frontlidar"
    )
    top_rplidar_port_arg = launch_argument_creator.create(
        "top_rplidar_port", default="/dev/ttyHoriokart-toplidar"
    )

    use_gps_arg = launch_argument_creator.create(
        "use_gps", default="true")
    gps_port_arg = launch_argument_creator.create(
        "gps_port", default="/dev/ttyHoriokart-gps")

    use_rs_d435i_arg = launch_argument_creator.create(
        "use_rs_d435i", default="true")
    use_rs_d435_arg = launch_argument_creator.create(
        "use_rs_d435", default="false")

    pkg_name = "horiokart_drivers"
    pkg_share = get_package_share_directory(pkg_name)

    # Launch action group with ifconditions
    sensors_group = launch.actions.GroupAction(
        [
            # Wheel odometry
            Node(
                package=pkg_name,
                executable="wheel_odometry_node",
                name="wheel_odometry_node",
                output="screen",
                parameters=[{
                    "odometry.device_name": odom_port_arg.launch_config,

                    "odometry.frame_id": "odom",
                    "odometry.child_frame_id": "base_footprint",
                    "odometry.publish_rate": 20,
                    "odometry.publish_tf": False,

                    "odometry.inv_x": True,
                    "odometry.inv_y": True,
                    "odometry.inv_th": False,

                    "odometry.always_publish": False,
                    "odometry.error_recovery_count": 4,
                }],
                condition=launch.conditions.IfCondition(
                    use_odom_arg.launch_config),
            ),

            # RPLidar front
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="front_rplidar_node",
                output="screen",
                parameters=[{
                    "serial_port": front_rplidar_port_arg.launch_config,
                    "serial_baudrate": 115200,
                    "frame_id": "front_lrf_link",
                    "inverted": False,
                    "angle_compensate": True,
                }],
                remappings=[("scan", "scan_front_lidar")],
                condition=launch.conditions.IfCondition(
                    use_lidar_arg.launch_config),
            ),

            # RPLidar top
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="top_rplidar_node",
                output="screen",
                parameters=[{
                    "serial_port": top_rplidar_port_arg.launch_config,
                    "serial_baudrate": 1000000,
                    # "frame_id": "top_lidar_link",
                    "frame_id": "top_lrf_link",
                    "inverted": False,
                    "angle_compensate": True,
                }],
                remappings=[("scan", "scan_top_lidar")],
                condition=launch.conditions.IfCondition(
                    use_lidar_arg.launch_config),
            ),

            # GNSS
            Node(
                package="nmea_navsat_driver",
                executable="nmea_topic_serial_reader",
                name="gps_driver",
                output="screen",
                parameters=[{
                    "port": gps_port_arg.launch_config,
                    "baud": 38400,
                    "frame_id": "gps_link",
                }],
                condition=launch.conditions.IfCondition(
                    use_gps_arg.launch_config),
            ),

            # RealSense D435i
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory(
                        "realsense2_camera") + "/launch/rs_launch.py"
                ),
                launch_arguments={
                    "serial_no": "",
                    "align_depth.enable": "true",
                    "pointcloud.enable": "true",
                    "enable_gyro": "true",
                    "enable_accel": "true",
                    "unite_imu_method": "1",
                }.items(),
                condition=launch.conditions.IfCondition(
                    use_rs_d435i_arg.launch_config),
            ),
        ],
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            sensors_group,
        ]
    )
