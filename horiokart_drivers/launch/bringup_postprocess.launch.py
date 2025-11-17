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
    simulation_arg = launch_argument_creator.create(
        "simulation", default="false")
    use_odom_arg = launch_argument_creator.create(
        "use_odom", default="true")
    use_odom_tf_arg = launch_argument_creator.create(
        "use_odom_tf", default="true")
    use_realsense_arg = launch_argument_creator.create(
        "use_realsense", default="true")
    use_lidar_arg = launch_argument_creator.create(
        "use_lidar", default="true")
    use_gps_arg = launch_argument_creator.create(
        "use_gps", default="true")

    pkg_name = "horiokart_drivers"
    pkg_share = get_package_share_directory(pkg_name)

    sensors_processing_group = launch.actions.GroupAction(
        [
            # Wheel odometry tf broadcaster
            Node(
                package=pkg_name,
                executable="odometry_tf_broadcaster_node.py",
                name="odometry_tf_broadcaster_node",
                output="screen",
                parameters=[{
                    "odom_frame_id": "odom",
                    "child_frame_id": "base_footprint",
                    "use_sim_time": simulation_arg.launch_config,
                }],
                remappings=[("odom", "odom")],
                condition=launch.conditions.IfCondition(
                    launch.substitutions.AndSubstitution(
                        use_odom_arg.launch_config, use_odom_tf_arg.launch_config)
                ),
            ),

            # launch.actions.IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         pkg_share + "/launch/laser_filters.launch.py"
            #     ),
            #     condition=launch.conditions.IfCondition(
            #         use_lidar_arg.launch_config),
            # ),

            Node(
                package="nmea_navsat_driver",
                executable="nmea_topic_driver",
                name="nmea_topic_driver",
                output="screen",
                parameters=[{
                    "time_ref_source": "gps",
                    "use_sim_time": simulation_arg.launch_config,
                    "epe_quality2": 1.0,
                }],
                remappings=[
                    ("nmea_sentence", "nmea_sentence"),
                    ("fix", "gps/fix"),
                ],
                condition=launch.conditions.IfCondition(
                    use_gps_arg.launch_config),
            ),

            Node(
                package=pkg_name,
                executable="gnss_odometry_node.py",
                name="gnss_odometry_node",
                output="screen",
                parameters=[{
                    "use_sim_time": simulation_arg.launch_config,
                    "map_frame_id": "map",
                    "gps_frame_id": "gps_link",
                    # GNSS input selection: 'navsatfix' or 'navpvt'
                    "gnss_input": "navpvt",

                    # min_speed_for_heading: m/s (if ground speed < this, motion heading is ignored)
                    "min_speed_for_heading": 0.5,
                    # heading_smoothing_alpha: unitless (0..1), larger -> more weight to latest observation
                    "heading_smoothing_alpha": 0.6,

                    # apply_heading_invert: bool, multiply heading by -1 when True
                    "apply_heading_invert": True,
                    # apply_heading_add_pi: bool, add 180 deg (pi rad) to heading when True
                    "apply_heading_add_pi": True,

                    # NavPVT covariance/default parameters
                    "navpvt_hacc_to_pos_std_scale": 1.0,
                    "navpvt_vacc_to_pos_std_scale": 1.0,
                    "navpvt_headacc_to_yaw_var_scale": 1.5,
                }],
                remappings=[
                    ("/ublox/navpvt", "/navpvt"),
                ],
                condition=launch.conditions.IfCondition(
                    use_gps_arg.launch_config),
            ),

            # for converting Realsense pointcloud to laser scan
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan_node",
                output="screen",
                parameters=[{
                    "target_frame": "base_footprint",
                    "transform_tolerance": 0.5,
                    "min_height": 0.3,
                    "max_height": 1.5,
                    "angle_min": -3.14,
                    "angle_max": 3.14,
                    "angle_increment": 0.0058,
                    "scan_time": 0.1,
                    "range_min": 0.01,
                    "range_max": 3.0,
                    # "use_sim_time": simulation_arg.launch_config,
                    "use_sim_time": True,
                    "use_inf": True,
                    "inf_epsilon": 1.0,
                }],
                remappings=[
                    ("cloud_in", "/camera/camera/depth/color/points"),
                    ("scan", "/scan_from_realsense"),
                ],
                condition=launch.conditions.IfCondition(
                    use_realsense_arg.launch_config
                )
            ),

        ]
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            sensors_processing_group,
        ]
    )
