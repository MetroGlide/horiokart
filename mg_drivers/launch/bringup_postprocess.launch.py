#!/usr/bin/env python3

import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

from mg_utils.launch_argument import LaunchArgumentCreator


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
    use_sensor_data_qos_arg = launch_argument_creator.create(
        "use_sensor_data_qos", default="false")


    pkg_name = "mg_drivers"
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
                package="mg_drivers",
                executable="lidar_publish_controller_node.py",
                name="front_lidar_publish_controller_node",
                output="screen",
                parameters=[{
                }],
                remappings=[("scan_origin", "scan_front_lidar_origin"),
                            ("scan", "scan_front_lidar")],
                condition=launch.conditions.IfCondition(
                    use_lidar_arg.launch_config),
            ),

            #################################################

            # for converting Realsense pointcloud to laser scan
            # Node(
            #     package="pointcloud_to_laserscan",
            #     executable="pointcloud_to_laserscan_node",
            #     name="pointcloud_to_laserscan_node",
            #     output="screen",
            #     parameters=[{
            #         "target_frame": "base_footprint",
            #         "transform_tolerance": 0.5,
            #         "min_height": 0.3,
            #         "max_height": 1.5,
            #         "angle_min": -3.14,
            #         "angle_max": 3.14,
            #         "angle_increment": 0.0058,
            #         "scan_time": 0.1,
            #         "range_min": 0.01,
            #         "range_max": 3.0,
            #         # "use_sim_time": simulation_arg.launch_config,
            #         "use_sim_time": True,
            #         "use_inf": True,
            #         "inf_epsilon": 1.0,
            #     }],
            #     remappings=[
            #         ("cloud_in", "/camera/camera/depth/color/points"),
            #         ("scan", "/scan_from_realsense"),
            #     ],
            #     condition=launch.conditions.IfCondition(
            #         use_realsense_arg.launch_config
            #     )
            # ),

            # Depth postprocess node: statistical outlier removal + voxel downsampling
            Node(
                package=pkg_name,
                executable="depth_postprocess_node",
                name="depth_postprocess_node",
                output="screen",
                parameters=[{
                    # Voxel grid leaf size in meters
                    "voxel_leaf_size": 0.05,
                    # Enable statistical outlier removal (bool)
                    "use_statistical_outlier_removal": True,
                    # Mean K for StatisticalOutlierRemoval (int)
                    "sor_mean_k": 50,
                    # Stddev multiplier threshold for outlier removal (float)
                    "sor_std_mul": 1.0,
                    # Use simulation time if requested
                    "use_sim_time": simulation_arg.launch_config,
                }],
                remappings=[
                    ("points", "/camera/camera/depth/color/points"),
                    ("points_filtered", "/camera/depth/points_postprocessed"),
                ],
                condition=launch.conditions.IfCondition(
                    use_realsense_arg.launch_config
                ),
            ),

            # Obstacle Detection 3D
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "obstacle_detection_3d.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": simulation_arg.launch_config,
                    "use_sensor_data_qos": use_sensor_data_qos_arg.launch_config,
                }.items(),
                condition=launch.conditions.IfCondition(
                    use_realsense_arg.launch_config
                ),
            ),

        ]
    )

    return LaunchDescription(
        [
            *launch_argument_creator.get_created_declare_launch_args(),
            sensors_processing_group,
        ]
    )
