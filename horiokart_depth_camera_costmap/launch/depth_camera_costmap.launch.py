from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='horiokart_depth_camera_costmap',
            plugin='horiokart_depth_camera_costmap::DepthCameraCostmapLayer',
            name='depth_camera_costmap_layer',
            output='screen',
            parameters=[{
                'max_slope_angle_deg': 20.0,
                'max_step_height_m': 0.10,
                'grid_resolution_m': 0.10,
                'z_variance_threshold': 0.005,
                'normal_angle_threshold_deg': 10.0,
                'cost_traversable': 10,
                'cost_semi_traversable': 80,
                'cost_obstacle': 200,
                'cost_lethal': 255,
                'voxel_leaf_size_m': 0.05,
                'sor_mean_k': 50,
                'sor_stddev_mul_thresh': 1.0,
                'cluster_distance_threshold_m': 0.20,
                'cluster_min_points': 10,
                'pointcloud_topic': '/rs_d435/depth_registered/points',
                'marker_topic': '/depth_costmap/markers',
                'pointcloud_queue_size': 10,
                'marker_queue_size': 10,
                'target_frame': 'base_link',
                'tf_lookup_timeout_ms': 100,
                'tf_retry_count': 3,
                'tf_retry_backoff_ms': 50,
                'conditional_overwrite': True
            }]
        )
    ])
