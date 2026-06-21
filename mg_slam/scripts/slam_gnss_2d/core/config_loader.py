from rclpy.node import Node
from slam_gnss_2d.core.config import (
    TopicsConfig, MapConfig, KeyframeConfig, ScanMatchingConfig, IcpConfig, NdtConfig, LocalMapConfig,
    LoopClosureConfig, GnssConfig, GnssTopicsConfig, GnssValidationConfig, GnssAnchorConfig,
    GnssSigmaConfig, OptimizationConfig, Isam2Config, TrajectoryNoiseFilterConfig, SlamConfig
)


class ConfigLoader:
    """ROS2ノードのパラメータ宣言とSlamConfigの構築を行う"""

    @staticmethod
    def declare_params(node: Node) -> None:
        node.declare_parameter('topics.scan', '/scan_top_lidar')
        node.declare_parameter('topics.odom', '/odom')
        node.declare_parameter('map.resolution', 0.05)
        node.declare_parameter('map.expansion_margin', 100.0)
        node.declare_parameter('map.publish_hz', 1.0)
        node.declare_parameter('map.renderer', 'overwrite')
        node.declare_parameter('map.hit_threshold', 0.3)
        node.declare_parameter('keyframe.min_translation', 1.0)
        node.declare_parameter('keyframe.min_rotation', 0.1)
        node.declare_parameter('scan_matching.enabled', True)
        node.declare_parameter('scan_matching.type', 'ndt')
        node.declare_parameter('scan_matching.reference', 'scan_to_local_map')
        node.declare_parameter('scan_matching.max_failure_streak', 5)
        node.declare_parameter(
            'scan_matching.yaw_information_multiplier', 100.0)
        node.declare_parameter('scan_matching.icp.max_iterations', 100)
        node.declare_parameter('scan_matching.icp.tolerance', 1e-5)
        node.declare_parameter(
            'scan_matching.icp.max_correspondence_dist', 1.0)
        node.declare_parameter('scan_matching.ndt.cell_size', 1.0)
        node.declare_parameter('scan_matching.local_map.window', 30)
        node.declare_parameter('scan_matching.local_map.radius', 30.0)
        node.declare_parameter('loop_closure.enabled', True)
        node.declare_parameter('loop_closure.search_radius', 2.0)
        node.declare_parameter('loop_closure.min_node_gap', 50)
        node.declare_parameter('loop_closure.max_failure_streak', 3)
        node.declare_parameter('loop_closure.matcher_type', 'icp')
        node.declare_parameter(
            'loop_closure.yaw_information_multiplier', 100.0)
        node.declare_parameter('loop_closure.icp.max_iterations', 100)
        node.declare_parameter('loop_closure.icp.tolerance', 1e-5)
        node.declare_parameter('loop_closure.icp.max_correspondence_dist', 1.0)
        node.declare_parameter('loop_closure.ndt.cell_size', 1.0)
        node.declare_parameter('loop_closure.max_dyaw_deg', 145.0)
        node.declare_parameter('loop_closure.crossing_reject_deg', 45.0)
        node.declare_parameter('loop_closure.submap_radius', 5.0)
        node.declare_parameter('loop_closure.max_score', 0.0)
        node.declare_parameter('gnss.enabled', True)
        node.declare_parameter('gnss.source', 'navpvt')
        node.declare_parameter('gnss.topics.fix', '/gps/fix')
        node.declare_parameter('gnss.topics.navpvt', '/navpvt')
        node.declare_parameter('gnss.navpvt_hacc_scale', 1.0)
        node.declare_parameter('gnss.validation.max_sigma_m', 5.0)
        node.declare_parameter('save_dir', '')
        node.declare_parameter('gnss.anchor.min_fix_status', 0)
        node.declare_parameter('gnss.anchor.sigma_m', 0.05)
        node.declare_parameter('gnss.anchor.init_yaw_sigma_rad', 10.0)
        node.declare_parameter('gnss.anchor.init_distance_m', 2.0)
        node.declare_parameter('gnss.sigma.fix_m', 0.02)
        node.declare_parameter('gnss.sigma.float_m', 0.5)
        node.declare_parameter('gnss.sigma.factor_yaw_variance', 1e8)
        node.declare_parameter('optimization.backend', 'isam2')
        node.declare_parameter('optimization.isam2.relinearize_threshold', 0.1)
        node.declare_parameter('trajectory_noise_filter.enabled', False)
        node.declare_parameter('trajectory_noise_filter.type', 'clear')
        node.declare_parameter('trajectory_noise_filter.radius_m', 0.5)

    @staticmethod
    def build_config(node: Node) -> SlamConfig:
        return SlamConfig(
            topics=TopicsConfig(
                scan=node.get_parameter('topics.scan').value,
                odom=node.get_parameter('topics.odom').value,
            ),
            map=MapConfig(
                resolution=node.get_parameter('map.resolution').value,
                expansion_margin=node.get_parameter(
                    'map.expansion_margin').value,
                publish_hz=node.get_parameter('map.publish_hz').value,
                renderer=node.get_parameter('map.renderer').value,
                hit_threshold=node.get_parameter('map.hit_threshold').value,
            ),
            keyframe=KeyframeConfig(
                min_translation=node.get_parameter(
                    'keyframe.min_translation').value,
                min_rotation=node.get_parameter('keyframe.min_rotation').value,
            ),
            scan_matching=ScanMatchingConfig(
                enabled=node.get_parameter('scan_matching.enabled').value,
                type=node.get_parameter('scan_matching.type').value,
                reference=node.get_parameter('scan_matching.reference').value,
                max_failure_streak=node.get_parameter(
                    'scan_matching.max_failure_streak').value,
                yaw_information_multiplier=node.get_parameter(
                    'scan_matching.yaw_information_multiplier').value,
                icp=IcpConfig(
                    max_iterations=node.get_parameter(
                        'scan_matching.icp.max_iterations').value,
                    tolerance=node.get_parameter(
                        'scan_matching.icp.tolerance').value,
                    max_correspondence_dist=node.get_parameter(
                        'scan_matching.icp.max_correspondence_dist').value,
                ),
                ndt=NdtConfig(
                    cell_size=node.get_parameter(
                        'scan_matching.ndt.cell_size').value,
                ),
                local_map=LocalMapConfig(
                    window=node.get_parameter(
                        'scan_matching.local_map.window').value,
                    radius=node.get_parameter(
                        'scan_matching.local_map.radius').value,
                ),
            ),
            loop_closure=LoopClosureConfig(
                enabled=node.get_parameter('loop_closure.enabled').value,
                search_radius=node.get_parameter(
                    'loop_closure.search_radius').value,
                min_node_gap=node.get_parameter(
                    'loop_closure.min_node_gap').value,
                max_failure_streak=node.get_parameter(
                    'loop_closure.max_failure_streak').value,
                matcher_type=node.get_parameter(
                    'loop_closure.matcher_type').value,
                yaw_information_multiplier=node.get_parameter(
                    'loop_closure.yaw_information_multiplier').value,
                icp=IcpConfig(
                    max_iterations=node.get_parameter(
                        'loop_closure.icp.max_iterations').value,
                    tolerance=node.get_parameter(
                        'loop_closure.icp.tolerance').value,
                    max_correspondence_dist=node.get_parameter(
                        'loop_closure.icp.max_correspondence_dist').value,
                ),
                ndt=NdtConfig(
                    cell_size=node.get_parameter(
                        'loop_closure.ndt.cell_size').value,
                ),
                max_dyaw_deg=node.get_parameter(
                    'loop_closure.max_dyaw_deg').value,
                crossing_reject_deg=node.get_parameter(
                    'loop_closure.crossing_reject_deg').value,
                submap_radius=node.get_parameter(
                    'loop_closure.submap_radius').value,
                max_score=node.get_parameter('loop_closure.max_score').value,
            ),
            gnss=GnssConfig(
                enabled=node.get_parameter('gnss.enabled').value,
                source=node.get_parameter('gnss.source').value,
                topics=GnssTopicsConfig(
                    fix=node.get_parameter('gnss.topics.fix').value,
                    navpvt=node.get_parameter('gnss.topics.navpvt').value,
                ),
                navpvt_hacc_scale=node.get_parameter(
                    'gnss.navpvt_hacc_scale').value,
                validation=GnssValidationConfig(
                    max_sigma_m=node.get_parameter(
                        'gnss.validation.max_sigma_m').value,
                ),
                anchor=GnssAnchorConfig(
                    min_fix_status=node.get_parameter(
                        'gnss.anchor.min_fix_status').value,
                    sigma_m=node.get_parameter('gnss.anchor.sigma_m').value,
                    init_yaw_sigma_rad=node.get_parameter(
                        'gnss.anchor.init_yaw_sigma_rad').value,
                    init_distance_m=node.get_parameter(
                        'gnss.anchor.init_distance_m').value,
                ),
                sigma=GnssSigmaConfig(
                    fix_m=node.get_parameter('gnss.sigma.fix_m').value,
                    float_m=node.get_parameter('gnss.sigma.float_m').value,
                    factor_yaw_variance=node.get_parameter(
                        'gnss.sigma.factor_yaw_variance').value,
                ),
            ),
            optimization=OptimizationConfig(
                backend=node.get_parameter('optimization.backend').value,
                isam2=Isam2Config(
                    relinearize_threshold=node.get_parameter(
                        'optimization.isam2.relinearize_threshold').value,
                )
            ),
            trajectory_noise_filter=TrajectoryNoiseFilterConfig(
                enabled=node.get_parameter('trajectory_noise_filter.enabled').value,
                type=node.get_parameter('trajectory_noise_filter.type').value,
                radius_m=node.get_parameter('trajectory_noise_filter.radius_m').value,
            ),
        )
