from __future__ import annotations

from .config import SlamConfig
from .gnss.aligner_base import GnssAlignerBase
from .pose_graph.base import PoseGraphBuilderBase
from .scan_matching.base import ScanMatcherBase
from .scan_matching.reference_provider.base import ReferenceProviderBase


def build_pose_graph_builder(config: SlamConfig) -> PoseGraphBuilderBase:
    """設定オブジェクトからポーズグラフビルダーを構築して返す。ROS 非依存。

    Args:
        config: slam_gnss_2d の全パラメータ集約オブジェクト。

    Returns:
        PoseGraphBuilderBase の具体実装インスタンス。

    Raises:
        ValueError: 未知のビルダー/マッチャー/プロバイダー種別が指定された場合。
    """
    builder_type = config.pose_graph_builder
    if not config.enable_scan_matching:
        builder_type = 'odom_only'
    elif not config.enable_loop_closure and builder_type == 'loop_closure':
        builder_type = 'scan_matching'

    if builder_type == 'scan_matching':
        from .pose_graph.scan_matching_builder import ScanMatchingBuilder
        return ScanMatchingBuilder(
            matcher=_build_matcher(config),
            provider=_build_reference_provider(config),
            min_translation=config.min_translation,
            min_rotation=config.min_rotation,
            max_failure_streak=config.matcher_max_failure_streak,
        )
    elif builder_type == 'loop_closure':
        from .pose_graph.scan_matching_builder import ScanMatchingBuilder
        from .pose_graph.loop_closure_builder import LoopClosureBuilder
        from .optimizer.gtsam_optimizer import GTSAMOptimizer
        inner = ScanMatchingBuilder(
            matcher=_build_matcher(config),
            provider=_build_reference_provider(config),
            min_translation=config.min_translation,
            min_rotation=config.min_rotation,
            max_failure_streak=config.matcher_max_failure_streak,
        )
        return LoopClosureBuilder(
            inner=inner,
            loop_matcher=_build_loop_matcher(config),
            optimizer=GTSAMOptimizer(),
            loop_closure_search_radius=config.loop_closure_search_radius,
            loop_closure_min_node_gap=config.loop_closure_min_node_gap,
            loop_closure_max_failure_streak=config.loop_closure_max_failure_streak,
            optimize_every_n_loops=config.optimize_every_n_loops,
            max_loop_dyaw_deg=config.loop_closure_max_dyaw_deg,
            loop_closure_crossing_reject_deg=config.loop_closure_crossing_reject_deg,
            loop_closure_submap_radius=config.loop_closure_submap_radius,
            loop_closure_max_score=config.loop_closure_max_score,
        )
    elif builder_type == 'odom_only':
        from .pose_graph.odom_builder import OdomOnlyBuilder
        return OdomOnlyBuilder(
            min_translation=config.min_translation,
            min_rotation=config.min_rotation,
        )
    else:
        raise ValueError(
            f"Unknown pose_graph_builder: '{builder_type}'. "
            "Valid options: 'odom_only', 'scan_matching', 'loop_closure'"
        )


def _build_matcher(config: SlamConfig) -> ScanMatcherBase:
    if config.scan_matcher_type == 'icp':
        from .scan_matching.icp_matcher import ICPMatcher
        return ICPMatcher(
            max_iterations=config.icp_max_iterations,
            tolerance=config.icp_tolerance,
            max_correspondence_dist=config.icp_max_correspondence_dist,
        )
    elif config.scan_matcher_type == 'ndt':
        from .scan_matching.ndt_matcher import NDTMatcher
        return NDTMatcher(
            max_iterations=config.icp_max_iterations,
            tolerance=config.icp_tolerance,
            cell_size=config.ndt_cell_size,
        )
    else:
        raise ValueError(
            f"Unknown scan_matcher_type: '{config.scan_matcher_type}'. "
            "Valid options: 'icp', 'ndt'"
        )


def _build_loop_matcher(config: SlamConfig) -> ScanMatcherBase:
    """ループクロージャ検証専用のスキャンマッチャーを構築する。

    NDT はセル分割の回転対称性により false positive を生じやすいため、
    ループクロージャには ICP を使用することを推奨する。
    loop_closure_matcher_type パラメータで設定を上書きできる。
    """
    if config.loop_closure_matcher_type == 'icp':
        from .scan_matching.icp_matcher import ICPMatcher
        return ICPMatcher(
            max_iterations=config.icp_max_iterations,
            tolerance=config.icp_tolerance,
            max_correspondence_dist=config.icp_max_correspondence_dist,
        )
    elif config.loop_closure_matcher_type == 'ndt':
        from .scan_matching.ndt_matcher import NDTMatcher
        return NDTMatcher(
            max_iterations=config.icp_max_iterations,
            tolerance=config.icp_tolerance,
            cell_size=config.ndt_cell_size,
        )
    else:
        raise ValueError(
            f"Unknown loop_closure_matcher_type: '{config.loop_closure_matcher_type}'. "
            "Valid options: 'icp', 'ndt'"
        )


def _build_reference_provider(config: SlamConfig) -> ReferenceProviderBase:
    if config.scan_reference == 'scan_to_scan':
        from .scan_matching.reference_provider.scan_to_scan import ScanToScanProvider
        return ScanToScanProvider()
    elif config.scan_reference == 'scan_to_local_map':
        from .scan_matching.reference_provider.local_map import LocalMapProvider
        return LocalMapProvider(
            window=config.local_map_window,
            radius=config.local_map_radius,
        )
    else:
        raise ValueError(
            f"Unknown scan_reference: '{config.scan_reference}'. "
            "Valid options: 'scan_to_scan', 'scan_to_local_map'"
        )


def build_gnss_aligner(config: SlamConfig) -> GnssAlignerBase:
    """設定オブジェクトから GNSS アライナーを構築して返す。ROS 非依存。

    Args:
        config: slam_gnss_2d の全パラメータ集約オブジェクト。

    Returns:
        GnssAlignerBase の具体実装インスタンス。

    Raises:
        ValueError: 未知のアライナー種別が指定された場合。
    """
    if config.gnss_aligner == 'kinematic_heading':
        from .gnss.kinematic_aligner import KinematicHeadingAligner
        return KinematicHeadingAligner(
            min_speed_ms=config.kinematic_min_speed_ms,
        )
    elif config.gnss_aligner == 'precision_weighted':
        from .gnss.precision_weighted_aligner import PrecisionWeightedAligner
        return PrecisionWeightedAligner(
            min_speed_ms=config.kinematic_min_speed_ms,
            default_noise_xy_m=config.gnss_noise_xy_m,
            max_time_delta_s=config.gnss_max_time_delta_s,
        )
    else:
        raise ValueError(
            f"Unknown gnss_aligner: '{config.gnss_aligner}'. "
            "Valid options: 'kinematic_heading', 'precision_weighted'"
        )


def build_gnss_source(config: SlamConfig, bag_path: str):
    """rosbag2 から GNSS データを読み取るソースを構築して返す。

    gnss_source_type の値に応じて BagGnssSource または
    BagNavPVTSource を選択する。

    Args:
        config: slam_gnss_2d の全パラメータ集約オブジェクト。
        bag_path: 読み取る rosbag2 ディレクトリ・ファイルパス。

    Returns:
        GnssSourceBase の具体実装インスタンス。

    Raises:
        ValueError: 未知の gnss_source_type が指定された場合。
    """
    if config.gnss_source_type == 'navsat_fix':
        from .input.ros2.bag_reader import BagGnssSource
        return BagGnssSource(bag_path, config.gnss_topic)
    elif config.gnss_source_type == 'navpvt':
        from .input.ros2.bag_reader import BagNavPVTSource
        return BagNavPVTSource(
            bag_path=bag_path,
            navpvt_topic=config.gnss_navpvt_topic,
            hacc_scale=config.navpvt_hacc_scale,
        )
    else:
        raise ValueError(
            f"Unknown gnss_source_type: '{config.gnss_source_type}'. "
            "Valid options: 'navsat_fix', 'navpvt'"
        )
