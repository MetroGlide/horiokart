from __future__ import annotations

from slam_gnss_2d.core.config import SlamConfig
from slam_gnss_2d.input.ros2.bag_reader import BagGnssSource, BagNavPVTSource
from slam_gnss_2d.map_manager.counting_renderer import CountingRenderer
from slam_gnss_2d.map_manager.overwrite_renderer import OverwriteRenderer
from slam_gnss_2d.pose_graph.base import PoseGraphBuilderBase
from slam_gnss_2d.pose_graph.loop_closure_builder import LoopClosureBuilder
from slam_gnss_2d.pose_graph.odom_builder import OdomOnlyBuilder
from slam_gnss_2d.pose_graph.scan_matching_builder import ScanMatchingBuilder
from slam_gnss_2d.scan_matching.base import ScanMatcherBase
from slam_gnss_2d.scan_matching.csm_matcher import CSMMatcher
from slam_gnss_2d.scan_matching.icp_matcher import ICPMatcher
from slam_gnss_2d.scan_matching.ndt_matcher import NDTMatcher
from slam_gnss_2d.scan_matching.reference_provider.base import ReferenceProviderBase
from slam_gnss_2d.scan_matching.reference_provider.local_map import LocalMapProvider
from slam_gnss_2d.scan_matching.reference_provider.scan_to_scan import ScanToScanProvider
from slam_gnss_2d.map_manager.base import MapRendererBase


def build_pose_graph_builder(config: SlamConfig) -> PoseGraphBuilderBase:
    """設定オブジェクトからポーズグラフビルダーを構築して返す。ROS 非依存。

    Args:
        config: slam_gnss_2d の全パラメータ集約オブジェクト。

    Returns:
        PoseGraphBuilderBase の具体実装インスタンス。

    Raises:
        ValueError: 未知のビルダー/マッチャー/プロバイダー種別が指定された場合。
    """
    if config.loop_closure.enabled:
        builder_type = 'loop_closure'
    elif config.scan_matching.enabled:
        builder_type = 'scan_matching'
    else:
        builder_type = 'odom_only'

    if builder_type == 'scan_matching':
        return ScanMatchingBuilder(
            matcher=_build_matcher(config),
            provider=_build_reference_provider(config),
            min_translation=config.keyframe.min_translation,
            min_rotation=config.keyframe.min_rotation,
            max_failure_streak=config.scan_matching.max_failure_streak,
        )
    elif builder_type == 'loop_closure':
        inner = ScanMatchingBuilder(
            matcher=_build_matcher(config),
            provider=_build_reference_provider(config),
            min_translation=config.keyframe.min_translation,
            min_rotation=config.keyframe.min_rotation,
            max_failure_streak=config.scan_matching.max_failure_streak,
        )
        return LoopClosureBuilder(
            inner=inner,
            loop_matcher=_build_loop_matcher(config),
            loop_closure_search_radius=config.loop_closure.search_radius,
            loop_closure_min_node_gap=config.loop_closure.min_node_gap,
            loop_closure_max_failure_streak=config.loop_closure.max_failure_streak,
            max_loop_dyaw_deg=config.loop_closure.max_dyaw_deg,
            loop_closure_crossing_reject_deg=config.loop_closure.crossing_reject_deg,
            loop_closure_submap_radius=config.loop_closure.submap_radius,
            loop_closure_max_score=config.loop_closure.max_score,
        )
    elif builder_type == 'odom_only':
        return OdomOnlyBuilder(
            min_translation=config.keyframe.min_translation,
            min_rotation=config.keyframe.min_rotation,
        )
    else:
        raise ValueError(
            f"Unknown pose_graph_builder: '{builder_type}'. "
            "Valid options: 'odom_only', 'scan_matching', 'loop_closure'"
        )


def _build_matcher(config: SlamConfig) -> ScanMatcherBase:
    if config.scan_matching.type == 'icp':
        return ICPMatcher(
            max_iterations=config.scan_matching.icp.max_iterations,
            tolerance=config.scan_matching.icp.tolerance,
            max_correspondence_dist=config.scan_matching.icp.max_correspondence_dist,
            robust_kernel=config.scan_matching.icp.robust_kernel,
            robust_kernel_scale=config.scan_matching.icp.robust_kernel_scale,
            yaw_information_multiplier=config.scan_matching.yaw_information_multiplier,
        )
    elif config.scan_matching.type == 'ndt':
        return NDTMatcher(
            max_iterations=config.scan_matching.icp.max_iterations,
            tolerance=config.scan_matching.icp.tolerance,
            cell_sizes=tuple(config.scan_matching.ndt.cell_sizes),
            use_bilinear=config.scan_matching.ndt.use_bilinear,
            yaw_information_multiplier=config.scan_matching.yaw_information_multiplier,
        )
    elif config.scan_matching.type == 'csm':
        return CSMMatcher(
            linear_search_window=config.scan_matching.csm.linear_search_window,
            angular_search_window=config.scan_matching.csm.angular_search_window,
            linear_step=config.scan_matching.csm.linear_step,
            angular_step=config.scan_matching.csm.angular_step,
            yaw_information_multiplier=config.scan_matching.yaw_information_multiplier,
        )
    else:
        raise ValueError(
            f"Unknown scan_matcher_type: '{config.scan_matching.type}'. "
            "Valid options: 'icp', 'ndt', 'csm'"
        )


def _build_loop_matcher(config: SlamConfig) -> ScanMatcherBase:
    """ループクロージャ検証専用のスキャンマッチャーを構築する。

    NDT はセル分割の回転対称性により false positive を生じやすいため、
    ループクロージャには ICP を使用することを推奨する。
    loop_closure_matcher_type パラメータで設定を上書きできる。
    """
    if config.loop_closure.matcher_type == 'icp':
        return ICPMatcher(
            max_iterations=config.loop_closure.icp.max_iterations,
            tolerance=config.loop_closure.icp.tolerance,
            max_correspondence_dist=config.loop_closure.icp.max_correspondence_dist,
            robust_kernel=config.loop_closure.icp.robust_kernel,
            robust_kernel_scale=config.loop_closure.icp.robust_kernel_scale,
            yaw_information_multiplier=config.loop_closure.yaw_information_multiplier,
        )
    elif config.loop_closure.matcher_type == 'ndt':
        return NDTMatcher(
            max_iterations=config.loop_closure.icp.max_iterations,
            tolerance=config.loop_closure.icp.tolerance,
            cell_sizes=tuple(config.loop_closure.ndt.cell_sizes),
            use_bilinear=config.loop_closure.ndt.use_bilinear,
            yaw_information_multiplier=config.loop_closure.yaw_information_multiplier,
        )
    elif config.loop_closure.matcher_type == 'csm':
        return CSMMatcher(
            linear_search_window=config.loop_closure.csm.linear_search_window,
            angular_search_window=config.loop_closure.csm.angular_search_window,
            linear_step=config.loop_closure.csm.linear_step,
            angular_step=config.loop_closure.csm.angular_step,
            yaw_information_multiplier=config.loop_closure.yaw_information_multiplier,
        )
    else:
        raise ValueError(
            f"Unknown loop_closure_matcher_type: '{config.loop_closure.matcher_type}'. "
            "Valid options: 'icp', 'ndt', 'csm'"
        )


def _build_reference_provider(config: SlamConfig) -> ReferenceProviderBase:
    if config.scan_matching.reference == 'scan_to_scan':
        return ScanToScanProvider()
    elif config.scan_matching.reference == 'scan_to_local_map':
        return LocalMapProvider(
            window=config.scan_matching.local_map.window,
            radius=config.scan_matching.local_map.radius,
        )
    else:
        raise ValueError(
            f"Unknown scan_reference: '{config.scan_matching.reference}'. "
            "Valid options: 'scan_to_scan', 'scan_to_local_map'"
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
    if config.gnss.source == 'navsat_fix':
        return BagGnssSource(bag_path, config.gnss.topics.fix)
    elif config.gnss.source == 'navpvt':
        return BagNavPVTSource(
            bag_path=bag_path,
            navpvt_topic=config.gnss.topics.navpvt,
            hacc_scale=config.gnss.navpvt_hacc_scale,
        )
    else:
        raise ValueError(
            f"Unknown gnss_source_type: '{config.gnss.source}'. "
            "Valid options: 'navsat_fix', 'navpvt'"
        )


def build_renderer(config: SlamConfig) -> MapRendererBase:
    """SlamConfig に基づいて MapRendererBase 実装を生成する。"""
    if config.map.renderer == 'counting':
        return CountingRenderer(
            resolution=config.map.resolution,
            expansion_margin=config.map.expansion_margin,
            hit_threshold=config.map.hit_threshold,
        )
    else:
        return OverwriteRenderer(
            resolution=config.map.resolution,
            expansion_margin=config.map.expansion_margin,
        )
