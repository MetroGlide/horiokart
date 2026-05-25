from __future__ import annotations

from .config import SlamConfig
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
    if config.pose_graph_builder == 'scan_matching':
        from .pose_graph.scan_matching_builder import ScanMatchingBuilder
        return ScanMatchingBuilder(
            matcher=_build_matcher(config),
            provider=_build_reference_provider(config),
            min_translation=config.min_translation,
            min_rotation=config.min_rotation,
            max_failure_streak=config.matcher_max_failure_streak,
        )
    elif config.pose_graph_builder == 'odom_only':
        from .pose_graph.odom_builder import OdomOnlyBuilder
        return OdomOnlyBuilder(
            min_translation=config.min_translation,
            min_rotation=config.min_rotation,
        )
    else:
        raise ValueError(
            f"Unknown pose_graph_builder: '{config.pose_graph_builder}'. "
            "Valid options: 'odom_only', 'scan_matching'"
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
