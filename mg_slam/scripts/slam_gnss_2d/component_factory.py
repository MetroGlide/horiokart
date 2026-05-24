from __future__ import annotations

from .pose_graph.base import PoseGraphBuilderBase


def build_pose_graph_builder(
    builder_type: str,
    min_translation: float,
    min_rotation: float,
    icp_max_iterations: int = 30,
    icp_tolerance: float = 1e-4,
    icp_max_correspondence_dist: float = 0.5,
) -> PoseGraphBuilderBase:
    """ポーズグラフビルダーを構築して返す。ROS 非依存。

    Args:
        builder_type: "odom_only" または "scan_matching"。
        min_translation: キーフレーム追加の最小移動距離 [m]。
        min_rotation: キーフレーム追加の最小回転量 [rad]。
        icp_max_iterations: ICP 最大反復回数（scan_matching 時のみ使用）。
        icp_tolerance: ICP 収束判定閾値（scan_matching 時のみ使用）。
        icp_max_correspondence_dist: ICP 最大対応点距離 [m]（scan_matching 時のみ使用）。

    Returns:
        PoseGraphBuilderBase の具体実装インスタンス。

    Raises:
        ValueError: 未知の builder_type が指定された場合。
    """
    if builder_type == 'scan_matching':
        from .scan_matching.icp_matcher import ICPMatcher
        from .pose_graph.scan_matching_builder import ScanMatchingBuilder
        matcher = ICPMatcher(
            max_iterations=icp_max_iterations,
            tolerance=icp_tolerance,
            max_correspondence_dist=icp_max_correspondence_dist,
        )
        return ScanMatchingBuilder(
            matcher=matcher,
            min_translation=min_translation,
            min_rotation=min_rotation,
        )
    elif builder_type == 'odom_only':
        from .pose_graph.odom_builder import OdomOnlyBuilder
        return OdomOnlyBuilder(
            min_translation=min_translation,
            min_rotation=min_rotation,
        )
    else:
        raise ValueError(
            f"Unknown pose_graph_builder: '{builder_type}'. "
            "Valid options: 'odom_only', 'scan_matching'"
        )
