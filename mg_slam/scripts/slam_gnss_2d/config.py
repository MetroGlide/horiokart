from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SlamConfig:
    """slam_gnss_2d の全パラメータ集約オブジェクト（ROS 非依存）。

    slam_node.py のみがこのオブジェクトを生成し、
    コアロジック群はこのオブジェクトから必要なフィールドのみ参照する。
    """

    # 購読トピック名
    scan_topic: str = '/scan_top_lidar'
    odom_topic: str = '/odom'

    # 占有格子マップ設定
    map_resolution: float = 0.05          # [m/px]
    map_expansion_margin: float = 100.0   # [m]

    # キーフレーム採択閾値
    min_translation: float = 0.3   # [m]
    min_rotation: float = 0.1      # [rad]

    # 配信
    map_publish_hz: float = 1.0    # [Hz]

    # ポーズグラフ構築
    pose_graph_builder: str = 'scan_matching'   # "odom_only" | "scan_matching"
    scan_matcher_type: str = 'icp'              # "icp" | "ndt"
    # "scan_to_scan" | "scan_to_local_map"
    scan_reference: str = 'scan_to_scan'

    # ICP パラメータ
    icp_max_iterations: int = 30
    icp_tolerance: float = 1e-4
    icp_max_correspondence_dist: float = 1.0   # [m]

    # NDT パラメータ（scan_matcher_type == "ndt" のとき使用）
    ndt_cell_size: float = 1.0   # [m]

    # ローカルマップパラメータ（scan_reference == "scan_to_local_map" のとき使用）
    local_map_window: int = 20      # スライディングウィンドウ幅 [ノード数]
    local_map_radius: float = 15.0  # 参照点群の抽出半径 [m]
