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

    # スキャンマッチング失敗対策
    matcher_max_failure_streak: int = 5  # 連続失敗がこの回数に達したら odom フォールバック

    # ループクロージャパラメータ（pose_graph_builder == "loop_closure" のとき使用）
    loop_closure_search_radius: float = 2.0   # ループ候補ノード間距面間値 [m]
    loop_closure_min_node_gap: int = 50        # ループ候補の最小ノード間隔
    loop_closure_max_failure_streak: int = 3   # ループ検証連続失敗上限
    optimize_every_n_loops: int = 1            # N本ループ辺追加ごとに最適化
    # ループ検証専用マッチャー。NDTはセル対称性によるfalse positiveリスクがあるため
    # 連続マッチング（scan_matcher_type）とは独立して設定できる。
    loop_closure_matcher_type: str = 'icp'    # "icp" | "ndt"
    # ループ辺のdyaw絶対値上限 [deg]。Uターンループを許容する値
    loop_closure_max_dyaw_deg: float = 90.0
