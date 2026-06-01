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
    # パス交差による false positive を拒否する dyaw 境界 [deg]。
    # |dyaw| がこの値より大きく (180 - この値) より小さい場合（交差帯域）はループ辺を拒否する。
    # 0.0 のとき無効（全 dyaw を許容）。
    # 例: 45.0 → [45°, 135°] 帯域を拒否、≤45° の同方向と ≥135° の Uターンのみ許容。
    loop_closure_crossing_reject_deg: float = 0.0
    # ループ検証時のサブマップ合成半径 [m]。候補ノードからこの範囲内のノードのスキャンを
    # 合成して参照点群とする。0以下にすると候補ノード1枚のみ使用（無効化）。
    loop_closure_submap_radius: float = 5.0
    # ループ辺のマッチングスコア上限。ICP では平均点対線残差 [m]、NDT では平均負対数尤度。
    # 0.0 のとき無効（スコアによる排除なし）。
    loop_closure_max_score: float = 0.0

    # GNSS 拘束（use_gnss == True のとき slam_offline_node.py が使用する）
    use_gnss: bool = False
    gnss_topic: str = '/gps/fix'
    # GNSS 位置ノイズ [m]。position_covariance が不定の場合のフォールバック値
    gnss_noise_xy_m: float = 3.0
    # GNSS アライナー種別。"kinematic_heading" | "precision_weighted"
    gnss_aligner: str = 'kinematic_heading'
    # KinematicHeadingAligner / PrecisionWeightedAligner が有効とみなす最小移動速度 [m/s]。
    # この速度未満の区間は座標系回転の推定に使わない。
    kinematic_min_speed_ms: float = 0.5
    # GNSS 測位とポーズノードのタイムスタンプ差の上限 [s]。
    # この時間差を超えた GNSS 測位は拘束として使用しない。
    gnss_max_time_delta_s: float = 5.0
    # GNSS ソース種別。"navsat_fix" | "navpvt"
    gnss_source_type: str = 'navsat_fix'
    # NavPVT トピック名（gnss_source_type == "navpvt" のとき使用）
    gnss_navpvt_topic: str = '/ublox/navpvt'
    # NavPVT h_acc → pos_std 変換スケール。実機キャリブレーション用。
    navpvt_hacc_scale: float = 1.0
