from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TopicsConfig:
    """購読トピック名設定"""
    scan: str = '/scan_top_lidar'
    odom: str = '/odom'

@dataclass(frozen=True)
class MapConfig:
    """占有格子マップ設定"""
    resolution: float = 0.05          # [m/px]
    expansion_margin: float = 100.0   # [m]
    publish_hz: float = 1.0           # 配信周期 [Hz]
    renderer: str = 'overwrite'       # レンダラーの種別 ('overwrite' | 'counting')
    hit_threshold: float = 0.3        # 占有と判定するヒット率の閾値（counting用）

@dataclass(frozen=True)
class KeyframeConfig:
    """キーフレーム採択閾値"""
    min_translation: float = 1.0   # [m]
    min_rotation: float = 0.1      # [rad]

@dataclass(frozen=True)
class IcpConfig:
    """ICP パラメータ"""
    max_iterations: int = 100
    tolerance: float = 1e-5
    max_correspondence_dist: float = 1.0   # [m] 大きいと誤対応リスク増
    robust_kernel: str = 'huber'           # 'none', 'huber', 'cauchy'
    robust_kernel_scale: float = 0.1       # ロバストカーネルのスケールパラメータ

@dataclass(frozen=True)
class NdtConfig:
    """NDT パラメータ"""
    cell_size: float = 1.0   # [m] 互換性のため残す
    cell_sizes: tuple[float, ...] = (1.0,) # マルチ解像度探索用のセルサイズリスト
    use_bilinear: bool = False             # Bilinear補間の有効化フラグ

@dataclass(frozen=True)
class LocalMapConfig:
    """ローカルマップパラメータ（scan_reference == "scan_to_local_map" のとき使用）"""
    window: int = 30      # スライディングウィンドウ幅 [ノード数]
    radius: float = 30.0  # 参照点群の抽出半径 [m]

@dataclass(frozen=True)
class CsmConfig:
    """CSM パラメータ"""
    linear_search_window: float = 1.0
    angular_search_window: float = 0.5
    linear_step: float = 0.05
    angular_step: float = 0.02

@dataclass(frozen=True)
class ScanMatchingConfig:
    """スキャンマッチング設定"""
    enabled: bool = True
    type: str = 'ndt'              # "icp" | "ndt" | "csm"
    reference: str = 'scan_to_local_map' # "scan_to_scan" | "scan_to_local_map"
    max_failure_streak: int = 5    # 連続失敗がこの回数に達したら odom フォールバック
    icp: IcpConfig = IcpConfig()
    ndt: NdtConfig = NdtConfig()
    csm: CsmConfig = CsmConfig()
    local_map: LocalMapConfig = LocalMapConfig()

@dataclass(frozen=True)
class LoopClosureConfig:
    """ループクロージャパラメータ"""
    enabled: bool = True
    search_radius: float = 2.0   # ループ候補ノード間距面間値 [m]
    min_node_gap: int = 50       # ループ候補の最小ノード間隔
    max_failure_streak: int = 3  # ループ検証連続失敗上限
    # ループ検証専用マッチャー。NDTはセル対称性によるfalse positiveリスクがあるため
    # 連続マッチング（scan_matching.type）とは独立して設定できる。
    matcher_type: str = 'icp'    # "icp" | "ndt" | "csm"
    icp: IcpConfig = IcpConfig()
    ndt: NdtConfig = NdtConfig()
    csm: CsmConfig = CsmConfig()
    # ループ辺のdyaw絶対値上限 [deg]。Uターンループを許容する値
    max_dyaw_deg: float = 145.0
    # パス交差による false positive を拒否する dyaw 境界 [deg]。
    # |dyaw| がこの値より大きく (180 - この値) より小さい場合（交差帯域）はループ辺を拒否する。
    # 0.0 のとき無効（全 dyaw を許容）。
    crossing_reject_deg: float = 45.0
    # ループ検証時のサブマップ合成半径 [m]。候補ノードからこの範囲内のノードのスキャンを
    # 合成して参照点群とする。0以下にすると候補ノード1枚のみ使用（無効化）。
    submap_radius: float = 5.0
    # ループ辺のマッチングスコア上限。ICP では平均点対線残差 [m]、NDT では平均負対数尤度。
    # 0.0 のとき無効（スコアによる排除なし）。
    max_score: float = 0.0

@dataclass(frozen=True)
class GnssTopicsConfig:
    fix: str = '/gps/fix'
    navpvt: str = '/navpvt' # NavPVT トピック名（source == "navpvt" のとき使用）

@dataclass(frozen=True)
class GnssValidationConfig:
    max_sigma_m: float = 5.0 # h_acc 導出のσ 上限 [m]。これを超える測位は拘束をスキップ。
    missing_grace_frames: int = 30 # センサー欠測時降格ポリシー

@dataclass(frozen=True)
class GnssAnchorConfig:
    min_fix_status: int = 0
    sigma_m: float = 0.05
    init_yaw_sigma_rad: float = 10.0
    init_distance_m: float = 2.0

@dataclass(frozen=True)
class GnssSigmaConfig:
    fix_m: float = 0.02
    float_m: float = 0.5
    factor_yaw_variance: float = 1e8

@dataclass(frozen=True)
class GnssConfig:
    """GNSS 拘束設定"""
    enabled: bool = True
    source: str = 'navpvt' # GNSS ソース種別。"navsat_fix" | "navpvt"
    topics: GnssTopicsConfig = GnssTopicsConfig()
    navpvt_hacc_scale: float = 1.0 # NavPVT h_acc → pos_std 変換スケール。実機キャリブレーション用。
    validation: GnssValidationConfig = GnssValidationConfig()
    anchor: GnssAnchorConfig = GnssAnchorConfig()
    sigma: GnssSigmaConfig = GnssSigmaConfig()

@dataclass(frozen=True)
class Isam2Config:
    relinearize_threshold: float = 0.1

@dataclass(frozen=True)
class OptimizationConfig:
    """最適化設定"""
    backend: str = 'gtsam' # "isam2" | "gtsam"
    incremental: bool = True
    optimize_every_n_loops: int = 3 # N本ループ辺追加ごとに最適化
    rerender_threshold_m: float = 0.1
    isam2: Isam2Config = Isam2Config()

@dataclass(frozen=True)
class SlamConfig:
    """slam_gnss_2d の全パラメータ集約オブジェクト（ROS 非依存）。

    slam_node.py のみがこのオブジェクトを生成し、
    コアロジック群はこのオブジェクトから必要なフィールドのみ参照する。
    """
    topics: TopicsConfig = TopicsConfig()
    map: MapConfig = MapConfig()
    keyframe: KeyframeConfig = KeyframeConfig()
    scan_matching: ScanMatchingConfig = ScanMatchingConfig()
    loop_closure: LoopClosureConfig = LoopClosureConfig()
    gnss: GnssConfig = GnssConfig()
    optimization: OptimizationConfig = OptimizationConfig()
