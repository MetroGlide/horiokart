from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TopicsConfig:
    """購読トピック名設定"""
    scan: str
    odom: str


@dataclass(frozen=True)
class MapConfig:
    """占有格子マップ設定"""
    resolution: float          # [m/px]
    expansion_margin: float   # [m]
    publish_hz: float           # 配信周期 [Hz]
    renderer: str       # レンダラーの種別 ('overwrite' | 'counting')
    hit_threshold: float        # 占有と判定するヒット率の閾値（counting用）


@dataclass(frozen=True)
class KeyframeConfig:
    """キーフレーム採択閾値"""
    min_translation: float   # [m]
    min_rotation: float      # [rad]


@dataclass(frozen=True)
class IcpConfig:
    """ICP パラメータ"""
    max_iterations: int
    tolerance: float
    max_correspondence_dist: float   # [m] 大きいと誤対応リスク増
    robust_kernel: str           # 'none', 'huber', 'cauchy'
    robust_kernel_scale: float       # ロバストカーネルのスケールパラメータ


@dataclass(frozen=True)
class NdtConfig:
    """NDT パラメータ"""
    cell_size: float   # [m] 互換性のため残す
    cell_sizes: tuple[float, ...]  # マルチ解像度探索用のセルサイズリスト
    use_bilinear: bool             # Bilinear補間の有効化フラグ


@dataclass(frozen=True)
class LocalMapConfig:
    """ローカルマップパラメータ（scan_reference == "scan_to_local_map" のとき使用）"""
    window: int      # スライディングウィンドウ幅 [ノード数]
    radius: float  # 参照点群の抽出半径 [m]


@dataclass(frozen=True)
class CsmConfig:
    """CSM パラメータ"""
    linear_search_window: float
    angular_search_window: float
    linear_step: float
    angular_step: float


@dataclass(frozen=True)
class ScanMatchingConfig:
    """スキャンマッチング設定"""
    enabled: bool
    type: str              # "icp" | "ndt" | "csm"
    reference: str  # "scan_to_scan" | "scan_to_local_map"
    max_failure_streak: int    # 連続失敗がこの回数に達したら odom フォールバック
    yaw_information_multiplier: float  # Yaw情報行列の倍率
    icp: IcpConfig
    ndt: NdtConfig
    csm: CsmConfig
    local_map: LocalMapConfig


@dataclass(frozen=True)
class LoopClosureConfig:
    """ループクロージャパラメータ"""
    enabled: bool
    search_radius: float   # ループ候補ノード間距面間値 [m]
    min_node_gap: int       # ループ候補の最小ノード間隔
    max_failure_streak: int  # ループ検証連続失敗上限
    # ループ検証専用マッチャー。NDTはセル対称性によるfalse positiveリスクがあるため
    # 連続マッチング（scan_matching.type）とは独立して設定できる。
    matcher_type: str    # "icp" | "ndt" | "csm"
    yaw_information_multiplier: float  # Yaw情報行列の倍率
    icp: IcpConfig
    ndt: NdtConfig
    csm: CsmConfig
    # ループ辺のdyaw絶対値上限 [deg]。Uターンループを許容する値
    max_dyaw_deg: float
    # パス交差による false positive を拒否する dyaw 境界 [deg]。
    # |dyaw| がこの値より大きく (180 - この値) より小さい場合（交差帯域）はループ辺を拒否する。
    # 0.0 のとき無効（全 dyaw を許容）。
    crossing_reject_deg: float
    # ループ検証時のサブマップ合成半径 [m]。候補ノードからこの範囲内のノードのスキャンを
    # 合成して参照点群とする。0以下にすると候補ノード1枚のみ使用（無効化）。
    submap_radius: float
    # ループ辺のマッチングスコア上限。ICP では平均点対線残差 [m]、NDT では平均負対数尤度。
    # 0.0 のとき無効（スコアによる排除なし）。
    max_score: float


@dataclass(frozen=True)
class GnssTopicsConfig:
    fix: str
    navpvt: str  # NavPVT トピック名（source == "navpvt" のとき使用）


@dataclass(frozen=True)
class GnssValidationConfig:
    max_sigma_m: float  # h_acc 導出のσ 上限 [m]。これを超える測位は拘束をスキップ。


@dataclass(frozen=True)
class GnssAnchorConfig:
    min_fix_status: int
    sigma_m: float
    init_yaw_sigma_rad: float
    init_distance_m: float


@dataclass(frozen=True)
class GnssSigmaConfig:
    fix_m: float
    float_m: float
    factor_yaw_variance: float


@dataclass(frozen=True)
class GnssConfig:
    """GNSS 拘束設定"""
    enabled: bool
    source: str  # GNSS ソース種別。"navsat_fix" | "navpvt"
    topics: GnssTopicsConfig
    navpvt_hacc_scale: float  # NavPVT h_acc → pos_std 変換スケール。実機キャリブレーション用。
    validation: GnssValidationConfig
    anchor: GnssAnchorConfig
    sigma: GnssSigmaConfig


@dataclass(frozen=True)
class Isam2Config:
    relinearize_threshold: float


@dataclass(frozen=True)
class OptimizationConfig:
    """最適化設定"""
    backend: str  # "isam2" | "gtsam" (バッチ用)
    isam2: Isam2Config


@dataclass(frozen=True)
class TrajectoryNoiseFilterConfig:
    """軌跡ベースのノイズ除去設定"""
    enabled: bool
    type: str        # "clear" | "attenuate"
    radius_m: float      # クリア/減衰させる半径 [m]


@dataclass(frozen=True)
class SlamConfig:
    """slam_gnss_2d の全パラメータ集約オブジェクト（ROS 非依存）。

    slam_node.py のみがこのオブジェクトを生成し、
    コアロジック群はこのオブジェクトから必要なフィールドのみ参照する。
    """
    topics: TopicsConfig
    map: MapConfig
    keyframe: KeyframeConfig
    scan_matching: ScanMatchingConfig
    loop_closure: LoopClosureConfig
    gnss: GnssConfig
    optimization: OptimizationConfig
    trajectory_noise_filter: TrajectoryNoiseFilterConfig
