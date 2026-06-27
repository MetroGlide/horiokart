from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class ScanData:
    """2D LiDARスキャン1フレームのデータコンテナ。ROSに非依存。"""

    timestamp: float
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float = 0.1
    range_max: float = 30.0


@dataclass
class OdomData:
    """オドメトリ1時刻のデータコンテナ。ROSに非依存。"""

    timestamp: float
    x: float
    y: float
    yaw: float


@dataclass
class GnssData:
    """GNSS測位1時刻のデータコンテナ。

    Phase 1〜3 では使用しない。
    Phase 4 で x/y に UTM等の平面直角座標を設定する。
    ROS2GnssSource の段階では longitude/latitude を仮置きする（Phase 4でUTM変換）。
    covariance は shape (2, 2) の共分散行列。
    """

    timestamp: float
    x: float
    y: float
    covariance: np.ndarray
    # 測位品質ステータス。NavSatFix.status.status 互換（未設定時は -1）。
    fix_status: int = -1
    latitude: float = 0.0
    longitude: float = 0.0


@dataclass
class SensorFrame:
    """1つのスキャン時刻に同期されたセンサデータ群のパケット"""

    scan: ScanData
    odom: OdomData
    gnss: Optional[GnssData] = None


@dataclass
class ScanProcessResult:
    """1フレームの処理結果を格納するコンテナ"""

    node: Optional[PoseNode]
    loop_closed: bool = False
    rerender_required: bool = False
    new_seq_edge: Optional[PoseEdge] = None
    new_loop_edges: list[PoseEdge] = None
    new_gnss_prior: Optional[GnssPrior] = None


@dataclass
class PoseNode:
    """ポーズグラフの1ノード。推定姿勢とスキャンデータを保持する。"""

    index: int
    timestamp: float
    x: float
    y: float
    yaw: float
    scan: Optional[ScanData] = None


@dataclass
class MatchResult:
    """スキャンマッチングの結果。ScanMatcherBase.match() が返す。"""

    dx: float
    dy: float
    dyaw: float
    converged: bool
    # shape (3, 3) — 拘束の情報行列。Phase 3 の GTSAMOptimizer が使用。
    information: np.ndarray
    # マッチング品質スコア（小さいほど良い）。
    # ICPでは有効対応点の平均点対線残差 [m]、NDTでは平均負対数尤度。
    # 0.0 はスコア未計算または未収束を示す。
    score: float = 0.0


@dataclass
class PoseEdge:
    """ポーズグラフのノード間拘束辺。Phase 3 の GTSAMOptimizer が使用。"""

    from_index: int
    to_index: int
    dx: float
    dy: float
    dyaw: float
    information: np.ndarray  # shape (3, 3)
    score: float = 0.0
    is_odom_fallback: bool = False


@dataclass
class GnssPrior:
    """GNSS絶対位置拘束。GTSAMOptimizer.optimize() の入力として使用する。

    GnssConstraintInserter.build_priors() が生成し、GTSAMOptimizer が
    PriorFactorPose2（yaw自由度を緩く固定）として投入する。
    """

    node_index: int
    x: float             # SLAM座標系でのGNSS x座標 [m]
    y: float             # SLAM座標系でのGNSS y座標 [m]
    information: np.ndarray  # shape (2, 2) — xy平面の情報行列
