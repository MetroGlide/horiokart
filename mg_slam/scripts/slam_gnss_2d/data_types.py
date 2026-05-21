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


@dataclass
class PoseNode:
    """ポーズグラフの1ノード。推定姿勢とスキャンデータを保持する。"""

    index: int
    timestamp: float
    x: float
    y: float
    yaw: float
    scan: Optional[ScanData] = None
