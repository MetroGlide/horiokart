from __future__ import annotations

import math

import numpy as np

from slam_gnss_2d.core.data_types import ScanData


def angle_diff(a: float, b: float) -> float:
    """角度差 a - b を [-pi, pi] に正規化して返す。"""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def normalize_angle(angle: float) -> float:
    """角度を [-pi, pi] に正規化して返す。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q) -> float:
    """Quaternion から yaw 角を取り出す。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def scan_to_points(scan: ScanData) -> np.ndarray:
    """有効レンジのみを2D点群 (N, 2) に変換する。"""
    n = len(scan.ranges)
    angles = scan.angle_min + np.arange(n) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid = (ranges >= scan.range_min) & (ranges <= scan.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))
