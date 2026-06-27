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


def world_delta_to_local(dx_w: float, dy_w: float, reference_yaw: float) -> tuple[float, float]:
    """ワールド座標の並進差分を基準yawのローカル座標へ変換する。"""
    cos_yaw = math.cos(-reference_yaw)
    sin_yaw = math.sin(-reference_yaw)
    return (
        cos_yaw * dx_w - sin_yaw * dy_w,
        sin_yaw * dx_w + cos_yaw * dy_w,
    )


def local_delta_to_world(dx_local: float, dy_local: float, reference_yaw: float) -> tuple[float, float]:
    """ローカル座標の並進差分を基準yawのワールド座標へ変換する。"""
    cos_yaw = math.cos(reference_yaw)
    sin_yaw = math.sin(reference_yaw)
    return (
        cos_yaw * dx_local - sin_yaw * dy_local,
        sin_yaw * dx_local + cos_yaw * dy_local,
    )


def points_local_to_world(local_pts: np.ndarray, origin_x: float, origin_y: float, yaw: float) -> np.ndarray:
    """ローカル点群 (N,2) をワールド座標へ変換する。"""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    wx = cos_yaw * local_pts[:, 0] - sin_yaw * local_pts[:, 1] + origin_x
    wy = sin_yaw * local_pts[:, 0] + cos_yaw * local_pts[:, 1] + origin_y
    return np.column_stack((wx, wy))


def points_world_to_local(world_pts: np.ndarray, origin_x: float, origin_y: float, yaw: float) -> np.ndarray:
    """ワールド点群 (N,2) を基準poseのローカル座標へ変換する。"""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    rotation_inv = np.array([[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]])
    translated = world_pts - np.array([origin_x, origin_y])
    return (rotation_inv @ translated.T).T
