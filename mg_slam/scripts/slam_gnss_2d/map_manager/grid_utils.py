from __future__ import annotations

import math

import cv2
import numpy as np

from slam_gnss_2d.core.data_types import PoseNode


def compute_square_bounds(
    nodes: list[PoseNode],
    resolution: float,
    expansion_margin: float,
) -> tuple[float, float, int]:
    """全ノードを含む正方形マップの原点とサイズを計算する。"""
    xs = np.fromiter((n.x for n in nodes), dtype=np.float64, count=len(nodes))
    ys = np.fromiter((n.y for n in nodes), dtype=np.float64, count=len(nodes))
    origin_x = float(xs.min()) - expansion_margin
    origin_y = float(ys.min()) - expansion_margin
    max_x = float(xs.max()) + expansion_margin
    max_y = float(ys.max()) + expansion_margin
    size = max(
        math.ceil((max_x - origin_x) / resolution),
        math.ceil((max_y - origin_y) / resolution),
    )
    return origin_x, origin_y, size


def world_to_pixel(
    wx: float,
    wy: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> tuple[int, int]:
    """世界座標をマップピクセル座標へ変換する。"""
    px = int((wx - origin_x) / resolution)
    py = int((wy - origin_y) / resolution)
    return px, py


def in_bounds(px: int, py: int, map_size: int) -> bool:
    """ピクセル座標が正方形マップ内か判定する。"""
    return 0 <= px < map_size and 0 <= py < map_size


def scan_hits_to_pixels(
    node: PoseNode,
    origin_x: float,
    origin_y: float,
    resolution: float,
    map_size: int,
) -> tuple[int, int, np.ndarray, np.ndarray, np.ndarray]:
    """ノードの有効スキャン終端をピクセル座標へ変換する。"""
    scan = node.scan
    robot_px, robot_py = world_to_pixel(
        node.x, node.y, origin_x, origin_y, resolution)
    if scan is None:
        empty = np.empty(0, dtype=np.int32)
        return robot_px, robot_py, empty, empty, np.empty(0, dtype=bool)

    angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid_mask = (ranges > scan.range_min) & (ranges < scan.range_max)

    cos_yaw = np.cos(node.yaw)
    sin_yaw = np.sin(node.yaw)
    r_v = ranges[valid_mask]
    a_v = angles[valid_mask]
    lx = r_v * np.cos(a_v)
    ly = r_v * np.sin(a_v)
    wx = node.x + cos_yaw * lx - sin_yaw * ly
    wy = node.y + sin_yaw * lx + cos_yaw * ly

    hit_px = ((wx - origin_x) / resolution).astype(np.int32)
    hit_py = ((wy - origin_y) / resolution).astype(np.int32)
    bounds_mask = (
        (hit_px >= 0) & (hit_px < map_size) &
        (hit_py >= 0) & (hit_py < map_size)
    )
    return robot_px, robot_py, hit_px, hit_py, bounds_mask


def build_trajectory_mask(
    shape: tuple[int, int],
    nodes: list[PoseNode],
    radius_m: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> np.ndarray:
    """軌跡周辺セルを示すboolマスクを生成する。"""
    radius_px = max(1, int(radius_m / resolution))
    pts = np.empty((1, len(nodes), 2), dtype=np.int32)
    for i, node in enumerate(nodes):
        px, py = world_to_pixel(node.x, node.y, origin_x, origin_y, resolution)
        pts[0, i, 0] = px
        pts[0, i, 1] = py

    mask = np.zeros(shape, dtype=np.uint8)
    cv2.polylines(mask, pts, isClosed=False, color=1, thickness=radius_px * 2)
    for i in range(len(nodes)):
        cv2.circle(
            mask,
            (int(pts[0, i, 0]), int(pts[0, i, 1])),
            radius_px,
            color=1,
            thickness=-1,
        )
    return mask > 0
