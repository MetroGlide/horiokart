from __future__ import annotations

import math

import numpy as np

from slam_gnss_2d.core.data_types import GnssData, PoseNode, ScanData
from slam_gnss_2d.core.geometry import (
    points_local_to_world,
    points_world_to_local,
    scan_to_points,
)
from slam_gnss_2d.input.time_series import nearest_by_timestamp


def build_submap_points(
    center_node_idx: int,
    nodes: list[PoseNode],
    node_scans: dict[int, ScanData],
    radius: float,
) -> np.ndarray | None:
    """指定ノード周辺の点群を合成してサブマップを構築する。"""
    center_node = nodes[center_node_idx]
    near_nodes = []
    for node in nodes:
        if node.index in node_scans:
            dist = math.hypot(node.x - center_node.x, node.y - center_node.y)
            if dist <= radius:
                near_nodes.append((node, node_scans[node.index]))

    if not near_nodes:
        return None

    world_pts_list = []
    for node, scan in near_nodes:
        local_pts = scan_to_points(scan)
        world_pts_list.append(
            points_local_to_world(local_pts, node.x, node.y, node.yaw)
        )

    world_pts = np.concatenate(world_pts_list, axis=0)

    return points_world_to_local(
        world_pts,
        center_node.x,
        center_node.y,
        center_node.yaw,
    )


def find_nearest_scan(
    scans: list[ScanData],
    timestamp: float,
    max_diff: float,
) -> ScanData | None:
    """指定時刻に最も近いScanを閾値付きで返す。"""
    if not scans:
        return None
    timestamps = [scan.timestamp for scan in scans]
    candidate = nearest_by_timestamp(scans, timestamps, timestamp)

    if candidate is not None and abs(candidate.timestamp - timestamp) <= max_diff:
        return candidate
    return None


def sigma_from_covariance_or_status(gnss: GnssData, config) -> float:
    """共分散またはfix_statusからGNSS拘束の標準偏差を求める。"""
    cov_xx = float(gnss.covariance[0, 0]
                   ) if gnss.covariance is not None else 0.0
    if cov_xx > 0.0:
        return math.sqrt(cov_xx)

    status = gnss.fix_status
    if status >= 2:
        return config.gnss.sigma.fix_m
    if status >= 0:
        return config.gnss.sigma.float_m
    return -1.0
