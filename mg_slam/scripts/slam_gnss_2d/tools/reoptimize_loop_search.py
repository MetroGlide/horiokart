from __future__ import annotations

import math
from typing import Callable

import numpy as np
from scipy.spatial import KDTree

from slam_gnss_2d.core.data_types import OdomData, PoseEdge, PoseNode, ScanData
from slam_gnss_2d.core.geometry import scan_to_points
from slam_gnss_2d.scan_matching.base import ScanMatcherBase
from slam_gnss_2d.tools.reoptimize_geometry import build_submap_points


def search_new_loop_edges(
    *,
    optimized_nodes: list[PoseNode],
    node_scans: dict[int, ScanData],
    existing_loop_edges: list[PoseEdge],
    loop_matcher: ScanMatcherBase,
    search_radius: float,
    min_node_gap: int,
    submap_radius: float,
    max_score: float,
    max_dyaw_deg: float,
    crossing_reject_deg: float,
    logger_info: Callable[[str], None],
) -> list[PoseEdge]:
    """KDTreeで新規ループ候補を探索し、追加可能なループ辺を返す。"""
    existing_pairs = {(e.from_index, e.to_index) for e in existing_loop_edges}
    existing_pairs |= {(e.to_index, e.from_index) for e in existing_loop_edges}

    valid_nodes = [
        node for node in optimized_nodes if node.index in node_scans]
    if not valid_nodes:
        return []

    positions = np.array([[node.x, node.y] for node in valid_nodes])
    tree = KDTree(positions)

    new_edges: list[PoseEdge] = []
    for node in valid_nodes:
        neighbor_idxs = tree.query_ball_point([node.x, node.y], search_radius)
        for idx in neighbor_idxs:
            candidate = valid_nodes[idx]
            if abs(node.index - candidate.index) < min_node_gap:
                continue

            pair = (min(node.index, candidate.index),
                    max(node.index, candidate.index))
            if pair in existing_pairs:
                continue

            src_pts = (
                build_submap_points(
                    candidate.index,
                    optimized_nodes,
                    node_scans,
                    submap_radius,
                )
                if submap_radius > 0
                else scan_to_points(node_scans[candidate.index])
            )
            if src_pts is None or len(src_pts) == 0:
                continue

            cos_yaw = math.cos(-candidate.yaw)
            sin_yaw = math.sin(-candidate.yaw)
            dx_w = node.x - candidate.x
            dy_w = node.y - candidate.y
            initial_guess = OdomData(
                timestamp=node.timestamp,
                x=cos_yaw * dx_w - sin_yaw * dy_w,
                y=sin_yaw * dx_w + cos_yaw * dy_w,
                yaw=node.yaw - candidate.yaw,
            )

            loop_matcher.set_target_cloud(src_pts)
            result = loop_matcher.match(
                dst=node_scans[node.index],
                initial_guess=initial_guess,
            )
            if not result.converged:
                continue
            if max_score > 0.0 and result.score > max_score:
                continue

            abs_dyaw = abs(result.dyaw)
            if abs_dyaw > math.radians(max_dyaw_deg):
                continue

            crossing_rad = math.radians(crossing_reject_deg)
            if crossing_rad > 0.0 and crossing_rad <= abs_dyaw <= math.pi - crossing_rad:
                continue

            edge = PoseEdge(
                from_index=candidate.index,
                to_index=node.index,
                dx=result.dx,
                dy=result.dy,
                dyaw=result.dyaw,
                information=result.information,
            )
            new_edges.append(edge)
            existing_pairs.add(pair)
            logger_info(
                f"New loop edge found: {candidate.index} -> {node.index} "
                f"(score={result.score:.4f})"
            )

    return new_edges
