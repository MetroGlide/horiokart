from __future__ import annotations

import math
from collections.abc import Sequence

from mg_msgs.msg import PoseGraphDiff

from slam_gnss_2d.core.data_types import GnssPrior, PoseEdge, PoseNode


def split_edges(
    all_edges: Sequence[PoseEdge],
    loop_edges: Sequence[PoseEdge],
) -> tuple[list[PoseEdge], list[PoseEdge]]:
    """全エッジを連続辺とループ辺に分離する。"""
    loop_edge_set = {(e.from_index, e.to_index) for e in loop_edges}
    seq_edges = [
        e for e in all_edges
        if (e.from_index, e.to_index) not in loop_edge_set
    ]
    return seq_edges, list(loop_edges)


def build_pose_graph_diff(
    *,
    nodes: Sequence[PoseNode],
    seq_edges: Sequence[PoseEdge] = (),
    loop_edges: Sequence[PoseEdge] = (),
    priors: Sequence[GnssPrior] = (),
    loop_closed: bool = False,
    full_refresh_needed: bool = False,
) -> PoseGraphDiff:
    """PoseGraphDiff メッセージへポーズグラフ情報を詰める。"""
    msg = PoseGraphDiff()
    msg.loop_closed = loop_closed
    msg.full_refresh_needed = full_refresh_needed

    for node in nodes:
        msg.new_node_indices.append(node.index)
        msg.new_node_x.append(float(node.x))
        msg.new_node_y.append(float(node.y))
        msg.new_node_yaw.append(float(node.yaw))
        msg.new_node_timestamps.append(float(node.timestamp))

    for edge in seq_edges:
        msg.seq_edge_from.append(edge.from_index)
        msg.seq_edge_to.append(edge.to_index)
        msg.seq_edge_score.append(float(getattr(edge, 'score', 0.0)))
        edge_type = 1 if getattr(edge, 'is_odom_fallback', False) else 0
        msg.seq_edge_type.append(edge_type)
        msg.seq_edge_info_diag.extend(_information_diag(edge))

    for prior in priors:
        msg.prior_node_indices.append(prior.node_index)
        if prior.information[0, 0] > 0:
            sigma = 1.0 / math.sqrt(prior.information[0, 0])
        else:
            sigma = -1.0
        msg.prior_sigma_m.append(float(sigma))
        msg.prior_gnss_status.append(0)

    for edge in loop_edges:
        msg.loop_edge_from.append(edge.from_index)
        msg.loop_edge_to.append(edge.to_index)
        msg.loop_edge_score.append(float(getattr(edge, 'score', 0.0)))
        msg.loop_edge_info_diag.extend(_information_diag(edge))

    return msg


def _information_diag(edge: PoseEdge) -> list[float]:
    return [
        float(edge.information[0, 0]),
        float(edge.information[1, 1]),
        float(edge.information[2, 2]),
    ]
