from __future__ import annotations

import logging
from typing import Sequence

import numpy as np
from gtsam import (
    BetweenFactorPose2,
    LevenbergMarquardtOptimizer,
    LevenbergMarquardtParams,
    NonlinearFactorGraph,
    Pose2,
    PriorFactorPose2,
    Values,
    noiseModel,
)

from .base import GraphOptimizerBase
from ..data_types import GnssPrior, PoseEdge, PoseNode

_logger = logging.getLogger(__name__)

# 最初のノードを固定するアンカー拘束の分散値 (x, y, yaw)
# 非常に小さい値で最初のノードを強く固定する
_ANCHOR_VARIANCES = np.array([1e-6, 1e-6, 1e-8])

# GNSS prior の yaw 分散値 — 山の大きさで x/y のみを拘束し yaw は自由にする
_GNSS_YAW_VARIANCE = 1e6  # [rad^2]


class GTSAMOptimizer(GraphOptimizerBase):
    """GTSAM LevenbergMarquardt による 2D ポーズグラフ最適化。"""

    def optimize(
        self,
        nodes: list[PoseNode],
        edges: list[PoseEdge],
        gnss_priors: Sequence[GnssPrior] = (),
    ) -> list[PoseNode]:
        if len(nodes) < 2 or not edges:
            return list(nodes)

        graph = NonlinearFactorGraph()
        initial = Values()

        for node in nodes:
            initial.insert(node.index, Pose2(node.x, node.y, node.yaw))

        # 最初のノードをアンカー固定（グラフのゲージ自由度を除去する）
        anchor = nodes[0]
        prior_noise = noiseModel.Diagonal.Variances(_ANCHOR_VARIANCES)
        graph.add(PriorFactorPose2(
            anchor.index,
            Pose2(anchor.x, anchor.y, anchor.yaw),
            prior_noise,
        ))

        for edge in edges:
            noise = noiseModel.Gaussian.Information(edge.information)
            graph.add(BetweenFactorPose2(
                edge.from_index,
                edge.to_index,
                Pose2(edge.dx, edge.dy, edge.dyaw),
                noise,
            ))

        # GNSS絶対位置拘束を PriorFactorPose2 として投入する
        # yaw 分散を大きく設定し、x/y のみをグローバル座標で拘束する
        for gnss_prior in gnss_priors:
            info_3x3 = np.zeros((3, 3))
            info_3x3[:2, :2] = gnss_prior.information
            info_3x3[2, 2] = 1.0 / _GNSS_YAW_VARIANCE
            gnss_noise = noiseModel.Gaussian.Information(info_3x3)
            node_initial = initial.atPose2(gnss_prior.node_index)
            graph.add(PriorFactorPose2(
                gnss_prior.node_index,
                Pose2(gnss_prior.x, gnss_prior.y, node_initial.theta()),
                gnss_noise,
            ))

        params = LevenbergMarquardtParams()
        params.setVerbosity('SILENT')
        result = LevenbergMarquardtOptimizer(graph, initial, params).optimize()

        updated: list[PoseNode] = []
        for node in nodes:
            pose = result.atPose2(node.index)
            updated.append(PoseNode(
                index=node.index,
                timestamp=node.timestamp,
                x=pose.x(),
                y=pose.y(),
                yaw=pose.theta(),
                scan=node.scan,
            ))

        _logger.info(
            f'GTSAMOptimizer: {len(nodes)} nodes, {len(edges)} edges, '
            f'{len(list(gnss_priors))} gnss_priors optimized'
        )
        return updated
