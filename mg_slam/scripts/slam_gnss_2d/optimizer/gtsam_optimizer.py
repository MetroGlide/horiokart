from __future__ import annotations

import logging

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
from ..data_types import PoseEdge, PoseNode

_logger = logging.getLogger(__name__)

# 最初のノードを固定するアンカー拘束の分散値 (x, y, yaw)
# 非常に小さい値で最初のノードを強く固定する
_ANCHOR_VARIANCES = np.array([1e-6, 1e-6, 1e-8])


class GTSAMOptimizer(GraphOptimizerBase):
    """GTSAM LevenbergMarquardt による 2D ポーズグラフ最適化。"""

    def optimize(
        self,
        nodes: list[PoseNode],
        edges: list[PoseEdge],
    ) -> list[PoseNode]:
        if len(nodes) < 2 or not edges:
            return list(nodes)

        graph = NonlinearFactorGraph()
        initial = Values()

        # 最初のノードをアンカー固定（グラフのゲージ自由度を除去する）
        anchor = nodes[0]
        prior_noise = noiseModel.Diagonal.Variances(_ANCHOR_VARIANCES)
        graph.add(PriorFactorPose2(
            anchor.index,
            Pose2(anchor.x, anchor.y, anchor.yaw),
            prior_noise,
        ))

        for node in nodes:
            initial.insert(node.index, Pose2(node.x, node.y, node.yaw))

        for edge in edges:
            noise = noiseModel.Gaussian.Information(edge.information)
            graph.add(BetweenFactorPose2(
                edge.from_index,
                edge.to_index,
                Pose2(edge.dx, edge.dy, edge.dyaw),
                noise,
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
            f'GTSAMOptimizer: {len(nodes)} nodes, {len(edges)} edges optimized'
        )
        return updated
