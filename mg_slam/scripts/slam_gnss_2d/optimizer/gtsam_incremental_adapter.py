from __future__ import annotations

import logging
import numpy as np

from slam_gnss_2d.optimizer.base import IncrementalOptimizerBase
from slam_gnss_2d.optimizer.gtsam_optimizer import GTSAMOptimizer
from slam_gnss_2d.core.data_types import GnssPrior, PoseEdge, PoseNode

_logger = logging.getLogger(__name__)


class GTSAMIncrementalAdapter(IncrementalOptimizerBase):
    """GTSAMOptimizer (バッチLM法) をインクリメンタルなインターフェースでラップするアダプタ。"""

    def __init__(self) -> None:
        self._optimizer = GTSAMOptimizer()
        self._nodes: dict[int, tuple[float, float, float]] = {}
        self._edges: list[PoseEdge] = []
        self._priors: list[GnssPrior] = []
        self._latest_result: dict[int, tuple[float, float, float]] = {}
        self._initialized = False

    def initialize(
        self,
        node_index: int,
        x: float,
        y: float,
        theta: float,
        pos_sigma: float,
        yaw_sigma: float,
    ) -> None:
        self._nodes.clear()
        self._edges.clear()
        self._priors.clear()
        self._latest_result.clear()
        self._nodes[node_index] = (x, y, theta)
        self._latest_result[node_index] = (x, y, theta)
        self._initialized = True

    def add_between_factor(
        self,
        from_index: int,
        to_index: int,
        dx: float,
        dy: float,
        dyaw: float,
        information,
    ) -> None:
        if not self._initialized:
            return
        self._edges.append(PoseEdge(
            from_index=from_index,
            to_index=to_index,
            dx=dx,
            dy=dy,
            dyaw=dyaw,
            information=information,
        ))

    def add_gnss_prior(
        self,
        node_index: int,
        x: float,
        y: float,
        sigma_xy: float,
        yaw_variance: float,
    ) -> None:
        if not self._initialized:
            return
        info_2x2 = np.zeros((2, 2), dtype=np.float64)
        inv_var = 1.0 / max(sigma_xy * sigma_xy, 1e-12)
        info_2x2[0, 0] = inv_var
        info_2x2[1, 1] = inv_var
        self._priors.append(GnssPrior(
            node_index=node_index,
            x=x,
            y=y,
            information=info_2x2,
        ))

    def add_initial_estimate(
        self,
        node_index: int,
        x: float,
        y: float,
        theta: float,
    ) -> None:
        if not self._initialized:
            return
        if node_index not in self._nodes:
            self._nodes[node_index] = (x, y, theta)
            self._latest_result[node_index] = (x, y, theta)

    def update(self) -> None:
        if not self._initialized:
            return

        nodes_list = []
        for idx in sorted(self._nodes.keys()):
            x, y, yaw = self._nodes[idx]
            nodes_list.append(PoseNode(
                index=idx,
                timestamp=0.0,
                x=x,
                y=y,
                yaw=yaw,
                scan=None,
            ))

        updated_nodes = self._optimizer.optimize(nodes_list, self._edges, self._priors)
        for n in updated_nodes:
            self._latest_result[n.index] = (n.x, n.y, n.yaw)
            self._nodes[n.index] = (n.x, n.y, n.yaw)

    def get_pose(self, node_index: int) -> tuple[float, float, float] | None:
        return self._latest_result.get(node_index)

    def get_all_poses(self) -> dict[int, tuple[float, float, float]]:
        return dict(self._latest_result)
