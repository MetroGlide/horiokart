from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

from ..data_types import GnssData, PoseEdge, PoseNode
from .anchor_manager import GnssAnchorManager
from ..optimizer.isam2_optimizer import ISAM2Optimizer
import logging

_logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class GnssAnchoredParams:
    init_distance_m: float
    anchor_min_fix_status: int
    anchor_sigma_m: float
    init_yaw_sigma_rad: float
    gnss_fix_sigma_m: float
    gnss_float_sigma_m: float
    gnss_factor_yaw_variance: float
    # h_acc 導出のσ 上限。これを超える拘束はスキップする
    gnss_max_sigma_m: float = 2.0
    # 位置がこの閾値以上変化した場合、全ノードを更新しマップを再描画する
    gnss_rerender_threshold_m: float = 0.1


class GnssAnchoredRunner:
    """GNSSアンカー基準の1-phase SLAM制御（INITIALIZING→RUNNING）。"""

    def __init__(
        self,
        params: GnssAnchoredParams,
        optimizer: ISAM2Optimizer,
    ) -> None:
        self._params = params
        self._optimizer = optimizer
        self._anchor = GnssAnchorManager()
        self._state = 'INITIALIZING'
        self._latest_gnss: GnssData | None = None
        self._last_node_index = -1
        self._last_gnss_ts_used = float('-inf')

    @property
    def state(self) -> str:
        return self._state

    @property
    def anchor(self) -> tuple[float, float] | None:
        return self._anchor.anchor_utm

    def on_gnss(self, gnss: GnssData | None) -> bool:
        if gnss is None:
            return False
        self._latest_gnss = gnss
        return self._anchor.try_set_anchor(gnss, self._params.anchor_min_fix_status)

    def process(
        self,
        nodes: list[PoseNode],
        edges: list[PoseEdge],
        latest_node: PoseNode,
        latest_edge: PoseEdge | None,
    ) -> tuple[dict[int, tuple[float, float, float]], bool]:
        if not nodes:
            return {}, False
        if not self._anchor.is_initialized or self._latest_gnss is None:
            return {}, False

        if self._state == 'INITIALIZING':
            lx, ly = self._anchor.to_local(self._latest_gnss)
            dist = math.hypot(lx, ly)
            if dist < self._params.init_distance_m:
                _logger.debug(f'Waiting for initialization distance: {dist:.2f}m / {self._params.init_distance_m:.2f}m')
                return {}, False
            theta0 = math.atan2(ly, lx)
            self._initialize_graph(nodes, edges, theta0)
            self._state = 'RUNNING'
            self._last_node_index = nodes[-1].index
            self._add_gnss_prior_for_latest(nodes)
            self._optimizer.update()
            return self._optimizer.get_all_poses(), True

        if latest_edge is not None and latest_node.index > self._last_node_index:
            prev_pose = self._optimizer.get_pose(latest_edge.from_index)
            if prev_pose is None:
                return {}, False
            px, py, pyaw = prev_pose
            c = math.cos(pyaw)
            s = math.sin(pyaw)
            x = px + c * latest_edge.dx - s * latest_edge.dy
            y = py + s * latest_edge.dx + c * latest_edge.dy
            yaw = pyaw + latest_edge.dyaw
            self._optimizer.add_initial_estimate(latest_node.index, x, y, yaw)
            self._optimizer.add_between_factor(
                latest_edge.from_index,
                latest_edge.to_index,
                latest_edge.dx,
                latest_edge.dy,
                latest_edge.dyaw,
                latest_edge.information,
            )
            self._last_node_index = latest_node.index

        pre_update_pose = self._optimizer.get_pose(latest_node.index)

        prior_added = self._add_gnss_prior_for_latest(nodes)
        self._optimizer.update()
        
        post_update_pose = self._optimizer.get_pose(latest_node.index)
        if post_update_pose is None:
            return {}, False

        if prior_added and pre_update_pose is not None:
            dx = post_update_pose[0] - pre_update_pose[0]
            dy = post_update_pose[1] - pre_update_pose[1]
            dist = math.hypot(dx, dy)
            if dist > self._params.gnss_rerender_threshold_m:
                _logger.info(f'GNSS optimization caused a jump of {dist:.3f}m. Rerendering map.')
                return self._optimizer.get_all_poses(), True

        return {latest_node.index: post_update_pose}, False

    def _initialize_graph(
        self,
        nodes: list[PoseNode],
        edges: list[PoseEdge],
        theta0: float,
    ) -> None:
        node0 = nodes[0]
        rot = theta0 - node0.yaw
        c = math.cos(rot)
        s = math.sin(rot)

        self._optimizer.initialize(
            node_index=node0.index,
            x=0.0,
            y=0.0,
            theta=theta0,
            pos_sigma=self._params.anchor_sigma_m,
            yaw_sigma=self._params.init_yaw_sigma_rad,
        )

        transformed: dict[int, tuple[float, float, float]] = {}
        for n in nodes:
            dx = n.x - node0.x
            dy = n.y - node0.y
            x = c * dx - s * dy
            y = s * dx + c * dy
            yaw = n.yaw + rot
            transformed[n.index] = (x, y, yaw)

        for idx, pose in transformed.items():
            if idx == node0.index:
                continue
            self._optimizer.add_initial_estimate(
                idx, pose[0], pose[1], pose[2])

        for e in edges:
            self._optimizer.add_between_factor(
                e.from_index,
                e.to_index,
                e.dx,
                e.dy,
                e.dyaw,
                e.information,
            )

    def _add_gnss_prior_for_latest(self, nodes: list[PoseNode]) -> bool:
        gnss = self._latest_gnss
        if gnss is None:
            return False
        if gnss.timestamp <= self._last_gnss_ts_used:
            return False

        sigma_xy = self._sigma_from_covariance_or_status(gnss)
        if sigma_xy <= 0.0:
            _logger.warning('GNSS prior skipped: invalid sigma_xy <= 0')
            return False
        if sigma_xy > self._params.gnss_max_sigma_m:
            _logger.warning(
                f'GNSS prior skipped: sigma_xy {sigma_xy:.2f} > max {self._params.gnss_max_sigma_m:.2f}')
            return False

        gx, gy = self._anchor.to_local(gnss)
        node = self._nearest_node(nodes, gnss.timestamp)
        self._optimizer.add_gnss_prior(
            node.index,
            gx,
            gy,
            sigma_xy,
            self._params.gnss_factor_yaw_variance,
        )
        self._last_gnss_ts_used = gnss.timestamp
        _logger.info(
            f'GNSS prior added to node {node.index}: sigma_xy={sigma_xy:.2f}m')
        return True

    def _sigma_from_covariance_or_status(self, gnss: GnssData) -> float:
        """h_acc 導出の共分散が有効ならそのσを返す。
        共割散がゼロ行列の場合は fix_status に応じた固定値にフォールバックする。"""
        import math
        cov_xx = float(gnss.covariance[0, 0]
                       ) if gnss.covariance is not None else 0.0
        if cov_xx > 0.0:
            return math.sqrt(cov_xx)
        return self._sigma_from_status(gnss.fix_status)

    def _sigma_from_status(self, status: int) -> float:
        if status >= 2:
            return self._params.gnss_fix_sigma_m
        if status >= 0:
            return self._params.gnss_float_sigma_m
        return -1.0

    @staticmethod
    def _nearest_node(nodes: list[PoseNode], timestamp: float) -> PoseNode:
        timestamps = [n.timestamp for n in nodes]
        idx = bisect.bisect_left(timestamps, timestamp)
        if idx == 0:
            return nodes[0]
        if idx >= len(nodes):
            return nodes[-1]
        prev = nodes[idx - 1]
        next_ = nodes[idx]
        if abs(timestamp - prev.timestamp) <= abs(next_.timestamp - timestamp):
            return prev
        return next_
