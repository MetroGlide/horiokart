from __future__ import annotations

import logging
from abc import ABC, abstractmethod

from slam_gnss_2d.core.config import TrajectoryNoiseFilterConfig
from slam_gnss_2d.core.data_types import PoseNode
from slam_gnss_2d.map_manager.base import MapRendererBase

_logger = logging.getLogger(__name__)


class TrajectoryNoiseFilterBase(ABC):
    """ポーズグラフの軌跡周辺のノイズ除去を行うコンポーネントの基底インターフェース"""

    @abstractmethod
    def apply(self, renderer: MapRendererBase, nodes: list[PoseNode]) -> None:
        pass


class TrajectoryNoiseFilter(TrajectoryNoiseFilterBase):
    def __init__(self, config: TrajectoryNoiseFilterConfig) -> None:
        self._config = config

    def apply(self, renderer: MapRendererBase, nodes: list[PoseNode]) -> None:
        if not self._config.enabled or not nodes:
            return

        _logger.info(
            f"Applying trajectory noise filter (type={self._config.type}, radius={self._config.radius_m}m)")
        renderer.apply_trajectory_mask(
            nodes, self._config.radius_m, self._config.type)
