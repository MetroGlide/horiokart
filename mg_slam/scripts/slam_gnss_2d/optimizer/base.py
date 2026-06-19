from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Sequence

from slam_gnss_2d.core.data_types import GnssPrior, PoseEdge, PoseNode


class GraphOptimizerBase(ABC):
    """ポーズグラフ最適化の抽象基底クラス。GTSAM 等に差し替え可能。"""

    @abstractmethod
    def optimize(
        self,
        nodes: list[PoseNode],
        edges: list[PoseEdge],
        gnss_priors: Sequence[GnssPrior] = (),
    ) -> list[PoseNode]:
        """グラフ最適化を実行し、更新された PoseNode のリストを返す。

        ノードの順序・インデックスは入力と同一であること。
        edges: 連続辺・ループ辺を含むポーズグラフの全拘束。GTSAM BetweenFactor として使用する。
        gnss_priors: GNSS 絶対位置拘束。空の場合は GNSS なしの最適化（Phase 1〜3 相当）。
        """
        raise NotImplementedError


class IncrementalOptimizerBase(ABC):
    """インクリメンタル最適化器の抽象基底クラス。"""

    @abstractmethod
    def initialize(
        self,
        node_index: int,
        x: float,
        y: float,
        theta: float,
        pos_sigma: float,
        yaw_sigma: float,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def add_between_factor(
        self,
        from_index: int,
        to_index: int,
        dx: float,
        dy: float,
        dyaw: float,
        information,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def add_gnss_prior(
        self,
        node_index: int,
        x: float,
        y: float,
        sigma_xy: float,
        yaw_variance: float,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def add_initial_estimate(
        self,
        node_index: int,
        x: float,
        y: float,
        theta: float,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def update(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_pose(self, node_index: int) -> tuple[float, float, float] | None:
        raise NotImplementedError

    @abstractmethod
    def get_all_poses(self) -> dict[int, tuple[float, float, float]]:
        raise NotImplementedError
