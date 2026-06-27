from __future__ import annotations

import logging

import numpy as np
import cv2

from slam_gnss_2d.map_manager.base import MapRendererBase
from slam_gnss_2d.core.data_types import PoseNode
from slam_gnss_2d.map_manager.grid_utils import (
    build_trajectory_mask,
    compute_square_bounds,
    in_bounds,
    scan_hits_to_pixels,
    world_to_pixel,
)

_logger = logging.getLogger(__name__)


class OverwriteRenderer(MapRendererBase):
    """OpenCV cv2.line を使った Ray-Casting による占有格子マップ（上書き方式）実装。

    内部マップは uint8 で管理する:
        128 = unknown
        255 = free（cv2.line で空き領域を描画）
          0 = occupied（ヒット点を上書き）

    to_occupancy_array() で ROS2 OccupancyGrid 形式（-1 / 0 / 100）に変換して返す。
    マップの境界は rerender_all() が呼ばれるたびに全ノードから動的に計算される。
    """

    def __init__(
        self,
        resolution: float,
        expansion_margin: float,
    ) -> None:
        """
        Args:
            resolution: マップの解像度 [m/pixel]
            expansion_margin: 境界計算時に全ノード位置に追加するマージン [m]
        """
        self._resolution = resolution
        self._expansion_margin = expansion_margin
        self._map_size = 1
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._map = np.full((1, 1), 128, dtype=np.uint8)
        self._render_count = 0

    def add_node(self, node: PoseNode) -> bool:
        if node.scan is None:
            return True
        robot_px, robot_py = self._world_to_pixel(node.x, node.y)
        if not self._in_bounds(robot_px, robot_py):
            return False
        self._render_node(node)
        return True

    def rerender_all(self, nodes: list[PoseNode]) -> None:
        if not nodes:
            return
        new_origin_x, new_origin_y, new_size = compute_square_bounds(
            nodes,
            self._resolution,
            self._expansion_margin,
        )
        _logger.debug(
            f'Map recomputed (Overwrite): size={new_size}px '
            f'({new_size * self._resolution:.0f}m), '
            f'origin=({new_origin_x:.1f}, {new_origin_y:.1f})'
        )
        self._origin_x = new_origin_x
        self._origin_y = new_origin_y
        self._map_size = new_size
        self._map = np.full((new_size, new_size), 128, dtype=np.uint8)
        self._render_count = 0
        for node in nodes:
            if node.scan is not None:
                self._render_node(node)

    def to_occupancy_array(self) -> tuple[np.ndarray, float, float, float]:
        data = np.where(
            self._map == 128, -1,
            np.where(self._map == 255, 0, 100)
        ).astype(np.int8)
        return data, self._origin_x, self._origin_y, self._resolution

    def apply_trajectory_mask(self, nodes: list[PoseNode], radius_m: float, filter_type: str = 'clear') -> None:
        if not nodes:
            return

        mask_bool = build_trajectory_mask(
            self._map.shape,
            nodes,
            radius_m,
            self._origin_x,
            self._origin_y,
            self._resolution,
        )
        if filter_type == 'clear':
            self._map[mask_bool] = 255
        elif filter_type == 'attenuate':
            _logger.warning(
                "filter_type='attenuate' is not fully supported in OverwriteRenderer.")
            # 上書き方式では確率の減衰ができないため、暫定的にクリアする
            self._map[mask_bool] = 255

    def _world_to_pixel(self, wx: float, wy: float) -> tuple[int, int]:
        return world_to_pixel(
            wx, wy, self._origin_x, self._origin_y, self._resolution)

    def _in_bounds(self, px: int, py: int) -> bool:
        return in_bounds(px, py, self._map_size)

    def _render_node(self, node: PoseNode) -> None:
        robot_px, robot_py, hit_px, hit_py, in_bounds_mask = scan_hits_to_pixels(
            node,
            self._origin_x,
            self._origin_y,
            self._resolution,
            self._map_size,
        )
        if not self._in_bounds(robot_px, robot_py):
            return

        self._render_count += 1

        hit_px_valid = hit_px[in_bounds_mask]
        hit_py_valid = hit_py[in_bounds_mask]
        n_hits = len(hit_px_valid)

        if n_hits > 0:
            # free ライン描画: N 本の 2 点ラインを cv2.polylines で C++ 側に一括委譲
            # shape (N, 2, 1, 2): pts[i] は (start, end) の 2 点ポリライン
            pts = np.empty((n_hits, 2, 1, 2), dtype=np.int32)
            pts[:, 0, 0, 0] = robot_px
            pts[:, 0, 0, 1] = robot_py
            pts[:, 1, 0, 0] = hit_px_valid
            pts[:, 1, 0, 1] = hit_py_valid
            cv2.polylines(self._map, pts, False, 255, 1)

            # occupied 点を一括で書き込む
            self._map[hit_py_valid, hit_px_valid] = 0

        if self._render_count == 1:
            _logger.debug(
                f'First render (Overwrite): robot=({node.x:.2f}, {node.y:.2f}), '
                f'hits={n_hits}, oob_hits={int((~in_bounds_mask).sum())}'
            )
        elif self._render_count % 10 == 0:
            _logger.debug(
                f'Render #{self._render_count} (Overwrite): '
                f'robot=({node.x:.2f}, {node.y:.2f}), '
                f'hits={n_hits}, oob_hits={int((~in_bounds_mask).sum())}'
            )
