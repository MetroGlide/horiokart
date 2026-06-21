from __future__ import annotations

import logging
import math

import numpy as np
import cv2

from slam_gnss_2d.map_manager.base import MapRendererBase
from slam_gnss_2d.core.data_types import PoseNode

_logger = logging.getLogger(__name__)


class CountingRenderer(MapRendererBase):
    """ヒット・ミスカウンタ方式による占有格子マップ実装。

    各セルへのレーザーの通過回数（ミス数）と到達回数（ヒット数）を個別にカウントし、
    確率比率 (hit / (hit + miss)) に基づいて占有・空きを判定することで、
    ノイズや位置ズレによる地図情報の不適切な上書きを防ぐ。

    to_occupancy_array() で ROS2 OccupancyGrid 形式（-1 / 0 / 100）に変換して返す。
    """

    def __init__(
        self,
        resolution: float = 0.05,
        expansion_margin: float = 100.0,
        hit_threshold: float = 0.3,
    ) -> None:
        """
        Args:
            resolution: マップの解像度 [m/pixel]
            expansion_margin: 境界計算時に全ノード位置に追加するマージン [m]
            hit_threshold: 占有（Occupied）と判定するヒット数の比率閾値 [0.0 ~ 1.0]
        """
        self._resolution = resolution
        self._expansion_margin = expansion_margin
        self._hit_threshold = hit_threshold
        self._map_size = 1
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._hit_map = np.zeros((1, 1), dtype=np.int32)
        self._miss_map = np.zeros((1, 1), dtype=np.int32)
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
        xs = np.fromiter((n.x for n in nodes),
                         dtype=np.float64, count=len(nodes))
        ys = np.fromiter((n.y for n in nodes),
                         dtype=np.float64, count=len(nodes))
        new_origin_x = float(xs.min()) - self._expansion_margin
        new_origin_y = float(ys.min()) - self._expansion_margin
        new_max_x = float(xs.max()) + self._expansion_margin
        new_max_y = float(ys.max()) + self._expansion_margin
        new_size = max(
            math.ceil((new_max_x - new_origin_x) / self._resolution),
            math.ceil((new_max_y - new_origin_y) / self._resolution),
        )
        _logger.debug(
            f'Map recomputed (Counting): size={new_size}px '
            f'({new_size * self._resolution:.0f}m), '
            f'origin=({new_origin_x:.1f}, {new_origin_y:.1f})'
        )
        self._origin_x = new_origin_x
        self._origin_y = new_origin_y
        self._map_size = new_size
        self._hit_map = np.zeros((new_size, new_size), dtype=np.int32)
        self._miss_map = np.zeros((new_size, new_size), dtype=np.int32)
        self._render_count = 0
        for node in nodes:
            if node.scan is not None:
                self._render_node(node)

    def to_occupancy_array(self) -> tuple[np.ndarray, float, float, float]:
        total = self._hit_map + self._miss_map
        valid_mask = total > 0

        # 初期値は unknown (-1)
        data = np.full(self._hit_map.shape, -1, dtype=np.int8)

        if np.any(valid_mask):
            # ゼロ除算を避けるために有効なセルのみ計算
            ratio = np.zeros_like(self._hit_map, dtype=np.float32)
            ratio[valid_mask] = self._hit_map[valid_mask].astype(np.float32) / total[valid_mask]

            occupied_mask = valid_mask & (ratio >= self._hit_threshold)
            free_mask = valid_mask & (ratio < self._hit_threshold)

            data[occupied_mask] = 100
            data[free_mask] = 0

        return data, self._origin_x, self._origin_y, self._resolution

    def apply_trajectory_mask(self, nodes: list[PoseNode], radius_m: float, filter_type: str = 'clear') -> None:
        if not nodes:
            return

        radius_px = max(1, int(radius_m / self._resolution))
        pts = np.empty((1, len(nodes), 2), dtype=np.int32)
        for i, node in enumerate(nodes):
            px, py = self._world_to_pixel(node.x, node.y)
            pts[0, i, 0] = px
            pts[0, i, 1] = py

        mask = np.zeros(self._hit_map.shape, dtype=np.uint8)
        # 軌跡を描画。lineType=cv2.LINE_8
        cv2.polylines(mask, pts, isClosed=False, color=1, thickness=radius_px * 2)

        # 頂点の丸めのために各ノードに円も描画する（厚い線分では角が欠ける場合があるため）
        for i in range(len(nodes)):
            cv2.circle(mask, (int(pts[0, i, 0]), int(pts[0, i, 1])), radius_px, color=1, thickness=-1)

        mask_bool = mask > 0

        if filter_type == 'clear':
            self._hit_map[mask_bool] = 0
            self._miss_map[mask_bool] += 1
        elif filter_type == 'attenuate':
            # ヒットカウントを減衰させる（微小なヒットは0にする）
            self._hit_map[mask_bool] = np.maximum(0, self._hit_map[mask_bool] - 2)
            self._hit_map[mask_bool] //= 2
            
            # hitが減っても、missが0のままだと hit/(hit+miss) = 1.0 となり占有判定されてしまう。
            # ロボットの軌跡上である以上「空間が空いていた」という証拠でもあるため、missを追加する。
            self._miss_map[mask_bool] += 2

    def _world_to_pixel(self, wx: float, wy: float) -> tuple[int, int]:
        px = int((wx - self._origin_x) / self._resolution)
        py = int((wy - self._origin_y) / self._resolution)
        return px, py

    def _in_bounds(self, px: int, py: int) -> bool:
        return 0 <= px < self._map_size and 0 <= py < self._map_size

    def _render_node(self, node: PoseNode) -> None:
        scan = node.scan
        angles = scan.angle_min + \
            np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid_mask = (ranges > scan.range_min) & (ranges < scan.range_max)

        cos_yaw = np.cos(node.yaw)
        sin_yaw = np.sin(node.yaw)

        robot_px, robot_py = self._world_to_pixel(node.x, node.y)
        if not self._in_bounds(robot_px, robot_py):
            return

        self._render_count += 1

        # 有効点の座標変換を一括計算
        r_v = ranges[valid_mask]
        a_v = angles[valid_mask]
        lx = r_v * np.cos(a_v)
        ly = r_v * np.sin(a_v)
        wx = node.x + cos_yaw * lx - sin_yaw * ly
        wy = node.y + sin_yaw * lx + cos_yaw * ly

        hit_px = ((wx - self._origin_x) / self._resolution).astype(np.int32)
        hit_py = ((wy - self._origin_y) / self._resolution).astype(np.int32)
        in_bounds = (
            (hit_px >= 0) & (hit_px < self._map_size) &
            (hit_py >= 0) & (hit_py < self._map_size)
        )
        hit_px_valid = hit_px[in_bounds]
        hit_py_valid = hit_py[in_bounds]
        n_hits = len(hit_px_valid)

        if n_hits > 0:
            # 描画対象の最小バウンディングボックスを計算（高速化のため）
            min_px = int(min(robot_px, hit_px_valid.min()))
            max_px = int(max(robot_px, hit_px_valid.max()))
            min_py = int(min(robot_py, hit_py_valid.min()))
            max_py = int(max(robot_py, hit_py_valid.max()))

            h = max_py - min_py + 1
            w = max_px - min_px + 1

            # ローカルマスクの作成 (miss を 1 として描画)
            local_mask = np.zeros((h, w), dtype=np.uint8)
            pts = np.empty((n_hits, 2, 1, 2), dtype=np.int32)
            pts[:, 0, 0, 0] = robot_px - min_px
            pts[:, 0, 0, 1] = robot_py - min_py
            pts[:, 1, 0, 0] = hit_px_valid - min_px
            pts[:, 1, 0, 1] = hit_py_valid - min_py

            cv2.polylines(local_mask, pts, False, 1, 1)

            # ヒット点は miss から除外するため、ローカルマスク側で 0 に戻す
            local_hit_px = hit_px_valid - min_px
            local_hit_py = hit_py_valid - min_py
            local_mask[local_hit_py, local_hit_px] = 0

            # グローバルマップへの加算
            self._miss_map[min_py:max_py+1, min_px:max_px+1] += local_mask
            np.add.at(self._hit_map, (hit_py_valid, hit_px_valid), 1)

        if self._render_count == 1:
            _logger.debug(
                f'First render (Counting): robot=({node.x:.2f}, {node.y:.2f}), '
                f'hits={n_hits}, oob_hits={int((~in_bounds).sum())}'
            )
        elif self._render_count % 10 == 0:
            _logger.debug(
                f'Render #{self._render_count} (Counting): '
                f'robot=({node.x:.2f}, {node.y:.2f}), '
                f'hits={n_hits}, oob_hits={int((~in_bounds).sum())}'
            )
