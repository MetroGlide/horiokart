#!/usr/bin/env python3
"""slam_gnss_2d ROS2 エントリポイント。

どのコンポーネント実装を組み合わせるかをここで決定する。
コアロジック（pose_graph / map_manager）は ROS に非依存。
"""
from __future__ import annotations

import logging

import rclpy

from slam_gnss_2d.config import SlamConfig
from slam_gnss_2d.input.base import OdomSourceBase, ScanSourceBase
from slam_gnss_2d.input.ros2.ros_adapter import ROS2OdomSource, ROS2ScanSource
from slam_gnss_2d.slam_node_base import SlamNodeBase


class SlamGnss2DNode(SlamNodeBase):
    def __init__(self) -> None:
        super().__init__('slam_gnss_2d_node')

    def _setup_io(self, cfg: SlamConfig) -> tuple[ScanSourceBase, OdomSourceBase]:
        return ROS2ScanSource(self, cfg.scan_topic), ROS2OdomSource(self, cfg.odom_topic)


def main(args=None):
    logging.basicConfig(level=logging.INFO,
                        format='%(name)s %(levelname)s: %(message)s')
    rclpy.init(args=args)
    node = SlamGnss2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
