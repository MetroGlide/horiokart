#!/usr/bin/env python3
"""slam_gnss_2d Phase 1 ROS2 エントリポイント。

どのコンポーネント実装を組み合わせるかをここで決定する。
コアロジック（pose_graph / map_manager）は ROS に非依存。
"""
from __future__ import annotations

import logging
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node

from slam_gnss_2d.data_types import ScanData
from slam_gnss_2d.input.ros2.ros_adapter import ROS2OdomSource, ROS2ScanSource
from slam_gnss_2d.map_manager.opencv_renderer import OpenCVRenderer
from slam_gnss_2d.pose_graph.odom_builder import OdomOnlyBuilder


class SlamGnss2DNode(Node):
    def __init__(self) -> None:
        super().__init__('slam_gnss_2d_node')
        self._declare_params()

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        resolution = self.get_parameter('map_resolution').value
        map_size = self.get_parameter('map_size').value
        origin_x = self.get_parameter('map_origin_x').value
        origin_y = self.get_parameter('map_origin_y').value
        min_trans = self.get_parameter('min_translation').value
        min_rot = self.get_parameter('min_rotation').value
        map_publish_hz = self.get_parameter('map_publish_hz').value

        # 差し替えポイント: ここの実装クラスを差し替えるだけで動作が変わる
        self._scan_source = ROS2ScanSource(self, scan_topic)
        self._odom_source = ROS2OdomSource(self, odom_topic)
        self._pose_graph = OdomOnlyBuilder(
            min_translation=min_trans,
            min_rotation=min_rot,
        )
        self._renderer = OpenCVRenderer(
            resolution=resolution,
            map_size=map_size,
            origin_x=origin_x,
            origin_y=origin_y,
        )

        self._map_pub = self.create_publisher(
            OccupancyGrid, 'slam_gnss_2d/map', 1)
        self._path_pub = self.create_publisher(Path, 'slam_gnss_2d/path', 1)

        self._path_msg = Path()
        self._path_msg.header.frame_id = 'odom'
        self._map_dirty = False

        self._scan_source.set_scan_callback(self._on_scan)
        self._odom_source.start()
        self._scan_source.start()

        self.create_timer(1.0 / map_publish_hz, self._publish_map_timer)

        self._scan_recv_count = 0
        self._odom_miss_count = 0
        self._node_count = 0
        self._last_stat_time = time.monotonic()

        self.get_logger().info(
            f'slam_gnss_2d_node started (Phase 1: OdomOnly)\n'
            f'  scan: {scan_topic}, odom: {odom_topic}\n'
            f'  map: {map_size}x{map_size}px @ {resolution}m/px, '
            f'origin=({origin_x}, {origin_y})'
        )

    def _declare_params(self) -> None:
        self.declare_parameter('scan_topic', '/scan_top_lidar')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_size', 2000)
        self.declare_parameter('map_origin_x', -50.0)
        self.declare_parameter('map_origin_y', -50.0)
        self.declare_parameter('min_translation', 0.3)
        self.declare_parameter('min_rotation', 0.1)
        self.declare_parameter('map_publish_hz', 1.0)

    def _on_scan(self, scan: ScanData) -> None:
        self._scan_recv_count += 1
        odom = self._odom_source.get_odom_at(scan.timestamp)
        if odom is None:
            self._odom_miss_count += 1
            self.get_logger().warn(
                f'No odom for scan ts={scan.timestamp:.3f} (miss #{self._odom_miss_count})'
            )
            return

        node = self._pose_graph.add_scan(scan, odom)
        if node is None:
            self.get_logger().debug(
                f'Scan #{self._scan_recv_count} rejected: '
                f'below threshold at ({odom.x:.2f}, {odom.y:.2f})'
            )
            return

        self._node_count += 1
        if self._node_count == 1 or self._node_count % 10 == 0:
            self.get_logger().info(
                f'Node #{node.index}: x={node.x:.2f} y={node.y:.2f} '
                f'yaw={math.degrees(node.yaw):.1f}deg'
            )

        self._renderer.add_node(node)
        self._map_dirty = True
        self._publish_path(node)

    def _publish_map_timer(self) -> None:
        now = time.monotonic()
        if now - self._last_stat_time >= 30.0:
            self.get_logger().info(
                f'[stat] nodes={self._node_count}, '
                f'scans={self._scan_recv_count}, '
                f'odom_miss={self._odom_miss_count}'
            )
            self._last_stat_time = now

        if not self._map_dirty:
            return
        self._map_dirty = False

        data, origin_x, origin_y, resolution = self._renderer.to_occupancy_array()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = resolution
        msg.info.width = int(data.shape[1])
        msg.info.height = int(data.shape[0])
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.data = data.flatten().tolist()
        self._map_pub.publish(msg)
        self.get_logger().debug(
            f'Map published: occupied={int((data == 100).sum())}, '
            f'free={int((data == 0).sum())} px'
        )

    def _publish_path(self, node) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = node.x
        pose.pose.position.y = node.y
        pose.pose.orientation.w = math.cos(node.yaw / 2.0)
        pose.pose.orientation.z = math.sin(node.yaw / 2.0)
        self._path_msg.header.stamp = pose.header.stamp
        self._path_msg.poses.append(pose)
        self._path_pub.publish(self._path_msg)

    def destroy_node(self) -> None:
        self._scan_source.stop()
        self._odom_source.stop()
        super().destroy_node()


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
