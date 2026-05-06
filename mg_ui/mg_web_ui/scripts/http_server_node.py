#!/usr/bin/env python3
"""dist/ ディレクトリを HTTP で静的配信する ROS2 ノード。SPA ルーティング対応。"""
import os
import threading
import http.server
import functools
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class SpaHandler(http.server.SimpleHTTPRequestHandler):
    """存在しないパスは index.html にフォールバックする SPA 対応ハンドラ。"""

    def do_GET(self):
        path = self.translate_path(self.path)
        if not os.path.exists(path) or os.path.isdir(path):
            self.path = '/index.html'
        super().do_GET()


class HttpServerNode(Node):
    def __init__(self):
        super().__init__('http_server_node')

        self.declare_parameter('port', 8080)
        self.declare_parameter('dist_dir', '')

        port: int = self.get_parameter(
            'port').get_parameter_value().integer_value
        dist_dir_param: str = self.get_parameter(
            'dist_dir').get_parameter_value().string_value

        if dist_dir_param:
            dist_dir = Path(dist_dir_param)
        else:
            share_dir = Path(get_package_share_directory('mg_web_ui'))
            dist_dir = share_dir / 'frontend' / 'dist'

        if not dist_dir.exists():
            self.get_logger().error(f'dist dir not found: {dist_dir}')
            raise FileNotFoundError(str(dist_dir))

        handler = functools.partial(SpaHandler, directory=str(dist_dir))
        self._server = http.server.ThreadingHTTPServer(('', port), handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(f'serving {dist_dir} on port {port}')

    def destroy_node(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HttpServerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
