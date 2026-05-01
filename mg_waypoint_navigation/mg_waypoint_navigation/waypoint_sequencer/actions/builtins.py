"""組み込みアクション: load_map, amcl_reset, wait"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

from mg_waypoint_navigation.waypoint_sequencer.actions.base import BaseAction

if TYPE_CHECKING:
    import rclpy.node
    from mg_waypoint_navigation.waypoint import ActionConfig


class LoadMapAction(BaseAction):
    """測位マップ / 計画マップを map_server にロードする"""

    def execute(self) -> None:
        import rclpy
        from nav2_msgs.srv import LoadMap

        if self._config.localization:
            client = self._node.create_client(LoadMap, "/map_server/load_map")
            if client.wait_for_service(timeout_sec=5.0):
                req = LoadMap.Request()
                req.map_url = self._config.localization
                future = client.call_async(req)
                rclpy.spin_until_future_complete(
                    self._node, future, timeout_sec=10.0)
            else:
                self._node.get_logger().error("LoadMap service not available")

        if self._config.planning:
            client = self._node.create_client(
                LoadMap, "/planning_map_server/load_map")
            if client.wait_for_service(timeout_sec=5.0):
                req = LoadMap.Request()
                req.map_url = self._config.planning
                future = client.call_async(req)
                rclpy.spin_until_future_complete(
                    self._node, future, timeout_sec=10.0)
            else:
                self._node.get_logger().error("Planning LoadMap service not available")


class AmclResetAction(BaseAction):
    """AMCL のパーティクルフィルタをリセットする"""

    def execute(self) -> None:
        import rclpy
        from std_srvs.srv import Empty

        client = self._node.create_client(
            Empty, "/reinitialize_global_localization")
        if client.wait_for_service(timeout_sec=5.0):
            future = client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=5.0)
        else:
            self._node.get_logger().error(
                "reinitialize_global_localization service not available"
            )


class WaitAction(BaseAction):
    """countdown_ms ミリ秒待機する"""

    def execute(self) -> None:
        ms = self._config.countdown_ms
        if ms > 0:
            self._node.get_logger().info(f"WaitAction: waiting {ms} ms")
            time.sleep(ms / 1000.0)


class WaitTriggerAction(BaseAction):
    """外部トリガー（start()）待ちに移行するアクション。execute() 自体は何もしない。
    FSM が on_reached_actions に wait_trigger を検出した時点で IDLE へ遷移し、
    次の start() 呼び出しまで待機する。"""

    def execute(self) -> None:
        pass
