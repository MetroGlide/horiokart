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


class SetNavigationModeAction(BaseAction):
    """ナビゲーションモード（normal / queue_wait等）を切り替え、
    必要なNav2パラメータ（global_costmapの障害物レイヤー等）を動的に変更する。"""

    def execute(self) -> None:
        import rclpy
        from rcl_interfaces.srv import SetParametersAtomically
        from rcl_interfaces.msg import Parameter, ParameterType

        mode = self._config.mode
        client = self._node.create_client(
            SetParametersAtomically,
            "/global_costmap/global_costmap/set_parameters_atomically"
        )
        if client.wait_for_service(timeout_sec=5.0):
            req = SetParametersAtomically.Request()

            # queue_waitの場合はグローバルコストマップの動的障害物を無視してパスを引かせる
            enabled = (mode != "queue_wait")

            p1 = Parameter()
            p1.name = "top_obstacle_layer.enabled"
            p1.value.type = ParameterType.PARAMETER_BOOL
            p1.value.bool_value = enabled

            p2 = Parameter()
            p2.name = "obstacle_stvl_layer.enabled"
            p2.value.type = ParameterType.PARAMETER_BOOL
            p2.value.bool_value = enabled

            req.parameters = [p1, p2]

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)

            if future.result() is not None and future.result().result.successful:
                self._node.get_logger().info(f"Set navigation mode to '{mode}' (global obstacles enabled: {enabled})")
            else:
                self._node.get_logger().error(f"Failed to set navigation mode to '{mode}'")
        else:
            self._node.get_logger().error("global_costmap set_parameters_atomically service not available")
