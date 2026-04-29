"""importlib による汎用サービスコール / トピックパブリッシュアクション"""
from __future__ import annotations

import importlib
from typing import TYPE_CHECKING

from mg_waypoint_navigation.waypoint_sequencer.actions.base import BaseAction

if TYPE_CHECKING:
    import rclpy.node
    from mg_waypoint_navigation.waypoint import ActionConfig


class GenericServiceAction(BaseAction):
    """YAML で指定したサービスを呼び出す"""

    def __init__(self, config: "ActionConfig", node: "rclpy.node.Node"):
        super().__init__(config, node)
        module = importlib.import_module(config.srv_module)
        srv_type = getattr(module, config.srv_class)
        self._client = node.create_client(srv_type, config.service)
        self._srv_type = srv_type

    def execute(self) -> None:
        timeout = 5.0
        if not self._client.wait_for_service(timeout_sec=timeout):
            self._node.get_logger().error(
                f"Service {self._config.service} not available after {timeout}s"
            )
            return

        request = self._srv_type.Request()
        for key, value in self._config.request.items():
            setattr(request, key, value)

        future = self._client.call_async(request)
        import rclpy
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout)
        if future.result() is None:
            self._node.get_logger().error(
                f"Service call to {self._config.service} timed out"
            )


class GenericPublishAction(BaseAction):
    """YAML で指定したトピックにメッセージを1回パブリッシュする"""

    def __init__(self, config: "ActionConfig", node: "rclpy.node.Node"):
        super().__init__(config, node)
        module = importlib.import_module(config.msg_module)
        msg_type = getattr(module, config.msg_class)
        self._publisher = node.create_publisher(msg_type, config.topic, 1)
        self._msg_type = msg_type

    def execute(self) -> None:
        msg = self._msg_type()
        for key, value in self._config.data.items():
            setattr(msg, key, value)
        self._publisher.publish(msg)
        self._node.get_logger().debug(
            f"Published to {self._config.topic}: {self._config.data}"
        )
