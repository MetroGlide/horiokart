"""アクション基底クラス"""
from __future__ import annotations

import abc
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import rclpy.node
    from mg_waypoint_navigation.waypoint import ActionConfig


class BaseAction(abc.ABC):
    """on_reached_actions の各アクションが実装するインターフェース"""

    def __init__(self, config: "ActionConfig", node: "rclpy.node.Node"):
        self._config = config
        self._node = node

    @abc.abstractmethod
    def execute(self) -> None:
        """アクションを実行する。同期的に完了すること。"""
        ...

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(type={self._config.type})"
