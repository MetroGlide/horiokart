"""on_reached_actions を別スレッドで順次実行するエグゼキュータ"""
from __future__ import annotations

import threading
from typing import Callable, List, TYPE_CHECKING

if TYPE_CHECKING:
    import rclpy.node
    from mg_waypoint_navigation.waypoint import ActionConfig


class ActionExecutor:
    """アクションリストを別スレッドで順次実行し、完了時にコールバックを呼ぶ"""

    def __init__(self, node: "rclpy.node.Node"):
        self._node = node
        self._thread: threading.Thread | None = None

    def execute(
        self,
        actions: List["ActionConfig"],
        done_callback: Callable[[], None],
    ) -> None:
        from mg_waypoint_navigation.waypoint_sequencer.actions import build_action

        built = [build_action(a, self._node) for a in actions]

        def _run():
            for action in built:
                try:
                    action.execute()
                except Exception as e:
                    self._node.get_logger().error(
                        f"Action {action} raised exception: {e}"
                    )
            done_callback()

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()
