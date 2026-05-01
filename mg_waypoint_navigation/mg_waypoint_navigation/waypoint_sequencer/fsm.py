"""ウェイポイントシーケンサーの有限状態機械"""
from __future__ import annotations

import threading
import time
from typing import Callable, Dict, List, Optional

import rclpy.node

from mg_waypoint_navigation.waypoint import WaypointList
from mg_waypoint_navigation.waypoint_sequencer.action_executor import ActionExecutor
from mg_waypoint_navigation.waypoint_sequencer.navigator import (
    NavigationResult,
    WaypointNavigator,
)
from mg_waypoint_navigation.waypoint_sequencer.states import (
    ALLOWED_TRANSITIONS,
    CommandResult,
    SequencerState,
)


class CountdownTimer:
    """指定時間後にコールバックを呼ぶ一発タイマー。スレッドセーフ。"""

    def __init__(self, on_done: Callable[[], None]):
        self._on_done = on_done
        self._timer: Optional[threading.Timer] = None
        self._duration_ms: int = 0
        self._start: float = 0.0

    def start(self, duration_ms: int) -> None:
        self.cancel()
        self._duration_ms = max(0, duration_ms)
        self._start = time.monotonic()
        self._timer = threading.Timer(
            self._duration_ms / 1000.0, self._on_done)
        self._timer.daemon = True
        self._timer.start()

    def cancel(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    @property
    def remaining_ms(self) -> int:
        if self._timer is None:
            return 0
        elapsed_ms = (time.monotonic() - self._start) * 1000
        return max(0, int(self._duration_ms - elapsed_ms))


class PauseSlotManager:
    """Named Pause Slot を管理する。"""

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._slots: Dict[str, float] = {}

    @property
    def is_active(self) -> bool:
        return bool(self._slots)

    @property
    def requesters(self) -> List[str]:
        return list(self._slots.keys())

    def add(self, requester_id: str, heartbeat_period_s: float) -> None:
        self._slots[requester_id] = heartbeat_period_s

    def remove(self, requester_id: str) -> None:
        self._slots.pop(requester_id, None)

    def clear_all(self) -> None:
        self._slots.clear()


class WaypointSequencerFSM:
    """
    SequencerState の遷移と副作用を管理するFSM。

    - 各遷移中状態(ON_STARTING/ON_ARRIVING)への入場は専用エントリー関数経由のみ。
    - 処理完了後は内部コールバックで次の安定状態へ移行する。
    - 外部から _transition() を呼ばない。
    - IDLE は「未開始」と「途中ウェイポイントのトリガー待ち」を兼ねる（_current_index で区別）。
    """

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._navigator = WaypointNavigator(node)
        self._executor = ActionExecutor(node)

        self._state = SequencerState.IDLE
        self._lock = threading.RLock()

        self._waypoints: WaypointList = WaypointList()
        self._current_index: int = 0

        self._stop_pending: bool = False
        self._pause_pending: bool = False
        self._pre_suspend_state: SequencerState = SequencerState.IDLE
        self._saved_countdown_ms: int = 0

        self._countdown_timer = CountdownTimer(self._on_starting_done)
        self._pause_manager = PauseSlotManager(node)

        self._on_state_changed: Optional[Callable[[
            SequencerState], None]] = None

    # ------------------------------------------------------------------
    # 外部コールバック登録
    # ------------------------------------------------------------------

    def set_on_state_changed(self, cb: Callable[[SequencerState], None]) -> None:
        self._on_state_changed = cb

    # ------------------------------------------------------------------
    # 状態参照
    # ------------------------------------------------------------------

    @property
    def state(self) -> SequencerState:
        return self._state

    @property
    def current_index(self) -> int:
        return self._current_index

    @property
    def total_waypoints(self) -> int:
        return self._waypoints.get_size()

    @property
    def countdown_ms_remaining(self) -> int:
        if self._state != SequencerState.ON_STARTING:
            return 0
        return self._countdown_timer.remaining_ms

    @property
    def pause_requesters(self) -> List[str]:
        return self._pause_manager.requesters

    @property
    def distance_remaining(self) -> float:
        return self._navigator.distance_remaining

    # ------------------------------------------------------------------
    # ウェイポイント設定
    # ------------------------------------------------------------------

    def load_waypoints(self, waypoints: WaypointList) -> None:
        with self._lock:
            self._waypoints = waypoints
            self._current_index = 0

    def set_next_index(self, index: int) -> bool:
        """IDLE / SUSPENDED 時のみインデックスを変更できる"""
        with self._lock:
            if self._state not in (SequencerState.IDLE, SequencerState.SUSPENDED):
                return False
            if index < 0 or index >= self._waypoints.get_size():
                return False
            self._current_index = index
            return True

    # ------------------------------------------------------------------
    # 外部コマンド
    # ------------------------------------------------------------------

    def start(self, countdown_ms: int) -> CommandResult:
        with self._lock:
            if self._state == SequencerState.IDLE:
                if self._waypoints.get_size() == 0:
                    return CommandResult(False, "No waypoints loaded")
                self._enter_on_starting(countdown_ms)
                return CommandResult(True, "OK")

            if self._state == SequencerState.GOAL_REACHED:
                self._current_index = 0
                self._enter_on_starting(countdown_ms)
                return CommandResult(True, "OK")

            return CommandResult(False, f"Cannot start from state {self._state.value}")

    def stop(self) -> CommandResult:
        with self._lock:
            if self._state == SequencerState.IDLE:
                return CommandResult(True, "Already idle")

            if self._state == SequencerState.ON_STARTING:
                self._countdown_timer.cancel()
                self._pause_manager.clear_all()
                self._transition(SequencerState.IDLE)
                return CommandResult(True, "OK")

            if self._state == SequencerState.NAVIGATING:
                self._navigator.cancel()
                self._pause_manager.clear_all()
                self._transition(SequencerState.IDLE)
                return CommandResult(True, "OK")

            if self._state == SequencerState.ON_ARRIVING:
                self._stop_pending = True
                return CommandResult(True, "Stop deferred until actions complete")

            if self._state in (
                SequencerState.SUSPENDED,
                SequencerState.GOAL_REACHED,
                SequencerState.ERROR,
            ):
                self._pause_manager.clear_all()
                self._transition(SequencerState.IDLE)
                return CommandResult(True, "OK")

            return CommandResult(False, f"Unhandled state {self._state.value}")

    def pause_request(
        self,
        requester_id: str,
        active: bool,
        heartbeat_period_s: float,
        reason: str = "",
    ) -> None:
        with self._lock:
            if active:
                self._pause_manager.add(requester_id, heartbeat_period_s)
                self._apply_pause()
            else:
                self._pause_manager.remove(requester_id)
                self._try_resume()

    # ------------------------------------------------------------------
    # エントリー関数（各状態への唯一の入口）
    # ------------------------------------------------------------------

    def _enter_on_starting(self, countdown_ms: int) -> None:
        self._transition(SequencerState.ON_STARTING)
        self._countdown_timer.start(countdown_ms)

    def _enter_navigating(self) -> None:
        waypoint = self._waypoints.get(self._current_index)
        self._transition(SequencerState.NAVIGATING)
        self._navigator.send_goal(waypoint, self._on_navigation_result)

    def _enter_on_arriving(self, actions) -> None:
        self._transition(SequencerState.ON_ARRIVING)
        self._executor.execute(actions, self._on_arriving_done)

    # ------------------------------------------------------------------
    # 内部完了コールバック
    # ------------------------------------------------------------------

    def _on_starting_done(self) -> None:
        with self._lock:
            if self._state != SequencerState.ON_STARTING:
                return
            self._enter_navigating()

    def _on_navigation_result(self, result: NavigationResult) -> None:
        with self._lock:
            if self._state != SequencerState.NAVIGATING:
                return

            if result == NavigationResult.CANCELED:
                return

            if result != NavigationResult.SUCCEEDED:
                self._node.get_logger().error(
                    f"Navigation to waypoint {self._current_index} failed"
                )
                self._transition(SequencerState.ERROR)
                return

            waypoint = self._waypoints.get(self._current_index)
            if waypoint.on_reached_actions:
                self._enter_on_arriving(waypoint.on_reached_actions)
            else:
                self._advance_to_next()

    def _on_arriving_done(self) -> None:
        with self._lock:
            self._advance_to_next()

    # ------------------------------------------------------------------
    # ウェイポイント進行ロジック
    # ------------------------------------------------------------------

    def _advance_to_next(self) -> None:
        if self._stop_pending:
            self._stop_pending = False
            self._pause_pending = False
            self._pause_manager.clear_all()
            self._transition(SequencerState.IDLE)
            return

        if self._pause_pending:
            self._pause_pending = False
            self._pre_suspend_state = SequencerState.ON_ARRIVING
            self._transition(SequencerState.SUSPENDED)
            return

        waypoint = self._waypoints.get(self._current_index)
        self._current_index += 1

        has_wait_trigger = any(
            a.type == "wait_trigger" for a in waypoint.on_reached_actions)
        if has_wait_trigger:
            if self._current_index >= self._waypoints.get_size():
                self._transition(SequencerState.GOAL_REACHED)
            else:
                self._transition(SequencerState.IDLE)
            return

        if self._current_index >= self._waypoints.get_size():
            self._transition(SequencerState.GOAL_REACHED)
            return

        self._enter_navigating()

    # ------------------------------------------------------------------
    # Named Pause Slot 管理
    # ------------------------------------------------------------------

    def _apply_pause(self) -> None:
        if self._state == SequencerState.ON_ARRIVING:
            self._pause_pending = True
            return
        if self._state == SequencerState.ON_STARTING:
            self._saved_countdown_ms = self._countdown_timer.remaining_ms
            self._countdown_timer.cancel()
            self._pre_suspend_state = SequencerState.ON_STARTING
            self._transition(SequencerState.SUSPENDED)
        elif self._state == SequencerState.NAVIGATING:
            self._navigator.cancel()
            self._pre_suspend_state = SequencerState.NAVIGATING
            self._transition(SequencerState.SUSPENDED)

    def _try_resume(self) -> None:
        if self._pause_manager.is_active or self._state != SequencerState.SUSPENDED:
            return
        if self._pre_suspend_state == SequencerState.ON_STARTING:
            self._enter_on_starting(self._saved_countdown_ms)
        elif self._pre_suspend_state == SequencerState.NAVIGATING:
            self._enter_navigating()
        elif self._pre_suspend_state == SequencerState.ON_ARRIVING:
            self._advance_to_next()
        else:
            self._transition(SequencerState.IDLE)

    # ------------------------------------------------------------------
    # 状態遷移
    # ------------------------------------------------------------------

    def _transition(self, new_state: SequencerState) -> None:
        allowed = ALLOWED_TRANSITIONS.get(self._state, frozenset())
        if new_state not in allowed:
            self._node.get_logger().error(
                f"FSM: invalid transition {self._state.value} -> {new_state.value}"
            )
            return
        old = self._state
        self._state = new_state
        self._node.get_logger().info(f"FSM: {old.value} -> {new_state.value}")
        if self._on_state_changed:
            self._on_state_changed(new_state)
