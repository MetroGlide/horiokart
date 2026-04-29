"""ウェイポイントシーケンサーの有限状態機械"""
from __future__ import annotations

import threading
import time
from typing import Callable, ClassVar, Dict, FrozenSet, List, Optional, Tuple

import rclpy.node

from mg_waypoint_navigation.waypoint import WaypointList
from mg_waypoint_navigation.waypoint_sequencer.action_executor import ActionExecutor
from mg_waypoint_navigation.waypoint_sequencer.navigator import (
    NavigationResult,
    WaypointNavigator,
)
from mg_waypoint_navigation.waypoint_sequencer.states import SequencerState


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
    ROS通信は呼び出し元ノードが担い、FSMはコールバックで通知を受け取る。
    """

    _ALLOWED_TRANSITIONS: ClassVar[Dict[SequencerState, FrozenSet[SequencerState]]] = {
        SequencerState.IDLE: frozenset({
            SequencerState.COUNTDOWN,
        }),
        SequencerState.COUNTDOWN: frozenset({
            SequencerState.NAVIGATING,
            SequencerState.SUSPENDED,
            SequencerState.IDLE,
        }),
        SequencerState.NAVIGATING: frozenset({
            SequencerState.NAVIGATING,
            SequencerState.EXECUTING_ACTIONS,
            SequencerState.WAITING_TRIGGER,
            SequencerState.GOAL_REACHED,
            SequencerState.ERROR,
            SequencerState.SUSPENDED,
            SequencerState.IDLE,
        }),
        SequencerState.EXECUTING_ACTIONS: frozenset({
            SequencerState.NAVIGATING,
            SequencerState.WAITING_TRIGGER,
            SequencerState.GOAL_REACHED,
            SequencerState.SUSPENDED,
            SequencerState.IDLE,
        }),
        SequencerState.WAITING_TRIGGER: frozenset({
            SequencerState.COUNTDOWN,
            SequencerState.SUSPENDED,
            SequencerState.IDLE,
        }),
        SequencerState.GOAL_REACHED: frozenset({
            SequencerState.COUNTDOWN,
            SequencerState.IDLE,
        }),
        SequencerState.ERROR: frozenset({
            SequencerState.IDLE,
        }),
        SequencerState.SUSPENDED: frozenset({
            SequencerState.COUNTDOWN,
            SequencerState.NAVIGATING,
            SequencerState.WAITING_TRIGGER,
            SequencerState.GOAL_REACHED,
            SequencerState.IDLE,
            SequencerState.SUSPENDED_UNRESPONSIVE,
        }),
        SequencerState.SUSPENDED_UNRESPONSIVE: frozenset({
            SequencerState.IDLE,
        }),
    }

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._navigator = WaypointNavigator(node)
        self._executor = ActionExecutor(node)

        self._state = SequencerState.IDLE
        self._lock = threading.Lock()

        self._waypoints: WaypointList = WaypointList()
        self._current_index: int = 0

        self._stop_pending: bool = False
        self._pause_pending: Optional[str] = None
        self._pre_suspend_state: SequencerState = SequencerState.IDLE
        self._saved_countdown_ms: int = 0

        self._countdown_timer = CountdownTimer(self._on_countdown_done)
        self._pause_manager = PauseSlotManager(node)

        self._pause_dispatch: Dict[SequencerState, Callable[[], None]] = {
            SequencerState.COUNTDOWN: self._pause_from_countdown,
            SequencerState.NAVIGATING: self._pause_from_navigating,
            SequencerState.WAITING_TRIGGER: self._pause_from_waiting_trigger,
        }
        self._resume_dispatch: Dict[SequencerState, Callable[[], None]] = {
            SequencerState.COUNTDOWN: self._resume_to_countdown,
            SequencerState.NAVIGATING: self._resume_to_navigating,
            SequencerState.EXECUTING_ACTIONS: self._resume_from_executing_actions,
            SequencerState.WAITING_TRIGGER: self._resume_to_waiting_trigger,
        }

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
        if self._state != SequencerState.COUNTDOWN:
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
        """IDLE / WAITING_TRIGGER / SUSPENDED 時のみインデックスを変更できる"""
        with self._lock:
            if self._state not in (
                SequencerState.IDLE,
                SequencerState.WAITING_TRIGGER,
                SequencerState.SUSPENDED,
            ):
                return False
            if index < 0 or index >= self._waypoints.get_size():
                return False
            self._current_index = index
            return True

    # ------------------------------------------------------------------
    # 外部コマンド
    # ------------------------------------------------------------------

    def on_start(self, countdown_ms: int) -> Tuple[bool, str]:
        with self._lock:
            if self._state == SequencerState.IDLE:
                if self._waypoints.get_size() == 0:
                    return False, "No waypoints loaded"
                self._start_countdown(countdown_ms)
                return True, "OK"

            if self._state == SequencerState.GOAL_REACHED:
                self._current_index = 0
                self._start_countdown(countdown_ms)
                return True, "OK"

            if self._state == SequencerState.WAITING_TRIGGER:
                self._start_countdown(countdown_ms)
                return True, "OK"

            return False, f"Cannot start from state {self._state.value}"

    def on_stop(self) -> Tuple[bool, str]:
        with self._lock:
            if self._state == SequencerState.IDLE:
                return True, "Already idle"

            if self._state == SequencerState.COUNTDOWN:
                self._countdown_timer.cancel()
                self._pause_manager.clear_all()
                self._transition(SequencerState.IDLE)
                return True, "OK"

            if self._state == SequencerState.NAVIGATING:
                self._navigator.cancel()
                self._pause_manager.clear_all()
                self._transition(SequencerState.IDLE)
                return True, "OK"

            if self._state == SequencerState.EXECUTING_ACTIONS:
                self._stop_pending = True
                return True, "Stop deferred until actions complete"

            if self._state in (
                SequencerState.WAITING_TRIGGER,
                SequencerState.SUSPENDED,
                SequencerState.SUSPENDED_UNRESPONSIVE,
                SequencerState.GOAL_REACHED,
                SequencerState.ERROR,
            ):
                self._pause_manager.clear_all()
                self._transition(SequencerState.IDLE)
                return True, "OK"

            return False, f"Unhandled state {self._state.value}"

    def on_pause_request(
        self,
        requester_id: str,
        active: bool,
        heartbeat_period_s: float,
        reason: str = "",
    ) -> None:
        with self._lock:
            if active:
                self._pause_manager.add(requester_id, heartbeat_period_s)
                self._apply_pause(requester_id)
            else:
                self._pause_manager.remove(requester_id)
                self._try_resume()

    # ------------------------------------------------------------------
    # ナビゲーション・アクション完了コールバック
    # ------------------------------------------------------------------

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
                self._transition(SequencerState.EXECUTING_ACTIONS)
                self._executor.execute(
                    waypoint.on_reached_actions, self._on_actions_done)
            else:
                self._process_waypoint_completion()

    def _on_actions_done(self) -> None:
        with self._lock:
            self._process_waypoint_completion()

    # ------------------------------------------------------------------
    # ウェイポイント進行ロジック
    # ------------------------------------------------------------------

    def _process_waypoint_completion(self) -> None:
        if self._stop_pending:
            self._stop_pending = False
            self._pause_pending = None
            self._pause_manager.clear_all()
            self._transition(SequencerState.IDLE)
            return

        if self._pause_pending is not None:
            self._pause_pending = None
            self._pre_suspend_state = SequencerState.EXECUTING_ACTIONS
            self._transition(SequencerState.SUSPENDED)
            return

        self._advance_waypoint()

    def _advance_waypoint(self) -> None:
        waypoint = self._waypoints.get(self._current_index)
        if any(a.type == "wait" and a.countdown_ms == 0 for a in waypoint.on_reached_actions):
            self._transition(SequencerState.WAITING_TRIGGER)
            return

        self._current_index += 1
        if self._current_index >= self._waypoints.get_size():
            self._transition(SequencerState.GOAL_REACHED)
            return

        self._navigate_to_current()

    def _navigate_to_current(self) -> None:
        waypoint = self._waypoints.get(self._current_index)
        self._transition(SequencerState.NAVIGATING)
        self._navigator.send_goal(waypoint, self._on_navigation_result)

    # ------------------------------------------------------------------
    # カウントダウン
    # ------------------------------------------------------------------

    def _start_countdown(self, countdown_ms: int) -> None:
        self._transition(SequencerState.COUNTDOWN)
        self._countdown_timer.start(countdown_ms)

    def _on_countdown_done(self) -> None:
        with self._lock:
            if self._state != SequencerState.COUNTDOWN:
                return
            self._navigate_to_current()

    # ------------------------------------------------------------------
    # Named Pause Slot 管理
    # ------------------------------------------------------------------

    def _apply_pause(self, requester_id: str) -> None:
        if self._state == SequencerState.EXECUTING_ACTIONS:
            self._pause_pending = requester_id
            return
        handler = self._pause_dispatch.get(self._state)
        if handler:
            handler()

    def _pause_from_countdown(self) -> None:
        self._saved_countdown_ms = self._countdown_timer.remaining_ms
        self._countdown_timer.cancel()
        self._pre_suspend_state = SequencerState.COUNTDOWN
        self._transition(SequencerState.SUSPENDED)

    def _pause_from_navigating(self) -> None:
        self._navigator.cancel()
        self._pre_suspend_state = SequencerState.NAVIGATING
        self._transition(SequencerState.SUSPENDED)

    def _pause_from_waiting_trigger(self) -> None:
        self._pre_suspend_state = SequencerState.WAITING_TRIGGER
        self._transition(SequencerState.SUSPENDED)

    def _try_resume(self) -> None:
        if self._pause_manager.is_active or self._state != SequencerState.SUSPENDED:
            return
        handler = self._resume_dispatch.get(self._pre_suspend_state)
        if handler:
            handler()
        else:
            self._transition(SequencerState.IDLE)

    def _resume_to_countdown(self) -> None:
        self._start_countdown(self._saved_countdown_ms)

    def _resume_to_navigating(self) -> None:
        self._navigate_to_current()

    def _resume_from_executing_actions(self) -> None:
        self._advance_waypoint()

    def _resume_to_waiting_trigger(self) -> None:
        self._transition(SequencerState.WAITING_TRIGGER)

    # ------------------------------------------------------------------
    # 状態遷移
    # ------------------------------------------------------------------

    def _transition(self, new_state: SequencerState) -> None:
        allowed = self._ALLOWED_TRANSITIONS.get(self._state, frozenset())
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
