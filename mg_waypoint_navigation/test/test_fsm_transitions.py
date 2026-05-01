"""
WaypointSequencerFSM の状態遷移単体テスト。
ROS2 不要: conftest.py が ROS2 依存を sys.modules でモックする。
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from mg_waypoint_navigation.waypoint import ActionConfig, NavigationConfig, Waypoint, WaypointList
from mg_waypoint_navigation.waypoint_sequencer.fsm import WaypointSequencerFSM
from mg_waypoint_navigation.waypoint_sequencer.navigator import NavigationResult
from mg_waypoint_navigation.waypoint_sequencer.states import ALLOWED_TRANSITIONS, SequencerState


# ---------------------------------------------------------------------------
# ヘルパー
# ---------------------------------------------------------------------------

def _make_wl(*action_lists) -> WaypointList:
    """各要素のアクションリストを持つ WaypointList を生成する。"""
    wl = WaypointList()
    for i, actions in enumerate(action_lists):
        wl.add(Waypoint(
            index=i,
            pose=MagicMock(),
            navigation=NavigationConfig(),
            on_reached_actions=actions,
        ))
    return wl


# ---------------------------------------------------------------------------
# フィクスチャ
# ---------------------------------------------------------------------------

@pytest.fixture
def env():
    """FSM + モックナビゲーター + モックアクションエグゼキューターを返す。"""
    node = MagicMock()
    logger = MagicMock()
    node.get_logger.return_value = logger

    nav_cb: list = [None]
    exec_cb: list = [None]

    with (
        patch("mg_waypoint_navigation.waypoint_sequencer.fsm.WaypointNavigator") as MockNav,
        patch("mg_waypoint_navigation.waypoint_sequencer.fsm.ActionExecutor") as MockExec,
    ):
        mock_nav = MagicMock()
        mock_exec = MagicMock()
        MockNav.return_value = mock_nav
        MockExec.return_value = mock_exec

        mock_nav.send_goal.side_effect = lambda wp, cb: nav_cb.__setitem__(
            0, cb)
        mock_exec.execute.side_effect = lambda actions, cb: exec_cb.__setitem__(
            0, cb)

        fsm = WaypointSequencerFSM(node)

        yield SimpleNamespace(
            fsm=fsm,
            node=node,
            logger=logger,
            nav=mock_nav,
            executor=mock_exec,
            fire_nav_success=lambda: nav_cb[0](NavigationResult.SUCCEEDED),
            fire_nav_failed=lambda: nav_cb[0](NavigationResult.FAILED),
            fire_nav_canceled=lambda: nav_cb[0](NavigationResult.CANCELED),
            fire_countdown=fsm._on_starting_done,
            fire_actions_done=lambda: exec_cb[0](),
        )


# ---------------------------------------------------------------------------
# 正常フロー
# ---------------------------------------------------------------------------

class TestNormalFlow:
    def test_start_from_idle(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        result = env.fsm.start(0)
        assert result.success
        assert env.fsm.state == SequencerState.ON_STARTING

    def test_start_no_waypoints_fails(self, env):
        result = env.fsm.start(0)
        assert not result.success
        assert env.fsm.state == SequencerState.IDLE

    def test_countdown_transitions_to_navigating(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        assert env.fsm.state == SequencerState.NAVIGATING
        assert env.nav.send_goal.called

    def test_nav_success_no_actions_advances_to_next(self, env):
        env.fsm.load_waypoints(_make_wl([], []))
        env.fsm.start(0)
        env.fire_countdown()
        assert env.fsm.current_index == 0
        env.fire_nav_success()
        assert env.fsm.state == SequencerState.NAVIGATING
        assert env.fsm.current_index == 1

    def test_nav_success_with_actions_to_on_arriving(self, env):
        env.fsm.load_waypoints(_make_wl([ActionConfig(type="wait")]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()
        assert env.fsm.state == SequencerState.ON_ARRIVING

    def test_on_arriving_not_last_wp_to_navigating(self, env):
        env.fsm.load_waypoints(_make_wl([ActionConfig(type="wait")], []))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()
        assert env.fsm.state == SequencerState.ON_ARRIVING
        env.fire_actions_done()
        assert env.fsm.state == SequencerState.NAVIGATING
        assert env.fsm.current_index == 1

    def test_on_arriving_last_wp_to_goal_reached(self, env):
        env.fsm.load_waypoints(_make_wl([ActionConfig(type="wait")]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()
        env.fire_actions_done()
        assert env.fsm.state == SequencerState.GOAL_REACHED

    def test_nav_success_last_wp_no_actions_to_goal_reached(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()
        assert env.fsm.state == SequencerState.GOAL_REACHED

    def test_goal_reached_restart_resets_index(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()
        assert env.fsm.state == SequencerState.GOAL_REACHED
        result = env.fsm.start(0)
        assert result.success
        assert env.fsm.current_index == 0
        assert env.fsm.state == SequencerState.ON_STARTING

    def test_full_sequence_three_wps_no_actions(self, env):
        env.fsm.load_waypoints(_make_wl([], [], []))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()
        assert env.fsm.current_index == 1
        env.fire_nav_success()
        assert env.fsm.current_index == 2
        env.fire_nav_success()
        assert env.fsm.state == SequencerState.GOAL_REACHED


# ---------------------------------------------------------------------------
# wait_trigger
# ---------------------------------------------------------------------------

class TestWaitTrigger:
    def test_middle_wp_wait_trigger_to_idle(self, env):
        env.fsm.load_waypoints(_make_wl(
            [],
            [ActionConfig(type="wait_trigger")],
            [],
        ))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()  # wp0: no actions
        assert env.fsm.current_index == 1
        assert env.fsm.state == SequencerState.NAVIGATING
        env.fire_nav_success()  # wp1: wait_trigger
        assert env.fsm.state == SequencerState.ON_ARRIVING
        env.fire_actions_done()
        assert env.fsm.state == SequencerState.IDLE
        assert env.fsm.current_index == 2

    def test_last_wp_wait_trigger_to_goal_reached(self, env):
        env.fsm.load_waypoints(
            _make_wl([], [ActionConfig(type="wait_trigger")]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()  # wp0
        env.fire_nav_success()  # wp1 (last, wait_trigger)
        assert env.fsm.state == SequencerState.ON_ARRIVING
        env.fire_actions_done()
        assert env.fsm.state == SequencerState.GOAL_REACHED

    def test_start_after_wait_trigger_resumes_from_next_index(self, env):
        env.fsm.load_waypoints(_make_wl(
            [],
            [ActionConfig(type="wait_trigger")],
            [],
        ))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()  # wp0
        env.fire_nav_success()  # wp1 wait_trigger
        env.fire_actions_done()
        assert env.fsm.state == SequencerState.IDLE
        assert env.fsm.current_index == 2

        env.fsm.start(0)
        env.fire_countdown()
        assert env.fsm.state == SequencerState.NAVIGATING
        navigated_wp = env.nav.send_goal.call_args_list[-1][0][0]
        assert navigated_wp.index == 2

    def test_wait_trigger_mixed_actions_to_idle(self, env):
        """wait_trigger と他アクションの混在でも全実行後に IDLE"""
        actions = [ActionConfig(type="wait"),
                   ActionConfig(type="wait_trigger")]
        env.fsm.load_waypoints(_make_wl([], actions, []))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()  # wp0
        env.fire_nav_success()  # wp1
        assert env.fsm.state == SequencerState.ON_ARRIVING
        env.fire_actions_done()
        assert env.fsm.state == SequencerState.IDLE
        assert env.fsm.current_index == 2


# ---------------------------------------------------------------------------
# stop
# ---------------------------------------------------------------------------

class TestStop:
    def test_stop_from_idle_is_noop(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        result = env.fsm.stop()
        assert result.success
        assert env.fsm.state == SequencerState.IDLE

    def test_stop_from_on_starting(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        result = env.fsm.stop()
        assert result.success
        assert env.fsm.state == SequencerState.IDLE

    def test_stop_from_navigating(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        result = env.fsm.stop()
        assert result.success
        assert env.fsm.state == SequencerState.IDLE
        env.nav.cancel.assert_called()

    def test_stop_deferred_from_on_arriving(self, env):
        env.fsm.load_waypoints(_make_wl([ActionConfig(type="wait")]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()
        assert env.fsm.state == SequencerState.ON_ARRIVING
        result = env.fsm.stop()
        assert result.success
        assert env.fsm.state == SequencerState.ON_ARRIVING  # まだ ON_ARRIVING
        env.fire_actions_done()
        assert env.fsm.state == SequencerState.IDLE

    def test_stop_from_goal_reached(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()
        assert env.fsm.state == SequencerState.GOAL_REACHED
        result = env.fsm.stop()
        assert result.success
        assert env.fsm.state == SequencerState.IDLE

    def test_stop_from_error(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_failed()
        assert env.fsm.state == SequencerState.ERROR
        result = env.fsm.stop()
        assert result.success
        assert env.fsm.state == SequencerState.IDLE


# ---------------------------------------------------------------------------
# pause / resume
# ---------------------------------------------------------------------------

class TestPauseResume:
    def test_pause_from_navigating(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fsm.pause_request("p1", True, 1.0)
        assert env.fsm.state == SequencerState.SUSPENDED
        env.nav.cancel.assert_called()

    def test_pause_from_on_starting(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(60_000)
        env.fsm.pause_request("p1", True, 1.0)
        assert env.fsm.state == SequencerState.SUSPENDED

    def test_resume_to_navigating(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fsm.pause_request("p1", True, 1.0)
        assert env.fsm.state == SequencerState.SUSPENDED
        env.fsm.pause_request("p1", False, 1.0)
        assert env.fsm.state == SequencerState.NAVIGATING

    def test_resume_to_on_starting(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(60_000)
        env.fsm.pause_request("p1", True, 1.0)
        assert env.fsm.state == SequencerState.SUSPENDED
        env.fsm.pause_request("p1", False, 1.0)
        assert env.fsm.state == SequencerState.ON_STARTING

    def test_pause_deferred_from_on_arriving(self, env):
        env.fsm.load_waypoints(_make_wl([ActionConfig(type="wait")]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_success()
        assert env.fsm.state == SequencerState.ON_ARRIVING
        env.fsm.pause_request("p1", True, 1.0)
        assert env.fsm.state == SequencerState.ON_ARRIVING  # deferred
        env.fire_actions_done()
        assert env.fsm.state == SequencerState.SUSPENDED

    def test_multi_slot_resume_requires_all_released(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fsm.pause_request("p1", True, 1.0)
        env.fsm.pause_request("p2", True, 1.0)
        assert env.fsm.state == SequencerState.SUSPENDED
        env.fsm.pause_request("p1", False, 1.0)
        assert env.fsm.state == SequencerState.SUSPENDED  # p2 まだアクティブ
        env.fsm.pause_request("p2", False, 1.0)
        assert env.fsm.state == SequencerState.NAVIGATING

    def test_stop_from_suspended(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fsm.pause_request("p1", True, 1.0)
        result = env.fsm.stop()
        assert result.success
        assert env.fsm.state == SequencerState.IDLE


# ---------------------------------------------------------------------------
# error
# ---------------------------------------------------------------------------

class TestError:
    def test_nav_failure_to_error(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_failed()
        assert env.fsm.state == SequencerState.ERROR

    def test_nav_canceled_ignored(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_canceled()
        assert env.fsm.state == SequencerState.NAVIGATING

    def test_stop_from_error_to_idle(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        env.fsm.start(0)
        env.fire_countdown()
        env.fire_nav_failed()
        env.fsm.stop()
        assert env.fsm.state == SequencerState.IDLE


# ---------------------------------------------------------------------------
# set_next_index
# ---------------------------------------------------------------------------

class TestSetNextIndex:
    def test_set_index_in_idle(self, env):
        env.fsm.load_waypoints(_make_wl([], [], []))
        assert env.fsm.set_next_index(2)
        assert env.fsm.current_index == 2

    def test_set_index_in_suspended(self, env):
        env.fsm.load_waypoints(_make_wl([], [], []))
        env.fsm.start(0)
        env.fire_countdown()
        env.fsm.pause_request("p1", True, 1.0)
        assert env.fsm.set_next_index(2)
        assert env.fsm.current_index == 2

    def test_set_index_in_navigating_fails(self, env):
        env.fsm.load_waypoints(_make_wl([], []))
        env.fsm.start(0)
        env.fire_countdown()
        assert not env.fsm.set_next_index(1)

    def test_set_index_out_of_range_fails(self, env):
        env.fsm.load_waypoints(_make_wl([]))
        assert not env.fsm.set_next_index(5)


# ---------------------------------------------------------------------------
# 禁止遷移（ALLOWED_TRANSITIONS の外側）
# ---------------------------------------------------------------------------

def _forbidden_transition_cases():
    all_states = list(SequencerState)
    cases = []
    for from_state in all_states:
        allowed = ALLOWED_TRANSITIONS.get(from_state, frozenset())
        for to_state in all_states:
            if to_state not in allowed:
                cases.append(pytest.param(
                    from_state, to_state,
                    id=f"{from_state.value}->{to_state.value}",
                ))
    return cases


@pytest.mark.parametrize("from_state,to_state", _forbidden_transition_cases())
def test_forbidden_transition(env, from_state, to_state):
    env.fsm._state = from_state
    env.logger.error.reset_mock()
    env.fsm._transition(to_state)
    assert env.fsm.state == from_state, (
        f"Forbidden transition {from_state.value} -> {to_state.value} "
        "should not change state"
    )
    env.logger.error.assert_called()
