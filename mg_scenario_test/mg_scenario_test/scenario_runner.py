from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy

if TYPE_CHECKING:
    import rclpy.node
    from mg_scenario_test.adapters.base import SimulatorAdapter
    from mg_scenario_test.event_executor import EventExecutor
    from mg_scenario_test.scenario import (
        EventSpec,
        GoalEventSpec,
        GoalSpec,
        PoseSpec,
        Scenario,
    )


@dataclass
class ScenarioResult:
    success: bool
    elapsed_sec: float
    reached_count: int
    total_count: int
    failed_index: int = -1


@dataclass
class _GoalPlan:
    pose: "PoseSpec"
    before: "List[EventSpec]"
    during: "List[EventSpec]"
    after: "List[EventSpec]"
    goal_index: int


class ScenarioRunner:
    """シナリオを実行するクラス。

    ScenarioTestNode の別スレッドから呼び出す。
    Node は MultiThreadedExecutor で spin している必要がある。
    """

    _SEP = "=" * 60

    def __init__(
        self,
        node: "rclpy.node.Node",
        adapter: "SimulatorAdapter",
        event_executor: "EventExecutor",
    ):
        self._node = node
        self._adapter = adapter
        self._event_executor = event_executor
        self._nav_client = ActionClient(
            node, NavigateToPose, "navigate_to_pose")

    def execute(self, scenario: "Scenario") -> ScenarioResult:
        self._event_executor.sequencer_namespace = scenario.sequencer_namespace
        if scenario.waypoints_nav_mode == "sequencer":
            return self._execute_sequencer_mode(scenario)
        return self._execute_direct_mode(scenario)

    def _execute_direct_mode(self, scenario: "Scenario") -> ScenarioResult:
        plans = self._build_goal_plans(scenario)
        total = len(plans)
        start_time = time.monotonic()
        log = self._node.get_logger()

        log.info(self._SEP)
        log.info(f"  Scenario : {scenario.scenario_name}")
        log.info(f"  World    : {scenario.world_name}")
        log.info(f"  Goals    : {total}")
        log.info(self._SEP)

        result = ScenarioResult(
            success=False, elapsed_sec=0.0,
            reached_count=0, total_count=total,
        )
        try:
            for plan in plans:
                log.info(
                    f"[Goal {plan.goal_index + 1}/{total}] "
                    f"target=({plan.pose.x:.2f}, {plan.pose.y:.2f})"
                )

                if plan.before:
                    log.info(f"  >> before events ({len(plan.before)} items)")
                stop_before = threading.Event()
                self._event_executor.execute_events(plan.before, stop_before)

                if plan.during:
                    log.info(
                        f"  >> during events ({len(plan.during)} items) [parallel]")
                during_stop = threading.Event()
                during_thread = threading.Thread(
                    target=self._event_executor.execute_events,
                    args=(plan.during, during_stop),
                    daemon=True,
                )
                during_thread.start()

                log.info(f"  >> navigating ...")
                nav_ok = self._navigate(plan.pose)

                during_stop.set()
                during_thread.join(timeout=10.0)

                if not nav_ok:
                    elapsed = time.monotonic() - start_time
                    log.error(
                        f"[Goal {plan.goal_index + 1}/{total}] FAILED — navigation did not succeed"
                    )
                    result = ScenarioResult(
                        success=False,
                        elapsed_sec=elapsed,
                        reached_count=plan.goal_index,
                        total_count=total,
                        failed_index=plan.goal_index,
                    )
                    return result

                log.info(f"  >> after events ({len(plan.after)} items)")
                stop_after = threading.Event()
                self._event_executor.execute_events(plan.after, stop_after)

                log.info(f"[Goal {plan.goal_index + 1}/{total}] REACHED")

            elapsed = time.monotonic() - start_time
            result = ScenarioResult(
                success=True,
                elapsed_sec=elapsed,
                reached_count=total,
                total_count=total,
            )
            return result

        finally:
            if scenario.finally_events:
                log.info(
                    f"[Finally] running {len(scenario.finally_events)} cleanup event(s)")
                stop = threading.Event()
                self._event_executor.execute_events(
                    scenario.finally_events, stop)
                log.info("[Finally] done")

    def _execute_sequencer_mode(self, scenario: "Scenario") -> ScenarioResult:
        """waypoint_sequencer に走行を委ねるモード。

        各 waypoint の before を実行後、trigger_waypoint イベントで sequencer を起動し、
        current_index の変化を監視して during/after を実行する。
        wait_trigger の有無に関わらず到達を検出できる。
        """
        log = self._node.get_logger()
        ns = scenario.sequencer_namespace

        all_wps = self._get_waypoints(scenario)
        if all_wps is None:
            log.error(
                "[ScenarioRunner] Cannot get waypoints: waypoints_file not set "
                f"and /{ns}/waypoints topic unavailable"
            )
            return ScenarioResult(
                success=False, elapsed_sec=0.0, reached_count=0, total_count=0
            )

        total = len(all_wps)
        start_index = scenario.start_waypoint_index
        run_total = total - start_index
        events_by_index = {
            ge.waypoint_index: ge for ge in (scenario.goal_events or [])
        }
        start_time = time.monotonic()

        log.info(self._SEP)
        log.info(f"  Scenario : {scenario.scenario_name}")
        log.info(f"  World    : {scenario.world_name}")
        log.info(f"  Mode     : sequencer  (ns={ns})")
        log.info(f"  Goals    : {total}  (start_index={start_index})")
        log.info(self._SEP)

        result = ScenarioResult(
            success=False, elapsed_sec=0.0,
            reached_count=0, total_count=run_total,
        )
        try:
            for loop_i, wp in enumerate(all_wps[start_index:]):
                seq_index = start_index + loop_i  # sequencer の current_index
                wp_index = wp.index if hasattr(wp, "index") else seq_index
                ge = events_by_index.get(wp_index)
                before = ge.before if ge else []
                during = ge.during if ge else []
                after = ge.after if ge else []

                try:
                    pos_x = wp.pose.pose.position.x
                    pos_y = wp.pose.pose.position.y
                    pos_str = f"({pos_x:.2f}, {pos_y:.2f})"
                except Exception:
                    pos_str = "(unknown)"

                log.info(
                    f"[Goal {seq_index + 1}/{total}] "
                    f"target={pos_str} wp_index={wp_index} [sequencer]"
                )

                if before:
                    log.info(f"  >> before events ({len(before)} items)")
                stop_before = threading.Event()
                self._event_executor.execute_events(before, stop_before)

                if during:
                    log.info(
                        f"  >> during events ({len(during)} items) [parallel]")
                during_stop = threading.Event()
                during_thread = threading.Thread(
                    target=self._event_executor.execute_events,
                    args=(during, during_stop),
                    daemon=True,
                )
                during_thread.start()

                log.info(
                    f"  >> waiting for sequencer to pass waypoint {seq_index} ..."
                )
                nav_ok = self._wait_sequencer_done(
                    seq_index, ns, timeout_sec=300.0
                )

                during_stop.set()
                during_thread.join(timeout=10.0)

                if not nav_ok:
                    elapsed = time.monotonic() - start_time
                    log.error(
                        f"[Goal {seq_index + 1}/{total}] FAILED "
                        f"\u2014 sequencer did not reach waypoint {seq_index}"
                    )
                    result = ScenarioResult(
                        success=False,
                        elapsed_sec=elapsed,
                        reached_count=loop_i,
                        total_count=run_total,
                        failed_index=seq_index,
                    )
                    return result

                if after:
                    log.info(f"  >> after events ({len(after)} items)")
                stop_after = threading.Event()
                self._event_executor.execute_events(after, stop_after)

                log.info(f"[Goal {seq_index + 1}/{total}] REACHED")

            elapsed = time.monotonic() - start_time
            result = ScenarioResult(
                success=True,
                elapsed_sec=elapsed,
                reached_count=run_total,
                total_count=run_total,
            )
            return result

        finally:
            if scenario.finally_events:
                log.info(
                    f"[Finally] running {len(scenario.finally_events)} cleanup event(s)"
                )
                stop = threading.Event()
                self._event_executor.execute_events(
                    scenario.finally_events, stop)
                log.info("[Finally] done")

    def _get_waypoints(self, scenario: "Scenario"):
        """waypoints_file またはトピックからウェイポイントリストを取得する。"""
        if scenario.waypoints_file:
            from mg_waypoint_navigation.waypoint import WaypointsLoader
            wl = WaypointsLoader(scenario.waypoints_file).load()
            return wl.get_all()
        return self._fetch_waypoints_from_topic(scenario.sequencer_namespace)

    def _fetch_waypoints_from_topic(self, ns: str):
        """latched な /{ns}/waypoints トピックから WaypointInfo リストを取得する。"""
        from mg_msgs.msg import WaypointList as WaypointListMsg
        log = self._node.get_logger()
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        received: List = [None]
        done_event = threading.Event()

        def cb(msg):
            received[0] = msg.waypoints
            done_event.set()

        sub = self._node.create_subscription(
            WaypointListMsg, f"/{ns}/waypoints", cb, latched_qos
        )
        log.info(f"[ScenarioRunner] Waiting for /{ns}/waypoints topic ...")
        done_event.wait(timeout=5.0)
        self._node.destroy_subscription(sub)

        if received[0] is None:
            return None
        log.info(
            f"[ScenarioRunner] Received {len(received[0])} waypoints from topic"
        )
        return list(received[0])

    def _wait_sequencer_done(
        self, expected_index: int, ns: str, timeout_sec: float = 300.0
    ) -> bool:
        """sequencer の current_index が expected_index を超えるまで待つ。

        wait_trigger の有無に関わらず動作する:
        - wait_trigger なし: 次 waypoint へ移行するため current_index が増加
        - wait_trigger あり: IDLE になる際に current_index が増加
        """
        from mg_msgs.msg import SequencerStatus
        done_event = threading.Event()
        result_holder: List[bool] = [False]

        def cb(msg: SequencerStatus):
            idx = msg.current_index
            state = msg.state
            if idx > expected_index:
                result_holder[0] = True
                done_event.set()
            elif state == "ERROR":
                self._node.get_logger().error(
                    f"[ScenarioRunner] sequencer entered ERROR state at index {idx}"
                )
                done_event.set()

        sub = self._node.create_subscription(
            SequencerStatus, f"/{ns}/status", cb, 10
        )
        try:
            done_event.wait(timeout=timeout_sec)
        finally:
            self._node.destroy_subscription(sub)
        return result_holder[0]

    def _build_goal_plans(self, scenario: "Scenario") -> "List[_GoalPlan]":
        if scenario.goals is not None:
            return [
                _GoalPlan(
                    pose=goal.pose,
                    before=goal.before,
                    during=goal.during,
                    after=goal.after,
                    goal_index=i,
                )
                for i, goal in enumerate(scenario.goals)
            ]

        from mg_waypoint_navigation.waypoint import WaypointsLoader

        wl = WaypointsLoader(scenario.waypoints_file).load()
        events_by_index = {
            ge.waypoint_index: ge for ge in (scenario.goal_events or [])
        }

        plans: List[_GoalPlan] = []
        for i, wp in enumerate(wl.get_all()):
            ge = events_by_index.get(wp.index)
            pos = wp.pose.pose.position
            q = wp.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            from mg_scenario_test.scenario import PoseSpec
            pose_spec = PoseSpec(frame="absolute", x=pos.x,
                                 y=pos.y, z=pos.z, yaw=yaw)
            plans.append(
                _GoalPlan(
                    pose=pose_spec,
                    before=ge.before if ge else [],
                    during=ge.during if ge else [],
                    after=ge.after if ge else [],
                    goal_index=i,
                )
            )
        return plans

    def _navigate(self, pose_spec: "PoseSpec") -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self._node.get_logger().error(
                "[ScenarioRunner] navigate_to_pose action server not available"
            )
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(pose_spec)

        done_event = threading.Event()
        result_holder: List[bool] = [False]
        goal_handle_holder = [None]
        navigate_timeout_sec = float(
            getattr(self, "_navigate_timeout_sec", 300.0)
        )
        cancel_wait_sec = float(getattr(self, "_navigate_cancel_wait_sec", 5.0))

        def goal_response_cb(future):
            try:
                handle = future.result()
                goal_handle_holder[0] = handle
                if not handle.accepted:
                    self._node.get_logger().warn(
                        "[ScenarioRunner] NavigateToPose goal rejected"
                    )
                    done_event.set()
                    return
                result_future = handle.get_result_async()
                result_future.add_done_callback(result_cb)
            except Exception as exc:
                self._node.get_logger().error(
                    f"[ScenarioRunner] Failed to process NavigateToPose goal response: {exc}"
                )
                done_event.set()

        def result_cb(future):
            try:
                status = future.result().status
                result_holder[0] = status == GoalStatus.STATUS_SUCCEEDED
            except Exception as exc:
                self._node.get_logger().error(
                    f"[ScenarioRunner] Failed to get NavigateToPose result: {exc}"
                )
                result_holder[0] = False
            finally:
                done_event.set()

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(goal_response_cb)

        if not done_event.wait(timeout=navigate_timeout_sec):
            self._node.get_logger().error(
                "[ScenarioRunner] NavigateToPose did not finish within "
                f"{navigate_timeout_sec:.1f}s; cancelling goal"
            )
            handle = goal_handle_holder[0]
            if handle is not None and handle.accepted:
                try:
                    cancel_done_event = threading.Event()
                    cancel_future = handle.cancel_goal_async()

                    def cancel_done_cb(_future):
                        cancel_done_event.set()

                    cancel_future.add_done_callback(cancel_done_cb)
                    cancel_done_event.wait(timeout=cancel_wait_sec)
                except Exception as exc:
                    self._node.get_logger().warn(
                        f"[ScenarioRunner] Failed to cancel timed out NavigateToPose goal: {exc}"
                    )
            return False

        return result_holder[0]

    @staticmethod
    def _make_pose_stamped(pose_spec: "PoseSpec") -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position = Point(x=pose_spec.x, y=pose_spec.y, z=pose_spec.z)
        qz = math.sin(pose_spec.yaw / 2.0)
        qw = math.cos(pose_spec.yaw / 2.0)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        return msg
