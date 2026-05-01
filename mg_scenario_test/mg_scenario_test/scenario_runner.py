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
        plans = self._build_goal_plans(scenario)
        total = len(plans)
        start_time = time.monotonic()

        for plan in plans:
            self._node.get_logger().info(
                f"[ScenarioRunner] Goal {plan.goal_index + 1}/{total}: "
                f"({plan.pose.x:.2f}, {plan.pose.y:.2f})"
            )

            stop_before = threading.Event()
            self._event_executor.execute_events(plan.before, stop_before)

            during_stop = threading.Event()
            during_thread = threading.Thread(
                target=self._event_executor.execute_events,
                args=(plan.during, during_stop),
                daemon=True,
            )
            during_thread.start()

            nav_ok = self._navigate(plan.pose)

            during_stop.set()
            during_thread.join(timeout=10.0)

            if not nav_ok:
                elapsed = time.monotonic() - start_time
                self._node.get_logger().error(
                    f"[ScenarioRunner] Navigation failed at goal {plan.goal_index}"
                )
                return ScenarioResult(
                    success=False,
                    elapsed_sec=elapsed,
                    reached_count=plan.goal_index,
                    total_count=total,
                    failed_index=plan.goal_index,
                )

            stop_after = threading.Event()
            self._event_executor.execute_events(plan.after, stop_after)

        elapsed = time.monotonic() - start_time
        return ScenarioResult(
            success=True,
            elapsed_sec=elapsed,
            reached_count=total,
            total_count=total,
        )

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

        def goal_response_cb(future):
            handle = future.result()
            if not handle.accepted:
                self._node.get_logger().warn(
                    "[ScenarioRunner] NavigateToPose goal rejected"
                )
                done_event.set()
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(result_cb)

        def result_cb(future):
            status = future.result().status
            result_holder[0] = status == GoalStatus.STATUS_SUCCEEDED
            done_event.set()

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(goal_response_cb)

        done_event.wait()
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
