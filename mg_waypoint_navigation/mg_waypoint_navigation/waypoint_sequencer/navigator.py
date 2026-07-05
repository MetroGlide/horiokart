"""Nav2 NavigateToPose アクションクライアントのラッパー"""
from __future__ import annotations

import enum
from typing import Callable, Optional
from math import sqrt, atan2

import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from mg_waypoint_navigation.waypoint import Waypoint


class NavigationResult(enum.Enum):
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class WaypointNavigator:
    """Nav2 NavigateToPose を呼び出す非同期ラッパー"""

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._action_client = ActionClient(
            node, NavigateToPose, "navigate_to_pose")
        self._goal_handle: Optional[ClientGoalHandle] = None
        self._result_callback: Optional[Callable[[
            NavigationResult], None]] = None
        self._distance_remaining: float = 0.0
        self._through_tolerance: Optional[float] = None
        self._through_cancel: bool = False
        self._path_computed: bool = False
        self._waypoint: Optional[Waypoint] = None

        self._bt_xml_normal = node.declare_parameter(
            "bt_xml_normal",
            get_package_share_directory("mg_waypoint_navigation")
            + "/behavior_trees/mg_navigate_to_pose.xml"
        ).value
        self._bt_xml_queue_wait = node.declare_parameter(
            "bt_xml_queue_wait",
            get_package_share_directory("mg_waypoint_navigation")
            + "/behavior_trees/mg_navigate_to_pose_queue_wait.xml"
        ).value

    def send_goal(
        self,
        waypoint: Waypoint,
        result_callback: Callable[[NavigationResult], None],
        navigation_mode: str = "normal",
    ) -> None:
        self._result_callback = result_callback
        self._through_tolerance = (
            waypoint.navigation.through_tolerance
            if waypoint.navigation.is_through_point
            else None
        )
        self._through_cancel = False
        self._path_computed = False
        self._waypoint = waypoint

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error("navigate_to_pose action server not available")
            result_callback(NavigationResult.FAILED)
            return

        goal = NavigateToPose.Goal()
        goal.pose = waypoint.pose
        goal.behavior_tree = (
            self._bt_xml_queue_wait
            if navigation_mode == "queue_wait"
            else self._bt_xml_normal
        )

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    @property
    def distance_remaining(self) -> float:
        return self._distance_remaining

    def cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _goal_response_callback(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().warn("NavigateToPose goal rejected")
            if self._result_callback:
                self._result_callback(NavigationResult.FAILED)
            return

        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_done_callback)

    def _result_done_callback(self, future) -> None:
        from action_msgs.msg import GoalStatus

        self._goal_handle = None
        status = future.result().status
        if self._through_cancel:
            result = NavigationResult.SUCCEEDED
        elif status == GoalStatus.STATUS_SUCCEEDED:
            result = NavigationResult.SUCCEEDED
        elif status == GoalStatus.STATUS_CANCELED:
            result = NavigationResult.CANCELED
        else:
            result = NavigationResult.FAILED

        if self._result_callback:
            self._result_callback(result)

    def _check_actual_arrival_through_tolerance(self, current_pose) -> bool:
        if self._waypoint is None:
            return False

        # compute distance to goal from current_pose and self._waypoint.pose
        dx = self._waypoint.pose.pose.position.x - current_pose.position.x
        dy = self._waypoint.pose.pose.position.y - current_pose.position.y
        distance = sqrt(dx * dx + dy * dy)

        return distance <= self._through_tolerance if self._through_tolerance is not None else False

    def _feedback_callback(self, feedback_msg) -> None:
        _old_distance_remaining = self._distance_remaining
        self._distance_remaining = feedback_msg.feedback.distance_remaining

        if self._through_tolerance is None or self._through_cancel:
            return

        if not self._path_computed:
            if self._distance_remaining > 0.0 and self._distance_remaining != _old_distance_remaining:
                self._node.get_logger().info(
                    f"Path computed. Distance to goal: {self._distance_remaining:.2f} m"
                )
                self._path_computed = True
            return
        if self._distance_remaining <= self._through_tolerance and self._check_actual_arrival_through_tolerance(feedback_msg.feedback.current_pose.pose):
            self._node.get_logger().info(
                f"Within through tolerance ({self._through_tolerance} m). Canceling goal to proceed to next waypoint."
            )
            self._through_cancel = True
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
