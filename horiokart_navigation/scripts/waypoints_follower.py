#!/usr/bin/env python3
import os

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.clock import Clock
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from std_msgs.msg import Int16

from action_msgs.msg import GoalStatus

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav2_msgs.action import NavigateToPose

from dataclasses import dataclass
from typing import Any, List
import time

from horiokart_navigation.waypoint import WaypointList, Waypoint, WaypointsLoader, OnReachedAction


class WaypointManager:
    def __init__(self, waypoints: WaypointList):
        self.waypoints = waypoints
        self.waypoints.sort_by_index()

        self._current_index = 0

    def get_waypoint(self) -> Waypoint:
        if self._current_index >= self.waypoints.get_size():
            return self.waypoints.get(-1)
        return self.waypoints.get(self._current_index)

    def reached(self):
        self._current_index += 1

    def is_goal_reached(self) -> bool:
        return self._current_index >= self.waypoints.get_size()

    def force_set_current_index(self, index: int):
        self._current_index = index

    def get_current_index(self) -> int:
        return self._current_index

    def get_all_waypoints(self) -> List[Waypoint]:
        return self.waypoints.get_all()

    @staticmethod
    def load_waypoints_from_file(file_path, node=None):
        if not os.path.exists(file_path):
            error_txt = f"File not found: {file_path}"
            if node is not None:
                node.get_logger().error(error_txt)
            else:
                print(error_txt)
            return

        loader = WaypointsLoader(file_path=file_path)
        return WaypointManager(loader.load())


@dataclass
class FollowingStatus:
    goal_status: int
    current_pose: PoseStamped
    distance_remaining: float


class WaypointsFollower:
    def __init__(self, node) -> None:
        self.node = node

        # Action client for /follow_waypoints
        self.node.get_logger().info('Waiting for /follow_waypoints action server...')
        self.client = ActionClient(
            self.node, NavigateToPose, 'navigate_to_pose')

        while not self.client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().info('Action server not available, waiting...')
        self.node.get_logger().info('Action server available')

        self._stop_request = True

        self.current_following_status = FollowingStatus(
            goal_status=GoalStatus.STATUS_UNKNOWN,
            current_pose=PoseStamped(),
            distance_remaining=0.0)

        self.goal_handle = None

    def send_goal(self, waypoint: Waypoint):
        if self._stop_request:
            self.node.get_logger().info("Stop requested")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint.pose
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

        self.client.wait_for_server()
        future = self.client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._send_goal_done_callback)

    def stop_request(self):
        self._stop_request = True
        self.cancel_goal()

    def start_request(self):
        self._stop_request = False

    def cancel_goal(self):
        if self.goal_handle is None:
            return

        if self.goal_handle.status == GoalStatus.STATUS_EXECUTING \
                or self.goal_handle.status == GoalStatus.STATUS_UNKNOWN:

            self.node.get_logger().info("Canceling goal")
            self.goal_handle.cancel_goal_async()
            # self.node.get_logger().info("Goal canceled")

    def _feedback_callback(self, feedback_msg):
        self.node.get_logger().debug(f"Nav to pose Feedback: {feedback_msg}")
        if self.goal_handle is None:
            return
        self.current_following_status = FollowingStatus(
            goal_status=self.goal_handle.status,
            current_pose=feedback_msg.feedback.current_pose,
            distance_remaining=feedback_msg.feedback.distance_remaining)

    def _send_goal_done_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error("Goal rejected")
            return

        self.current_following_status = FollowingStatus(
            goal_status=self.goal_handle.status,
            current_pose=PoseStamped(),
            distance_remaining=0.0)

        self.node.get_logger().info("Goal accepted")
        self._result_future = self.goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.node.get_logger().info("Goal result received")

        self.current_following_status = FollowingStatus(
            goal_status=self.goal_handle.status,
            current_pose=PoseStamped(),
            distance_remaining=0.0)


class RetryWaiter:
    def __init__(self, node, retry_duration_sec=5.0, retry_func=None):
        self.node = node
        self.retry_duration_sec = retry_duration_sec

        self._retry_start_time = None
        self._retry_func = retry_func

    def tick(self):
        if self._retry_start_time is None:
            self._retry_start_time = self.node.get_clock().now()
            return

        if self.node.get_clock().now() - self._retry_start_time >= rclpy.duration.Duration(seconds=self.retry_duration_sec):
            self._retry_start_time = None
            if self._retry_func is not None:
                self._retry_func()

    def reset(self):
        self._retry_start_time = None


class WaypointsFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        load_path = self.declare_parameter(
            'load_path', "/root/ros2_data/new_waypoints.yaml").value

        self.waypoint_manager = WaypointManager.load_waypoints_from_file(
            load_path, node=self)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._waypoints_follower = WaypointsFollower(self)

        self._marker_array_publisher = self.create_publisher(
            MarkerArray,
            '~/waypoints_markers',
            1)

        self.publish_waypoints_markers()

        self._stop_request = True
        self._stop_srv = self.create_service(
            Trigger,
            '~/stop',
            self._stop_callback)
        self._start_srv = self.create_service(
            Trigger,
            '~/start',
            self._start_callback)

        self._set_next_waypoint_index_sub = self.create_subscription(
            Int16,
            '~/set_next_waypoint_index',
            self._set_next_waypoint_index_callback,
            1)

        self._state_unknown_retry_waiter = RetryWaiter(
            self, retry_duration_sec=5.0, retry_func=self._start_following)

    def _stop_callback(self, request, response):
        self._stop_following()
        response.success = True
        return response

    def _stop_following(self):
        self.get_logger().info("Stop following waypoints")
        self._stop_request = True
        self._waypoints_follower.stop_request()

    def _start_callback(self, request, response):
        if not self._stop_request:
            self.get_logger().error(f"Already running")

            response.success = False
            return response
        self.get_logger().info("Start following waypoints")
        self._stop_request = False

        self._start_following()

        response.success = True
        return response

    def _start_following(self):
        if self._stop_request:
            self.get_logger().error("Stop requested!!")
            return
        self._waypoints_follower.start_request()
        self._waypoints_follower.send_goal(
            self.waypoint_manager.get_waypoint())

    def _set_next_waypoint_index_callback(self, msg):
        self.get_logger().warn(
            f"Temporary subscriber ! Future: use service to set next waypoint index")

        if not self._stop_request:
            self.get_logger().error(f"Cannot set next waypoint index while running")
            return

        self.get_logger().info(
            f"Set next waypoint index: {msg.data}. Before: {self.waypoint_manager._current_index}")
        self.waypoint_manager.force_set_current_index(msg.data)

    def publish_waypoints_markers(self):
        marker_array = MarkerArray()

        for waypoint in self.waypoint_manager.get_all_waypoints():
            marker = self.create_marker(waypoint)
            marker_array.markers.append(marker)

        self._marker_array_publisher.publish(marker_array)

    def create_marker(self, waypoint: Waypoint):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = waypoint.index
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = waypoint.pose.pose
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        return marker

    def _calc_distance(self, p1: Point, p2: Point) -> float:
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5

    def get_distance(self, waypoint: Waypoint) -> float:
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Failed to lookup transform: {e}")
            return 10000.0

        return self._calc_distance(waypoint.pose.pose.position, transform.transform.translation)

    def _check_reached(self, waypoint: Waypoint) -> bool:
        if not self._waypoints_follower.current_following_status.goal_status == GoalStatus.STATUS_EXECUTING:
            self.get_logger().error(
                f"Goal status is not executing: {self._waypoints_follower.current_following_status.goal_status}")
            return True

        distance = self._waypoints_follower.current_following_status.distance_remaining
        if distance <= waypoint.reach_tolerance:
            self.get_logger().info(
                f"Distance remaining: {distance}. Reach tolerance: {waypoint.reach_tolerance}")
            return True

    def _on_reached_action(self, waypoint: Waypoint):
        if OnReachedAction.WAIT_TRIGGER.value in waypoint.on_reached_action:
            self.get_logger().info(
                f"OnReachedAction: Wait for trigger to continue to next waypoint")
            # self._stop_following()
            self._stop_request = True

    def _on_reached(self, waypoint: Waypoint):
        self._on_reached_action(waypoint)

        self.waypoint_manager.reached()

    def _on_goal_reached(self):
        self.get_logger().info("Goal reached!")
        self._stop_request = True

    def _follow_waypoints(self):
        self.publish_waypoints_markers()
        if self._stop_request:
            self.get_logger().log("Waiting for start trigger", LoggingSeverity.INFO,
                                  throttle_duration_sec=2.0, throttle_time_source_type=self.get_clock())
            return

        current_waypoint = self.waypoint_manager.get_waypoint()

        if not self._check_reached(waypoint=current_waypoint):
            self.get_logger().log(f"running to {self.waypoint_manager.get_current_index()}. Distance remaining: {self._waypoints_follower.current_following_status.distance_remaining} m",
                                  LoggingSeverity.INFO, throttle_duration_sec=2.0, throttle_time_source_type=self.get_clock())
            return

        # When reached
        if self.get_distance(current_waypoint) < self.waypoint_manager.get_waypoint().reach_tolerance:
            self.get_logger().info(
                f"Waypoint {current_waypoint.index} reached. Next waypoint: {current_waypoint.index + 1}")
            self._on_reached(current_waypoint)

            if self.waypoint_manager.is_goal_reached():
                self._on_goal_reached()
                return

            self._state_unknown_retry_waiter.reset()
            self._start_following()

        # When reached but not close enough
        else:
            self._state_unknown_retry_waiter.tick()
            self.get_logger().log(
                f"Waypoint {current_waypoint.index} reached. But not close enough. Retry waypoint {current_waypoint.index}",
                LoggingSeverity.INFO,
                throttle_duration_sec=1.0, throttle_time_source_type=self.get_clock())

    def run(self):
        rate = 1 / 5.0
        self.timer = self.create_timer(rate, self._follow_waypoints)


def main(args=None):
    rclpy.init(args=args)
    waypoints_node = WaypointsFollowerNode()

    waypoints_node.run()
    rclpy.spin(waypoints_node)
    waypoints_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
