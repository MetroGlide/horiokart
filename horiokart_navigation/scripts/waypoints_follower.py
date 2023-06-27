#!/usr/bin/env python3
import os

import rclpy
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
from typing import List
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

        self.goal_status = GoalStatus.STATUS_UNKNOWN
        self.goal_status_sub = self.node.create_subscription(
            GoalStatus, 'navigate_to_pose/status', self._goal_status_callback, 1)

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
        future.add_done_callback(self._response_callback)

    def stop_request(self):
        self._stop_request = True
        self.cancel_goal()

    def start_request(self):
        self._stop_request = False

    def cancel_goal(self):
        if self.goal_handle.status == GoalStatus.STATUS_EXECUTING \
                or self.goal_handle.status == GoalStatus.STATUS_UNKNOWN:

            self.node.get_logger().info("Canceling goal")
            self.goal_handle.cancel_goal_async()
            # self.node.get_logger().info("Goal canceled")

    def _feedback_callback(self, feedback_msg):
        self.goal_status = self.goal_handle.status

    def _response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error("Goal rejected")
            return

        self.node.get_logger().info("Goal accepted")
        self._result_future = self.goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.node.get_logger().info("Goal result received")

    def _goal_status_callback(self, msg):
        self.node.get_logger().info(f"Goal status: {msg.status}")
        self.goal_status = msg.status


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

    def _check_reached(self, waypoint: Waypoint) -> bool:
        if self._waypoints_follower.goal_status == GoalStatus.STATUS_SUCCEEDED \
            or self._waypoints_follower.goal_status == GoalStatus.STATUS_ABORTED \
                or self._waypoints_follower.goal_status == GoalStatus.STATUS_CANCELED:
            # TODO: validate estimated position
            return True

        try:
            transform = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Failed to lookup transform: {e}")
            return False

        distance = self._calc_distance(
            waypoint.pose.pose.position, transform.transform.translation)
        if distance <= waypoint.reach_tolerance:
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

    def _follow_waypoints(self):
        self.publish_waypoints_markers()
        if self._stop_request:
            self.get_logger().info("Waiting for start trigger")
            return

        current_waypoint = self.waypoint_manager.get_waypoint()
        if self._check_reached(waypoint=current_waypoint):
            self.get_logger().info(
                f"Waypoint {current_waypoint.index} reached. Next waypoint: {current_waypoint.index + 1}")
            self._on_reached(current_waypoint)

            if self.waypoint_manager.is_goal_reached():
                self.get_logger().info("Goal reached!")
                self._stop_request = True

                return

            self._start_following()

        else:
            self.get_logger().info(f"running...")

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
