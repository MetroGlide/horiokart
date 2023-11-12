#!/usr/bin/env python3
import os

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.clock import Clock
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Int16

from action_msgs.msg import GoalStatus

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap

from dataclasses import dataclass
from typing import Any, List
import time

from horiokart_navigation.waypoint import WaypointList, Waypoint, WaypointsLoader, OnReachedAction

from ament_index_python.packages import get_package_share_directory


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
    goal_status: int = GoalStatus.STATUS_UNKNOWN
    current_pose: PoseStamped = PoseStamped()
    distance_remaining: float = 0.0
    navigation_time: float = 0.0
    number_of_recoveries: int = 0
    estimated_time_remaining: float = 0.0


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

        self.current_following_status = FollowingStatus()

        self.goal_handle = None

    def send_goal(self, waypoint: Waypoint):
        if self._stop_request:
            self.node.get_logger().info("Stop requested")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint.pose
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

        pkg_name = "horiokart_navigation"
        # self.node.get_logger().info(f"pkg_name: {pkg_name}")

        goal_msg.behavior_tree = get_package_share_directory(
            pkg_name) + "/behavior_trees/mg_navigate_to_pose_recovery_only_wait.xml"

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
            distance_remaining=feedback_msg.feedback.distance_remaining,
            navigation_time=feedback_msg.feedback.navigation_time,
            number_of_recoveries=feedback_msg.feedback.number_of_recoveries,
            estimated_time_remaining=feedback_msg.feedback.estimated_time_remaining)

    def _send_goal_done_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error("Goal rejected")
            return

        self.current_following_status = FollowingStatus(
            goal_status=self.goal_handle.status,
            current_pose=PoseStamped()
        )

        self.node.get_logger().info("Goal accepted")
        self._result_future = self.goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.node.get_logger().info("Goal result received")

        self.current_following_status = FollowingStatus(
            goal_status=self.goal_handle.status,
            current_pose=PoseStamped()
        )


class RetryWaiter:
    def __init__(self, node, retry_duration_sec=2.0, retry_func=None):
        self.node = node
        self.retry_duration_sec = retry_duration_sec

        self._retry_start_time = None
        self._retry_func = retry_func

    def tick(self):
        if self._retry_start_time is None:
            self._retry_start_time = self.node.get_clock().now()
            return

        if self.node.get_clock().now() - self._retry_start_time >= rclpy.duration.Duration(seconds=self.retry_duration_sec):
            self.node.get_logger().error(
                f"Error detected. Retry function called. func name: {self._retry_func.__name__}")
            self._retry_start_time = None
            if self._retry_func is not None:
                self._retry_func()

    def reset(self):
        self._retry_start_time = None


class WaypointsFollowerNode(Node):
    class ServiceFuture:
        def __init__(self, future, service_name, logger, timeout_sec=5.0, callback=None):
            self.future = future
            self.service_name = service_name
            self.logger = logger
            self.timeout_sec = timeout_sec
            self.callback = callback

        def __bool__(self):
            # TODO: implement timeout

            if self.future.done():
                try:
                    response = self.future.result()
                    self.logger.info(
                        f"Service call done: {self.service_name}")

                    if self.callback is not None:
                        self.callback(response, self.logger)

                except Exception as e:
                    self.get_logger().error(
                        f"Service call failed: {self.service_name}. Exception: {e}")
                return True

            return False

    def __init__(self):
        super().__init__('waypoint_follower_node')

        load_path = self.declare_parameter(
            # 'load_path', "/root/ros2_data/new_waypoints.yaml").value
            'load_path', "/root/ros2_data/map/waypoints_list.yaml").value

        self.through_point_tolerance = self.declare_parameter(
            'through_point_tolerance', 3.0).value

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

        self._load_localization_map_srv_client = self.create_client(
            LoadMap, "map_server/load_map")
        self._load_localization_map_srv_client.wait_for_service()

        self._load_planning_map_srv_client = self.create_client(
            LoadMap, "planning_map_server/load_map")
        self._load_planning_map_srv_client.wait_for_service()

        self._change_front_lidar_publish_state_srv_client = self.create_client(
            SetBool, "front_lidar_publish_controller_node/change_publish_state")

        self._change_amcl_publish_state_srv_client = self.create_client(
            SetBool, "amcl_publish_controller_node/change_publish_state")

        self._amcl_initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "initialpose",
            1)

        self._on_reached_actions_progress_list = []

        self.get_logger().info("Waypoints follower node initialized")

    def _stop_callback(self, request, response) -> Trigger.Response:
        response.success = self._stop_following()
        return response

    def _stop_following(self) -> bool:
        if self._stop_request:
            self.get_logger().error("Already stopped")
            return False

        self.get_logger().info("Stop following waypoints")
        self._stop_request = True
        self._waypoints_follower.stop_request()
        return True

    def _start_callback(self, request, response) -> Trigger.Response:
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
        self.get_logger().debug(
            f"Goal status: {self._waypoints_follower.current_following_status.goal_status}"
        )
        if self._waypoints_follower.current_following_status.goal_status != GoalStatus.STATUS_EXECUTING:
            self.get_logger().error(
                f"Goal status is not executing: {self._waypoints_follower.current_following_status.goal_status}")
            return True

        if self._waypoints_follower.current_following_status.distance_remaining == 0.0 \
                and self._waypoints_follower.current_following_status.estimated_time_remaining == 0.0:
            self.get_logger().error(
                f"Distance remaining: 0.0. Estimated time: 0.0. Maybe error. Retry")
            return True

        if self._check_actual_reached(waypoint):
            return True

        return False

    def _check_actual_reached(self, waypoint: Waypoint) -> bool:
        distance = self.get_distance(waypoint)
        if waypoint.is_through_point and distance <= self.through_point_tolerance:
            self.get_logger().info(
                f"Reach point! Distance remaining: {distance}. Through point tolerance: {self.through_point_tolerance}")
            return True
        elif distance <= waypoint.reach_tolerance:
            self.get_logger().info(
                f"Reach point! Distance remaining: {distance}. Reach tolerance: {waypoint.reach_tolerance}")
            return True
        else:
            return False

    def _check_reached_actions_done(self):
        if all(self._on_reached_actions_progress_list):
            self._on_reached_actions_progress_list = []
            return True

        return False

    def _reload_map_response_callback(self, response, logger):
        if response.result == LoadMap.Response.RESULT_SUCCESS:
            logger.info(f"Reload map success")
        else:
            result_str = "Unknown"
            if response.result == LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST:
                result_str = "Map does not exist"
            elif response.result == LoadMap.Response.RESULT_INVALID_MAP_DATA:
                result_str = "Invalid map data"
            elif response.result == LoadMap.Response.RESULT_INVALID_MAP_METADATA:
                result_str = "Invalid map metadata"
            elif response.result == LoadMap.Response.RESULT_UNDEFINED_FAILURE:
                result_str = "Undefined failure"

            logger.error(
                f"Reload map failed: {result_str}")

    def _on_reached_action_reload_map(self, waypoint: Waypoint):
        map_path = os.environ.get("MAP_PATH", "")
        if map_path == "":
            self.get_logger().error(
                "MAP_PATH environment variable is not set")
            return

        if waypoint.localization_map_yaml != "":
            # map directory from environment variable MAP_PATH
            load_localization_map_yaml_path = os.path.join(
                map_path, waypoint.localization_map_yaml)

            self.get_logger().info(
                f"Load localization map yaml: {load_localization_map_yaml_path}")

            self._load_localization_map_srv_client.wait_for_service()
            request = LoadMap.Request()
            request.map_url = load_localization_map_yaml_path
            future = self._load_localization_map_srv_client.call_async(
                request)

            self._on_reached_actions_progress_list.append(
                self.ServiceFuture(future, "load_localization_map", self.get_logger(),
                                   callback=self._reload_map_response_callback))

        if waypoint.planning_map_yaml != "":
            load_planning_map_yaml_path = os.path.join(
                map_path, waypoint.planning_map_yaml)

            self.get_logger().info(
                f"Load planning map yaml: {load_planning_map_yaml_path}")

            self._load_planning_map_srv_client.wait_for_service()
            request = LoadMap.Request()
            request.map_url = load_planning_map_yaml_path
            future = self._load_planning_map_srv_client.call_async(
                request)

            self._on_reached_actions_progress_list.append(
                self.ServiceFuture(future, "load_planning_map", self.get_logger(),
                                   callback=self._reload_map_response_callback))

        self.get_logger().info(f"Reload map action resistered")

    def _on_reached_action_front_lidar_on_off(self, waypoint: Waypoint, state: bool):
        self._change_front_lidar_publish_state_srv_client.wait_for_service()
        request = SetBool.Request()
        request.data = state

        future = self._change_front_lidar_publish_state_srv_client.call_async(
            request)

        self._on_reached_actions_progress_list.append(
            self.ServiceFuture(future, "change_front_lidar_publish_state", self.get_logger(),
                               callback=self._change_front_lidar_publish_state_callback))

    def _on_reached_action_amcl_on_off(self, waypoint: Waypoint, state: bool):
        if state:
            # reset amcl init pose
            for _ in range(5):
                try:
                    transform = self._tf_buffer.lookup_transform(
                        "map", "base_footprint", rclpy.time.Time())
                    break
                except Exception as e:
                    self.get_logger().error(f"Failed to lookup transform: {e}")

            if transform is not None:
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.pose.position = Point(
                    x=transform.transform.translation.x,
                    y=transform.transform.translation.y,
                    z=transform.transform.translation.z
                )
                pose.pose.pose.orientation = transform.transform.rotation

                pose.pose.covariance = [
                    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
                ]

                self._amcl_initialpose_publisher.publish(pose)
                self.get_logger().info(f"Reset amcl init pose")

        self._change_amcl_publish_state_srv_client.wait_for_service()
        request = SetBool.Request()
        request.data = state

        future = self._change_amcl_publish_state_srv_client.call_async(
            request)

        self._on_reached_actions_progress_list.append(
            self.ServiceFuture(future, "change_amcl_publish_state", self.get_logger(),
                               callback=self._change_amcl_publish_state_callback))

    def _change_front_lidar_publish_state_callback(self, response, logger):
        if response.success:
            logger.info(f"Change front lidar publish state success")
        else:
            logger.error(f"Change front lidar publish state failed")

    def _change_amcl_publish_state_callback(self, response, logger):
        if response.success:
            logger.info(f"Change amcl publish state success")
        else:
            logger.error(f"Change amcl publish state failed")

    def _on_reached_action(self, waypoint: Waypoint, on_reached_action: OnReachedAction):
        if OnReachedAction.WAIT_TRIGGER == on_reached_action:
            self.get_logger().info(
                f"OnReachedAction: Wait for trigger to continue to next waypoint")
            # self._stop_following()
            self._stop_request = True

        elif OnReachedAction.RELOAD_MAP == on_reached_action:
            self.get_logger().info(
                f"OnReachedAction: Reload map")

            self._on_reached_action_reload_map(waypoint)

        elif OnReachedAction.FROMT_LIDAR_OFF == on_reached_action:
            self.get_logger().info(
                f"OnReachedAction: Front lidar off")

            self._on_reached_action_front_lidar_on_off(waypoint, False)

        elif OnReachedAction.FRONT_LIDAR_ON == on_reached_action:
            self.get_logger().info(
                f"OnReachedAction: Front lidar on")

            self._on_reached_action_front_lidar_on_off(waypoint, True)

        elif OnReachedAction.AMCL_ON == on_reached_action:
            self.get_logger().info(
                f"OnReachedAction: AMCL on")

            self._on_reached_action_amcl_on_off(waypoint, True)

        elif OnReachedAction.AMCL_OFF == on_reached_action:
            self.get_logger().info(
                f"OnReachedAction: AMCL off")

            self._on_reached_action_amcl_on_off(waypoint, False)

    def _on_reached(self, waypoint: Waypoint):
        for on_reached_action in waypoint.on_reached_action:
            self._on_reached_action(waypoint, on_reached_action)

        self.waypoint_manager.reached()

    def _on_goal_reached(self):
        self.get_logger().info("Goal reached!")
        self._stop_request = True

    def _follow_waypoints(self):
        self.publish_waypoints_markers()
        if not self._check_reached_actions_done():
            self.get_logger().log("Waiting for reached actions done..", LoggingSeverity.INFO,
                                  throttle_duration_sec=2.0, throttle_time_source_type=self.get_clock())
            return

        if self._stop_request:
            self.get_logger().log("Waiting for start trigger..", LoggingSeverity.INFO,
                                  throttle_duration_sec=2.0, throttle_time_source_type=self.get_clock())
            return

        current_waypoint = self.waypoint_manager.get_waypoint()

        if not self._check_reached(waypoint=current_waypoint):
            self.get_logger().log(f"running to {self.waypoint_manager.get_current_index()}. Distance remaining: {self._waypoints_follower.current_following_status.distance_remaining} m",
                                  LoggingSeverity.INFO, throttle_duration_sec=2.0, throttle_time_source_type=self.get_clock())
            return

        if self._check_actual_reached(current_waypoint):  # Double check
            # When actual reached
            self.get_logger().info(
                f"Waypoint {current_waypoint.index} ACUTUAL reached! Next waypoint: {current_waypoint.index + 1}")
            self._on_reached(current_waypoint)

            if self.waypoint_manager.is_goal_reached():
                self._on_goal_reached()
                return

            self._state_unknown_retry_waiter.reset()
            self._start_following()

        else:
            # When reached but not close enough
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
