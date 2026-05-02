from __future__ import annotations

import math
import threading
import time
from typing import TYPE_CHECKING, Dict, List

import rclpy.time
import rclpy.duration
from std_msgs.msg import Int16

if TYPE_CHECKING:
    import rclpy.node
    from mg_scenario_test.adapters.base import SimulatorAdapter
    from mg_scenario_test.scenario import EventSpec, ObstacleDef, PoseSpec


class EventExecutor:
    """シナリオイベントリストを逐次実行する。

    stop_event がセットされると delay イベントで中断し、以降のイベントはスキップする。
    """

    def __init__(
        self,
        node: "rclpy.node.Node",
        adapter: "SimulatorAdapter",
        obstacles: "Dict[str, ObstacleDef]",
    ):
        self._node = node
        self._adapter = adapter
        self._obstacles = obstacles
        self._initialpose_pub = node.create_publisher(
            __import__("geometry_msgs.msg", fromlist=[
                       "PoseWithCovarianceStamped"])
            .PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )
        self._tf_buffer = __import__("tf2_ros", fromlist=["Buffer"]).Buffer()
        self._tf_listener = __import__("tf2_ros", fromlist=["TransformListener"]).TransformListener(
            self._tf_buffer, node
        )
        self._spawned_names: List[str] = []
        self.sequencer_namespace: str = "waypoint_sequencer_node"
        self._sequencer_index_pub = node.create_publisher(
            Int16,
            f"/{self.sequencer_namespace}/set_next_waypoint_index",
            1,
        )

    def execute_events(
        self, events: "List[EventSpec]", stop_event: threading.Event
    ) -> None:
        for event in events:
            if stop_event.is_set():
                break
            try:
                self._dispatch(event, stop_event)
            except Exception as e:
                self._node.get_logger().error(
                    f"[EventExecutor] Event '{event.type}' raised: {e}"
                )

    def _dispatch(self, event: "EventSpec", stop_event: threading.Event) -> None:
        t = event.type
        if t == "reset_pose":
            self._handle_reset_pose(event)
        elif t == "set_amcl_initial_pose":
            self._handle_set_amcl_initial_pose(event)
        elif t == "delay":
            self._handle_delay(event, stop_event)
        elif t == "spawn_obstacle":
            self._handle_spawn_obstacle(event)
        elif t == "despawn_obstacle":
            self._handle_despawn_obstacle(event)
        elif t == "cleanup_all_obstacles":
            self._handle_cleanup_all_obstacles()
        elif t == "trigger_waypoint":
            self._handle_trigger_waypoint(event)
        elif t == "set_sequencer_index":
            self._handle_set_sequencer_index(event)
        else:
            self._node.get_logger().warn(
                f"[EventExecutor] Unknown event type: {t}")

    def _handle_reset_pose(self, event: "EventSpec") -> None:
        if event.pose is None:
            return
        resolved = self._resolve_pose(event.pose)
        ok = self._adapter.set_robot_pose(
            self._node.get_parameter("robot_name").value
            if self._node.has_parameter("robot_name")
            else "mg",
            resolved,
        )
        if not ok:
            self._node.get_logger().error("[EventExecutor] reset_pose failed")

    def _handle_set_amcl_initial_pose(self, event: "EventSpec") -> None:
        if event.pose is None:
            return
        resolved = self._resolve_pose(event.pose)
        self._publish_initialpose(resolved)

    def _handle_delay(self, event: "EventSpec", stop_event: threading.Event) -> None:
        deadline = time.monotonic() + event.sec
        while time.monotonic() < deadline:
            if stop_event.is_set():
                break
            time.sleep(0.05)

    def _handle_spawn_obstacle(self, event: "EventSpec") -> None:
        name = event.obstacle
        if name not in self._obstacles:
            self._node.get_logger().error(
                f"[EventExecutor] Obstacle '{name}' not defined in scenario"
            )
            return
        pose_spec = event.spawn_pose or event.pose
        if pose_spec is None:
            self._node.get_logger().error(
                f"[EventExecutor] spawn_obstacle '{name}' missing pose"
            )
            return
        resolved_pose = self._resolve_pose(pose_spec)
        ok = self._adapter.spawn_entity(
            name, self._obstacles[name].model, resolved_pose)
        if ok:
            if name not in self._spawned_names:
                self._spawned_names.append(name)
        else:
            self._node.get_logger().error(
                f"[EventExecutor] spawn_obstacle '{name}' failed"
            )

    def _handle_despawn_obstacle(self, event: "EventSpec") -> None:
        name = event.obstacle
        ok = self._adapter.despawn_entity(name)
        if ok:
            self._spawned_names = [n for n in self._spawned_names if n != name]
        else:
            self._node.get_logger().warn(
                f"[EventExecutor] despawn_obstacle '{name}' failed (may not exist)"
            )

    def _handle_cleanup_all_obstacles(self) -> None:
        targets = list(self._spawned_names)
        if not targets:
            self._node.get_logger().info(
                "[EventExecutor] cleanup_all_obstacles: nothing to remove")
            return
        self._node.get_logger().info(
            f"[EventExecutor] cleanup_all_obstacles: removing {targets}"
        )
        for name in targets:
            ok = self._adapter.despawn_entity(name)
            if ok:
                self._spawned_names = [
                    n for n in self._spawned_names if n != name]
            else:
                self._node.get_logger().warn(
                    f"[EventExecutor] cleanup: despawn '{name}' failed (may not exist)"
                )

    def _handle_trigger_waypoint(self, event: "EventSpec") -> None:
        from mg_msgs.srv import StartSequence
        service_name = f"/{self.sequencer_namespace}/start"
        client = self._node.create_client(StartSequence, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error(
                f"[EventExecutor] trigger_waypoint: {service_name} not available"
            )
            return
        req = StartSequence.Request()
        req.countdown_ms = event.countdown_ms
        self._node.get_logger().info(
            f"[EventExecutor] trigger_waypoint: calling {service_name} "
            f"(countdown_ms={event.countdown_ms})"
        )
        future = client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        done.wait(timeout=10.0)
        if not future.done() or future.result() is None:
            self._node.get_logger().error(
                "[EventExecutor] trigger_waypoint: service call timed out"
            )
        elif not future.result().success:
            self._node.get_logger().warn(
                f"[EventExecutor] trigger_waypoint: {future.result().message}"
            )
        else:
            self._node.get_logger().info(
                f"[EventExecutor] trigger_waypoint: accepted ({future.result().message})"
            )

    def _handle_set_sequencer_index(self, event: "EventSpec") -> None:
        msg = Int16()
        msg.data = event.target_index
        self._sequencer_index_pub.publish(msg)
        self._node.get_logger().info(
            f"[EventExecutor] set_sequencer_index: published {event.target_index} → "
            f"/{self.sequencer_namespace}/set_next_waypoint_index"
        )

    def _resolve_pose(self, pose_spec: "PoseSpec") -> "PoseSpec":
        from mg_scenario_test.scenario import PoseSpec

        if pose_spec.frame == "absolute":
            return pose_spec

        try:
            transform = self._tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=3.0),
            )
        except Exception as e:
            self._node.get_logger().error(
                f"[EventExecutor] TF lookup map->base_footprint failed: {e}. "
                "Falling back to absolute interpretation."
            )
            return PoseSpec(
                frame="absolute",
                x=pose_spec.x,
                y=pose_spec.y,
                z=pose_spec.z,
                yaw=pose_spec.yaw,
            )

        t = transform.transform.translation
        r = transform.transform.rotation
        robot_yaw = math.atan2(
            2.0 * (r.w * r.z + r.x * r.y),
            1.0 - 2.0 * (r.y * r.y + r.z * r.z),
        )

        map_x = (
            t.x
            + pose_spec.x * math.cos(robot_yaw)
            - pose_spec.y * math.sin(robot_yaw)
        )
        map_y = (
            t.y
            + pose_spec.x * math.sin(robot_yaw)
            + pose_spec.y * math.cos(robot_yaw)
        )
        map_z = t.z + pose_spec.z
        map_yaw = robot_yaw + pose_spec.yaw

        return PoseSpec(frame="absolute", x=map_x, y=map_y, z=map_z, yaw=map_yaw)

    def _publish_initialpose(self, pose: "PoseSpec") -> None:
        from geometry_msgs.msg import PoseWithCovarianceStamped

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = 0.0
        qz = math.sin(pose.yaw / 2.0)
        qw = math.cos(pose.yaw / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        self._initialpose_pub.publish(msg)
