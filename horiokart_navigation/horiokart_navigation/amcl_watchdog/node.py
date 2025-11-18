"""AMCL watchdog node implementation placed inside the python package.

This module is imported by the installed script wrapper so that package
imports work correctly after `colcon build` and `source install/setup.bash`.
"""
from typing import Optional
import threading
import time

from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

from .metrics import compute
from .detectors import ConsecutiveThresholdDetector
from .handlers import GnssAmclInitializerHandler, InitialPosePublisherHandler
from .types import RecoveryContext


class AmclWatchdogNode(Node):
    def __init__(self):
        super().__init__('amcl_watchdog')

        # parameters
        metric = self.declare_parameter(
            'metric', 'trace_xy').get_parameter_value().string_value
        threshold = self.declare_parameter(
            'threshold', 2.0).get_parameter_value().double_value
        consecutive_count = int(self.declare_parameter(
            'consecutive_count', 3).get_parameter_value().integer_value)
        monitor_topic = self.declare_parameter(
            'monitor_topic', '/amcl_pose').get_parameter_value().string_value
        initializer_type = self.declare_parameter(
            'initializer.type', 'service').get_parameter_value().string_value
        initializer_service = self.declare_parameter(
            'initializer.service_name', '/gnss_amcl_initializer_node/request_reinit').get_parameter_value().string_value
        initializer_topic = self.declare_parameter(
            'initializer.topic_name', '/initialpose').get_parameter_value().string_value
        call_timeout = float(self.declare_parameter(
            'initializer.call_timeout_sec', 5.0).get_parameter_value().double_value)
        self._recovery_backoff = float(self.declare_parameter(
            'recovery_backoff_sec', 60.0).get_parameter_value().double_value)
        self._max_retries = int(self.declare_parameter(
            'max_retries', 3).get_parameter_value().integer_value)
        min_interval = float(self.declare_parameter(
            'min_interval_between_events', 0.0).get_parameter_value().double_value)

        self.metric_name = metric
        self.threshold = float(threshold)

        # components
        self.detector = ConsecutiveThresholdDetector(
            metric_name=metric, threshold=self.threshold, consecutive_count=consecutive_count, min_interval_between_events=min_interval)

        if initializer_type == 'topic':
            self.recovery_handler = InitialPosePublisherHandler(
                self, topic_name=initializer_topic)
        else:
            # default: try to call service, fallback to topic
            self.recovery_handler = GnssAmclInitializerHandler(
                self, service_name=initializer_service, fallback_topic=initializer_topic, call_timeout_sec=call_timeout)

        self._last_recovery_time = 0.0

        # subscription
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, monitor_topic, self._on_amcl_pose, 10)
        self.subscription  # prevent unused

        self.get_logger().info(
            f"amcl_watchdog started: metric={metric} threshold={self.threshold} consecutive_count={consecutive_count}")

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        # safe compute metric
        try:
            metric_value = compute(self.metric_name, msg.pose.covariance)
        except Exception as e:
            self.get_logger().warning(f"Failed to compute metric: {e}")
            return

        self.get_logger().debug(f"metric {self.metric_name}={metric_value}")

        event = self.detector.feed(metric_value)
        if event is None:
            return

        # check backoff
        now = time.time()
        if now - self._last_recovery_time < self._recovery_backoff:
            self.get_logger().info("Recovery suppressed due to backoff")
            return

        # run recovery in background thread to avoid blocking subscription callbacks
        t = threading.Thread(target=self._run_recovery,
                             args=(msg, event), daemon=True)
        t.start()

    def _run_recovery(self, amcl_msg: PoseWithCovarianceStamped, event) -> None:
        self.get_logger().info(
            f"Anomaly detected: {event.metric_name}={event.metric_value} threshold={event.threshold}")
        ctx = RecoveryContext(
            amcl_pose_msg=amcl_msg, metric_name=event.metric_name, metric_value=event.metric_value)

        tries = 0
        success = False
        last_msg = ''
        while tries < self._max_retries and not success:
            tries += 1
            self.get_logger().info(
                f"Attempting recovery (try {tries}/{self._max_retries})")
            try:
                res = self.recovery_handler.attempt_recovery(ctx)
                success = bool(res.success)
                last_msg = res.message or ''
            except Exception as e:
                last_msg = str(e)
                success = False
            if not success:
                time.sleep(1.0)

        if success:
            self.get_logger().info(f"Recovery succeeded: {last_msg}")
            self._last_recovery_time = time.time()
        else:
            self.get_logger().error(
                f"Recovery failed after {tries} tries: {last_msg}")


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = AmclWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down amcl_watchdog')
    finally:
        node.destroy_node()
        rclpy.shutdown()
