"""AMCL watchdog node implementation placed inside the python package.

This module is imported by the installed script wrapper so that package
"""
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

from .metrics import compute
from .detector import ConsecutiveThresholdDetector, RecoveryContext
from .handlers import GnssAmclInitializerHandler



class AmclWatchdogNode(Node):
    def __init__(self):
        super().__init__('amcl_watchdog_node')

        # declare and read parameters
        self._declare_parameters()

        # initialize components and ROS communications
        self._init_communications()

        # state
        self._last_recovery_time = 0.0
        self._recovery_lock = threading.Lock()
        self._in_recovery = False

        self.get_logger().info(
            f"amcl_watchdog started: metric={self.metric_name} threshold={self.threshold} consecutive_count={self.consecutive_count}")

    def _declare_parameters(self) -> None:
        """
        Declare and read node parameters (only operational parameters).
        """
        self.metric_name = self.declare_parameter(
            'metric', 'trace_xy').get_parameter_value().string_value
        self.threshold = float(self.declare_parameter(
            'threshold', 2.0).get_parameter_value().double_value)
        self.consecutive_count = int(self.declare_parameter(
            'consecutive_count', 3).get_parameter_value().integer_value)
        self.initializer_type = self.declare_parameter(
            'initializer.type', 'service').get_parameter_value().string_value
        self.call_timeout = float(self.declare_parameter(
            'initializer.call_timeout_sec', 5.0).get_parameter_value().double_value)
        self._recovery_backoff = float(self.declare_parameter(
            'recovery_backoff_sec', 20.0).get_parameter_value().double_value)
        self._max_retries = int(self.declare_parameter(
            'max_retries', 3).get_parameter_value().integer_value)
        self.min_interval = float(self.declare_parameter(
            'min_interval_between_events', 0.0).get_parameter_value().double_value)

        # detector config
        self.detector = ConsecutiveThresholdDetector(metric_name=self.metric_name, threshold=self.threshold,
                                                     consecutive_count=self.consecutive_count, min_interval_between_events=self.min_interval)

    def _init_communications(self) -> None:
        """
        Initialize ROS publishers/subscriptions/clients and recovery handler.
        """
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl_pose, 10)

        # Recovery handler selection
        if self.initializer_type == 'service':
            self.recovery_handler = GnssAmclInitializerHandler(
                self, service_name='request_reinit', call_timeout_sec=self.call_timeout)
        else:
            self.get_logger().error(
                f"Unknown initializer.type '{self.initializer_type}'; no recovery handler configured")
            raise RuntimeError(
                f"Unknown initializer.type '{self.initializer_type}'")

        # Recovery is executed in a background thread to avoid blocking the main
        # executor loop. The handler performs non-blocking polling on service futures.

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        # safe compute metric
        # If we're already performing recovery, ignore incoming pose samples.
        with self._recovery_lock:
            if self._in_recovery:
                return
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

        # Run recovery in a background thread to avoid blocking the main
        # rclpy spin loop. The handler performs non-blocking polling on service
        # futures; running it in a separate thread prevents interference with the
        # node's executor.

        def _recovery_worker(amcl_msg: PoseWithCovarianceStamped, ev):
            with self._recovery_lock:
                self._in_recovery = True
            try:
                self._run_recovery(amcl_msg, ev)
            finally:
                with self._recovery_lock:
                    self._in_recovery = False

        t = threading.Thread(target=_recovery_worker,
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
    rclpy.init(args=args)
    node = AmclWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down amcl_watchdog')
    finally:
        node.destroy_node()
        rclpy.shutdown()
