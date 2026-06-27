"""AMCL watchdog node: AMCL の共分散を監視し、精度低下時に GNSS で再初期化する。"""
import time
import threading

import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger

from .metrics import compute


class AmclWatchdogNode(Node):
    def __init__(self):
        super().__init__('amcl_watchdog_node')

        # --- パラメータ ---
        self._metric       = self.declare_parameter('metric', 'trace_xy').value
        self._threshold    = float(self.declare_parameter('threshold', 2.0).value)
        self._consec_count = int(self.declare_parameter('consecutive_count', 3).value)
        self._backoff      = float(self.declare_parameter('recovery_backoff_sec', 20.0).value)
        self._max_retries  = int(self.declare_parameter('max_retries', 3).value)
        self._svc_timeout  = float(self.declare_parameter('initializer.call_timeout_sec', 5.0).value)
        self._min_interval = float(self.declare_parameter('min_interval_between_events', 0.0).value)

        # --- 状態変数 ---
        self._consec       = 0           # 連続超過カウンタ
        self._last_event   = 0.0         # 最後に異常検知した時刻 (フラッピング抑止)
        self._last_recovery = 0.0        # 最後にリカバリ開始した時刻 (バックオフ)
        self._in_recovery  = False
        self._lock         = threading.Lock()

        # --- ROS インターフェース ---
        self._reinit_client = self.create_client(Trigger, 'request_reinit')
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, 10)

        self.get_logger().info(
            f"amcl_watchdog started: metric={self._metric} threshold={self._threshold} "
            f"consec={self._consec_count}"
        )

    # ------------------------------------------------------------------ #
    # コールバック
    # ------------------------------------------------------------------ #

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            if self._in_recovery:
                return

        try:
            value = compute(self._metric, msg.pose.covariance)
        except ValueError as e:
            self.get_logger().warning(f"metric error: {e}")
            return

        self.get_logger().debug(f"{self._metric}={value:.4f}")

        if not self._is_anomaly(value):
            return

        now = time.monotonic()
        if now - self._last_recovery < self._backoff:
            self.get_logger().info("recovery suppressed (backoff)")
            return

        threading.Thread(
            target=self._recovery_thread, args=(value,), daemon=True
        ).start()

    # ------------------------------------------------------------------ #
    # 異常判定（状態を持つカウンタ）
    # ------------------------------------------------------------------ #

    def _is_anomaly(self, value: float) -> bool:
        now = time.monotonic()
        if value > self._threshold:
            self._consec += 1
        else:
            self._consec = 0
            return False

        if self._consec < self._consec_count:
            return False
        if now - self._last_event < self._min_interval:
            return False

        self._last_event = now
        self._consec = 0
        return True

    # ------------------------------------------------------------------ #
    # リカバリ（バックグラウンドスレッド）
    # ------------------------------------------------------------------ #

    def _recovery_thread(self, metric_value: float) -> None:
        with self._lock:
            self._in_recovery = True
        try:
            self._do_recovery(metric_value)
        finally:
            with self._lock:
                self._in_recovery = False

    def _do_recovery(self, metric_value: float) -> None:
        self.get_logger().info(
            f"anomaly detected: {self._metric}={metric_value:.4f} (threshold={self._threshold})"
        )
        msg = ""
        for attempt in range(1, self._max_retries + 1):
            self.get_logger().info(f"reinit attempt {attempt}/{self._max_retries}")
            ok, msg = self._call_reinit()
            if ok:
                self.get_logger().info(f"reinit succeeded: {msg}")
                self._last_recovery = time.monotonic()
                return
            if attempt < self._max_retries:
                time.sleep(1.0)
        self.get_logger().error(f"reinit failed after {self._max_retries} attempts: {msg}")

    def _call_reinit(self) -> tuple[bool, str]:
        """Trigger サービスを呼び出し (ok, message) を返す。"""
        if not self._reinit_client.wait_for_service(timeout_sec=1.0):
            return False, "service not available"

        future = self._reinit_client.call_async(Trigger.Request())
        start = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=self._svc_timeout)
        while not future.done():
            if (self.get_clock().now() - start) > timeout:
                return False, "timed out"
            time.sleep(0.05)

        result = future.result()
        if result is None:
            return False, "call failed"
        return bool(result.success), result.message


def main(args=None):
    rclpy.init(args=args)
    node = AmclWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down amcl_watchdog")
    finally:
        node.destroy_node()
        rclpy.shutdown()
