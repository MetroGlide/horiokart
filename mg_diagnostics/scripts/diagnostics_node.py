#!/usr/bin/env python3
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from mg_msgs.msg import SequencerStatus


class TopicRateMonitor:
    """トピックの配信レートを滑動平均で計測する。"""

    def __init__(self, window: int = 10):
        self._timestamps: deque = deque(maxlen=window)
        self._lock = threading.Lock()

    def tick(self) -> None:
        with self._lock:
            self._timestamps.append(time.monotonic())

    def hz(self) -> float:
        with self._lock:
            if len(self._timestamps) < 2:
                return 0.0
            span = self._timestamps[-1] - self._timestamps[0]
            if span <= 0.0:
                return 0.0
            return (len(self._timestamps) - 1) / span


_BEST_EFFORT_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')

        self._declare_parameters()

        self._sequencer_monitor = TopicRateMonitor()
        self._amcl_monitor = TopicRateMonitor()
        self._amcl_covariance_trace_xy: float = 0.0

        self.create_subscription(
            SequencerStatus,
            'waypoint_sequencer_node/status',
            lambda msg: self._sequencer_monitor.tick(),
            _BEST_EFFORT_QOS,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._on_amcl_pose,
            10,
        )

        self._pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._publish)

    def _declare_parameters(self) -> None:
        self.declare_parameter('monitored_nodes', [
            'waypoint_sequencer_node',
            'amcl',
            'bt_navigator',
            'controller_server',
            'planner_server',
        ])
        self.declare_parameter('sequencer_expected_hz', 10.0)
        self.declare_parameter('amcl_expected_hz', 1.0)
        self.declare_parameter('hz_warn_ratio', 0.5)
        self.declare_parameter('amcl_covariance_warn', 2.0)
        self.declare_parameter('amcl_covariance_error', 10.0)

        self._monitored_nodes: list[str] = list(
            self.get_parameter(
                'monitored_nodes').get_parameter_value().string_array_value
        )
        self._sequencer_expected_hz: float = (
            self.get_parameter(
                'sequencer_expected_hz').get_parameter_value().double_value
        )
        self._amcl_expected_hz: float = (
            self.get_parameter(
                'amcl_expected_hz').get_parameter_value().double_value
        )
        self._hz_warn_ratio: float = (
            self.get_parameter(
                'hz_warn_ratio').get_parameter_value().double_value
        )
        self._amcl_warn: float = (
            self.get_parameter(
                'amcl_covariance_warn').get_parameter_value().double_value
        )
        self._amcl_error: float = (
            self.get_parameter(
                'amcl_covariance_error').get_parameter_value().double_value
        )

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl_monitor.tick()
        cov = msg.pose.covariance
        self._amcl_covariance_trace_xy = cov[0] + cov[7]

    def _make_hz_status(self, name: str, hz: float, expected: float) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = f'topic/{name}'
        status.hardware_id = 'ros2_topic'
        status.values.append(KeyValue(key='hz', value=f'{hz:.2f}'))
        status.values.append(
            KeyValue(key='expected_hz', value=f'{expected:.2f}'))

        if hz >= expected * self._hz_warn_ratio:
            status.level = DiagnosticStatus.OK
            status.message = f'{hz:.1f} Hz'
        elif hz > 0.0:
            status.level = DiagnosticStatus.WARN
            status.message = f'low rate: {hz:.1f} Hz (expected {expected:.1f})'
        else:
            status.level = DiagnosticStatus.WARN
            status.message = 'no data'

        return status

    def _check_nodes(self) -> list[DiagnosticStatus]:
        alive_names = {name for name,
                       _ in self.get_node_names_and_namespaces()}
        statuses = []
        for node_name in self._monitored_nodes:
            status = DiagnosticStatus()
            status.name = f'node/{node_name}'
            status.hardware_id = 'ros2_node'
            if node_name in alive_names:
                status.level = DiagnosticStatus.OK
                status.message = 'alive'
            else:
                status.level = DiagnosticStatus.WARN
                status.message = 'not found'
            statuses.append(status)
        return statuses

    def _check_topics(self) -> list[DiagnosticStatus]:
        return [
            self._make_hz_status(
                'waypoint_sequencer_node/status',
                self._sequencer_monitor.hz(),
                self._sequencer_expected_hz,
            ),
            self._make_hz_status(
                'amcl_pose',
                self._amcl_monitor.hz(),
                self._amcl_expected_hz,
            ),
        ]

    def _check_amcl_quality(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = 'localization/amcl_covariance'
        status.hardware_id = 'amcl'
        val = self._amcl_covariance_trace_xy
        status.values.append(KeyValue(key='trace_xy', value=f'{val:.4f}'))

        if val >= self._amcl_error:
            status.level = DiagnosticStatus.ERROR
            status.message = f'covariance too large: {val:.3f}'
        elif val >= self._amcl_warn:
            status.level = DiagnosticStatus.WARN
            status.message = f'covariance elevated: {val:.3f}'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'ok: {val:.3f}'

        return status

    def _publish(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.extend(self._check_nodes())
        msg.status.extend(self._check_topics())
        msg.status.append(self._check_amcl_quality())
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
