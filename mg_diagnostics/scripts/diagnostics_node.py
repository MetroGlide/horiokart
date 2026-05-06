#!/usr/bin/env python3
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav2_msgs.msg import CollisionDetectorState
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

MONITORED_TOPICS: list[dict] = [
    {
        "name": "waypoint_sequencer_node/status",
        "msg_type": SequencerStatus,
        "expected_hz": 10.0,
        "qos": _BEST_EFFORT_QOS,
    },
    {
        "name": "amcl_pose",
        "msg_type": PoseWithCovarianceStamped,
        "expected_hz": 1.0,
        "qos": None,
    },
    {
        "name": "cmd_vel",
        "msg_type": Twist,
        "expected_hz": 10.0,
        "qos": _BEST_EFFORT_QOS,
    },
    {
        "name": "odom",
        "msg_type": Odometry,
        "expected_hz": 20.0,
        "qos": _BEST_EFFORT_QOS,
    },
    {
        "name": "scan_top_lidar",
        "msg_type": LaserScan,
        "expected_hz": 10.0,
        "qos": _BEST_EFFORT_QOS,
    },
    {
        "name": "scan_front_lidar",
        "msg_type": LaserScan,
        "expected_hz": 10.0,
        "qos": _BEST_EFFORT_QOS,
    },
    {
        "name": "motor_driver_node/emergency_stop",
        "msg_type": Bool,
        "expected_hz": 1.0,
        "qos": None,
    },
    {
        "name": "collision_detector_state",
        "msg_type": CollisionDetectorState,
        "expected_hz": 5.0,
        "qos": _BEST_EFFORT_QOS,
    },
]


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')

        self._declare_parameters()

        self._rate_monitors: dict[str, TopicRateMonitor] = {}
        for topic in MONITORED_TOPICS:
            name = topic["name"]
            monitor = TopicRateMonitor()
            self._rate_monitors[name] = monitor
            qos = topic["qos"] if topic["qos"] is not None else 10
            self.create_subscription(
                topic["msg_type"],
                name,
                lambda msg, m=monitor: m.tick(),
                qos,
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
            'collision_monitor',
            'motor_driver_node',
        ])
        self.declare_parameter('hz_warn_ratio', 0.5)

        self._monitored_nodes: list[str] = list(
            self.get_parameter(
                'monitored_nodes').get_parameter_value().string_array_value
        )
        self._hz_warn_ratio: float = (
            self.get_parameter(
                'hz_warn_ratio').get_parameter_value().double_value
        )

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
                topic["name"],
                self._rate_monitors[topic["name"]].hz(),
                topic["expected_hz"],
            )
            for topic in MONITORED_TOPICS
        ]

    def _publish(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.extend(self._check_nodes())
        msg.status.extend(self._check_topics())
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
