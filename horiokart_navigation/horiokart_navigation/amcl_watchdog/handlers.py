"""Recovery handlers: abstract interface and concrete implementations.

Handlers receive a RecoveryContext and attempt to reinitialize AMCL.
Default strategy: publish to /initialpose or call a std_srvs/Trigger service.
"""
from abc import ABC, abstractmethod
import time
from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger

from .types import RecoveryContext, RecoveryResult


class RecoveryHandler(ABC):
    @abstractmethod
    def attempt_recovery(self, ctx: RecoveryContext) -> RecoveryResult:
        pass


class InitialPosePublisherHandler(RecoveryHandler):
    def __init__(self, node: Node, topic_name: str = '/initialpose'):
        self._node = node
        self._topic = topic_name
        self._pub = self._node.create_publisher(
            PoseWithCovarianceStamped, self._topic, 1)

    def attempt_recovery(self, ctx: RecoveryContext) -> RecoveryResult:
        # If we have an amcl_pose_msg, reuse its pose; otherwise publish empty pose (0,0,0)
        msg = PoseWithCovarianceStamped()
        if ctx.amcl_pose_msg is not None:
            try:
                # try to copy pose/covariance if the message type matches
                msg.header = ctx.amcl_pose_msg.header
                msg.pose = ctx.amcl_pose_msg.pose
            except Exception:
                # fallback: leave default
                pass
        self._pub.publish(msg)
        self._node.get_logger().info(f"Published initialpose to {self._topic}")
        return RecoveryResult(success=True, message=f"published to {self._topic}")


class GnssAmclInitializerHandler(RecoveryHandler):
    """Try calling a Trigger-like service; fallback to topic publish if service not available."""

    def __init__(self, node: Node, service_name: Optional[str] = None, fallback_topic: str = '/initialpose', call_timeout_sec: float = 5.0):
        self._node = node
        self._service_name = service_name
        self._fallback_topic = fallback_topic
        self._call_timeout_sec = float(call_timeout_sec)
        # prepare fallback publisher
        self._pub = self._node.create_publisher(
            PoseWithCovarianceStamped, self._fallback_topic, 1)

    def attempt_recovery(self, ctx: RecoveryContext) -> RecoveryResult:
        if self._service_name:
            client = self._node.create_client(Trigger, self._service_name)
            if not client.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().warning(
                    f"Service {self._service_name} not available, falling back to topic")
            else:
                req = Trigger.Request()
                future = client.call_async(req)
                # block here with spin_until_future_complete but do not block the entire process (caller should call in thread)
                rclpy.spin_until_future_complete(
                    self._node, future, timeout_sec=self._call_timeout_sec)
                if future.done() and future.result() is not None:
                    res = future.result()
                    # std_srvs/Trigger has 'success' and 'message'
                    ok = getattr(res, 'success', False)
                    msg = getattr(res, 'message', '')
                    return RecoveryResult(success=bool(ok), message=f"service {self._service_name}: {msg}")
                else:
                    return RecoveryResult(success=False, message=f"service {self._service_name} call failed or timed out")

        # fallback: publish initialpose (may be consumed by some initializer nodes)
        msg = PoseWithCovarianceStamped()
        if ctx.amcl_pose_msg is not None:
            try:
                msg.header = ctx.amcl_pose_msg.header
                msg.pose = ctx.amcl_pose_msg.pose
            except Exception:
                pass
        self._pub.publish(msg)
        self._node.get_logger().info(
            f"Fallback: published initialpose to {self._fallback_topic}")
        return RecoveryResult(success=True, message=f"published to {self._fallback_topic}")
