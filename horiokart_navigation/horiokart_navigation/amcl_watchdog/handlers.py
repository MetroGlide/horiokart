"""Recovery handlers: abstract interface and concrete implementations.

Handlers receive a RecoveryContext and attempt to reinitialize AMCL.
"""
from abc import ABC, abstractmethod
from typing import Optional
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from .types import RecoveryContext, RecoveryResult


class RecoveryHandler(ABC):
    @abstractmethod
    def attempt_recovery(self, ctx: RecoveryContext) -> RecoveryResult:
        pass


class GnssAmclInitializerHandler(RecoveryHandler):
    """Call a Trigger-like service to request GNSS-based AMCL initialization.
    """

    def __init__(self, node: Node, service_name: Optional[str] = 'request_reinit', call_timeout_sec: float = 5.0):
        self._node = node
        self._service_name = service_name
        self._call_timeout_sec = float(call_timeout_sec)
        self._client = None
        if self._service_name:
            self._client = self._node.create_client(Trigger, self._service_name)

    def attempt_recovery(self, ctx: RecoveryContext) -> RecoveryResult:
        if not self._service_name:
            return RecoveryResult(success=False, message='no service_name configured')
        if self._client is None:
            self._client = self._node.create_client(Trigger, self._service_name)
        if not self._client.wait_for_service(timeout_sec=1.0):
            return RecoveryResult(success=False, message=f'service {self._service_name} not available')

        req = Trigger.Request()
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=self._call_timeout_sec)
        if future.done() and future.result() is not None:
            res = future.result()
            ok = getattr(res, 'success', False)
            msg = getattr(res, 'message', '')
            return RecoveryResult(success=bool(ok), message=f"service {self._service_name}: {msg}")
        return RecoveryResult(success=False, message=f"service {self._service_name} call failed or timed out")
