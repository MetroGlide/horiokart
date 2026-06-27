"""Recovery handlers: abstract interface and concrete implementations.

Handlers receive a RecoveryContext and attempt to reinitialize AMCL.
"""
from abc import ABC, abstractmethod
import time
from typing import Optional
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from .detector import RecoveryContext, RecoveryResult


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
            self._client = self._node.create_client(
                Trigger, self._service_name)

    def attempt_recovery(self, ctx: RecoveryContext) -> RecoveryResult:
        if not self._service_name or self._client is None:
            return RecoveryResult(success=False, message='no service_name configured')
        if not self._client.wait_for_service(timeout_sec=1.0):
            return RecoveryResult(success=False, message=f'service {self._service_name} not available')

        req = Trigger.Request()
        future = self._client.call_async(req)

        # Poll the future done status from background thread to avoid
        # using spin_until_future_complete, which conflicts with the main executor.
        start_time = self._node.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=self._call_timeout_sec)
        while not future.done():
            if (self._node.get_clock().now() - start_time) > timeout:
                return RecoveryResult(success=False, message=f"service {self._service_name} call timed out")
            time.sleep(0.05)

        if future.result() is not None:
            res = future.result()
            ok = getattr(res, 'success', False)
            msg = getattr(res, 'message', '')
            return RecoveryResult(success=bool(ok), message=f"service {self._service_name}: {msg}")
        return RecoveryResult(success=False, message=f"service {self._service_name} call failed")

