"""Anomaly detectors for scalar metrics."""
from abc import ABC, abstractmethod
import time
from typing import Optional
from .types import AnomalyEvent


class AnomalyDetector(ABC):
    @abstractmethod
    def feed(self, value: float) -> Optional[AnomalyEvent]:
        """Feed a new metric value. Return AnomalyEvent when anomaly detected."""


class ConsecutiveThresholdDetector(AnomalyDetector):
    def __init__(self, metric_name: str, threshold: float, consecutive_count: int, min_interval_between_events: float = 0.0):
        self.metric_name = metric_name
        self.threshold = float(threshold)
        self.consecutive_count = int(consecutive_count)
        self.min_interval_between_events = float(min_interval_between_events)
        self._count = 0
        self._last_event_time = 0.0

    def feed(self, value: float) -> Optional[AnomalyEvent]:
        now = time.time()
        if value is None:
            # ignore
            self._count = 0
            return None
        if value > self.threshold:
            self._count += 1
        else:
            self._count = 0

        if self._count >= self.consecutive_count:
            if now - self._last_event_time < self.min_interval_between_events:
                # suppressed due to min interval (flapping guard)
                return None
            self._last_event_time = now
            # reset counter to avoid immediate re-fire; keep behavior configurable later
            self._count = 0
            return AnomalyEvent(metric_name=self.metric_name, metric_value=value, threshold=self.threshold, consecutive_count=self.consecutive_count)
        return None
