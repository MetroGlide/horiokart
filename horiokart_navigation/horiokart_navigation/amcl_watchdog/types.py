from dataclasses import dataclass
from typing import Optional


@dataclass
class AnomalyEvent:
    metric_name: str
    metric_value: float
    threshold: float
    consecutive_count: int


@dataclass
class RecoveryContext:
    # original amcl pose message (may be used by handlers)
    amcl_pose_msg: object
    metric_name: str
    metric_value: float


@dataclass
class RecoveryResult:
    success: bool
    message: Optional[str] = None
