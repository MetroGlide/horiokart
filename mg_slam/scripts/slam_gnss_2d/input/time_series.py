from __future__ import annotations

import bisect
from typing import Optional, Sequence, TypeVar

from slam_gnss_2d.core.data_types import GnssData, OdomData
from slam_gnss_2d.core.geometry import angle_diff

T = TypeVar('T')


def nearest_by_timestamp(
    items: Sequence[T],
    timestamps: Sequence[float],
    timestamp: float,
) -> Optional[T]:
    """時刻に最も近い要素を返す。"""
    if not items:
        return None
    idx = bisect.bisect_left(timestamps, timestamp)
    if idx == 0:
        return items[0]
    if idx >= len(items):
        return items[-1]
    prev = items[idx - 1]
    next_ = items[idx]
    if abs(timestamp - timestamps[idx - 1]) <= abs(timestamp - timestamps[idx]):
        return prev
    return next_


def interpolate_odom(
    odom_list: Sequence[OdomData],
    timestamps: Sequence[float],
    timestamp: float,
) -> Optional[OdomData]:
    """OdomData を指定時刻へ線形補間して返す。"""
    if not odom_list:
        return None
    idx = bisect.bisect_left(timestamps, timestamp)
    if idx == 0:
        return odom_list[0]
    if idx >= len(odom_list):
        return odom_list[-1]
    prev = odom_list[idx - 1]
    next_ = odom_list[idx]
    t_span = next_.timestamp - prev.timestamp
    if t_span < 1e-9:
        return prev
    alpha = (timestamp - prev.timestamp) / t_span
    return OdomData(
        timestamp=timestamp,
        x=prev.x + alpha * (next_.x - prev.x),
        y=prev.y + alpha * (next_.y - prev.y),
        yaw=prev.yaw + alpha * angle_diff(next_.yaw, prev.yaw),
    )


def interpolate_gnss(
    gnss_list: Sequence[GnssData],
    timestamps: Sequence[float],
    timestamp: float,
) -> Optional[GnssData]:
    """GnssData を指定時刻へ線形補間して返す。"""
    if not gnss_list:
        return None
    idx = bisect.bisect_left(timestamps, timestamp)
    if idx == 0:
        return gnss_list[0]
    if idx >= len(gnss_list):
        return gnss_list[-1]
    prev = gnss_list[idx - 1]
    next_ = gnss_list[idx]
    t_span = next_.timestamp - prev.timestamp
    if t_span < 1e-9:
        return prev
    alpha = (timestamp - prev.timestamp) / t_span
    return GnssData(
        timestamp=timestamp,
        x=prev.x + alpha * (next_.x - prev.x),
        y=prev.y + alpha * (next_.y - prev.y),
        covariance=prev.covariance + alpha * (next_.covariance - prev.covariance),
        fix_status=prev.fix_status,
        latitude=prev.latitude + alpha * (next_.latitude - prev.latitude),
        longitude=prev.longitude + alpha * (next_.longitude - prev.longitude),
    )
