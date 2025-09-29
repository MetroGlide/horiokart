#!/usr/bin/env python3
"""
Simple GNSS extractor for ROS2 bag files.
Modes:
 - listen: subscribe to /fix while playing a bag via `ros2 bag play` externally
 - play-and-capture: spawn `ros2 bag play <bag>` and subscribe locally to capture messages

Outputs JSON: {"records": [{"time": float_seconds, "lat": ..., "lon": ..., "alt": ..., "status": int, "covariance": [9 floats] or null }, ...]}

This is intentionally minimal and uses rclpy for subscription when available.
"""

import argparse
import json
import sys
import time
from typing import List, Dict, Any

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix
    RCLPY_AVAILABLE = True
except Exception:
    RCLPY_AVAILABLE = False


class GNSSCollector(Node):
    def __init__(self, topic: str = '/fix'):
        super().__init__('gnss_extractor_tmp')
        self.topic = topic
        self.records: List[Dict[str, Any]] = []
        self.sub = self.create_subscription(NavSatFix, self.topic, self.cb, 10)

    def cb(self, msg: NavSatFix):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rec = {
            'time': float(t),
            'lat': float(msg.latitude),
            'lon': float(msg.longitude),
            'alt': float(msg.altitude),
            'status': int(msg.status.status) if hasattr(msg, 'status') else 0,
            'covariance': list(msg.position_covariance) if hasattr(msg, 'position_covariance') else None,
        }
        self.records.append(rec)


def capture_via_rclpy(topic: str, duration: float) -> List[Dict[str, Any]]:
    if not RCLPY_AVAILABLE:
        raise RuntimeError('rclpy not available in this environment')
    rclpy.init()
    node = GNSSCollector(topic)
    try:
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return node.records


def write_json(out_path: str, records: List[Dict[str, Any]]):
    with open(out_path, 'w') as f:
        json.dump({'records': records}, f, indent=2)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Extract NavSatFix from ros2 bag by subscribing while playing')
    parser.add_argument(
        'bag', nargs='?', help='Path to ros2 bag to play (optional)')
    parser.add_argument('--topic', default='/fix', help='NavSatFix topic')
    parser.add_argument('--out', default='gnss.json', help='Output JSON file')
    parser.add_argument('--duration', type=float, default=30.0,
                        help='How long to listen (seconds)')
    parser.add_argument('--mode', choices=['rclpy-listen', 'play-capture'], default='rclpy-listen',
                        help='Capture mode')
    args = parser.parse_args(argv)

    if args.mode == 'rclpy-listen':
        if not RCLPY_AVAILABLE:
            print(
                'rclpy not available; cannot listen. Try mode=play-capture', file=sys.stderr)
            return 2
        print(f'Listening for {args.duration}s on topic {args.topic}...')
        recs = capture_via_rclpy(args.topic, args.duration)
        write_json(args.out, recs)
        print(f'Wrote {len(recs)} records to {args.out}')
        return 0

    # play-capture: spawn ros2 bag play and capture for duration
    import subprocess
    import shlex

    if args.bag is None:
        print('bag path required for play-capture mode', file=sys.stderr)
        return 2

    if not RCLPY_AVAILABLE:
        print('rclpy not available; play-capture mode requires rclpy to subscribe', file=sys.stderr)
        return 2

    # Start ros2 bag play
    cmd = f'ros2 bag play {shlex.quote(args.bag)}'
    print('Starting:', cmd)
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        recs = capture_via_rclpy(args.topic, args.duration)
        write_json(args.out, recs)
        print(f'Wrote {len(recs)} records to {args.out}')
    finally:
        try:
            p.terminate()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
