#!/usr/bin/env python3
"""
GNSS <-> PoseGraph time matching and constraint generation.

Inputs:
 - posegraph_json: {nodes:[{id, pose:[x,y,theta], timestamp}], ...}
 - gnss_json: {records:[{time, x, y, accuracy?}]}

Outputs:
 - constraints: {constraints:[{node_idx, x, y, weight, source_time, dt}]}

This module provides functions usable from CLI and unit tests.
"""
from typing import List, Dict, Any, Optional
import json
import math
import numpy as np
try:
    from scipy.interpolate import UnivariateSpline
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


def _median(lst: List[float]) -> float:
    s = sorted(lst)
    n = len(s)
    if n == 0:
        return 0.0
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


def _mad_filter(points: List[Dict[str, Any]], key: str = 'x', threshold: float = 3.5) -> List[Dict[str, Any]]:
    # Median Absolute Deviation outlier rejection on key
    vals = [p[key] for p in points]
    med = _median(vals)
    devs = [abs(v - med) for v in vals]
    mad = _median(devs)
    if mad == 0:
        return points
    filtered = []
    for p, d in zip(points, devs):
        if abs(d / mad) <= threshold:
            filtered.append(p)
    return filtered


def _mahalanobis_filter(points: List[Dict[str, Any]], threshold: float = 5.0) -> List[Dict[str, Any]]:
    # points: list of {'x','y'}
    if len(points) == 0:
        return points
    arr = np.array([[p['x'], p['y']] for p in points])
    cov = np.cov(arr.T)
    # regularize
    if cov.shape == (2, 2):
        try:
            inv = np.linalg.inv(cov)
        except np.linalg.LinAlgError:
            inv = np.linalg.pinv(cov + np.eye(2) * 1e-6)
    else:
        inv = np.linalg.pinv(np.cov(arr.T) + np.eye(2) * 1e-6)
    mean = np.mean(arr, axis=0)
    filtered = []
    for p, v in zip(points, arr):
        d2 = float((v - mean).T.dot(inv).dot(v - mean))
        if d2 <= threshold ** 2:
            filtered.append(p)
    return filtered


def match_posegraph_with_gnss(posegraph: Dict[str, Any], gnss: Dict[str, Any], window: float = 2.0, mad_threshold: float = 3.5, use_y: bool = True, weight_mode: str = 'var_interp', interp_method: str = 'linear', mah_threshold: Optional[float] = None) -> Dict[str, Any]:
    """Associate GNSS samples to posegraph nodes via time-window and linear interpolation.

    Returns constraints JSON.
    """
    nodes = posegraph.get('nodes', [])
    records = gnss.get('records', [])

    # build time-index for GNSS (assume records have 'time' in seconds)
    records_sorted = sorted(records, key=lambda r: r['time'])

    # Helper: find surrounding samples for time t
    def find_surrounding(t: float) -> Optional[List[Dict[str, Any]]]:
        # find i such that records[i].time <= t <= records[i+1].time
        for i in range(len(records_sorted) - 1):
            t0 = records_sorted[i]['time']
            t1 = records_sorted[i + 1]['time']
            if t0 <= t <= t1:
                return [records_sorted[i], records_sorted[i + 1]]
        return None

    # Precompute spline if requested
    spline_x = None
    spline_y = None
    if interp_method == 'spline' and _HAS_SCIPY and len(records_sorted) >= 3:
        times = np.array([r['time'] for r in records_sorted])
        xs = np.array([r['x'] for r in records_sorted])
        ys = np.array([r['y'] for r in records_sorted])
        try:
            spline_x = UnivariateSpline(times, xs, s=0)
            spline_y = UnivariateSpline(times, ys, s=0)
        except Exception:
            spline_x = None
            spline_y = None

    constraints = []
    for idx, node in enumerate(nodes):
        t = node.get('timestamp')
        if t is None:
            continue
        # find exact or surrounding
        exact = [r for r in records_sorted if abs(r['time'] - t) < 1e-6]
        if exact:
            r = exact[0]
            dt = abs(r['time'] - t)
            weight = 1.0 / (r.get('accuracy', 5.0) **
                            2) if r.get('accuracy') else 1.0
            constraints.append(
                {'node_idx': idx, 'x': r['x'], 'y': r['y'], 'weight': weight, 'source_time': r['time'], 'dt': dt})
            continue

        # surrounding -> interpolation
        if interp_method == 'spline' and spline_x is not None and spline_y is not None:
            try:
                x = float(spline_x(t))
                y = float(spline_y(t))
            except Exception:
                s = find_surrounding(t)
                if not s:
                    continue
                t0, t1 = s[0]['time'], s[1]['time']
                if t1 == t0:
                    continue
                alpha = (t - t0) / (t1 - t0)
                x = (1 - alpha) * s[0]['x'] + alpha * s[1]['x']
                y = (1 - alpha) * s[0]['y'] + alpha * s[1]['y']
        else:
            s = find_surrounding(t)
            if s:
                t0, t1 = s[0]['time'], s[1]['time']
                if t1 == t0:
                    continue
                alpha = (t - t0) / (t1 - t0)
                x = (1 - alpha) * s[0]['x'] + alpha * s[1]['x']
                y = (1 - alpha) * s[0]['y'] + alpha * s[1]['y']
            acc0 = s[0].get('accuracy', None)
            acc1 = s[1].get('accuracy', None)
            # choose weight mode
            if weight_mode == 'var_interp':
                a0 = acc0 if acc0 is not None else 5.0
                a1 = acc1 if acc1 is not None else 5.0
                var = ((1 - alpha) * a0) ** 2 + (alpha * a1) ** 2
                weight = 1.0 / var if var > 0 else 1.0
            elif weight_mode == 'cov_trace':
                cov0 = s[0].get('cov')
                cov1 = s[1].get('cov')
                if cov0 and cov1 and len(cov0) >= 4 and len(cov1) >= 4:
                    t0 = cov0[0] + cov0[3]
                    t1 = cov1[0] + cov1[3]
                    v = ((1 - alpha) * t0 + alpha * t1)
                    weight = 1.0 / v if v > 0 else 1.0
                else:
                    a0 = acc0 if acc0 is not None else 5.0
                    a1 = acc1 if acc1 is not None else 5.0
                    var = ((1 - alpha) * a0) ** 2 + (alpha * a1) ** 2
                    weight = 1.0 / var if var > 0 else 1.0
            elif weight_mode == 'hdop':
                hd0 = s[0].get('hdop', 5.0)
                hd1 = s[1].get('hdop', 5.0)
                val = (1 - alpha) * hd0 + alpha * hd1
                weight = 1.0 / (val ** 2) if val > 0 else 1.0
            else:
                weight = 1.0
            dt = min(abs(t - t0), abs(t - t1))
            if dt <= window:
                constraints.append(
                    {'node_idx': idx, 'x': x, 'y': y, 'weight': weight, 'source_time': t, 'dt': dt})

    # Outlier rejection: apply MAD and optionally Mahalanobis on (x,y)
    if constraints:
        if use_y:
            # filter by both axes using MAD
            filtered_x = _mad_filter(
                [c for c in constraints], 'x', threshold=mad_threshold)
            allowed_x = set([f['x'] for f in filtered_x])
            filtered_y = _mad_filter(
                [c for c in constraints], 'y', threshold=mad_threshold)
            allowed_y = set([f['y'] for f in filtered_y])
            filtered = [c for c in constraints if (
                c['x'] in allowed_x and c['y'] in allowed_y)]
        else:
            filtered_x = _mad_filter(
                [c for c in constraints], 'x', threshold=mad_threshold)
            allowed_x = set([f['x'] for f in filtered_x])
            filtered = [c for c in constraints if c['x'] in allowed_x]
        constraints = filtered
        # Mahalanobis-based filtering across remaining constraints if requested
        if mah_threshold is not None and len(constraints) >= 3:
            constraints = _mahalanobis_filter(
                constraints, threshold=mah_threshold)

    return {'constraints': constraints}


def cli():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--posegraph', required=True)
    parser.add_argument('--gnss', required=True)
    parser.add_argument('--out', default='constraints.json')
    parser.add_argument('--window', type=float, default=2.0)
    parser.add_argument('--mad-threshold', type=float, default=3.5)
    parser.add_argument('--no-y-filter', dest='use_y', action='store_false')
    parser.add_argument(
        '--weight-mode', choices=['fixed', 'var_interp', 'cov_trace', 'hdop'], default='var_interp')
    args = parser.parse_args()
    with open(args.posegraph) as f:
        pg = json.load(f)
    with open(args.gnss) as f:
        gnss = json.load(f)
    cons = match_posegraph_with_gnss(
        pg, gnss, window=args.window, mad_threshold=args.mad_threshold, use_y=args.use_y, weight_mode=args.weight_mode)
    with open(args.out, 'w') as f:
        json.dump(cons, f, indent=2)


if __name__ == '__main__':
    cli()
