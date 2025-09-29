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
try:
    import numpy.linalg as la
except Exception:
    la = None


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


def _kalman_estimate_at_time(records: List[Dict[str, Any]], t_query: float):
    """Run a simple Constant-Velocity Kalman Filter over GNSS records (x,y) and return state at t_query.

    Returns (x,y,cov2x2) or raises on failure.
    """
    if len(records) == 0:
        raise RuntimeError('no records')
    # Build state vector [x, y, vx, vy]
    # Initialize with first two samples if available
    times = [r['time'] for r in records]
    xs = [r['x'] for r in records]
    ys = [r['y'] for r in records]
    # basic process noise and measurement noise heuristics
    q_pos = 0.1
    q_vel = 1e-2
    R_base = 1.0

    # state and cov init
    x = np.array([xs[0], ys[0], 0.0, 0.0], dtype=float)
    P = np.eye(4) * 1.0

    def F(dt):
        M = np.eye(4)
        M[0, 2] = dt
        M[1, 3] = dt
        return M

    H = np.zeros((2, 4))
    H[0, 0] = 1.0
    H[1, 1] = 1.0

    for i in range(1, len(records)):
        t0 = times[i - 1]
        t1 = times[i]
        dt = t1 - t0
        if dt <= 0:
            continue
        # Predict
        Fm = F(dt)
        Q = np.diag([q_pos * dt, q_pos * dt, q_vel * dt, q_vel * dt])
        x = Fm @ x
        P = Fm @ P @ Fm.T + Q
        # Measurement
        z = np.array([xs[i], ys[i]], dtype=float)
        R = np.eye(2) * (records[i].get('accuracy', R_base) ** 2)
        # Update
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        yv = z - (H @ x)
        x = x + K @ yv
        P = (np.eye(4) - K @ H) @ P
        # If query time falls between t0 and t1, interpolate predict to query
        if t_query >= t0 and t_query <= t1:
            dtq = t_query - t1
            if abs(dtq) < 1e-9:
                # return current x
                return float(x[0]), float(x[1]), P[:2, :2]
            # predict forward dtq from current last state
            Fq = F(dtq)
            xq = Fq @ x
            Pq = Fq @ P @ Fq.T + \
                np.diag([q_pos * abs(dtq), q_pos * abs(dtq),
                        q_vel * abs(dtq), q_vel * abs(dtq)])
            return float(xq[0]), float(xq[1]), Pq[:2, :2]

    # If query after last measurement, predict forward
    dt_last = t_query - times[-1]
    if dt_last < 0:
        # query before first sample: return first
        return float(xs[0]), float(ys[0]), np.eye(2) * R_base
    Fq = F(dt_last)
    xq = Fq @ x
    Pq = Fq @ P @ Fq.T + np.diag([q_pos * abs(dt_last), q_pos *
                                 abs(dt_last), q_vel * abs(dt_last), q_vel * abs(dt_last)])
    return float(xq[0]), float(xq[1]), Pq[:2, :2]


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
        # apply node-level time offset if present in gnss metadata (handled externally)
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

    # surrounding -> interpolation: support spline, kalman, or linear interpolation
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
            # compute weights and cov as in linear case
            s = find_surrounding(t)
            if not s:
                continue
            t0, t1 = s[0]['time'], s[1]['time']
            if t1 == t0:
                continue
            alpha = (t - t0) / (t1 - t0)
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
                    t0v = cov0[0] + cov0[3]
                    t1v = cov1[0] + cov1[3]
                    v = ((1 - alpha) * t0v + alpha * t1v)
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
                cov = None
                cov0 = s[0].get('cov')
                cov1 = s[1].get('cov')
                if cov0 and cov1 and len(cov0) >= 4 and len(cov1) >= 4:
                    try:
                        c0 = np.array(cov0, dtype=float)
                        c1 = np.array(cov1, dtype=float)
                        c0m = c0.reshape((-1,))[:4].reshape((2, 2))
                        c1m = c1.reshape((-1,))[:4].reshape((2, 2))
                        cov = (1 - alpha) * c0m + alpha * c1m
                        cov = cov.tolist()
                    except Exception:
                        cov = None
                constraints.append({'node_idx': idx, 'x': x, 'y': y,
                                   'weight': weight, 'source_time': t, 'dt': dt, 'cov': cov})

        elif interp_method == 'kalman':
            try:
                est_x, est_y, est_cov = _kalman_estimate_at_time(
                    records_sorted, t)
            except Exception:
                continue
            weight = 1.0
            cov = None
            if est_cov is not None:
                cov = est_cov.tolist()
                tr = float(np.trace(est_cov))
                if tr > 0:
                    weight = 1.0 / tr
            dt = 0.0
            constraints.append({'node_idx': idx, 'x': float(est_x), 'y': float(
                est_y), 'weight': weight, 'source_time': t, 'dt': dt, 'cov': cov})

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
                if weight_mode == 'var_interp':
                    a0 = acc0 if acc0 is not None else 5.0
                    a1 = acc1 if acc1 is not None else 5.0
                    var = ((1 - alpha) * a0) ** 2 + (alpha * a1) ** 2
                    weight = 1.0 / var if var > 0 else 1.0
                elif weight_mode == 'cov_trace':
                    cov0 = s[0].get('cov')
                    cov1 = s[1].get('cov')
                    if cov0 and cov1 and len(cov0) >= 4 and len(cov1) >= 4:
                        t0v = cov0[0] + cov0[3]
                        t1v = cov1[0] + cov1[3]
                        v = ((1 - alpha) * t0v + alpha * t1v)
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
                    cov = None
                    cov0 = s[0].get('cov')
                    cov1 = s[1].get('cov')
                    if cov0 and cov1 and len(cov0) >= 4 and len(cov1) >= 4:
                        try:
                            c0 = np.array(cov0, dtype=float)
                            c1 = np.array(cov1, dtype=float)
                            c0m = c0.reshape((-1,))[:4].reshape((2, 2))
                            c1m = c1.reshape((-1,))[:4].reshape((2, 2))
                            cov = (1 - alpha) * c0m + alpha * c1m
                            cov = cov.tolist()
                        except Exception:
                            cov = None
                    constraints.append(
                        {'node_idx': idx, 'x': x, 'y': y, 'weight': weight, 'source_time': t, 'dt': dt, 'cov': cov})

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
    parser.add_argument(
        '--interp-method', choices=['linear', 'spline', 'kalman'], default='linear')
    parser.add_argument('--mah-threshold', type=float, default=None)
    parser.add_argument('--time-offset', type=float, default=0.0,
                        help='apply a fixed time offset (seconds) to posegraph timestamps before matching')
    parser.add_argument('--auto-time-offset', action='store_true',
                        help='auto-estimate time offset by grid search (range +-5s, step 0.1s)')
    args = parser.parse_args()
    with open(args.posegraph) as f:
        pg = json.load(f)
    with open(args.gnss) as f:
        gnss = json.load(f)
    # apply fixed time offset to posegraph nodes if requested
    if abs(args.time_offset) > 0.0:
        for n in pg.get('nodes', []):
            if 'timestamp' in n:
                n['timestamp'] = n['timestamp'] + args.time_offset
    # auto-time-offset: grid search (coarse)
    if args.auto_time_offset:
        best_offset = 0.0
        best_matches = -1
        for off in np.arange(-5.0, 5.0 + 1e-9, 0.1):
            # copy posegraph and shift timestamps
            pg_copy = json.loads(json.dumps(pg))
            for n in pg_copy.get('nodes', []):
                if 'timestamp' in n:
                    n['timestamp'] = n['timestamp'] + off
            cons = match_posegraph_with_gnss(pg_copy, gnss, window=args.window, mad_threshold=args.mad_threshold, use_y=args.use_y,
                                             weight_mode=args.weight_mode, interp_method=args.interp_method, mah_threshold=args.mah_threshold)
            matches = len(cons.get('constraints', []))
            if matches > best_matches:
                best_matches = matches
                best_offset = off
        # apply best_offset
        for n in pg.get('nodes', []):
            if 'timestamp' in n:
                n['timestamp'] = n['timestamp'] + best_offset
        print(
            f'auto-time-offset selected: {best_offset} s with {best_matches} matches')
    cons = match_posegraph_with_gnss(
        pg, gnss, window=args.window, mad_threshold=args.mad_threshold, use_y=args.use_y, weight_mode=args.weight_mode, interp_method=args.interp_method, mah_threshold=args.mah_threshold)
    with open(args.out, 'w') as f:
        json.dump(cons, f, indent=2)


if __name__ == '__main__':
    cli()
