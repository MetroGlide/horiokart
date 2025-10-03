"""Simple Constant-Velocity Kalman filter helper for GNSS records.

Provides a small, dependency-free CVKF used by gnss matching when
`--interp-method=kalman` is requested. The API mirrors the internal
_kalman_estimate_at_time used previously in gnss_match.
"""
from typing import List, Dict, Any, Tuple
import numpy as np


def kalman_estimate_at_time(records: List[Dict[str, Any]], t_query: float) -> Tuple[float, float, np.ndarray]:
    """Estimate x,y and 2x2 covariance at t_query using a simple CV Kalman filter.

    records: list of {'time': float, 'x': float, 'y': float, optional 'accuracy'}
    returns: x, y, cov (2x2 numpy array)
    """
    if len(records) == 0:
        raise RuntimeError('no records')

    # sort records by time
    rec = sorted(records, key=lambda r: r['time'])
    times = [r['time'] for r in rec]
    xs = [r['x'] for r in rec]
    ys = [r['y'] for r in rec]

    # If t_query matches a sample exactly, return that measurement and its cov/accuracy
    for r in rec:
        if abs(r['time'] - t_query) < 1e-9:
            # build covariance
            if 'cov' in r and r['cov'] is not None:
                cov = np.array(r['cov'], dtype=float)
                try:
                    cov2 = cov.reshape((2, 2))
                except Exception:
                    cov2 = np.eye(2) * float(r.get('accuracy', 1.0)) ** 2
            else:
                cov2 = np.eye(2) * float(r.get('accuracy', 1.0)) ** 2
            return float(r['x']), float(r['y']), cov2

    # If only two records, do direct linear interpolation/extrapolation (stable, deterministic)
    if len(rec) == 1:
        r = rec[0]
        cov2 = np.eye(2) * float(r.get('accuracy', 1.0)) ** 2
        return float(r['x']), float(r['y']), cov2
    if len(rec) == 2:
        t0 = times[0]
        t1 = times[1]
        x0 = xs[0]
        x1 = xs[1]
        y0 = ys[0]
        y1 = ys[1]
        if t1 == t0:
            alpha = 0.0
        else:
            alpha = (t_query - t0) / (t1 - t0)
        xq = (1 - alpha) * x0 + alpha * x1
        yq = (1 - alpha) * y0 + alpha * y1
        # simple cov: interpolate accuracies if present
        a0 = float(rec[0].get('accuracy', 1.0))
        a1 = float(rec[1].get('accuracy', 1.0))
        a = max(min((1 - alpha) * a0 + alpha * a1, max(a0, a1)), min(a0, a1))
        cov2 = np.eye(2) * (a ** 2)
        return float(xq), float(yq), cov2

    # Process/measurement noise heuristics
    q_pos = 0.1
    q_vel = 1e-2
    R_base = 1.0

    # state: [x, y, vx, vy]
    x = np.array([xs[0], ys[0], 0.0, 0.0], dtype=float)
    P = np.eye(4) * 1.0

    def F(dt: float) -> np.ndarray:
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
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        yv = z - (H @ x)
        x = x + K @ yv
        P = (np.eye(4) - K @ H) @ P

        # If query time falls between t0 and t1, interpolate predict to query
        if t_query >= t0 and t_query <= t1:
            dtq = t_query - t1
            if abs(dtq) < 1e-9:
                return float(x[0]), float(x[1]), P[:2, :2]
            Fq = F(dtq)
            xq = Fq @ x
            Pq = Fq @ P @ Fq.T + np.diag([q_pos * abs(dtq), q_pos * abs(dtq), q_vel * abs(dtq), q_vel * abs(dtq)])
            return float(xq[0]), float(xq[1]), Pq[:2, :2]

    # If query after last measurement, predict forward
    dt_last = t_query - times[-1]
    if dt_last < 0:
        return float(xs[0]), float(ys[0]), np.eye(2) * R_base
    Fq = F(dt_last)
    xq = Fq @ x
    Pq = Fq @ P @ Fq.T + np.diag([q_pos * abs(dt_last), q_pos * abs(dt_last), q_vel * abs(dt_last), q_vel * abs(dt_last)])
    return float(xq[0]), float(xq[1]), Pq[:2, :2]


__all__ = ['kalman_estimate_at_time', 'kalman_estimate']


def kalman_estimate(records: List[Dict[str, Any]], t_query: float) -> Tuple[float, float, List[List[float]]]:
    """Wrapper that returns cov as nested lists for compatibility with tests.

    Internally delegates to kalman_estimate_at_time which returns a numpy array.
    """
    x, y, cov = kalman_estimate_at_time(records, t_query)
    cov_list = cov.tolist() if hasattr(cov, 'tolist') else [[float(cov[0, 0]), float(cov[0, 1])], [float(cov[1, 0]), float(cov[1, 1])]]
    return x, y, cov_list

