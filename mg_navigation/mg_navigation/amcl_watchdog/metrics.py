"""Metric computations for AMCL covariance monitoring."""
from typing import Sequence
import math


def compute_trace_xy(cov: Sequence[float]) -> float:
    """Return trace over x and y (var_x + var_y)."""
    if cov is None or len(cov) < 36:
        raise ValueError("covariance must be a 36-length sequence")
    var_x = cov[0]   # cov[0*6+0]
    var_y = cov[7]   # cov[1*6+1]
    return float(var_x + var_y)


def compute_determinant_xy(cov: Sequence[float]) -> float:
    """Return determinant of the 2x2 xy covariance matrix."""
    if cov is None or len(cov) < 36:
        raise ValueError("covariance must be a 36-length sequence")
    a = cov[0]   # cov[0*6+0]
    b = cov[1]   # cov[0*6+1]
    c = cov[6]   # cov[1*6+0]
    d = cov[7]   # cov[1*6+1]
    # determinant of [[a,b],[c,d]]
    return float(a * d - b * c)


def compute_max_eigenvalue_xy(cov: Sequence[float]) -> float:
    """Compute the maximum eigenvalue of the 2x2 xy covariance matrix."""
    if cov is None or len(cov) < 36:
        raise ValueError("covariance must be a 36-length sequence")
    a = cov[0]
    b = cov[1]
    c = cov[6]
    d = cov[7]
    # eigenvalues of 2x2 matrix: lambda = (trace +/- sqrt(trace^2 - 4 det))/2
    tr = a + d
    det = a * d - b * c
    disc = tr * tr - 4.0 * det
    if disc < 0:
        # numerical guard: clamp
        disc = 0.0
    sqrt_disc = math.sqrt(disc)
    lambda1 = 0.5 * (tr + sqrt_disc)
    lambda2 = 0.5 * (tr - sqrt_disc)
    return float(max(lambda1, lambda2))


def compute(metric: str, cov: Sequence[float]) -> float:
    m = metric.lower()
    if m == "trace_xy":
        return compute_trace_xy(cov)
    if m == "determinant_xy":
        return compute_determinant_xy(cov)
    if m == "max_eigenvalue_xy":
        return compute_max_eigenvalue_xy(cov)
    raise ValueError(f"unknown metric: {metric}")
