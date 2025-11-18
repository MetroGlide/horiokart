"""Metric computations for AMCL covariance monitoring."""
from typing import Sequence
import math


def _cov_index(i: int, j: int) -> int:
    return i * 6 + j


def compute_trace_xy(cov: Sequence[float]) -> float:
    """Return trace over x and y (var_x + var_y)."""
    if cov is None or len(cov) < 36:
        raise ValueError("covariance must be a 36-length sequence")
    var_x = cov[_cov_index(0, 0)]
    var_y = cov[_cov_index(1, 1)]
    return float(var_x + var_y)


def compute_determinant_xy(cov: Sequence[float]) -> float:
    """Return determinant of the 2x2 xy covariance matrix."""
    if cov is None or len(cov) < 36:
        raise ValueError("covariance must be a 36-length sequence")
    a = cov[_cov_index(0, 0)]
    b = cov[_cov_index(0, 1)]
    c = cov[_cov_index(1, 0)]
    d = cov[_cov_index(1, 1)]
    # determinant of [[a,b],[c,d]]
    return float(a * d - b * c)


def compute_max_eigenvalue_xy(cov: Sequence[float]) -> float:
    """Compute the maximum eigenvalue of the 2x2 xy covariance matrix."""
    if cov is None or len(cov) < 36:
        raise ValueError("covariance must be a 36-length sequence")
    a = cov[_cov_index(0, 0)]
    b = cov[_cov_index(0, 1)]
    c = cov[_cov_index(1, 0)]
    d = cov[_cov_index(1, 1)]
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
