from horiokart_slam.tools import gnss_kalman


def make_records_simple():
    # three records at t=0,1,2 with x offset 0..2
    return [
        {'time': 0.0, 'x': 0.0, 'y': 0.0, 'accuracy': 1.0},
        {'time': 1.0, 'x': 1.0, 'y': 0.0, 'accuracy': 1.0},
        {'time': 2.0, 'x': 2.0, 'y': 0.0, 'accuracy': 1.0},
    ]


def test_kalman_at_exact_sample():
    recs = make_records_simple()
    x, y, cov = gnss_kalman.kalman_estimate_at_time(recs, 1.0)
    assert abs(x - 1.0) < 1e-6
    assert abs(y - 0.0) < 1e-6
    assert cov.shape == (2, 2)


def test_kalman_interpolation_and_extrapolation():
    # midpoint
    recs = make_records_simple()
    x, y, cov = gnss_kalman.kalman_estimate_at_time(recs, 0.5)
    assert abs(x - 0.5) < 0.5
    # extrapolate forward
    x2, y2, cov2 = gnss_kalman.kalman_estimate_at_time(recs, 3.0)
    # with only 3 points, a simple CVKF may not reach exact 3.0, but should be >2.0
    assert x2 >= 2.0
