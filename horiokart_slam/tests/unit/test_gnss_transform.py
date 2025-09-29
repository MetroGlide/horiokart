from importlib import import_module
import os
import math

# import by path to workspace module
from importlib import util
mod_path = '/root/ros2_ws/src/horiokart/horiokart_slam/tools/gnss_transform.py'
spec = util.spec_from_file_location('gnss_transform', mod_path)
if spec is None or spec.loader is None:
    raise ImportError(f'Cannot load module from {mod_path}')
module = util.module_from_spec(spec)
spec.loader.exec_module(module)  # type: ignore[attr-defined]


def test_to_utm_basic():
    # Tokyo approx
    lat, lon = 35.6895, 139.6917
    x, y, zone = module.to_utm(lat, lon)
    # Basic checks: x,y finite and zone contains 'N' for northern hemisphere
    assert isinstance(x, float)
    assert isinstance(y, float)
    assert 'N' in zone
    # Rough distance from origin (lat=0 lon=0) should be > 0
    assert math.hypot(x, y) > 1000


def test_to_utm_southern():
    lat, lon = -33.86, 151.2093  # Sydney
    x, y, zone = module.to_utm(lat, lon)
    assert 'S' in zone
    assert isinstance(x, float) and isinstance(y, float)
