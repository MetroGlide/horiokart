import json
import os
import tempfile
import importlib.util


def import_gnss_extract_module():
    # Explicit workspace path to the gnss_extract script
    path = '/root/ros2_ws/src/horiokart/horiokart_slam/tools/gnss_extract.py'
    path = os.path.normpath(path)
    spec = importlib.util.spec_from_file_location('gnss_extract', path)
    if spec is None or spec.loader is None:
        raise ImportError(f'Cannot load gnss_extract from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore[attr-defined]
    return module

ge = import_gnss_extract_module()


def test_write_json_tmpfile():
    records = [
        {'time': 123.456, 'lat': 35.0, 'lon': 139.0, 'alt': 10.0, 'status': 0, 'covariance': None}
    ]
    fd, path = tempfile.mkstemp(suffix='.json')
    os.close(fd)
    try:
        ge.write_json(path, records)
        with open(path, 'r') as f:
            data = json.load(f)
        assert 'records' in data
        assert isinstance(data['records'], list)
        assert data['records'][0]['lat'] == 35.0
    finally:
        os.remove(path)
