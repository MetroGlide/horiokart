#!/usr/bin/env python3
"""
GNSS transform utilities: lat/lon -> UTM (x,y)
This implementation REQUIRES pyproj to be installed.
API:
  - to_utm(lat, lon) -> (x, y, zone_str)
"""
from typing import Tuple

try:
    from pyproj import Transformer
except Exception as e:
    raise ImportError(
        'pyproj is required for gnss_transform. Install with: pip install pyproj') from e


def _utm_zone_from_lon(lon: float) -> int:
    return int((lon + 180) / 6) + 1


def to_utm(lat: float, lon: float) -> Tuple[float, float, str]:
    """Convert lat, lon (deg) to UTM x, y in meters and zone string like '32N'."""
    zone = _utm_zone_from_lon(lon)
    epsg = 32600 + zone if lat >= 0 else 32700 + zone
    transformer = Transformer.from_crs(
        'EPSG:4326', f'EPSG:{epsg}', always_xy=True)
    x, y = transformer.transform(lon, lat)
    zone_str = f"{zone}{'N' if lat >= 0 else 'S'}"
    return float(x), float(y), zone_str


def main_cli():
    import argparse
    parser = argparse.ArgumentParser(description='Convert lat/lon to UTM x,y')
    parser.add_argument('lat', type=float)
    parser.add_argument('lon', type=float)
    args = parser.parse_args()
    x, y, zone = to_utm(args.lat, args.lon)
    print(f'{x} {y} {zone}')


if __name__ == '__main__':
    main_cli()
