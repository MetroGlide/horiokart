#!/usr/bin/env python3
"""
generate_static_transforms.py

This script reads `gnss_to_map_base_data.yaml` which contains pairs of pixel coords and lat/lon
for one or more maps/labels, converts pixel->map(m) using `map.yaml` resolution/origin, converts
lat/lon->UTM and estimates a rigid transform (rotation + translation) from UTM->map per label.

The output is a YAML with a `transforms:` list, each entry containing `label`, `map_name`, and `transform: [x,y,yaw]`.
"""
import argparse
import os
import math
import yaml
import numpy as np
import pyproj
from PIL import Image


def estimate_transform_from_correspondences(utm_xy, map_xy):
    # Procrustes-like solution (map = R * utm + t)
    if utm_xy.shape[0] < 2:
        return None
    mu_utm = np.mean(utm_xy, axis=0)
    mu_map = np.mean(map_xy, axis=0)
    X = utm_xy - mu_utm
    Y = map_xy - mu_map
    U, S, Vt = np.linalg.svd(np.dot(Y.T, X))
    R = np.dot(U, Vt)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(U, Vt)
    theta = math.atan2(R[1, 0], R[0, 0])
    t = mu_map - np.dot(R, mu_utm)
    return (float(t[0]), float(t[1]), float(theta))


def read_map_yaml(map_yaml_path):
    with open(map_yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    # Expect keys: resolution, origin (x,y,yaw), image
    resolution_val = data.get('resolution')
    if resolution_val is None:
        raise ValueError(f"Missing required 'resolution' key in map YAML file: {map_yaml_path}")
    resolution = float(resolution_val)
    origin = data.get('origin', [0.0, 0.0, 0.0])
    image_path = data.get('image', None)
    image_height = None
    if image_path:
        if not os.path.isabs(image_path):
            image_path = os.path.join(
                os.path.dirname(map_yaml_path), image_path)
        try:
            img = Image.open(image_path)
            _, image_height = img.size
        except Exception as e:
            raise RuntimeError(f"Failed to open image file '{image_path}' referenced in map YAML '{map_yaml_path}': {e}")
    return resolution, origin, image_height


def pixel_to_map(u, v, resolution, origin_x, origin_y, image_height):
    # Convert pixel coordinates (u,v) to map meters using resolution and origin.
    # Image origin is top-left (pixel y=0 at top), so flip Y using image_height.
    map_x = origin_x + (u * resolution)
    map_y = origin_y + ((image_height - v) * resolution)
    return map_x, map_y


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--base-yaml-name',
                   default='gnss_to_map_base_data.yaml', help='base data yaml')
    p.add_argument('--map-dir', default='/root/ros2_data/map',
                   help='directory to search map yaml files')
    p.add_argument(
        '--out-yaml-name', default='gnss_to_map_static_transforms.yaml', help='output yaml')
    p.add_argument('--utm-zone', type=int, default=54,
                   help='UTM zone for conversion')
    # image height is read from map.yaml image path; no CLI option needed
    args = p.parse_args()

    with open(os.path.join(args.map_dir, args.base_yaml_name), 'r') as f:
        base = yaml.safe_load(f)

    if not base:
        print('base yaml empty')
        return

    base_data = base.get('base_data', [])
    proj = pyproj.Proj(proj='utm', zone=args.utm_zone,
                       ellps='WGS84', south=False)

    out = {'transforms': []}

    for rec in base_data:
        map_name = rec.get('map_name')
        label = rec.get('label')
        points = rec.get('points', [])
        if not map_name or not label or len(points) < 2:
            print(
                f"Skipping label {label} for map {map_name}: need >=2 points")
            continue

        # locate map.yaml
        map_yaml_path = os.path.join(args.map_dir, map_name)
        if not os.path.isfile(map_yaml_path):
            print(f"map yaml not found: {map_yaml_path}")
            continue

        resolution, origin, image_height = read_map_yaml(map_yaml_path)
        origin_x, origin_y = float(origin[0]), float(origin[1])
        if image_height is None:
            print(
                f"map.yaml for {map_name} does not contain an 'image' entry or image not found; cannot determine image height. Skipping.")
            continue

        utm_xy = []
        map_xy = []
        for pnt in points:
            pixel = pnt.get('pixel')
            latlon = pnt.get('latlon')
            if pixel is None or latlon is None:
                continue
            u, v = float(pixel[0]), float(pixel[1])
            lat, lon = float(latlon[0]), float(latlon[1])
            # note: proj expects lon, lat order
            x, y = proj(lon, lat)
            mx, my = pixel_to_map(
                u, v, resolution, origin_x, origin_y, image_height)
            utm_xy.append([x, y])
            map_xy.append([mx, my])

        utm_arr = np.array(utm_xy)
        map_arr = np.array(map_xy)
        est = estimate_transform_from_correspondences(utm_arr, map_arr)
        if est is None:
            print(f"Failed to estimate for label {label}")
            continue

        out['transforms'].append(
            {'label': label, 'map_name': map_name, 'transform': [est[0], est[1], est[2]]})
        print(
            f"Estimated {label}: x={est[0]:.3f}, y={est[1]:.3f}, yaw={est[2]:.3f}")

    out_path = os.path.join(args.map_dir, args.out_yaml_name)
    with open(out_path, 'w') as f:
        yaml.safe_dump(out, f)
    print(f"Wrote {out_path}")


if __name__ == '__main__':
    main()
