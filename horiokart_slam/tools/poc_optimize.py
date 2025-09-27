"""Simple Python PoC optimizer: read posegraph JSON and GNSS fixtures, convert GNSS to UTM, associate by nearest timestamp, and do a lightweight least-squares position shift for nodes with GNSS.
This is intentionally minimal to validate data flows.
"""
import json
import math
import os
from datetime import datetime

import numpy as np
from pyproj import Proj, transform
from scipy.optimize import least_squares

ROOT = os.path.dirname(os.path.dirname(__file__))
FIXTURES = os.path.join(ROOT, '..', 'specs',
                        '001-slam-toolbox-gnss', 'tests', 'fixtures')
POSEGRAPH = os.path.join(FIXTURES, 'posegraph_small.json')
GNSS = os.path.join(FIXTURES, 'gnss_small.json')


def load_json(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def latlon_to_utm(lat, lon):
    # Use pyproj to project to UTM zone automatically
    proj_utm = Proj(proj='utm', zone=int((lon + 180) / 6) + 1, ellps='WGS84')
    e, n = proj_utm(lon, lat)
    return e, n


def main():
    pg = load_json(POSEGRAPH)
    gnss = load_json(GNSS)

    # Build node index
    nodes = pg['nodes']
    id_to_idx = {n['id']: i for i, n in enumerate(nodes)}

    # Convert GNSS to UTM
    for g in gnss:
        g['easting'], g['northing'] = latlon_to_utm(g['lat'], g['lon'])

    # Simple association: assign GNSS to nearest node by time (ISO strings)
    def parse_t(s):
        return datetime.fromisoformat(s.replace('Z', '+00:00'))

    node_times = [parse_t(n['t']) for n in nodes]

    associations = {}
    for g in gnss:
        gt = parse_t(g['timestamp'])
        diffs = [abs((gt - nt).total_seconds()) for nt in node_times]
        idx = int(np.argmin(diffs))
        associations[idx] = g

    # Simple residual: for nodes with GNSS, shift node x,y to GNSS easting/northing by additive XY
    x0 = []
    bounds = ([], [])
    for n in nodes:
        x0.append(n['pose'][0])
        x0.append(n['pose'][1])
    x0 = np.array(x0)

    def residuals(x):
        res = []
        for i, n in enumerate(nodes):
            xi = x[2*i]
            yi = x[2*i+1]
            if i in associations:
                g = associations[i]
                res.append(xi - g['easting'])
                res.append(yi - g['northing'])
        return np.array(res)

    if len(associations) == 0:
        print('No associations; nothing to optimize')
        return

    sol = least_squares(residuals, x0)
    xopt = sol.x

    # write optimized posegraph JSON
    for i, n in enumerate(nodes):
        n['pose'][0] = float(xopt[2*i])
        n['pose'][1] = float(xopt[2*i+1])

    out_path = os.path.join('/tmp', 'posegraph_optimized.json')
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(pg, f, indent=2)
    print('Wrote', out_path)


if __name__ == '__main__':
    main()
