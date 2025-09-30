from horiokart_slam.tools.gnss_optimize import optimize_posegraph
import sys
import os
import json
import numpy as np
from pathlib import Path

# ensure repository root (/app) is on sys.path for imports during pytest
THIS_DIR = os.path.abspath(os.path.dirname(__file__))
REPO_ROOT = os.path.abspath(os.path.join(THIS_DIR, '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def make_linear_posegraph(n=3, dx=1.0):
    nodes = []
    edges = []
    for i in range(n):
        nodes.append({'pose': [i * dx, 0.0, 0.0]})
        if i > 0:
            edges.append({'from_idx': i-1, 'to_idx': i,
                         'dx': dx, 'dy': 0.0, 'dtheta': 0.0})
    return {'nodes': nodes, 'edges': edges}


def make_gnss_constraints(pg):
    cons = []
    for i, n in enumerate(pg['nodes']):
        x = n['pose'][0] + 0.1  # small offset
        y = n['pose'][1] - 0.05
        cons.append({'node_idx': i, 'x': x, 'y': y, 'weight': 1.0})
    return {'constraints': cons}


def test_optimize_without_yaw():
    pg = make_linear_posegraph(3, dx=1.0)
    cons = make_gnss_constraints(pg)
    pg_opt, res = optimize_posegraph(
        pg, cons, include_yaw=False, fix_first_node=True)
    # check updated positions roughly match GNSS (offset applied)
    for i, n in enumerate(pg_opt['nodes']):
        assert abs(n['pose'][0] - (i*1.0 + 0.1)) < 0.5
        assert abs(n['pose'][1] - (-0.05)) < 0.5


def test_optimize_with_yaw():
    pg = make_linear_posegraph(3, dx=1.0)
    # introduce small yaw offsets
    for i in range(len(pg['nodes'])):
        pg['nodes'][i]['pose'][2] = 0.01 * i
    cons = make_gnss_constraints(pg)
    pg_opt, res = optimize_posegraph(
        pg, cons, include_yaw=True, fix_first_node=True)
    # yaw should be preserved/close to initial small offsets
    for i, n in enumerate(pg_opt['nodes']):
        assert abs(n['pose'][2] - 0.01 * i) < 0.2
