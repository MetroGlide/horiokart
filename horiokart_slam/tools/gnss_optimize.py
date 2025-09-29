#!/usr/bin/env python3
"""GNSS-constrained optimizer for 2D posegraphs.

This extends the PoC optimizer to accept constraints JSON produced by gnss_match.py
and applies a robust Huber-like loss through scipy.optimize.least_squares.
"""
from typing import List, Dict, Any
import json
import numpy as np
from math import atan2


def build_xy_array(nodes: List[Dict[str, Any]]) -> np.ndarray:
    # returns (N,2) array
    arr = np.array([[n['pose'][0], n['pose'][1]] for n in nodes], dtype=float)
    return arr


def residuals_flat(xy_flat: np.ndarray, nodes_init: np.ndarray, constraints: List[Dict[str, Any]], w_gnss: float = 1.0):
    # xy_flat: length 2N
    xy = xy_flat.reshape((-1, 2))
    res = []
    # GNSS residuals: for each constraint, difference between node xy and gnss
    for c in constraints:
        idx = c['node_idx']
        gx = c['x']
        gy = c['y']
        w = c.get('weight', 1.0)
        dx = xy[idx, 0] - gx
        dy = xy[idx, 1] - gy
        # scale by sqrt(weight)
        sw = np.sqrt(w) * w_gnss
        res.append(sw * dx)
        res.append(sw * dy)
    return np.array(res)


def residuals_with_edges(xy_flat: np.ndarray, nodes_init: np.ndarray, constraints: List[Dict[str, Any]], edges: List[Dict[str, Any]], edge_weight_scale: float = 1.0, w_gnss: float = 1.0):
    xy = xy_flat.reshape((-1, 2))
    res = list(residuals_flat(xy_flat, nodes_init, constraints, w_gnss))
    # edges residuals: for each edge, compute ((xb-xa) - measured_dx), scaled
    if edges:
        # build map of node indices: constraints use node_idx but edges reference node ids
        # assume posegraph nodes order corresponds to indices
        # we cannot map by id here, so edges should use indices in this PoC or provide 'from_idx'/'to_idx'
        for e in edges:
            if 'from_idx' in e and 'to_idx' in e:
                a = e['from_idx']
                b = e['to_idx']
            else:
                # best-effort: try to use 'from'/'to' as indices
                try:
                    a = int(e['from'])
                    b = int(e['to'])
                except Exception:
                    continue
            if a < 0 or b < 0 or a >= xy.shape[0] or b >= xy.shape[0]:
                continue
            xa = xy[a]
            xb = xy[b]
            dx_meas = e.get('dx', xb[0] - xa[0])
            dy_meas = e.get('dy', xb[1] - xa[1])
            rx = ((xb[0] - xa[0]) - dx_meas) * edge_weight_scale
            ry = ((xb[1] - xa[1]) - dy_meas) * edge_weight_scale
            res.append(rx)
            res.append(ry)
    return np.array(res)


def optimize_posegraph(posegraph: Dict[str, Any], constraints: Dict[str, Any], loss: str = 'huber', f_scale: float = 1.0, edge_weight_scale: float = 1.0, max_nfev: int = 200):
    from scipy.optimize import least_squares

    nodes = posegraph.get('nodes', [])
    xy0 = build_xy_array(nodes)
    cons = constraints.get('constraints', [])

    edges = posegraph.get('edges', [])

    def fun(x):
        # GNSS residuals + edges
        r = residuals_with_edges(
            x, xy0, cons, edges, edge_weight_scale=edge_weight_scale)
        return r

    x0 = xy0.flatten()
    # allow configurable loss and f_scale
    res = least_squares(fun, x0, loss=loss, f_scale=f_scale, max_nfev=max_nfev)
    x_opt = res.x.reshape((-1, 2))

    # write back into posegraph copy
    out_pg = dict(posegraph)
    out_nodes = []
    for i, n in enumerate(nodes):
        nn = dict(n)
        nn['pose'] = [float(x_opt[i, 0]), float(
            x_opt[i, 1]), nn.get('pose', [0, 0, 0])[2]]
        out_nodes.append(nn)
    out_pg['nodes'] = out_nodes
    return out_pg, res


def cli():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--posegraph', required=True)
    parser.add_argument('--constraints', required=True)
    parser.add_argument('--out', default='posegraph_opt.json')
    parser.add_argument('--loss', default='huber',
                        help='loss for least_squares (e.g., linear, soft_l1, huber, cauchy)')
    parser.add_argument('--f-scale', type=float, default=1.0,
                        help='f_scale parameter for robust loss')
    parser.add_argument('--edge-weight-scale', type=float, default=1.0,
                        help='scale to apply to edge residuals when integrated')
    args = parser.parse_args()
    with open(args.posegraph) as f:
        pg = json.load(f)
    with open(args.constraints) as f:
        cons = json.load(f)
    pg_opt, res = optimize_posegraph(
        pg, cons, loss=args.loss, f_scale=args.f_scale, edge_weight_scale=args.edge_weight_scale)
    with open(args.out, 'w') as f:
        json.dump(pg_opt, f, indent=2)
    print('Optimization result:', res.message)


if __name__ == '__main__':
    cli()
