#!/usr/bin/env python3
"""GNSS-constrained optimizer for 2D posegraphs.

This extends the PoC optimizer to accept constraints JSON produced by gnss_match.py
and applies a robust Huber-like loss through scipy.optimize.least_squares.
"""
from typing import List, Dict, Any, Optional
import json
import numpy as np
from math import atan2, sin, cos, pi


def build_xy_array(nodes: List[Dict[str, Any]]) -> np.ndarray:
    # returns (N,2) array
    arr = np.array([[n['pose'][0], n['pose'][1]] for n in nodes], dtype=float)
    return arr


class IndexMap:
    """Helper to map node index -> variable indices for x,y,(theta).

    If include_yaw is False, var_per_node == 2 and idx_theta returns None.
    """

    def __init__(self, num_nodes: int, include_yaw: bool = False):
        self.num_nodes = num_nodes
        self.include_yaw = bool(include_yaw)
        self.var_per_node = 3 if self.include_yaw else 2
        self.var_size = self.num_nodes * self.var_per_node

    def idx_x(self, i: int) -> int:
        return i * self.var_per_node

    def idx_y(self, i: int) -> int:
        return i * self.var_per_node + 1

    def idx_theta(self, i: int) -> Optional[int]:
        if not self.include_yaw:
            return None
        return i * self.var_per_node + 2


def wrap_to_pi(a: float) -> float:
    # normalize to [-pi, pi]
    return (a + pi) % (2 * pi) - pi


def residuals_flat(z_flat: np.ndarray, nodes_init: np.ndarray, constraints: List[Dict[str, Any]], idx_map: IndexMap, w_gnss: float = 1.0):
    # z_flat: length var_size (2N or 3N)
    # returns a 1D residual array for GNSS constraints (position-only)
    if idx_map.var_per_node == 2:
        xy = z_flat.reshape((-1, 2))
    else:
        xy = z_flat.reshape((-1, idx_map.var_per_node))[:, :2]
    res = []
    # GNSS residuals: for each constraint, difference between node xy and gnss
    for c in constraints:
        idx = c['node_idx']
        gx = c['x']
        gy = c['y']
        w = c.get('weight', 1.0)
        dx = xy[idx, 0] - gx
        dy = xy[idx, 1] - gy
        r2 = np.array([dx, dy], dtype=float)
        # If cov present, compute sqrt-information and apply: r_weighted = sqrt_info @ r2
        cov = c.get('cov')
        if cov is not None:
            # cov may be a flat list (4) or 3x3, or nested list
            cov_arr = None
            try:
                carr = np.array(cov, dtype=float)
                if carr.size == 4:
                    cov_arr = carr.reshape((2, 2))
                elif carr.size >= 9:
                    cov_arr = carr.reshape((3, 3))[:2, :2]
                elif carr.ndim == 2 and carr.shape == (2, 2):
                    cov_arr = carr
            except Exception:
                cov_arr = None
            if cov_arr is not None:
                # store the covariance for weighting later via a placeholder in constraint
                c['_cov_arr'] = cov_arr
                # append raw residual scaled by sqrt(weight) for fallback
                sw = np.sqrt(w) * w_gnss
                res.append(sw * dx)
                res.append(sw * dy)
                continue
        # fallback: no covariance
        sw = np.sqrt(w) * w_gnss
        res.append(sw * dx)
        res.append(sw * dy)
    return np.array(res)


def residuals_with_edges(z_flat: np.ndarray, nodes_init: np.ndarray, constraints: List[Dict[str, Any]], idx_map: IndexMap, edges: List[Dict[str, Any]], edge_weight_scale: float = 1.0, w_gnss: float = 1.0, angle_weight: float = 1.0):
    # z_flat may be 2N or 3N
    if idx_map.var_per_node == 2:
        xy = z_flat.reshape((-1, 2))
    else:
        xy = z_flat.reshape((-1, idx_map.var_per_node))
    res = list(residuals_flat(z_flat, nodes_init,
               constraints, idx_map, w_gnss))
    # Now replace constraint residuals with covariance-weighted residuals where cov provided
    # constraints that had cov will have '_cov_arr' set; find and replace corresponding entries
    ri = 0
    for c in constraints:
        # each constraint contributed 2 residuals in residuals_flat
        if '_cov_arr' in c:
            idx = c['node_idx']
            gx = c['x']
            gy = c['y']
            dx = xy[idx, 0] - gx
            dy = xy[idx, 1] - gy
            r2 = np.array([dx, dy], dtype=float)
            cov = c['_cov_arr']
            # leave regularization and inversion to caller (optimize_posegraph) by using info stored in c if provided
            info = c.get('_info_matrix')
            if info is None:
                # fallback: use identity
                info = np.eye(2)
            # compute sqrt-information via Cholesky if possible
            try:
                L = np.linalg.cholesky(info)
                sqrt_info = L
            except Exception:
                # eigen-decompose and sqrt
                vals, vecs = np.linalg.eigh(info)
                vals[vals < 0] = 0.0
                sqrt_info = vecs @ np.diag(np.sqrt(vals)) @ vecs.T
            # apply sqrt_info and append into res replacement positions
            r_w = sqrt_info @ r2
            res[ri] = r_w[0]
            res[ri + 1] = r_w[1]
        ri += 2
    # edges residuals: for each edge, compute ((xb-xa) - measured_dx) (and angle if include_yaw), scaled
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
            # positions
            xa = np.array([z_flat[idx_map.idx_x(a)], z_flat[idx_map.idx_y(a)]])
            xb = np.array([z_flat[idx_map.idx_x(b)], z_flat[idx_map.idx_y(b)]])
            dx_meas = e.get('dx', xb[0] - xa[0])
            dy_meas = e.get('dy', xb[1] - xa[1])
            if idx_map.include_yaw:
                # compute relative in frame a
                theta_a = z_flat[idx_map.idx_theta(a)]
                theta_b = z_flat[idx_map.idx_theta(b)]
                ca = cos(theta_a)
                sa = sin(theta_a)
                R_a_T = np.array([[ca, sa], [-sa, ca]])  # R(θ)^T
                delta = R_a_T @ (xb - xa)
                dtheta_pred = wrap_to_pi(theta_b - theta_a)
                rx = (delta[0] - dx_meas) * edge_weight_scale
                ry = (delta[1] - dy_meas) * edge_weight_scale
                rth = wrap_to_pi(dtheta_pred - e.get('dtheta', 0.0)
                                 ) * angle_weight * edge_weight_scale
                res.append(float(rx))
                res.append(float(ry))
                res.append(float(rth))
            else:
                rx = ((xb[0] - xa[0]) - dx_meas) * edge_weight_scale
                ry = ((xb[1] - xa[1]) - dy_meas) * edge_weight_scale
                res.append(float(rx))
                res.append(float(ry))
    return np.array(res)


def optimize_posegraph(posegraph: Dict[str, Any], constraints: Dict[str, Any], loss: str = 'huber', f_scale: float = 1.0, edge_weight_scale: float = 1.0, max_nfev: int = 200, cov_regularization: float = 1e-6, include_yaw: bool = False, fix_first_node: bool = True, fix_weight: float = 1e3, angle_weight: float = 1.0):
    from scipy.optimize import least_squares

    nodes = posegraph.get('nodes', [])
    # build initial pose array (N x 3) with yaw default 0 if missing
    N = len(nodes)
    pose0 = np.zeros((N, 3), dtype=float)
    for i, n in enumerate(nodes):
        p = n.get('pose', [0.0, 0.0, 0.0])
        pose0[i, 0] = float(p[0])
        pose0[i, 1] = float(p[1])
        pose0[i, 2] = float(p[2]) if len(p) > 2 else 0.0
    cons = constraints.get('constraints', [])

    edges = posegraph.get('edges', [])
    cov_reg = float(
        cov_regularization) if cov_regularization is not None else 1e-6

    # build index map (2N or 3N)
    idx_map = IndexMap(len(nodes), include_yaw=include_yaw)

    # initial variable vector z0
    z0 = np.zeros(idx_map.var_size, dtype=float)
    for i in range(len(nodes)):
        z0[idx_map.idx_x(i)] = pose0[i, 0]
        z0[idx_map.idx_y(i)] = pose0[i, 1]
        if idx_map.include_yaw:
            z0[idx_map.idx_theta(i)] = pose0[i, 2]

    def fun(x):
        # GNSS residuals + edges
        # If constraints contain cov, build per-constraint info matrices once
        for c in cons:
            if 'cov' in c and '_info_matrix' not in c:
                try:
                    carr = np.array(c['cov'], dtype=float)
                    if carr.size == 4:
                        cov2 = carr.reshape((2, 2))
                    elif carr.size >= 9:
                        cov2 = carr.reshape((3, 3))[:2, :2]
                    else:
                        cov2 = None
                except Exception:
                    cov2 = None
                if cov2 is not None:
                    # regularize and invert
                    reg = cov_reg if cov_reg is not None else 1e-6
                    cov_reg_mat = cov2 + reg * np.eye(2)
                    try:
                        info = np.linalg.inv(cov_reg_mat)
                    except Exception:
                        # fallback pseudo-inverse
                        info = np.linalg.pinv(cov_reg_mat)
                    c['_info_matrix'] = info
        r = residuals_with_edges(
            x, pose0, cons, idx_map, edges, edge_weight_scale=edge_weight_scale, w_gnss=1.0, angle_weight=angle_weight)

        # virtual observation for gauge fixing (fix first node)
        if fix_first_node and len(nodes) > 0:
            # anchor first node to its initial pose via a strong residual (virtual obs)
            fx = x[idx_map.idx_x(0)] - pose0[0, 0]
            fy = x[idx_map.idx_y(0)] - pose0[0, 1]
            r = np.concatenate(
                [r, np.array([fix_weight * fx, fix_weight * fy])])
            if idx_map.include_yaw:
                fth = wrap_to_pi(x[idx_map.idx_theta(0)] - pose0[0, 2])
                r = np.concatenate(
                    [r, np.array([fix_weight * angle_weight * fth])])
        return r

    x0 = z0
    # allow configurable loss and f_scale
    res = least_squares(fun, x0, loss=loss, f_scale=f_scale, max_nfev=max_nfev)
    # reshape according to idx_map
    if idx_map.include_yaw:
        x_opt = res.x.reshape((-1, 3))
    else:
        x_opt = res.x.reshape((-1, 2))

    # write back into posegraph copy
    out_pg = dict(posegraph)
    out_nodes = []
    for i, n in enumerate(nodes):
        nn = dict(n)
        if idx_map.include_yaw:
            nn['pose'] = [float(x_opt[i, 0]), float(
                x_opt[i, 1]), float(x_opt[i, 2])]
        else:
            # keep original yaw
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
    parser.add_argument('--cov-regularization', type=float, default=1e-6,
                        help='regularization added to 2x2 GNSS covariance before inversion')
    parser.add_argument('--include-yaw', action='store_true',
                        help='include yaw (theta) into optimization (default: False)')
    # fix-first-node: provide both --fix-first-node and --no-fix-first-node to allow toggling
    parser.add_argument('--fix-first-node', dest='fix_first_node', action='store_true',
                        help='apply virtual observation to fix first node pose (default: True)')
    parser.add_argument('--no-fix-first-node', dest='fix_first_node', action='store_false',
                        help='do not apply virtual observation to fix first node pose')
    parser.set_defaults(fix_first_node=True)
    parser.add_argument('--fix-weight', type=float, default=1e3,
                        help='weight for virtual observation used to fix first node')
    parser.add_argument('--angle-weight', type=float, default=1.0,
                        help='scaling applied to angle residuals (rad)')
    args = parser.parse_args()
    with open(args.posegraph) as f:
        pg = json.load(f)
    with open(args.constraints) as f:
        cons = json.load(f)
    pg_opt, res = optimize_posegraph(
        pg, cons, loss=args.loss, f_scale=args.f_scale, edge_weight_scale=args.edge_weight_scale, cov_regularization=args.cov_regularization, include_yaw=args.include_yaw, fix_first_node=args.fix_first_node, fix_weight=args.fix_weight, angle_weight=args.angle_weight)
    with open(args.out, 'w') as f:
        json.dump(pg_opt, f, indent=2)
    print('Optimization result:', res.message)


if __name__ == '__main__':
    cli()
