import json
import numpy as np
from horiokart_slam.tools.gnss_optimize import optimize_posegraph


def make_posegraph(n=3):
    nodes = []
    for i in range(n):
        nodes.append({'id': i, 'pose': [float(i), 0.0, 0.0]})
    edges = []
    return {'nodes': nodes, 'edges': edges}


def test_covariance_weighting_changes_solution():
    pg = make_posegraph(2)
    # two nodes at (0,0) and (1,0)
    # create a GNSS constraint for node 0 that strongly pulls to (10,0)
    cons = {'constraints': [
        {'node_idx': 0, 'x': 10.0, 'y': 0.0,
            'weight': 1.0, 'cov': [10.0, 0.0, 0.0, 10.0]},
        {'node_idx': 1, 'x': 1.0, 'y': 0.0, 'weight': 1.0}
    ]}
    # without regularization -> info ~ inv(cov) small -> constraint weak
    pg_opt1, res1 = optimize_posegraph(
        pg, cons, loss='linear', f_scale=1.0, edge_weight_scale=1.0, cov_regularization=1e-8)
    # with larger regularization (effectively smaller cov) -> stronger constraint
    pg_opt2, res2 = optimize_posegraph(
        pg, cons, loss='linear', f_scale=1.0, edge_weight_scale=1.0, cov_regularization=1.0)
    x1 = pg_opt1['nodes'][0]['pose'][0]
    x2 = pg_opt2['nodes'][0]['pose'][0]
    assert abs(x2 - 10.0) < abs(x1 - 10.0) + 1e-6
