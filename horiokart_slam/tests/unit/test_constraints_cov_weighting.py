import json
from copy import deepcopy
from horiokart_slam.tools import gnss_optimize


def make_posegraph(n):
    nodes = []
    for i in range(n):
        nodes.append({'idx': i, 'pose': [float(i * 1.0), 0.0, 0.0]})
    return {'nodes': nodes, 'edges': []}


def test_cov_weighting_effect():
    pg = make_posegraph(3)
    # Two GNSS constraints for node 1: one very tight (small cov), one loose
    constraints = [
        {'node_idx': 1, 'x': 10.0, 'y': 0.0, 'cov': [0.01, 0.0, 0.0, 0.01]},
        {'node_idx': 1, 'x': 11.0, 'y': 0.0, 'cov': [10.0, 0.0, 0.0, 10.0]},
    ]
    pg1 = deepcopy(pg)
    out = gnss_optimize.optimize_posegraph(deepcopy(pg1), constraints, fix_first_node=False)
    # Node 1 should move toward ~10.0 because the tight cov dominates
    n1 = next(n for n in out['nodes'] if n['idx'] == 1)
    assert abs(n1['pose'][0] - 10.0) < 0.5
