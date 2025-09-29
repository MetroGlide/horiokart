from tools import gnss_match, gnss_optimize
import json
import tempfile
import os
import math
import sys

# Ensure local package path is on sys.path so tests can import tools
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def make_fixture():
    # Simple posegraph: 3 nodes at t=0,1,2 with poses near origin
    pg = {'nodes': [
        {'pose': [0.0, 0.0, 0.0], 'timestamp': 0.0},
        {'pose': [1.0, 0.0, 0.0], 'timestamp': 1.0},
        {'pose': [2.0, 0.0, 0.0], 'timestamp': 2.0},
    ]}
    # GNSS records slightly shifted (+10m in x)
    gnss = {'records': [
        {'time': 0.0, 'x': 10.0, 'y': 0.0, 'accuracy': 1.0},
        {'time': 1.0, 'x': 11.0, 'y': 0.0, 'accuracy': 1.0},
        {'time': 2.0, 'x': 12.0, 'y': 0.0, 'accuracy': 1.0},
    ]}
    return pg, gnss


def test_match_basic():
    pg, gnss = make_fixture()
    cons = gnss_match.match_posegraph_with_gnss(pg, gnss, window=0.5)
    assert 'constraints' in cons
    assert len(cons['constraints']) == 3
    # check interpolation exactness
    for i, c in enumerate(cons['constraints']):
        assert abs(c['x'] - (10.0 + i)) < 1e-6


def test_match_weight_modes():
    pg, gnss = make_fixture()
    # var_interp should give weights > 0
    cons = gnss_match.match_posegraph_with_gnss(
        pg, gnss, window=0.5, weight_mode='var_interp')
    assert all(c['weight'] > 0 for c in cons['constraints'])
    # fixed mode should set weight to 1.0 for interpolated ones
    cons2 = gnss_match.match_posegraph_with_gnss(
        pg, gnss, window=0.5, weight_mode='fixed')
    assert all(abs(c['weight'] - 1.0) < 1e-9 for c in cons2['constraints'])


def test_match_cov_and_hdop_modes():
    pg, gnss = make_fixture()
    # Add cov and hdop fields to GNSS fixtures
    gnss_cov = {'records': [
        {'time': 0.0, 'x': 10.0, 'y': 0.0, 'cov': [1.0, 0.0, 0.0, 1.0]},
        {'time': 1.0, 'x': 11.0, 'y': 0.0, 'cov': [1.0, 0.0, 0.0, 1.0]},
        {'time': 2.0, 'x': 12.0, 'y': 0.0, 'cov': [1.0, 0.0, 0.0, 1.0]},
    ]}
    cons_cov = gnss_match.match_posegraph_with_gnss(
        pg, gnss_cov, window=0.5, weight_mode='cov_trace')
    assert all(c['weight'] > 0 for c in cons_cov['constraints'])

    gnss_hdop = {'records': [
        {'time': 0.0, 'x': 10.0, 'y': 0.0, 'hdop': 0.5},
        {'time': 1.0, 'x': 11.0, 'y': 0.0, 'hdop': 0.8},
        {'time': 2.0, 'x': 12.0, 'y': 0.0, 'hdop': 1.2},
    ]}
    cons_hdop = gnss_match.match_posegraph_with_gnss(
        pg, gnss_hdop, window=0.5, weight_mode='hdop')
    assert all(c['weight'] > 0 for c in cons_hdop['constraints'])


def test_optimize_moves_nodes():
    pg, gnss = make_fixture()
    cons = gnss_match.match_posegraph_with_gnss(pg, gnss, window=0.5)
    pg_opt, res = gnss_optimize.optimize_posegraph(pg, cons)
    # Nodes should have moved near GNSS x (~+10..+12)
    for i, n in enumerate(pg_opt['nodes']):
        assert abs(n['pose'][0] - (10.0 + i)) < 0.5


def test_optimize_with_edge_preservation():
    pg, gnss = make_fixture()
    # define edges between consecutive nodes with measured dx ~1.0
    pg['edges'] = [
        {'from_idx': 0, 'to_idx': 1, 'dx': 1.0, 'dy': 0.0},
        {'from_idx': 1, 'to_idx': 2, 'dx': 1.0, 'dy': 0.0},
    ]
    cons = gnss_match.match_posegraph_with_gnss(pg, gnss, window=0.5)
    # set a high edge weight so optimizer respects edges
    pg_opt, res = gnss_optimize.optimize_posegraph(
        pg, cons, edge_weight_scale=10.0)
    # distances between consecutive nodes should remain ~1.0

    def dist(a, b):
        return math.hypot(a['pose'][0] - b['pose'][0], a['pose'][1] - b['pose'][1])
    d01 = dist(pg_opt['nodes'][0], pg_opt['nodes'][1])
    d12 = dist(pg_opt['nodes'][1], pg_opt['nodes'][2])
    assert abs(d01 - 1.0) < 0.2
    assert abs(d12 - 1.0) < 0.2
