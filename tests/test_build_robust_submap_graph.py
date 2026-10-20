import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from build_robust_submap_graph import (
    annotate_structural_support, compose_measurements, edge_covariance,
    extra_constraint_role, inverse, prefer_successful_replacements,
    reverse_measurement, skew, transform_frame_edge,
)


def pose(translation, rotvec):
    out = np.eye(4)
    out[:3, 3] = translation
    out[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    return out


def perturb(vector):
    algebra = np.zeros((4, 4))
    algebra[:3, :3] = skew(vector[3:])
    algebra[:3, 3] = vector[:3]
    return expm(algebra)


def tangent(transform):
    algebra = np.real_if_close(logm(transform))
    return np.r_[algebra[:3, 3], algebra[2, 1], algebra[0, 2], algebra[1, 0]]


def jacobian(function, size):
    base = function(np.zeros(size))
    epsilon = 1e-6
    return np.stack([
        (tangent(inverse(base) @ function(np.eye(size)[i] * epsilon))
         - tangent(inverse(base) @ function(-np.eye(size)[i] * epsilon))) / (2 * epsilon)
        for i in range(size)], axis=1)


def test_covariance_propagation_matches_numerical_se3_jacobians():
    first = pose([1.2, -.8, 2.], [.3, -.5, .8])
    second = pose([-2., 1., .7], [-.2, .6, .4])
    la = pose([.5, -.7, 1.3], [-.6, .2, .1])
    lb = pose([3., -2., 1.], [.5, .4, -.2])
    rng = np.random.default_rng(12)
    a, b = rng.normal(size=(6, 6)), rng.normal(size=(6, 6))
    ca, cb = a @ a.T * 1e-4, b @ b.T * 1e-4

    value, cov = reverse_measurement(first, ca)
    derivative = jacobian(lambda d: inverse(first @ perturb(d)), 6)
    np.testing.assert_allclose(value, inverse(first), atol=1e-12)
    np.testing.assert_allclose(cov, derivative @ ca @ derivative.T, rtol=1e-6, atol=1e-9)

    value, cov = compose_measurements(first, ca, second, cb)
    derivative = jacobian(lambda d: first @ perturb(d[:6]) @ second @ perturb(d[6:]), 12)
    joint = np.zeros((12, 12)); joint[:6, :6] = ca; joint[6:, 6:] = cb
    np.testing.assert_allclose(value, first @ second, atol=1e-12)
    np.testing.assert_allclose(cov, derivative @ joint @ derivative.T, rtol=1e-6, atol=1e-9)

    value, cov = transform_frame_edge(first, ca, la, lb)
    derivative = jacobian(lambda d: la @ first @ perturb(d) @ inverse(lb), 6)
    np.testing.assert_allclose(value, la @ first @ inverse(lb), atol=1e-12)
    np.testing.assert_allclose(cov, derivative @ ca @ derivative.T, rtol=1e-6, atol=1e-9)


def test_covariance_source_is_disclosed_and_indefinite_input_rejected():
    expected = np.diag([.01, .02, .03, .004, .005, .006])
    actual, diagnostic = edge_covariance({'metrics': {'covariance_6x6': expected.tolist()}}, True)
    np.testing.assert_allclose(actual, expected)
    assert diagnostic['source'] == 'metrics.covariance_6x6'
    assert not diagnostic['was_clipped']
    with pytest.raises(ValueError, match='non-positive'):
        edge_covariance({'information': np.diag([1, 1, 1, 1, 1, -1]).tolist()})


def test_local_evidence_is_never_promoted_to_training():
    assert extra_constraint_role({'local_geometry_verified': True,
                                  'joint_role': 'local_evidence_only'}) == 'local_evidence_only'
    assert extra_constraint_role({'accepted': True, 'joint_role': 'local_evidence_only',
                                  'held_out': True}) == 'local_evidence_only'
    with pytest.raises(ValueError, match='also marked heldout'):
        extra_constraint_role({'accepted': True, 'joint_role': 'train', 'held_out': True})
    with pytest.raises(ValueError, match='positive evidence'):
        extra_constraint_role({'joint_role': 'train'})


def test_native_replacement_requires_surviving_mapped_extra():
    native = {'id': 'native', '_source_kind': 'native', '_frame_key': ('a', 'b')}
    kept, replaced = prefer_successful_replacements([native])
    assert [row['id'] for row in kept] == ['native'] and not replaced
    kept, replaced = prefer_successful_replacements([
        native, {'id': 'extra', '_source_kind': 'extra', '_frame_key': ('a', 'b')}])
    assert [row['id'] for row in kept] == ['extra'] and replaced == ['native']
    assert all(not key.startswith('_') for row in kept for key in row)


def test_heldout_cannot_provide_structural_cycle_support():
    edges = [
        {'id': 't', 'from': 'a', 'to': 'b', 'type': 'structural', 'role': 'train', 'provenance': {}},
        {'id': 'o', 'from': 'b', 'to': 'c', 'type': 'odometry', 'role': 'train'},
        {'id': 'h', 'from': 'a', 'to': 'c', 'type': 'loop', 'role': 'heldout'},
    ]
    assert annotate_structural_support(edges)[0]['graph_bridge']
    edges[-1]['role'] = 'train'
    support = annotate_structural_support(edges)[0]
    assert support['cycle_supported'] and support['non_structural_alternate_path']
