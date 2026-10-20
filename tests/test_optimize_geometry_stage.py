import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from optimize_geometry_stage import normal_residual, optimize_normals, reserve_native_edges, cross_audit_failures, paired_plane_residual, optimize_four_dof
from optimize_session import Edge, inverse


def test_normal_factors_cannot_flatten_stairs_or_train_heldout_edges():
    poses = {i: np.eye(4) for i in (1, 2, 3)}
    poses[2][:3, 3] = [1., 0., 2.]
    poses[3][:3, 3] = [2., 0., 4.]
    info = np.eye(6)*100
    edges = [Edge(a, b, 0, inverse(poses[a]) @ poses[b], info, info) for a, b in ((1, 2), (2, 3))]
    # An intentionally impossible audit is not a fitting constraint.
    edges.append(Edge(1, 3, 3, np.eye(4), info*1e8, info*1e8, True))
    normals = [{'global_id': i, 'normal_local': [0., 0., 1.]} for i in poses]
    result, report = optimize_normals(poses, edges, [[1, 2, 3]], normals, 10)
    assert report['success']
    assert report['active_relative_edges'] == 2
    assert report['plane_height_factors'] == 0
    for i in poses:
        np.testing.assert_allclose(result[i], poses[i], atol=1e-9)


def test_normal_orientation_factor_is_yaw_invariant_and_sensitive_to_tilt():
    rotations = Rotation.from_euler('z', [0., .9]).as_matrix()
    np.testing.assert_allclose(normal_residual(rotations, np.array([0, 1]), np.array([[0., 0., 1.]]*2)), 0., atol=1e-9)
    tilted = Rotation.from_euler('x', [.1]).as_matrix()
    assert np.linalg.norm(normal_residual(tilted, np.array([0]), np.array([[0., 0., 1.]]))) > 1


def test_reserving_a_pair_removes_native_training_leak_without_mutating_input():
    native = {'lobby': {'verified_loops': [{'from_id': 3, 'to_id': 9, 'held_out': False}],
                        'odometry': [{'from_id': 3, 'to_id': 4, 'held_out': False}]}}
    result = reserve_native_edges(native, {(('lobby', 3), ('lobby', 9))})
    assert result['lobby']['verified_loops'][0]['held_out']
    assert not native['lobby']['verified_loops'][0]['held_out']
    assert not result['lobby']['odometry'][0]['held_out']


def test_distinct_connections_do_not_mask_bad_holdout_median():
    rows = [dict(session_a='lobby', session_b='floor2', held_out=True, translation_m=.18, rotation_deg=1.),
            dict(session_a='floor2', session_b='stairs', held_out=True, translation_m=.05, rotation_deg=1.)]
    assert cross_audit_failures(rows) == ['floor2__lobby_cross_audit_translation_m_failed']


def test_audited_connection_cannot_hide_a_different_unaudited_connection():
    rows = [dict(session_a='lobby', session_b='floor2', held_out=True,
                 translation_m=.05, rotation_deg=1.),
            dict(session_a='lobby', session_b='stairs', held_out=False,
                 translation_m=.001, rotation_deg=.01)]
    assert cross_audit_failures(rows) == ['lobby__stairs_missing_whole_pair_cross_audit']


def test_four_dof_excludes_reserved_odometry_even_with_huge_information():
    poses = {i: np.eye(4) for i in (1, 2, 3)}
    poses[2][:3, 3] = [1., 0., 2.]
    poses[3][:3, 3] = [2., 0., 4.]
    info = np.eye(6)*100
    edges = [Edge(1, 2, 0, inverse(poses[1]) @ poses[2], info, info),
             Edge(2, 3, 3, inverse(poses[2]) @ poses[3], info, info),
             Edge(1, 3, 0, np.eye(4), info*1e8, info*1e8, True)]
    result, report = optimize_four_dof(poses, edges, [[1, 2, 3]], 10)
    assert report['success']
    for node in poses:
        np.testing.assert_allclose(result[node], poses[node], atol=1e-9)


def test_finite_plane_factor_leaves_tangential_translation_unconstrained():
    r = np.eye(3)[None]
    z = np.zeros((1, 3))
    n = np.array([[1., 0., 0.]])
    np.testing.assert_allclose(paired_plane_residual(r, z, r, np.array([[0., 2., 4.]]), n, n, z, z), 0.)
    residual = paired_plane_residual(r, z, r, np.array([[.08, 2., 4.]]), n, n, z, z)
    np.testing.assert_allclose(np.linalg.norm(residual), 1.)
