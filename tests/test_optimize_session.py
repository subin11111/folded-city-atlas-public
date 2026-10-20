import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from optimize_session import (Edge, block_huber, checked_information, checked_transform,
    connected_components, correction_poses, inverse, optimize, relative_error, trimmed_icp)


def transform(xyz, rotation):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(rotation).as_matrix()
    result[:3, 3] = xyz
    return result


def test_zero_rotation_rejected_before_projection():
    with pytest.raises(ValueError, match="singular"):
        checked_transform(np.zeros((3, 4), dtype="<f4").tobytes())


def test_measurement_maps_j_cloud_into_i_and_gauge_invariant():
    a = transform([2., 3., 4.], [.2, -.4, .7])
    b = transform([6., 1., -2.], [-.1, .6, -.4])
    z = inverse(a) @ b
    point = np.array([.3, .7, 1.2, 1.])
    np.testing.assert_allclose(a @ z @ point, b @ point, atol=1e-12)
    np.testing.assert_allclose(relative_error(z, a, b), np.zeros(6), atol=1e-12)
    g = transform([7., 2., -1.], [1., .4, .7])
    np.testing.assert_allclose(relative_error(z, g @ a, g @ b), np.zeros(6), atol=1e-12)
    assert np.linalg.norm(relative_error(inverse(z), a, b)) > 1


def test_complete_residual_error_frame():
    a = transform([1., 2., 3.], [.2, .5, .7])
    z = transform([-.2, 1., 2.], [.4, -.2, .1])
    perturbation = transform([.1, .2, .3], [.02, -.04, .05])
    b = a @ z @ perturbation
    np.testing.assert_allclose(relative_error(z, a, b), [.1, .2, .3, .02, -.04, .05], atol=1e-12)


def test_full_information_whitening_and_invalid_covariance():
    rng = np.random.default_rng(4)
    a = rng.normal(size=(6, 6))
    info = a.T @ a + np.eye(6)
    checked = checked_information(info.astype("<f8").tobytes())
    e = rng.normal(size=6)
    w = np.linalg.cholesky(checked).T @ e
    np.testing.assert_allclose(w @ w, e @ info @ e)
    info[0, 0] = -1
    with pytest.raises(ValueError, match="nonpositive"):
        checked_information(info.astype("<f8").tobytes())


def test_huber_is_per_six_vector_block():
    residual = np.array([[3., 4., 0., 0., 0., 0.], [.1, 0., 0., 0., 0., 0.]])
    robust = block_huber(residual, 2.)
    np.testing.assert_allclose(.5 * np.sum(robust**2, axis=1), [8., .005])


def test_all_components_preserved_without_id_neighbor_attachment():
    edge = Edge(1, 8, 0, np.eye(4), np.eye(6), np.eye(6))
    components = connected_components([1, 8, 9, 20], [edge])
    assert components == [[1, 8], [9], [20]]
    raw = {i: transform([i, 0., 0.], [.1, .2, .3]) for i in [1, 8, 9, 20]}
    changed = correction_poses(raw, sorted(raw), [8], np.array([.4, -.2, .1, .3]))
    np.testing.assert_array_equal(changed[9], raw[9])
    np.testing.assert_array_equal(changed[20], raw[20])
    np.testing.assert_allclose(changed[8][:3, :3].T @ [0., 0., 1.], raw[8][:3, :3].T @ [0., 0., 1.])


def test_trimmed_icp_direction_on_nonplanar_geometry():
    rng = np.random.default_rng(12)
    source = rng.uniform(-1, 1, (1500, 3))
    known = transform([.04, -.03, .02], [.015, -.01, .02])
    target = source @ known[:3, :3].T + known[:3, 3]
    estimated, _ = trimmed_icp(target, source, np.eye(4))
    np.testing.assert_allclose(estimated, known, atol=1e-5)


def test_solver_uses_measurement_direction_and_keeps_isolated_pose():
    truth = {i: transform([i - 1., .2 * i, .1 * i], [.2, -.3, .4]) for i in [1, 2, 3, 9]}
    raw = {i: pose.copy() for i, pose in truth.items()}
    raw[3][:3, 3] += [.15, -.1, .08]
    raw[3][:3, :3] = Rotation.from_rotvec([0., 0., .05]).as_matrix() @ raw[3][:3, :3]
    information = np.diag([100., 200., 300., 400., 500., 600.])
    edges = [Edge(a, b, 0, inverse(truth[a]) @ truth[b], information, information) for a, b in [(1, 2), (2, 3)]]
    optimized, report = optimize(raw, edges, [], [[1, 2, 3], [9]], max_evaluations=60)
    assert report["success"]
    for node_id in [1, 2, 3]:
        np.testing.assert_allclose(optimized[node_id], truth[node_id], atol=1e-6)
    np.testing.assert_array_equal(optimized[9], raw[9])
