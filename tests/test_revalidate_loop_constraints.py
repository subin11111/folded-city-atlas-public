import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from revalidate_loop_constraints import (covariance_model, delta_summary,
    exclusion_mask, graph_holdout, holdout_score, split_correspondences,
    conditioning_diagnostic, planar_aware_support)
from check_session_overlap import apply_transform, rigid_fit
from optimize_session import Edge, inverse, relative_error


def test_measurement_direction_and_right_tangent():
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', [.2, -.3, .7]).as_matrix()
    t[:3, 3] = [.4, -.6, 1.2]
    source = np.random.default_rng(5).normal(size=(50, 3))
    target = apply_transform(t, source)
    fit = rigid_fit(source, target)
    np.testing.assert_allclose(fit, t, atol=1e-12)
    np.testing.assert_allclose(relative_error(fit, np.eye(4), t), 0., atol=1e-12)
    assert holdout_score(fit, source, target)['rmse_m'] < 1e-12
    assert holdout_score(inverse(fit), source, target)['rmse_m'] > 1


def test_exclusion_contains_complete_depth_patch_and_clips_boundaries():
    mask = exclusion_mask((12, 16), [[0, 0], [8, 7]], 3)
    assert mask[:4, :4].all()
    assert mask[4:11, 5:12].all()
    assert not mask[11, 15]
    assert mask.dtype == bool


def test_correspondence_split_is_reproducible_and_depth_neighbour_disjoint():
    points = np.array([[x, y] for x in range(4, 96, 8) for y in range(4, 96, 8)])
    points = np.vstack([points, points[:5], points[:5]+[1, 1]])
    train, holdout, masks = split_correspondences(points, points, (100, 100), (100, 100), 3)
    again = split_correspondences(points, points, (100, 100), (100, 100), 3)
    np.testing.assert_array_equal(train, again[0])
    np.testing.assert_array_equal(holdout, again[1])
    assert len(train) >= 12 and len(holdout) >= 5
    assert set(train).isdisjoint(holdout)
    assert not masks[0][points[train, 1], points[train, 0]].any()
    distance = np.max(np.abs(points[train, None]-points[None, holdout]), axis=2)
    assert distance.min() > 3


def test_covariance_is_explicit_spd_model_not_residual_fit():
    model = covariance_model()
    covariance, info = np.array(model['covariance_6x6']), np.array(model['information_6x6'])
    np.testing.assert_allclose(covariance @ info, np.eye(6), atol=1e-12)
    np.testing.assert_allclose(np.sqrt(np.diag(covariance)[:3]), .10)
    np.testing.assert_allclose(np.degrees(np.sqrt(np.diag(covariance)[3:])), 3.)
    assert 'not fitted/calibrated' in model['source']
    assert np.linalg.eigvalsh(covariance).min() > 0


def test_previous_whole_pair_holdout_cannot_be_promoted():
    edge = Edge(63, 41, 1, np.eye(4), np.eye(6), np.eye(6))
    holdout, reason = graph_holdout('bridge_out1f', edge, {(41, 63)}, {(41, 63)})
    assert holdout and 'preserved_previous' in reason
    assert graph_holdout('bridge_out1f', edge, set(), {(41, 63)})[0] is False


def test_geometry_diagnostic_exposes_collinearity():
    line = np.column_stack([np.arange(20), np.zeros((20, 2))])
    diagnostic = conditioning_diagnostic(line)
    assert diagnostic['centred_rotation_normal_eigenvalues_m2'][0] == 0
    assert diagnostic['rotation_normal_condition'] > 1e12


def test_planar_noncollinear_correspondences_observe_rigid_rotation():
    import json
    x, y = np.meshgrid(np.linspace(-.5, .5, 6), np.linspace(-.4, .4, 6))
    points = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    pixels = points[:, :2]*300 + [500, 500]
    accepted, diagnostic = planar_aware_support(points, pixels, (1000, 1000))
    assert accepted
    assert not diagnostic['previous_volume_gate_passed']
    assert diagnostic['centred_rotation_normal_eigenvalues_per_point_m2'][0] > .02
    assert diagnostic['std_m'][2] == 0
    json.dumps(diagnostic)


def test_planar_gate_still_rejects_narrow_or_collinear_matches():
    x = np.linspace(-1, 1, 30)
    line = np.column_stack([x, x*.01, x*0])
    pixels = np.column_stack([x*300+400, np.sin(x)*100+400])
    assert not planar_aware_support(line, pixels, (1000, 1000))[0]
