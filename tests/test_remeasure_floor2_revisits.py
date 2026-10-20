import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from remeasure_floor2_revisits import (SETTINGS, conflict_free_tracks, freeze_selection,
    make_bundle_problem, rigid_observability)


def test_selection_never_fits_reserved_whole_frames():
    result = freeze_selection(range(1, 1219))
    assert len(result) == 2
    for spec in result:
        train = set(sum(spec["train_frames"], []))
        heldout = set(sum(spec["test_frames"], []))
        assert not train & heldout
        assert len(train) == 18 and len(heldout) == 8
    assert result[1]["historical_holdout_promoted_to_development"]


def test_track_union_rejects_contradictory_same_frame_features():
    tracks, rejected = conflict_free_tracks([
        (1., (1, 10), (2, 10)), (.9, (2, 10), (3, 20)),
        (.8, (1, 30), (3, 20))])
    assert rejected == 1
    assert [(1, 10), (2, 10), (3, 20)] in tracks
    assert all(len(t) == len({n for n, _ in t}) for t in tracks)


def test_fixed_landmark_rank_accepts_planar_noncollinear_points_not_a_line():
    plane = np.array([[x, y, 2.] for x in (-1., 0., 1.) for y in (-1., 0., 1.)])
    assert rigid_observability(plane)["rank"] == 6
    assert rigid_observability(np.array([[x, 0., 0.] for x in range(10)]))["rank"] < 6


def synthetic_bundle():
    truth = {n: np.eye(4) for n in (1, 2, 10, 11)}
    for node, x in [(2, .12), (10, .45), (11, .57)]:
        truth[node][0, 3] = x
    points = np.array([[x, y, z] for x in (-.4, .15, .5) for y in (-.3, .3)
                       for z in (2., 2.6)])
    k = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
    calibration = {n: (k, np.eye(4)) for n in truth}
    observations = []
    for node, pose in truth.items():
        for j, point in enumerate(points):
            p = point - pose[:3, 3]
            uv = np.array([500*p[0]/p[2]+320, 500*p[1]/p[2]+240])
            # One camera lacks all depth, but still constrains actual RGB reprojection.
            observations.append((node, j, uv, p[2] if node != 11 else np.nan))
    initial = {n: p.copy() for n, p in truth.items()}
    initial[10][0, 3] += .20; initial[11][0, 3] += .20
    priors = [(1, 2, np.linalg.inv(truth[1]) @ truth[2], .08, .05),
              (10, 11, np.linalg.inv(truth[10]) @ truth[11], .08, .05)]
    return truth, make_bundle_problem(list(truth), 1, initial, observations,
                                      points + [.04, -.03, .02], priors, calibration)


def test_bundle_really_optimizes_camera_and_landmark_unknowns():
    truth, (x0, residual, sparsity, unpack) = synthetic_bundle()
    fit = least_squares(residual, x0, jac_sparsity=sparsity, max_nfev=300,
                        x_scale="jac", ftol=1e-9, xtol=1e-9, gtol=1e-9)
    poses, landmarks = unpack(fit.x)
    assert fit.cost < 1e-9
    assert np.allclose(poses[1], np.eye(4))
    assert np.linalg.norm(poses[10][:3, 3] - truth[10][:3, 3]) < 1e-4
    assert np.linalg.norm(fit.x[:18]) > .1
    assert np.linalg.norm(landmarks - x0[18:].reshape(-1, 3)) > .05


def test_bundle_sparse_incidence_contains_all_actual_derivatives():
    _, (x0, residual, sparsity, _) = synthetic_bundle()
    baseline = residual(x0)
    mask = sparsity.toarray().astype(bool)
    for j in range(len(x0)):
        changed = x0.copy(); changed[j] += 1e-6
        actual = np.abs(residual(changed)-baseline) > 1e-9
        assert np.all(mask[actual, j])


def test_bundle_holdout_is_not_a_fit_parameter():
    import inspect
    assert "test" not in inspect.signature(make_bundle_problem).parameters
    assert SETTINGS["minimum_track_frames_each_visit"] == 2
