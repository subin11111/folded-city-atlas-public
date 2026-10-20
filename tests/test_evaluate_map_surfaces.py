import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from evaluate_map_surfaces import (Settings, anchor_align, fit_planes, pair_metrics,
                                  parse_overrides, patch_correspondence, temporal_bands)


def grid_plane(z, seed, n=32):
    rng = np.random.default_rng(seed)
    x, y = np.meshgrid(np.linspace(-.8, .8, n), np.linspace(-.8, .8, n))
    points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))
    return points + rng.normal(0, .002, points.shape)


def transform(xyz=(0., 0., 0.), rotation=(0., 0., 0.)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(rotation).as_matrix()
    result[:3, 3] = xyz
    return result


def test_horizontal_fit_and_heldout_validity():
    points = grid_plane(-1.4, 4)
    rng = np.random.default_rng(8)
    training = np.vstack((points[::2], rng.uniform(-1, 1, (80, 3))))
    patches, diagnostics = fit_planes(training, points[1::2], "horizontal", Settings(), 3)
    assert len(patches) == 1
    assert diagnostics[0]["accepted"]
    assert abs(patches[0]["center"][2] + 1.4) < .01
    assert diagnostics[0]["heldout_inlier_fraction"] > .95


def test_separate_horizontal_levels_not_flattened():
    a, b = grid_plane(-1.4, 1), grid_plane(-1.05, 2)
    train, test = np.vstack((a[::2], b[::2])), np.vstack((a[1::2], b[1::2]))
    patches, _ = fit_planes(train, test, "horizontal", Settings(), 7)
    assert len(patches) == 2
    assert np.ptp([p["center"][2] for p in patches]) == pytest.approx(.35, abs=.01)


def test_steep_and_near_vertical_planes_not_floors():
    points = grid_plane(-1.4, 1)
    points[:, 2] += points[:, 0] * .5
    patches, _ = fit_planes(points[::2], points[1::2], "horizontal", Settings(), 7)
    assert not patches


def test_wall_detected_separately():
    points = grid_plane(-1.4, 1)[:, [2, 1, 0]]
    patches, _ = fit_planes(points[::2], points[1::2], "wall", Settings(), 7)
    assert len(patches) == 1
    assert abs(patches[0]["normal"][0]) > .99


def test_inconsistent_test_points_reject_plane():
    points = grid_plane(-1.4, 1)
    testing = points[1::2].copy()
    testing[:, 2] += .09
    patches, diagnostics = fit_planes(points[::2], testing, "horizontal", Settings(), 7)
    assert not patches
    assert "heldout_inlier_fraction" in diagnostics[0]["rejection_reasons"]


def test_plane_pair_is_gauge_invariant_and_vertical_shift_visible():
    points = grid_plane(-1.4, 1)
    a = {"kind": "horizontal", "normal": np.array([0., 0., 1.]),
         "center": np.array([0., 0., -1.4]), "training": points[::2], "testing": points[1::2]}
    b = dict(a)
    p, q = transform(), transform((0, 0, .25))
    gauge = transform((3, -2, 8), (.4, .1, -.7))
    first, second = pair_metrics(a, b, p, q), pair_metrics(a, b, gauge @ p, gauge @ q)
    assert first["symmetric_plane_abs_median_m"] == pytest.approx(.25, abs=.002)
    for key in first:
        assert first[key] == pytest.approx(second[key], abs=1e-10)
    assert patch_correspondence(a, b, p, Settings())
    assert patch_correspondence(a, b, q, Settings()) is None


def test_temporal_bands_preserve_height_change_and_gauge_translation():
    ids = list(range(100))
    stamps = {i: float(i) for i in ids}
    poses = {i: transform((0, 0, np.clip((i - 25) * .1, 0, 4))) for i in ids}
    rows, _ = temporal_bands(ids, stamps, poses, Settings())
    assert rows[45]["proxy_label"] == "height_increase"
    assert rows[10]["proxy_label"] == "locally_stable_height"
    aligned, anchor, _ = anchor_align(poses, {i: transform((0, 0, 10)) @ p for i, p in poses.items()})
    assert anchor == 0
    np.testing.assert_allclose(aligned[99], poses[99])


def test_override_names_and_reserved_labels():
    assert ("floor2", "joint") in parse_overrides(["floor2:joint:/tmp/poses.csv"])
    with pytest.raises(ValueError):
        parse_overrides(["../bad:joint:/tmp/poses.csv"])
    with pytest.raises(ValueError):
        parse_overrides(["floor2:raw:/tmp/poses.csv"])
    with pytest.raises(ValueError):
        parse_overrides(["floor2:joint:a", "floor2:joint:b"])
