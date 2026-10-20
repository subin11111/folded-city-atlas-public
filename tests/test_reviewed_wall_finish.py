import json
from pathlib import Path

import numpy as np
import open3d as o3d

from repair_reviewed_wall_finish import covered_by_actual_patches

CFG = json.loads((Path(__file__).resolve().parents[1]/'config/final_reviewed_wall_patches_wide.json').read_text())


def wall_mesh():
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(
        [[0, 0, .1], [0, 1, .1], [0, 0, 1.1], [0, 1, 1.1]]),
        o3d.utility.Vector3iVector([[0, 1, 2], [1, 3, 2]]))


def test_actual_wall_coverage_removes_only_covered_unprotected_old_faces():
    inside = np.array([[.03, .2, .3], [.03, .4, .3], [.03, .2, .5]], np.float32)
    outside = inside+[0, 2, 0]
    xyz = np.array([inside, outside, inside], np.float32)
    record = {'geometry': {'plane_normal': [1, 0, 0]}}
    keep = covered_by_actual_patches(o3d, xyz, np.array([False, False, True]),
        [wall_mesh()], [record], np.eye(4), np.eye(3), CFG)
    np.testing.assert_array_equal(keep, [False, True, True])


def test_samples_crossing_real_wall_boundary_are_not_removed():
    triangle = np.array([[[.03, .8, .3], [.03, 1.1, .3], [.03, .8, .6]]], np.float32)
    record = {'geometry': {'plane_normal': [1, 0, 0]}}
    keep = covered_by_actual_patches(o3d, triangle, np.array([False]),
        [wall_mesh()], [record], np.eye(4), np.eye(3), CFG)
    assert keep[0]


def test_explicitly_protected_stair_surface_is_preserved():
    triangle = np.array([[[.01, .2, .3], [.04, .2, .3], [.01, .4, .3]]], np.float32)
    record = {'geometry': {'plane_normal': [1, 0, 0]}}
    keep = covered_by_actual_patches(o3d, triangle, np.array([True]),
        [wall_mesh()], [record], np.eye(4), np.eye(3), CFG)
    assert keep[0]
