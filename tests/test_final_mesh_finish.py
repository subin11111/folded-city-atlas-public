import json
from pathlib import Path

import numpy as np

from finalize_display_mesh import (covered_horizontal_remnants, duplicate_keep,
    join_tiny_seams, prune_fragments, write_clean_glb)

CFG = json.loads((Path(__file__).resolve().parents[1]/'config/final_mesh_finish.json').read_text())


def test_independent_triangle_key_is_unoriented_and_translation_sensitive():
    from audit_final_mesh_finish import triangle_keys
    xyz = np.array([[[1, 0, 0], [0, 1, 0], [0, 0, 1]]], np.float32)
    np.testing.assert_array_equal(triangle_keys(xyz), triangle_keys(xyz[:, ::-1]))
    assert triangle_keys(xyz)[0] != triangle_keys(xyz+.001)[0]


def test_duplicate_triangle_prefers_semantic_or_authority_and_ignores_winding():
    faces = np.array([[0, 1, 2], [2, 0, 1], [3, 4, 5]])
    np.testing.assert_array_equal(duplicate_keep(faces, [1, 9, 0]), [False, True, True])


def test_tiny_real_semantic_component_is_preserved():
    vertices = np.array([[0, 0, 0], [.01, 0, 0], [0, .01, 0],
                         [1, 0, 0], [1.01, 0, 0], [1, .01, 0]])
    faces = np.array([[0, 1, 2], [3, 4, 5]])
    remove, _ = prune_fragments(vertices, faces, np.array([True, False]), CFG)
    np.testing.assert_array_equal(remove, [False, True])


def test_covered_removal_requires_all_samples_and_preserves_opening():
    triangle = np.array([[[.01, .01, .05], [.07, .01, .04], [.01, .07, .04]]], np.float32)
    mask = np.ones((2, 2), bool)
    removed = covered_horizontal_remnants(triangle, np.array([False]), mask, [0, 0], .1, 0., .16, CFG)
    assert removed[0]
    mask[0, 0] = False
    assert not covered_horizontal_remnants(triangle, np.array([False]), mask, [0, 0], .1, 0., .16, CFG)[0]
    assert not covered_horizontal_remnants(triangle, np.array([True]), ~mask, [0, 0], .1, 0., .16, CFG)[0]


def test_seams_join_small_coplanar_distinct_components_only():
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0],
                  [1.01, 0, 0], [2, 0, 0], [1.01, 1, 0]], np.float32)
    faces = np.array([[0, 1, 2], [3, 4, 5]])
    moved, report = join_tiny_seams(v, faces, np.array([False, False]), CFG)
    np.testing.assert_array_equal(moved[1], moved[3])
    assert report['joined_boundary_pairs'] == 1
    assert np.linalg.norm(moved-v, axis=1).max() < .01
    guarded, report = join_tiny_seams(v, faces, np.array([True, False]), CFG)
    np.testing.assert_array_equal(guarded, v)
    assert report['joined_boundary_pairs'] == 0


def test_large_or_non_coplanar_gap_is_not_filled():
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0],
                  [1.03, 0, 0], [2, 0, 0], [1.03, 1, 0]], np.float32)
    faces = np.array([[0, 1, 2], [3, 4, 5]])
    moved, report = join_tiny_seams(v, faces, np.array([False, False]), CFG)
    np.testing.assert_array_equal(moved, v)
    assert report['joined_boundary_pairs'] == 0


def test_index_filtered_glb_with_unused_accessor_vertices(tmp_path):
    from surface_model_utils import write_glb
    from clean_structural_fragments import filter_glb
    from audit_structural_delivery import glb_triangle_positions
    from refine_display_edges import unpack
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [2, 0, 0]], np.float32)
    original, filtered, target = [tmp_path/n for n in ['original.glb', 'filtered.glb', 'out.glb']]
    write_glb(original, v, np.array([[0, 1, 2], [1, 3, 2]]), np.tile([0, 0, 1], (4, 1)), np.ones((4, 3)))
    filter_glb(original, filtered, np.array([True, False]))
    xyz = glb_triangle_positions(filtered)
    source = np.unique(xyz.reshape(-1, 3), axis=0)
    moved = source.copy()
    moved[:, 0] += .001
    write_clean_glb(filtered, target, np.array([True]), source, moved)
    np.testing.assert_allclose(glb_triangle_positions(target), xyz+[.001, 0, 0], atol=1e-7)
    old, before = unpack(filtered)
    new, after = unpack(target)
    assert after[:len(before)] == before
    assert old['materials'] == new['materials']
