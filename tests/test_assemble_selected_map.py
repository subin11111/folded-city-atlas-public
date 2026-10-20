import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from assemble_selected_map import auxiliary_pose, fit_placements, physical_error, yaw_transform
from optimize_session import inverse


def test_rigid_placement_keeps_height_and_relative_geometry():
    a = yaw_transform([1., 2., 3., .1]); b = yaw_transform([4., 6., 8., -.2])
    t = yaw_transform([2., -1., 9., .4])
    np.testing.assert_allclose(inverse(t@a)@(t@b), inverse(a)@b, atol=1e-12)
    assert ((t@b)[2, 3]-(t@a)[2, 3]) == pytest.approx(b[2, 3]-a[2, 3])


def test_heldout_row_does_not_change_placement():
    poses = {'a': {1: np.eye(4)}, 'b': {2: np.eye(4)}}
    row = {'session_a': 'a', 'session_b': 'b', 'node_a': 1, 'node_b': 2,
           'T_a_b': yaw_transform([.2, -.1, .3, .03]), 'role': 'train'}
    expected, _ = fit_placements(poses, [row], 'a')
    audit = dict(row, T_a_b=yaw_transform([100., 100., 100., 1.]), role='audit')
    actual, _ = fit_placements(poses, [row, audit], 'a')
    np.testing.assert_allclose(actual['b'], expected['b'], atol=1e-12)
    np.testing.assert_allclose(actual['b'], row['T_a_b'], atol=1e-7)


def test_disconnected_training_graph_rejected_even_with_audit_connection():
    poses = {'a': {1: np.eye(4)}, 'b': {2: np.eye(4)}}
    row = {'session_a': 'a', 'session_b': 'b', 'node_a': 1, 'node_b': 2,
           'T_a_b': np.eye(4), 'role': 'audit'}
    with pytest.raises(ValueError, match='disconnected'):
        fit_placements(poses, [row], 'a')


def rows_for_aux():
    return {n: {'stamp': str(n), 'primary_submap_id': 's1' if n!=2 else '',
                'hard_boundary': 'False', 'quality_grade': 'C' if n==2 else 'A'} for n in [1, 2, 3]}


def test_attachment_only_missing_pose_uses_local_correction():
    frozen = {n: yaw_transform([n, 0., 0., 0.]) for n in [1, 2, 3]}
    correction = yaw_transform([.2, .3, .4, .1])
    selected = {n: correction@frozen[n] for n in [1, 3]}
    pose, report = auxiliary_pose(2, selected, frozen, rows_for_aux(), [{'submap_id': 's1'}])
    np.testing.assert_allclose(pose, correction@frozen[2], atol=1e-12)
    assert report['excluded_from_cloud_and_saved_selected_trajectory']
    assert set(selected) == {1, 3}


def test_missing_pose_cannot_cross_hard_boundary():
    rows = rows_for_aux(); rows[2]['hard_boundary'] = 'True'
    with pytest.raises(ValueError, match='hard boundary'):
        auxiliary_pose(2, {1: np.eye(4), 3: np.eye(4)}, {2: np.eye(4)}, rows, [{'submap_id': 's1'}])


def test_missing_pose_cannot_cross_submap():
    rows = rows_for_aux(); rows[3]['primary_submap_id'] = 's2'
    with pytest.raises(ValueError, match='submap/run boundary'):
        auxiliary_pose(2, {1: np.eye(4), 3: np.eye(4)}, {2: np.eye(4)}, rows, [{'submap_id': 's1'}])


def test_residual_direction_is_node_b_into_node_a():
    a = yaw_transform([1., 2., 3., .7]); b = yaw_transform([-1., 4., .5, -.2])
    np.testing.assert_allclose(physical_error(inverse(a)@b, a, b), np.zeros(6), atol=1e-12)
