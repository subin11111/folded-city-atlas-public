import numpy as np
import pytest
from explore_substairs_attachment import fit, geometry_guards, residual, transform


def fixture():
    expected = transform([2., -3., .7, 1.2])
    source = {1: transform([0, 0, 0, .1]), 2: transform([1, 2, 4.3, .5])}
    reference = {'a': {1: transform([-1, 3, 0, .4]), 2: transform([3, 4, 4., .8])}}
    rows = [{'edge_id': str(n), 'session_a': 'a', 'node_a': n, 'node_b': n,
             'T_a_b': (np.linalg.inv(reference['a'][n]) @ expected @ source[n]).tolist(),
             'eligible_for_joint': False} for n in source]
    return expected, source, reference, rows


def test_fit_known_rigid_placement_and_preserve_heights_and_flags():
    expected, source, reference, rows = fixture()
    actual, stats = fit(rows, reference, source)
    assert stats['success']
    np.testing.assert_allclose(actual, expected, atol=1e-7)
    assert all(not r['eligible_for_joint'] for r in rows)
    before = [p[2, 3] for p in source.values()]
    after = [(actual @ p)[2, 3] for p in source.values()]
    np.testing.assert_allclose(np.diff(after), np.diff(before), atol=1e-12)
    assert np.linalg.norm(residual(rows[1], actual, reference, source)) < 1e-7


def test_duplicate_edges_and_no_edges_rejected():
    _, source, reference, rows = fixture()
    with pytest.raises(ValueError):
        fit([rows[0], rows[0]], reference, source)
    with pytest.raises(ValueError):
        fit([], reference, source)


def test_geometry_guard_detects_regression_and_fails_closed():
    rows = [
        {'edge_id': 'bad_rmse', 'role': 'audit_not_used_in_fit',
         'direct_measurement_geometry': {'clipped_rmse_m': .04, 'overlap': .98},
         'placed_geometry': {'clipped_rmse_m': .095, 'overlap': .96}},
        {'edge_id': 'good', 'role': 'training',
         'direct_measurement_geometry': {'clipped_rmse_m': .04, 'overlap': .98},
         'placed_geometry': {'clipped_rmse_m': .05, 'overlap': .96}},
        {'edge_id': 'missing', 'role': 'audit_not_used_in_fit'},
    ]
    checks, failures = geometry_guards(rows)
    assert len(failures) == 2
    assert not checks[0]['passed'] and checks[1]['passed'] and not checks[2]['passed']
    assert checks[0]['clipped_rmse_increase_m'] == pytest.approx(.055)
    assert 'exceeds 0.020m' in failures[0]
    assert 'nonfinite' in failures[1]
