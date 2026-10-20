import copy
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from integrate_floor2_revisit_graph import integrate
from optimize_floor2_frames import node_key


def fixture():
    graph = dict(schema_version=1, metadata={}, submaps=[dict(id=node_key(n),
        session_id='floor2', pose=np.eye(4).tolist(), gauge=n == 1) for n in (1, 2)],
        edges=[dict(id='old', **{'from': node_key(1), 'to': node_key(2)},
            measurement=np.eye(4).tolist(), covariance=np.eye(6).tolist(),
            type='loop', role='heldout', region_group='same_location')])
    row = dict(a=1, b=2, accepted=True, bundle=dict(accepted=True, ba_executed=True),
        measurement_kind='joint_multiframe_RGBD_pose_landmark_BA', train_frames=[[1], [2]],
        test_frames=[[10], [20]], T_a_b=np.eye(4).tolist(), covariance_6x6=np.eye(6).tolist())
    return graph, dict(complete=True, inputs_unchanged=True, records=[row])


def test_replacement_archives_heldout_and_retains_region_without_mutating_input():
    g, r = fixture(); original = copy.deepcopy(g)
    output = integrate(g, r, 2.)
    assert g == original
    assert len(output['edges']) == 1
    edge = output['edges'][0]
    assert edge['role'] == 'train' and edge['region_group'] == 'same_location'
    assert edge['provenance']['former_heldout_now_development']
    np.testing.assert_array_equal(edge['covariance'], np.eye(6)*4)
    assert output['metadata']['archived_replaced_factors'] == original['edges']
    assert output['submaps'] == original['submaps']
    assert output['metadata']['accepted'] is False


def test_refuse_geometry_only_and_leaked_frames():
    g, r = fixture(); r['records'][0]['bundle']['ba_executed'] = False
    with pytest.raises(ValueError, match='geometry-only'): integrate(g, r)
    g, r = fixture(); r['records'][0]['test_frames'] = [[1], [20]]
    with pytest.raises(ValueError, match='split'): integrate(g, r)


def test_refuse_empty_duplicate_and_invalid_uncertainty():
    g, r = fixture(); r['records'][0]['accepted'] = False
    with pytest.raises(ValueError, match='No training'): integrate(g, r)
    g, r = fixture(); r['records'] *= 2
    with pytest.raises(ValueError, match='duplicate'): integrate(g, r)
    for multiplier in (0., .99, float('nan'), float('inf')):
        with pytest.raises(ValueError, match='multiplier'): integrate(g, r, multiplier)


def test_refuse_cross_region_leakage_even_for_rejected_region():
    g, r = fixture()
    other = copy.deepcopy(r['records'][0])
    other.update(accepted=False, test_frames=[[1], [30]])
    r['records'].append(other)
    with pytest.raises(ValueError, match='cross-region'): integrate(g, r)
