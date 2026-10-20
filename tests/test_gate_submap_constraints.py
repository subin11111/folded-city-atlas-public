import copy
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from gate_submap_constraints import dcs_information_weight, gate_manifest, main


def manifest():
    def measurement(x):
        return [x, 0, 0, 0, 0, 0, 1]
    return {
        'schema_version': 1,
        'submaps': [{'id': 'a', 'session_id': 's', 'pose': measurement(0)},
                    {'id': 'b', 'session_id': 's', 'pose': measurement(1)}],
        'edges': [{'id': name, 'from': 'a', 'to': 'b', 'measurement': measurement(dx),
                   'information': (np.eye(6) * 100).tolist(), 'type': kind, 'role': role,
                   'region_group': name}
                  for name, dx, kind, role in [
                      ('odom', 9, 'odometry', 'train'),
                      ('transition', 9, 'structural', 'train'),
                      ('good', 1, 'loop', 'train'),
                      ('bad', 9, 'cross_session', 'train'),
                      ('audit', 9, 'loop', 'heldout')]],
    }


def test_gate_only_freezes_initial_loop_selection_and_labels_bias():
    source = manifest(); before = copy.deepcopy(source)
    result = gate_manifest(source)
    assert source == before
    by_id = {edge['id']: edge for edge in result['edges']}
    assert all(by_id[key]['role'] == 'train' for key in ('odom', 'transition', 'good'))
    assert by_id['bad']['role'] == 'heldout'
    assert by_id['bad']['provenance']['selection_conditioned_diagnostic']
    assert not by_id['bad']['provenance']['independent_holdout']
    assert 'provenance' not in by_id['audit']
    diagnostic = result['metadata']['frozen_initial_dcs_gate']
    assert diagnostic['gated_edge_ids'] == ['bad']
    assert not result['metadata']['accepted']


@pytest.mark.parametrize('chi2,phi', [(float('nan'), 1), (-1, 1), (1, float('inf')), (1, 0)])
def test_gate_rejects_nonphysical_weight_inputs(chi2, phi):
    with pytest.raises(ValueError):
        dcs_information_weight(chi2, phi)


def test_gate_refuses_existing_output(tmp_path):
    with pytest.raises(FileExistsError):
        main(['--manifest', str(tmp_path / 'unused.json'), '--output', str(tmp_path)])
