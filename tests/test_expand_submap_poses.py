import csv
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from expand_submap_poses import (
    anchor_correction, apply_correction, as_transform, expand_session,
    interpolate_correction, merge_backend_reports, write_pose_csv, POSE_FIELDS,
)


def pose(x, yaw=0.):
    value = np.eye(4)
    value[0, 3] = x
    value[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    return value


def test_additive_translation_matches_backend_and_preserves_anchor():
    initial = pose(100., .8)
    candidate = pose(101., 1.2)
    correction = anchor_correction(initial, candidate)
    np.testing.assert_allclose(apply_correction(correction, initial), candidate, atol=1e-12)
    # A left SE(3) multiplication would incorrectly rotate a 100 m position.
    np.testing.assert_allclose(correction[:3, 3], [1., 0., 0.])
    midpoint = interpolate_correction(pose(0., -.2), pose(2., .4), .5)
    np.testing.assert_allclose(midpoint, pose(1., .1), atol=1e-12)


def setup_session(tmp_path):
    directory = tmp_path / 's'; directory.mkdir()
    frames = [
        {'node_id': i + 1, 'stamp': i, 'session_path_m': i,
         'primary_submap_id': 'a' if i <= 1 else 'b' if i == 2 else 'c',
         'usable_for_submap': True, 'quality_grade': 'C' if i == 4 else 'A'}
        for i in range(5)
    ]
    frames_path = tmp_path / 'frames.csv'
    with frames_path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(frames[0])); writer.writeheader(); writer.writerows(frames)
    submaps = [
        {'submap_id': 'a', 'run_id': 'r1', 'anchor_node_id': 1},
        {'submap_id': 'b', 'run_id': 'r1', 'anchor_node_id': 3},
        {'submap_id': 'c', 'run_id': 'r2', 'anchor_node_id': 4},
    ]
    submaps_path = tmp_path / 'submaps.json'; submaps_path.write_text(json.dumps({'submaps': submaps}))
    write_pose_csv(directory / 'initial.csv', [dict(zip(POSE_FIELDS, [i + 1, i, i, 0, 0, 0, 0, 0, 1])) for i in range(5)])
    graph = {row['submap_id']: {'session_id': 's', 'run_id': row['run_id'],
                              'anchor_node_id': row['anchor_node_id'],
                              'pose': pose(row['anchor_node_id'] - 1).tolist()}
             for row in submaps}
    quality = {'artifacts': {'frames_csv': str(frames_path), 'submaps_json': str(submaps_path)}}
    candidates = {'a': pose(0.), 'b': pose(4.), 'c': pose(13.)}
    return quality, graph, candidates


def test_expansion_does_not_interpolate_across_hard_run_boundary_and_omits_c(tmp_path):
    quality, graph, candidates = setup_session(tmp_path)
    rows, skipped, provenance = expand_session('s', quality, tmp_path, tmp_path, graph, candidates)
    assert [row['id'] for row in rows] == [1, 2, 3, 4]
    assert [row['x'] for row in rows] == [0., 2., 4., 13.]
    assert skipped == [{'node_id': 5, 'reason': 'quality_C_or_no_primary_submap'}]
    assert provenance['run_count'] == 2


def test_expansion_rejects_changed_initial_anchor(tmp_path):
    quality, graph, candidates = setup_session(tmp_path)
    graph['a']['pose'] = pose(7.).tolist()
    with pytest.raises(ValueError, match='differs from frozen common pose'):
        expand_session('s', quality, tmp_path, tmp_path, graph, candidates)


def test_expansion_rejects_tampered_nonanchor_source_file(tmp_path):
    quality, graph, candidates = setup_session(tmp_path)
    paths = [tmp_path / 'frames.csv', tmp_path / 'submaps.json', tmp_path / 's' / 'initial.csv']
    hashes = {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    initial = paths[-1]
    # Alter a nonanchor frame while preserving all anchor values.
    content = initial.read_text().replace('2,1,1,0,0', '2,1,99,0,0')
    initial.write_text(content)
    with pytest.raises(ValueError, match='source artifact differs'):
        expand_session('s', quality, tmp_path, tmp_path, graph, candidates, hashes)


def test_backend_reports_must_match_actual_graph_not_just_each_other():
    graph = {'schema_version': 1, 'name': '로비'}
    digest = hashlib.sha256(json.dumps(graph, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()).hexdigest()
    report = {'provenance': {'canonical_manifest_sha256': digest}, 'variants': {'dcs': {}}}
    assert set(merge_backend_reports(graph, [report])['variants']) == {'dcs'}
    with pytest.raises(ValueError, match='differs from supplied graph'):
        merge_backend_reports({**graph, 'schema_version': 2}, [report, report])
    with pytest.raises(ValueError, match='duplicate backend variant'):
        merge_backend_reports(graph, [report, report])


def test_improper_rotation_rejected():
    reflection = np.eye(4); reflection[0, 0] = -1
    with pytest.raises(ValueError, match=r'SO\(3\)'):
        as_transform(reflection)
