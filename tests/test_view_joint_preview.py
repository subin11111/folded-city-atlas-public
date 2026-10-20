import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from view_joint_preview import FRAME, PREFIX, build_config, cloud_bounds, load_preview


def make_preview(tmp_path):
    target = tmp_path / 'preview_clouds'
    target.mkdir()
    manifest = {'unvalidated_preview': True, 'accepted_by_validation': False,
                'navigation_ready': False, 'component_sessions': ['floor2', 'lobby'],
                'finished_utc': '2026-10-17T00:00:00Z'}
    (target / 'export_manifest.json').write_text(json.dumps(manifest))
    (tmp_path / 'joint_report.json').write_text(json.dumps({'accepted': False, 'sessions': ['floor2', 'lobby']}))
    (target / 'combined_candidate.ply').touch()
    for name in ('lobby', 'floor2'):
        (tmp_path / name).mkdir()
        (tmp_path / name / 'candidate_trajectory.csv').touch()
        (target / f'{name}_candidate.ply').touch()
    return target, manifest


def test_preview_requires_explicit_warning_and_original_rejection(tmp_path):
    target, manifest = make_preview(tmp_path)
    assert load_preview(tmp_path)['target'] == target
    del manifest['unvalidated_preview']
    (target / 'export_manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='unvalidated_preview'):
        load_preview(tmp_path)
    manifest['unvalidated_preview'] = True
    (target / 'export_manifest.json').write_text(json.dumps(manifest))
    (tmp_path / 'joint_report.json').write_text(json.dumps({'accepted': True, 'sessions': ['floor2', 'lobby']}))
    with pytest.raises(ValueError, match='accepted=false'):
        load_preview(tmp_path)


def test_preview_excludes_disconnected_sessions(tmp_path):
    target, manifest = make_preview(tmp_path)
    manifest['component_sessions'].append('outside')
    (target / 'export_manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='disconnected'):
        load_preview(tmp_path)


def test_display_defaults_no_visual_z_offsets():
    template = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/folded_city.rviz').read_text())
    config = build_config(template, np.array([-10., -5., -2.]), np.array([10., 5., 8.]))
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == FRAME
    displays = manager['Displays']
    clouds = [d for d in displays if d['Class'] == 'rviz_default_plugins/PointCloud2']
    assert [d['Enabled'] for d in clouds] == [True, False, False]
    assert clouds[0]['Color Transformer'] == 'RGB8'
    assert all(d['Topic']['Value'].startswith(PREFIX + '/') for d in displays)
    paths = [d for d in displays if d['Class'] == 'rviz_default_plugins/Path']
    assert len(paths) == 2
    assert all(p['Enabled'] and p['Offset'] == {'X': 0, 'Y': 0, 'Z': 0} for p in paths)
    assert manager['Views']['Current']['Focal Point'] == {'X': 0., 'Y': 0., 'Z': 3.}
    assert template['Visualization Manager']['Global Options']['Fixed Frame'] == 'map'


def test_exact_cloud_bounds_and_truncation(tmp_path):
    path = tmp_path / 'cloud.ply'
    header = ('ply\nformat binary_little_endian 1.0\nelement vertex 2\n'
              'property float x\nproperty float y\nproperty float z\n'
              'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode()
    array = np.array([(1., 4., -2., 10, 20, 30), (-3., 6., 7., 40, 50, 60)],
                     dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    path.write_bytes(header + array.tobytes())
    low, high = cloud_bounds(path)
    np.testing.assert_equal(low, [-3., 4., -2.])
    np.testing.assert_equal(high, [1., 6., 7.])
    path.write_bytes(header + array[:1].tobytes())
    with pytest.raises(ValueError, match='Truncated'):
        cloud_bounds(path)
