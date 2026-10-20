import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from view_all_sessions_preview import (
    COLORS, FRAME, PREFIX, SESSION_ORDER, STATUS, build_config,
    bundle_file, identity_transform_spec, load_preview, validate_trajectory,
)


def make_bundle(tmp_path, names=SESSION_ORDER):
    manifest = {'status': STATUS, 'unvalidated_preview': True,
                'accepted_by_validation': False, 'navigation_ready': False,
                'frame': FRAME, 'finished_utc': '2026-10-17T00:00:00Z',
                'sessions': {}, 'combined_cloud': 'combined.ply'}
    (tmp_path / 'combined.ply').touch()
    for name in names:
        (tmp_path / f'{name}.ply').touch()
        (tmp_path / f'{name}_trajectory.csv').write_text('id,x,y,z,qx,qy,qz,qw\n1,0,0,0,0,0,0,1\n')
        manifest['sessions'][name] = {'cloud': f'{name}.ply', 'trajectory': f'{name}_trajectory.csv',
                                      'placement': 'unvalidated_hypothesis'}
    path = tmp_path / 'manifest.json'
    path.write_text(json.dumps(manifest))
    return path, manifest


@pytest.mark.parametrize('count', [3, 4, 5])
def test_load_supported_partial_bundles(tmp_path, count):
    path, _ = make_bundle(tmp_path, SESSION_ORDER[:count])
    preview = load_preview(path)
    assert tuple(preview['sessions']) == SESSION_ORDER[:count]
    assert preview['combined_cloud'] == tmp_path / 'combined.ply'


@pytest.mark.parametrize('field,value', [
    ('status', 'FINAL'), ('unvalidated_preview', False),
    ('accepted_by_validation', True), ('navigation_ready', True),
    ('frame', 'map'), ('finished_utc', None),
])
def test_no_accidental_final_map_claim(tmp_path, field, value):
    path, manifest = make_bundle(tmp_path)
    manifest[field] = value
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        load_preview(path)


def test_sessions_need_explicit_placements_and_files(tmp_path):
    path, manifest = make_bundle(tmp_path)
    manifest['sessions']['outside']['placement'] = ''
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='placement'):
        load_preview(path)
    manifest['sessions']['outside']['placement'] = 'hypothesis'
    manifest['sessions']['outside']['cloud'] = 'absent.ply'
    path.write_text(json.dumps(manifest))
    with pytest.raises(FileNotFoundError):
        load_preview(path)


def test_bundle_rejects_absolute_escape_and_symlink(tmp_path):
    bundle = tmp_path / 'bundle'
    bundle.mkdir()
    outside = tmp_path / 'outside.ply'
    outside.touch()
    (bundle / 'link.ply').symlink_to(outside)
    for name in (str(outside), '../outside.ply', 'link.ply'):
        with pytest.raises(ValueError):
            bundle_file(bundle, name)


@pytest.mark.parametrize('row', ['', '1,nan,0,0,0,0,0,1\n', '1,0,0,0,0,0,0,0\n'])
def test_invalid_trajectories_rejected(tmp_path, row):
    path = tmp_path / 'path.csv'
    path.write_text('id,x,y,z,qx,qy,qz,qw\n' + row)
    with pytest.raises(ValueError):
        validate_trajectory(path)


def test_display_defaults_warning_not_covering_map_and_camera_bounds():
    template = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/folded_city.rviz').read_text())
    config = build_config(template, [-10, -5, -2], [10, 5, 8], SESSION_ORDER)
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == FRAME
    clouds = [d for d in manager['Displays'] if d['Class'] == 'rviz_default_plugins/PointCloud2']
    paths = [d for d in manager['Displays'] if d['Class'] == 'rviz_default_plugins/Path']
    warnings = [d for d in manager['Displays'] if d['Class'] == 'rviz_default_plugins/Marker']
    assert [cloud['Enabled'] for cloud in clouds] == [True] + [False] * 5
    assert clouds[0]['Color Transformer'] == 'RGB8'
    assert 'UNVALIDATED' in clouds[0]['Name']
    assert len(set(COLORS.values())) == 5
    assert len(paths) == 5
    assert all(path['Enabled'] and path['Offset'] == {'X': 0, 'Y': 0, 'Z': 0} for path in paths)
    assert not warnings[0]['Enabled']
    assert all(d['Topic']['Value'].startswith(PREFIX + '/') for d in manager['Displays'])
    assert manager['Views']['Current']['Focal Point'] == {'X': 0., 'Y': 0., 'Z': 3.}
    assert template['Visualization Manager']['Global Options']['Fixed Frame'] == 'map'


def test_display_tf_is_isolated_identity_only():
    spec = identity_transform_spec()
    assert spec['parent'] == FRAME
    assert spec['child'] == 'preview_axes'
    assert not {'map', 'odom', 'base_link'} & {spec['parent'], spec['child']}
    np.testing.assert_equal(spec['translation'], [0, 0, 0])
    np.testing.assert_equal(spec['rotation'], [0, 0, 0, 1])


def test_invalid_bounds_rejected():
    with pytest.raises(ValueError, match='bounds'):
        build_config({}, [0, 0, np.nan], [1, 1, 1], SESSION_ORDER)
