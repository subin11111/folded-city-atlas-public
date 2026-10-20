"""Independent RViz configs without running ROS publishers or a graphical UI."""
import json
import math
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import view_session


def make_session(run, session):
    root = run / session
    root.mkdir(parents=True)
    for name in ('cloud_raw_filtered.ply', 'cloud_optimized_filtered.ply', 'cloud_app_filtered.ply',
                 'raw_main_trajectory.csv', 'optimized_main_trajectory.csv'):
        (root / name).write_text('placeholder\n')
    reference = run / 'audit' / session
    reference.mkdir(parents=True)
    (reference / 'app_optimized_trajectory.csv').write_text('placeholder\n')
    (root / 'export_optimized.json').write_text(json.dumps({
        'cloud_bounds_min_m': [-1., -2., 0.], 'cloud_bounds_max_m': [3., 4., 6.],
    }))
    return root


def forbid_external_processes(monkeypatch):
    def unexpected(*args, **kwargs):
        pytest.fail('Config-only mode must not start any external process')
    monkeypatch.setattr(view_session.os, 'execv', unexpected)
    monkeypatch.setattr(view_session.subprocess, 'Popen', unexpected)
    monkeypatch.setattr(view_session.subprocess, 'run', unexpected)


@pytest.mark.parametrize('session', ['outside', 'lobby', 'bridge_out1f', 'floor2', 'bridge_12_sub'])
def test_config_only_keeps_independent_frame_topics_and_camera(tmp_path, monkeypatch, session):
    root = make_session(tmp_path, session)
    forbid_external_processes(monkeypatch)
    view_session.main([session, '--run', str(tmp_path), '--config-only'])
    config = yaml.safe_load((root / 'comparison.rviz').read_text())
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == session + '_map'
    displays = manager['Displays']
    topic_values = {display['Topic']['Value'] for display in displays if isinstance(display.get('Topic'), dict)}
    expected_suffixes = {'cloud_raw', 'cloud_optimized', 'cloud_app',
                         'trajectory_raw', 'trajectory_optimized', 'trajectory_app'}
    assert topic_values == {'/folded_city/' + session + '/' + suffix for suffix in expected_suffixes}
    assert not any(display.get('Class') == 'rviz_default_plugins/TF' for display in displays)
    assert not any(display.get('Name') in ('Optimized No-Ground', 'Optimized Ground') for display in displays)
    paths = [display for display in displays if display.get('Class') == 'rviz_default_plugins/Path']
    assert len(paths) == 3
    assert all(display['Offset'] == {'X': 0, 'Y': 0, 'Z': 0} for display in paths)
    app_cloud = next(display for display in displays if display['Name'] == 'App Saved Cloud')
    assert not app_cloud['Enabled'] and not app_cloud['Value']
    camera = manager['Views']['Current']
    assert camera['Focal Point'] == {'X': 1., 'Y': 1., 'Z': 3.}
    assert camera['Distance'] == pytest.approx(1.6 * math.sqrt(88))
    assert not (root / 'rviz_publisher.log').exists()


@pytest.mark.parametrize('session', ['../outside', 'a/b', '/tmp/outside', '.', '..',
                                      '2floor', '_floor2', 'Floor2', 'floor-2', 'floor 2', '', 'a\nb'])
def test_unsafe_session_identifiers_rejected_before_file_access(tmp_path, monkeypatch, session):
    forbid_external_processes(monkeypatch)
    with pytest.raises(SystemExit) as error:
        view_session.main([session, '--run', str(tmp_path), '--config-only'])
    assert error.value.code == 2
    assert list(tmp_path.iterdir()) == []


def test_config_only_cannot_be_combined_with_publisher_only(tmp_path, monkeypatch):
    forbid_external_processes(monkeypatch)
    with pytest.raises(SystemExit) as error:
        view_session.main(['floor2', '--run', str(tmp_path), '--config-only', '--publisher-only'])
    assert error.value.code == 2
    assert list(tmp_path.iterdir()) == []
