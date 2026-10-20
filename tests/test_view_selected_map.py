import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from optimize_session import sha256_file
from view_selected_map import FRAME, PREFIX, load_selected


def bundle(tmp_path):
    combined = tmp_path/'combined.ply'; combined.write_bytes(b'fixture')
    manifest = {'status': 'EXPLORATORY_ALL_SESSION_PREVIEW_NOT_FOR_NAVIGATION',
        'user_designated_working_map': True, 'selection_fixed': True,
        'unvalidated_preview': True, 'accepted_by_validation': False, 'navigation_ready': False,
        'frame': FRAME, 'topic_prefix': PREFIX, 'finished_utc': 'test',
        'combined_cloud': combined.name, 'combined_sha256': sha256_file(combined),
        'sessions': {}, 'selection': {'sessions': {}}}
    for name in ['lobby', 'floor2', 'outside']:
        (tmp_path/(name+'.ply')).touch()
        (tmp_path/(name+'.csv')).write_text('id,x,y,z,qx,qy,qz,qw\n1,0,0,0,0,0,0,1\n')
        manifest['sessions'][name] = {'cloud': name+'.ply', 'trajectory': name+'.csv',
                                      'candidate': 'test', 'placement': 'rigid'}
        manifest['selection']['sessions'][name] = {'candidate': 'test'}
    path = tmp_path/'manifest.json'; path.write_text(json.dumps(manifest))
    return path, manifest


def test_selected_loader_requires_explicit_selection(tmp_path):
    path, manifest = bundle(tmp_path)
    assert len(load_selected(path)['sessions']) == 3
    manifest['user_designated_working_map'] = False; path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='explicitly selected'):
        load_selected(path)


@pytest.mark.parametrize('key,value', [('frame', 'map'), ('topic_prefix', '/folded_city/exploratory_all_preview'),
                                       ('navigation_ready', True), ('accepted_by_validation', True)])
def test_selected_loader_does_not_promote_or_mix_frames(tmp_path, key, value):
    path, manifest = bundle(tmp_path); manifest[key] = value; path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        load_selected(path)


def test_selected_map_hash_is_checked(tmp_path):
    path, _ = bundle(tmp_path); (tmp_path/'combined.ply').write_bytes(b'changed')
    with pytest.raises(ValueError, match='hash differs'):
        load_selected(path)


def test_selected_candidate_cannot_silently_change(tmp_path):
    path, manifest = bundle(tmp_path); manifest['sessions']['floor2']['candidate'] = 'different'
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='Candidate selection differs'):
        load_selected(path)
