import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_config(name):
    return json.loads((ROOT / 'config' / name).read_text())


def test_final_selection_matches_current_map_and_native_display():
    selection = read_config('final_map_selection.json')
    display = read_config('current_local_view.json')
    policy = read_config('structural_display_policy.json')
    sensor_map = read_config('current_map.json')
    assert selection['status'] == display['status'] == policy['status'] == 'USER_SELECTED_FINAL_WORKING_MAP'
    assert selection['accepted_by'] == 'user'
    assert selection['selected_map_manifest'] == sensor_map['manifest']
    assert selection['selected_map_manifest_sha256'] == sensor_map['manifest_sha256'] == display['selected_map_manifest_sha256']
    assert selection['structural_manifest'] == display['structural_manifest']
    assert selection['structural_manifest_sha256'] == display['structural_manifest_sha256'] == policy['structural_manifest_sha256']
    assert selection['session_pose_choices'] == {
        'outside': 'J01', 'lobby': 'N05', 'bridge_out1f': 'H03',
        'floor2': 'J04', 'bridge_12_sub': 'H03',
    }
    for field in ('final_map_selection', 'structural_selection_policy'):
        digest = hashlib.sha256((ROOT / display[field]).read_bytes()).hexdigest()
        assert display[field + '_sha256'] == digest


def test_visual_acceptance_does_not_imply_navigation_or_source_replacement():
    for name in ('final_map_selection.json', 'current_local_view.json', 'structural_display_policy.json'):
        config = read_config(name)
        assert config['candidate_requires_user_review'] is False
        assert config['navigation_ready'] is False
    for name in ('final_map_selection.json', 'current_local_view.json'):
        config = read_config(name)
        assert config['public_release_ready'] is False
        assert config['source_map_replaced'] is False
        assert config['equipment_void_is_walkable'] is False
