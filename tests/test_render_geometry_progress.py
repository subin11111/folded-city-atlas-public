import hashlib
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from render_geometry_progress import capture_input_provenance, finalize_manifest, render_height_profiles, variant_title


def test_selected_variant_title_does_not_claim_unapplied_plane_factors():
    assert 'finite plane factors' in variant_title('pgo6_planes')
    assert 'normal factors' in variant_title('pgo6_normals')
    assert 'plane' not in variant_title('pgo6_attitude')
    assert variant_title('pgo4') != '4DoF robust graph'  # Preserve both dictionary panels.


def test_height_plot_is_indexed_and_input_provenance_detects_database_changes(tmp_path):
    db = tmp_path/'input.db'
    db.write_bytes(b'synthetic immutable input')
    config_path = tmp_path/'config.json'
    config_path.write_text('{}')
    pose_path = tmp_path/'poses.csv'
    pose_path.write_text('synthetic input poses')
    config = {'floor2': {'database': str(db)}}
    manifest = capture_input_provenance(config_path, config, {'candidate': {'floor2': pose_path}})
    assert manifest['source_hashes'][str(config_path)] == hashlib.sha256(b'{}').hexdigest()
    assert str(pose_path) in manifest['source_hashes']
    assert any(p.endswith('/render_geometry_progress.py') for p in manifest['source_hashes'])
    assert any(p.endswith('/check_session_overlap.py') for p in manifest['source_hashes'])
    poses = {'synthetic candidate': {n: {1: np.eye(4), 2: np.eye(4)} for n in ('floor2', 'bridge_12_sub')}}
    sessions = {n: SimpleNamespace(stamps={1: 10., 2: 11.}) for n in ('floor2', 'bridge_12_sub')}
    filename = '04_height_profiles.png'
    manifest['figures'] = {filename: render_height_profiles(poses, sessions, tmp_path/filename)}
    assert manifest['figures'][filename]['common_frame_counts'] == {'floor2': 2, 'bridge_12_sub': 2}
    assert not manifest['figures'][filename]['surveyed_floor_height']
    finalize_manifest(manifest, config, tmp_path)
    assert manifest['originals_unchanged']
    assert set(manifest['figure_hashes']) == {filename}
    db.write_bytes(b'changed input')
    with pytest.raises(ValueError, match='Source DB hashes changed'):
        finalize_manifest(manifest, config, tmp_path)


def test_unindexed_figure_cannot_silently_disappear_from_manifest(tmp_path):
    (tmp_path/'unindexed.png').write_bytes(b'not rendered by this test')
    with pytest.raises(ValueError, match='inventory differs'):
        finalize_manifest({'figures': {}, 'database_hashes_before': {}}, {}, tmp_path)
