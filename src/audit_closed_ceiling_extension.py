#!/usr/bin/env python3
"""Check real reused floor skins, added cavity skins and immutable map inputs."""
import argparse
import json
from pathlib import Path

import numpy as np

from audit_structural_delivery import glb_triangle_positions
from audit_unified_presentation import embedded_images
from complete_service_slabs import closed_slab_arrays, mask_lookup
from extend_closed_ceiling_bodies import complete_floor_cells
from optimize_session import sha256_file, utc_now

ROOT = Path(__file__).resolve().parents[1]


def welded_edge_counts(triangles, tolerance=1e-5):
    """Weld float32 export noise, not centimetre-scale physical gaps."""
    quantized = np.rint(np.asarray(triangles).reshape(-1, 3)/tolerance).astype(np.int64)
    _, inverse = np.unique(quantized, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    if np.any(np.diff(np.sort(faces, axis=1), axis=1) == 0):
        raise ValueError('Degenerate skin triangle')
    edges = np.sort(np.r_[faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]], axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return int((counts == 1).sum()), int((counts > 2).sum())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--full-history-hashes', action='store_true', help='Also reread all historical intermediate bundles, not only direct inputs.')
    a = p.parse_args()
    run = json.loads((a.bundle/'manifest.json').read_text())
    base = Path(run['closure_source_manifest']).parent
    cfg = run['ceiling_expansion_and_closure']['config']
    q = np.load(a.bundle/'closure_plan.npz')
    low, cell, body = q['low'], float(q['cell']), q['body']
    assert not (body & (q['void'] | q['stair'] | ~q['actual_top'])).any()
    assert cfg['navigation_export'] is False and cfg['service_void_is_not_walkable'] is True
    assert cfg['upper_floor_m'] > cfg['lower_extension_height_m']
    vertices, parts = closed_slab_arrays(body, low, cell, cfg['lower_extension_height_m'], cfg['upper_floor_m'])
    expected_bottom, expected_top, expected_sides = [vertices[f].astype(np.float32) for f in parts]
    actual_tops, actual_skins, sessions = [], [], {}
    import open3d as o3d
    for name, spec in run['sessions'].items():
        old = glb_triangle_positions(base/name/'clean_textured.glb')
        xyz = glb_triangle_positions(a.bundle/name/'clean_textured.glb')
        mesh = o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'))
        assert np.array_equal(xyz, np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32))
        keep = np.load(a.bundle/name/'closure_retained_faces.npy')
        count = spec['closure_retained_triangles']
        assert np.array_equal(xyz[:count], old[keep]), 'Retained XYZ changed: '+name
        old_floors = (np.max(abs(old[:, :, 2]-2.60), axis=1) < 2e-5) | (np.max(abs(old[:, :, 2]+1.55), axis=1) < 2e-5)
        assert keep[old_floors].all(), 'Floor removed: '+name
        images = embedded_images(base/name/'clean_textured.glb')
        assert embedded_images(a.bundle/name/'clean_textured.glb')[:len(images)] == images
        _, top_ids = complete_floor_cells(xyz[:count], low, body.shape, cell, cfg['upper_floor_m'])
        if name in ['lobby', 'floor2', 'bridge_12_sub']:
            top = xyz[top_ids]
            actual_tops.append(top[mask_lookup(body, low, cell, top.mean(1)[:, :2])])
        new = xyz[count:]
        cursor = 0
        for part in spec['closure_appended']:
            assert part['walkable'] is False and part['navigation_exported'] is False
            faces = new[cursor:cursor+part['triangles']]
            if part['part'] == 'underside':
                assert np.array_equal(faces, expected_bottom)
                actual_skins.append(faces)
            elif part['part'] == 'perimeter_and_void_sides':
                assert np.array_equal(faces, expected_sides)
                actual_skins.append(faces)
            elif part['part'] == 'expanded_upper_roof':
                assert np.max(abs(faces[:, :, 2]-float(q['roof_height']))) < 2e-5
            else:
                raise ValueError('Unknown appended geometry role')
            cursor += len(faces)
        assert cursor == len(new)
        sessions[name] = {'retained_XYZ_exact': True, 'all_source_floor_triangles_retained': int(old_floors.sum()),
                          'source_embedded_images_exact': len(images), 'actual_GLB_PLY_equal': True,
                          'new_triangles': len(new), 'removed_coplanar_ceiling_triangles': int((~keep).sum())}
        print('Verified floor and cavity skins: '+name, flush=True)
    # Deduplicate overlapping real source sessions, not an invented analytic top.
    tops = np.concatenate(actual_tops)
    key = np.rint((tops[:, :, :2]-low)/cell).astype(int)
    key = np.sort(key[:, :, 0]+key[:, :, 1]*(body.shape[1]+1), axis=1)
    _, ids = np.unique(key, axis=0, return_index=True)
    tops = tops[ids]
    assert len(tops) == len(expected_top), 'Real floor does not cover the complete top skin'
    actual_body = np.concatenate([tops, *actual_skins])
    boundary, nonmanifold = welded_edge_counts(actual_body)
    assert boundary == 0 and nonmanifold == 0, 'Actual reused-floor cavity is not closed'
    signed_volume = abs(np.einsum('ij,ij->i', actual_body[:, 0], np.cross(actual_body[:, 1], actual_body[:, 2])).sum()/6)
    expected_volume = body.sum()*cell**2*(cfg['upper_floor_m']-cfg['lower_extension_height_m'])
    assert abs(signed_volume-expected_volume) < .01
    # From below every accepted body cell, a real rendered cap must be hit at
    # the underside, not at the upper-floor plane. Also check inside the void.
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(actual_body.reshape(-1, 3)),
                         o3d.core.Tensor(np.arange(actual_body.size//3).reshape(-1, 3).astype(np.uint32))))
    y, x = np.where(body)
    xy = low+np.c_[x+.37, y+.41]*cell
    bottom, top = cfg['lower_extension_height_m'], cfg['upper_floor_m']
    rays = np.c_[xy, np.full(len(xy), bottom-.1), np.zeros((len(xy), 2)), np.ones(len(xy))].astype(np.float32)
    hit = scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()
    assert np.isfinite(hit).all() and abs(hit-.1).max() < 2e-5
    points = np.c_[xy, np.full(len(xy), (bottom+top)/2)].astype(np.float32)
    maximum = 0.
    for direction in np.r_[np.eye(3), -np.eye(3)]:
        hit = scene.cast_rays(o3d.core.Tensor(np.c_[points, np.broadcast_to(direction, points.shape)].astype(np.float32)))['t_hit'].numpy()
        assert np.isfinite(hit).all(), 'Service void leaks in a cardinal direction'
        maximum = max(maximum, float(hit.max()))
    selected_pointer = json.loads((ROOT/'config/current_map.json').read_text())
    selected_path = ROOT/selected_pointer['manifest']
    assert sha256_file(selected_path) == run['selected_manifest_sha256']
    selected = json.loads(selected_path.read_text())
    database_hashes = {}
    for name, spec in selected['database_integrity'].items():
        database_hashes[name] = sha256_file(Path(spec['database']))
        assert database_hashes[name] == spec['sha256']
        print('Verified unchanged source DB: '+name, flush=True)
    checked, deferred = [], []
    direct_files = {str((ROOT/v).resolve()) for v in ['src/extend_closed_ceiling_bodies.py', 'config/ceiling_expansion_and_closure.json',
                    'src/match_ceiling_extension_materials.py', 'config/ceiling_material_continuity.json']}
    direct_folders = {base.resolve()} | {(base/name).resolve() for name in run['sessions']}
    for path, digest in run['inputs_sha256'].items():
        file = Path(path)
        if a.full_history_hashes or path in direct_files or file.parent.resolve() in direct_folders:
            assert sha256_file(file) == digest, 'Source binding changed: '+path
            checked.append(path)
        else:
            deferred.append(path)
    result = {'created_utc': utc_now(), 'bundle_manifest_sha256': sha256_file(a.bundle/'manifest.json'),
              'audit_source_sha256': sha256_file(Path(__file__)), 'sessions': sessions,
              'actual_reused_floor_and_appended_skins_closed': True, 'boundary_edges': boundary,
              'nonmanifold_edges': nonmanifold, 'float32_weld_tolerance_m': 1e-5,
              'actual_closed_cavity_volume_m3': float(signed_volume), 'expected_cavity_volume_m3': float(expected_volume),
              'underside_cell_rays_passed': len(xy), 'inside_six_direction_cell_rays_passed': len(xy)*6,
              'largest_inside_cardinal_ray_m': maximum, 'floor_ceiling_height_not_merged': True,
              'added_geometry_is_walkable': False, 'navigation_exported': False,
              'lower_cap_inside_atrium_cells': 0, 'lower_cap_inside_stair_guard_cells': 0,
              'selected_map_unchanged': True, 'source_databases_unchanged': database_hashes,
              'checked_input_bindings_valid': True, 'freshly_checked_inputs': checked,
              'historical_input_hashes_carried_forward_not_reread': deferred,
              'full_history_hash_scan': a.full_history_hashes,
              'scope': 'Closed display equipment cavities only, not surveyed slab dimensions, full-building watertightness or navigation approval.'}
    a.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
