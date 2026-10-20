#!/usr/bin/env python3
"""Audit delivered scalar-Z poses, paired sample XY, meshes and depth probes."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import sqlite3

import numpy as np

from audit_display_depth import compare_depths, depth_image
from audit_structural_delivery import glb_triangle_positions
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file, utc_now
from surface_model_utils import decode_frame

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--poses', type=Path, required=True); p.add_argument('--rebuild', type=Path, required=True)
    p.add_argument('--variant', default='Z02'); p.add_argument('--native', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    import open3d as o3d
    r = json.loads((args.poses/'report.json').read_text())
    m = json.loads((args.rebuild/'manifest.json').read_text())
    mp = ROOT/r['config']['source_manifest']; selected = json.loads(mp.read_text())
    for manifest, key in [(r, 'inputs_sha256'), (m, 'inputs_sha256')]:
        for file, digest in manifest[key].items():
            assert sha256_file(Path(file)) == digest, file
    base = mp.parent/selected['sessions']['floor2']['trajectory']; after = args.poses/(args.variant+'.csv')
    b, a = load_pose_csv(base), load_pose_csv(after); ids = sorted(b)
    with base.open(newline='') as f: old_rows = list(csv.DictReader(f))
    with after.open(newline='') as f: new_rows = list(csv.DictReader(f))
    assert all(all(x[k] == y[k] for k in x if k != 'z') for x, y in zip(old_rows, new_rows))
    assert all(np.array_equal(a[n][:3, :3], b[n][:3, :3]) and np.array_equal(a[n][:2, 3], b[n][:2, 3]) for n in ids)
    protected = json.loads((ROOT/'outputs/student_union_photo_semantics_v10/manifest.json').read_text())['sessions']['floor2']['stair_guard']['camera_nodes']
    assert all(np.array_equal(a[n], b[n]) for n in protected if n in a)
    endpoints = set()
    for e in selected['connection_evaluations']:
        for suffix in ['a', 'b']:
            if e['session_'+suffix] == 'floor2': endpoints.add(e['node_'+suffix])
    assert all(np.array_equal(a[n], b[n]) for n in endpoints)
    previews = {c: o3d.io.read_point_cloud(str(args.rebuild/c/'depth_preview.ply')) for c in ['J04', args.variant]}
    old, new = (np.asarray(previews[c].points) for c in ['J04', args.variant])
    assert np.array_equal(old[:, :2], new[:, :2])
    assert np.array_equal(np.asarray(previews['J04'].colors), np.asarray(previews[args.variant].colors))
    nodes = np.load(args.rebuild/'preview_frame_ids.npy')
    expected = np.array([a[int(n)][2, 3]-b[int(n)][2, 3] for n in nodes])
    error = abs((new[:, 2]-old[:, 2])-expected)
    assert error.max() < 1e-6
    meshes, geometry = {}, {}
    for c in ['J04', args.variant]:
        mesh = o3d.io.read_triangle_mesh(str(args.rebuild/c/'tsdf_5cm.ply'))
        xyz = np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32)
        assert np.array_equal(xyz, glb_triangle_positions(args.rebuild/c/'tsdf_rgb.glb'))
        meshes[c] = mesh
        geometry[c] = {'triangles': len(xyz), 'actual_GLB_PLY_coordinates_equal': True,
            'ply_sha256': sha256_file(args.rebuild/c/'tsdf_5cm.ply'), 'glb_sha256': sha256_file(args.rebuild/c/'tsdf_rgb.glb')}
    db = Path(selected['database_integrity']['floor2']['database'])
    checks = {}
    with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as con:
        for n in [1, 280, 282, 553, 620, 642, 726, 728, 828, 1031, 1078, 1218]:
            if n not in a: continue
            f = decode_frame(con, n, selected['sessions']['floor2']['preprocessing'], b[n])
            g = dict(f, world_optical=f['world_optical'].copy()); g['world_optical'][2, 3] += a[n][2, 3]-b[n][2, 3]
            before = depth_image(o3d, meshes['J04'], f); after_depth = depth_image(o3d, meshes[args.variant], g)
            checks[str(n)] = compare_depths(before, after_depth, f['depth'])
            checks[str(n)]['scope'] = 'Each reconstruction uses its own calibrated candidate camera; same original depth pixels. Reused integrated data, NOT independent pose accuracy.'
            print('Audited original depth view '+str(n), flush=True)
    native = None
    if args.native:
        native = json.loads((args.native/'native_loaded.json').read_text())
        assert native['variant'] == args.variant and native['xy_rotations_unchanged']
        assert len(native['files']) == 5
        for record in native['files'].values(): assert sha256_file(Path(record['path'])) == record['sha256']
        image = args.native/('01_'+args.variant+'.png')
        assert image.is_file(); args.output.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(image, args.output.parent/'native_z_only.png')
        native['scene_render_sha256'] = sha256_file(image)
        native['scene_render_scope'] = 'Actual native scene-render callback; not a desktop screenshot.'
    db_integrity = {}
    for name, record in selected['database_integrity'].items():
        digest = sha256_file(Path(record['database']))
        assert digest == record['sha256']
        db_integrity[name] = {'database_sha256_matches_selected': True}
    result = {'created_utc': utc_now(), 'candidate': args.variant,
        'all_non_z_csv_fields_exact': True, 'pose_xy_and_all_rotations_exact': True,
        'reviewed_stair_pose_count': len(set(protected)&set(ids)), 'reviewed_stair_pose_movement_m': 0.,
        'all_current_cross_session_endpoint_poses_exact': sorted(endpoints),
        'paired_depth_preview_points': len(old), 'paired_preview_xy_rgb_order_exact': True,
        'maximum_point_delta_z_vs_pose_delta_error_m': float(error.max()),
        'all_valid_original_samples_xy_exact': m['valid_depth_samples_per_variant'],
        'input_bindings_still_valid': True, 'database_integrity': db_integrity,
        'geometry': geometry, 'floor_before': r['baseline_floor'], 'floor_after': r['variants'][args.variant]['floor'],
        'depth_checks': checks, 'native': native,
        'selected_map_manifest_sha256': sha256_file(mp), 'source_selected_map_replaced': False,
        'optimizer_report_sha256': sha256_file(args.poses/'report.json'),
        'rebuild_manifest_sha256': sha256_file(args.rebuild/'manifest.json'),
        'audit_source_sha256': sha256_file(Path(__file__)), 'navigation_ready': False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print('Z-only delivery verified: '+str(args.output), flush=True)


if __name__ == '__main__': main()
