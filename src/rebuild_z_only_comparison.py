#!/usr/bin/env python3
"""Re-integrate identical original RGB-D with J04 and scalar-Z-only poses.

Reconstruction is independent for each candidate; no old mesh is warped. Every
per-frame depth sample has exactly identical XY/RGB before/after. TSDF extraction
can change iso-surface vertices through voxel fusion; it is NOT an XY pose edit.
"""
import argparse
import gc
import hashlib
import json
from pathlib import Path
import sqlite3
import time

import cv2
import numpy as np

from export_rtabmap_db import load_pose_csv, write_binary_ply
from optimize_session import apply, inverse, sha256_file, utc_now
from reconstruct_surface_portfolio import clean_mesh
from repair_surface_fragmentation import export_rgb, prune, topology
from surface_model_utils import decode_frame, unproject

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--poses', type=Path, required=True)
    p.add_argument('--variants', nargs='+', default=['Z02'])
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    import open3d as o3d
    cv2.setNumThreads(2)
    report_path = args.poses/'report.json'; report = json.loads(report_path.read_text())
    if any(not report['variants'][c]['guards_passed'] for c in args.variants):
        raise ValueError('Do not reconstruct a failed engineering-guard candidate as selected')
    mp = ROOT/report['config']['source_manifest']; selected = json.loads(mp.read_text())
    session = selected['sessions']['floor2']; db = Path(selected['database_integrity']['floor2']['database'])
    base_path = mp.parent/session['trajectory']
    pp = {'J04': load_pose_csv(base_path)}
    pp.update({c: load_pose_csv(args.poses/(c+'.csv')) for c in args.variants})
    paths = [Path(__file__), report_path, mp, base_path, db, ROOT/'src/surface_model_utils.py',
        ROOT/'src/reconstruct_surface_portfolio.py', ROOT/'src/repair_surface_fragmentation.py',
        ROOT/'src/session_preprocessing.py', *[args.poses/(c+'.csv') for c in args.variants]]
    hashes = {str(p.resolve()): sha256_file(p) for p in paths}
    ids = sorted(pp['J04'])
    assert all(set(poses) == set(ids) for poses in pp.values())
    for code, poses in pp.items():
        assert all(np.array_equal(poses[n][:3, :3], pp['J04'][n][:3, :3]) and
                   np.array_equal(poses[n][:2, 3], pp['J04'][n][:2, 3]) for n in ids)
    volumes = {c: o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=.05,
        sdf_trunc=.15, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8) for c in pp}
    preview = {c: [] for c in pp}; colors = []; nodes = []
    input_xy_hash = hashlib.sha256(); valid_count = 0; records = []; started = time.monotonic()
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    for i, n in enumerate(ids):
        f = decode_frame(con, n, session['preprocessing'], pp['J04'][n])
        depth = f['depth']; h, w = depth.shape; k = f['k_depth']
        color = cv2.resize(f['rgb'], (w, h), interpolation=cv2.INTER_AREA)
        optical = unproject(depth, k); valid = depth > 0
        xyz = apply(optical[valid], f['world_optical'])
        valid_count += len(xyz); input_xy_hash.update(np.ascontiguousarray(xyz[:, :2]).tobytes())
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.ascontiguousarray(color)),
            o3d.geometry.Image(depth), depth_scale=1., depth_trunc=session['preprocessing']['max_depth']+.001,
            convert_rgb_to_intensity=False)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, k[0, 0], k[1, 1], k[0, 2], k[1, 2])
        sample = valid.copy(); yy, xx = np.indices(depth.shape); sample &= (xx%4 == 0)&(yy%4 == 0)
        source_sample = apply(optical[sample], f['world_optical'])
        colors.append(color[sample]); nodes.append(np.full(sample.sum(), n, np.int32))
        for code in pp:
            dz = float(pp[code][n][2, 3]-pp['J04'][n][2, 3])
            pose = f['world_optical'].copy(); pose[2, 3] += dz
            # Actual reprojected point coordinates, not a changed calibration.
            check = apply(optical[valid], pose)
            assert np.array_equal(check[:, :2], xyz[:, :2])
            assert np.allclose(check[:, 2]-xyz[:, 2], dz, atol=1e-12, rtol=0.)
            volumes[code].integrate(rgbd, intrinsic, inverse(pose))
            pts = source_sample.copy(); pts[:, 2] += dz; preview[code].append(pts.astype(np.float32))
        records.append({'node': n, 'valid_depth_samples': len(xyz), 'xy_exact_all_variants': True})
        if i%75 == 0:
            print(json.dumps({'depth_frame': i+1, 'total': len(ids), 'variants': list(pp),
                'seconds': round(time.monotonic()-started, 1)}), flush=True)
    con.close(); rgb = np.concatenate(colors); index = np.concatenate(nodes); models = {}
    np.save(args.output/'preview_frame_ids.npy', index)
    points = {}
    for code, volume in volumes.items():
        dest = args.output/code; dest.mkdir()
        points[code] = np.concatenate(preview[code]); preview[code].clear()
        assert np.array_equal(points[code][:, :2], points['J04'][:, :2])
        write_binary_ply(dest/'depth_preview.ply', points[code], rgb)
        mesh = clean_mesh(volume.extract_triangle_mesh())
        mesh, cleanup = prune(mesh, .005)
        o3d.io.write_triangle_mesh(str(dest/'tsdf_5cm.ply'), mesh)
        export_rgb(mesh, dest/'tsdf_rgb.glb')
        models[code] = {'topology': topology(mesh), 'micro_component_pruning': cleanup,
            'preview_points': len(points[code]), 'preview_xy_equals_J04_exact': True,
            'preview_xy_sha256': hashlib.sha256(np.ascontiguousarray(points[code][:, :2]).tobytes()).hexdigest(),
            'pose_csv': str(base_path if code == 'J04' else args.poses/(code+'.csv')),
            'outputs': {p.name: {'sha256': sha256_file(p), 'bytes': p.stat().st_size} for p in dest.iterdir()}}
        print(json.dumps({'reconstructed': code, 'triangles': len(mesh.triangles), 'preview_points': len(points[code])}), flush=True)
        del mesh; gc.collect()
    assert all(sha256_file(Path(p)) == h for p, h in hashes.items())
    manifest = {'created_utc': utc_now(), 'inputs_sha256': hashes, 'all_inputs_unchanged': True,
        'source_selected_map_manifest_sha256': sha256_file(mp), 'source_selected_map_replaced': False,
        'session': 'floor2', 'variants': models, 'valid_depth_samples_per_variant': valid_count,
        'frames': len(ids), 'frame_checks': records, 'original_sample_xy_sha256': input_xy_hash.hexdigest(),
        'preview_sampling': 'Deterministic every fourth depth row/column, no voxel resampling; identical points/order/RGB and XY in every variant.',
        'reconstruction': {'voxel_m': .05, 'truncation_m': .15, 'all_frames': True, 'all_valid_depth_pixels': True,
            'depth_preprocessing_changed': False, 'hole_filling': False, 'decimation': False,
            'tsdf_xy_vertex_note': 'Different Z changes voxel fusion/iso-surface extraction. Mesh vertex arrays need not share XY; calibrated per-frame XY and pose XY/rotations remain exactly fixed.'},
        'other_four_sessions_changed': False, 'navigation_ready': False,
        'limitations': ['Only scalar pose Z was estimated; no independent surveyed validation.',
            'This is matched raw-TSDF comparison, not the earlier flattened/photo-semantic display.',
            'Remaining tilted planes/XY disagreement cannot be corrected by Z offsets.'],
        'seconds': time.monotonic()-started}
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({'complete': str(args.output), 'seconds': manifest['seconds'], 'xy_exact': True}), flush=True)


if __name__ == '__main__': main()
