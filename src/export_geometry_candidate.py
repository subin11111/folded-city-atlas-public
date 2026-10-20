#!/usr/bin/env python3
"""Rebuild all RGB-D frames of an explicitly experimental geometry candidate."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from export_rtabmap_db import voxel_reduce, write_binary_ply
from optimize_session import sha256_file, utc_now
from render_session_report import read_ply


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--graph', type=Path, required=True)
    parser.add_argument('--variant', default='pgo6_planes')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = json.loads((args.graph/'report.json').read_text())
    if args.variant not in report['variants']:
        raise ValueError('Unknown completed graph variant')
    if args.output.exists():
        raise FileExistsError('Choose fresh output; existing clouds are never replaced')
    args.output.mkdir(parents=True)
    config = json.loads(Path('config/student_union_all_sessions.json').read_text())['sessions']
    manifest = {'started_utc': utc_now(), 'argv': sys.argv,
        'status': 'EXPERIMENTAL_GEOMETRY_CANDIDATE_NOT_FOR_NAVIGATION',
        'accepted_by_validation': False, 'navigation_ready': False,
        'source_graph': str(args.graph), 'variant': args.variant,
        'graph_components': report['components'], 'all_sessions_connected': report['all_sessions_connected'],
        'graph_solver': report['variants'][args.variant]['solver'],
        'graph_failures': report['variants'][args.variant]['failures'],
        'coordinate_frame': 'map_geometry_candidate', 'sessions': {}, 'commands': [],
        'method': 'Original RGB-D frames reprojected through candidate per-frame poses; not deformation of an existing cloud',
        'display_placement_note': 'Disconnected graph components retain previous preview gauge for visual comparison; not verified cross-session registration',
        'settings': {'frame_stride': 1, 'pixel_stride': 1, 'minimum_depth_m': .25,
            'maximum_depth_m': 4., 'minimum_confidence': 50, 'voxel_m': .05, 'semantic_masks': None},
        'source_hashes': {str(p): sha256_file(p) for p in [Path(__file__), Path('src/export_rtabmap_db.py'), args.graph/'report.json']},
        'original_hashes_before': {n: sha256_file(Path(s['database'])) for n, s in config.items()}}
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='2', MKL_NUM_THREADS='1')
    clouds = []
    for name, spec in config.items():
        source = args.graph/name/(args.variant+'.csv')
        manifest['source_hashes'][str(source)] = sha256_file(source)
        cloud, summary = args.output/(name+'.ply'), args.output/(name+'.json')
        command = [sys.executable, 'src/export_rtabmap_db.py', spec['database'], '--pose-csv', str(source),
            '--missing-pose-policy', 'skip', '--frame-stride', '1', '--pixel-stride', '1',
            '--min-depth', '.25', '--max-depth', '4', '--min-confidence', '50', '--voxel', '.05',
            '--cloud', str(cloud), '--trajectory', str(args.output/(name+'_trajectory.csv')), '--summary', str(summary)]
        start = time.monotonic()
        with (args.output/(name+'.log')).open('w') as stream:
            proc = subprocess.run(command, env=env, stdout=stream, stderr=subprocess.STDOUT)
        manifest['commands'].append({'argv': command, 'exit_code': proc.returncode, 'elapsed_seconds': time.monotonic()-start})
        if proc.returncode:
            (args.output/'incomplete_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
            raise RuntimeError('Export failed for '+name)
        data = read_ply(cloud); clouds.append(data)
        manifest['sessions'][name] = {'cloud': cloud.name, 'point_count': len(data),
            'trajectory': name+'_trajectory.csv', 'export_summary': json.loads(summary.read_text()), 'cloud_sha256': sha256_file(cloud)}
        print(f'{name}: {len(data):,} points', flush=True)
    combined = np.concatenate(clouds)
    xyz = np.column_stack([combined[c] for c in ('x', 'y', 'z')])
    rgb = np.column_stack([combined[c] for c in ('r', 'g', 'b')])
    xyz, rgb, _ = voxel_reduce(xyz, rgb, .05)
    write_binary_ply(args.output/'combined_candidate.ply', xyz, rgb)
    manifest['combined_points'] = len(xyz)
    manifest['combined_sha256'] = sha256_file(args.output/'combined_candidate.ply')
    manifest['original_hashes_after'] = {n: sha256_file(Path(s['database'])) for n, s in config.items()}
    manifest['originals_unchanged'] = manifest['original_hashes_before'] == manifest['original_hashes_after']
    if not manifest['originals_unchanged']:
        raise ValueError('Source DB hashes changed')
    manifest['finished_utc'] = utc_now()
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({'output': str(args.output), 'points': len(xyz), 'navigation_ready': False}))


if __name__ == '__main__':
    main()
