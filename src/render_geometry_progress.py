#!/usr/bin/env python3
"""Render identical RGB-D samples through every recorded geometry stage."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from check_session_overlap import DEFAULTS, Session
from export_rtabmap_db import load_pose_csv
from optimize_session import apply, sha256_file, utc_now

COLORS = {'lobby': '#e99731', 'floor2': '#1594c7', 'bridge_out1f': '#d358a4',
          'outside': '#309c54', 'bridge_12_sub': '#8c59c6'}


def variant_title(variant):
    return {'pgo4': '4DoF robust graph (selected candidate)',
            'pgo6_attitude': 'Bounded SE3 + baseline attitude prior',
            'pgo6_normals': 'Bounded SE3 + local normal factors',
            'pgo6_planes': 'Bounded SE3 + finite plane factors'}.get(
                variant, 'Selected candidate: '+variant)


def capture_input_provenance(config_path, config, stages):
    source_dir = Path(__file__).resolve().parent
    paths = [Path(__file__).resolve(), config_path,
             source_dir/'check_session_overlap.py', source_dir/'export_rtabmap_db.py',
             source_dir/'optimize_session.py']
    paths.extend(p for stage in stages.values() for p in stage.values())
    return {'source_hashes': {str(p): sha256_file(p) for p in paths},
            'database_hashes_before': {n: sha256_file(Path(s['database'])) for n, s in config.items()}}


def finalize_manifest(manifest, config, output):
    actual = {p.name for p in output.glob('*.png')}
    if actual != set(manifest['figures']):
        raise ValueError('Rendered PNG inventory differs from manifest figure entries')
    manifest['figure_hashes'] = {n: sha256_file(output/n) for n in sorted(actual)}
    manifest['database_hashes_after'] = {n: sha256_file(Path(s['database'])) for n, s in config.items()}
    manifest['originals_unchanged'] = manifest['database_hashes_before'] == manifest['database_hashes_after']
    if not manifest['originals_unchanged']:
        raise ValueError('Source DB hashes changed during rendering')
    manifest['finished_utc'] = utc_now()


def compare_clouds(panels, path, title, labels=None):
    fig, axes = plt.subplots(2, len(panels), figsize=(5*len(panels), 9), squeeze=False,
                             constrained_layout=True)
    all_points = np.concatenate([p for clouds in panels.values() for p in clouds.values()])
    bounds = [(np.percentile(all_points[:, d], .1)-.2, np.percentile(all_points[:, d], 99.9)+.2) for d in range(3)]
    for column, (label, clouds) in enumerate(panels.items()):
        for name, points in clouds.items():
            for row, (x, y) in enumerate(((0, 1), (0, 2))):
                ax = axes[row, column]
                ax.scatter(points[:, x], points[:, y], s=.2, alpha=.5,
                           color=COLORS.get(name, '#49576c') if labels is None else labels[name], label=name)
                ax.set(xlim=bounds[x], ylim=bounds[y], xlabel='XYZ'[x]+' (m)', ylabel='XYZ'[y]+' (m)')
                ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.2)
        axes[0, column].set_title(label, fontsize=10)
    axes[0, -1].legend(fontsize=7, markerscale=5)
    fig.suptitle(title, fontsize=12)
    fig.savefig(path, dpi=150); plt.close(fig)
    return {'common_bounds_m': bounds, 'projection': ['XY', 'XZ'], 'sampling_identical_across_panels': True}


def render_height_profiles(poses, sessions, path):
    fig, axes = plt.subplots(1, 2, figsize=(15, 5), constrained_layout=True)
    counts = {}
    for ax, n in zip(axes, ('floor2', 'bridge_12_sub')):
        ids = sorted(set.intersection(*(set(p[n]) for p in poses.values())))
        counts[n] = len(ids)
        t = [sessions[n].stamps[i]-sessions[n].stamps[ids[0]] for i in ids]
        for label, pp in poses.items():
            ax.plot(t, [pp[n][i][2, 3] for i in ids], label=label, lw=1.)
        ax.set(title=n, xlabel='Elapsed capture time (s)', ylabel='Camera Z (m)'); ax.grid(alpha=.2)
    axes[1].legend(fontsize=8)
    fig.suptitle('Stair / floor transitions retained | camera Z is not a surveyed floor height')
    fig.savefig(path, dpi=150); plt.close(fig)
    return {'projection': ['elapsed_capture_time_s', 'camera_z_m'],
            'sessions': list(counts), 'common_frame_counts': counts,
            'sampling_identical_across_panels': True, 'surveyed_floor_height': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--graph', type=Path, required=True)
    parser.add_argument('--baseline-graph', type=Path)
    parser.add_argument('--variant', default='pgo6_planes')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Choose new render output')
    args.output.mkdir(parents=True)
    root = Path(__file__).resolve().parents[1]
    config_path = root/'config/student_union_all_sessions.json'
    config = json.loads(config_path.read_text())['sessions']
    preview = root/'outputs/student_union_20261017_all_preview/bundle'
    candidate_title = variant_title(args.variant)
    stages = {'Previous RViz preview': {n: preview/f'{n}_trajectory.csv' for n in config},
              'Frozen baseline + training placement': {n: args.graph/n/'initial.csv' for n in config},
              '4DoF robust graph': {n: args.graph/n/'pgo4.csv' for n in config},
              candidate_title: {n: args.graph/n/(args.variant+'.csv') for n in config}}
    manifest = {'created_utc': utc_now(), 'argv': sys.argv, 'variant': args.variant,
        'source_graph': str(args.graph), 'baseline_graph': str(args.baseline_graph) if args.baseline_graph else None,
        'stage_titles': list(stages),
        'method': 'same deterministic frame/pixel samples, common axes, no image-based alignment',
        'settings': {'frame_stride': 8, 'maximum_points_per_global_frame': 500,
                     'cloud_pixel_stride': 2, 'depth_range_m': [DEFAULTS['min_depth_m'], DEFAULTS['max_depth_m']],
                     'minimum_confidence': DEFAULTS['min_confidence'], 'voxel_m': DEFAULTS['voxel_m'],
                     'plot_bounds_percentiles': [.1, 99.9]},
        **capture_input_provenance(config_path, config, stages), 'figures': {}}
    poses = {label: {n: load_pose_csv(p) for n, p in paths.items()} for label, paths in stages.items()}
    sessions = {n: Session(Path(s['database']), n) for n, s in config.items()}
    sampled = {label: {} for label in stages}
    for n, session in sessions.items():
        ids = sorted(set.intersection(*(set(p[n]) for p in poses.values())))
        arrays = {label: [] for label in stages}
        for node in ids[::8]:
            cloud = session.cloud(node)
            if not len(cloud):
                continue
            take = np.linspace(0, len(cloud)-1, min(500, len(cloud))).astype(int)
            for label in stages:
                arrays[label].append(apply(cloud[take], poses[label][n][node]))
        for label in stages:
            sampled[label][n] = np.concatenate(arrays[label])
        session.geometry.clear()
    manifest['figures']['01_global_progress.png'] = compare_clouds(sampled, args.output/'01_global_progress.png',
        'Geometry experiment progression | all variants remain experimental; disconnected components are display placements')
    for n in config:
        manifest['figures'][f'02_{n}_progress.png'] = compare_clouds({k: {n: v[n]} for k, v in sampled.items()},
            args.output/f'02_{n}_progress.png', n+' | identical observations across all stages')
    pairs = [('lobby', 595, 'lobby', 650), ('floor2', 548, 'floor2', 981),
             ('floor2', 656, 'floor2', 1055), ('outside', 69, 'outside', 323),
             ('lobby', 327, 'floor2', 51), ('lobby', 462, 'bridge_12_sub', 36),
             ('floor2', 560, 'bridge_12_sub', 162), ('lobby', 39, 'bridge_out1f', 79),
             ('outside', 263, 'bridge_out1f', 117)]
    for sa, a, sb, b in pairs:
        panels = {}
        for label, pp in poses.items():
            panels[label] = {f'{sa}:{a}': apply(sessions[sa].cloud(a), pp[sa][a]),
                             f'{sb}:{b}': apply(sessions[sb].cloud(b), pp[sb][b])}
        filename = f'03_{sa}{a}_{sb}{b}.png'
        manifest['figures'][filename] = compare_clouds(panels, args.output/filename,
            f'{sa}:{a} vs {sb}:{b} | frame-level duplicate-wall diagnostic, not surveyed accuracy',
            {f'{sa}:{a}': '#1675bb', f'{sb}:{b}': '#ec7d27'})
    manifest['figures']['04_height_profiles.png'] = render_height_profiles(
        poses, sessions, args.output/'04_height_profiles.png')
    if args.baseline_graph:
        comparisons = {'Before added surface / visual edges': args.baseline_graph, 'After added evidence': args.graph}
        panels = {}
        for label, directory in comparisons.items():
            panels[label] = {}
            for n in config:
                variant = 'pgo6_normals' if directory == args.baseline_graph else args.variant
                source_path = directory/n/(variant+'.csv')
                pp = load_pose_csv(source_path)
                manifest['source_hashes'][str(source_path)] = sha256_file(source_path)
                old = poses[candidate_title][n]
                arrays = []
                for node in sorted(set(pp) & set(old))[::8]:
                    cloud = sessions[n].cloud(node)
                    if len(cloud):
                        arrays.append(apply(cloud[np.linspace(0, len(cloud)-1, min(500, len(cloud))).astype(int)], pp[node]))
                panels[label][n] = np.concatenate(arrays)
        manifest['figures']['05_added_evidence_ablation.png'] = compare_clouds(panels,
            args.output/'05_added_evidence_ablation.png',
            f'Before/after added measurements (pgo6_normals to {args.variant}) | no acceptance implied')
    for session in sessions.values():
        session.connection.close()
    finalize_manifest(manifest, config, args.output)
    (args.output/'render_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({'output': str(args.output), 'figures': len(list(args.output.glob('*.png')))}))


if __name__ == '__main__':
    main()
