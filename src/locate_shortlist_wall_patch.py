#!/usr/bin/env python3
"""Read-only locator for the historically paired 656/1055 finite wall patches.

Does not optimize, establish physical correspondence, or replace map artifacts.
"""
import json
from pathlib import Path
import sqlite3

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from check_session_overlap import Session
from evaluate_submap_candidates import fixed_patches
from export_rtabmap_db import load_pose_csv
from export_submap_candidate_clouds import render_projection
from optimize_session import Clouds, apply, inverse, sha256_file
from render_session_report import read_ply
from revalidate_loop_constraints import SPECS


def main():
    root = Path(__file__).resolve().parents[1]
    dest = root / 'docs/assets/visual-shortlist-review/wall_656_1055_locator.png'
    report_path = dest.with_suffix('.json')
    if dest.exists() or report_path.exists():
        raise FileExistsError('Refusing to overwrite diagnostic evidence')
    portfolio = root / 'outputs/student_union_20261017_session_portfolios_v1/floor2'
    summary_path = root / 'docs/assets/session-portfolios/floor2/summary.json'
    summary = json.loads(summary_path.read_text())
    stages = {s['id']: s for s in summary['stages']}
    quality = json.loads((root / 'outputs/student_union_20261017_submap_quality/manifest.json').read_text())
    db = Path(quality['sessions']['floor2']['source_database'])
    raw_path = root / 'outputs' / SPECS['floor2'][1] / 'floor2/raw_main_trajectory.csv'
    paths = [db, raw_path, summary_path, Path(__file__)]
    paths += [Path(stages[c]['trajectory']) for c in ['H00', 'J04']]
    paths += [portfolio / 'clouds' / (c + '.ply') for c in ['H00', 'J04']]
    hashes = {str(p): sha256_file(p) for p in paths}
    raw = load_pose_csv(raw_path)
    with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as connection:
        clouds = Clouds(connection, None)
        patches = {node: fixed_patches(clouds, raw, node)['wall', index]
                   for node, index in [(656, 0), (1055, 1)]}
    fig, axes = plt.subplots(3, 2, figsize=(12, 15), constrained_layout=True)
    colors = ['#1778bc', '#ef8c20']
    world = {}
    for col, code in enumerate(['H00', 'J04']):
        poses = load_pose_csv(Path(stages[code]['trajectory']))
        points = [apply(patches[n]['testing'], poses[n]) for n in [656, 1055]]
        world[code] = points
        bounds = summary['export_settings']['common_full_cloud_bounds']
        render_projection(axes[0, col], read_ply(portfolio / 'clouds' / (code + '.ply')),
                          'x', 'y', bounds, title=code + ': location on the entire map')
        joined = np.concatenate(points)
        low, high = joined[:, :2].min(axis=0) - .3, joined[:, :2].max(axis=0) + .3
        axes[0, col].add_patch(Rectangle(low, *(high-low), fill=False,
                                        edgecolor='red', linewidth=2))
        axes[0, col].annotate('656 / 1055 patch\n(enlarged below)',
                              xy=joined[:, :2].mean(axis=0), xytext=(-8, -19),
                              arrowprops={'arrowstyle': '->', 'color': 'red'}, color='red', fontsize=10)
        for p, color, node in zip(points, colors, [656, 1055]):
            axes[1, col].scatter(p[:, 0], p[:, 1], s=13, color=color, label=str(node))
        gap = 4.048412629829 if code == 'H00' else 54.423920420501
        axes[1, col].set(title=f'{code}: finite patch plane-distance {gap:.2f} cm',
                        xlabel='X (m)', ylabel='Y (m)')
        axes[1, col].set_aspect('equal', adjustable='box')
        axes[1, col].grid(alpha=.2)
        axes[1, col].legend()
    all_points = np.concatenate([p for pair in world.values() for p in pair])
    for ax in axes[1]:
        ax.set(xlim=(all_points[:, 0].min()-.15, all_points[:, 0].max()+.15),
               ylim=(all_points[:, 1].min()-.15, all_points[:, 1].max()+.15))
    session = Session(db, 'floor2')
    for col, (node, color) in enumerate(zip([656, 1055], colors)):
        image = cv2.cvtColor(session.image(node), cv2.COLOR_BGR2RGB)
        _, _, width, height, intrinsic, local = session.depth(node)
        optical = apply(patches[node]['testing'], inverse(local))
        projected = optical @ intrinsic.T
        uv = projected[:, :2] / projected[:, 2:3]
        uv *= np.array([image.shape[1]/width, image.shape[0]/height])
        valid = ((optical[:, 2] > 0) & (uv[:, 0] >= 0) & (uv[:, 0] < image.shape[1])
                 & (uv[:, 1] >= 0) & (uv[:, 1] < image.shape[0]))
        axes[2, col].imshow(image)
        axes[2, col].scatter(uv[valid, 0], uv[valid, 1], s=10, color=color, alpha=.8)
        axes[2, col].set_title(f'Original RGB #{node}: exact tested depth points')
        axes[2, col].axis('off')
    session.connection.close()
    fig.suptitle('Where did 4.05 -> 54.42 cm come from?\n'
                 'A small historical wall-patch pair, NOT a surveyed whole-wall error', fontsize=15)
    fig.savefig(dest, dpi=160)
    plt.close(fig)
    assert all(sha256_file(Path(p)) == h for p, h in hashes.items())
    report_path.write_text(json.dumps({'input_hashes': hashes, 'figure_sha256': sha256_file(dest),
        'frames': [656, 1055], 'planes': [0, 1], 'source_inputs_unchanged': True,
        'same_physical_surface_independently_confirmed': False,
        'note': 'Historical tentative correspondence; this figure locates the exact tested points, not surveyed truth.'}, indent=2)+'\n')
    print(dest)


if __name__ == '__main__':
    main()
