#!/usr/bin/env python3
"""Render offline numerical trajectory and point-cloud comparisons, no alignment fitting."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    return np.array([float(r['stamp']) for r in rows]), np.array([[float(r[c]) for c in ('x', 'y', 'z')] for r in rows])


def read_ply(path: Path) -> np.ndarray:
    with path.open('rb') as stream:
        count = None
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('Incomplete PLY header')
            if line.startswith(b'element vertex '):
                count = int(line.split()[-1])
            if line.strip() == b'end_header':
                break
        if count is None:
            raise ValueError('No vertex count')
        data = np.fromfile(stream, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                          ('r', 'u1'), ('g', 'u1'), ('b', 'u1')], count=count)
    if len(data) != count:
        raise ValueError('Truncated PLY')
    return data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--reference', type=Path)
    parser.add_argument('--title', required=True)
    args = parser.parse_args()
    series = [('Node.pose odometry', args.session / 'raw_main_trajectory.csv', '#4d7ac7'),
              ('Offline robust PGO', args.session / 'optimized_main_trajectory.csv', '#ec6b24')]
    if args.reference:
        series.insert(1, ('App saved optimized', args.reference, '#3c9e7d'))
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), constrained_layout=True)
    start = min(trajectory(path)[0].min() for _, path, _ in series)
    for label, path, color in series:
        stamps, xyz = trajectory(path)
        axes[0].plot(xyz[:, 0], xyz[:, 1], color=color, lw=1.1, label=label)
        axes[1].plot(stamps - start, xyz[:, 2], color=color, lw=1.1, label=label)
        axes[2].plot(xyz[:, 0], xyz[:, 2], color=color, lw=1.1, label=label)
    for ax, labels in zip(axes, [('X (m)', 'Y (m)'), ('Elapsed time (s)', 'Z (m)'), ('X (m)', 'Z (m)')]):
        ax.set_xlabel(labels[0]); ax.set_ylabel(labels[1]); ax.grid(alpha=0.25)
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].legend(fontsize=8)
    fig.suptitle(args.title + ' — same session coordinate frame, no best-fit alignment')
    fig.savefig(args.session / 'trajectory_comparison.png', dpi=170)
    plt.close(fig)

    cloud_paths = [args.session / 'cloud_raw_filtered.ply', args.session / 'cloud_app_filtered.ply',
                   args.session / 'cloud_optimized_filtered.ply']
    if all(p.exists() for p in cloud_paths):
        clouds = [read_ply(p) for p in cloud_paths]
        bounds = {k: (min(np.percentile(c[k], 0.5) for c in clouds),
                      max(np.percentile(c[k], 99.5) for c in clouds)) for k in ('x', 'y', 'z')}
        fig, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
        for col, (cloud, label) in enumerate(zip(clouds, ['Node.pose', 'App saved optimized', 'Offline robust PGO'])):
            indices = np.linspace(0, len(cloud) - 1, min(150000, len(cloud))).astype(int)
            sample = cloud[indices]
            for row, (x, y) in enumerate([('x', 'y'), ('x', 'z')]):
                axes[row, col].scatter(sample[x], sample[y], c=sample['z'], s=0.15, cmap='viridis',
                                       vmin=bounds['z'][0], vmax=bounds['z'][1], rasterized=True)
                axes[row, col].set_xlim(bounds[x]); axes[row, col].set_ylim(bounds[y])
                axes[row, col].set_aspect('equal', adjustable='box')
                axes[row, col].set_xlabel(x.upper() + ' (m)'); axes[row, col].set_ylabel(y.upper() + ' (m)')
                axes[row, col].set_title(label)
        fig.suptitle(args.title + ' — identical frames/masks/filters; color = height; common 0.5–99.5 percentile limits')
        fig.savefig(args.session / 'cloud_comparison.png', dpi=170)
        plt.close(fig)
        (args.session / 'plot_settings.json').write_text(json.dumps({'cloud_display_percentiles': [0.5, 99.5],
            'max_display_points': 150000, 'bounds': bounds,
            'note': 'Plot sampling and clipping only; exported clouds retain every valid voxel.'}, indent=2) + '\n')


if __name__ == '__main__':
    main()
