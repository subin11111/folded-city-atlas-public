#!/usr/bin/env python3
"""Read-only separation of rigid placement, pose deformation, and surface filling.

Candidate differences are NOT surveyed error. Historical finite-patch identities
are conditional and reused; no correspondence or trajectory is promoted here.
The first pose removes the SE(3) gauge, and a second all-position rigid fit tests
whether the remaining difference can be explained by one rigid placement alone.
"""
import argparse
import csv
import json
from pathlib import Path
import sqlite3

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from check_session_overlap import Session, rigid_fit
from evaluate_map_surfaces import pair_metrics
from evaluate_submap_candidates import fixed_patches
from export_rtabmap_db import load_pose_csv
from optimize_session import Clouds, apply, inverse, sha256_file, utc_now

ROOT = Path(__file__).resolve().parents[1]


def first_pose_align(reference, candidate):
    ids = sorted(set(reference) & set(candidate))
    if not ids:
        raise ValueError('No common poses')
    gauge = reference[ids[0]] @ inverse(candidate[ids[0]])
    return {n: gauge @ candidate[n] for n in ids}, ids[0], gauge


def relative_change(reference, candidate, a, b):
    """Difference at b after aligning a exactly; invariant to candidate gauge."""
    aligned_b = reference[a] @ inverse(candidate[a]) @ candidate[b]
    delta = aligned_b[:3, 3] - reference[b][:3, 3]
    rotation = aligned_b[:3, :3] @ reference[b][:3, :3].T
    return {'delta_xyz_m': delta.tolist(),
            'delta_z_m': float(delta[2]),
            'relative_rotation_deg': float(np.degrees(Rotation.from_matrix(rotation).magnitude())),
            'relative_up_tilt_deg': float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1, 1))))}


def rigid_position_residual(reference, candidate):
    ids = sorted(set(reference) & set(candidate))
    source = np.array([candidate[n][:3, 3] for n in ids])
    target = np.array([reference[n][:3, 3] for n in ids])
    transform = rigid_fit(source, target)
    residual = apply(source, transform) - target
    return ids, transform, residual


def vertical_patch_metric(a, b, pa, pb):
    """Distances to the same frozen planes, expressed along the local map Z axis.

    Not an exact same-pixel correspondence, floor identity, or absolute height.
    A tilted finite patch can mix height and lateral displacement. Keep the
    original normal-distance and angular metrics beside this diagnostic.
    """
    wa, wb = apply(a['testing'], pa), apply(b['testing'], pb)
    na, nb = pa[:3, :3] @ a['normal'], pb[:3, :3] @ b['normal']
    ca, cb = apply(a['center'][None], pa)[0], apply(b['center'][None], pb)[0]
    if min(abs(na[2]), abs(nb[2])) < .8:
        return {'eligible': False, 'reason': 'Not near-horizontal in this pose'}
    fa, rb = (wb-ca) @ na / na[2], (wa-cb) @ nb / nb[2]
    # Both terms use positive world-Z orientation; reverse is sign inverted.
    signed = np.r_[fa, -rb]
    return {'eligible': True,
            'absolute_median_m': float(np.median(abs(signed))),
            'signed_median_m': float(np.median(signed)),
            'plane_a_up_tilt_deg': float(np.degrees(np.arccos(np.clip(abs(na[2]), 0, 1)))),
            'plane_b_up_tilt_deg': float(np.degrees(np.arccos(np.clip(abs(nb[2]), 0, 1))))}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    summary_path = ROOT/'docs/assets/session-portfolios/floor2/summary.json'
    summary = json.loads(summary_path.read_text())
    stages = {s['id']: s for s in summary['stages']}
    codes = ['O00', 'H00', 'J04', 'F03', 'N06']
    raw_path = ROOT/'outputs/student_union_20261017_added_reviewed/floor2/raw_main_trajectory.csv'
    db = ROOT/'data/raw/rtabmap/261017-013501-2f.db'
    selected_manifest = ROOT/'outputs/student_union_selected_20261017_v1/manifest.json'
    selected = json.loads(selected_manifest.read_text())
    selected_trajectory = selected_manifest.parent/selected['sessions']['floor2']['trajectory']
    old_path = ROOT/'outputs/student_union_20261017_independent_v1/evaluation/report.json'
    paths = [Path(__file__), summary_path, raw_path, db, selected_manifest,
             selected_trajectory, old_path]
    paths += [Path(stages[c]['trajectory']) for c in codes]
    hashes = {str(path.resolve()): sha256_file(path) for path in paths}
    poses = {c: load_pose_csv(Path(stages[c]['trajectory'])) for c in codes}
    ids = sorted(poses['H00'])
    assert all(set(poses[c]) == set(ids) for c in codes)
    for code in codes:
        assert hashes[str(Path(stages[code]['trajectory']).resolve())] == stages[code]['trajectory_sha256']
    ref = poses['H00']
    source_choice = load_pose_csv(selected_trajectory)
    assert all(np.allclose(source_choice[n], poses['J04'][n], atol=1e-8) for n in ids)
    rows, candidates, aligned = [], {}, {}
    pair_ids = [(1, 1218), (7, 1197), (14, 1196), (282, 728),
                (642, 1031), (620, 1025)]
    for c in codes:
        aligned[c], anchor, gauge = first_pose_align(ref, poses[c])
        _, rigid, residual = rigid_position_residual(ref, poses[c])
        values = []
        for i, n in enumerate(ids):
            delta = aligned[c][n][:3, 3] - ref[n][:3, 3]
            rc = aligned[c][n][:3, :3] @ ref[n][:3, :3].T
            rv = np.degrees(Rotation.from_matrix(rc).as_rotvec())
            tilt = float(np.degrees(np.arccos(np.clip(rc[2, 2], -1, 1))))
            row = {'candidate': c, 'node': n, 'camera_z_m': float(aligned[c][n][2, 3]),
                   'z_change_from_H00_m': float(delta[2]), 'up_tilt_change_deg': tilt,
                   'map_x_rotvec_deg': float(rv[0]), 'map_y_rotvec_deg': float(rv[1]),
                   'rigid_fit_residual_z_m': float(residual[i, 2]),
                   'rigid_fit_residual_xyz_norm_m': float(np.linalg.norm(residual[i]))}
            rows.append(row)
            values.append(row)
        candidates[c] = {
            'label': stages[c]['label'], 'trajectory_sha256': stages[c]['trajectory_sha256'],
            'anchor_node': anchor, 'first_pose_gauge': gauge.tolist(),
            'best_all_position_rigid_fit': rigid.tolist(),
            'rigid_fit_z_abs_p90_m': float(np.percentile(abs(residual[:, 2]), 90)),
            'rigid_fit_z_range_m': float(np.ptp(residual[:, 2])),
            'rigid_fit_position_rms_m': float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))),
            'z_change_from_H00_range_m': float(np.ptp([r['z_change_from_H00_m'] for r in values])),
            'z_change_from_H00_abs_max_m': float(max(abs(r['z_change_from_H00_m']) for r in values)),
            'up_tilt_change_max_deg': float(max(r['up_tilt_change_deg'] for r in values)),
            'pair_relative_changes': {f'{a}_{b}': relative_change(ref, poses[c], a, b) for a, b in pair_ids}}
    raw = load_pose_csv(raw_path)
    historical = json.loads(old_path.read_text())['comparisons']
    pairs = [r for r in historical if r['session'] == 'floor2' and r['kind'] == 'horizontal'
             and {r['a'], r['b']} <= set(ids)]
    cache, patch_rows = {}, []
    with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as con:
        clouds = Clouds(con, None)
        for r in pairs:
            for n in [r['a'], r['b']]:
                if n not in cache:
                    cache[n] = fixed_patches(clouds, raw, n)
            for c in codes:
                a = cache[r['a']][r['kind'], r['plane_a']]
                b = cache[r['b']][r['kind'], r['plane_b']]
                metric = pair_metrics(a, b, poses[c][r['a']], poses[c][r['b']])
                vertical = vertical_patch_metric(a, b, aligned[c][r['a']], aligned[c][r['b']])
                patch_rows.append({'candidate': c, 'a': r['a'], 'b': r['b'],
                    'plane_a': r['plane_a'], 'plane_b': r['plane_b'],
                    'normal_distance_m': metric['symmetric_plane_abs_median_m'],
                    'normal_difference_deg': metric['normal_difference_degrees'], 'vertical': vertical})
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    colors = {'O00': '#999999', 'H00': '#2876a0', 'J04': '#d25632', 'F03': '#54944c', 'N06': '#8762a6'}
    for c in codes:
        rr = [r for r in rows if r['candidate'] == c]
        axes[0, 0].plot(ids, [r['camera_z_m'] for r in rr], label=c, color=colors[c], linewidth=1.3)
        axes[0, 1].plot(ids, [r['z_change_from_H00_m']*100 for r in rr], color=colors[c], label=c)
    axes[0, 0].set(title='Camera Z: first pose SE(3)-aligned (NOT floor height)', xlabel='Frame ID', ylabel='Z (m)')
    axes[0, 1].set(title='Pose Z change from H00 (H00 is NOT ground truth)', xlabel='Frame ID', ylabel='Change (cm)')
    rr = [r for r in rows if r['candidate'] == 'J04']
    xy = np.array([ref[n][:2, 3] for n in ids])
    sc = axes[1, 0].scatter(xy[:, 0], xy[:, 1], c=[r['rigid_fit_residual_z_m']*100 for r in rr],
                          cmap='coolwarm', vmin=-30, vmax=30, s=9)
    fig.colorbar(sc, ax=axes[1, 0], label='Residual Z (cm), best rigid position fit')
    axes[1, 0].set(title='J04 vs H00: non-rigid remainder after best SE(3)', xlabel='H00 X (m)', ylabel='H00 Y (m)')
    axes[1, 0].set_aspect('equal', adjustable='box')
    # Frame 503 is absent from the common selected 925-frame support. Do not
    # reintroduce a historical patch unavailable in every compared candidate.
    focus = [(620, 1025), (642, 1031)]
    for j, c in enumerate(codes):
        vals = [next(r['normal_distance_m']*100 for r in patch_rows
                     if r['candidate'] == c and (r['a'], r['b']) == pair) for pair in focus]
        axes[1, 1].bar(np.arange(len(focus))+(j-2)*.15, vals, .15, label=c, color=colors[c])
    axes[1, 1].set_xticks(np.arange(len(focus)), [f'{a}/{b}' for a, b in focus])
    axes[1, 1].set(title='Same frozen horizontal patches; reused conditional diagnostics', xlabel='Frame pair (NOT surveyed)', ylabel='Median normal distance (cm)')
    for ax in axes.ravel():
        ax.grid(alpha=.15)
    axes[0, 0].legend(ncol=5)
    axes[1, 1].legend(ncol=5)
    fig.suptitle('Height drift/deformation audit | selected floor2 = J04\nNo pose optimization or display changes in this diagnostic', fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(out/'pose_z_deformation.png', dpi=160)
    plt.close(fig)
    # Locate the tested finite surfaces on original RGB. Their identities remain
    # historical hypotheses, not a new manual/independent correspondence claim.
    session = Session(db, 'floor2')
    fig, axes = plt.subplots(2, 2, figsize=(10, 12))
    for rowindex, pair in enumerate([(620, 1025), (642, 1031)]):
        r = next(r for r in pairs if (r['a'], r['b']) == pair)
        for col, (n, index) in enumerate(zip(pair, [r['plane_a'], r['plane_b']])):
            rgb = cv2.cvtColor(session.image(n), cv2.COLOR_BGR2RGB)
            _, _, width, height, k, local = session.depth(n)
            optical = apply(cache[n]['horizontal', index]['testing'], inverse(local))
            uvw = optical @ k.T
            uv = uvw[:, :2] / uvw[:, 2:3] * [rgb.shape[1]/width, rgb.shape[0]/height]
            valid = (optical[:, 2] > 0) & (uv[:, 0] >= 0) & (uv[:, 0] < rgb.shape[1]) & (uv[:, 1] >= 0) & (uv[:, 1] < rgb.shape[0])
            axes[rowindex, col].imshow(rgb)
            axes[rowindex, col].scatter(uv[valid, 0], uv[valid, 1], s=4, c='#00ffff', alpha=.7)
            axes[rowindex, col].set_title(f'Original RGB #{n}: fixed depth-test patch {index}')
            axes[rowindex, col].axis('off')
    session.connection.close()
    fig.suptitle('Original-photo locations of reused horizontal tests\nNot surveyed correspondences; no rendered/fabricated RGB', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(out/'horizontal_test_photos.png', dpi=140)
    plt.close(fig)
    with (out/'per_pose_changes.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    assert all(sha256_file(Path(path)) == digest for path, digest in hashes.items())
    result = {'created_utc': utc_now(), 'input_sha256': hashes, 'input_files_unchanged': True,
        'pose_changes': 0, 'selected_floor2_exactly_J04': True, 'common_pose_count': len(ids),
        'candidates': candidates, 'horizontal_patch_rows': patch_rows,
        'selected_map_manifest_sha256': hashes[str(selected_manifest.resolve())],
        'limitations': ['Differences from H00 are not absolute error.',
            'Historical finite-patch identities and repeated evaluation are conditional, not surveyed ground truth.',
            'Camera Z contains stairs and hand-height motion, not only drift.',
            'The restroom photo pair 282/728 measures pose deformation, not exact same camera location.',
            'Horizontal patches elsewhere cannot alone prove the cause of the photographed restroom gap.'],
        'figures': {p.name: sha256_file(p) for p in out.glob('*.png')}}
    (out/'summary.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({'output': str(out), 'J04': candidates['J04'], 'patch_comparisons': len(patch_rows)}, indent=2), flush=True)


if __name__ == '__main__':
    main()
