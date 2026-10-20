#!/usr/bin/env python3
"""Freeze selected session geometries and fit rigid, explicitly uncertain joins.

No raw DB or historical candidate is modified. Whole-session xyz+yaw variables
preserve the chosen internal geometry and actual stair/sloped height variation.
Historical rejected links remain rejected; a selected working map is not a
survey-validated or navigation-approved map.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
import yaml

from build_all_session_preview import checked_rigid
from check_session_overlap import Session
from export_rtabmap_db import load_pose_csv, voxel_reduce, write_binary_ply
from export_submap_candidate_clouds import render_projection
from optimize_session import apply, geometric_score, inverse, sha256_file, trajectory, utc_now
from render_session_report import read_ply
from revalidate_loop_constraints import depth_pixels, exclusion_mask
from view_all_sessions_preview import build_config

ROOT = Path(__file__).resolve().parents[1]
PORTFOLIO = ROOT / 'docs/assets/session-portfolios'
OLD = ROOT / 'outputs/student_union_20261017_all_preview'
GEOMETRY = ROOT / 'outputs/student_union_20261017_geometry_review/graph_final'
QUALITY = ROOT / 'outputs/student_union_20261017_submap_quality'
FRAME = 'map_selected_working'
PREFIX = '/folded_city/selected_working'


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def yaw_transform(values):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('z', values[3]).as_matrix()
    result[:3, 3] = values[:3]
    return result


def physical_error(measurement, a, b):
    delta = inverse(np.asarray(measurement)) @ inverse(a) @ b
    return np.r_[delta[:3, 3], Rotation.from_matrix(delta[:3, :3]).as_rotvec()]


def huber_block(value, delta=3.):
    norm = np.linalg.norm(value)
    return value if norm <= delta else value*np.sqrt(2*delta*norm-delta**2)/norm


def fit_placements(poses, rows, anchor, sigma_m=.15, sigma_deg=5.):
    """Fit session variables only; average correlated edges per connection.

    Audit rows are excluded from both initialization and objective. There is no
    heldout-driven selection of the model, weights or fitted variant.
    """
    train = [r for r in rows if r['role'] == 'train']
    names = sorted(poses)
    variable = [n for n in names if n != anchor]
    # Reject disconnected training graphs rather than assign an arbitrary pose.
    reached = {anchor}
    while True:
        new = reached | {r[k] for r in train for k in ('session_a', 'session_b')
                         if r['session_a'] in reached or r['session_b'] in reached}
        if new == reached:
            break
        reached = new
    if reached != set(names):
        raise ValueError('Training graph is disconnected')
    counts = Counter(tuple(sorted((r['session_a'], r['session_b']))) for r in train)
    scale = np.r_[np.repeat(sigma_m, 3), np.repeat(np.radians(sigma_deg), 3)]

    def decode(x):
        return {anchor: np.eye(4), **{n: yaw_transform(x[4*i:4*i+4]) for i, n in enumerate(variable)}}

    def objective(x):
        placed = decode(x)
        return np.concatenate([huber_block(physical_error(r['T_a_b'],
            placed[r['session_a']]@poses[r['session_a']][r['node_a']],
            placed[r['session_b']]@poses[r['session_b']][r['node_b']])/scale)
            /np.sqrt(counts[tuple(sorted((r['session_a'], r['session_b'])))]) for r in train])

    zero = np.zeros(4*len(variable))
    limits = np.tile([5., 5., 3., np.radians(20.)], len(variable))
    fit = least_squares(objective, zero, bounds=(-limits, limits), max_nfev=600,
                        ftol=1e-11, xtol=1e-11, gtol=1e-11)
    if not fit.success or np.any(fit.active_mask):
        raise ValueError('Placement solver did not converge inside its conservative bounds')
    return decode(fit.x), {'success': bool(fit.success), 'nfev': fit.nfev,
        'initial_cost': float(np.sum(objective(zero)**2)/2), 'final_cost': float(fit.cost),
        'optimality': float(fit.optimality), 'active_bounds': int(np.count_nonzero(fit.active_mask)),
        'variables': variable, 'anchor': anchor, 'translation_sigma_m': sigma_m,
        'rotation_sigma_deg': sigma_deg, 'loss': 'block Huber delta 3; one averaged information unit per connection',
        'sigma_scope': 'Conservative modelling assumptions, not calibrated sensor covariance',
        'internal_frame_variables': 0, 'bounds_m_deg': [5, 5, 3, 20]}


def auxiliary_pose(node, poses, frozen, frame_rows, submaps):
    """Recover attachment-only C pose inside ONE uninterrupted existing submap.

    Never restore its depth to the map and never cross hard boundaries. A left
    rigid correction is interpolated in timestamp between retained neighbours.
    This is a conditional transport assumption, not a new observation.
    """
    if node in poses:
        return poses[node], {'method': 'exact_selected_pose'}
    left = max((n for n in poses if n < node), default=None)
    right = min((n for n in poses if n > node), default=None)
    if left is None or right is None or right-left > 5:
        raise ValueError('Missing pose is not narrowly bracketed')
    primary = frame_rows[left]['primary_submap_id']
    if not primary or primary != frame_rows[right]['primary_submap_id']:
        raise ValueError('Auxiliary pose would cross submap/run boundary')
    between = [r for n, r in frame_rows.items() if left < n <= right]
    if any(str(r.get('hard_boundary')).lower() == 'true' for r in between):
        raise ValueError('Auxiliary pose would cross a hard boundary')
    if primary not in {s['submap_id'] for s in submaps}:
        raise ValueError('Unknown primary submap')
    t0, t1, tn = [float(frame_rows[n]['stamp']) for n in [left, right, node]]
    if not t0 < tn < t1:
        raise ValueError('Nonmonotonic timestamps')
    fraction = (tn-t0)/(t1-t0)
    corrections = [poses[n]@inverse(frozen[n]) for n in [left, right]]
    correction = np.eye(4)
    correction[:3, 3] = (1-fraction)*corrections[0][:3, 3]+fraction*corrections[1][:3, 3]
    correction[:3, :3] = Slerp([0., 1.], Rotation.from_matrix(
        np.stack([c[:3, :3] for c in corrections])))(fraction).as_matrix()
    return correction@frozen[node], {'method': 'attachment_only_same_submap_left_correction_interpolation',
        'bracket': [left, right], 'primary_submap': primary, 'fraction': fraction,
        'excluded_from_cloud_and_saved_selected_trajectory': True,
        'quality_grade': frame_rows[node]['quality_grade'], 'not_a_new_measurement': True}


def outside_fit(report, poses, sessions, track):
    """Refit the pre-existing union-excluded training points in CURRENT poses.

    Old MAP-to-MAP transforms cannot be reused after a non-rigid pose change.
    Each matcher/pair is averaged within one correlated doorway, not counted as
    an independent link. Union masks reserve every recorded holdout neighbourhood.
    """
    source, target, members, masks = [], [], [], {}
    obs = []
    for row in report['observations']:
        path = Path(row['evidence']['path']); track(path)
        z = np.load(path)
        learned = row['matcher'] == 'LoFTR'
        tr = z['train_indices'] if learned else z['train'][z['inliers']]
        held = z['heldout_indices'] if learned else z['heldout']
        pa = z['rgb_pixels_a'] if learned else z['pixels_a']
        pb = z['rgb_pixels_b'] if learned else z['pixels_b']
        if learned:
            # Reproduce the original visual-inlier test, not a new candidate-pose test.
            learned_report = ROOT/'outputs/student_union_20261017_revalidated/learned_cross_session/outside__bridge_out1f/verification_checkpoint.json'
            track(learned_report)
            visual = next(r for r in read(learned_report)['pair_results']
                          if (r['outside_id'], r['lobby_id']) == (row['node_a'], row['node_b']))
            tr = tr[np.linalg.norm(apply(z['source'][tr], np.asarray(visual['visual_T_a_b']))-z['target'][tr], axis=1)<.08]
        pixels = []
        for name, node, uv in [('outside', row['node_a'], pa), ('bridge_out1f', row['node_b'], pb)]:
            dp = depth_pixels(sessions[name], node, uv); pixels.append(dp)
            mask = exclusion_mask(sessions[name].depth(node)[0].shape, dp[held], 3)
            key = name, node
            masks[key] = masks.get(key, np.zeros_like(mask)) | mask
        obs.append((row, z, tr, held, pixels))
    report_rows = []
    for i, (row, z, tr, held, pixels) in enumerate(obs):
        a, b = row['node_a'], row['node_b']
        kept = [j for j in tr if not masks['outside', a][pixels[0][j, 1], pixels[0][j, 0]]
                and not masks['bridge_out1f', b][pixels[1][j, 1], pixels[1][j, 0]]]
        source.append(apply(z['source'][kept], poses['bridge_out1f'][b]))
        target.append(apply(z['target'][kept], poses['outside'][a]))
        members.extend([i]*len(kept))
        report_rows.append({'matcher': row['matcher'], 'node_a': a, 'node_b': b,
            'training_before_union_exclusion': len(tr), 'training_after_union_exclusion': len(kept),
            'original_failed_gates': row['original_failed_gates']})
    source, target, members = np.concatenate(source), np.concatenate(target), np.asarray(members)
    if len(source)<8:
        raise ValueError('Insufficient union-excluded exterior support')
    weights = np.array([1/np.sqrt(np.count_nonzero(members==g)) for g in members])
    def objective(x):
        return np.concatenate([huber_block(r/.08, 1.)*w for r, w in
            zip(apply(source, yaw_transform(x))-target, weights)])
    fit = least_squares(objective, np.zeros(4), max_nfev=400, ftol=1e-11, xtol=1e-11, gtol=1e-11)
    if not fit.success:
        raise ValueError('Exterior fit did not converge')
    t = yaw_transform(fit.x)
    for result, (row, z, tr, held, pixels) in zip(report_rows, obs):
        predicted = inverse(poses['outside'][row['node_a']])@t@poses['bridge_out1f'][row['node_b']]
        errors = np.linalg.norm(apply(z['source'][held], predicted)-z['target'][held], axis=1)
        result['reserved_correspondence_rmse_m'] = float(np.sqrt(np.mean(errors**2)))
        result['reserved_correspondence_count'] = len(held)
    return t, {'success': bool(fit.success), 'nfev': fit.nfev, 'cost': float(fit.cost),
        'training_points': len(source), 'observations': report_rows,
        'strict_accepted': False, 'independent_physical_regions': 1,
        'loss': '3D point-block Huber 8cm; equal pair/matcher weight; union holdout depth-neighbour exclusion',
        'fit_direction': 'T_selected_outside_selected_bridge',
        'scope': 'Current selected poses, reused raw correspondences; no fresh independent validation'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection', type=Path, default=ROOT/'config/student_union_selected_map_20261017.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--assets', type=Path, required=True)
    args = parser.parse_args(); output = args.output.resolve(); assets = args.assets.resolve()
    if output.exists() or assets.exists():
        raise FileExistsError('Use fresh output and evidence folders')
    selection = read(args.selection); inputs = {}
    def track(p):
        path = Path(p).resolve()
        inputs[str(path)] = sha256_file(path)
    track(args.selection); track(Path(__file__))
    for filename in ['optimize_session.py', 'export_rtabmap_db.py', 'build_all_session_preview.py',
                     'revalidate_loop_constraints.py', 'check_session_overlap.py']:
        track(ROOT/'src'/filename)
    quality_path = QUALITY/'manifest.json'; track(quality_path); quality = read(quality_path)
    poses, sources, dbsessions = {}, {}, {}
    for name, chosen in selection['sessions'].items():
        summary_path = PORTFOLIO/name/'summary.json'; track(summary_path); summary = read(summary_path)
        stage = next(s for s in summary['stages'] if s['id'] == chosen['candidate'])
        export = next(s for s in summary['exports'] if s['id'] == chosen['candidate'])
        for p, expected in [(stage['trajectory'], stage['trajectory_sha256']), (export['cloud'], export['sha256'])]:
            if sha256_file(Path(p)) != expected:
                raise ValueError('Frozen selected candidate hash differs')
            track(p)
        poses[name] = load_pose_csv(Path(stage['trajectory']))
        db = Path(quality['sessions'][name]['source_database']); track(db)
        if inputs[str(db.resolve())] != quality['sessions'][name]['database_sha256_before']:
            raise ValueError('Original database changed')
        dbsessions[name] = Session(db, name)
        sources[name] = {'stage': stage, 'export': export, 'database': str(db),
                         'selected_profile': summary['selected_profile']}
    constraints_path = GEOMETRY/'used_constraints.json'; track(constraints_path)
    rows, skipped = [], []
    for original in read(constraints_path):
        a, b = original['session_a'], original['session_b']
        if a == b:
            continue
        row = {k: original[k] for k in ['edge_id', 'session_a', 'node_a', 'session_b', 'node_b', 'T_a_b']}
        row.update(role='audit' if original.get('held_out') else 'train',
                   historical_eligible_for_joint=original.get('eligible_for_joint'), source=str(constraints_path))
        if row['node_a'] not in poses[a] or row['node_b'] not in poses[b]:
            skipped.append({**row, 'reason': 'Selected A/B poses do not contain endpoint; no interpolation across hard boundary'})
            continue
        rows.append(row)
    # Preserve an additional original elevator audit; never add it to fitting.
    audit_path = ROOT/'outputs/student_union_20261017_revalidated/cross_session_clahe_depth/lobby__bridge_12_sub/accepted_frame_edges.json'
    track(audit_path)
    for r in read(audit_path):
        if r['node_a'] == 443:
            rows.append({k: r[k] for k in ['edge_id', 'session_a', 'node_a', 'session_b', 'node_b', 'T_a_b']} |
                        {'role': 'audit', 'historical_eligible_for_joint': r.get('eligible_for_joint'), 'source': str(audit_path)})
    interior = {n: poses[n] for n in ['floor2', 'lobby', 'bridge_12_sub']}
    placement, solver = fit_placements(interior, rows, 'floor2')
    bridge_report_path = OLD/'lobby_bridge/attachment.json'; track(bridge_report_path)
    bridge_report = read(bridge_report_path)
    door = next(r for r in bridge_report['rows'] if (r['outside_id'], r['lobby_id']) == (39, 79))
    frame_path = QUALITY/'bridge_out1f/frames.csv'; submap_path = QUALITY/'bridge_out1f/submaps.json'
    frozen_path = Path(quality['sessions']['bridge_out1f']['frozen_pose_csv'])
    for p in [frame_path, submap_path, frozen_path]:
        track(p)
    frame_rows = {int(r['node_id']): r for r in csv.DictReader(frame_path.open())}
    bridge79, auxiliary = auxiliary_pose(79, poses['bridge_out1f'], load_pose_csv(frozen_path),
                                        frame_rows, read(submap_path)['submaps'])
    # A single existing doorway places only the bridge rigid body, not the interior.
    fixed_lobby = {n: placement['lobby']@p for n, p in poses['lobby'].items()}
    bridge_support = dict(poses['bridge_out1f']); bridge_support[79] = bridge79
    door_row = {'edge_id': 'lobby:39__bridge_out1f:79', 'session_a': 'lobby', 'node_a': 39,
                'session_b': 'bridge_out1f', 'node_b': 79,
                'T_a_b': door['lobby_node_to_outside_node'], 'role': 'train',
                'source': str(bridge_report_path), 'historical_eligible_for_joint': False}
    t_door, door_solver = fit_placements({'lobby': fixed_lobby, 'bridge_out1f': bridge_support},
                                       [door_row], 'lobby', sigma_m=.30, sigma_deg=8.)
    placement['bridge_out1f'] = t_door['bridge_out1f']
    # Exterior: refit metric point measurements after internal J01 pose changes.
    outside_path = OLD/'outside_bridge/attachment_candidate.json'; track(outside_path)
    t_outside_bridge, exterior_solver = outside_fit(read(outside_path), poses, dbsessions, track)
    placement['outside'] = checked_rigid(placement['bridge_out1f']@inverse(t_outside_bridge))
    # Audit known alternative doorway 11/23 if both exact poses are retained.
    for r in bridge_report['rows']:
        if (r['outside_id'], r['lobby_id']) == (11, 23) and 23 in poses['bridge_out1f']:
            rows.append({'edge_id': 'lobby:11__bridge_out1f:23', 'session_a': 'lobby', 'node_a': 11,
                'session_b': 'bridge_out1f', 'node_b': 23, 'T_a_b': r['lobby_node_to_outside_node'],
                'role': 'audit', 'historical_eligible_for_joint': False, 'source': str(bridge_report_path)})
        elif (r['outside_id'], r['lobby_id']) == (11, 23):
            skipped.append({'edge_id': 'lobby:11__bridge_out1f:23', 'role': 'audit',
                            'reason': 'bridge23 excluded from selected A/B poses; not restored'})
    rows.append(door_row)
    evaluation = []
    eval_poses = dict(poses); eval_poses['bridge_out1f'] = bridge_support
    for row in rows:
        sa, sb, a, b = row['session_a'], row['session_b'], row['node_a'], row['node_b']
        predicted = inverse(placement[sa]@eval_poses[sa][a])@placement[sb]@eval_poses[sb][b]
        error = physical_error(row['T_a_b'], placement[sa]@eval_poses[sa][a], placement[sb]@eval_poses[sb][b])
        ca, cb = dbsessions[sa].cloud(a), dbsessions[sb].cloud(b)
        direct = geometric_score(ca, cb, np.asarray(row['T_a_b']), .20)
        placed = geometric_score(ca, cb, predicted, .20)
        evaluation.append({**row, 'translation_error_m': float(np.linalg.norm(error[:3])),
            'rotation_error_deg': float(np.degrees(np.linalg.norm(error[3:]))),
            'direct_measurement_geometry': direct, 'placed_geometry': placed,
            'rmse_increase_m': placed['clipped_rmse_m']-direct['clipped_rmse_m'],
            'geometry_guard_passed': bool(placed['clipped_rmse_m']<=direct['clipped_rmse_m']+.02
                                          and placed['overlap']>=direct['overlap']-.10)})
    output.mkdir(parents=True); assets.mkdir(parents=True)
    manifest = {'started_utc': utc_now(), 'status': 'EXPLORATORY_ALL_SESSION_PREVIEW_NOT_FOR_NAVIGATION',
        'user_designated_working_map': True, 'selection_fixed': True, 'selection': selection,
        'unvalidated_preview': True, 'accepted_by_validation': False, 'navigation_ready': False,
        'frame': FRAME, 'topic_prefix': PREFIX, 'combined_cloud': 'combined.ply', 'sessions': {},
        'method': 'Selected immutable session geometries + hierarchical rigid xyz/yaw placement; no internal re-optimization',
        'placement_solver': solver, 'door_solver': door_solver, 'bridge79_attachment_only_pose': auxiliary,
        'exterior_solver': exterior_solver, 'connection_evaluations': evaluation,
        'skipped_measurements': skipped,
        'limitations': ['User visual selection is not survey or navigation validation.',
            'Every historical cross-session strict eligibility flag remains false.',
            'Single entrance support cannot validate outdoor/bridge or bridge/lobby globally.',
            'Some C-grade endpoints are absent; they are not silently reintroduced into clouds.',
            'Bridge79 is attachment-only transported inside one submap, with larger model uncertainty.',
            'Historical audits are reused/correlated development observations, not new independent truth.',
            'Selected J04 retains the documented 656/1055 finite-patch gap; nothing is flattened or erased.',
            'Any cross-session mismatch that cannot be fixed rigidly is retained, not hidden by warping.']}
    xyzs, rgbs, samples = [], [], {}
    for name in selection['sessions']:
        t = checked_rigid(placement[name])
        if not np.allclose(t[2, :3], [0, 0, 1], atol=1e-8):
            raise ValueError('Placement must preserve up direction and height variation')
        source = sources[name]; cloud = read_ply(Path(source['export']['cloud']))
        xyz = apply(np.column_stack([cloud[k] for k in ['x', 'y', 'z']]), t).astype(np.float32)
        rgb = np.column_stack([cloud[k] for k in ['r', 'g', 'b']])
        write_binary_ply(output/(name+'.ply'), xyz, rgb)
        selected = poses[name]; moved = {n: t@p for n, p in selected.items()}
        stamps = dbsessions[name].stamps
        trajectory(output/(name+'_trajectory.csv'), moved, stamps)
        # Exact relative-pose invariance in double precision, before PLY float32 export.
        anchor = min(selected)
        max_error = max(float(np.max(np.abs(inverse(selected[anchor])@p-inverse(moved[anchor])@moved[n])))
                        for n, p in selected.items())
        if max_error>1e-8:
            raise ValueError('Selected internal geometry changed')
        entry = {'candidate': selection['sessions'][name]['candidate'], 'reason': selection['sessions'][name]['reason'],
            'cloud': name+'.ply', 'trajectory': name+'_trajectory.csv', 'placement': 'selected internal poses frozen; rigid xyz+yaw only',
            'point_count': len(xyz), 'pose_count': len(selected), 'source_cloud': source['export']['cloud'],
            'source_trajectory': source['stage']['trajectory'], 'preprocessing': source['selected_profile'],
            'T_working_selected': t.tolist(), 'internal_shape_changed': False, 'vertical_scale': 1.,
            'maximum_internal_relative_matrix_error': max_error,
            'camera_z_span_before_m': float(np.ptp([p[2, 3] for p in selected.values()])),
            'camera_z_span_after_m': float(np.ptp([p[2, 3] for p in moved.values()])),
            'bounds_min_m': xyz.min(axis=0).tolist(), 'bounds_max_m': xyz.max(axis=0).tolist()}
        manifest['sessions'][name] = entry
        samples[name] = xyz[np.linspace(0, len(xyz)-1, min(140000, len(xyz))).astype(int)]
        xyzs.append(xyz); rgbs.append(rgb)
        print(f'{name} {entry["candidate"]}: frozen {len(selected)} poses, placed {len(xyz):,} points', flush=True)
    combined, rgb, _ = voxel_reduce(np.concatenate(xyzs), np.concatenate(rgbs), .05)
    write_binary_ply(output/'combined.ply', combined, rgb)
    manifest['combined_points'] = len(combined); manifest['combined_voxel_m'] = .05
    manifest['combined_sha256'] = sha256_file(output/'combined.ply')
    colors = {'outside': '#57a85a', 'lobby': '#e9a235', 'bridge_out1f': '#c65f9a',
              'floor2': '#268dc0', 'bridge_12_sub': '#8e69b9'}
    figures = []
    def save(fig, filename):
        path = assets/filename; fig.savefig(path, dpi=170); plt.close(fig)
        figures.append({'file': filename, 'sha256': sha256_file(path), 'synthetic': False})
    fig, axes = plt.subplots(1, 2, figsize=(16, 8), constrained_layout=True)
    for name, sample in samples.items():
        for ax, (i, j) in zip(axes, [(0, 1), (0, 2)]):
            ax.scatter(sample[:, i], sample[:, j], s=.3, alpha=.4, color=colors[name],
                       label=name+' '+selection['sessions'][name]['candidate'], rasterized=True)
    for ax, ylabel in zip(axes, ['Y (m)', 'Z (m)']):
        ax.set(xlabel='X (m)', ylabel=ylabel); ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.15)
    axes[1].legend(fontsize=9)
    fig.suptitle('Selected working map: five frozen geometries, rigid connections\n'
                 '2F/main stair J04 fixed | connection hypotheses, NOT navigation-validated')
    save(fig, 'selected_map_sessions.png')
    all_cloud = read_ply(output/'combined.ply')
    bounds = {k: [float(all_cloud[k].min())-.2, float(all_cloud[k].max())+.2] for k in ['x', 'y', 'z']}
    fig, axes = plt.subplots(1, 2, figsize=(16, 8), constrained_layout=True)
    for ax, (first, second) in zip(axes, [('x', 'y'), ('x', 'z')]):
        render_projection(ax, all_cloud, first, second, bounds, title=first.upper()+'-'+second.upper(), max_points=850000)
    fig.suptitle('Selected working map | actual RGB-D, height colours | no floor flattening or internal warping')
    save(fig, 'selected_map_height.png')
    # Explicit before/after of the same fixed session samples, not a new filtered picture.
    fig, axes = plt.subplots(1, 2, figsize=(16, 8), constrained_layout=True)
    for name, sample in samples.items():
        original = apply(sample, inverse(placement[name]))
        for ax, pp in zip(axes, [original, sample]):
            ax.scatter(pp[:, 0], pp[:, 1], s=.3, alpha=.4, color=colors[name], rasterized=True)
    for ax, title in zip(axes, ['Selected local candidates BEFORE rigid joins', 'AFTER rigid joins; internal shapes unchanged']):
        ax.set(title=title, xlabel='X (m)', ylabel='Y (m)'); ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.15)
    low = np.min(np.vstack([ax.get_xlim()[0] for ax in axes])); high = max(ax.get_xlim()[1] for ax in axes)
    ylow = min(ax.get_ylim()[0] for ax in axes); yhigh = max(ax.get_ylim()[1] for ax in axes)
    for ax in axes:
        ax.set(xlim=(low, high), ylim=(ylow, yhigh))
    save(fig, 'selected_map_before_after.png')
    # Geometry scores remain labelled even when they fail; no validation flags promoted.
    fig, ax = plt.subplots(figsize=(13, 6), constrained_layout=True)
    labels = [r['edge_id']+' ['+r['role']+']' for r in evaluation]
    x = np.arange(len(labels))
    ax.barh(x-.17, [r['direct_measurement_geometry']['clipped_rmse_m']*100 for r in evaluation], height=.34, label='Direct local measurement')
    ax.barh(x+.17, [r['placed_geometry']['clipped_rmse_m']*100 for r in evaluation], height=.34, label='Selected rigid map')
    ax.set(yticks=x, yticklabels=labels, xlabel='Clipped point-cloud RMSE (cm); correlated development observations',
           title='Connection audit: rigid joins cannot remove internal inconsistencies')
    ax.legend(); ax.grid(axis='x', alpha=.15)
    save(fig, 'connection_audit.png')
    # Isolate viewer topics/frame from the old preview, so two publishers cannot mix maps.
    import view_all_sessions_preview as viewer
    old_frame, old_prefix = viewer.FRAME, viewer.PREFIX
    try:
        viewer.FRAME, viewer.PREFIX = FRAME, PREFIX
        config = build_config(yaml.safe_load((ROOT/'config/folded_city.rviz').read_text()),
                              combined.min(axis=0), combined.max(axis=0), manifest['sessions'])
        config['Visualization Manager']['Displays'][0]['Name'] = 'SELECTED WORKING MAP | merged RGB | navigation unvalidated'
        config_path = output/'SELECTED_WORKING_MAP.rviz'
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    finally:
        viewer.FRAME, viewer.PREFIX = old_frame, old_prefix
    manifest['rviz_config'] = config_path.name
    manifest['inputs_sha256'] = inputs
    manifest['all_inputs_unchanged'] = all(sha256_file(Path(p)) == h for p, h in inputs.items())
    if not manifest['all_inputs_unchanged']:
        raise ValueError('Historical input changed')
    manifest['database_integrity'] = {n: {'database': str(s.path), 'sha256': inputs[str(s.path)], 'unchanged': True}
                                      for n, s in dbsessions.items()}
    manifest['finished_utc'] = utc_now()
    write(output/'manifest.json', manifest)
    write(assets/'summary.json', manifest)
    write(assets/'manifest.json', {'created_utc': utc_now(), 'figures': figures,
        'summary_sha256': sha256_file(assets/'summary.json'), 'output_manifest': str(output/'manifest.json'),
        'source_sha256': sha256_file(Path(__file__)), 'all_inputs_unchanged': True})
    for s in dbsessions.values():
        s.connection.close()
    print(json.dumps({'complete': True, 'combined_points': len(combined), 'manifest': str(output/'manifest.json'),
                      'all_inputs_unchanged': True, 'navigation_ready': False}))


if __name__ == '__main__':
    main()
