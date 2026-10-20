#!/usr/bin/env python3
"""Explicitly unvalidated rigid placement of the secondary-stair session.

No source constraints, trajectories, or database are edited. A single yaw and
translation are fitted, so all internal heights and relative poses survive.
This intentionally does not grant strict joint-optimization eligibility.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from check_session_overlap import Session
from export_rtabmap_db import load_pose_csv, write_binary_ply
from optimize_session import geometric_score, inverse, sha256_file, trajectory, utc_now
from render_session_report import read_ply


def transform(values):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec([0., 0., values[3]]).as_matrix()
    result[:3, 3] = values[:3]
    return result


def residual(row, placement, reference, source):
    predicted = inverse(reference[row['session_a']][row['node_a']]) @ placement @ source[row['node_b']]
    error = inverse(np.asarray(row['T_a_b'])) @ predicted
    return np.r_[error[:3, 3], Rotation.from_matrix(error[:3, :3]).as_rotvec()]


def fit(rows, reference, source):
    """Equal support per training region; fixed tilt, block Huber delta 3."""
    if not rows:
        raise ValueError('At least one explicit training pair is needed')
    if len({r['edge_id'] for r in rows}) != len(rows):
        raise ValueError('Duplicate measurements cannot count as new evidence')
    inferred = [reference[r['session_a']][r['node_a']] @ np.asarray(r['T_a_b']) @
                inverse(source[r['node_b']]) for r in rows]
    angles = np.array([np.arctan2(t[1, 0]-t[0, 1], t[0, 0]+t[1, 1]) for t in inferred])
    yaw = np.arctan2(np.sin(angles).mean(), np.cos(angles).mean())
    rotation = transform([0, 0, 0, yaw])[:3, :3]
    translations = [reference[r['session_a']][r['node_a']] @ np.asarray(r['T_a_b']) for r in rows]
    translation0 = np.median([t[:3, 3]-rotation @ source[r['node_b']][:3, 3]
                             for r, t in zip(rows, translations)], axis=0)
    def objective(values):
        blocks = []
        for row in rows:
            r = residual(row, transform(values), reference, source) / np.r_[np.repeat(.10, 3), np.repeat(np.deg2rad(3.), 3)]
            norm = np.linalg.norm(r)
            if norm > 3.:
                r *= np.sqrt(6.*norm-9.) / norm
            blocks.append(r)
        return np.concatenate(blocks)
    solution = least_squares(objective, np.r_[translation0, yaw], max_nfev=250,
                             ftol=1e-11, xtol=1e-11, gtol=1e-11)
    return transform(solution.x), {'success': bool(solution.success), 'nfev': solution.nfev,
                                  'cost': solution.cost, 'message': solution.message}


def geometry_guards(evaluations, max_rmse_increase=.02, max_overlap_loss=.10):
    """Reapply existing joint geometry limits; missing/nonfinite data fail closed."""
    checks, failures = [], []
    for row in evaluations:
        direct, placed = row.get('direct_measurement_geometry', {}), row.get('placed_geometry', {})
        values = [direct.get('clipped_rmse_m'), placed.get('clipped_rmse_m'),
                  direct.get('overlap'), placed.get('overlap')]
        valid = all(v is not None and np.isfinite(v) for v in values)
        increase = float(values[1]-values[0]) if valid else None
        loss = float(values[2]-values[3]) if valid else None
        reasons = []
        if not valid:
            reasons.append('missing or nonfinite geometry metrics')
        else:
            if increase > max_rmse_increase:
                reasons.append(f'clipped RMSE increase {increase:.6f}m exceeds {max_rmse_increase:.3f}m')
            if loss > max_overlap_loss:
                reasons.append(f'overlap loss {loss:.6f} exceeds {max_overlap_loss:.3f}')
        checks.append({'edge_id': row['edge_id'], 'role': row['role'],
                       'clipped_rmse_increase_m': increase, 'overlap_loss': loss,
                       'max_clipped_rmse_increase_m': max_rmse_increase,
                       'max_overlap_loss': max_overlap_loss, 'passed': not reasons,
                       'failure_reasons': reasons})
        failures.extend(f'Geometry guard {row["edge_id"]}: {reason}' for reason in reasons)
    return checks, failures


def refresh_geometry_guards(path):
    """Update only derived guard metadata of this program's existing report."""
    report = json.loads(path.read_text())
    if report.get('method') != 'rigid 4DoF xyz+yaw placement, block Huber':
        raise ValueError('Not an explore_substairs_attachment report')
    checks, failures = geometry_guards(report['pair_evaluations'])
    report['geometry_guard_evaluation'] = checks
    report['validation_failures'] = [r for r in report['validation_failures'] if not r.startswith('Geometry guard ')] + failures
    report['accepted_by_validation'] = False
    report['navigation_ready'] = False
    report['guard_metadata_update'] = {
        'utc': utc_now(), 'method': 'Evaluate cached direct/placed geometry only; no solver, pose, or cloud rerun',
        'source_sha256': sha256_file(Path(__file__)), 'original_run_source_sha256_retained': True,
        'command': sys.argv}
    path.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'updated': str(path), 'additional_failures': failures}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/student_union_20261017_all_preview/substairs'))
    parser.add_argument('--refresh-geometry-guards', type=Path,
                        help='Reevaluate existing report geometry guards only; preserve poses/clouds')
    args = parser.parse_args()
    if args.refresh_geometry_guards:
        refresh_geometry_guards(args.refresh_geometry_guards.resolve())
        return
    output = args.output.resolve()
    if output.exists():
        raise ValueError('Use fresh output: '+str(output))
    output.mkdir(parents=True)
    root = Path('outputs/student_union_20261017_revalidated').resolve()
    config = json.loads(Path('config/student_union_all_sessions.json').read_text())['sessions']
    paths = {s: root/'joint_disjoint_hypothesis/component_0'/s/'candidate_trajectory.csv' for s in ['lobby', 'floor2']}
    paths['bridge_12_sub'] = root/'intra_session/bridge_12_sub/candidate_nominal/main_trajectory.csv'
    reference = {s: load_pose_csv(paths[s]) for s in ['lobby', 'floor2']}
    source = load_pose_csv(paths['bridge_12_sub'])
    inputs = [root/'cross_session_planar_interior'/p/'accepted_frame_edges.json'
              for p in ['lobby__bridge_12_sub', 'floor2__bridge_12_sub']]
    rows = [dict(row, provenance=str(p)) for p in inputs for row in json.loads(p.read_text())]
    secondary_path = root/'cross_session_clahe_depth/lobby__bridge_12_sub/accepted_frame_edges.json'
    rows += [dict(row, provenance=str(secondary_path)) for row in json.loads(secondary_path.read_text()) if row['node_a'] == 443]
    # One geometrically verified pair per physical region, not two nearby door
    # views at 2F counted as independent. Test pairs are never used by fit().
    train_ids = {'lobby:462__bridge_12_sub:36', 'floor2:553__bridge_12_sub:159'}
    training = [r for r in rows if r['edge_id'] in train_ids]
    placement, solver = fit(training, reference, source)
    alternatives = {}
    for name, support in [('lobby_anchor_only', training[:1]), ('floor2_anchor_only', training[1:])]:
        t, stats = fit(support, reference, source)
        alternatives[name] = {'T_joint_sub': t.tolist(), 'solver': stats,
                              'all_pair_errors': {r['edge_id']: residual(r, t, reference, source).tolist() for r in rows}}
    db_sessions = {s: Session(Path(config[s]['database']), s) for s in paths}
    evaluations = []
    for row in rows:
        err = residual(row, placement, reference, source)
        implied = reference[row['session_a']][row['node_a']] @ np.asarray(row['T_a_b']) @ inverse(source[row['node_b']])
        a, b = db_sessions[row['session_a']].cloud(row['node_a']), db_sessions['bridge_12_sub'].cloud(row['node_b'])
        predicted = inverse(reference[row['session_a']][row['node_a']]) @ placement @ source[row['node_b']]
        evaluations.append({'edge_id': row['edge_id'], 'role': 'training' if row['edge_id'] in train_ids else 'audit_not_used_in_fit',
                            'source_file': row['provenance'], 'original_eligible_for_joint': row['eligible_for_joint'],
                            'local_geometry_verified': row['local_geometry_verified'],
                            'translation_error_m': float(np.linalg.norm(err[:3])),
                            'rotation_error_deg': float(np.rad2deg(np.linalg.norm(err[3:]))),
                            'signed_error_translation_in_measurement_frame_m': err[:3].tolist(),
                            'T_joint_sub_implied_by_pair': implied.tolist(),
                            'source_camera_z_m': float(source[row['node_b']][2, 3]),
                            'reference_camera_z_m': float(reference[row['session_a']][row['node_a']][2, 3]),
                            'direct_measurement_geometry': geometric_score(a, b, np.asarray(row['T_a_b']), threshold=.20),
                            'placed_geometry': geometric_score(a, b, predicted, threshold=.20)})
    transformed = {node: placement @ pose for node, pose in source.items()}
    trajectory(output/'candidate_trajectory.csv', transformed, db_sessions['bridge_12_sub'].stamps)
    source_cloud = root/'intra_session/bridge_12_sub/candidate_nominal/cloud.ply'
    cloud = read_ply(source_cloud)
    xyz = np.column_stack([cloud[k] for k in ['x', 'y', 'z']])
    moved = xyz @ placement[:3, :3].T + placement[:3, 3]
    colors = np.column_stack([cloud[k] for k in ['r', 'g', 'b']])
    write_binary_ply(output/'candidate_cloud.ply', moved, colors)
    failures = ['Only two training regions; fewer than three independent cross-session supports',
                'No independent withheld physical region: 2F test shares door and 1F legacy test has ICP pixel leakage',
                'Parent lobby-floor2 component still failed its 0.15m held-out translation gate']
    if max(e['translation_error_m'] for e in evaluations) > .35:
        failures.append('At least one local correspondence has rigid placement translation error above 0.35m')
    if max(e['rotation_error_deg'] for e in evaluations) > 8.:
        failures.append('At least one rotation error above 8 degrees')
    geometry_checks, geometry_failures = geometry_guards(evaluations)
    failures.extend(geometry_failures)
    hashes = {str(p): sha256_file(p) for p in [Path(__file__).resolve(), *paths.values(), *inputs, secondary_path, source_cloud]}
    result = {'created_utc': utc_now(), 'status': 'UNVALIDATED_PREVIEW_NOT_FOR_NAVIGATION',
              'accepted_by_validation': False, 'navigation_ready': False, 'method': 'rigid 4DoF xyz+yaw placement, block Huber',
              'parameters': {'translation_sigma_m': .10, 'rotation_sigma_deg': 3., 'block_huber_delta': 3.,
                             'max_nfev': 250, 'ftol_xtol_gtol': 1e-11, 'fixed_roll_pitch_deg': [0., 0.]},
              'source_pose_csv': str(paths['bridge_12_sub']), 'reference_pose_csvs': {s: str(paths[s]) for s in reference},
              'source_cloud': str(source_cloud), 'T_joint_sub': placement.tolist(),
              'transform_convention': 'T_joint_sub maps the selected substairs session map into the existing lobby-rooted joint preview frame',
              'training_pair_count': len(training), 'audit_pair_count': len(rows)-len(training),
              'independent_training_regions': 2, 'independent_withheld_regions': 0,
              'solver': solver, 'pair_evaluations': evaluations, 'anchor_sensitivity': alternatives,
              'validation_failures': failures, 'geometry_guard_evaluation': geometry_checks,
              'visual_review': ['1F lobby 462 / sub 36: same stair and wall; repeated treads remain an alias risk.',
                                '2F 553 / sub 159 and 560 / sub 162: same door, signboard and paper notices; correlated nearby views.',
                                '1F lobby 443 / sub 60: same elevator surround; moving floor indicator is not static evidence.'],
              'height_preservation': {'source_camera_z_range_m': float(np.ptp([p[2, 3] for p in source.values()])),
                                      'placed_camera_z_range_m': float(np.ptp([p[2, 3] for p in transformed.values()])),
                                      'vertical_scale': 1., 'floor_flattening': False, 'internal_pose_changes': False},
              'output_cloud_points': len(cloud), 'source_sha256': hashes,
              'command': sys.argv, 'limitation': 'Placement of independently rebuilt points, not new full graph optimization; no new correspondences or covariance calibration.'}
    (output/'attachment_report.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    for label, poses in [*reference.items(), ('substairs_preview', transformed)]:
        values = np.array([p[:3, 3] for p in poses.values()])
        for ax, cols in zip(axes, [(0, 1), (0, 2)]):
            ax.plot(values[:, cols[0]], values[:, cols[1]], lw=.8, label=label)
            ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.2)
    axes[0].set(xlabel='X (m)', ylabel='Y (m)'); axes[1].set(xlabel='X (m)', ylabel='Camera Z (m)')
    axes[1].legend(fontsize=8)
    fig.suptitle('UNVALIDATED rigid substairs placement | internal stair height preserved')
    fig.savefig(output/'placement_trajectories.png', dpi=170); plt.close(fig)
    for session in db_sessions.values():
        session.connection.close()
    print(json.dumps({'output': str(output), 'T_joint_sub': placement.tolist(), 'pairs': evaluations,
                      'validation_failures': failures}, indent=2))


if __name__ == '__main__':
    main()
