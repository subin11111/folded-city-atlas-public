#!/usr/bin/env python3
"""Reproducible geometry experiments with frozen baselines and explicit audits.

Every original frame is a graph node. Measured relative-pose edges use block
Huber loss. The SE(3) ablation adds an original-attitude regularizer and sparse
observed horizontal normals; there is deliberately no common floor height.
Unsupported sessions remain separate graph components, even when placed in a
common exploratory display frame. This program never grants navigation approval.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
import shutil
import platform

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from check_session_overlap import Session
from evaluate_map_surfaces import Settings, fit_planes
from export_rtabmap_db import load_pose_csv
from joint_optimize_sessions import initialize_maps, records, residual_rows
from optimize_session import (Clouds, Edge, apply, block_huber, connected_components,
    geometric_score, inverse, optimize, sha256_file, trajectory, utc_now, write_csv)

ROOT = Path(__file__).resolve().parents[1]
PRIOR = ROOT / 'outputs/student_union_20261017_revalidated'
PREVIEW = ROOT / 'outputs/student_union_20261017_all_preview/bundle'


def normal_residual(rotations, node_indices, normals, sigma_degrees=3.):
    """Orientation-only factor: completely independent of camera translations."""
    predicted = np.einsum('nij,nj->ni', rotations[node_indices], normals)
    predicted *= np.where(predicted[:, 2:3] < 0, -1., 1.)
    return (predicted - [0., 0., 1.]) / np.radians(sigma_degrees)


def paired_plane_residual(ra, ta, rb, tb, normal_a, normal_b, center_a, center_b,
                          distance_sigma=.08, angle_sigma_deg=3.):
    """Finite same-plane observations constrain normals/distance, not tangent shift."""
    na = np.einsum('nij,nj->ni', ra, normal_a)
    nb = np.einsum('nij,nj->ni', rb, normal_b)
    nb *= np.where(np.sum(na*nb, axis=1, keepdims=True) < 0, -1., 1.)
    ca = np.einsum('nij,nj->ni', ra, center_a) + ta
    cb = np.einsum('nij,nj->ni', rb, center_b) + tb
    delta = cb-ca
    distances = np.column_stack((np.sum(delta*na, axis=1), np.sum(delta*nb, axis=1))) / (distance_sigma*np.sqrt(2.))
    return np.column_stack((distances, (na-nb)/np.radians(angle_sigma_deg)))


def optimize_normals(initial, edges, components, normals, max_evaluations=200, paired_planes=None):
    paired_planes = paired_planes or []
    ids = sorted(initial)
    lookup = {n: i for i, n in enumerate(ids)}
    anchors = {c[0] for c in components}
    variable = [n for n in ids if n not in anchors]
    vi = np.array([lookup[n] for n in variable], int)
    vmap = {n: i for i, n in enumerate(variable)}
    active = [e for e in edges if not e.held_out]
    ai = np.array([lookup[e.a] for e in active], int)
    bi = np.array([lookup[e.b] for e in active], int)
    ri = np.array([initial[n][:3, :3] for n in ids])
    ti = np.array([initial[n][:3, 3] for n in ids])
    mr = np.array([e.measurement[:3, :3].T for e in active])
    mt = np.array([e.measurement[:3, 3] for e in active])
    whitening = np.array([np.linalg.cholesky(e.effective_information).T for e in active])
    ni = np.array([lookup[n['global_id']] for n in normals], int)
    nn = np.array([n['normal_local'] for n in normals], float).reshape(-1, 3)
    up0 = ri.transpose(0, 2, 1)[:, :, 2]
    pa = np.array([lookup[p['global_a']] for p in paired_planes], int)
    pb = np.array([lookup[p['global_b']] for p in paired_planes], int)
    plane_arrays = [np.array([p[k] for p in paired_planes], float).reshape(-1, 3)
                    for k in ('normal_a', 'normal_b', 'center_a', 'center_b')]

    def decode(x):
        correction = np.zeros((len(ids), 6))
        correction[vi] = x.reshape(-1, 6)
        return Rotation.from_rotvec(correction[:, 3:]).as_matrix() @ ri, ti + correction[:, :3]

    def residual(x):
        r, t = decode(x)
        inv_a = r[ai].transpose(0, 2, 1)
        pr = inv_a @ r[bi]
        pt = np.einsum('nij,nj->ni', inv_a, t[bi] - t[ai])
        er = mr @ pr
        et = np.einsum('nij,nj->ni', mr, pt - mt)
        errors = np.column_stack((et, Rotation.from_matrix(er).as_rotvec()))
        factors = block_huber(np.einsum('nij,nj->ni', whitening, errors), 3.).ravel()
        prior = ((r.transpose(0, 2, 1)[vi, :, 2] - up0[vi]) / np.radians(2.)).ravel()
        floor = block_huber(normal_residual(r, ni, nn), 2.).ravel()
        pairs = block_huber(paired_plane_residual(r[pa], t[pa], r[pb], t[pb], *plane_arrays), 3.).ravel()
        return np.r_[factors, prior, floor, pairs]

    sparsity = lil_matrix((len(active)*6 + len(variable)*3 + len(normals)*3 + len(paired_planes)*5, len(variable)*6), dtype=np.int8)
    for k, e in enumerate(active):
        for n in (e.a, e.b):
            if n in vmap:
                j = vmap[n]*6
                sparsity[k*6:(k+1)*6, j:j+6] = 1
    offset = len(active)*6
    for j in range(len(variable)):
        sparsity[offset+j*3:offset+(j+1)*3, j*6+3:j*6+6] = 1
    offset += len(variable)*3
    for k, row in enumerate(normals):
        if row['global_id'] in vmap:
            j = vmap[row['global_id']]*6
            sparsity[offset+k*3:offset+(k+1)*3, j+3:j+6] = 1
    offset += len(normals)*3
    for k, plane in enumerate(paired_planes):
        for node in (plane['global_a'], plane['global_b']):
            if node in vmap:
                j = vmap[node]*6
                sparsity[offset+k*5:offset+(k+1)*5, j:j+6] = 1
    x0 = np.zeros(len(variable)*6)
    limits = np.tile([3., 3., 1., np.radians(5.), np.radians(5.), np.radians(20.)], len(variable))
    start = time.monotonic()
    fit = least_squares(residual, x0, jac_sparsity=sparsity.tocsr(), bounds=(-limits, limits),
        method='trf', tr_solver='lsmr', x_scale='jac', max_nfev=max_evaluations,
        ftol=1e-6, xtol=1e-7, gtol=1e-6, tr_options={'atol': 1e-7, 'btol': 1e-7, 'maxiter': 300})
    r, t = decode(fit.x)
    result = {}
    for k, n in enumerate(ids):
        result[n] = np.eye(4)
        result[n][:3, :3], result[n][:3, 3] = r[k], t[k]
    tilt = np.degrees(np.arccos(np.clip(np.sum(r.transpose(0, 2, 1)[:, :, 2]*up0, axis=1), -1, 1)))
    report = {'success': bool(fit.success), 'nfev': int(fit.nfev), 'message': fit.message,
        'initial_cost': float(np.sum(residual(x0)**2)/2), 'final_cost': float(fit.cost),
        'active_bounds': int(np.count_nonzero(fit.active_mask)), 'maximum_tilt_change_deg': float(tilt.max()),
        'physical_tilt_guard_passed': bool(tilt.max() <= 5. + 1e-6),
        'elapsed_seconds': time.monotonic()-start, 'active_relative_edges': len(active),
        'normal_factors': len(normals), 'attitude_prior_sigma_deg': 2., 'normal_sigma_deg': 3.,
        'attitude_source': 'selected baseline attitude regularization, not a newly measured IMU signal',
        'plane_height_factors': sum(p.get('kind') == 'horizontal' for p in paired_planes),
        'absolute_global_floor_height_factors': 0, 'same_finite_plane_factors': len(paired_planes),
        'same_plane_sigma_m_deg': [.08, 3.],
        'loss': 'edge and same-plane block Huber delta=3; horizontal normal block Huber delta=2'}
    return result, report


def edge_from_row(row, index):
    st = max(.10, float(row.get('sigma_translation_m', .10)))
    sr = np.radians(max(3., float(row.get('sigma_rotation_deg', 3.))))
    info = np.diag([st**-2]*3 + [sr**-2]*3)
    return Edge(index[(row['session_a'], row['node_a'])], index[(row['session_b'], row['node_b'])],
        3, np.asarray(row['T_a_b']), info, info, bool(row.get('held_out', False)))


def approved_local_rows(paths):
    result = []
    for path in paths:
        for row in records(path):
            if not (row.get('local_geometry_verified') is True or row.get('accepted') is True):
                continue
            if 'T_a_b' not in row:
                continue
            result.append(dict(row, source_file=str(path)))
    return result


def reserve_native_edges(stored, reserved_keys):
    """A newly reserved pair cannot retain an alternate native training factor."""
    result = {}
    for name, groups in stored.items():
        result[name] = {}
        for kind, edges in groups.items():
            if kind not in ('odometry', 'verified_loops'):
                result[name][kind] = edges
                continue
            result[name][kind] = [dict(e, held_out=True) if tuple(sorted(((name, e['from_id']),
                (name, e['to_id'])))) in reserved_keys else dict(e) for e in edges]
    return result


def cross_audit_failures(rows):
    """Audit every connection, including ones with no reserved measurement."""
    groups = {}
    for row in rows:
        groups.setdefault(tuple(sorted((row['session_a'], row['session_b']))), []).append(row)
    failures = []
    if not groups:
        return ['missing_whole_pair_cross_audit']
    for pair, connection in groups.items():
        group = [row for row in connection if row.get('held_out')]
        if not group:
            failures.append('__'.join(pair)+'_missing_whole_pair_cross_audit')
            continue
        for field, median, maximum in [('translation_m', .15, .35), ('rotation_deg', 3., 8.)]:
            values = np.array([r[field] for r in group], float)
            if not np.isfinite(values).all() or np.median(values) > median or values.max() > maximum:
                failures.append('__'.join(pair)+'_cross_audit_'+field+'_failed')
    return failures


def optimize_four_dof(initial, edges, components, max_evaluations=200):
    """The legacy solver does not filter odometry holdouts; filter at entry."""
    return optimize(initial, [e for e in edges if e.type == 0 and not e.held_out],
                    [e for e in edges if e.type != 0], components,
                    max_evaluations=max_evaluations)


def collect_normals(names, sessions, local, index):
    normals, diagnostic = [], []
    settings = Settings(max_planes_per_kind=2)
    for name in names:
        # Exterior and entrance bridge may include a ramp; no horizontal assumption there.
        if name not in ('lobby', 'floor2', 'bridge_12_sub'):
            continue
        clouds = Clouds(sessions[name].connection, None)
        for node in sorted(local[name])[::12]:
            train, test = clouds.get(node)
            rotation = local[name][node][:3, :3]
            planes, _ = fit_planes(train @ rotation.T, test @ rotation.T,
                                  'horizontal', settings, 9827 + node)
            eligible = [p for p in planes if -.4 > p['center'][2] > -2.4]
            if eligible:
                p = max(eligible, key=lambda p: len(p['training']))
                normal = rotation.T @ p['normal']
                normals.append({'session': name, 'node': node, 'global_id': index[(name, node)],
                    'normal_local': normal.tolist(), 'center_local': (rotation.T @ p['center']).tolist(),
                    'training_points': len(p['training']), 'reserved_points': len(p['testing']),
                    'assumption': 'below-camera observed horizontal patch, not a verified floor label'})
            diagnostic.append({'session': name, 'node': node, 'patches': len(planes), 'eligible': len(eligible)})
            clouds.cache.clear()
        print(f'normals {name}: {sum(n["session"] == name for n in normals)}', flush=True)
    return normals, diagnostic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baselines', type=Path, required=True)
    parser.add_argument('--extra-edges', type=Path, action='append', default=[])
    parser.add_argument('--plane-constraints', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-evaluations', type=int, default=200)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Choose a new experiment directory')
    args.output.mkdir(parents=True)
    shutil.copyfile(Path(__file__), args.output/'source_snapshot.py')
    config = json.loads((ROOT/'config/student_union_all_sessions.json').read_text())['sessions']
    selection = json.loads(args.baselines.read_text())['sessions']
    local = {n: load_pose_csv(Path(s['selected_pose_csv'])) for n, s in selection.items()}
    sessions = {n: Session(Path(config[n]['database']), n) for n in local}
    index = {(n, k): i+1 for i, (n, k) in enumerate((n, k) for n in local for k in sorted(local[n]))}
    reverse = {v: k for k, v in index.items()}
    stored = {n: json.loads((Path(config[n]['reviewed_dir'])/'validated_edges.json').read_text()) for n in local}
    cross_paths = [PRIOR/'cross_session/disjoint_hypothesis/drift_hypothesis_edges.json',
        PRIOR/'cross_session_planar_interior/lobby__bridge_12_sub/accepted_frame_edges.json',
        PRIOR/'cross_session_planar_interior/floor2__bridge_12_sub/accepted_frame_edges.json']
    cross = approved_local_rows(cross_paths)
    # Preserve the old unused 2F substairs pair as an audit, not a third support region.
    for row in cross:
        if row['session_b'] == 'bridge_12_sub':
            row['held_out'] = row['node_a'] == 560
            row['initialization_allowed'] = not row['held_out']
    extra = approved_local_rows(args.extra_edges)
    intra = approved_local_rows([PRIOR/'intra_session/constraints.json'])
    rows = cross + intra + extra
    original_holdout = {(n, min(e['from_id'], e['to_id']), max(e['from_id'], e['to_id']))
        for n in local for e in stored[n]['verified_loops'] if e.get('held_out')}
    unique = {}
    for r in rows:
        a, b = (r['session_a'], int(r['node_a'])), (r['session_b'], int(r['node_b']))
        if a not in index or b not in index:
            continue
        if a[0] == b[0] and (a[0], min(a[1], b[1]), max(a[1], b[1])) in original_holdout:
            r = dict(r, held_out=True, initialization_allowed=False)
        key = tuple(sorted((a, b)))
        if key in unique and unique[key].get('held_out'):
            r = dict(r, held_out=True, initialization_allowed=False)
        unique[key] = r
    rows = list(unique.values())
    reserved_keys = {tuple(sorted(((r['session_a'], r['node_a']), (r['session_b'], r['node_b']))))
                     for r in rows if r.get('held_out')}
    stored = reserve_native_edges(stored, reserved_keys)
    plane_rows = []
    if args.plane_constraints:
        payload = json.loads(args.plane_constraints.read_text())
        for plane in payload['constraints']:
            if plane.get('accepted_for_experiment') is not True:
                continue
            a, b = (plane['session_a'], plane['node_a']), (plane['session_b'], plane['node_b'])
            if a[0] != b[0]:
                raise ValueError('This experiment only accepts intra-session finite-plane factors')
            if a not in index or b not in index:
                continue
            if tuple(sorted((a, b))) in reserved_keys or (a[0] == b[0] and (a[0], min(a[1], b[1]), max(a[1], b[1])) in original_holdout):
                raise ValueError('A reserved pair cannot supply training plane factors')
            for key in ('normal_a', 'normal_b', 'center_a', 'center_b'):
                value = np.asarray(plane[key])
                if value.shape != (3,) or not np.isfinite(value).all():
                    raise ValueError('Invalid finite plane geometry')
                if key.startswith('normal') and abs(np.linalg.norm(value)-1.) > 1e-5:
                    raise ValueError('Plane normals must be unit vectors')
            plane_rows.append(dict(plane, global_a=index[a], global_b=index[b]))
    replacements = {tuple(sorted(((r['session_a'], r['node_a']), (r['session_b'], r['node_b']))))
                    for r in rows if not r.get('held_out')}
    edges = []
    for n in local:
        for kind in ('odometry', 'verified_loops'):
            for e in stored[n][kind]:
                a, b = (n, e['from_id']), (n, e['to_id'])
                if a not in index or b not in index:
                    continue
                if kind != 'odometry' and tuple(sorted((a, b))) in replacements:
                    continue
                edges.append(Edge(index[a], index[b], e['type'], np.asarray(e['measurement']),
                    np.asarray(e['information']), np.asarray(e['effective_information']), bool(e.get('held_out'))))
    edges += [edge_from_row(r, index) for r in rows]
    components = connected_components(list(reverse), [e for e in edges if not e.held_out])
    initial = {}
    component_names = []
    for component in components:
        names = sorted({reverse[g][0] for g in component})
        root = 'lobby' if 'lobby' in names else names[0]
        anchor = index[(root, min(local[root]))]
        if anchor in component:
            component.remove(anchor); component.insert(0, anchor)
        if len(names) > 1:
            placement = initialize_maps({n: local[n] for n in names},
                [r for r in rows if r['session_a'] in names and r['session_b'] in names
                 and r['session_a'] != r['session_b']], root)
        else:
            placement = {root: np.eye(4)}
        # Component gauge uses one old display anchor solely for comparison.
        old = load_pose_csv(PREVIEW/f'{root}_trajectory.csv')
        node = min(set(old) & set(local[root]))
        gauge = old[node] @ inverse(local[root][node])
        if np.degrees(np.arccos(np.clip(gauge[2, 2], -1., 1.))) > .1:
            raise ValueError('Display gauge tilts world up; local-normal target would be invalid')
        for g in component:
            n, k = reverse[g]
            initial[g] = gauge @ placement[n] @ local[n][k]
        component_names.append(names)
    normals, normal_diagnostics = collect_normals(local, sessions, local, index)
    (args.output/'normal_observations.json').write_text(json.dumps({'observations': normals, 'diagnostics': normal_diagnostics}, indent=2)+'\n')
    (args.output/'used_constraints.json').write_text(json.dumps(rows, indent=2)+'\n')
    report = {'started_utc': utc_now(), 'argv': sys.argv, 'components': component_names,
        'all_sessions_connected': len(components) == 1, 'navigation_ready': False,
        'source_hashes': {str(p): sha256_file(p) for p in [Path(__file__), args.baselines, *cross_paths, *args.extra_edges,
            PRIOR/'intra_session/constraints.json', ROOT/'config/student_union_all_sessions.json',
            *[ROOT/'src'/f for f in ('optimize_session.py', 'joint_optimize_sessions.py', 'evaluate_map_surfaces.py', 'check_session_overlap.py')],
            *[Path(s['selected_pose_csv']) for s in selection.values()],
            *[Path(config[n]['reviewed_dir'])/'validated_edges.json' for n in local],
            *[PREVIEW/f'{n}_trajectory.csv' for n in local], *([args.plane_constraints] if args.plane_constraints else [])]},
        'database_hashes_before': {n: sha256_file(s.path) for n, s in sessions.items()},
        'baseline_selection': selection, 'variants': {}, 'relative_edges': len(edges),
        'local_normal_factors': len(normals), 'new_independent_imu_factors': 0,
        'normal_height_factors': 0, 'cross_region_minimum_target': 3,
        'same_finite_plane_factors': len(plane_rows),
        'software': {'python': platform.python_version(), 'numpy': np.__version__,
                     'scipy': __import__('scipy').__version__, 'opencv': __import__('cv2').__version__},
        'parameters': {'normal_frame_stride': 12, 'plane_fit': vars(Settings(max_planes_per_kind=2)),
                       'normal_below_camera_interval_m': [-2.4, -.4],
                       'excluded_normal_sessions': ['outside', 'bridge_out1f'],
                       'max_evaluations': args.max_evaluations,
                       'new_edge_minimum_sigma_translation_m': .10, 'new_edge_minimum_sigma_rotation_deg': 3.,
                       'edge_sigma_source': 'explicit modeling assumption, not sensor-calibrated covariance'},
        'limitations': ['Old whole-link audits influenced past policy choices; not fresh generalization data.',
                       'No surveyed dimensions or floor elevations provided.',
                       'Below-camera horizontal patch factors assume horizontality; not semantic floor truth.',
                       'Locally verified substairs links are hypotheses without enough independent regions.']}
    variants = {'initial': initial}
    for n in local:
        (args.output/n).mkdir()
        trajectory(args.output/n/'initial.csv', {k: initial[index[(n, k)]] for k in local[n]}, sessions[n].stamps)
    print('starting 4DoF full graph', flush=True)
    candidate, solver = optimize_four_dof(initial, edges, components,
                                         max_evaluations=args.max_evaluations)
    variants['pgo4'] = candidate; report['variants']['pgo4'] = {'solver': solver}
    for n in local:
        trajectory(args.output/n/'pgo4.csv', {k: candidate[index[(n, k)]] for k in local[n]}, sessions[n].stamps)
    candidates = [('pgo6_attitude', [], []), ('pgo6_normals', normals, [])]
    if plane_rows:
        candidates.append(('pgo6_planes', normals, plane_rows))
    for label, plane_factors, same_planes in candidates:
        print('starting '+label+', from same initial graph', flush=True)
        candidate, solver = optimize_normals(initial, edges, components, plane_factors, args.max_evaluations, same_planes)
        variants[label] = candidate; report['variants'][label] = {'solver': solver}
        for n in local:
            trajectory(args.output/n/(label+'.csv'), {k: candidate[index[(n, k)]] for k in local[n]}, sessions[n].stamps)
    geometry, residuals = [], []
    clouds = {n: Clouds(s.connection, None) for n, s in sessions.items()}
    checks = []
    for n in local:
        for e in stored[n]['verified_loops']:
            if e.get('held_out'):
                checks.append((n, e['from_id'], n, e['to_id'], 'original_whole_pair_audit'))
        ids = sorted(local[n])
        checks += [(n, ids[i], n, ids[i+1], 'adjacent_audit') for i in range(0, len(ids)-1, max(1, len(ids)//35))]
    checks += [(r['session_a'], r['node_a'], r['session_b'], r['node_b'],
                'cross_audit' if r.get('held_out') else 'cross_training_diagnostic') for r in rows if r['session_a'] != r['session_b']]
    for sa, a, sb, b, role in checks:
        if (sa, a) not in index or (sb, b) not in index:
            continue
        ca, cb = clouds[sa].get(a)[1], clouds[sb].get(b)[1]
        for label, poses in variants.items():
            score = geometric_score(ca, cb, inverse(poses[index[(sa, a)]]) @ poses[index[(sb, b)]], threshold=.20)
            geometry.append(dict(session_a=sa, node_a=a, session_b=sb, node_b=b, role=role, variant=label, **score))
    for label, poses in variants.items():
        residuals += [dict(r, variant=label) for r in residual_rows([r for r in rows if r['session_a'] != r['session_b']], poses, index)]
    write_csv(args.output/'geometry_audits.csv', geometry)
    write_csv(args.output/'cross_residuals.csv', residuals)
    for label in report['variants']:
        rr = [r for r in residuals if r['variant'] == label and r['held_out']]
        failures = []
        if not report['variants'][label]['solver']['success']:
            failures.append('solver_not_converged')
        solver = report['variants'][label]['solver']
        if solver.get('active_bounds', 0):
            failures.append('active_correction_bounds')
        if solver.get('physical_tilt_guard_passed') is False:
            failures.append('physical_tilt_guard_failed')
        if solver.get('final_cost', solver.get('final_block_huber_cost', 0)) > solver.get('initial_cost', solver.get('initial_block_huber_cost', 0)) + 1e-8:
            failures.append('objective_increased')
        if len(components) > 1:
            failures.append('all_sessions_not_connected_by_verified_measurements')
        failures += cross_audit_failures([r for r in residuals if r['variant'] == label])
        # Region coverage is unproven for old substairs local evidence, regardless of fit.
        failures.append('independent_region_coverage_not_established_for_all_connections')
        by_session = {}
        for n in local:
            by_session[n] = {}
            for role in ('original_whole_pair_audit', 'adjacent_audit'):
                before = [r for r in geometry if r['session_a'] == n and r['session_b'] == n and r['role'] == role and r['variant'] == 'initial']
                after = [r for r in geometry if r['session_a'] == n and r['session_b'] == n and r['role'] == role and r['variant'] == label]
                if before:
                    values = {'pairs': len(before), 'before_rmse_m': float(np.median([r['clipped_rmse_m'] for r in before])),
                        'after_rmse_m': float(np.median([r['clipped_rmse_m'] for r in after])),
                        'before_overlap': float(np.median([r['overlap'] for r in before])),
                        'after_overlap': float(np.median([r['overlap'] for r in after]))}
                    values['passed_regression_guard'] = values['after_rmse_m'] <= values['before_rmse_m']+.005 and values['after_overlap'] >= values['before_overlap']-.03
                    if not values['passed_regression_guard']:
                        failures.append(n+'_'+role+'_regression')
                    by_session[n][role] = values
        report['variants'][label].update(accepted=False, failures=failures, cross_whole_pair_audits=rr,
            session_geometry=by_session, heights={n: {'before_camera_z_range_m': float(np.ptp([initial[index[(n, k)]][2, 3] for k in local[n]])),
            'after_camera_z_range_m': float(np.ptp([variants[label][index[(n, k)]][2, 3] for k in local[n]])),
            'surveyed_floor_height': None} for n in local})
    report['database_hashes_after'] = {n: sha256_file(s.path) for n, s in sessions.items()}
    report['originals_unchanged'] = report['database_hashes_before'] == report['database_hashes_after']
    report['finished_utc'] = utc_now()
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    for session in sessions.values():
        session.connection.close()
    print(json.dumps({'output': str(args.output), 'components': component_names, 'originals_unchanged': report['originals_unchanged']}, indent=2))


if __name__ == '__main__':
    main()
