#!/usr/bin/env python3
"""Independently remeasure stored intra-session loops with RGB-D evidence.

T_a_b maps node b points into node a. Neither stored link poses nor map poses
initialize the visual fit. Reserved feature correspondences, including their
depth-pixel neighbourhoods, never enter RANSAC/refit or ICP. This is a sample
holdout, not an independent sensor or surveyed-ground-truth experiment.

The nominal 0.10 m / 3 degree and weak 0.20 m / 5 degree covariances are explicit
isotropic modelling assumptions, NOT calibrated sensor uncertainty. They are
only offered for pairs passing visual, 3-D support, holdout and bidirectional
ICP gates. Existing whole-pair graph holdouts are never promoted to training.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import shlex
import sys
import time

import cv2
import numpy as np
import scipy
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from check_session_overlap import (DEFAULTS, Session, apply_transform,
    geometry_support, mutual_ratio_matches, refine_icp, rigid_ransac)
from export_rtabmap_db import load_pose_csv
from optimize_session import (Clouds, Edge, checked_transform, connected_components,
    error_statistics, geometry_metrics, inverse, optimize, read_edges,
    relative_error, saved_app_poses, sha256_file, trajectory, utc_now)


ROOT = Path(__file__).resolve().parents[1]
SPECS = {
    'outside': ('261016-234854-outside.db', 'student_union_20261017_reviewed'),
    'lobby': ('261017-000622-1f.db', 'student_union_20261017_reviewed'),
    'bridge_out1f': ('261017-011331-bridgeout1f.db', 'student_union_20261017_added_reviewed'),
    'floor2': ('261017-013501-2f.db', 'student_union_20261017_added_reviewed'),
    'bridge_12_sub': ('261017-013904-bridg12_2.db', 'student_union_20261017_added_reviewed'),
}
SETTINGS = {
    **DEFAULTS,
    'holdout_depth_exclusion_radius_pixels': 3,
    'minimum_icp_cloud_points': 100,
    'maximum_icp_cycle_translation_m': .10,
    'maximum_icp_cycle_rotation_deg': 2.,
    'nominal_sigma_translation_m': .10,
    'nominal_sigma_rotation_deg': 3.,
    'weak_sigma_translation_m': .20,
    'weak_sigma_rotation_deg': 5.,
    'maximum_tilt_disagreement_deg': 10.,
    'new_graph_holdout_modulus': 5,
    'planar_aware_max_rotation_normal_condition': 100.,
    'planar_aware_min_rotation_normal_eigenvalue_per_point_m2': .020,
    'planar_aware_note': 'Third spatial standard deviation is diagnostic only: noncollinear fixed 3D correspondences on a plane determine a rigid transform. This differs from untextured point-to-plane ICP.',
}


def delta_summary(first, second):
    delta = inverse(first) @ second
    return {'translation_m': float(np.linalg.norm(delta[:3, 3])),
            'rotation_deg': float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()))}


def depth_pixels(session, node, pixels):
    h, w = session.depth(node)[0].shape
    ih, iw = session.image(node).shape[:2]
    return np.rint(np.asarray(pixels) * [w / iw, h / ih]).astype(int)


def exclusion_mask(shape, centers, radius):
    """Closed Chebyshev neighbourhood, including all 3x3 median-depth inputs."""
    mask = np.zeros(shape, dtype=bool)
    h, w = shape
    for x, y in np.asarray(centers, dtype=int).reshape(-1, 2):
        if 0 <= x < w and 0 <= y < h:
            mask[max(0, y-radius):min(h, y+radius+1), max(0, x-radius):min(w, x+radius+1)] = True
    return mask


def split_correspondences(pixels_a, pixels_b, shape_a, shape_b, seed):
    """Deduplicate depth samples; reserve random features; remove train leakage."""
    used_a, used_b, unique = set(), set(), []
    for i, (a, b) in enumerate(zip(pixels_a, pixels_b)):
        aa, bb = tuple(a), tuple(b)
        if aa in used_a or bb in used_b:
            continue
        unique.append(i)
        used_a.add(aa)
        used_b.add(bb)
    order = np.random.default_rng(seed).permutation(unique)
    count = min(len(order), max(5, int(np.ceil(len(order)*DEFAULTS['heldout_fraction']))))
    holdout = order[:count]
    masks = [exclusion_mask(shape, pix[holdout], SETTINGS['holdout_depth_exclusion_radius_pixels'])
             for shape, pix in ((shape_a, pixels_a), (shape_b, pixels_b))]
    # A >3 pixel centre separation also prevents 3x3 depth-patch overlap.
    train = np.asarray([i for i in order[count:]
        if not masks[0][pixels_a[i, 1], pixels_a[i, 0]]
        and not masks[1][pixels_b[i, 1], pixels_b[i, 0]]], dtype=int)
    return train, holdout, masks


def cloud_without_holdout(session, node, excluded):
    depth, confidence, cw, ch, k, local = session.depth(node)
    h, w = depth.shape
    vv, uu = np.mgrid[0:h:2, 0:w:2]
    z = depth[::2, ::2]
    valid = (np.isfinite(z) & (z >= DEFAULTS['min_depth_m']) &
             (z <= DEFAULTS['max_depth_m']) & (confidence[::2, ::2] >= 50) &
             ~excluded[::2, ::2])
    z = z[valid]
    optical = np.column_stack(((uu[valid]*cw/w-k[0, 2])*z/k[0, 0],
                               (vv[valid]*ch/h-k[1, 2])*z/k[1, 1], z))
    cloud = apply_transform(local, optical)
    _, indices = np.unique(np.floor(cloud / DEFAULTS['voxel_m']).astype(np.int64),
                           axis=0, return_index=True)
    return cloud[indices]


def holdout_score(transform, source, target):
    errors = np.linalg.norm(apply_transform(transform, source)-target, axis=1)
    inliers = errors < DEFAULTS['ransac_threshold_m']
    return {'count': len(errors), 'inliers': int(inliers.sum()),
            'inlier_ratio': float(inliers.mean()) if len(errors) else 0.,
            'rmse_m': float(np.sqrt(np.mean(errors**2))) if len(errors) else None,
            'median_m': float(np.median(errors)) if len(errors) else None,
            'errors_m': errors.tolist()}


def covariance_model(sigma_translation_m=.10, sigma_rotation_deg=3.):
    covariance = np.diag([sigma_translation_m**2]*3 + [np.radians(sigma_rotation_deg)**2]*3)
    return {'sigma_translation_m': sigma_translation_m,
            'sigma_rotation_deg': sigma_rotation_deg,
            'covariance_6x6': covariance.tolist(),
            'information_6x6': np.linalg.inv(covariance).tolist(),
            'tangent': 'right measurement error inv(T_a_b) @ inv(T_w_a) @ T_w_b; translation then rotation-vector',
            'source': 'explicit isotropic sensitivity model; not fitted/calibrated sensor uncertainty'}


def conditioning_diagnostic(points):
    """Centred point-to-point rotation information; diagnostics, not a covariance."""
    p = points - points.mean(axis=0)
    rotation_h = np.eye(3)*np.sum(p*p) - p.T @ p
    eigenvalues = np.linalg.eigvalsh(rotation_h)
    return {'centred_rotation_normal_eigenvalues_m2': eigenvalues.tolist(),
            'centred_rotation_normal_eigenvalues_per_point_m2': (eigenvalues/len(points)).tolist(),
            'rotation_normal_condition': float(eigenvalues[-1]/max(eigenvalues[0], 1e-15)),
            'note': 'Unweighted correspondence geometry only; ignores correlated depth noise and image association uncertainty.'}


def planar_aware_support(points, pixels, image_shape):
    """Fixed 3D-to-3D matches need noncollinearity, not volumetric support.

After eliminating the free translation by centring, the rotation normal matrix
is sum(||p||^2 I - pp^T). Its minimum eigenvalue is positive for a noncollinear
plane; rejecting every small third positional singular value would incorrectly
equate textured planar registration with point-to-plane ICP degeneracy.
"""
    old_accepted, diagnostic = geometry_support(points, pixels, image_shape)
    diagnostic.update(conditioning_diagnostic(points))
    diagnostic['previous_volume_gate_passed'] = bool(old_accepted)
    failures = []
    for ok, reason in (
        (diagnostic['std_m'][1] >= .15, 'second_spatial_std_below_0.15m'),
        (diagnostic['image_hull_fraction'] >= .04, 'image_hull_below_0.04'),
        (diagnostic['rotation_normal_condition'] <= SETTINGS['planar_aware_max_rotation_normal_condition'], 'rotation_normal_condition_above_100'),
        (diagnostic['centred_rotation_normal_eigenvalues_per_point_m2'][0] >= SETTINGS['planar_aware_min_rotation_normal_eigenvalue_per_point_m2'], 'rotation_normal_min_information_below_0.020m2_per_point')):
        if not ok:
            failures.append(reason)
    diagnostic['planar_aware_gate_failures'] = failures
    diagnostic['planar_aware_gate_passed'] = not failures
    diagnostic['third_std_role'] = 'diagnostic_only_not_rigid_fit_degeneracy'
    return not failures, diagnostic


def heldout_pair_key(a, b):
    return (min(a, b), max(a, b))


def graph_holdout(session, edge, old_holdouts, old_trained):
    pair = heldout_pair_key(edge.a, edge.b)
    if pair in old_holdouts:
        return True, 'preserved_previous_whole_pair_holdout'
    if pair in old_trained:
        return False, 'previous_training_pair'
    # Reserve newly introduced pairs before evaluating any of their metrics.
    import hashlib
    token = f'{session}:{pair[0]}:{pair[1]}:{DEFAULTS["seed"]}'.encode()
    reserved = int.from_bytes(hashlib.sha256(token).digest()[:4], 'little') % 5 == 0
    return reserved, 'new_pair_predeclared_hash_holdout' if reserved else 'new_pair_predeclared_hash_training'


def revalidate_pair(session, edge, output, held_out=False, role_reason=''):
    a, b = edge.a, edge.b
    record = {'session': session.label, 'a': a, 'b': b,
        'session_a': session.label, 'node_a': a, 'session_b': session.label, 'node_b': b,
        'link_type': edge.type, 'held_out': held_out, 'graph_role_reason': role_reason,
        'accepted': False, 'eligible_for_joint': False, 'reason': 'not_evaluated',
        'stored_T_a_b': edge.measurement.tolist(),
        'stored_information_6x6': edge.information.tolist(),
        'stored_formal_sigma_translation_xyz_m': np.sqrt(np.diag(np.linalg.inv(edge.information))[:3]).tolist(),
        'stored_formal_sigma_rotation_xyz_deg': np.degrees(np.sqrt(np.diag(np.linalg.inv(edge.information))[3:])).tolist(),
        'measurement_initialization': 'independent RGB-D 3D RANSAC; no stored or map transform used',
        'semantic_masks': 'absent; prior YOLO trial failed visual QC'}
    ka, da, _ = session.feature(a, full=True)
    kb, db, _ = session.feature(b, full=True)
    matches = mutual_ratio_matches(da, db, DEFAULTS['lowe_ratio'])
    record['mutual_visual_matches'] = len(matches)
    ia, ib = session.image(a), session.image(b)
    display = cv2.drawMatches(ia, ka, ib, kb, matches[:40], None,
                             flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(output/'matches'/f'{a}_{b}.jpg'), cv2.resize(display, (960, 640)))
    if len(matches) < DEFAULTS['minimum_mutual_matches']:
        record['reason'] = 'too_few_mutual_visual_matches'
        return record
    pixa = np.asarray([ka[m.queryIdx].pt for m in matches])
    pixb = np.asarray([kb[m.trainIdx].pt for m in matches])
    target, source = session.unproject(a, pixa), session.unproject(b, pixb)
    valid = np.isfinite(target).all(axis=1) & np.isfinite(source).all(axis=1)
    target, source, pixa, pixb = target[valid], source[valid], pixa[valid], pixb[valid]
    record['valid_depth_matches'] = len(source)
    if len(source) < DEFAULTS['minimum_depth_matches']:
        record['reason'] = 'too_few_valid_depth_matches'
        return record
    pixel_a, pixel_b = depth_pixels(session, a, pixa), depth_pixels(session, b, pixb)
    seed = DEFAULTS['seed'] + a*10007 + b
    train, holdout, masks = split_correspondences(pixel_a, pixel_b,
        session.depth(a)[0].shape, session.depth(b)[0].shape, seed)
    record.update({'seed': seed, 'train_count': len(train), 'heldout_count': len(holdout),
        'discarded_duplicate_or_holdout_neighbour_matches': len(source)-len(train)-len(holdout),
        'excluded_depth_pixels_a': int(masks[0].sum()), 'excluded_depth_pixels_b': int(masks[1].sum()),
        'depth_exclusion_radius_pixels': SETTINGS['holdout_depth_exclusion_radius_pixels'],
        'sample_holdout_scope': 'Feature indices and depth neighbourhoods excluded from RANSAC/refit/ICP. Nearby sensor samples remain correlated; not independent ground truth.'})
    evidence = output/'correspondences'/f'{a}_{b}.npz'
    np.savez_compressed(evidence, target=target, source=source, rgb_pixels_a=pixa,
        rgb_pixels_b=pixb, depth_pixels_a=pixel_a, depth_pixels_b=pixel_b,
        train_indices=train, heldout_indices=holdout)
    record['correspondence_evidence'] = str(evidence.resolve())
    if len(train) < DEFAULTS['minimum_train_inliers'] or len(holdout) < 5:
        record['reason'] = 'too_few_disjoint_training_or_heldout_matches'
        return record
    transform, inliers, trials = rigid_ransac(source[train], target[train], seed)
    record.update({'ransac_train_inliers': int(inliers.sum()),
        'ransac_train_inlier_ratio': float(inliers.mean()), 'ransac_valid_trials': trials,
        'visual_T_a_b': transform.tolist()})
    if inliers.sum() < 12 or inliers.mean() < .4:
        record['reason'] = 'insufficient_independent_3d_ransac_consensus'
        return record
    support = []
    for points, pixels, shape, label in ((target[train][inliers], pixa[train][inliers], ia.shape, 'a'),
                                       (source[train][inliers], pixb[train][inliers], ib.shape, 'b')):
        ok, diagnostic = planar_aware_support(points, pixels, shape)
        record['spatial_support_'+label] = diagnostic
        support.append(ok)
    # Always report holdout quality of the independently fitted visual transform,
    # even if the geometry later fails a degeneracy/ICP gate.
    record['visual_heldout'] = holdout_score(transform, source[holdout], target[holdout])
    if not all(support):
        record['reason'] = 'ill_conditioned_or_narrow_visual_support'
        return record
    cloud_a, cloud_b = cloud_without_holdout(session, a, masks[0]), cloud_without_holdout(session, b, masks[1])
    record['icp_cloud_points'] = [len(cloud_a), len(cloud_b)]
    if min(len(cloud_a), len(cloud_b)) < 100:
        record['reason'] = 'insufficient_holdout_excluded_icp_cloud'
        return record
    forward, fm = refine_icp(cloud_a, cloud_b, transform)
    reverse, rm = refine_icp(cloud_b, cloud_a, inverse(transform))
    cycle = delta_summary(np.eye(4), forward @ reverse)
    held = holdout_score(forward, source[holdout], target[holdout])
    reverse_held = holdout_score(reverse, target[holdout], source[holdout])
    world_error = session.raw_poses[a][:3, :3] @ forward[:3, :3] @ session.raw_poses[b][:3, :3].T
    tilt = float(np.degrees(np.arccos(np.clip(world_error[2, 2], -1, 1))))
    record.update({'forward_icp': fm, 'reverse_icp': rm, 'icp_cycle': cycle,
        'heldout': held, 'reverse_heldout': reverse_held,
        'T_a_b': forward.tolist(), 'T_b_a_independent_icp': reverse.tolist(),
        'stored_to_verified_delta': delta_summary(edge.measurement, forward),
        'raw_pose_relative_to_verified_delta': delta_summary(inverse(session.raw_poses[a]) @ session.raw_poses[b], forward),
        'irreducible_world_tilt_deg_for_4dof': tilt,
        'stored_holdout': holdout_score(edge.measurement, source[holdout], target[holdout]),
        'raw_pose_holdout': holdout_score(inverse(session.raw_poses[a]) @ session.raw_poses[b], source[holdout], target[holdout])})
    failures = []
    for name, score in (('forward', held), ('reverse', reverse_held)):
        if score['inliers'] < 5 or score['inlier_ratio'] < .5 or score['rmse_m'] > .07:
            failures.append(name+'_heldout_feature_validation_failed')
    for name, score in (('forward', fm), ('reverse', rm)):
        if score['symmetric_overlap'] < .4 or score['symmetric_rmse_m'] > .07:
            failures.append(name+'_icp_overlap_or_rmse_failed')
        if score['refinement_m'] > .15 or score['refinement_deg'] > 5:
            failures.append(name+'_icp_large_change_from_independent_visual_fit')
    if cycle['translation_m'] > .10 or cycle['rotation_deg'] > 2:
        failures.append('bidirectional_icp_cycle_failed')
    if tilt > 10:
        failures.append('tilt_incompatible_with_current_4dof_model')
    record['accepted'] = not failures
    record['eligible_for_joint'] = not failures and not held_out
    record['reason'] = ';'.join(failures) if failures else 'independent_rgbd_ransac_holdout_bidirectional_icp_passed'
    record.update(covariance_model())
    record['weak_sensitivity_model'] = covariance_model(.20, 5.)
    record['covariance_caution'] = 'Fixed model assumptions chosen before results; no statistical calibration or accuracy claim. Geometry conditioning reported separately; no residual-based overconfidence.'
    return record


def old_graph_roles(previous):
    payload = json.loads((previous/'validated_edges.json').read_text())
    loops = payload['verified_loops']
    holdouts = {heldout_pair_key(e['from_id'], e['to_id']) for e in loops if e.get('held_out')}
    trained = {heldout_pair_key(e['from_id'], e['to_id']) for e in loops if not e.get('held_out')}
    return holdouts, trained, loops


def evaluate_candidates(session, edges, records, previous, output):
    """Optional model sensitivity; no changes to prior outputs or canonical maps."""
    raw = {n: checked_transform(blob) for n, blob in session.connection.execute('SELECT id,pose FROM Node')}
    odom = [e for e in edges if e.type == 0]
    components = connected_components(sorted(raw), odom)
    old_holdout, _, old_loops = old_graph_roles(previous)
    old_edge = {heldout_pair_key(e['from_id'], e['to_id']): e for e in old_loops}
    new_by_pair = {heldout_pair_key(r['a'], r['b']): r for r in records if r['accepted']}
    # Preserve existing accepted native constraints unless independently replaced;
    # rejected revalidation is evidence, not authority to delete old measurements.
    reserved = old_holdout | {heldout_pair_key(r['a'], r['b']) for r in records if r['held_out']}
    old_poses = load_pose_csv(previous/'optimized_trajectory.csv')
    clouds = Clouds(session.connection, None)
    held_pairs = sorted(old_holdout | {heldout_pair_key(r['a'], r['b']) for r in records if r['held_out'] and r['accepted']})
    # Whole-pair holdouts are not graph factors. Their sample clouds may overlap
    # neighbouring frames: this does not create an independent sensor benchmark.
    summaries = {'raw': geometry_metrics(held_pairs, raw, clouds)[0],
                 'previous_candidate': geometry_metrics(held_pairs, old_poses, clouds)[0],
                 'app_saved': geometry_metrics(held_pairs, saved_app_poses(session.connection), clouds)[0]}
    variants = []
    for variant, st, sr in (('nominal', .10, 3.), ('weak', .20, 5.)):
        loops = []
        for key, e in old_edge.items():
            if key in new_by_pair:
                continue
            loops.append(Edge(e['from_id'], e['to_id'], e['type'], np.asarray(e['measurement']),
                np.asarray(e['information']), np.asarray(e['effective_information']), key in reserved))
        for key, r in new_by_pair.items():
            info = np.asarray(covariance_model(st, sr)['information_6x6'])
            loops.append(Edge(r['a'], r['b'], r['link_type'], np.asarray(r['T_a_b']), info, info, key in reserved))
        poses, solver = optimize(raw, odom, loops, components)
        metric, rows = geometry_metrics(held_pairs, poses, clouds)
        correction = [np.linalg.norm(poses[n][:3, 3]-raw[n][:3, 3]) for n in raw]
        baseline = summaries['previous_candidate']
        regression = (metric.get('count', 0) > 0 and
            (metric['clipped_rmse_median_m'] > baseline['clipped_rmse_median_m']+.005 or
             metric['overlap_median'] < baseline['overlap_median']-.03))
        solver_ok = solver['success'] and solver['active_bounds'] == 0 and solver['final_block_huber_cost'] <= solver['initial_block_huber_cost']+1e-8
        # A numerical candidate is not selected/promoted automatically, especially
        # for sessions lacking any independent whole-pair graph holdout.
        solver['accepted_for_output'] = False
        evidence = {'variant': variant, 'solver': solver, 'heldout_geometry': metric,
            'whole_pair_holdouts': [list(p) for p in held_pairs],
            'numerically_converged_unbounded': solver_ok,
            'heldout_regression_vs_previous_candidate': bool(regression),
            'status': 'experimental_candidate_only_not_promoted',
            'max_translation_correction_m': float(max(correction)),
            'model': covariance_model(st, sr),
            'training_residuals': error_statistics([e for e in loops if not e.held_out], poses),
            'heldout_residuals': error_statistics([e for e in loops if e.held_out], poses),
            'new_constraint_realized_residuals': [{
                'a': r['a'], 'b': r['b'], 'held_out': r['held_out'],
                'previous': delta_summary(np.asarray(r['T_a_b']), inverse(old_poses[r['a']]) @ old_poses[r['b']]),
                'candidate': delta_summary(np.asarray(r['T_a_b']), inverse(poses[r['a']]) @ poses[r['b']])}
                for r in records if r['accepted']],
            'caution': 'Same-session depth geometry, not ground truth. Empty whole-pair holdout means no independent graph improvement claim.'}
        folder = output/('candidate_'+variant)
        folder.mkdir(parents=True, exist_ok=True)
        trajectory(folder/'trajectory.csv', poses, session.stamps)
        trajectory(folder/'main_trajectory.csv', {n: poses[n] for n in components[0]}, session.stamps)
        (folder/'evaluation.json').write_text(json.dumps(evidence, indent=2)+'\n')
        variants.append(evidence)
    return {'baselines': summaries, 'variants': variants}


def run_session(name, output, do_optimize=False):
    filename, prior_root = SPECS[name]
    previous = ROOT/'outputs'/prior_root/name
    folder = output/name
    folder.mkdir(parents=True, exist_ok=True)
    (folder/'matches').mkdir(exist_ok=True)
    (folder/'correspondences').mkdir(exist_ok=True)
    database = ROOT/'data/raw/rtabmap'/filename
    before = database.stat()
    session = Session(database, name)
    session.raw_poses = {n: checked_transform(blob) for n, blob in
                         session.connection.execute('SELECT id,pose FROM Node')}
    all_edges, invalid_rows = read_edges(session.connection, set(session.ids), .03, .5)
    odom = [e for e in all_edges if e.type == 0]
    components = connected_components(session.ids, odom)
    lookup = {n: i for i, c in enumerate(components) for n in c}
    old_holdouts, old_trained, old_loops = old_graph_roles(previous)
    old_status = {heldout_pair_key(int(e['from_id']), int(e['to_id'])): e
                  for e in csv.DictReader((previous/'loop_validation.csv').open())}
    priority = {(41, 63): 0, (75, 209): 0, (474, 940): 0, (548, 981): 1, (503, 961): 2}
    loops = sorted((e for e in all_edges if e.type in (1, 2)),
                   key=lambda e: (priority.get((e.a, e.b), 10), e.a, e.b))
    records = []
    for index, edge in enumerate(loops):
        reserved, role = graph_holdout(name, edge, old_holdouts, old_trained)
        if lookup[edge.a] != lookup[edge.b]:
            row = {'session': name, 'a': edge.a, 'b': edge.b, 'session_a': name, 'node_a': edge.a,
                'session_b': name, 'node_b': edge.b, 'held_out': reserved, 'accepted': False,
                'eligible_for_joint': False, 'reason': 'different_odometry_components_not_merged'}
        else:
            row = revalidate_pair(session, edge, folder, reserved, role)
        row['previous_verification'] = old_status.get(heldout_pair_key(edge.a, edge.b))
        records.append(row)
        # Progress is separately labelled; only constraints.json is complete.
        (folder/'progress.json').write_text(json.dumps({'complete': False, 'records': records}, indent=2)+'\n')
        print(f'{name} {index+1}/{len(loops)} {edge.a}->{edge.b}: {row["reason"]}', flush=True)
    accepted = [r for r in records if r['accepted']]
    report = {'schema_version': 1, 'complete': True, 'session': name, 'created_utc': utc_now(),
        'coordinate_convention': 'T_a_b maps node_b local points into node_a local points',
        'source': {'database': str(database), 'bytes': before.st_size, 'mtime_ns': before.st_mtime_ns},
        'source_code': {p.name: sha256_file(p) for p in (Path(__file__), ROOT/'src/check_session_overlap.py', ROOT/'src/optimize_session.py')},
        'command': shlex.join(sys.argv),
        'versions': {'python': sys.version, 'numpy': np.__version__, 'scipy': scipy.__version__, 'opencv': cv2.__version__},
        'settings': SETTINGS, 'component_sizes': [len(c) for c in components],
        'summary': {'valid_stored_loop_count': len(loops), 'accepted': len(accepted),
                    'eligible_for_joint': sum(r['eligible_for_joint'] for r in accepted),
                    'accepted_whole_pair_holdouts': sum(r['held_out'] for r in accepted)},
        'invalid_native_link_rows': invalid_rows, 'preserved_old_graph_holdout_pairs': [list(p) for p in sorted(old_holdouts)],
        'records': records,
        'limitations': ['No semantic masks because preceding model failed visual quality review.',
            'The only geometry used is existing RGB-D; no independent survey or sensor calibration.',
            'Fixed isotropic weight models are sensitivity assumptions, not sensor covariances.',
            'Rejected revalidation does not automatically delete an old graph edge.',
            'Narrow/collinear visual support is rejected. Planar fixed 3D correspondences are tested with the centred rotation normal matrix, not rejected solely for low third positional variance.',
            'Four-DoF candidates preserve raw tilt and genuine Z differences; no floor flattening.']}
    if do_optimize and any(r['eligible_for_joint'] for r in accepted):
        report['candidate_sensitivity'] = evaluate_candidates(session, all_edges, records, previous, folder)
    after = database.stat()
    report['source_unchanged_size_mtime'] = before.st_size == after.st_size and before.st_mtime_ns == after.st_mtime_ns
    if not report['source_unchanged_size_mtime']:
        raise RuntimeError('Input database changed during read-only analysis')
    (folder/'constraints.json').write_text(json.dumps(report, indent=2)+'\n')
    session.connection.close()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions', nargs='+', choices=SPECS, default=['bridge_out1f', 'bridge_12_sub', 'floor2', 'outside', 'lobby'])
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/student_union_20261017_revalidated/intra_session')
    parser.add_argument('--evaluate-candidates', action='store_true')
    args = parser.parse_args()
    cv2.setNumThreads(2)
    cv2.setRNGSeed(DEFAULTS['seed'])
    started = time.monotonic()
    reports = []
    for name in args.sessions:
        report = run_session(name, args.output, args.evaluate_candidates)
        reports.append(report)
        print(json.dumps({'session': name, **report['summary']}), flush=True)
    aggregate = {'complete': True, 'created_utc': utc_now(), 'sessions': args.sessions,
        'runtime_seconds': time.monotonic()-started,
        'coordinate_convention': 'T_a_b maps node_b local points into node_a local points',
        'records': [r for report in reports for r in report['records']],
        'session_reports': [str((args.output/r['session']/'constraints.json').resolve()) for r in reports]}
    (args.output/'constraints.json').write_text(json.dumps(aggregate, indent=2)+'\n')


if __name__ == '__main__':
    main()
