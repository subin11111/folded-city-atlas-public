#!/usr/bin/env python3
"""Recover cross-session RGB-D evidence with asymmetric-depth PnP.

Read-only DBs; original canonical acceptance thresholds are retained. Previous
whole-pair holdouts and adjacent frames are never offered as training. Candidate
selection uses prior retrieval only, never this run's validation scores. PnP is
estimated from one camera's depth and the other camera's RGB so missing depth on
one side need not prevent pose estimation. Both-depth reserved correspondences
still audit metric geometry; no image-only result is promoted as metric truth.
"""
from __future__ import annotations

import argparse
from collections import Counter, OrderedDict
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import check_session_overlap as c
import revalidate_cross_sessions as x
import revalidate_loop_constraints as i
from optimize_session import sha256_file, utc_now

ROOT = Path(__file__).resolve().parents[1]
SETTINGS = {
    'seed': 20261017, 'max_features': 6000, 'sift_contrast_threshold': .02,
    'clahe_clip_limit': 2., 'clahe_grid': [8, 8], 'ratio': .75,
    'pnp_ransac_iterations': 2000, 'pnp_ransac_reprojection_px': 6.,
    'pnp_confidence': .999, 'symmetric_refinement_loss': 'huber',
    'symmetric_refinement_scale_px': 3., 'sample_holdout_fraction': .25,
    'holdout_depth_exclusion_radius': 3, 'region_radius_m': 2.,
    'minimum_independent_regions': 3, 'reserved_region_modulus': 4,
    'source': 'https://docs.opencv.org/4.12.0/d5/d1f/calib3d_solvePnP.html',
}

# Explicit assistant visual annotations made after inspecting the original
# 720x960 RGB images. These are retrospective checks, not human/survey labels.
# Fixed opaque panel and door-frame junctions only; no glass-reflected lights.
MANUAL_ANNOTATIONS = [
    {'session_a':'lobby','node_a':39,'session_b':'bridge_out1f','node_b':79,
     'uncertainty_pixels':5.,'feasibility':'Eight recognizable fixed frame/panel corners; low-light blur limits precision.',
     'landmarks':[
        ['upper_panel_top_left',[217,176],[151,107]],
        ['upper_panel_top_right',[388,177],[353,114]],
        ['upper_panel_bottom_left',[228,401],[162,382]],
        ['upper_panel_bottom_right',[391,402],[355,386]],
        ['door_frame_top_left',[456,238],[431,194]],
        ['door_frame_top_right',[625,234],[626,191]],
        ['door_frame_bottom_left',[449,599],[423,607]],
        ['door_frame_bottom_right',[606,600],[603,611]]]},
    {'session_a':'outside','node_a':263,'session_b':'bridge_out1f','node_b':117,
     'uncertainty_pixels':7.,'feasibility':'Only two bottom fixed frame junctions are unambiguous in both crops; upper junctions are outside bridge image. Posters are behind reflective glass and cannot establish trustworthy LiDAR surface identity.',
     'landmarks':[
        ['glazing_left_bottom_junction',[263,637],[284,381]],
        ['glazing_center_bottom_junction',[447,686],[418,416]]]},
]


class RGBSession(c.Session):
    """CLAHE RootSIFT without the old two-sided valid-depth detector mask."""
    def __init__(self, path, label):
        super().__init__(path, label)
        self.feature_cache = OrderedDict()
        self.image_cache = OrderedDict()

    def image(self, node):
        if node not in self.image_cache:
            self.image_cache[node] = super().image(node)
            if len(self.image_cache) > 24:
                self.image_cache.popitem(last=False)
        self.image_cache.move_to_end(node)
        return self.image_cache[node]

    def feature(self, node, full=True):
        if node not in self.feature_cache:
            gray = cv2.cvtColor(self.image(node), cv2.COLOR_BGR2GRAY)
            gray = cv2.createCLAHE(clipLimit=SETTINGS['clahe_clip_limit'],
                                  tileGridSize=tuple(SETTINGS['clahe_grid'])).apply(gray)
            detector = cv2.SIFT_create(nfeatures=SETTINGS['max_features'],
                                      contrastThreshold=SETTINGS['sift_contrast_threshold'])
            keys, descriptors = detector.detectAndCompute(gray, None)
            self.feature_cache[node] = keys, c.root_sift(descriptors), 1.
            if len(self.feature_cache) > 48:
                self.feature_cache.popitem(last=False)
        self.feature_cache.move_to_end(node)
        return self.feature_cache[node]

    def depth(self, node):
        if node not in self.geometry and len(self.geometry) >= 48:
            self.geometry.pop(next(iter(self.geometry)))
        return super().depth(node)


def intrinsics_at_rgb(session, node):
    _, _, cw, ch, k, local = session.depth(node)
    height, width = session.image(node).shape[:2]
    k = k.copy()
    k[0, :] *= width / cw
    k[1, :] *= height / ch
    return k, local


def fit_pnp(points, pixels, intrinsic, local, seed):
    """Return source-node -> target-node, including optical extrinsics."""
    cv2.setRNGSeed(int(seed % 2147483647))
    if len(points) < 12:
        return None, {'accepted': False, 'reason': 'fewer_than_12_training_points'}
    ok, rv, tv, inliers = cv2.solvePnPRansac(
        np.ascontiguousarray(points, dtype=np.float64),
        np.ascontiguousarray(pixels, dtype=np.float64), intrinsic, None,
        iterationsCount=SETTINGS['pnp_ransac_iterations'],
        reprojectionError=SETTINGS['pnp_ransac_reprojection_px'],
        confidence=SETTINGS['pnp_confidence'], flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None or len(inliers) < 12:
        return None, {'accepted': False, 'reason': 'insufficient_pnp_ransac_consensus',
                      'inlier_count': 0 if inliers is None else len(inliers)}
    inliers = inliers.ravel()
    rv, tv = cv2.solvePnPRefineLM(points[inliers], pixels[inliers], intrinsic, None, rv, tv)
    optical = np.eye(4)
    optical[:3, :3] = cv2.Rodrigues(rv)[0]
    optical[:3, 3] = tv.ravel()
    return local @ optical, {'accepted': True, 'inlier_count': len(inliers),
                             'inlier_ratio': len(inliers) / len(points),
                             'inlier_indices': inliers.tolist()}


def split_asymmetric(pixels_a, pixels_b, shape_a, shape_b, seed):
    """Reserved samples are defined without examining either depth or fit."""
    return i.split_correspondences(pixels_a, pixels_b, shape_a, shape_b, seed)


def historical_sample_evidence(first, second, a, b):
    """Exact-pair prior sample holdouts; duplicate matcher exports deduplicated."""
    folder=f'{first.label}__{second.label}'
    paths=sorted((ROOT/'outputs/student_union_20261017_revalidated').glob(
        f'*/{folder}/correspondences/{a}_{b}.npz'))
    preview_folder={('outside','bridge_out1f'):'outside_bridge',
                    ('lobby','bridge_out1f'):'lobby_bridge'}.get((first.label,second.label))
    if preview_folder:
        preview=ROOT/'outputs/student_union_20261017_all_preview'/preview_folder/'correspondences'/f'{a}_{b}.npz'
        if preview.exists():paths.append(preview)
    seen=set();records=[]
    for path in paths:
        digest=sha256_file(path)
        if digest in seen:continue
        data=np.load(path,allow_pickle=False)
        held_key='heldout_indices' if 'heldout_indices' in data else 'heldout'
        a_key='rgb_pixels_a' if 'rgb_pixels_a' in data else 'pixels_a'
        b_key='rgb_pixels_b' if 'rgb_pixels_b' in data else 'pixels_b'
        if not all(k in data for k in (held_key,a_key,b_key,'source','target')):
            continue
        seen.add(digest)
        held=data[held_key]
        records.append({'path':str(path),'sha256':digest,
            'pixels_a':i.depth_pixels(first,a,data[a_key][held]),
            'pixels_b':i.depth_pixels(second,b,data[b_key][held]),
            'source':data['source'][held], 'target':data['target'][held]})
    return records


def refine_symmetric(initial, first, second, a, b, source, target, pa, pb, train):
    valid_a = train[np.isfinite(target[train]).all(1)]
    valid_b = train[np.isfinite(source[train]).all(1)]
    # Only training correspondences consistent with their corresponding PnP
    # seed enter joint refit. Holdouts are never filtered by model residual.
    def residual(parameters, ia, ib):
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_rotvec(parameters[3:]).as_matrix()
        transform[:3, 3] = parameters[:3]
        qa = x.project_node_points(first, a, c.apply_transform(transform, source[ib]))
        qb = x.project_node_points(second, b, c.apply_transform(np.linalg.inv(transform), target[ia]))
        return np.nan_to_num(np.r_[(qa-pa[ib]).ravel(), (qb-pb[ia]).ravel()],
                             nan=10000, posinf=10000, neginf=-10000) / 3.
    seed = np.r_[initial[:3, 3], Rotation.from_matrix(initial[:3, :3]).as_rotvec()]
    ea = np.linalg.norm(x.project_node_points(second, b,
         c.apply_transform(np.linalg.inv(initial), target[valid_a]))-pb[valid_a], axis=1)
    eb = np.linalg.norm(x.project_node_points(first, a,
         c.apply_transform(initial, source[valid_b]))-pa[valid_b], axis=1)
    selected_a, selected_b = valid_a[ea < 12.], valid_b[eb < 12.]
    if min(len(selected_a), len(selected_b)) < 12:
        return initial, {'success': False, 'reason': 'insufficient_bidirectional_training_support'}
    fit = least_squares(residual, seed, args=(selected_a, selected_b), loss='huber',
                        f_scale=1., max_nfev=80, x_scale='jac')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(fit.x[3:]).as_matrix()
    transform[:3, 3] = fit.x[:3]
    return transform, {'success': bool(fit.success), 'nfev': fit.nfev,
                       'cost': fit.cost, 'training_support_a': len(selected_a),
                       'training_support_b': len(selected_b)}


def score_pose(first, second, a, b, transform, source, target, pa, pb, held):
    both = held[np.isfinite(source[held]).all(1) & np.isfinite(target[held]).all(1)]
    metrics = {'heldout_count_both_depth': len(both)}
    if len(both):
        metrics['heldout_3d'] = i.holdout_score(transform, source[both], target[both])
    ea, eb = [], []
    va = held[np.isfinite(source[held]).all(1)]
    vb = held[np.isfinite(target[held]).all(1)]
    if len(va):
        ea = np.linalg.norm(x.project_node_points(first, a,
             c.apply_transform(transform, source[va]))-pa[va], axis=1)
    if len(vb):
        eb = np.linalg.norm(x.project_node_points(second, b,
             c.apply_transform(np.linalg.inv(transform), target[vb]))-pb[vb], axis=1)
    errors = np.r_[ea, eb]
    metrics['heldout_reprojection'] = {
        'count': len(errors), 'all_finite': bool(np.isfinite(errors).all()),
        'median_px': float(np.median(errors)) if len(errors) else None,
        'p90_px': float(np.percentile(errors, 90)) if len(errors) else None}
    return metrics


def geometry_at_pose(ca, cb, transform):
    moved = c.apply_transform(transform, cb)
    da = cKDTree(ca).query(moved, k=1)[0]
    db = cKDTree(moved).query(ca, k=1)[0]
    fa, fb = float(np.mean(da < .15)), float(np.mean(db < .15))
    pooled = np.r_[da[da < .15], db[db < .15]]
    return {'symmetric_overlap': 2*fa*fb/max(fa+fb, 1e-12),
            'symmetric_rmse_m': float(np.sqrt(np.mean(pooled**2))) if len(pooled) else None}


def draw_pair(first, second, a, b, pa, pb, train, held, path, caption):
    ia, ib = first.image(a), second.image(b)
    canvas = np.hstack([ia, ib])
    train = np.asarray(train, dtype=int)
    held = np.asarray(held, dtype=int)
    for indices, colour in ((train[:60], (90, 210, 90)), (held[:40], (0, 180, 255))):
        for n in indices:
            p = tuple(np.rint(pa[n]).astype(int))
            q = tuple((np.rint(pb[n])+[ia.shape[1], 0]).astype(int))
            cv2.circle(canvas, p, 5, colour, 2)
            cv2.circle(canvas, q, 5, colour, 2)
            cv2.line(canvas, p, q, colour, 1)
    canvas = cv2.resize(canvas, (1080, 720))
    footer = np.full((74, 1080, 3), 25, dtype=np.uint8)
    for k, line in enumerate([f'{first.label}:{a} / {second.label}:{b}',
                              caption[:120], 'Green=train; orange=reserved; automated correspondences, no manual clicks']):
        cv2.putText(footer, line, (10, 20+24*k), cv2.FONT_HERSHEY_SIMPLEX, .45, (240,240,240), 1)
    cv2.imwrite(str(path), np.vstack([canvas, footer]))


def evaluate(first, second, row, output, preserve_prior_samples=False):
    a, b = row['node_a'], row['node_b']
    result = dict(row, accepted=False, eligible_for_joint=False,
                  measurement_source='CLAHE unmasked RootSIFT + asymmetric EPnP-RANSAC + symmetric Huber reprojection',
                  transform_convention='T_a_b maps node B coordinates into node A coordinates')
    ka, da, _ = first.feature(a)
    kb, db, _ = second.feature(b)
    matches = c.mutual_ratio_matches(da, db, SETTINGS['ratio'])
    pa = np.array([ka[m.queryIdx].pt for m in matches]).reshape(-1, 2)
    pb = np.array([kb[m.trainIdx].pt for m in matches]).reshape(-1, 2)
    result['mutual_matches'] = len(matches)
    if len(matches) < 20:
        result['reason'] = 'fewer_than_20_mutual_rgb_matches'
        draw_pair(first, second, a, b, pa, pb, range(len(pa)), [], output/'matches'/f'{a}_{b}.jpg', result['reason'])
        return result
    target, source = first.unproject(a, pa), second.unproject(b, pb)
    pix_a, pix_b = i.depth_pixels(first, a, pa), i.depth_pixels(second, b, pb)
    seed = SETTINGS['seed'] + a*10007+b
    train, held, masks = split_asymmetric(pix_a, pix_b, first.depth(a)[0].shape,
                                          second.depth(b)[0].shape, seed)
    previous=historical_sample_evidence(first,second,a,b) if preserve_prior_samples else []
    original_train_count=len(train)
    for old in previous:
        masks[0] |= i.exclusion_mask(masks[0].shape,old['pixels_a'],3)
        masks[1] |= i.exclusion_mask(masks[1].shape,old['pixels_b'],3)
    if previous:
        train=np.asarray([n for n in train if not masks[0][pix_a[n,1],pix_a[n,0]]
                          and not masks[1][pix_b[n,1],pix_b[n,0]]],dtype=int)
    result.update(historical_sample_masks_preserved=preserve_prior_samples,
                  historical_sample_evidence=[{'path':p['path'],'sha256':p['sha256'],
                      'count':len(p['source'])} for p in previous],
                  train_removed_by_historical_masks=original_train_count-len(train))
    va = train[np.isfinite(target[train]).all(1)]
    vb = train[np.isfinite(source[train]).all(1)]
    both_train = train[np.isfinite(target[train]).all(1) & np.isfinite(source[train]).all(1)]
    result.update(train_count=len(train), heldout_count=len(held),
                  train_depth_a=len(va), train_depth_b=len(vb), train_both_depth=len(both_train),
                  heldout_policy='3-pixel depth neighborhoods excluded from both fits and both ICP clouds; no holdout filtering',
                  ransac_seed=seed)
    evidence = output/'correspondences'/f'{a}_{b}.npz'
    np.savez_compressed(evidence, target=target, source=source, rgb_pixels_a=pa, rgb_pixels_b=pb,
                        train_indices=train, heldout_indices=held)
    result['correspondence_evidence'] = str(evidence.resolve())
    k_a, l_a = intrinsics_at_rgb(first, a)
    k_b, l_b = intrinsics_at_rgb(second, b)
    forward, fm = fit_pnp(source[vb], pa[vb], k_a, l_a, seed)
    reverse, rm = fit_pnp(target[va], pb[va], k_b, l_b, seed+1)
    result.update(pnp_forward=fm, pnp_reverse=rm)
    if forward is None or reverse is None:
        result['reason'] = 'bidirectional_pnp_insufficient_support'
        draw_pair(first, second, a, b, pa, pb, train, held, output/'matches'/f'{a}_{b}.jpg', result['reason'])
        return result
    cycle = i.delta_summary(np.eye(4), forward @ reverse)
    result.update(pnp_cycle=cycle, pnp_T_a_b=forward.tolist(), pnp_T_b_a=reverse.tolist())
    # Predeclared final estimate: joint symmetric pixel objective. No heldout
    # score is involved in choosing a seed or choosing between algorithms.
    transform, refinement = refine_symmetric(forward, first, second, a, b, source, target, pa, pb, train)
    result.update(T_a_b=transform.tolist(), symmetric_refinement=refinement,
                  measurement_metrics=score_pose(first, second, a, b, transform, source, target, pa, pb, held))
    errors = np.linalg.norm(c.apply_transform(transform, source[both_train])-target[both_train], axis=1)
    consensus = errors < .08
    result['train_3d_inliers'] = int(consensus.sum())
    result['train_3d_inlier_ratio'] = float(consensus.mean()) if len(consensus) else 0.
    failures = []
    if np.count_nonzero(np.isfinite(source).all(1)&np.isfinite(target).all(1)) < 20:
        failures.append('canonical_minimum_20_both_depth_matches')
    if min(len(va), len(vb)) < 12 or min(fm['inlier_ratio'], rm['inlier_ratio']) < .4:
        failures.append('pnp_support')
    if consensus.sum() < 12 or result['train_3d_inlier_ratio'] < .4:
        failures.append('canonical_training_3d_support')
    if consensus.sum() >= 3:
        for label, points, pixels, session, node in (
            ('a', target[both_train][consensus], pa[both_train][consensus], first, a),
            ('b', source[both_train][consensus], pb[both_train][consensus], second, b)):
            good, diagnostic = i.planar_aware_support(points, pixels, session.image(node).shape)
            result['spatial_support_'+label] = diagnostic
            if not good: failures.append('spatial_support_'+label)
    if cycle['translation_m'] > .10 or cycle['rotation_deg'] > 2.:
        failures.append('independent_pnp_cycle')
    ca, cb = i.cloud_without_holdout(first, a, masks[0]), i.cloud_without_holdout(second, b, masks[1])
    if min(len(ca), len(cb)) >= 100:
        result['geometry_at_measurement'] = geometry_at_pose(ca, cb, transform)
        fwd, fi = c.refine_icp(ca, cb, transform)
        rev, ri = c.refine_icp(cb, ca, np.linalg.inv(transform))
        result.update(icp_forward=fi, icp_reverse=ri, icp_cycle=i.delta_summary(np.eye(4), fwd@rev),
                      icp_T_a_b=fwd.tolist(),
                      icp_metrics=score_pose(first, second, a, b, fwd, source, target, pa, pb, held))
        for label, metric in (('forward', fi), ('reverse', ri)):
            if metric['symmetric_overlap'] < .4 or metric['symmetric_rmse_m'] > .07:
                failures.append('canonical_icp_geometry_'+label)
            if metric['refinement_m'] > .15 or metric['refinement_deg'] > 5.:
                failures.append('canonical_icp_large_change_'+label)
        if result['icp_cycle']['translation_m'] > .10 or result['icp_cycle']['rotation_deg'] > 2.:
            failures.append('canonical_icp_cycle')
        geom = result['geometry_at_measurement']
        if geom['symmetric_overlap'] < .4 or geom['symmetric_rmse_m'] is None or geom['symmetric_rmse_m'] > .07:
            failures.append('geometry_at_measurement')
    else:
        failures.append('insufficient_excluded_icp_cloud')
    held3d = result['measurement_metrics'].get('heldout_3d', {})
    if held3d.get('count', 0) < 5 or held3d.get('inliers', 0) < 5 or held3d.get('inlier_ratio', 0.) < .5 or held3d.get('rmse_m', 99.) > .07:
        failures.append('canonical_heldout_3d')
    rp = result['measurement_metrics']['heldout_reprojection']
    if rp['count'] < 10 or not rp['all_finite'] or rp['median_px'] > 6. or rp['p90_px'] > 15.:
        failures.append('canonical_heldout_reprojection')
    world_r = first.raw_poses[a][:3, :3] @ transform[:3, :3] @ second.raw_poses[b][:3, :3].T
    result['gravity_tilt_deg'] = float(np.degrees(np.arccos(np.clip(world_r[2,2], -1., 1.))))
    if result['gravity_tilt_deg'] > 10.:
        failures.append('gravity_disagreement')
    result['historical_sample_holdout_scores']=[]
    for old in previous:
        score=i.holdout_score(transform,old['source'],old['target'])
        result['historical_sample_holdout_scores'].append({'path':old['path'],**score})
        if score['inliers'] < 5 or score['inlier_ratio'] < .5 or score['rmse_m'] > .07:
            failures.append('historical_sample_holdout_3d')
    result.update(accepted=not failures, failed_gates=failures,
                  reason='strict_asymmetric_depth_verified' if not failures else ';'.join(failures),
                  covariance_model=i.covariance_model())
    draw_pair(first, second, a, b, pa, pb, train, held, output/'matches'/f'{a}_{b}.jpg', result['reason'])
    return result


def canonical_graph_rows(rows):
    """Only truly new pairs; retain all earlier measurement and holdout metadata."""
    previous=set()
    for path in (ROOT/'outputs/student_union_20261017_revalidated').glob('*/**/accepted_frame_edges.json'):
        for row in json.loads(path.read_text()):
            previous.add((row['session_a'],row['node_a'],row['session_b'],row['node_b']))
    result=[]
    for row in rows:
        key=(row['session_a'],row['node_a'],row['session_b'],row['node_b'])
        if not row['accepted'] or key in previous:continue
        edge={k:row[k] for k in ('session_a','node_a','session_b','node_b','T_a_b',
              'correlation_group','role_reason','measurement_source','correspondence_evidence')}
        held=row['joint_role']=='heldout'
        edge.update(edge_id=f'{key[0]}:{key[1]}__{key[2]}:{key[3]}',
                    accepted=True,local_geometry_verified=True,eligible_for_joint=False,
                    held_out=held,joint_holdout=held,joint_role='heldout' if held else 'train',
                    initialization_allowed=False,sigma_translation_m=.10,sigma_rotation_deg=3.,
                    metrics=row['measurement_metrics'],original_roles_preserved=True,
                    duplicate_remeasurement=False,transform_convention=row['transform_convention'],
                    historical_sample_masks_preserved=row.get('historical_sample_masks_preserved',False),
                    historical_sample_holdout_scores=row.get('historical_sample_holdout_scores',[]),
                    promotion_requirement='Global agreement, at least three independent regions and unused-region audit')
        result.append(edge)
    return result


def manual_audit(output):
    """Save honest manual feasibility evidence, without inventing constraints."""
    output.mkdir(parents=True,exist_ok=True)
    sessions={name:RGBSession(ROOT/'data/raw/rtabmap'/db,name) for name,db in x.SESSION_DATABASES.items()}
    reports=[]
    for annotation in MANUAL_ANNOTATIONS:
        sa,sb=annotation['session_a'],annotation['session_b']
        a,b=annotation['node_a'],annotation['node_b']
        first,second=sessions[sa],sessions[sb]
        pa=np.array([l[1] for l in annotation['landmarks']],dtype=float)
        pb=np.array([l[2] for l in annotation['landmarks']],dtype=float)
        target,source=first.unproject(a,pa),second.unproject(b,pb)
        valid=np.isfinite(target).all(1)&np.isfinite(source).all(1)
        row={**annotation,'annotation_origin':'Assistant visual inspection of original720x960 images; not user clicks or surveyed truth',
             'role':'retrospective_manual_validation_only_never_training',
             'independence':'Not an untouched holdout: these frame pixels may have contributed to earlier dense ICP. No acceptance promotion.',
             'valid_depth_both':int(valid.sum()),'accepted':False,'eligible_for_joint':False,
             'failure_reasons':['Fewer than20 canonical correspondence samples; 12training+5heldout cannot be supplied by8or2 corners.',
                                'No new spatially independent region; these are previously searched doorway views.'],
             'landmark_observations':[]}
        for n,landmark in enumerate(annotation['landmarks']):
            row['landmark_observations'].append({'identity':landmark[0],
                 'rgb_pixel_a':pa[n].tolist(),'rgb_pixel_b':pb[n].tolist(),
                 'depth_valid_a':bool(np.isfinite(target[n]).all()),
                 'depth_valid_b':bool(np.isfinite(source[n]).all()),
                 'point_a_m':target[n].tolist() if np.isfinite(target[n]).all() else None,
                 'point_b_m':source[n].tolist() if np.isfinite(source[n]).all() else None})
        automatic=None
        for path in (ROOT/'outputs/student_union_20261017_geometry_review/connections/report.json',
                     ROOT/'outputs/student_union_20261017_geometry_review/connections/focused_recovery/report.json'):
            if path.exists():
                automatic=next((r for r in json.loads(path.read_text())['results'] if
                    (r['session_a'],r['node_a'],r['session_b'],r['node_b'])==(sa,a,sb,b)),automatic)
        if automatic and 'T_a_b' in automatic:
            row['existing_measurement_source']=automatic['correspondence_evidence']
            row['existing_measurement_audit']=score_pose(first,second,a,b,np.array(automatic['T_a_b']),
                                                         source,target,pa,pb,np.arange(len(pa)))
        image=np.hstack([first.image(a),second.image(b)])
        for n,(p,q) in enumerate(zip(pa,pb)):
            colour=(50,210,90) if valid[n] else (40,70,245)
            p=tuple(p.astype(int));q=tuple((q+[720,0]).astype(int))
            for location in (p,q):
                cv2.circle(image,location,8,colour,2)
                cv2.putText(image,str(n+1),(location[0]+9,location[1]-5),cv2.FONT_HERSHEY_SIMPLEX,.8,colour,2)
            cv2.line(image,p,q,colour,1)
        footer=np.full((110,1440,3),25,dtype=np.uint8)
        lines=[f'{sa}:{a} / {sb}:{b}: manual frame corners; uncertainty +/-{annotation["uncertainty_pixels"]:.0f}px',
               f'{int(valid.sum())}/{len(valid)} corners have valid depth on both sides (green); invalid/depth-edge corners red',
               'Retrospective inspection only; not surveyed landmarks or new independent holdout; no constraint promoted']
        for n,line in enumerate(lines):
            cv2.putText(footer,line,(10,27+32*n),cv2.FONT_HERSHEY_SIMPLEX,.58,(240,240,240),1)
        name=f'{sa}_{a}__{sb}_{b}_manual_corner_audit.jpg'
        cv2.imwrite(str(output/name),np.vstack([image,footer]))
        row['annotated_image']=str(output/name);reports.append(row)
    x.atomic_json(output/'manual_landmarks_report.json',{'annotations':reports,
        'method':'Original-resolution visual corner identification + unchanged confidence/patch-range RGB-D unprojection',
        'manual_fit_attempted':False,'reason':'Insufficient new independent high-confidence corner/depth support; preserve unvalidated status',
        'source_sha256':sha256_file(Path(__file__)),'created_utc':utc_now()})
    for session in sessions.values():session.connection.close()
    print(json.dumps([{'pair':[r['session_a'],r['node_a'],r['session_b'],r['node_b']],
                      'valid_depth_both':r['valid_depth_both'],'annotated_count':len(r['landmarks'])} for r in reports],indent=2))


def render_progress(output):
    """Show the actual sequence, including candidates withdrawn by stricter audit."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from export_rtabmap_db import load_pose_csv
    base=ROOT/'outputs/student_union_20261017_geometry_review/connections'
    initial=json.loads((base/'report.json').read_text())
    focused=json.loads((base/'focused_recovery/report.json').read_text())
    union=json.loads((base/'union_audit/report.json').read_text())
    output.mkdir(parents=True,exist_ok=True)
    records=[]
    allrows=initial['results']+focused['results']
    for sa,sb in x.PRIORITY_PAIRS:
        rows=[r for r in allrows if (r['session_a'],r['session_b'])==(sa,sb)]
        passed=[r for r in union['results'] if (r['session_a'],r['session_b'])==(sa,sb) and r['accepted']]
        records.append({'connection':f'{sa} / {sb}','unique_pairs':len(rows),
                        'pnp_estimates':sum('T_a_b' in r for r in rows),
                        'fresh_split_passes':sum(r['accepted'] for r in rows),
                        'historical_union_passes':len(passed),
                        'new_graph_candidates':len(canonical_graph_rows(passed))})
    fig,axes=plt.subplots(1,2,figsize=(14,5),constrained_layout=True)
    y=np.arange(len(records))
    for name,colour in [('unique_pairs','#b4bec8'),('pnp_estimates','#477bb3'),('fresh_split_passes','#d69a30')]:
        axes[0].barh(y,[r[name] for r in records],color=colour,label=name.replace('_',' '))
    axes[0].set_yticks(y,[r['connection'] for r in records]);axes[0].legend(fontsize=8)
    axes[0].set_title('415 different pairs; 67 PnP estimates')
    for off,key,colour in [(-.18,'fresh_split_passes','#d69a30'),(.18,'new_graph_candidates','#378957')]:
        axes[1].barh(y+off,[r[key] for r in records],height=.33,color=colour,label=key.replace('_',' '))
    axes[1].set_yticks(y,[r['connection'] for r in records]);axes[1].set_xlim(0,3)
    axes[1].legend(fontsize=8);axes[1].set_title('Final audit: preserve ALL earlier sample holdouts')
    fig.suptitle('Connection recovery: initial candidates, deeper search, holdout-union correction')
    fig.savefig(output/'connection_recovery_progress.png',dpi=180);plt.close(fig)
    # Same withheld RGB-D samples, same view limits at every processing step.
    first_row=next(r for r in initial['results'] if (r['session_a'],r['node_a'],r['session_b'],r['node_b'])==('lobby',495,'bridge_12_sub',85))
    final_row=next(r for r in union['results'] if (r['session_a'],r['node_a'],r['session_b'],r['node_b'])==('lobby',495,'bridge_12_sub',85))
    session_a=RGBSession(ROOT/'data/raw/rtabmap'/x.SESSION_DATABASES['lobby'],'lobby')
    session_b=RGBSession(ROOT/'data/raw/rtabmap'/x.SESSION_DATABASES['bridge_12_sub'],'bridge_12_sub')
    bundle=ROOT/'outputs/student_union_20261017_all_preview/bundle'
    pose_a=load_pose_csv(bundle/'lobby_trajectory.csv')[495]
    pose_b=load_pose_csv(bundle/'bridge_12_sub_trajectory.csv')[85]
    ca,cb=session_a.cloud(495),session_b.cloud(85)
    world_a=c.apply_transform(pose_a,ca)
    data=np.load(first_row['correspondence_evidence'])
    held=data['heldout_indices']
    held=held[np.isfinite(data['source'][held]).all(1)&np.isfinite(data['target'][held]).all(1)]
    matrices=[np.linalg.inv(pose_a)@pose_b,np.array(first_row['T_a_b']),np.array(final_row['T_a_b'])]
    titles=['Prior combined preview','Asymmetric PnP candidate','Prior-sample-union audited PnP']
    scores=[i.holdout_score(t,data['source'][held],data['target'][held]) for t in matrices]
    world_b=[c.apply_transform(pose_a@t,cb) for t in matrices]
    center=c.apply_transform(pose_a,data['target'][held]).mean(0)
    fig,axes=plt.subplots(2,3,figsize=(14,8),constrained_layout=True)
    for col in range(3):
        for row,dims in enumerate([(0,1),(0,2)]):
            axis=axes[row,col]
            axis.scatter(world_a[:,dims[0]],world_a[:,dims[1]],s=1,color='#2b98c4',alpha=.65,label='lobby495')
            axis.scatter(world_b[col][:,dims[0]],world_b[col][:,dims[1]],s=1,color='#e18a37',alpha=.6,label='substairs85')
            axis.set_xlim(center[dims[0]]-1.7,center[dims[0]]+1.7)
            axis.set_ylim(center[dims[1]]-1.7,center[dims[1]]+1.7)
            axis.set_aspect('equal');axis.grid(alpha=.15)
            axis.set_xlabel('world X (m)');axis.set_ylabel('world '+('Y' if row==0 else 'Z')+' (m)')
            if row==0:axis.set_title(f'{titles[col]}\nSame heldout RMSE {100*scores[col]["rmse_m"]:.2f}cm')
    axes[0,0].legend(markerscale=4,fontsize=8)
    fig.suptitle('Local stair evidence, fixed crop and scale at every step\nOnly this pair is shown; global placement still requires joint validation')
    fig.savefig(output/'stair_pair_alignment_progress.png',dpi=200);plt.close(fig)
    session_a.connection.close();session_b.connection.close()
    x.atomic_json(output/'progress_metrics.json',{'connections':records,
        'unique_pair_count':len(allrows),'fresh_pair_passes':sum(r['accepted'] for r in allrows),
        'historical_union_passes':sum(r['accepted'] for r in union['results']),
        'truly_new_graph_candidates':len(canonical_graph_rows(union['results'])),
        'local_stair_progress':[{'stage':title,'heldout':score} for title,score in zip(titles,scores)],
        'withdrawn_after_union':[{k:r[k] for k in ('session_a','node_a','session_b','node_b','reason')}
                                 for r in union['results'] if not r['accepted']],
        'figure_scope':'Pair-local metric evidence, not proof of final globally optimized map',
        'source_sha256':sha256_file(Path(__file__))})


def prior_candidates(sa, sb):
    records = {}
    base = ROOT/'outputs/student_union_20261017_revalidated'
    for path in sorted(base.glob(f'*/{sa}__{sb}/verification_checkpoint.json')):
        doc = json.loads(path.read_text())
        for row in doc.get('pair_results', doc.get('results', [])):
            a, b = row.get('outside_id'), row.get('lobby_id')
            if a is None or b is None:
                continue
            score = max(row.get('mutual_matches', 0), row.get('learned_confident_matches', 0),
                        row.get('valid_depth_matches', 0), row.get('retrieval_mutual_matches', 0))
            if (a,b) not in records or score > records[a,b]['retrieval_score']:
                records[a,b] = {'node_a': a, 'node_b': b, 'retrieval_score': score,
                                'prior_candidate_source': str(path.relative_to(ROOT))}
    # Previously tested neighborhood frames are candidates, never new places.
    fixed = {('outside','bridge_out1f'):[(263,117),(267,94),(270,94),(141,118),(99,129)],
             ('lobby','bridge_out1f'):[(11,23),(39,79),(49,38),(958,69)],
             ('lobby','bridge_12_sub'):[(462,36),(443,60),(396,21),(495,86)],
             ('floor2','bridge_12_sub'):[(553,159),(560,162)]}
    for a,b in fixed.get((sa,sb),[]):
        records.setdefault((a,b), {'node_a':a,'node_b':b,'retrieval_score':0.,'prior_candidate_source':'previous_visual_review'})
    return list(records.values())


def prior_holdout_nodes(sa, sb):
    nodes_a, nodes_b = set(), set()
    for path in sorted((ROOT/'outputs/student_union_20261017_revalidated').glob('**/*edges.json')):
        if 'dependencies' in path.parts:
            continue
        try:
            edges = json.loads(path.read_text())
        except (ValueError, OSError):
            continue
        if not isinstance(edges, list):
            continue
        for e in edges:
            if e.get('session_a') == sa and e.get('session_b') == sb and (
                e.get('joint_role') == 'heldout' or e.get('held_out', False)):
                nodes_a.add(e['node_a']); nodes_b.add(e['node_b'])
    return nodes_a, nodes_b


def select_regions(rows, first, second, limit):
    rows = sorted(rows, key=lambda r: (-r['retrieval_score'],r['node_a'],r['node_b']))
    groups = []
    for row in rows:
        a, b = row['node_a'], row['node_b']
        pa, pb = first.raw_poses[a][:3,3], second.raw_poses[b][:3,3]
        group = next((g for g in groups if (
            np.linalg.norm(pa-g['position_a']) < SETTINGS['region_radius_m'] or
            np.linalg.norm(pb-g['position_b']) < SETTINGS['region_radius_m'])), None)
        if group is None:
            group = dict(id=len(groups), position_a=pa, position_b=pb, rows=[])
            groups.append(group)
        group['rows'].append(row)
    # Round robin deliberately spends search effort outside the best local door.
    selected = []
    for depth in range(max((len(g['rows']) for g in groups), default=0)):
        for group in groups:
            if depth < len(group['rows']) and len(selected) < limit:
                row = dict(group['rows'][depth], region_id=group['id'],
                           region_center_a=group['position_a'].tolist(),
                           region_center_b=group['position_b'].tolist())
                selected.append(row)
    old_a, old_b = prior_holdout_nodes(first.label, second.label)
    for row in selected:
        a,b = row['node_a'],row['node_b']
        near_old = any(abs(first.stamps[a]-first.stamps[n]) < 8. for n in old_a) or any(
                      abs(second.stamps[b]-second.stamps[n]) < 8. for n in old_b)
        reserved = row['region_id'] % SETTINGS['reserved_region_modulus'] == 3
        row.update(session_a=first.label, session_b=second.label,
                   correlation_group=f'{first.label}__{second.label}:region{row["region_id"]}',
                   joint_role='heldout' if near_old or reserved else 'train_candidate',
                   role_reason='preserve_old_whole_pair_holdout_neighborhood' if near_old else
                      'predeclared_region_holdout' if reserved else 'predeclared_region_training_candidate',
                   selection_reason='prior retrieval ranked within 2m correlated camera neighborhoods; round-robin regions',
                   independence_caveat='Camera-position region labels are proxies; repeated views can still show the same physical surface.')
    return selected, len(groups)


def render_summary(results, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(13,5), constrained_layout=True)
    names = [f'{a}/{b}' for a,b in x.PRIORITY_PAIRS]
    counts, poses, accepted, regions = [], [], [], []
    for a,b in x.PRIORITY_PAIRS:
        rows=[r for r in results if (r['session_a'],r['session_b'])==(a,b)]
        counts.append(len(rows));poses.append(sum('T_a_b' in r for r in rows))
        accepted.append(sum(r['accepted'] for r in rows))
        regions.append(len(set(r['correlation_group'] for r in rows if r['accepted'])))
    positions=np.arange(len(names))
    axes[0].barh(positions, counts, label='searched', color='#9aa6b2')
    axes[0].barh(positions, poses, label='bidirectional PnP fits', color='#4176b5')
    axes[0].barh(positions, accepted, label='all gates passed', color='#3c9c67')
    axes[0].set_yticks(positions,names);axes[0].legend(loc='lower right');axes[0].set_xlabel('Pairs (not independent places)')
    axes[1].barh(positions,regions,color='#3c9c67');axes[1].axvline(3,color='#c94b40',linestyle='--',label='required spatial regions')
    axes[1].set_yticks(positions,names);axes[1].set_xlabel('Distinct accepted camera neighborhoods');axes[1].legend()
    fig.suptitle('New measurement: asymmetric RGB-D PnP, unchanged strict geometric gates')
    fig.savefig(output/'connection_search_summary.png',dpi=180);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/student_union_20261017_geometry_review/connections')
    parser.add_argument('--limit-per-connection',type=int,default=50)
    parser.add_argument('--pair',action='append',help='Optional sessionA:sessionB subset')
    parser.add_argument('--selection',choices=['diverse','ranked'],default='diverse')
    parser.add_argument('--skip-report',type=Path,action='append',default=[],
                        help='Skip already evaluated pairs; group assignments still use full candidate pool')
    parser.add_argument('--audit-passing-report',type=Path,
                        help='Remeasure only passes of a prior report, preserving their exact role metadata')
    parser.add_argument('--preserve-prior-sample-holdouts',action=argparse.BooleanOptionalAction,default=True,
                        help='Default: union all historical exact-pair holdout masks and audit their3D residuals. Disable only to reproduce the labeled first exploratory trial.')
    parser.add_argument('--manual-audit-only',action='store_true')
    parser.add_argument('--render-progress-only',action='store_true')
    args=parser.parse_args()
    if args.manual_audit_only:
        manual_audit(args.output)
        return
    if args.render_progress_only:
        render_progress(args.output)
        return
    if (args.output/'report.json').exists():parser.error('Choose fresh output directory')
    cv2.setNumThreads(2)
    args.output.mkdir(parents=True,exist_ok=True)
    sessions={name:RGBSession(ROOT/'data/raw/rtabmap'/db,name) for name,db in x.SESSION_DATABASES.items()}
    hashes={name:sha256_file(s.path) for name,s in sessions.items()}
    pairs=[tuple(p.split(':')) for p in args.pair] if args.pair else x.PRIORITY_PAIRS
    audit_rows=None
    if args.audit_passing_report:
        audit_rows=[r for r in json.loads(args.audit_passing_report.read_text())['results'] if r['accepted']]
        pairs=list(dict.fromkeys((r['session_a'],r['session_b']) for r in audit_rows))
    all_rows=[];candidate_manifest=[];plans={}
    skipped=set()
    for prior in args.skip_report:
        skipped.update((r['session_a'],r['node_a'],r['session_b'],r['node_b'])
                       for r in json.loads(prior.read_text())['results'])
    for sa,sb in pairs:
        candidates=prior_candidates(sa,sb)
        plan,groups=select_regions(candidates,sessions[sa],sessions[sb],len(candidates))
        if args.selection=='ranked':
            plan.sort(key=lambda r:(-r['retrieval_score'],r['node_a'],r['node_b']))
        plan=[r for r in plan if (sa,r['node_a'],sb,r['node_b']) not in skipped][:args.limit_per_connection]
        if audit_rows is not None:
            keys=('node_a','node_b','session_a','session_b','region_id','region_center_a','region_center_b',
                  'correlation_group','joint_role','role_reason','selection_reason','independence_caveat',
                  'retrieval_score','prior_candidate_source')
            plan=[{k:r[k] for k in keys} for r in audit_rows if (r['session_a'],r['session_b'])==(sa,sb)]
        plans[sa,sb]=plan
        candidate_manifest.extend(plan)
    x.atomic_json(args.output/'predeclared_candidates.json',candidate_manifest)
    for sa,sb in pairs:
        out=args.output/f'{sa}__{sb}'
        (out/'matches').mkdir(parents=True,exist_ok=True);(out/'correspondences').mkdir(exist_ok=True)
        plan=plans[sa,sb]
        for session,key in [(sessions[sa],'node_a'),(sessions[sb],'node_b')]:
            ids=list(dict.fromkeys(r[key] for r in plan))[:40]
            if ids:c.contact_sheet(session,ids,out/f'{session.label}_candidate_frames.jpg')
        rows=[]
        for number,row in enumerate(plan):
            result=evaluate(sessions[sa],sessions[sb],row,out,args.preserve_prior_sample_holdouts)
            rows.append(result);all_rows.append(result)
            x.atomic_json(out/'checkpoint.json',{'rows':rows,'complete':False})
            print(f'{sa}/{sb} {number+1}/{len(plan)} {row["node_a"]}/{row["node_b"]}: {result["reason"]}',flush=True)
        accepted=[r for r in rows if r['accepted']]
        training=[r for r in accepted if r['joint_role']=='train_candidate']
        # A local gate pass alone does not prove three independent global links.
        groups=len(set(r['correlation_group'] for r in training))
        summary={'complete':True,'searched':len(rows),'accepted_local_pairs':len(accepted),
                 'accepted_training_camera_regions':groups,'joint_connection_approved':False,
                 'reason':'Global joint optimization and whole-region geometry audit still required', 'rows':rows}
        x.atomic_json(out/'report.json',summary)
    edges=[]
    for row in all_rows:
        if row['accepted']:
            edge={key:row[key] for key in ('session_a','node_a','session_b','node_b','T_a_b',
                  'correlation_group','joint_role','role_reason','measurement_metrics','measurement_source','correspondence_evidence')}
            edge.update(eligible_for_joint=False,local_measurement_accepted=True,
                        initialization_allowed=False,covariance_model=i.covariance_model(),
                        promotion_requirement='Global agreement, at least three independent regions and unused-region audit')
            edges.append(edge)
    x.atomic_json(args.output/'candidate_frame_edges.json',edges)
    x.atomic_json(args.output/'graph_candidates.json',canonical_graph_rows(all_rows))
    render_summary(all_rows,args.output)
    report={'created_utc':utc_now(),'argv':sys.argv,'settings':SETTINGS,'canonical_gates':c.DEFAULTS,
            'source_sha256':{str(Path(__file__).relative_to(ROOT)):sha256_file(Path(__file__))},
            'database_sha256_before':hashes,'database_unchanged':all(sha256_file(s.path)==hashes[n] for n,s in sessions.items()),
            'searched':len(all_rows),'bidirectional_pnp_estimates':sum('T_a_b' in r for r in all_rows),
            'strict_local_passes':sum(r['accepted'] for r in all_rows),'navigation_ready':False,
            'failure_counts':dict(Counter(f for r in all_rows for f in r.get('failed_gates',[r['reason']]))),
            'limitations':['All pixel correspondences are automated; no manual clicks or surveyed dimensions.',
              'Spatial camera groups are a conservative search proxy, not independently surveyed landmarks.',
              'Prior retrieval selection makes this an adaptive recovery experiment, not blind evaluation.',
              'All old holdout frame neighborhoods and new reserved regions remain held out.',
              'PnP only improves measurement formulation; missing overlap and reflection cannot be repaired by a solver.'],
            'results':all_rows}
    x.atomic_json(args.output/'report.json',report)
    for s in sessions.values():s.connection.close()
    print(json.dumps({k:report[k] for k in ('searched','bidirectional_pnp_estimates','strict_local_passes','database_unchanged')},indent=2))


if __name__=='__main__':main()
