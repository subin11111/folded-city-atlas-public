#!/usr/bin/env python3
"""Exploratory, never automatically accepted, exterior/bridge local matching.

Revisit the visually identified entrance side panel in adjacent frames omitted
by retrieval temporal NMS. Report every canonical gate even when an early gate
fails, allowing a human to distinguish narrow visual support from bad geometry.
No raw DB writes and no graph-edge promotion are performed.
"""
from pathlib import Path
import argparse
import json
import sys
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import check_session_overlap as c
import revalidate_loop_constraints as i
import revalidate_cross_sessions as x
from export_rtabmap_db import load_pose_csv


ROOT = Path(__file__).resolve().parents[1]


def evaluate(a, b, n, m, output):
    ka, da, _ = a.feature(n, True)
    kb, db, _ = b.feature(m, True)
    matches = c.mutual_ratio_matches(da, db, .75)
    result = dict(node_a=n, node_b=m, mutual_matches=len(matches), strict_accepted=False,
                  exploration_only=True, session_a='outside', session_b='bridge_out1f')
    if len(matches) < 8:
        return dict(result, failure='too_few_mutual_matches_for_exploration')
    pa = np.asarray([ka[z.queryIdx].pt for z in matches])
    pb = np.asarray([kb[z.trainIdx].pt for z in matches])
    target, source = a.unproject(n, pa), b.unproject(m, pb)
    valid = np.isfinite(target).all(1) & np.isfinite(source).all(1)
    source, target, pa, pb = source[valid], target[valid], pa[valid], pb[valid]
    result['depth_matches'] = len(source)
    if len(source) < 10:
        return dict(result, failure='too_few_depth_matches_for_exploration')
    seed = c.DEFAULTS['seed'] + n * 10007 + m
    train, held, masks = i.split_correspondences(i.depth_pixels(a, n, pa), i.depth_pixels(b, m, pb),
                                                a.depth(n)[0].shape, b.depth(m)[0].shape, seed)
    result.update(seed=seed, train_count=len(train), heldout_count=len(held))
    if len(train) < 6 or len(held) < 5:
        return dict(result, failure='too_few_disjoint_matches_for_exploration')
    visual, inliers, trials = c.rigid_ransac(source[train], target[train], seed)
    result.update(train_inliers=int(inliers.sum()), train_inlier_ratio=float(inliers.mean()))
    if inliers.sum() < 6:
        return dict(result, failure='too_few_ransac_inliers_for_exploration')
    ga, gd_a = i.planar_aware_support(target[train][inliers], pa[train][inliers], a.image(n).shape)
    gb, gd_b = i.planar_aware_support(source[train][inliers], pb[train][inliers], b.image(m).shape)
    ca, cb = i.cloud_without_holdout(a, n, masks[0]), i.cloud_without_holdout(b, m, masks[1])
    if min(len(ca), len(cb)) < 100:
        return dict(result, failure='too_few_disjoint_cloud_points')
    forward, fm = c.refine_icp(ca, cb, visual)
    reverse, rm = c.refine_icp(cb, ca, np.linalg.inv(visual))
    cycle = i.delta_summary(np.eye(4), forward @ reverse)
    hs = i.holdout_score(forward, source[held], target[held])
    rhs = i.holdout_score(reverse, target[held], source[held])
    visual_hs = i.holdout_score(visual, source[held], target[held])
    qa = x.project_node_points(a, n, c.apply_transform(forward, source[held]))
    qb = x.project_node_points(b, m, c.apply_transform(reverse, target[held]))
    errors = np.r_[np.linalg.norm(qa-pa[held], axis=1), np.linalg.norm(qb-pb[held], axis=1)]
    rp = dict(median_px=float(np.median(errors)), p90_px=float(np.percentile(errors, 90)),
              all_finite=bool(np.isfinite(errors).all()))
    failures = []
    if len(matches) < c.DEFAULTS['minimum_mutual_matches']: failures.append('minimum_mutual_matches')
    if len(source) < c.DEFAULTS['minimum_depth_matches']: failures.append('minimum_depth_matches')
    if inliers.sum() < c.DEFAULTS['minimum_train_inliers']: failures.append('minimum_train_inliers')
    if inliers.mean() < c.DEFAULTS['minimum_train_inlier_ratio']: failures.append('minimum_train_inlier_ratio')
    if not ga: failures.extend('outside_'+f for f in gd_a['planar_aware_gate_failures'])
    if not gb: failures.extend('bridge_'+f for f in gd_b['planar_aware_gate_failures'])
    for name, h in [('forward', hs), ('reverse', rhs)]:
        if h['inliers'] < 5 or h['inlier_ratio'] < .5 or h['rmse_m'] > .07:
            failures.append(name+'_heldout')
    for name, g in [('forward', fm), ('reverse', rm)]:
        if g['symmetric_overlap'] < .4 or g['symmetric_rmse_m'] > .07 or g['refinement_m'] > .15 or g['refinement_deg'] > 5:
            failures.append(name+'_icp')
    if cycle['translation_m'] > .1 or cycle['rotation_deg'] > 2: failures.append('icp_cycle')
    if not rp['all_finite'] or rp['median_px'] > 6 or rp['p90_px'] > 15: failures.append('reprojection')
    result.update(geometry_a=gd_a, geometry_b=gd_b, visual_T_a_b=visual.tolist(),
                  T_a_b=forward.tolist(), icp=fm, reverse_icp=rm, cycle=cycle,
                  heldout=hs, reverse_heldout=rhs, visual_heldout=visual_hs,
                  reprojection=rp, failed_strict_gates=failures,
                  strict_accepted=not failures,
                  sample_holdout_scope='3-pixel depth neighborhoods excluded from training/ICP; neighboring frames remain correlated')
    evidence = output/'correspondences'/f'{n}_{m}.npz'
    evidence.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(evidence, source=source, target=target, pixels_a=pa, pixels_b=pb,
                        train=train, heldout=held, inliers=inliers)
    result['evidence'] = str(evidence.resolve())
    display = cv2.drawMatches(a.image(n), ka, b.image(m), kb, matches, None,
                             flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(output/'matches'/f'{n}_{m}.jpg'), cv2.resize(display, (960,640)))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/student_union_20261017_all_preview/outside_bridge')
    parser.add_argument('--outside-start', type=int, default=259)
    parser.add_argument('--outside-end', type=int, default=267)
    parser.add_argument('--bridge-start', type=int, default=110)
    parser.add_argument('--bridge-end', type=int, default=125)
    args = parser.parse_args()
    cv2.setNumThreads(2)
    out = args.output.resolve(); (out/'matches').mkdir(parents=True, exist_ok=True)
    a = x.CachedSession(ROOT/'data/raw/rtabmap/261016-234854-outside.db', 'outside', out/'cache', 'clahe_depth')
    b = x.CachedSession(ROOT/'data/raw/rtabmap/261017-011331-bridgeout1f.db', 'bridge_out1f', out/'cache', 'clahe_depth')
    pa = ROOT/'outputs/student_union_20261017_reviewed/outside/optimized_main_trajectory.csv'
    pb = ROOT/'outputs/student_union_20261017_added_reviewed/bridge_out1f/optimized_main_trajectory.csv'
    poses_a, poses_b = load_pose_csv(pa), load_pose_csv(pb)
    ids_a = [n for n in a.ids if args.outside_start <= n <= args.outside_end and n in poses_a]
    ids_b = [n for n in b.ids if args.bridge_start <= n <= args.bridge_end and n in poses_b]
    c.contact_sheet(a, ids_a, out/'outside_frames.jpg')
    c.contact_sheet(b, ids_b, out/'bridge_frames.jpg')
    report = dict(method='Adjacent-frame SIFT/RootSIFT local search, explicit failed-gate evaluation',
                  purpose='Exploratory RViz attachment only, not validated joint graph',
                  visual_candidate_selection='Same entrance side panel identified in previous contact sheet outside263/bridge117',
                  database_fingerprints=[a.db_fingerprint,b.db_fingerprint],
                  pose_csv_a=c.fingerprint(pa), pose_csv_b=c.fingerprint(pb),
                  source_fingerprints=[c.fingerprint(Path(z.__file__)) for z in (c,i,x)] + [c.fingerprint(Path(__file__))],
                  parameters=dict(c.DEFAULTS), command=sys.argv, results=[])
    pairs = [(263,117)] + [(n,m) for n in ids_a for m in ids_b if (n,m)!=(263,117)]
    for number,(n,m) in enumerate(pairs):
        row = evaluate(a,b,n,m,out)
        if 'T_a_b' in row:
            world = poses_a[n] @ np.asarray(row['T_a_b']) @ np.linalg.inv(poses_b[m])
            row['T_outside_bridge'] = world.tolist()
            row['gravity_tilt_deg'] = float(np.degrees(np.arccos(np.clip(world[2,2],-1,1))))
        report['results'].append(row)
        report['status'] = 'completed' if number==len(pairs)-1 else 'in_progress'
        x.atomic_json(out/'report.json', report)
        print(number+1,len(pairs),n,m,row.get('depth_matches',0),row.get('heldout',{}).get('rmse_m'),row.get('failed_strict_gates',row.get('failure')),flush=True)
    a.connection.close(); b.connection.close()


if __name__ == '__main__': main()
