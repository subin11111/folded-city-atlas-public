#!/usr/bin/env python3
"""Compose an explicitly unvalidated exterior/bridge visualization transform."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import check_session_overlap as c
import revalidate_cross_sessions as x
import revalidate_loop_constraints as intra
from export_rtabmap_db import load_pose_csv

ROOT = Path(__file__).resolve().parents[1]


def map_transform(pose_a, node_transform, pose_b):
    """T_mapA_mapB for a column-point T_nodeA_nodeB measurement."""
    return pose_a @ node_transform @ np.linalg.inv(pose_b)


def yaw_transform(parameters):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('z', parameters[3]).as_matrix()
    result[:3, 3] = parameters[:3]
    return result


def fit_gravity_preserving(source, target, groups, initial):
    """Huber fixed point fit; each image-pair receives equal total weight.

    This reuses map-pose gravity as an assumption, not an independent IMU prior.
    z remains a free offset, and the trajectory's actual height variation stays.
    """
    yaw = np.arctan2(initial[1, 0], initial[0, 0])
    rotation = Rotation.from_euler('z', yaw).as_matrix()
    translation = np.median(target-source@rotation.T, axis=0)
    weights = np.zeros(len(source))
    for group in np.unique(groups):
        chosen = groups == group
        weights[chosen] = np.sqrt(len(source)/(len(np.unique(groups))*chosen.sum()))
    def residual(parameters):
        return ((c.apply_transform(yaw_transform(parameters), source)-target)*weights[:,None]).ravel()
    fit = least_squares(residual, np.r_[translation,yaw], loss='huber', f_scale=.08,
                        xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=200)
    return yaw_transform(fit.x), dict(success=bool(fit.success), evaluations=fit.nfev,
                                     cost=float(fit.cost), huber_scale_m=.08,
                                     weighting='equal total squared weight per correlated frame pair')


def geometry(target, source, transform):
    moved = c.apply_transform(transform, source)
    da = cKDTree(target).query(moved)[0]
    db = cKDTree(moved).query(target)[0]
    mask_a, mask_b = da < .15, db < .15
    fa, fb = mask_a.mean(), mask_b.mean()
    pooled = np.r_[da[mask_a], db[mask_b]]
    return dict(source_points=len(source), target_points=len(target),
                source_overlap=float(fa), target_overlap=float(fb),
                symmetric_overlap=float(2*fa*fb/max(fa+fb,1e-12)),
                clipped_rmse_m=float(np.sqrt(np.mean(pooled**2))) if len(pooled) else None,
                clip_distance_m=.15,
                caution='Nearest-point geometry, not correspondence proof or surveyed accuracy')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/student_union_20261017_all_preview/outside_bridge')
    args = parser.parse_args(); out = args.output.resolve(); out.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(2)
    report_path = out/'report.json'; previous = json.loads(report_path.read_text())
    learned_path = ROOT/'outputs/student_union_20261017_revalidated/learned_cross_session/outside__bridge_out1f/verification_checkpoint.json'
    learned = json.loads(learned_path.read_text())
    pa, pb = Path(previous['pose_csv_a']['path']), Path(previous['pose_csv_b']['path'])
    poses_a, poses_b = load_pose_csv(pa), load_pose_csv(pb)
    a = c.Session(ROOT/'data/raw/rtabmap/261016-234854-outside.db','outside')
    b = c.Session(ROOT/'data/raw/rtabmap/261017-011331-bridgeout1f.db','bridge_out1f')
    chosen = [(263,117),(262,117),(263,118)]
    records = []
    for na,nb in chosen:
        record = next(r for r in previous['results'] if (r['node_a'],r['node_b'])==(na,nb))
        z = np.load(record['evidence'])
        records.append(dict(matcher='SIFT', node_a=na,node_b=nb,
                            source=z['source'],target=z['target'], train=z['train'][z['inliers']],held=z['heldout'],
                            pixels_a=z['pixels_a'],pixels_b=z['pixels_b'],
                            evidence=record['evidence'],transform=np.asarray(record['T_outside_bridge']),
                            original_failed_gates=record['failed_strict_gates']))
    record = next(r for r in learned['pair_results'] if (r['outside_id'],r['lobby_id'])==(263,118))
    z = np.load(record['correspondence_evidence'])
    train = z['train_indices']
    distances = np.linalg.norm(c.apply_transform(np.asarray(record['visual_T_a_b']), z['source'][train])-z['target'][train],axis=1)
    records.append(dict(matcher='LoFTR',node_a=263,node_b=118,source=z['source'],target=z['target'],
                        train=train[distances<.08],held=z['heldout_indices'],
                        pixels_a=z['rgb_pixels_a'],pixels_b=z['rgb_pixels_b'],
                        evidence=record['correspondence_evidence'],
                        transform=map_transform(poses_a[263],np.asarray(record['lobby_node_to_outside_node']),poses_b[118]),
                        original_failed_gates=record['reason'].split(';')))
    sources, targets, groups = [], [], []
    for index,r in enumerate(records):
        r['world_source'] = c.apply_transform(poses_b[r['node_b']], r['source'])
        r['world_target'] = c.apply_transform(poses_a[r['node_a']], r['target'])
        sources.append(r['world_source'][r['train']]); targets.append(r['world_target'][r['train']])
        groups.extend([index]*len(r['train']))
    reference = records[-1]['transform']
    gravity, solver = fit_gravity_preserving(np.vstack(sources),np.vstack(targets),np.asarray(groups),reference)
    candidates = {'loftr_6d':reference,'gravity_preserving_4d':gravity}
    # Additional sensitivity fit reserves the UNION of every pair's held pixels
    # in a node, so other image pairs/matchers cannot train in that neighborhood.
    masks = {}
    for r in records:
        for label,session,node,pixels in [('a',a,r['node_a'],r['pixels_a']),('b',b,r['node_b'],r['pixels_b'])]:
            dp = intra.depth_pixels(session,node,pixels)
            r['depth_'+label] = dp
            key = (label,node)
            mask = intra.exclusion_mask(session.depth(node)[0].shape,dp[r['held']],3)
            masks[key] = masks.get(key,np.zeros_like(mask)) | mask
    disjoint_sources,disjoint_targets,disjoint_groups,counts = [],[],[],[]
    for index,r in enumerate(records):
        kept=[]
        for j in r['train']:
            xa,ya=r['depth_a'][j];xb,yb=r['depth_b'][j]
            if not masks['a',r['node_a']][ya,xa] and not masks['b',r['node_b']][yb,xb]: kept.append(j)
        counts.append(dict(matcher=r['matcher'],node_a=r['node_a'],node_b=r['node_b'],before=len(r['train']),after=len(kept)))
        if kept:
            disjoint_sources.append(r['world_source'][kept]);disjoint_targets.append(r['world_target'][kept])
            disjoint_groups.extend([index]*len(kept))
    disjoint_solver = dict(counts=counts,status='insufficient_training')
    if len(disjoint_groups)>=8 and len(np.unique(disjoint_groups))>=2:
        disjoint, extra = fit_gravity_preserving(np.vstack(disjoint_sources),np.vstack(disjoint_targets),np.asarray(disjoint_groups),reference)
        candidates['gravity_union_disjoint_4d'] = disjoint
        disjoint_solver.update(status='fit_completed',**extra)
    observations = []
    for r in records:
        detail = dict(matcher=r['matcher'],node_a=r['node_a'],node_b=r['node_b'],
                      evidence=c.fingerprint(Path(r['evidence'])), training_inliers=len(r['train']),
                      heldout_count=len(r['held']),original_failed_gates=r['original_failed_gates'],
                      T_outside_bridge=r['transform'].tolist(), heldout={})
        for name, transform in candidates.items():
            detail['heldout'][name] = intra.holdout_score(transform,r['world_source'][r['held']],r['world_target'][r['held']])
        observations.append(detail)
    checks = []
    # A broader front-door view and a second outside pass, never used in this fit.
    for na,nb in [(263,117),(267,94),(266,99),(142,121),(141,118)]:
        ca = c.apply_transform(poses_a[na],a.cloud(na)); cb = c.apply_transform(poses_b[nb],b.cloud(nb))
        checks.append(dict(node_a=na,node_b=nb, fitting_pair=(na,nb) in chosen,
                           metrics={name:geometry(ca,cb,T) for name,T in candidates.items()}))
    frames_b = [114,115,116,119,120,122,123]
    frames_a = [259,260,261,265,266,267]
    cloud_a = np.vstack([c.apply_transform(poses_a[n],a.cloud(n)) for n in frames_a])
    cloud_b = np.vstack([c.apply_transform(poses_b[n],b.cloud(n)) for n in frames_b])
    stable = []
    for r in records:
        probe = poses_b[118]
        stable.append(dict(matcher=r['matcher'],node_a=r['node_a'],node_b=r['node_b'],
                           difference_at_bridge118_m=float(np.linalg.norm((r['transform']@probe)[:3,3]-(reference@probe)[:3,3])),
                           rotation_difference_deg=float(np.degrees(Rotation.from_matrix(reference[:3,:3].T@r['transform'][:3,:3]).magnitude()))))
    payload = dict(status='exploratory_candidate_only_not_validated',strict_accepted=False,eligible_for_joint=False,
        session_a='outside',session_b='bridge_out1f',transform_convention='T_outside_bridge maps bridge reviewed-map points to outside reviewed-map points; column vectors',
        pose_csv_a=c.fingerprint(pa),pose_csv_b=c.fingerprint(pb),
        cloud_a=c.fingerprint(pa.parent/'cloud_optimized_filtered.ply'),
        cloud_b=c.fingerprint(pb.parent/'cloud_optimized_filtered.ply'),
        visual_review_files=[str((out/name).resolve()) for name in ['outside_frames.jpg','bridge_frames.jpg','matches/263_117.jpg','matches/263_118.jpg']],
        database_fingerprints=previous['database_fingerprints'],
        source_fingerprints=[c.fingerprint(Path(__file__)),c.fingerprint(report_path),c.fingerprint(learned_path)],
        visual_review='Contact sheets inspected: same opaque side-panel labels, entrance door frame, brick column and tactile paving. Glass reflections and changing nighttime lights reduce confidence; no floor/stair alias used.',
        selected_variant='gravity_preserving_4d', T_outside_bridge=gravity.tolist(),
        selection_note='For visualization preserve the session up directions, fitting free xyz+yaw to balanced training inliers. Neither variant passes strict link validation.',
        alternatives={name:T.tolist() for name,T in candidates.items()},solver=solver,observations=observations,
        union_disjoint_sensitivity_solver=disjoint_solver,
        pooled_fit_holdout_scope='Per-pair reserved rows are excluded from that pair training. The pooled SIFT/LoFTR fit has NO union exclusion across pairs/matchers; its reported reserved-row errors are correlated diagnostics, NOT a disjoint validation of this pooled fit.',
        local_frame_checks=checks,unfitted_nearby_frame_clouds=dict(outside_nodes=frames_a,bridge_nodes=frames_b,
            metrics={name:geometry(cloud_a,cloud_b,T) for name,T in candidates.items()}),
        stability=stable, limitations=[
            'All strong image constraints concentrate on one entrance-side patch; 0 independent strict cross-session links.',
            'Neighboring frames and different matchers reuse the same sensor data; not independent validation.',
            'Pooled-fit reserved-row diagnostics are not union-disjoint: another matcher/pair may train on the same depth neighborhood. They cannot certify this attachment.',
            'Heldout errors remain recorded and do not become accepted simply because an approximate preview is requested.',
            'No GNSS, manual translation, floor flattening, or raw depth-confidence relaxation was used.',
            'Single rigid attachment does not correct internal session drift; far parts of the outdoor map may disagree.'])
    x.atomic_json(out/'attachment_candidate.json',payload)
    print(json.dumps({k:payload[k] for k in ['status','selected_variant','T_outside_bridge','stability','unfitted_nearby_frame_clouds']},indent=2))
    a.connection.close();b.connection.close()


if __name__=='__main__':main()
