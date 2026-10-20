#!/usr/bin/env python3
"""Read-only, conservative RGB-D cross-session overlap audit.

All transforms act on column points. Pair transforms map lobby node coordinates
to outside node coordinates, including each calibrated optical->node extrinsic.
Only multiple independent RGB-D links may propose a session transform. A proposed
transform is evidence for later joint optimization, not a merged map.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shlex
import sqlite3
import sys
import time
import zlib
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from export_rtabmap_db import calibration_from_blob, load_pose_csv, transform_from_blob


DEFAULTS = {
    "seed": 20261017, "retrieval_max_side": 480, "retrieval_features": 600,
    "retrieval_index_features_per_frame": 300, "retrieval_neighbors": 8,
    "candidate_limit": 120, "candidate_temporal_nms_s": 3.0,
    "verification_features": 2500, "lowe_ratio": 0.75,
    "sift_octave_layers":3, "sift_contrast_threshold":0.04,"sift_edge_threshold":10,"sift_sigma":1.6,
    "flann_trees":5,"flann_checks":96,"retrieval_top_per_outside_frame":3,
    "minimum_mutual_matches": 20, "minimum_depth_matches": 20,
    "min_depth_m": 0.25, "max_depth_m": 5.0, "min_confidence": 50,
    "depth_patch_range_m": 0.12, "ransac_iterations": 2500,
    "depth_patch_side_pixels":3,"minimum_valid_patch_pixels":5,"minimum_ransac_triangle_cross_norm_m2":0.002,
    "ransac_threshold_m": 0.08, "minimum_train_inliers": 12,
    "minimum_train_inlier_ratio": 0.40, "heldout_fraction": 0.25,
    "minimum_heldout_inliers": 5, "minimum_heldout_inlier_ratio": 0.50,
    "maximum_heldout_rmse_m": 0.07, "minimum_image_hull_fraction": 0.04,
    "minimum_3d_second_std_m": 0.15, "minimum_3d_third_std_m": 0.025,
    "maximum_3d_condition": 80.0, "voxel_m": 0.05,
    "icp_iterations": 30, "icp_correspondence_m": 0.15,
    "icp_trim_percentile": 80, "minimum_symmetric_overlap": 0.40,
    "cloud_pixel_stride":2,"icp_minimum_correspondences":50,
    "icp_stop_translation_m":1e-5,"icp_stop_rotation_rad":1e-5,"icp_overlap_distance_m":0.15,
    "maximum_icp_rmse_m": 0.07, "maximum_icp_delta_m": 0.15,
    "maximum_icp_delta_deg": 5.0, "maximum_gravity_disagreement_deg": 10.0,
    "minimum_independent_links": 3, "independent_time_s": 8.0,
    "independent_position_m": 0.75, "session_consistency_m": 0.35,
    "session_consistency_deg": 5.0,
}


def apply_transform(transform, points):
    return points @ transform[:3, :3].T + transform[:3, 3]


def rigid_fit(source, target):
    if len(source) < 3:
        raise ValueError("At least three correspondences are required")
    source_center, target_center = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source-source_center).T @ (target-target_center))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = target_center - rotation @ source_center
    return transform


def rigid_ransac(source, target, seed, threshold=0.08, iterations=2500):
    rng = np.random.default_rng(seed)
    best = np.zeros(len(source), dtype=bool)
    transform = np.eye(4)
    valid_trials = 0
    if len(source) < 3:
        return transform, best, valid_trials
    for _ in range(iterations):
        indices = rng.choice(len(source), 3, replace=False)
        if any(np.linalg.norm(np.cross(points[1]-points[0], points[2]-points[0])) < DEFAULTS['minimum_ransac_triangle_cross_norm_m2']
               for points in (source[indices], target[indices])):
            continue
        candidate = rigid_fit(source[indices], target[indices])
        inliers = np.linalg.norm(apply_transform(candidate, source)-target, axis=1) < threshold
        valid_trials += 1
        if inliers.sum() > best.sum():
            best, transform = inliers, candidate
    if best.sum() >= 3:
        for _ in range(3):
            transform = rigid_fit(source[best], target[best])
            best = np.linalg.norm(apply_transform(transform, source)-target, axis=1) < threshold
            if best.sum() < 3:
                break
    return transform, best, valid_trials


def root_sift(descriptors):
    if descriptors is None:
        return np.empty((0, 128), dtype=np.float32)
    return np.sqrt(descriptors / np.maximum(descriptors.sum(axis=1, keepdims=True), 1e-12)).astype(np.float32)


def mutual_ratio_matches(first, second, ratio=0.75):
    if len(first) < 2 or len(second) < 2:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    forward = matcher.knnMatch(first, second, k=2)
    reverse = matcher.knnMatch(second, first, k=2)
    good_reverse = {(m.queryIdx, m.trainIdx) for m, n in reverse if m.distance < ratio*n.distance}
    return [m for m, n in forward if m.distance < ratio*n.distance and (m.trainIdx, m.queryIdx) in good_reverse]


class Session:
    def __init__(self, path, label, mask_dir=None):
        self.path, self.label = path.resolve(), label
        self.mask_dir=mask_dir
        self.connection = sqlite3.connect(f"file:{self.path}?mode=ro", uri=True)
        rows = self.connection.execute("SELECT id,stamp,pose FROM Node ORDER BY id").fetchall()
        self.ids = [r[0] for r in rows]
        self.stamps = {r[0]: float(r[1]) for r in rows}
        self.raw_poses = {r[0]: transform_from_blob(r[2]) for r in rows}
        self.features = {}
        self.geometry = {}

    def mask(self, node, shape):
        if self.mask_dir is None:
            return np.zeros(shape[:2],dtype=np.uint8)
        path=self.mask_dir/f'{node}.png'
        mask=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise ValueError(f'Requested semantic mask missing/unreadable: {path}')
        if mask.shape!=shape[:2]:
            raise ValueError(f'Semantic mask shape mismatch: {path}: {mask.shape} != {shape[:2]}')
        return mask

    def image(self, node):
        blob = self.connection.execute("SELECT image FROM Data WHERE id=?", (node,)).fetchone()[0]
        image = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Undecodable RGB: {self.label}/{node}")
        return image

    def feature(self, node, full=False):
        key = (node, full)
        if key in self.features:
            return self.features[key]
        image = self.image(node)
        feature_mask=np.where(self.mask(node,image.shape)>0,0,255).astype(np.uint8)
        scale = 1.0 if full else DEFAULTS["retrieval_max_side"]/max(image.shape[:2])
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if scale != 1.0:
            gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            feature_mask=cv2.resize(feature_mask,(gray.shape[1],gray.shape[0]),interpolation=cv2.INTER_NEAREST)
        detector = cv2.SIFT_create(nfeatures=DEFAULTS["verification_features"] if full else DEFAULTS["retrieval_features"],
            nOctaveLayers=DEFAULTS['sift_octave_layers'],contrastThreshold=DEFAULTS['sift_contrast_threshold'],
            edgeThreshold=DEFAULTS['sift_edge_threshold'],sigma=DEFAULTS['sift_sigma'])
        points, descriptors = detector.detectAndCompute(gray, feature_mask)
        value = (points, root_sift(descriptors), scale)
        # Keep memory bounded while another worker rebuilds full session clouds.
        if len(self.features)>=64:
            self.features.pop(next(iter(self.features)))
        self.features[key] = value
        return value

    def depth(self, node):
        if node in self.geometry:
            return self.geometry[node]
        row = self.connection.execute("SELECT depth,depth_confidence,calibration FROM Data WHERE id=?", (node,)).fetchone()
        packed = cv2.imdecode(np.frombuffer(row[0], np.uint8), cv2.IMREAD_UNCHANGED)
        if packed is None or packed.ndim != 3 or packed.shape[2] != 4:
            raise ValueError(f"Unexpected depth encoding: {self.label}/{node}")
        depth = np.ascontiguousarray(packed).view('<f4').reshape(packed.shape[:2])
        confidence_raw = zlib.decompress(row[1]) if row[1] else b''
        # Never silently assume high confidence for absent/malformed metadata.
        if len(confidence_raw) != depth.size:
            raise ValueError(f"Missing/malformed depth confidence: {self.label}/{node}")
        confidence = np.frombuffer(confidence_raw, np.uint8).reshape(depth.shape)
        if self.mask_dir:
            image=self.image(node)
            mask=self.mask(node,image.shape)
            mask=cv2.resize(mask,(depth.shape[1],depth.shape[0]),interpolation=cv2.INTER_NEAREST)
            confidence=confidence.copy()
            confidence[mask>0]=0
        width, height, intrinsic, local = calibration_from_blob(row[2])
        value = (depth, confidence, width, height, intrinsic, local)
        self.geometry[node] = value
        return value

    def unproject(self, node, pixels):
        depth, confidence, cw, ch, k, local = self.depth(node)
        image_h, image_w = self.image(node).shape[:2]
        dh, dw = depth.shape
        # RGB samples and calibration may have different dimensions.
        uv = np.asarray(pixels, dtype=float) * [cw/image_w, ch/image_h]
        duv = uv * [dw/cw, dh/ch]
        result = np.full((len(pixels), 3), np.nan)
        for index, (u, v) in enumerate(duv):
            x, y = int(round(u)), int(round(v))
            if x < 1 or y < 1 or x >= dw-1 or y >= dh-1:
                continue
            patch = depth[y-1:y+2, x-1:x+2]
            mask = (np.isfinite(patch) & (patch >= DEFAULTS['min_depth_m']) &
                    (patch <= DEFAULTS['max_depth_m']) &
                    (confidence[y-1:y+2, x-1:x+2] >= DEFAULTS['min_confidence']))
            if mask.sum() < DEFAULTS['minimum_valid_patch_pixels'] or np.ptp(patch[mask]) > DEFAULTS['depth_patch_range_m']:
                continue
            z = float(np.median(patch[mask]))
            point = np.array([(uv[index,0]-k[0,2])*z/k[0,0],
                              (uv[index,1]-k[1,2])*z/k[1,1], z])
            result[index] = local[:3,:3] @ point + local[:3,3]
        return result

    def cloud(self, node):
        depth, confidence, cw, ch, k, local = self.depth(node)
        h, w = depth.shape
        vv, uu = np.mgrid[0:h:2, 0:w:2]
        z = depth[::2,::2]
        valid = (np.isfinite(z) & (z >= DEFAULTS['min_depth_m']) & (z <= DEFAULTS['max_depth_m']) &
                 (confidence[::2,::2] >= DEFAULTS['min_confidence']))
        z = z[valid]
        optical = np.column_stack(((uu[valid]*cw/w-k[0,2])*z/k[0,0],
                                   (vv[valid]*ch/h-k[1,2])*z/k[1,1], z))
        points = apply_transform(local, optical)
        _, indices = np.unique(np.floor(points/DEFAULTS['voxel_m']).astype(np.int64), axis=0, return_index=True)
        return points[indices]


def contact_sheet(session, ids, destination, columns=5):
    tiles = []
    for node in ids:
        image = cv2.resize(session.image(node), (180,240))
        cv2.rectangle(image, (0,0), (180,30), (0,0,0), -1)
        cv2.putText(image, f'{session.label} #{node}', (5,20), cv2.FONT_HERSHEY_SIMPLEX, .43, (255,255,255), 1)
        tiles.append(image)
    if not tiles:
        return
    while len(tiles)%columns:
        tiles.append(np.zeros_like(tiles[0]))
    canvas = np.vstack([np.hstack(tiles[i:i+columns]) for i in range(0,len(tiles),columns)])
    cv2.imwrite(str(destination), canvas)


def retrieval(first, second, output):
    cv2.setRNGSeed(DEFAULTS['seed'])
    index_descriptors, owner = [], []
    for count, node in enumerate(second.ids):
        _, desc, _ = second.feature(node)
        desc = desc[:DEFAULTS['retrieval_index_features_per_frame']]
        if len(desc):
            index_descriptors.append(desc)
            owner.extend([node]*len(desc))
        if count%100 == 0:
            print(f'retrieval: {second.label} {count}/{len(second.ids)}', flush=True)
    descriptors = np.vstack(index_descriptors)
    owner = np.asarray(owner, dtype=np.int32)
    matcher = cv2.FlannBasedMatcher(dict(algorithm=1,trees=DEFAULTS['flann_trees']),dict(checks=DEFAULTS['flann_checks']))
    matcher.add([descriptors])
    matcher.train()
    candidates = []
    for count, node in enumerate(first.ids):
        _, desc, _ = first.feature(node)
        scores = Counter()
        if len(desc):
            matches = matcher.knnMatch(desc, k=DEFAULTS['retrieval_neighbors'])
            for neighbors in matches:
                seen = set()
                for rank, match in enumerate(neighbors):
                    other = int(owner[match.trainIdx])
                    if other in seen:
                        continue
                    seen.add(other)
                    # RootSIFT L2 distance weighting; ranking only, not geometry evidence.
                    scores[other] += max(0.0, 1.0-match.distance)/(1.0+rank)
            for other, score in scores.most_common(DEFAULTS['retrieval_top_per_outside_frame']):
                candidates.append({'outside_id':node, 'lobby_id':other,
                                   'retrieval_score':float(score/np.sqrt(max(1,len(desc))))})
        if count%50 == 0:
            print(f'retrieval: {first.label} {count}/{len(first.ids)}', flush=True)
    candidates.sort(key=lambda r:r['retrieval_score'], reverse=True)
    write_csv(output/'retrieval_candidates.csv', candidates)
    selected = []
    for candidate in candidates:
        a, b = candidate['outside_id'], candidate['lobby_id']
        if any(abs(first.stamps[a]-first.stamps[c['outside_id']]) < DEFAULTS['candidate_temporal_nms_s'] and
               abs(second.stamps[b]-second.stamps[c['lobby_id']]) < DEFAULTS['candidate_temporal_nms_s'] for c in selected):
            continue
        selected.append(candidate)
        if len(selected) >= DEFAULTS['candidate_limit']:
            break
    return selected, len(candidates), len(descriptors)


def geometry_support(points, pixels, image_shape):
    std = np.linalg.svd(points-points.mean(axis=0), compute_uv=False)/np.sqrt(max(1,len(points)-1))
    hull = cv2.convexHull(np.asarray(pixels,dtype=np.float32))
    fraction = float(cv2.contourArea(hull)/(image_shape[0]*image_shape[1]))
    condition = float(std[0]/max(std[2],1e-12))
    accepted = (std[1] >= DEFAULTS['minimum_3d_second_std_m'] and
                std[2] >= DEFAULTS['minimum_3d_third_std_m'] and
                condition <= DEFAULTS['maximum_3d_condition'] and
                fraction >= DEFAULTS['minimum_image_hull_fraction'])
    return accepted, {'std_m':std.tolist(),'condition':condition,'image_hull_fraction':fraction}


def refine_icp(target, source, initial):
    transform = initial.copy()
    tree = cKDTree(target)
    for iteration in range(DEFAULTS['icp_iterations']):
        moved = apply_transform(transform,source)
        distance, ids = tree.query(moved,k=1)
        mask = distance < DEFAULTS['icp_correspondence_m']
        if mask.sum() < DEFAULTS['icp_minimum_correspondences']:
            break
        mask &= distance <= np.percentile(distance[mask],DEFAULTS['icp_trim_percentile'])
        delta = rigid_fit(moved[mask],target[ids[mask]])
        transform = delta @ transform
        if np.linalg.norm(delta[:3,3]) < DEFAULTS['icp_stop_translation_m'] and Rotation.from_matrix(delta[:3,:3]).magnitude() < DEFAULTS['icp_stop_rotation_rad']:
            break
    moved = apply_transform(transform,source)
    forward = tree.query(moved,k=1)[0]
    backward = cKDTree(moved).query(target,k=1)[0]
    threshold=DEFAULTS['icp_overlap_distance_m']
    fa, fb = float(np.mean(forward<threshold)),float(np.mean(backward<threshold))
    pooled = np.concatenate((forward[forward<threshold],backward[backward<threshold]))
    delta = np.linalg.inv(initial) @ transform
    return transform, {'source_overlap':fa,'target_overlap':fb,
                       'symmetric_overlap':2*fa*fb/max(fa+fb,1e-12),
                       'symmetric_rmse_m':float(np.sqrt(np.mean(pooled**2))) if len(pooled) else 99.0,
                       'refinement_m':float(np.linalg.norm(delta[:3,3])),
                       'refinement_deg':float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude())),
                       'iterations':iteration+1}


def verify_pair(first, second, candidate, output):
    result = dict(candidate)
    result.update({'accepted':False, 'reason':'not_evaluated'})
    a, b = candidate['outside_id'],candidate['lobby_id']
    ka, da, _ = first.feature(a,full=True)
    kb, db, _ = second.feature(b,full=True)
    matches = mutual_ratio_matches(da,db,DEFAULTS['lowe_ratio'])
    result['mutual_matches'] = len(matches)
    ia, ib = first.image(a),second.image(b)
    # Every candidate has a reviewable pair image; up to40 matches are displayed.
    visual = cv2.drawMatches(ia,ka,ib,kb,matches[:40],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    visual = cv2.resize(visual,(720,480))
    cv2.imwrite(str(output/'matches'/f'{a:04d}_{b:04d}.jpg'),visual)
    if len(matches) < DEFAULTS['minimum_mutual_matches']:
        result['reason']='too_few_mutual_visual_matches'
        return result
    pixa = np.array([ka[m.queryIdx].pt for m in matches])
    pixb = np.array([kb[m.trainIdx].pt for m in matches])
    target, source = first.unproject(a,pixa),second.unproject(b,pixb)
    valid = np.isfinite(target).all(axis=1) & np.isfinite(source).all(axis=1)
    target, source,pixa,pixb = target[valid],source[valid],pixa[valid],pixb[valid]
    result['valid_depth_matches']=len(source)
    if len(source) < DEFAULTS['minimum_depth_matches']:
        result['reason']='too_few_valid_depth_matches'
        return result
    seed = DEFAULTS['seed']+a*10007+b
    indices = np.random.default_rng(seed).permutation(len(source))
    heldout_count=max(5,int(np.ceil(len(source)*DEFAULTS['heldout_fraction'])))
    holdout, train = indices[:heldout_count],indices[heldout_count:]
    transform,inliers,trials = rigid_ransac(source[train],target[train],seed,
        DEFAULTS['ransac_threshold_m'],DEFAULTS['ransac_iterations'])
    result.update({'ransac_seed':seed,'ransac_valid_trials':trials,'train_count':len(train),
                   'train_inliers':int(inliers.sum()),'train_inlier_ratio':float(inliers.mean()),
                   'heldout_count':len(holdout)})
    if inliers.sum() < DEFAULTS['minimum_train_inliers'] or inliers.mean() < DEFAULTS['minimum_train_inlier_ratio']:
        result['reason']='insufficient_3d_ransac_consensus'
        return result
    supported=[]
    for points,pixels,shape,label in ((target[train][inliers],pixa[train][inliers],ia.shape,'outside'),
                                     (source[train][inliers],pixb[train][inliers],ib.shape,'lobby')):
        accepted, metrics=geometry_support(points,pixels,shape)
        result[label+'_geometry']=metrics
        supported.append(accepted)
    if not all(supported):
        result['reason']='degenerate_or_narrow_spatial_support'
        return result
    target_cloud,source_cloud=first.cloud(a),second.cloud(b)
    result['outside_cloud_points'],result['lobby_cloud_points']=len(target_cloud),len(source_cloud)
    if min(len(target_cloud),len(source_cloud))<100:
        result['reason']='insufficient_cloud_support'
        return result
    refined,icp=refine_icp(target_cloud,source_cloud,transform)
    result['icp']=icp
    # Held-out feature correspondences are excluded from RANSAC/refit. Their
    # individual pixels may occur in the dense cloud; this is a correspondence
    # holdout, not a fully sensor-independent validation set.
    error=np.linalg.norm(apply_transform(refined,source[holdout])-target[holdout],axis=1)
    heldin=error<DEFAULTS['ransac_threshold_m']
    result.update({'heldout_inliers':int(heldin.sum()),'heldout_inlier_ratio':float(heldin.mean()),
                   'heldout_rmse_m':float(np.sqrt(np.mean(error**2))),
                   'heldout_median_m':float(np.median(error))})
    if (heldin.sum()<DEFAULTS['minimum_heldout_inliers'] or heldin.mean()<DEFAULTS['minimum_heldout_inlier_ratio'] or
        result['heldout_rmse_m']>DEFAULTS['maximum_heldout_rmse_m']):
        result['reason']='heldout_correspondence_validation_failed'
        return result
    if (icp['symmetric_overlap']<DEFAULTS['minimum_symmetric_overlap'] or
        icp['symmetric_rmse_m']>DEFAULTS['maximum_icp_rmse_m'] or
        icp['refinement_m']>DEFAULTS['maximum_icp_delta_m'] or
        icp['refinement_deg']>DEFAULTS['maximum_icp_delta_deg']):
        result['reason']='icp_validation_failed'
        return result
    result.update({'accepted':True,'reason':'pair_geometry_verified','lobby_node_to_outside_node':refined.tolist()})
    return result


def independent_consensus(candidates, first, second, poses_a, poses_b):
    candidates=[dict(c) for c in candidates if c['accepted']]
    for candidate in candidates:
        a,b=candidate['outside_id'],candidate['lobby_id']
        transform=poses_a[a] @ np.asarray(candidate['lobby_node_to_outside_node']) @ np.linalg.inv(poses_b[b])
        candidate['lobby_map_to_outside_map']=transform.tolist()
        angle=float(np.degrees(np.arccos(np.clip(transform[2,2],-1,1))))
        candidate['gravity_disagreement_deg']=angle
    eligible=[c for c in candidates if c['gravity_disagreement_deg']<=DEFAULTS['maximum_gravity_disagreement_deg']]
    best=[]
    for anchor in eligible:
        base=np.asarray(anchor['lobby_map_to_outside_map'])
        cluster=[]
        for candidate in eligible:
            delta=np.linalg.inv(base) @ np.asarray(candidate['lobby_map_to_outside_map'])
            if (np.linalg.norm(delta[:3,3])>DEFAULTS['session_consistency_m'] or
                np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude())>DEFAULTS['session_consistency_deg']):
                continue
            a,b=candidate['outside_id'],candidate['lobby_id']
            if any(abs(first.stamps[a]-first.stamps[c['outside_id']])<DEFAULTS['independent_time_s'] or
                   abs(second.stamps[b]-second.stamps[c['lobby_id']])<DEFAULTS['independent_time_s'] or
                   np.linalg.norm(poses_a[a][:3,3]-poses_a[c['outside_id']][:3,3])<DEFAULTS['independent_position_m'] or
                   np.linalg.norm(poses_b[b][:3,3]-poses_b[c['lobby_id']][:3,3])<DEFAULTS['independent_position_m'] for c in cluster):
                continue
            if any(np.linalg.norm((np.linalg.inv(np.asarray(c['lobby_map_to_outside_map'])) @
                                   np.asarray(candidate['lobby_map_to_outside_map']))[:3,3])>DEFAULTS['session_consistency_m'] or
                   np.degrees(Rotation.from_matrix((np.linalg.inv(np.asarray(c['lobby_map_to_outside_map'])) @
                                   np.asarray(candidate['lobby_map_to_outside_map']))[:3,:3]).magnitude())>DEFAULTS['session_consistency_deg']
                   for c in cluster):
                continue
            cluster.append(candidate)
        if len(cluster)>len(best):
            best=cluster
    return best,candidates


def write_csv(path, rows):
    if not rows:
        path.write_text('outside_id,lobby_id,accepted,reason\n',encoding='utf-8')
        return
    keys=list(dict.fromkeys(key for row in rows for key in row))
    with path.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({k:json.dumps(v) if isinstance(v,(dict,list)) else v for k,v in row.items()})


def fingerprint(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):
            digest.update(chunk)
    return {'path':str(path.resolve()),'bytes':path.stat().st_size,'sha256':digest.hexdigest()}


def mask_fingerprint(session):
    if session.mask_dir is None:
        return None
    digest=hashlib.sha256()
    for node in session.ids:
        path=session.mask_dir/f'{node}.png'
        if not path.exists():
            raise ValueError(f'Missing requested mask: {path}')
        digest.update(str(node).encode('ascii')+b'\0')
        digest.update(bytes.fromhex(fingerprint(path)['sha256']))
    return {'frame_count':len(session.ids),'ordered_png_sha256':digest.hexdigest()}


def candidate_sheet(results, output):
    # Highest feature-match counts are most informative for rejection review;
    # retrieval scores alone can rank completely unrelated low-texture images.
    selected=sorted(results,key=lambda row:row['mutual_matches'],reverse=True)[:24]
    tiles=[]
    for row in selected:
        image=cv2.imread(str(output/'matches'/f'{row["outside_id"]:04d}_{row["lobby_id"]:04d}.jpg'))
        if image is None:
            continue
        image=cv2.resize(image,(360,240))
        tile=np.zeros((280,360,3),dtype=np.uint8)
        tile[:240]=image
        text=f'out{row["outside_id"]}/lobby{row["lobby_id"]} visual={row["mutual_matches"]} depth={row.get("valid_depth_matches","-")}'
        cv2.putText(tile,text,(4,255),cv2.FONT_HERSHEY_SIMPLEX,.39,(255,255,255),1)
        cv2.putText(tile,row['reason'],(4,272),cv2.FONT_HERSHEY_SIMPLEX,.32,(160,200,255),1)
        tiles.append(tile)
    if tiles:
        while len(tiles)%4:
            tiles.append(np.zeros_like(tiles[0]))
        canvas=np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)])
        cv2.imwrite(str(output/'candidate_contact_sheet.jpg'),canvas)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outside-db',type=Path,required=True)
    parser.add_argument('--lobby-db',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--outside-poses',type=Path)
    parser.add_argument('--lobby-poses',type=Path)
    parser.add_argument('--mask-dir-a',type=Path,help='Outside RGB-sized semantic exclusion PNGs, one {id}.png per frame')
    parser.add_argument('--mask-dir-b',type=Path,help='Lobby RGB-sized semantic exclusion PNGs, one {id}.png per frame')
    parser.add_argument('--candidate-limit',type=int,default=DEFAULTS['candidate_limit'])
    parser.add_argument('--reuse-pair-report',action='store_true',help='Recheck consensus from existing pair geometry without retrieval')
    args=parser.parse_args()
    DEFAULTS['candidate_limit']=args.candidate_limit
    cv2.setNumThreads(1)
    start=time.monotonic()
    invocation=shlex.join([sys.executable,*sys.argv])
    source_hash=fingerprint(Path(__file__))['sha256']
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'matches').mkdir(exist_ok=True)
    first,second=Session(args.outside_db,'outside',args.mask_dir_a),Session(args.lobby_db,'lobby',args.mask_dir_b)
    masks={'outside':str(args.mask_dir_a.resolve()) if args.mask_dir_a else None,
           'lobby':str(args.mask_dir_b.resolve()) if args.mask_dir_b else None}
    fingerprints=[fingerprint(args.outside_db),fingerprint(args.lobby_db)]
    mask_hashes={'outside':mask_fingerprint(first),'lobby':mask_fingerprint(second)}
    report_path=args.output/'overlap_report.json'
    if args.reuse_pair_report:
        old=json.loads(report_path.read_text(encoding='utf-8'))
        if old['inputs']!=fingerprints:
            raise ValueError('Input fingerprint differs from saved audit')
        if old.get('semantic_mask_directories')!=masks:
            raise ValueError('Mask directories differ from saved audit; full verification required')
        if old.get('semantic_mask_fingerprints')!=mask_hashes:
            raise ValueError('Mask contents differ from saved audit; full verification required')
        if any(old['parameters'].get(key)!=value for key,value in DEFAULTS.items()):
            raise ValueError('Parameters differ from saved audit; full verification required')
        results=old['pair_results']
        retrieval_count,index_count=old['retrieval_candidate_count'],old['retrieval_index_descriptor_count']
    else:
        for session in (first,second):
            indices=np.unique(np.linspace(0,len(session.ids)-1,30).astype(int))
            contact_sheet(session,[session.ids[i] for i in indices],args.output/(session.label+'_overview.jpg'))
        selected,retrieval_count,index_count=retrieval(first,second,args.output)
        first.features.clear()
        second.features.clear()
        results=[]
        for index,candidate in enumerate(selected):
            result=verify_pair(first,second,candidate,args.output)
            results.append(result)
            print(f'verify {index+1}/{len(selected)} {result["outside_id"]}/{result["lobby_id"]}: {result["reason"]}',flush=True)
            if (index+1)%10==0 or index+1==len(selected):
                checkpoint={'status':'incomplete_pair_verification','completed':len(results),'selected':len(selected),
                            'inputs':fingerprints,'semantic_mask_fingerprints':mask_hashes,'parameters':DEFAULTS,
                            'source_code_sha256':source_hash,'command':invocation,'pair_results':results}
                (args.output/'pair_verification_checkpoint.json').write_text(json.dumps(checkpoint,indent=2),encoding='utf-8')
        (args.output/'pair_results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    optimized_ready=bool(args.outside_poses and args.lobby_poses and args.outside_poses.exists() and args.lobby_poses.exists())
    poses_a=load_pose_csv(args.outside_poses) if optimized_ready else first.raw_poses
    poses_b=load_pose_csv(args.lobby_poses) if optimized_ready else second.raw_poses
    missing=[(c['outside_id'],c['lobby_id']) for c in results if c['accepted'] and
             (c['outside_id'] not in poses_a or c['lobby_id'] not in poses_b)]
    for result in results:
        if (result['outside_id'],result['lobby_id']) in missing:
            result['accepted']=False
            result['reason']='node_excluded_from_optimized_odometry_component'
    consensus,with_transforms=independent_consensus(results,first,second,poses_a,poses_b)
    can_propose=optimized_ready and len(consensus)>=DEFAULTS['minimum_independent_links']
    status='proposed_transform_for_joint_validation' if can_propose else 'unmerged_insufficient_verified_overlap'
    if not optimized_ready and len(consensus)>=DEFAULTS['minimum_independent_links']:
        status='unmerged_waiting_for_optimized_trajectories'
    # A strict new audit must not leave a stale successful transform file.
    proposal_path=args.output/'proposed_lobby_to_outside_transform.json'
    if proposal_path.exists() and not can_propose:
        raise RuntimeError('Stale proposed transform exists; inspect/archive explicitly before rerunning')
    if can_propose:
        # A consensus medoid is an initialization only; no unverified averaging.
        proposal={'status':status,'transform':consensus[0]['lobby_map_to_outside_map'],
                  'independent_links':consensus,'note':'Initialization only, joint graph optimization and held-out map validation still required.'}
        proposal_path.write_text(json.dumps(proposal,indent=2),encoding='utf-8')
    report={'status':status,'inputs':fingerprints,'parameters':DEFAULTS,'node_counts':{'outside':len(first.ids),'lobby':len(second.ids)},
            'algorithms':['all-frame RootSIFT descriptor retrieval, FLANN randomized KD trees, weighted frame votes',
                          'mutual nearest-neighbor Lowe-ratio RootSIFT verification',
                          'confidence-gated metric RGB-D unprojection with calibrated optical-to-node transform',
                          'seeded 3D RANSAC SE(3), SVD/Kabsch refit, reflection correction',
                          'training-inlier 3D singular values and RGB convex-hull coverage degeneracy checks',
                          'trimmed point-to-point local ICP, symmetric nearest-neighbor overlap and RMSE',
                          'held-out feature correspondence residual checks after ICP',
                          'gravity and independently separated cross-session transform consensus'],
            'semantic_mask_directories':masks,
            'semantic_mask_fingerprints':mask_hashes,
            'limitations':['Retrieval is appearance-ranked and candidate-limited: rejection does not prove no physical overlap.',
                           'A single planar surface is deliberately insufficient even with visual texture.',
                           'Semantic exclusion masks are used only if paths are recorded; moving/static object labels can be imperfect.',
                           'Held-out feature matches are excluded from RANSAC but their pixels may occur in dense ICP clouds.',
                           'GNSS is not used as evidence for session connection.',
                           'No map merge or joint optimization is performed by this audit.'],
            'source_code_sha256':source_hash,'command':invocation,'cwd':str(Path.cwd()),
            'pair_results_reused':args.reuse_pair_report,
            'pair_verification_source_sha256':old.get('pair_verification_source_sha256',old['source_code_sha256']) if args.reuse_pair_report else source_hash,
            'stage_execution_counts':{'appearance_retrieval_frames':len(first.ids)+len(second.ids),
                'mutual_descriptor_verification_pairs':len(results),
                'depth_unprojection_pairs':sum('valid_depth_matches' in c for c in results),
                'ransac_pairs':sum('ransac_valid_trials' in c for c in results),
                'icp_pairs':sum('icp' in c for c in results),
                'heldout_correspondence_validation_pairs':sum('heldout_inliers' in c for c in results)},
            'library_versions':{'opencv':cv2.__version__,'numpy':np.__version__},
            'retrieval_candidate_count':retrieval_count,'retrieval_index_descriptor_count':index_count,
            'verified_candidate_count':len(results),'accepted_pair_count':sum(c['accepted'] for c in results),
            'rejection_counts':dict(Counter(c['reason'] for c in results if not c['accepted'])),
            'independent_consistent_link_count':len(consensus),'optimized_pose_consensus':optimized_ready,
            'pose_files':{'outside':str(args.outside_poses) if optimized_ready else 'database Node.pose',
                          'lobby':str(args.lobby_poses) if optimized_ready else 'database Node.pose'},
            'pair_results':results,'verified_links_with_session_transforms':with_transforms,
            'elapsed_seconds':time.monotonic()-start}
    report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
    write_csv(args.output/'accepted_pairs.csv',[c for c in results if c['accepted']])
    write_csv(args.output/'rejected_pairs.csv',[c for c in results if not c['accepted']])
    write_csv(args.output/'candidate_results.csv',results)
    candidate_sheet(results,args.output)
    print(json.dumps({key:report[key] for key in ['status','accepted_pair_count','independent_consistent_link_count','rejection_counts','elapsed_seconds']},indent=2),flush=True)


if __name__=='__main__':
    main()
