#!/usr/bin/env python3
"""Local neighbourhood checks and a non-promoted rigid bridge placement preview."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from check_session_overlap import apply_transform
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file, utc_now
from revalidate_cross_sessions import CachedSession, verify_planar_disjoint, atomic_json
from revalidate_loop_constraints import delta_summary, holdout_score


def yaw_transform(parameters):
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('z', parameters[3]).as_matrix()
    pose[:3, 3] = parameters[:3]
    return pose


def fit_placement(records, poses_a, poses_b):
    """Whole-session rigid placement only; no deformation or acceptance promotion."""
    proposals = [poses_a[r['outside_id']] @ np.asarray(r['lobby_node_to_outside_node']) @
                 np.linalg.inv(poses_b[r['lobby_id']]) for r in records]
    angles = np.array([np.arctan2(p[1, 0], p[0, 0]) for p in proposals])
    seed = np.r_[np.median([p[:3, 3] for p in proposals], axis=0),
                 np.arctan2(np.sin(angles).mean(), np.cos(angles).mean())]

    def residual(parameters):
        placement = yaw_transform(parameters)
        values = []
        for row in records:
            realized = np.linalg.inv(poses_a[row['outside_id']]) @ placement @ poses_b[row['lobby_id']]
            error = np.linalg.inv(np.asarray(row['lobby_node_to_outside_node'])) @ realized
            values.extend(np.r_[error[:3, 3] / .10,
                                Rotation.from_matrix(error[:3, :3]).as_rotvec() / np.radians(3)])
        return values

    fit = least_squares(residual, seed, loss='soft_l1', f_scale=1., max_nfev=200)
    return yaw_transform(fit.x), {
        'success': bool(fit.success), 'nfev': fit.nfev, 'cost': fit.cost,
        'model': '4DoF whole-session rigid placement, componentwise soft-L1; sigma .10m/3deg modelling assumptions',
        'source_pose_shape_preserved': True, 'floor_factors': 0, 'gnss_factors': 0,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/student_union_20261017_all_preview/lobby_bridge'))
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh output directory')
    (args.output / 'matches').mkdir(parents=True)
    (args.output / 'correspondences').mkdir()
    cv2.setNumThreads(2)
    paths = {
        'lobby_database': Path('data/raw/rtabmap/261017-000622-1f.db'),
        'bridge_database': Path('data/raw/rtabmap/261017-011331-bridgeout1f.db'),
        'lobby_poses': Path('outputs/student_union_20261017_revalidated/joint_disjoint_hypothesis/component_0/lobby/candidate_trajectory.csv'),
        'bridge_poses': Path('outputs/student_union_20261017_added_reviewed/bridge_out1f/optimized_main_trajectory.csv'),
    }
    before = {k: sha256_file(p) for k, p in paths.items()}
    first = CachedSession(paths['lobby_database'], 'lobby', args.output/'cache', strategy='clahe_depth')
    second = CachedSession(paths['bridge_database'], 'bridge_out1f', args.output/'cache', strategy='clahe_depth')
    aposes, bposes = load_pose_csv(paths['lobby_poses']), load_pose_csv(paths['bridge_poses'])
    candidates = set()
    for a, b, radius in [(11, 23, 4), (39, 79, 4), (49, 38, 2)]:
        for da in range(-radius, radius+1, 2):
            for db in range(-radius, radius+1, 2):
                if a+da in aposes and b+db in bposes:
                    candidates.add((a+da, b+db))
    rows = []
    for a, b in sorted(candidates):
        row = verify_planar_disjoint(first, second, {'outside_id':a, 'lobby_id':b,
                    'selection_reason':'fixed neighbourhood of visually reviewed shared doorway'}, args.output)
        rows.append(row)
        atomic_json(args.output/'checkpoint.json', {'complete':False, 'rows':rows})
        print(f'{len(rows)}/{len(candidates)} lobby{a}/bridge{b}: {row["reason"]}', flush=True)
    # Retain the stricter results verbatim. Pixel-gate failures are explicit
    # approximate placement evidence, never new verified pose-graph edges.
    evidence = [r for r in rows if r['accepted'] or r['reason'] == 'heldout_reprojection_failed']
    # Avoid dozens of adjacent images being treated as independent support.
    selected = []
    for row in sorted(evidence, key=lambda r:(not r['accepted'], r['heldout_rmse_m'])):
        a, b = row['outside_id'], row['lobby_id']
        if not any(abs(first.stamps[a]-first.stamps[r['outside_id']])<5 or
                   abs(second.stamps[b]-second.stamps[r['lobby_id']])<5 for r in selected):
            selected.append(row)
    result = {'created_utc':utc_now(), 'argv':sys.argv, 'status':'exploratory_only',
              'accepted_by_validation':False, 'navigation_ready':False,
              'visual_review':'39/79 shared dark doorway and adjacent fixtures; 11/23 same entry information sign, monitor and floor tiles; no manual correspondences',
              'strict_passed':sum(r['accepted'] for r in rows), 'checked':len(rows),
              'local_evidence_pairs':[[r['outside_id'],r['lobby_id']] for r in evidence],
              'fit_pairs':[[r['outside_id'],r['lobby_id']] for r in selected],
              'source_paths':{k:str(p.resolve()) for k,p in paths.items()}, 'source_sha256':before,
              'source_code_sha256':{str(p):sha256_file(p) for p in [Path(__file__),Path('src/revalidate_cross_sessions.py'),Path('src/revalidate_loop_constraints.py')]},
              'rows':rows,
              'limitations':['Neighbourhood search is selected after prior results, not blind validation.',
                             'Preview may use pairs failing the strict pixel reprojection gate; their failures remain recorded.',
                             'No accepted graph edge or original map is changed.']}
    if selected:
        transform, solver = fit_placement(selected, aposes, bposes)
        result.update(T_joint_bridge=transform.tolist(), solver=solver,
                      transform_convention='T_joint_bridge maps reviewed bridge_out1f session points into existing lobby+floor2 joint preview coordinates')
        diagnostics = []
        for r in evidence:
            realized = np.linalg.inv(aposes[r['outside_id']]) @ transform @ bposes[r['lobby_id']]
            data = np.load(r['correspondence_evidence'])
            held = data['heldout_indices']
            diagnostics.append({'pair':[r['outside_id'],r['lobby_id']], 'used_for_fit':r in selected,
                'strict_pair_accepted':r['accepted'], 'original_failure':r['reason'],
                'pose_residual':delta_summary(np.asarray(r['lobby_node_to_outside_node']),realized),
                'reserved_feature_residual':holdout_score(realized,data['source'][held],data['target'][held])})
        result['placement_diagnostics'] = diagnostics
    result['database_unchanged'] = all(sha256_file(paths[k]) == before[k] for k in ('lobby_database','bridge_database'))
    first.connection.close(); second.connection.close()
    atomic_json(args.output/'attachment.json',result)
    print(json.dumps({k:result.get(k) for k in ('strict_passed','checked','fit_pairs','T_joint_bridge','database_unchanged')},indent=2))


if __name__ == '__main__':
    main()
