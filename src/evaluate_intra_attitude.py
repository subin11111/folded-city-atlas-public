#!/usr/bin/env python3
"""Secondary, unpromoted bounded-attitude sensitivity on revalidated loops.

This does not add a gravity sensor measurement or a floor factor. It preserves
all previous graph holdouts and canonical outputs. Single-cycle experiments
without whole-pair holdouts cannot establish general map improvement.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from check_session_overlap import Session
from export_rtabmap_db import load_pose_csv
from optimize_bounded_attitude import optimize_attitude
from optimize_session import (Clouds, Edge, checked_transform, connected_components,
    geometric_score, geometry_metrics, inverse, read_edges, saved_app_poses,
    sha256_file, trajectory, utc_now)
from revalidate_loop_constraints import (ROOT, SPECS, covariance_model, delta_summary,
    heldout_pair_key, holdout_score, old_graph_roles)


def run(name, sigmas, input_root):
    filename, prior_root = SPECS[name]
    previous = ROOT/'outputs'/prior_root/name
    output = input_root/name
    source = output/'constraints.json'
    revalidated = json.loads(source.read_text())
    if not revalidated['complete']:
        raise ValueError('Revalidation must be complete before attitude trials')
    session = Session(ROOT/'data/raw/rtabmap'/filename, name)
    raw = {n: checked_transform(blob) for n, blob in session.connection.execute('SELECT id,pose FROM Node')}
    edges, _ = read_edges(session.connection, set(raw), .03, .5)
    odom = [e for e in edges if e.type == 0]
    components = connected_components(sorted(raw), odom)
    old_holdouts, _, old_loops = old_graph_roles(previous)
    new = {heldout_pair_key(r['a'], r['b']): r for r in revalidated['records'] if r['accepted']}
    reserved = old_holdouts | {heldout_pair_key(r['a'], r['b']) for r in revalidated['records'] if r['held_out']}
    loops = []
    for e in old_loops:
        key = heldout_pair_key(e['from_id'], e['to_id'])
        if key in new:
            continue
        loops.append(Edge(e['from_id'], e['to_id'], e['type'], np.array(e['measurement']),
            np.array(e['information']), np.array(e['effective_information']), key in reserved))
    info = np.asarray(covariance_model()['information_6x6'])
    for key, r in new.items():
        loops.append(Edge(r['a'], r['b'], r['link_type'], np.asarray(r['T_a_b']), info, info, key in reserved))
    baselines = {'raw': raw, 'previous_4dof': load_pose_csv(previous/'optimized_trajectory.csv'),
        'app_saved': saved_app_poses(session.connection),
        'remeasured_4dof_nominal': load_pose_csv(output/'candidate_nominal/trajectory.csv')}
    clouds = Clouds(session.connection, None)
    # These local loops are fitting constraints, not independent frame-pair tests.
    def evidence(poses):
        rows = []
        for e in loops:
            if e.a not in poses or e.b not in poses:
                continue
            transform = inverse(poses[e.a]) @ poses[e.b]
            _, ca = clouds.get(e.a)
            _, cb = clouds.get(e.b)
            row = {'a': e.a, 'b': e.b, 'held_out_pair': e.held_out,
                'graph_role': 'heldout' if e.held_out else 'training',
                'realized_measurement_residual': delta_summary(e.measurement, transform),
                'sampled_depth_geometry_diagnostic': geometric_score(ca, cb, transform, .20),
                'geometry_warning': 'Training-frame geometry for non-heldout pairs, not independent graph validation.'}
            key = heldout_pair_key(e.a, e.b)
            if key in new:
                r = new[key]
                data = np.load(r['correspondence_evidence'])
                idx = data['heldout_indices']
                row['reserved_feature_evidence'] = holdout_score(transform, data['source'][idx], data['target'][idx])
                row['feature_warning'] = 'Feature/depth pixels not fitted, but their pass/fail was used during measurement validation; graph training pair.'
            rows.append(row)
        return {'loops': rows, 'whole_pair_holdout_geometry': geometry_metrics(sorted(reserved), poses, clouds)[0],
                'z_extent_m': float(np.ptp([p[2, 3] for p in poses.values()]))}
    comparison = {label: evidence(poses) for label, poses in baselines.items()}
    for sigma in sigmas:
        poses, solver = optimize_attitude(raw, odom, loops, components, gravity_sigma_degrees=sigma,
                                         max_evaluations=500, maximum_tilt_degrees=5.)
        label = f'candidate_attitude_{sigma:g}deg'
        folder = output/label
        folder.mkdir(parents=True, exist_ok=True)
        trajectory(folder/'trajectory.csv', poses, session.stamps)
        trajectory(folder/'main_trajectory.csv', {n: poses[n] for n in components[0]}, session.stamps)
        corrections = []
        for n in sorted(raw):
            original_up, final_up = raw[n][2, :3], poses[n][2, :3]
            corrections.append({'node': n,
                'translation_m': float(np.linalg.norm(poses[n][:3, 3]-raw[n][:3, 3])),
                'delta_z_m': float(poses[n][2, 3]-raw[n][2, 3]),
                'tilt_change_deg': float(np.degrees(np.arccos(np.clip(original_up @ final_up, -1, 1)))),
                'rotation_change_deg': float(np.degrees(Rotation.from_matrix(poses[n][:3, :3] @ raw[n][:3, :3].T).magnitude()))})
        report = {'created_utc': utc_now(), 'session': name, 'status': 'experimental_not_promoted',
            'command': shlex.join(sys.argv), 'solver': solver,
            'model_note': 'Per-node original-up-vector soft regularizer, not independent gravity evidence. No floor factor.',
            'loop_weight_model': covariance_model(),
            'source_hashes': {str(p): sha256_file(p) for p in
                (Path(__file__), ROOT/'src/optimize_bounded_attitude.py', ROOT/'src/optimize_session.py', source)},
            'preserved_whole_pair_holdouts': [list(p) for p in sorted(reserved)],
            'independent_graph_improvement_claim_allowed': bool(reserved),
            'baselines': comparison, 'candidate': evidence(poses),
            'corrections': corrections,
            'max_translation_correction_m': max(r['translation_m'] for r in corrections),
            'max_abs_z_correction_m': max(abs(r['delta_z_m']) for r in corrections),
            'source_inputs': {'db': str(session.path), 'revalidated_constraints': str(source.resolve())}}
        (folder/'evaluation.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({'variant': label, 'solver': solver, 'max_translation_m': report['max_translation_correction_m'],
                          'max_abs_z_m': report['max_abs_z_correction_m'], 'candidate': report['candidate']}), flush=True)
    session.connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', choices=SPECS, default='bridge_12_sub')
    parser.add_argument('--sigmas', type=float, nargs='+', default=[2., 5.])
    parser.add_argument('--input-root', type=Path, default=ROOT/'outputs/student_union_20261017_revalidated/intra_session')
    args = parser.parse_args()
    run(args.session, args.sigmas, args.input_root)


if __name__ == '__main__':
    main()
