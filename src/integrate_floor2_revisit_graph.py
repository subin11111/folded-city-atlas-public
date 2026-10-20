#!/usr/bin/env python3
"""Integrate training-gated RGB-D BA measurements as an UNAPPROVED experiment.

An old factor with the same endpoints is archived, not counted twice. Previously
inspected heldout endpoints become development; historical metrics are no longer
pristine tests. Input poses, bounds and gauge remain those of the original graph.
Whole-frame evaluation and map promotion are deliberately separate operations.
"""
from __future__ import annotations

import argparse
import copy
import csv
import json
from pathlib import Path
import shutil
import sys

import numpy as np

from continue_floor2_graph import continue_graph
from optimize_floor2_frames import node_key
from optimize_robust_submaps import parse_manifest, transform_record
from optimize_session import sha256_file, trajectory, utc_now

ROOT = Path(__file__).resolve().parents[1]


def integrate(graph, remeasurement, sigma_multiplier=1.):
    if not np.isfinite(sigma_multiplier) or sigma_multiplier < 1:
        raise ValueError('Uncertainty multiplier must be finite and >= 1')
    if not remeasurement.get('complete') or not remeasurement.get('inputs_unchanged'):
        raise ValueError('Incomplete or changed remeasurement inputs')
    result = copy.deepcopy(graph)
    node_ids = {n['id'] for n in result['submaps']}
    accepted = [r for r in remeasurement['records'] if r.get('accepted')]
    train_ids = {n for r in accepted for visit in r['train_frames'] for n in visit}
    test_ids = {n for r in remeasurement['records'] for visit in r['test_frames'] for n in visit}
    if train_ids & test_ids:
        raise ValueError('Invalid cross-region train/test split')
    archived, added, seen = [], [], set()
    for row in remeasurement['records']:
        if not row.get('accepted'):
            continue
        bundle = row.get('bundle', {})
        if not bundle.get('ba_executed') or not bundle.get('accepted'):
            raise ValueError('A geometry-only hypothesis cannot enter as RGB-D BA')
        if row.get('measurement_kind') != 'joint_multiframe_RGBD_pose_landmark_BA':
            raise ValueError('Wrong measurement kind')
        a, b = node_key(row['a']), node_key(row['b'])
        pair = frozenset((a, b))
        if len(pair) != 2 or pair in seen or not pair <= node_ids:
            raise ValueError('Missing, duplicate or self endpoints')
        seen.add(pair)
        train = set(sum(row['train_frames'], []))
        test = set(sum(row['test_frames'], []))
        if train & test or not {row['a'], row['b']} <= train:
            raise ValueError('Invalid whole-frame train/test split')
        old = [e for e in result['edges'] if frozenset((e['from'], e['to'])) == pair
               and e['type'] in ('loop', 'registration')]
        groups = {e['region_group'] for e in old}
        if len(groups) > 1:
            raise ValueError('Old factors disagree on physical correlation group')
        result['edges'] = [e for e in result['edges'] if e not in old]
        archived.extend(old)
        edge_id = f"revisit_rgbd_ba:{row['a']}:{row['b']}"
        covariance = np.asarray(row['covariance_6x6']) * sigma_multiplier**2
        result['edges'].append(dict(id=edge_id, **{'from': a, 'to': b},
            measurement=row['T_a_b'], covariance=covariance.tolist(), type='loop', role='train',
            region_group=next(iter(groups), f"floor2_revisit:{row['a']}:{row['b']}"),
            provenance=dict(source='joint_multiframe_RGBD_pose_landmark_BA',
                frame_endpoints=['floor2', row['a'], 'floor2', row['b']],
                train_frame_ids=sorted(train), withheld_frame_ids=sorted(test),
                replaced_factor_ids=[e['id'] for e in old],
                former_heldout_now_development=any(e['role'] == 'heldout' for e in old),
                sigma_multiplier=sigma_multiplier,
                covariance_is_model_assumption_not_BA_marginal=True,
                local_pose_priors_correlated_with_existing_odometry=True,
                single_region_factor_not_one_factor_per_correlated_frame=True)))
        added.append(edge_id)
    if not added:
        raise ValueError('No training-gated BA measurements; graph unchanged')
    result['metadata'].update(created_utc=utc_now(), accepted=False, navigation=False,
        experiment='floor2_revisit_BA_integration', added_revisit_factor_ids=added,
        archived_replaced_factors=archived, revisit_sigma_multiplier=sigma_multiplier,
        whole_frame_evaluation_pending=True,
        integration_scope='Diagnostic candidate only, not publication or independent validation',
        joint_BA_scope='Local training windows; whole-session solve remains pose-only PGO')
    parse_manifest(result)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--remeasurements', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--sigma-multiplier', type=float, default=1.)
    p.add_argument('--quality', type=Path, default=ROOT/'outputs/student_union_20261017_submap_quality/floor2/frames.csv')
    p.add_argument('--outer-iterations', type=int, default=16)
    p.add_argument('--max-evaluations', type=int, default=500)
    a = p.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    manifest_path, report_path = a.source/'manifest.json', a.source/'report.json'
    original, prior = json.loads(manifest_path.read_text()), json.loads(report_path.read_text())
    if prior['graph_manifest_sha256'] != sha256_file(manifest_path):
        raise ValueError('Warm report graph binding mismatch')
    measured = json.loads(a.remeasurements.read_text())
    for path, digest in measured['input_hashes'].items():
        if sha256_file(Path(path)) != digest: raise ValueError(f'Remeasurement input changed: {path}')
    if original['metadata']['input_hashes'][str(a.quality.resolve())] != sha256_file(a.quality):
        raise ValueError('Quality source mismatch')
    initial = ROOT/'outputs/student_union_20261017_geometry_review/graph_final/floor2/initial.csv'
    if measured['input_hashes'].get(str(initial)) != original['metadata']['input_hashes'].get(str(initial)):
        raise ValueError('Revisit BA and frame graph initial pose sources differ')
    graph = integrate(original, measured, a.sigma_multiplier)
    nodes, edges = parse_manifest(graph)
    sources = [Path(__file__), ROOT/'src/continue_floor2_graph.py',
        ROOT/'src/optimize_floor2_frames.py', ROOT/'src/floor2_vectorized_solver.py',
        ROOT/'src/optimize_robust_submaps.py', ROOT/'src/optimize_session.py']
    paths = sources + [manifest_path, report_path, a.remeasurements, a.quality]
    hashes = {str(f.resolve()): sha256_file(f) for f in paths}
    a.output.mkdir(parents=True)
    for f in sources: shutil.copyfile(f, a.output/('source_'+f.name))
    (a.output/'manifest.json').write_text(json.dumps(graph, indent=2)+'\n')
    def checkpoint(row, poses, x):
        with (a.output/'iterations.jsonl').open('a') as stream:
            stream.write(json.dumps(row)+'\n')
        (a.output/'checkpoint.json').write_text(json.dumps(dict(iteration=row,
            poses={k: transform_record(v) for k, v in poses.items()}))+'\n')
        print(json.dumps(row), flush=True)
    poses, variant = continue_graph(nodes, edges, prior['variants']['baseline_huber'],
        outer_iterations=a.outer_iterations, max_evaluations=a.max_evaluations,
        linear_maxiter=10000, jacobian_scheme='3-point', checkpoint=checkpoint)
    ids = {n['id']: n['frame_id'] for n in graph['submaps']}
    frames = {int(r['node_id']): r for r in csv.DictReader(a.quality.open())}
    poses = {ids[k]: v for k, v in poses.items()}
    stamps = {k: float(r['stamp']) for k, r in frames.items()}
    ab = {k: v for k, v in poses.items() if frames[k]['quality_grade'] in ('A', 'B')}
    trajectory(a.output/'trajectory.csv', poses, stamps)
    trajectory(a.output/'trajectory_AB.csv', ab, stamps)
    unchanged = all(sha256_file(Path(k)) == v for k, v in hashes.items())
    if not unchanged: raise RuntimeError('Inputs changed during optimization')
    report = dict(complete=True, accepted=False, navigation=False, command=sys.argv,
        input_hashes=hashes, input_hashes_unchanged=unchanged,
        graph_manifest_sha256=sha256_file(a.output/'manifest.json'),
        trajectory_sha256=sha256_file(a.output/'trajectory.csv'),
        trajectory_AB_sha256=sha256_file(a.output/'trajectory_AB.csv'),
        full_frame_count=len(poses), export_AB_count=len(ab),
        default_rviz_replaced=False, other_sessions_modified=False,
        variants={'baseline_huber': variant})
    (a.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__': main()
