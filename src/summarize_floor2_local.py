#!/usr/bin/env python3
"""Bind floor2 solve/evaluation/export artifacts and recheck inputs at handoff.

This is a reproducibility and conditional-consistency receipt, not a surveyed
accuracy certificate. It never promotes a candidate or changes a map.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import shutil
import sys

import numpy as np

from export_rtabmap_db import load_pose_csv
from optimize_robust_submaps import _as_transform
from optimize_session import sha256_file, utc_now
from summarize_robust_submaps import canonical_manifest_sha256, connected_components

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT/'outputs/student_union_20261017_floor2_local_v1'


def read(path):
    return json.loads(Path(path).read_text())


def verify_hashes(hashes):
    for path, expected in hashes.items():
        if sha256_file(Path(path)) != expected:
            raise ValueError(f'Hash mismatch at handoff: {path}')


def graph_ablation(baseline, refined):
    if baseline['submaps'] != refined['submaps']:
        raise ValueError('Ablation node inventories/initial poses differ')
    before = {e['id']: e for e in baseline['edges']}
    after = {e['id']: e for e in refined['edges']}
    if len(before) != len(baseline['edges']) or len(after) != len(refined['edges']):
        raise ValueError('Duplicate edge IDs')
    if any(after.get(key) != edge for key, edge in before.items()):
        raise ValueError('Ablation modified a shared factor')
    added = [e for key, e in after.items() if key not in before]
    if len(added) != 3 or any(e['type'] != 'registration' or e['role'] != 'train' for e in added):
        raise ValueError('Expected exactly three auxiliary training registrations')
    nodes = [n['id'] for n in baseline['submaps']]
    train = [e for e in baseline['edges'] if e['role'] == 'train']
    weak = [e for e in train if e['type'] == 'structural']
    components = len(connected_components(nodes, train))
    bridges = [e['id'] for e in weak if len(connected_components(
        nodes, [r for r in train if r['id'] != e['id']])) > components]
    return {'same_initial_nodes': True, 'all_shared_factors_identical': True,
            'shared_factor_count': len(before), 'added_registration_ids': [e['id'] for e in added],
            'baseline_components': components,
            'refined_components': len(connected_components(nodes, [e for e in refined['edges'] if e['role']=='train'])),
            'baseline_components_without_structural': len(connected_components(
                nodes, [e for e in train if e['type'] != 'structural'])),
            'structural_count': len(weak), 'structural_graph_bridge_count': len(bridges),
            'structural_graph_bridge_ids': bridges,
            'baseline_edge_types': dict(Counter(e['type'] for e in baseline['edges'])),
            'limitation': 'Structural continuity couples modules; a graph bridge residual cannot independently validate its measurement.'}


def verify_solve(directory, graph):
    report = read(directory/'report.json')
    if not report['input_hashes_unchanged']:
        raise ValueError('Solve did not preserve inputs')
    verify_hashes(report['source_hashes'])
    if report['provenance']['canonical_manifest_sha256'] != canonical_manifest_sha256(graph):
        raise ValueError('Solver report not bound to this graph')
    if sha256_file(directory/'manifest.json') != report['graph_manifest_sha256']:
        raise ValueError('Graph file changed')
    for filename, key in [('trajectory.csv','trajectory_sha256'), ('trajectory_AB.csv','trajectory_AB_sha256')]:
        if sha256_file(directory/filename) != report[key]:
            raise ValueError('Trajectory file changed')
    variant = report['variants']['baseline_huber']
    expected = {e['id'] for e in graph['edges'] if e['role'] == 'train'}
    if set(variant['fit_edge_ids']) != expected or variant['heldout_edge_count_in_objective']:
        raise ValueError('Fit/holdout identity mismatch')
    if set(variant['heldout_edge_ids']) != {e['id'] for e in graph['edges'] if e['role']=='heldout'}:
        raise ValueError('Heldout identity mismatch')
    if set(variant['poses']) != {n['id'] for n in graph['submaps']}:
        raise ValueError('Solved pose node identities differ from graph')
    poses = load_pose_csv(directory/'trajectory.csv')
    if set(poses) != {n['frame_id'] for n in graph['submaps']}:
        raise ValueError('Pose export coverage mismatch')
    for node in graph['submaps']:
        if not np.allclose(poses[node['frame_id']], _as_transform(variant['poses'][node['id']],node['id']),atol=2e-8,rtol=0):
            raise ValueError('Exported pose differs from solved pose')
    usable = load_pose_csv(directory/'trajectory_AB.csv')
    if set(usable) != {n['frame_id'] for n in graph['submaps'] if n['quality'] in ('A','B')}:
        raise ValueError('A/B pose export coverage mismatch')
    if len(usable) != report['export_AB_count']:
        raise ValueError('A/B pose count differs from report')
    for node, pose in usable.items():
        if not np.allclose(pose, poses[node],atol=2e-8,rtol=0):
            raise ValueError('A/B pose differs from full solved pose')
    iterations = variant['solver_iterations']
    last = iterations[-1]
    return {'input_and_trajectory_hashes_rechecked': True,
            'report_graph_trajectory_binding_verified': True,
            'all_inner_solves_success': all(i['success'] for i in iterations),
            'outer_strict_stopping_condition_met': last['maximum_weight_change'] < 1e-5 and last['maximum_parameter_step'] < 1e-7,
            'outer_iterations': len(iterations), 'total_nfev': sum(i['nfev'] for i in iterations),
            'active_bounds_final': last['active_bounds'], 'last_iteration': last,
            'full_pose_count': len(poses), 'AB_pose_count': len(usable),
            'heldout_edges': variant['heldout_edge_ids'], 'components': variant['component_count'],
            'local_factors': [e for e in variant['edges'] if e['type']=='registration']}


def summarize(run, output):
    if output.exists():
        raise FileExistsError(output)
    graphs = {name: read(run/name/'manifest.json') for name in ('frame_baseline','frame_refined')}
    ablation = graph_ablation(graphs['frame_baseline'], graphs['frame_refined'])
    solves = {name: verify_solve(run/name, graph) for name,graph in graphs.items()}
    evaluation = read(run/'final_evaluation/report.json')
    for row in evaluation['pose_trajectories'].values():
        verify_hashes({row['path']:row['sha256']})
    for name in solves:
        if Path(evaluation['pose_trajectories'][name]['path']).resolve() != (run/name/'trajectory.csv').resolve():
            raise ValueError('Evaluation did not use the expected solve output')
    clouds = read(run/'clouds/manifest.json')
    cloud_records = {}
    for name,row in clouds['variants'].items():
        directory = run/'clouds'/name
        manifest_path = run/'clouds'/row['manifest']
        verify_hashes({str(manifest_path):row['manifest_sha256']})
        manifest = read(manifest_path)
        verify_hashes(manifest['input_hashes_before'])
        verify_hashes(manifest['rviz_hashes_before'])
        verify_hashes({manifest['source_database']: manifest['database_sha256_before']})
        verify_hashes({str(directory/file): digest for file,digest in manifest['output_hashes'].items()})
        cloud_records[name] = {'points':row['point_count'], 'contributing_frames':manifest['inventory']['decoded_depth_frames'],
                              'current_inputs_rviz_and_outputs_rehashed':True, 'cloud':str(run/'clouds'/row['cloud'])}
    reports = [run/'final_evaluation/report.json',run/'clouds/manifest.json',
               run/'remeasurements/report.json',run/'remeasurement_audit_v2/report.json']
    reports += [run/name/file for name in graphs for file in ('report.json','manifest.json')]
    result = {'created_utc':utc_now(), 'command':sys.argv, 'accepted':False, 'navigation':False,
              'default_rviz_replaced':False, 'ablation':ablation, 'solves':solves,
              'common_support_all_stages':evaluation['common_support_all_stages'],
              'clouds':cloud_records, 'report_hashes':{str(p):sha256_file(p) for p in reports},
              'source_sha256':sha256_file(Path(__file__)),
              'limitations':['Historical internal consistency, not independent ground truth.',
                             'No global promotion; mathematical convergence and code tests do not certify a map.',
                             'Weak continuity assumptions remain at low-quality/boundary frames.']}
    output.mkdir(parents=True)
    shutil.copyfile(__file__,output/'source_snapshot.py')
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'output':str(output),'accepted':False,'solves':solves},indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,default=RUN)
    p.add_argument('--output',type=Path,default=RUN/'handoff_audit')
    a=p.parse_args()
    summarize(a.run.resolve(),a.output.resolve())


if __name__=='__main__':main()
