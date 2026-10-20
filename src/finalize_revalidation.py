#!/usr/bin/env python3
"""Collect completed experiment evidence; never promotes a map or edits a DB."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path,
                        default=Path('outputs/student_union_20261017_revalidated'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Choose a fresh metrics file: ' + str(args.output))
    sources = {}

    def read(path):
        path = Path(path)
        sources[str(path)] = digest(path)
        return json.loads(path.read_text())

    config = read('config/student_union_all_sessions.json')['sessions']
    root = args.run
    intra = read(root / 'intra_session/constraints.json')
    if not intra['complete']:
        raise ValueError('Intra-session verification is incomplete')
    accepted = [row for row in intra['records'] if row['accepted']]
    data = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'status': 'experiments_complete_no_joint_map_accepted',
        'run': str(root),
        'absolute_accuracy_verified': False,
        'intra_session': {
            'checked': len(intra['records']),
            'accepted': len(accepted),
            'training_eligible': sum(bool(r['eligible_for_joint']) for r in accepted),
            'accepted_by_session': dict(Counter(r['session'] for r in accepted)),
            'accepted_pairs': [{k: r[k] for k in
                               ('session', 'node_a', 'node_b', 'held_out', 'eligible_for_joint')}
                              for r in accepted],
        },
    }
    cross = []
    for experiment in ('cross_session', 'cross_session_clahe_depth',
                       'cross_session_planar_bridge', 'cross_session_planar_interior'):
        for path in sorted((root / experiment).glob('*__*/report.json')):
            report = read(path)
            cross.append({
                'experiment': experiment,
                **{k: report[k] for k in ('session_a', 'session_b', 'checked_pairs',
                                         'local_geometry_verified_pairs', 'eligible_for_joint_pairs')},
            })
    data['sift_cross_session'] = {
        'attempts': sum(r['checked_pairs'] for r in cross),
        'count_note': 'Repeated frame pairs across verification policies are included; not unique pairs.',
        'runs': cross,
    }
    learned = []
    for experiment, pairs in [
        ('learned_cross_session', ('outside__bridge_out1f', 'lobby__bridge_out1f', 'lobby__floor2')),
        ('learned_cross_session_continuation', ('lobby__bridge_12_sub', 'floor2__bridge_12_sub')),
    ]:
        for pair in pairs:
            report = read(root / experiment / pair / 'report.json')
            learned.append({
                'experiment': experiment,
                **{k: report[k] for k in ('session_a', 'session_b', 'checked_pairs',
                                         'local_verified', 'eligible_for_joint')},
            })
    data['learned_cross_session'] = {
        'completed_attempts': sum(r['checked_pairs'] for r in learned),
        'runs': learned,
        'interrupted_trial_note': 'Original lobby/substairs partial checkpoint is retained but not counted. Completed continuation has separate source provenance.',
    }
    learned_intra = read(root / 'learned_intra/intra_priority/report.json')
    data['learned_intra'] = {'checked': len(learned_intra['records']),
                             'accepted': learned_intra['accepted']}
    data['targeted_original_holdout_audit'] = read(root / 'targeted_holdout_check/report.json')
    data['joint_trials'] = {}
    for path in sorted(root.glob('joint_*/joint_report.json')):
        report = read(path)
        components = [{k: c[k] for k in ('sessions', 'accepted', 'rejection_reasons',
                                         'solver', 'cross_constraints', 'cross_whole_pair_heldout',
                                         'training_errors', 'heldout_errors')}
                      for c in report['components']]
        data['joint_trials'][path.parent.name] = {
            'status': report['status'], 'components': components,
            'disconnected_sessions': report['disconnected_sessions'],
        }
    if any(c['accepted'] for r in data['joint_trials'].values() for c in r['components']):
        raise ValueError('An accepted joint candidate exists; revise the final disposition explicitly')
    data['independent_candidates'] = {}
    for path in sorted((root / 'intra_session').glob('*/candidate_*/evaluation.json')):
        report = read(path)
        data['independent_candidates'][str(path.parent.relative_to(root))] = {
            k: report[k] for k in ('status', 'solver', 'whole_pair_holdouts',
                                   'heldout_regression_vs_previous_candidate',
                                   'new_constraint_realized_residuals',
                                   'max_translation_correction_m', 'max_abs_z_correction_m',
                                   'independent_graph_improvement_claim_allowed') if k in report}
    data['surface_evaluation'] = {}
    integrity = {}
    for experiment in ('surface_evaluation', 'surface_evaluation_old',
                       'surface_evaluation_sub_candidates', 'surface_evaluation_joint_trial'):
        for path in sorted((root / experiment).glob('*/report.json')):
            report = read(path)
            data['surface_evaluation'][str(path.parent.relative_to(root))] = {
                k: report[k] for k in ('sampled_nodes', 'accepted_horizontal_patches',
                                       'accepted_wall_patches', 'pair_aggregates')}
            if experiment in ('surface_evaluation', 'surface_evaluation_old'):
                name = report['session']
                current = digest(config[name]['database'])
                integrity[name] = {
                    'path': config[name]['database'],
                    'sha256_before_surface_audit': report['database_sha256_before'],
                    'sha256_after_surface_audit': report['database_sha256_after'],
                    'sha256_at_finalization': current,
                    'unchanged': current == report['database_sha256_before'] == report['database_sha256_after'],
                }
    if set(integrity) != set(config) or not all(r['unchanged'] for r in integrity.values()):
        raise ValueError('Database integrity check failed')
    data['database_integrity'] = integrity
    data['new_clouds'] = {}
    for path in sorted((root / 'intra_session/bridge_12_sub').glob('candidate_*/export.json')):
        report = read(path)
        cloud = path.parent / 'cloud.ply'
        data['new_clouds'][str(cloud)] = {
            **{k: report[k] for k in ('output_points', 'selected_depth_frames', 'decoded_depth_frames',
                                     'voxel_size_m', 'confidence_threshold', 'depth_range_m',
                                     'frame_stride', 'pixel_stride')},
            'sha256': digest(cloud), 'promoted_to_default': False,
        }
    environment = os.environ.copy()
    environment.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='2')
    test_command = [sys.executable, '-m', 'pytest', '-q']
    tests = subprocess.run(test_command, env=environment, capture_output=True, text=True)
    data['tests'] = {'command': test_command, 'exit_code': tests.returncode,
                     'stdout': tests.stdout, 'stderr': tests.stderr}
    for pattern in ('src/*.py', 'tests/*.py', 'config/student_union_all_sessions.json'):
        for path in sorted(Path('.').glob(pattern)):
            sources[str(path)] = digest(path)
    data['source_sha256_at_finalization'] = sources
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({'metrics': str(args.output), 'sift_attempts': data['sift_cross_session']['attempts'],
                      'learned_attempts': data['learned_cross_session']['completed_attempts'],
                      'database_unchanged': True, 'tests_exit_code': tests.returncode}, indent=2))
    print(tests.stdout)
    return tests.returncode


if __name__ == '__main__':
    raise SystemExit(main())
