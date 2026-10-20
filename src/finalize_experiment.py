#!/usr/bin/env python3
"""Hash experiment outputs and source files; preserve compact numerical evidence for Git."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import subprocess
from zoneinfo import ZoneInfo


def fingerprint(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return {'bytes': path.stat().st_size, 'sha256': digest.hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=Path('outputs/student_union_20261017_reviewed'))
    parser.add_argument('--evidence', type=Path, default=Path('docs/experiments/student_union_20261017_metrics.json'))
    parser.add_argument('--sessions', nargs='+', default=['outside', 'lobby'])
    parser.add_argument('--independent', action='store_true', help='This run intentionally did not perform any cross-session registration')
    args = parser.parse_args()
    if len(set(args.sessions)) != len(args.sessions) or any(not re.fullmatch(r'[a-z][a-z0-9_]*', name) for name in args.sessions):
        parser.error('Session identifiers must be unique lowercase letters/digits/underscores, beginning with a letter')
    workspace = Path(__file__).resolve().parents[1]
    root = args.run.resolve()
    reports = {}
    for name in args.sessions:
        audit = json.loads((root / 'audit' / (name + '_audit.json')).read_text())
        db = Path(audit['db_path'])
        actual = fingerprint(db)
        if actual['sha256'] != audit['sha256_before']:
            raise RuntimeError(f'Original DB changed: {db}')
        optimization = json.loads((root / name / 'optimization.json').read_text())
        ablation = json.loads((root / name / 'stored_covariance_baseline/optimization.json').read_text())
        reports[name] = {
            'input_db': {'file': db.name, **actual}, 'optimization': optimization,
            'audit_summary': {'sqlite_quick_check': audit['quick_check'], 'node_count': audit['nodes']['count'],
                              'app_pose_count': audit['app_optimized_poses']['count'],
                              'raw_position_extent_xyz_m': audit['nodes']['raw_position_extent_xyz_m'],
                              'frames_decoded': audit['frames']['all_frames_decoded'],
                              'gnss_records': audit['gnss']['records'],
                              'gnss_reported_accuracy_m': audit['gnss']['reported_accuracy_m'],
                              'gnss_coarse_view_eligible_count': audit['gnss']['coarse_view_eligible_count'],
                              'original_preservation': audit['preservation']},
            'exports': {label: json.loads((root / name / f'export_{label}.json').read_text()) for label in ('raw', 'app', 'optimized')},
            'stored_covariance_ablation': {k: ablation[k] for k in ('parameters', 'solver', 'metrics', 'corrections')},
        }
    overlap = None if args.independent else json.loads((root / 'cross_session/overlap_report.json').read_text())
    if args.independent and (root / 'cross_session/overlap_report.json').exists():
        raise RuntimeError('An overlap report exists; do not label this run as registration-not-performed')
    masks_by_stage = {}
    for name, session in reports.items():
        masks_by_stage[name] = {
            'optimization': session['optimization']['parameters']['mask_dir'],
            'exports': {label: report['mask_directory'] for label, report in session['exports'].items()},
            'overlap': overlap['semantic_mask_directories'][name] if overlap else None,
        }
        paths = [masks_by_stage[name]['optimization'], *masks_by_stage[name]['exports'].values()]
        if overlap:
            paths.append(masks_by_stage[name]['overlap'])
        normalized = {str(Path(path).resolve()) if path else None for path in paths}
        if len(normalized) != 1:
            raise RuntimeError(f'Mixed semantic-mask configuration across stages: {name}')
    masks_enabled = any(v['optimization'] for v in masks_by_stage.values())
    # Numerical evidence only: no original imagery, per-frame coordinates, or weights in Git.
    for session in reports.values():
        for component in session['optimization']['components']:
            component.pop('node_ids', None)
    evidence = {
        'created_kst': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
        'run_directory': str(root), 'repository_base_commit_before_result_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=workspace, text=True).strip(),
        'semantic_masks_by_stage': masks_by_stage,
        'semantic_mask_decision': ('Experimental masks enabled; visual acceptance must be documented separately.' if masks_enabled else
            'Disabled based on the previous reviewed run mask-QC decision (low-light static-object false positives); no new masking trial in this independent run.' if args.independent else
            'Disabled. Current reviewed run rejected low-light static pillars, shadows, posters and vending-machine false positives; the masked trial remains in outputs/student_union_20261017.'),
        'verification': json.loads((root / 'verification.json').read_text()) if (root / 'verification.json').exists() else None,
        'run_definition': json.loads((root / 'run_definition.json').read_text()) if (root / 'run_definition.json').exists() else None,
        'sessions': reports,
        'cross_session': {k: overlap[k] for k in ('status', 'parameters', 'node_counts', 'verified_candidate_count',
                            'accepted_pair_count', 'rejection_counts', 'independent_consistent_link_count',
                            'retrieval_candidate_count', 'retrieval_index_descriptor_count', 'stage_execution_counts',
                            'command', 'source_code_sha256', 'library_versions', 'pair_results_reused',
                            'pair_verification_source_sha256', 'semantic_mask_fingerprints', 'algorithms', 'limitations')}
                         if overlap else {'status': 'not_requested_not_run', 'merged': False},
        'not_performed': ['GNSS pose priors', 'floor flattening', 'extrinsic recalibration',
                          'Cross-session registration and joint graph/map merge (not requested)' if args.independent else 'joint graph/map merge if overlap rejected',
                          'TSDF meshing and route generation deferred; this run evaluates independent session optimization and clouds'
                          if args.independent else 'TSDF meshing and route generation deferred; this run evaluates session registration and clouds'],
    }
    args.evidence.parent.mkdir(parents=True, exist_ok=True)
    args.evidence.write_text(json.dumps(evidence, indent=2, ensure_ascii=False) + '\n')
    source_paths = sorted([*workspace.glob('src/*.py'), *workspace.glob('scripts/*.sh'), *workspace.glob('tests/*.py'),
                           *workspace.glob('config/*.json'), *workspace.glob('config/*.rviz'),
                           *workspace.glob('requirements-processing.txt')])
    manifest = {'created_kst': evidence['created_kst'], 'run_directory': str(root),
                'source_files': {str(p.relative_to(workspace)): fingerprint(p) for p in source_paths},
                'outputs': {str(p.relative_to(root)): fingerprint(p) for p in sorted(root.rglob('*'))
                            if p.is_file() and p.name not in ('artifact_manifest.json', 'rviz_publisher.log')},
                'raw_database_unchanged': True}
    (root / 'artifact_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'evidence': str(args.evidence), 'manifest': str(root / 'artifact_manifest.json'),
                      'files': len(manifest['outputs']), 'raw_database_unchanged': True}))


if __name__ == '__main__':
    main()
