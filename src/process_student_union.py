#!/usr/bin/env python3
"""Reproducible independent-session pipeline; overlap audit never merges by default."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

STAGES = ['audit', 'masks', 'optimize', 'export', 'overlap', 'render']
DATABASES = {'outside': '261016-234854-outside.db', 'lobby': '261017-000622-1f.db'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=Path('outputs/student_union_20261017_reviewed'))
    parser.add_argument('--use-semantic-masks', action='store_true',
                        help='Experimental: requires visual QC; low-light false positives rejected for current final run')
    parser.add_argument('--start-at', choices=STAGES, default='audit')
    parser.add_argument('--stop-after', choices=STAGES, default='render')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    os.chdir(workspace)
    root = args.run.resolve()
    if args.start_at == 'audit' and (root / 'audit/audit_manifest.json').exists():
        parser.error('Choose a fresh --run directory, or explicitly resume using --start-at.')
    root.mkdir(parents=True, exist_ok=True)
    (root / 'logs').mkdir(exist_ok=True)
    environment = os.environ.copy()
    environment.update({'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '4', 'MKL_NUM_THREADS': '1'})
    enabled = STAGES[STAGES.index(args.start_at):STAGES.index(args.stop_after) + 1]
    if not enabled:
        parser.error('--stop-after precedes --start-at')

    def execute(stage, label, arguments):
        if stage not in enabled:
            return
        command = [sys.executable, *map(str, arguments)]
        started = time.monotonic()
        stamp = datetime.now(timezone.utc).isoformat()
        log_path = root / 'logs' / (label + '.log')
        print(shlex.join(command), flush=True)
        with log_path.open('w') as stream:
            result = subprocess.run(command, cwd=workspace, env=environment, stdout=stream, stderr=subprocess.STDOUT)
        record = {'stage': stage, 'label': label, 'argv': command, 'cwd': str(workspace),
                  'started_utc': stamp, 'elapsed_s': time.monotonic() - started, 'exit_code': result.returncode,
                  'log': str(log_path), 'thread_environment': {k: environment[k] for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')}}
        with (root / 'commands.jsonl').open('a') as stream:
            stream.write(json.dumps(record) + '\n')
        if result.returncode:
            raise RuntimeError(f'{label} failed; inspect {log_path}')

    dbs = {name: workspace / 'data/raw/rtabmap' / filename for name, filename in DATABASES.items()}
    execute('audit', 'audit', ['src/audit_sessions.py', '--skip-plots', '--output', root / 'audit', *dbs.values()])
    for name, db in dbs.items():
        directory = root / name
        reference = root / 'audit' / name / 'app_optimized_trajectory.csv'
        mask_args = ['--mask-dir', directory / 'masks'] if args.use_semantic_masks else []
        if args.use_semantic_masks:
            execute('masks', 'mask_' + name, ['src/mask_dynamic_objects.py', db, '--output', directory / 'masks',
                    '--weights', root / 'model_cache/yolov8n-seg.pt'])
        optimize = ['src/optimize_session.py', db, '--output', directory, *mask_args,
                    '--reference-trajectory', reference]
        execute('optimize', 'optimize_' + name, optimize)
        ablation = list(optimize)
        ablation[ablation.index('--output') + 1] = directory / 'stored_covariance_baseline'
        execute('optimize', 'ablation_' + name, [*ablation, '--odom-model-sigma-m', '0', '--odom-model-sigma-degrees', '0'])
        for label, pose in [('raw', directory / 'raw_main_trajectory.csv'),
                            ('optimized', directory / 'optimized_main_trajectory.csv'), ('app', reference)]:
            execute('export', f'export_{name}_{label}', ['src/export_rtabmap_db.py', db,
                '--cloud', directory / f'cloud_{label}_filtered.ply',
                '--trajectory', directory / f'export_{label}_trajectory.csv',
                '--summary', directory / f'export_{label}.json', '--pose-csv', pose,
                '--missing-pose-policy', 'skip', *mask_args,
                '--frame-stride', '1', '--min-depth', '0.25', '--max-depth', '4.0', '--voxel', '0.05',
                '--min-confidence', '50'])
    overlap_masks = ['--mask-dir-a', root / 'outside/masks', '--mask-dir-b', root / 'lobby/masks'] if args.use_semantic_masks else []
    execute('overlap', 'cross_session', ['src/check_session_overlap.py', '--outside-db', dbs['outside'],
        '--lobby-db', dbs['lobby'], '--output', root / 'cross_session', '--candidate-limit', '240',
        '--outside-poses', root / 'outside/optimized_main_trajectory.csv',
        '--lobby-poses', root / 'lobby/optimized_main_trajectory.csv', *overlap_masks])
    for name in dbs:
        execute('render', 'render_' + name, ['src/render_session_report.py', root / name,
            '--reference', root / 'audit' / name / 'app_optimized_trajectory.csv', '--title', 'Student Union / ' + name])
    print('Completed requested stages. Check cross_session/overlap_report.json; no forced merge is performed.', flush=True)


if __name__ == '__main__':
    main()
