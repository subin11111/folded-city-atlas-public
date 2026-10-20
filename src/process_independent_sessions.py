#!/usr/bin/env python3
"""Reuse the reviewed no-mask algorithm for named sessions, without any merging."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time

STAGES = ('audit', 'optimize', 'export', 'render')


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_definition(path: Path) -> dict:
    definition = json.loads(path.read_text())
    sessions = definition.get('sessions', {})
    if not sessions:
        raise ValueError('Configuration must contain at least one session')
    for name, entry in sessions.items():
        if not re.fullmatch(r'[a-z][a-z0-9_]*', name):
            raise ValueError(f'Unsafe session identifier: {name}')
        if not entry.get('database'):
            raise ValueError(f'Missing database for {name}')
    return definition


def check_comparison_coverage(directory: Path, reference: Path) -> None:
    def ids(path):
        with path.open() as stream:
            return {int(row['id']) for row in csv.DictReader(stream)}
    main_ids = ids(directory / 'raw_main_trajectory.csv')
    if not main_ids or main_ids != ids(reference):
        raise ValueError('App pose coverage differs from the main component; explicit frame selection is required')
    if main_ids != ids(directory / 'optimized_main_trajectory.csv'):
        raise ValueError('Raw and optimized main-component pose coverage differ')


def write_run_definition(root: Path, metadata: dict) -> None:
    """Serialize the short initialization step when session workers run concurrently."""
    with (root / '.definition.lock').open('a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        metadata_path = root / 'run_definition.json'
        if metadata_path.exists():
            if json.loads(metadata_path.read_text()) != metadata:
                raise ValueError('Existing run uses a different configuration; choose a fresh output directory')
        else:
            with metadata_path.open('x') as stream:
                json.dump(metadata, stream, ensure_ascii=False, indent=2)
                stream.write('\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('config/student_union_added_sessions.json'))
    parser.add_argument('--run', type=Path, default=Path('outputs/student_union_20261017_added_reviewed'))
    parser.add_argument('--sessions', nargs='+', help='Process only these configured sessions, e.g. when resuming')
    parser.add_argument('--start-at', choices=STAGES, default='audit')
    parser.add_argument('--stop-after', choices=STAGES, default='render')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    os.chdir(workspace)
    definition = load_definition(args.config)
    names = args.sessions or list(definition['sessions'])
    if len(set(names)) != len(names) or set(names) - set(definition['sessions']):
        parser.error('--sessions must be unique configured names')
    enabled = STAGES[STAGES.index(args.start_at):STAGES.index(args.stop_after) + 1]
    if not enabled:
        parser.error('--stop-after precedes --start-at')
    root = args.run.resolve()
    if 'audit' in enabled and (root / 'audit/audit_manifest.json').exists():
        parser.error('Choose a new --run directory, or explicitly resume with --start-at')
    databases = {name: (workspace / definition['sessions'][name]['database']).resolve() for name in names}
    if any(not path.is_file() for path in databases.values()):
        parser.error('A configured database does not exist')
    root.mkdir(parents=True, exist_ok=True)
    (root / 'logs').mkdir(exist_ok=True)
    metadata = {'configuration': definition, 'configuration_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
                'configuration_path': str(args.config.resolve()), 'semantic_masks': False,
                'cross_session_registration': 'not_requested_not_run', 'floor_flattening': False,
                'same_scientific_settings_as': 'outputs/student_union_20261017_reviewed'}
    write_run_definition(root, metadata)
    environment = os.environ.copy()
    environment.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')

    def execute(stage, name, arguments):
        if stage not in enabled:
            return
        command = [sys.executable, *map(str, arguments)]
        log_path = root / 'logs' / f'{name}_{stage}_{Path(arguments[0]).stem}.log'
        started, stamp = time.monotonic(), datetime.now(timezone.utc).isoformat()
        print(shlex.join(command), flush=True)
        with log_path.open('w') as stream:
            result = subprocess.run(command, cwd=workspace, env=environment, stdout=stream, stderr=subprocess.STDOUT)
        record = {'stage': stage, 'argv': command, 'cwd': str(workspace), 'started_utc': stamp,
                  'elapsed_seconds': time.monotonic() - started, 'exit_code': result.returncode, 'log': str(log_path),
                  'source_sha256': hashlib.sha256((workspace / arguments[0]).read_bytes()).hexdigest(),
                  'thread_environment': {key: environment[key] for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')}}
        with (root / f'commands_{name}.jsonl').open('a') as stream:
            stream.write(json.dumps(record) + '\n')
        if result.returncode:
            raise RuntimeError(f'{name}/{stage} failed; see {log_path}')

    execute('audit', 'audit', ['src/audit_sessions.py', *databases.values(), '--sessions', *names,
                              '--skip-plots', '--output', root / 'audit'])
    for name, db in databases.items():
        directory, reference = root / name, root / 'audit' / name / 'app_optimized_trajectory.csv'
        if any(stage in enabled for stage in ('optimize', 'export', 'render')):
            audit = json.loads((root / 'audit' / f'{name}_audit.json').read_text())
            if Path(audit['db_path']).resolve() != db or not audit['frames']['all_frames_decoded']:
                raise ValueError(f'Audit mismatch or decoding failures: {name}')
            if file_sha256(db) != audit['sha256_before']:
                raise ValueError(f'Database changed since audit: {name}; use a new run')
        optimize = ['src/optimize_session.py', db, '--output', directory, '--reference-trajectory', reference]
        execute('optimize', name, optimize)
        baseline = list(optimize)
        baseline[baseline.index('--output') + 1] = directory / 'stored_covariance_baseline'
        execute('optimize', name + '_ablation', [*baseline, '--odom-model-sigma-m', '0', '--odom-model-sigma-degrees', '0'])
        if 'export' in enabled or 'render' in enabled:
            check_comparison_coverage(directory, reference)
        for label, poses in [('raw', directory / 'raw_main_trajectory.csv'),
                             ('optimized', directory / 'optimized_main_trajectory.csv'), ('app', reference)]:
            execute('export', name + '_' + label, ['src/export_rtabmap_db.py', db,
                '--cloud', directory / f'cloud_{label}_filtered.ply', '--trajectory', directory / f'export_{label}_trajectory.csv',
                '--summary', directory / f'export_{label}.json', '--pose-csv', poses, '--missing-pose-policy', 'skip',
                '--frame-stride', '1', '--min-depth', '.25', '--max-depth', '4', '--voxel', '.05', '--min-confidence', '50'])
        execute('render', name, ['src/render_session_report.py', directory, '--reference', reference,
                                '--title', 'Student Union / ' + name])
        execute('render', name, ['src/view_session.py', name, '--run', root, '--config-only'])
    print('Completed requested independent stages. No cross-session registration, TF or merge was performed.', flush=True)


if __name__ == '__main__':
    main()
