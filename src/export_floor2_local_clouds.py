#!/usr/bin/env python3
"""Audited dense exports of the three floor2-only experimental candidates.

Run --variant once per candidate; every candidate directory must be new.
After all candidates exist, --finalize verifies their immutable manifests and
writes a new collection manifest. There is no polling, GUI or RViz mutation.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUN = ROOT / 'outputs/student_union_20261017_floor2_local_v1'
QUALITY = ROOT / 'outputs/student_union_20261017_submap_quality'
VARIANTS = ('isolation', 'frame_baseline', 'frame_refined')
SETTINGS = {'frame_stride': 1, 'pixel_stride': 1, 'voxel_m': .05,
            'min_depth_m': .25, 'max_depth_m': 4., 'min_confidence': 50,
            'batch_frames': 20, 'missing_pose_policy': 'skip'}


def now():
    return datetime.now(timezone.utc).isoformat()


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''): h.update(block)
    return h.hexdigest()


def json_read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def json_write(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')


def pose_ids(path):
    with Path(path).open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    ids = [int(row['id']) for row in rows]
    if not ids or len(ids) != len(set(ids)):
        raise ValueError('Empty or duplicate pose inventory')
    return sorted(ids)


def check_support(actual, expected):
    if sorted(actual) != sorted(expected):
        raise ValueError(f'A/B pose support mismatch: missing={sorted(set(expected)-set(actual))}, '
                         f'extra={sorted(set(actual)-set(expected))}')


def source_record(run, variant):
    if variant == 'isolation':
        report = run / 'isolation_expanded/manifest.json'
        data = json_read(report)
        record = data['variants']['dcs']['sessions']['floor2']
        pose = Path(record['trajectory_csv'])
        expected = record['trajectory_sha256']
    else:
        report = run / variant / 'report.json'
        data = json_read(report)
        if data.get('input_hashes_unchanged') is not True:
            raise ValueError('Frame solve input hashes were not verified')
        pose = run / variant / 'trajectory_AB.csv'
        expected = data['trajectory_AB_sha256']
    if sha256(pose) != expected:
        raise ValueError('Pose source does not match its upstream report hash')
    return pose.resolve(), report.resolve()


def exportable_ids(database):
    """Exactly the row eligibility predicate used by export_rtabmap_db."""
    with sqlite3.connect(f'file:{database.resolve()}?mode=ro', uri=True) as con:
        return [int(row[0]) for row in con.execute('''
            SELECT n.id FROM Node n JOIN Data d ON d.id=n.id
            WHERE n.pose IS NOT NULL AND d.image IS NOT NULL
              AND d.depth IS NOT NULL AND d.calibration IS NOT NULL ORDER BY n.id
        ''')]


def exporter_command(exporter, database, pose, work):
    return [sys.executable, str(exporter), str(database), '--pose-csv', str(pose),
            '--missing-pose-policy', 'skip', '--frame-stride', '1', '--pixel-stride', '1',
            '--voxel', '0.05', '--min-depth', '0.25', '--max-depth', '4',
            '--min-confidence', '50', '--batch-frames', '20',
            '--cloud', str(work/'floor2.ply'), '--trajectory', str(work/'trajectory.csv'),
            '--summary', str(work/'export_summary.json')]


def verify_inventory(summary, wanted, database_ids, exported):
    selected = sorted(set(database_ids) & set(wanted))
    check_support(selected, wanted)
    check_support(exported, wanted)
    skipped = sorted(set(database_ids) - set(wanted))
    if summary['skipped_missing_pose_ids'] != skipped:
        raise ValueError('Exporter missing-pose inventory differs from read-only DB inventory')
    if (summary['selected_depth_frames'] != len(selected)
            or summary['decoded_depth_frames'] != len(selected)):
        # The old exporter counts decoded frames but not their individual IDs.
        # Equal counts prove all selected frames contributed. If they differ,
        # do not invent IDs or silently publish unequal observation support.
        raise ValueError('Not every selected frame contributed depth points')
    return {'database_exportable_frame_ids': database_ids,
            'input_pose_frame_ids': wanted, 'selected_depth_frame_ids': selected,
            'decoded_contributing_frame_ids': selected,
            'decoded_id_inference': 'All selected frames contributed, verified by exact selected/decoded count equality.',
            'skipped_missing_pose_frame_ids': skipped,
            'pose_ids_absent_from_exportable_database': [],
            'selected_depth_frames': len(selected), 'decoded_depth_frames': len(selected),
            'skipped_missing_pose_count': len(skipped)}


def ply_vertex_count(path):
    with Path(path).open('rb') as stream:
        count = None
        for _ in range(100):
            line = stream.readline().decode('ascii').strip()
            if line.startswith('element vertex '): count = int(line.split()[-1])
            if line == 'end_header':
                if count is None: raise ValueError('PLY has no vertex count')
                return count
    raise ValueError('Invalid PLY header')


def export_variant(run, output, variant):
    destination = output / variant
    if destination.exists() or (output/'manifest.json').exists():
        raise FileExistsError(f'refusing to overwrite finalized output: {destination}')
    pose, report = source_record(run, variant)  # Fail promptly if a solve is not ready.
    quality_path = QUALITY / 'manifest.json'
    quality = json_read(quality_path)['sessions']['floor2']
    frames_path = Path(quality['artifacts']['frames_csv'])
    with frames_path.open(newline='') as stream:
        quality_rows = list(csv.DictReader(stream))
    wanted = sorted(int(row['node_id']) for row in quality_rows
                    if row['quality_grade'] in ('A', 'B') and row['primary_submap_id'])
    if len(wanted) != 925:
        raise ValueError('This frozen experiment expects exactly925 A/B frames')
    check_support(pose_ids(pose), wanted)
    database = Path(quality['source_database'])
    database_before = sha256(database)
    if database_before != quality['database_sha256_before']:
        raise ValueError('Database differs from original quality-audit hash')
    inventory_ids = exportable_ids(database)
    exporter = ROOT/'src/export_rtabmap_db.py'
    sources = [Path(__file__).resolve(), exporter, pose, report, quality_path, frames_path]
    hashes = {str(path.resolve()): sha256(path) for path in sources}
    rviz = {str(path.resolve()): sha256(path) for path in sorted((ROOT/'config').rglob('*.rviz'))}
    output.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix=f'.{variant}.tmp-', dir=output))
    manifest = {'created_utc': now(), 'variant': variant, 'accepted': False, 'navigation': False,
                'status': 'EXPERIMENTAL_FLOOR2_CAPTURE_CLOUD_NOT_APPROVED_MAP',
                'database_open_mode': 'read-only', 'database_sha256_before': database_before,
                'source_database': str(database.resolve()), 'input_hashes_before': hashes,
                'rviz_hashes_before': rviz, 'default_rviz_replaced': False,
                'export_settings': SETTINGS, 'source_pose': str(pose),
                'upstream_report': str(report), 'command': None,
                'limitations': ['Entire floor2 capture includes lobby and stairs; this is not a semantic upper-floor mask.',
                                'Identical A/B observation support does not prove registration correctness.',
                                'No extra ICP, synthetic point insertion, plane flattening, or cross-session merge in export.']}
    try:
        for source in sources[:2]: shutil.copyfile(source, work/('source_'+source.name))
        command = exporter_command(exporter, database.resolve(), pose, work)
        manifest['command'] = command
        manifest['published_output_directory'] = str(destination.resolve())
        manifest['command_output_directory_before_atomic_publish'] = str(work.resolve())
        environment = os.environ.copy()
        environment.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='2', MKL_NUM_THREADS='1')
        manifest['thread_environment'] = {k: environment[k] for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}
        started = time.monotonic()
        with (work/'export.log').open('w') as stream:
            process = subprocess.run(command, cwd=ROOT, env=environment, stdout=stream,
                                     stderr=subprocess.STDOUT)
        manifest['elapsed_seconds'] = time.monotonic()-started
        manifest['exit_code'] = process.returncode
        if process.returncode: raise RuntimeError(f'Exporter failed; see {work}/export.log')
        summary = json_read(work/'export_summary.json')
        manifest['inventory'] = verify_inventory(summary, wanted, inventory_ids, pose_ids(work/'trajectory.csv'))
        manifest['point_count'] = ply_vertex_count(work/'floor2.ply')
        if manifest['point_count'] != summary['output_points']:
            raise ValueError('PLY header and summary point counts differ')
        manifest['export_summary'] = summary
        manifest['input_hashes_after'] = {str(path.resolve()): sha256(path) for path in sources}
        manifest['database_sha256_after'] = sha256(database)
        manifest['rviz_hashes_after'] = {path: sha256(Path(path)) for path in rviz}
        manifest['input_hashes_unchanged'] = hashes == manifest['input_hashes_after']
        manifest['database_unchanged'] = database_before == manifest['database_sha256_after']
        manifest['rviz_unchanged'] = rviz == manifest['rviz_hashes_after']
        if not all(manifest[k] for k in ('input_hashes_unchanged','database_unchanged','rviz_unchanged')):
            raise RuntimeError('An input or RViz configuration changed during export')
        manifest['output_hashes'] = {str(path.relative_to(work)): sha256(path)
                                     for path in sorted(work.iterdir()) if path.is_file()}
        manifest['finished_utc'] = now()
        json_write(work/'manifest.json', manifest)
        if destination.exists(): raise FileExistsError(destination)
        work.rename(destination)
        print(json.dumps({'variant': variant, 'output': str(destination),
                          'points': manifest['point_count'], 'frames': len(wanted),
                          'accepted': False}), flush=True)
        return manifest
    except Exception:
        manifest['failed_utc'] = now()
        json_write(work/'incomplete_manifest.json', manifest)
        raise


def finalize(output):
    manifest_path = output/'manifest.json'
    if manifest_path.exists(): raise FileExistsError(manifest_path)
    records, record_hashes = {}, {}
    for variant in VARIANTS:
        path = output/variant/'manifest.json'
        row = json_read(path)
        for name, digest in row['output_hashes'].items():
            if sha256(output/variant/name) != digest:
                raise ValueError(f'Export artifact changed: {variant}/{name}')
        if not all(row.get(k) is True for k in ('input_hashes_unchanged','database_unchanged','rviz_unchanged')):
            raise ValueError(f'Unverified export {variant}')
        if row['export_settings'] != SETTINGS: raise ValueError('Different export settings')
        records[variant] = row
        record_hashes[variant] = sha256(path)
    reference = records['isolation']['inventory']['input_pose_frame_ids']
    for row in records.values():
        check_support(row['inventory']['input_pose_frame_ids'], reference)
        check_support(row['inventory']['decoded_contributing_frame_ids'], reference)
    if len({row['database_sha256_before'] for row in records.values()}) != 1:
        raise ValueError('Variants used different source DBs')
    result = {'created_utc': now(), 'accepted': False, 'navigation': False,
              'default_rviz_replaced': False, 'status': 'THREE_MATCHED_SUPPORT_EXPERIMENTAL_CLOUDS',
              'export_settings': SETTINGS, 'shared_frame_count': len(reference),
              'shared_frame_ids': reference, 'all_input_database_and_rviz_hashes_unchanged': True,
              'database_sha256': records['isolation']['database_sha256_before'],
              'variants': {name: {'cloud': f'{name}/floor2.ply',
                                  'cloud_sha256': row['output_hashes']['floor2.ply'],
                                  'manifest': f'{name}/manifest.json',
                                  'manifest_sha256': record_hashes[name],
                                  'point_count': row['point_count']}
                           for name,row in records.items()}}
    json_write(manifest_path, result)
    print(json.dumps({'output': str(output), 'counts': {k:v['point_count'] for k,v in records.items()},
                      'accepted': False}), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, default=DEFAULT_RUN)
    parser.add_argument('--output', type=Path, default=DEFAULT_RUN/'clouds')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--variant', choices=VARIANTS)
    mode.add_argument('--finalize', action='store_true')
    args = parser.parse_args()
    if args.finalize: finalize(args.output)
    else: export_variant(args.run_root, args.output, args.variant)


if __name__ == '__main__': main()
