#!/usr/bin/env python3
"""Publish compact, source-hashed geometry evidence without copying raw data.

Patch statistics give each fixed finite patch one vote. Reserved graph pairs,
training pairs and repeated per-point measurements must not be conflated.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def patch_summary(rows):
    values = np.asarray([r['symmetric_plane_abs_median_m'] for r in rows], float)
    if not len(values):
        return {'patches': 0, 'frame_pairs': 0, 'median_m': None, 'p90_m': None, 'maximum_m': None}
    if not np.isfinite(values).all():
        raise ValueError('Non-finite patch metrics must not be silently discarded')
    return {
        'patches': len(rows),
        'frame_pairs': len({(r['session'], min(r['a'], r['b']), max(r['a'], r['b'])) for r in rows}),
        'median_m': float(np.median(values)),
        'p90_m': float(np.percentile(values, 90)),
        'maximum_m': float(values.max()),
    }


def build_summary(root):
    paths = {
        'graph': root/'graph_final/report.json',
        'postrun_review': root/'graph_final/postrun_guard_review.json',
        'evaluation': root/'evaluation/final/report.json',
        'clouds': root/'candidate_clouds/manifest.json',
        'connections': root/'connections/progress/progress_metrics.json',
        'surface': root/'surfaces/summary.json',
    }
    data = {key: json.loads(path.read_text()) for key, path in paths.items()}
    graph, evaluation, clouds = (data[k] for k in ('graph', 'evaluation', 'clouds'))
    if graph['navigation_ready'] or clouds['navigation_ready'] or evaluation['navigation_ready']:
        raise ValueError('This experiment summary is explicitly for unapproved candidates')
    stages = list(dict.fromkeys(r['stage'] for r in evaluation['patch_metrics']))
    aggregates = {}
    for stage in stages:
        rows = [r for r in evaluation['patch_metrics'] if r['stage'] == stage]
        aggregates[stage] = {
            'all_fixed_patches': patch_summary(rows),
            'reserved_graph_holdout': patch_summary([r for r in rows if r['graph_role'] == 'reserved_graph_holdout']),
            'by_session': {name: patch_summary([r for r in rows if r['session'] == name])
                           for name in sorted({r['session'] for r in rows})},
        }
    examples = {('outside', 69, 323), ('lobby', 595, 650), ('floor2', 548, 981),
                ('floor2', 503, 961), ('floor2', 656, 1055), ('floor2', 642, 1031)}
    return {
        'status': 'EXPERIMENT_EXECUTED_GLOBAL_MAP_NOT_APPROVED',
        'source_reports': {k: {'path': str(p), 'sha256': digest(p)} for k, p in paths.items()},
        'summary_source_sha256': digest(Path(__file__)),
        'source_graph_finished_utc': graph['finished_utc'],
        'navigation_ready': False,
        'canonical_preview_replaced': False,
        'graph_components': graph['components'],
        'all_sessions_connected': graph['all_sessions_connected'],
        'baseline_selection': {k: {'selected': v['selected_label'], 'selected_pose_csv': v['selected_pose_csv']}
                               for k, v in graph['baseline_selection'].items()},
        'connection_recovery': data['connections'],
        'experimental_surface_constraints': data['surface']['accepted_experimental_constraints'],
        'same_finite_plane_factors': graph['same_finite_plane_factors'],
        'local_normal_factors': graph['local_normal_factors'],
        'new_independent_imu_factors': graph['new_independent_imu_factors'],
        'variants': graph['variants'],
        'postrun_guard_review': data['postrun_review'],
        'fixed_patch_aggregates': aggregates,
        'fixed_patch_metric_definition': 'Distribution of per-patch symmetric absolute plane-distance medians, metres; not surveyed error.',
        'fixed_patch_examples': [r for r in evaluation['comparisons'] if (r['session'], r['a'], r['b']) in examples],
        'regressions_over_5cm_vs_initial': {k: len(v) for k, v in evaluation['severe_regressions_by_stage'].items()},
        'regressions_over_5cm_vs_previous': {k: len(v) for k, v in evaluation['severe_regressions_vs_previous_by_stage'].items()},
        'candidate_clouds': {
            'manifest': str(paths['clouds']), 'variant': clouds['variant'],
            'points': clouds['combined_points'], 'sha256': clouds['combined_sha256'],
            'per_session_points': {k: v['point_count'] for k, v in clouds['sessions'].items()},
            'missing_pose_ids_skipped': {k: v['export_summary']['skipped_missing_pose_ids'] for k, v in clouds['sessions'].items()},
        },
        'source_databases_sha256': graph['database_hashes_after'],
        'all_original_hash_checks_passed': graph['originals_unchanged'] and evaluation['original_databases_unchanged'] and clouds['originals_unchanged'],
        'unavailable_ground_truth': ['surveyed_door_width', 'surveyed_corridor_width', 'surveyed_floor_elevations', 'fresh_independent_validation_capture'],
        'limitations': evaluation['limitations'] + graph['limitations'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('outputs/student_union_20261017_geometry_review'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = build_summary(args.root)
    # Refuse to replace an existing publication; use a fresh path for revisions.
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'output': str(args.output), 'navigation_ready': False, 'originals_unchanged': result['all_original_hash_checks_passed']}))


if __name__ == '__main__':
    main()
