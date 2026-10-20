#!/usr/bin/env python3
"""Review five separately optimized captures on identical observation support.

Stages are resumable and refuse to overwrite their published outputs. The
disjoint graph is an inventory of independent solves, not a merged map.
Historical patches are development diagnostics, not independent ground truth.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import copy
import csv
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from expand_submap_poses import main as expand_main
from export_rtabmap_db import load_pose_csv
from export_submap_candidate_clouds import cloud_bounds, render_projection
from optimize_robust_submaps import METHODS
from optimize_session import Clouds, geometry_metrics, inverse, sha256_file, trajectory, utc_now
from render_session_report import read_ply
from revalidate_loop_constraints import SPECS, delta_summary

ROOT = Path(__file__).resolve().parents[1]
QUALITY = ROOT / 'outputs/student_union_20261017_submap_quality'
SOURCE_GRAPH = ROOT / 'outputs/student_union_20261017_submap_robust/graph_input_v7/manifest.json'
INITIAL = ROOT / 'outputs/student_union_20261017_geometry_review/graph_final'
SPECIALIST = ROOT / 'outputs/student_union_20261017_floor2_revisit_v2/integration_nominal/trajectory_AB.csv'
LABELS = [*METHODS, 'reviewed_4dof', 'floor2_rgbd_ba']


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(path)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def quality_rows(session):
    with (QUALITY/session/'frames.csv').open() as stream:
        return list(csv.DictReader(stream))


def wanted_ids(session):
    return sorted(int(r['node_id']) for r in quality_rows(session)
                  if r['quality_grade'] in ('A', 'B') and r['primary_submap_id'])


def copy_filtered(source, destination, ids):
    source = Path(source)
    poses = load_pose_csv(source)
    if not set(ids) <= set(poses):
        raise ValueError(f'{source}: missing A/B poses {set(ids)-set(poses)}')
    with Path(source).open() as stream:
        stamps = {int(r['id']): float(r['stamp']) for r in csv.DictReader(stream)}
    if destination.exists():
        existing = load_pose_csv(destination)
        if sorted(existing) != ids or any(not np.array_equal(existing[n], poses[n])
                                         and not np.allclose(existing[n], poses[n], atol=1e-12, rtol=0)
                                         for n in ids):
            raise ValueError(f'existing intermediate trajectory differs: {destination}')
    else:
        trajectory(destination, {n: poses[n] for n in ids}, stamps)
    return {'trajectory_csv': str(destination.resolve()),
            'trajectory_sha256': sha256_file(destination),
            'source': str(source.resolve()), 'source_sha256': sha256_file(source),
            'output_frame_count': len(ids)}


def normalized_trajectory(source, reference, destination, ids):
    """Fix only the arbitrary session gauge at its first retained camera."""
    source = Path(source)
    pp, rr = load_pose_csv(source), load_pose_csv(reference)
    anchor = min(ids)
    gauge = rr[anchor] @ inverse(pp[anchor])
    with source.open() as stream:
        stamps = {int(r['id']): float(r['stamp']) for r in csv.DictReader(stream)}
    normalized = {n: gauge @ pp[n] for n in ids}
    if destination.exists():
        existing = load_pose_csv(destination)
        if sorted(existing) != ids or any(not np.allclose(existing[n],normalized[n],atol=1e-12,rtol=0) for n in ids):
            raise ValueError('Existing gauge-normalized trajectory differs')
    else:
        trajectory(destination, normalized, stamps)
    return {'source': str(source.resolve()), 'source_sha256': sha256_file(source),
            'display_gauge_anchor': anchor, 'left_rigid_gauge': gauge.tolist(),
            'trajectory': str(destination.resolve()), 'trajectory_sha256': sha256_file(destination),
            'internal_shape_changed_by_gauge': False}


def prepare(run):
    if (run/'expanded/manifest.json').exists():
        raise FileExistsError(run/'expanded/manifest.json')
    source = read(SOURCE_GRAPH)
    union = {'schema_version': 1, 'submaps': [], 'edges': [],
             'metadata': {'initial_graph': str(INITIAL), 'cross_session_edges': 0,
                          'status': 'DISJOINT_INDEPENDENT_SOLVES_INVENTORY',
                          'source_graph': str(SOURCE_GRAPH),
                          'source_graph_sha256': sha256_file(SOURCE_GRAPH),
                          'session_sources': {}}}
    expanded = {'inputs': {}, 'variants': {m: {'sessions': {}} for m in LABELS},
                'status': 'FIVE_INDEPENDENT_CAPTURES_NO_CROSS_SESSION_OPTIMIZATION',
                'accepted': False, 'navigation': False}
    for session in SPECS:
        isolated = run/'isolation'/session/'manifest.json'
        backend = run/'optimization'/session/'report.json'
        graph = read(isolated)
        if {n['session_id'] for n in graph['submaps']} != {session}:
            raise ValueError('independent session graph has foreign nodes')
        if any(e['type'] == 'cross_session' for e in graph['edges']):
            raise ValueError('independent graph has a cross-session factor')
        expansion_manifest = run/'expanded_sessions'/session/'manifest.json'
        if not expansion_manifest.exists():
            expand_main(['--quality-root', str(isolated.parent/'quality_view'),
                         '--graph-manifest', str(isolated), '--backend-report', str(backend),
                         '--initial-graph', str(INITIAL), '--output', str(expansion_manifest.parent)])
        session_expanded = read(expansion_manifest)
        if (session_expanded['inputs']['graph_manifest_sha256'] != sha256_file(isolated)
                or session_expanded['inputs']['backend_reports'][0]['sha256'] != sha256_file(backend)):
            raise ValueError('existing expansion provenance differs')
        ids = wanted_ids(session)
        for method in METHODS:
            record = session_expanded['variants'][method]['sessions'][session]
            if sorted(load_pose_csv(Path(record['trajectory_csv']))) != ids:
                raise ValueError('expanded pose support differs from frozen A/B support')
            expanded['variants'][method]['sessions'][session] = record
        reviewed = ROOT/'outputs'/SPECS[session][1]/session/'candidate_trajectory.csv'
        for label, path in [('reviewed_4dof', reviewed),
                            ('floor2_rgbd_ba', SPECIALIST if session == 'floor2'
                             else INITIAL/session/'initial.csv')]:
            record = copy_filtered(path, run/'expanded'/label/session/'trajectory.csv', ids)
            record['optimization_applied'] = label != 'floor2_rgbd_ba' or session == 'floor2'
            expanded['variants'][label]['sessions'][session] = record
        copy_filtered(INITIAL/session/'initial.csv', run/'baseline'/session/'trajectory.csv', ids)
        union['submaps'].extend(copy.deepcopy(graph['submaps']))
        union['edges'].extend(copy.deepcopy(graph['edges']))
        union['metadata']['session_sources'][session] = {
            'graph': str(isolated.resolve()), 'graph_sha256': sha256_file(isolated),
            'backend': str(backend.resolve()), 'backend_sha256': sha256_file(backend)}
    expected = {e['id'] for e in source['edges'] if e['type'] != 'cross_session'}
    if {e['id'] for e in union['edges']} != expected:
        raise ValueError('disjoint graph does not retain exactly all within-session edges')
    union_path = run/'disjoint_graph.json'
    write(union_path, union)
    expanded['inputs']['graph_manifest_sha256'] = sha256_file(union_path)
    write(run/'expanded/manifest.json', expanded)


def distribution(values):
    values = np.asarray(values, dtype=float)
    return {'count': len(values), 'median_m': float(np.median(values)) if len(values) else None,
            'p90_m': float(np.percentile(values, 90)) if len(values) else None,
            'maximum_m': float(np.max(values)) if len(values) else None}


def patch_stats(rows, label):
    return distribution([r['stage_plane_abs_median_m'][label] for r in rows])


def geometry_guard(before, after):
    if before.get('count', 0) == 0 or after.get('count', 0) != before.get('count', 0):
        return None
    return (after['clipped_rmse_median_m'] <= before['clipped_rmse_median_m'] + .005
            and after['overlap_median'] >= before['overlap_median'] - .03)


def choose_candidate(session, stats, geometry, solver):
    """Conservative development selection; empty validation cannot earn promotion."""
    baseline = stats.get('incumbent_preview')
    decisions = {}
    for label in LABELS:
        reasons = []
        g = geometry[label]
        if not solver[label]['last_inner_success'] or solver[label]['active_bounds']:
            reasons.append('numerical_termination_or_bounds')
        if solver[label].get('strict_outer_converged') is False:
            reasons.append('strict_outer_convergence_not_met')
        if geometry_guard(geometry['incumbent_preview']['adjacent'], g['adjacent']) is not True:
            reasons.append('adjacent_geometry_regressed_or_unavailable')
        if geometry['incumbent_preview']['heldout'].get('count', 0):
            if geometry_guard(geometry['incumbent_preview']['heldout'], g['heldout']) is not True:
                reasons.append('heldout_loop_geometry_regressed_or_missing')
        if not baseline or baseline['count'] < 3:
            reasons.append('insufficient_fixed_patch_validation')
        else:
            v = stats[label]
            if (v['median_m'] > baseline['median_m'] + .002
                    or v['p90_m'] > baseline['p90_m'] + .005
                    or v['maximum_m'] > baseline['maximum_m'] + .05):
                reasons.append('fixed_patch_distribution_regression')
            if v['severe_regression_count']:
                reasons.append('one_or_more_fixed_patches_worsen_over_5cm')
            if not (v['median_m'] < baseline['median_m'] - .005
                    or v['p90_m'] < baseline['p90_m'] - .005):
                reasons.append('no_material_5mm_distribution_improvement')
        decisions[label] = {'eligible_for_development_selection': not reasons,
                            'reasons': reasons}
    eligible = [m for m in LABELS if decisions[m]['eligible_for_development_selection']]
    chosen = min(eligible, key=lambda m: (stats[m]['p90_m'], stats[m]['maximum_m'],
                                        stats[m]['median_m'])) if eligible else 'incumbent_preview'
    return chosen, decisions


def review(run):
    evaluation = read(run/'evaluation/report.json')
    graph = read(run/'disjoint_graph.json')
    expanded = read(run/'expanded/manifest.json')
    quality = read(QUALITY/'manifest.json')
    result = {'created_utc': utc_now(), 'accepted': False, 'navigation': False,
              'status': 'PER_SESSION_DEVELOPMENT_SELECTION', 'sessions': {},
              'source_sha256': sha256_file(Path(__file__)),
              'evaluation_sha256': sha256_file(run/'evaluation/report.json'),
              'selection_policy': {'median_regression_tolerance_m': .002,
                  'p90_regression_tolerance_m': .005, 'maximum_regression_tolerance_m': .05,
                  'per_patch_severe_regression_m': .05, 'material_improvement_m': .005,
                  'adjacent_and_heldout_clipped_rmse_regression_m': .005,
                  'adjacent_and_heldout_overlap_regression': .03,
                  'minimum_fixed_patch_count': 3,
                  'note': 'Thresholds fixed before this review; repeated historical development observations are not independent ground truth.'}}
    all_geometry_rows = []
    for session in SPECS:
        q = quality['sessions'][session]
        db = Path(q['source_database'])
        db_before = sha256_file(db)
        if db_before != q['database_sha256_before']:
            raise ValueError('source DB changed since quality audit')
        ids = wanted_ids(session)
        paths = {'selected_initial': run/'baseline'/session/'trajectory.csv',
                 'incumbent_preview': ROOT/'outputs/student_union_20261017_all_preview/bundle'/f'{session}_trajectory.csv'}
        paths.update({m: Path(expanded['variants'][m]['sessions'][session]['trajectory_csv']) for m in LABELS})
        gauge_records = {}
        for m, path in list(paths.items()):
            destination = run/'normalized'/session/m/'trajectory.csv'
            gauge_records[m] = normalized_trajectory(path, run/'baseline'/session/'trajectory.csv', destination, ids)
            paths[m] = destination
        poses = {m: load_pose_csv(p) for m, p in paths.items()}
        if any(sorted(p) != ids for p in poses.values()):
            raise ValueError('candidate observation support differs')
        native = read(ROOT/'outputs'/SPECS[session][1]/session/'validated_edges.json')
        adjacent = [(e['from_id'], e['to_id']) for e in native['odometry']
                    if e['from_id'] in poses['selected_initial'] and e['to_id'] in poses['selected_initial']]
        indices = np.unique(np.rint(np.linspace(0, len(adjacent)-1, min(30, len(adjacent)))).astype(int))
        adjacent = [adjacent[i] for i in indices]
        held = [tuple(map(int, e['provenance']['frame_endpoints'][1::2]))
                for e in graph['edges'] if e['type'] == 'loop' and e['role'] == 'heldout'
                and e['provenance']['frame_endpoints'][0] == session]
        held = sorted(set(p for p in held if set(p) <= set(ids)))
        loops = [(e['from_id'], e['to_id']) for e in native['verified_loops']
                 if e['from_id'] in poses['selected_initial'] and e['to_id'] in poses['selected_initial']]
        geometry, correction, solver = {}, {}, {}
        backend = read(run/'optimization'/session/'report.json')
        with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as con:
            clouds = Clouds(con, None)
            for label, pp in poses.items():
                geometry[label] = {}
                for name, pairs in [('adjacent', adjacent), ('heldout', held), ('native_loops_diagnostic', loops)]:
                    summary, rows = geometry_metrics(pairs, pp, clouds)
                    geometry[label][name] = summary
                    all_geometry_rows.extend({'session': session, 'stage': label,
                                              'group': name, **r} for r in rows)
                shifts = [delta_summary(poses['selected_initial'][n], pp[n]) for n in ids]
                correction[label] = {'translation': distribution([v['translation_m'] for v in shifts]),
                                     'maximum_rotation_deg': max(v['rotation_deg'] for v in shifts),
                                     'z_span_m': float(np.ptp([pp[n][2, 3] for n in ids]))}
                if label in METHODS:
                    it = backend['variants'][label]['solver_iterations'][-1]
                    solver[label] = {'last_inner_success': it['success'], 'active_bounds': it['active_bounds'],
                                     'strict_outer_converged': it['maximum_parameter_step'] < 1e-7
                                     and it['maximum_weight_change'] < 1e-5,
                                     'last_iteration': it}
                elif label == 'reviewed_4dof':
                    old = read(ROOT/'outputs'/SPECS[session][1]/session/'optimization.json')
                    ss = old['solver']
                    solver[label] = {'last_inner_success': ss['success'], 'active_bounds': ss['active_bounds'],
                                     'strict_outer_converged': None, 'source_report': str(ROOT/'outputs'/SPECS[session][1]/session/'optimization.json')}
                else:
                    if label == 'floor2_rgbd_ba' and session == 'floor2':
                        prior = read(SPECIALIST.parent/'report.json')['variants']['baseline_huber']
                        final = prior['solver_iterations'][-1]
                        solver[label] = {'last_inner_success': final['success'],
                                         'active_bounds': final['active_bounds'],
                                         'strict_outer_converged': prior['strict_converged'],
                                         'last_iteration': final,
                                         'source_report': str(SPECIALIST.parent/'report.json')}
                    else:
                        solver[label] = {'last_inner_success': True, 'active_bounds': 0,
                                         'strict_outer_converged': None,
                                         'note': 'Reference baseline; no new solve represented by this label.'}
        common = [r for r in evaluation['comparisons'] if r['session'] == session
                  and all(r['stage_plane_abs_median_m'].get('previous_preview' if m == 'incumbent_preview' else m) is not None for m in paths)]
        common = copy.deepcopy(common)
        for r in common:
            r['stage_plane_abs_median_m']['incumbent_preview'] = r['stage_plane_abs_median_m']['previous_preview']
        stats = {m: patch_stats(common, m) for m in paths}
        for m in paths:
            stats[m]['severe_regression_count'] = sum(
                r['stage_plane_abs_median_m'][m] - r['stage_plane_abs_median_m']['incumbent_preview'] > .05 for r in common)
        chosen, decisions = choose_candidate(session, stats, geometry, solver)
        selected = run/'selected'/session/'trajectory.csv'
        copy_filtered(paths[chosen], selected, ids)
        attempted = [*METHODS, *(['floor2_rgbd_ba'] if session == 'floor2' else [])]
        best_attempt = min(attempted, key=lambda m: (stats[m]['severe_regression_count'], stats[m]['p90_m'],
                                                  stats[m]['maximum_m'])) if common else 'dcs'
        record = {'database': str(db), 'database_sha256_before': db_before,
                  'database_sha256_after': sha256_file(db), 'frame_count': q['frame_count'],
                  'quality_grade_counts': q['quality_grade_counts'], 'used_AB_frames': len(ids),
                  'submaps': q['submap_count'], 'native_loop_count': len(native['verified_loops']),
                  'retained_loop_roles': dict(Counter(e['role'] for e in graph['edges']
                      if e['type']=='loop' and e['provenance']['frame_endpoints'][0]==session)),
                  'common_fixed_patch_count': len(common), 'fixed_patch_statistics': stats,
                  'geometry': geometry, 'geometry_pairs': {'adjacent': adjacent, 'heldout': held},
                  'corrections': correction, 'solver': solver, 'decisions': decisions,
                  'selected_label': chosen, 'selected_trajectory': str(selected.resolve()),
                  'selected_trajectory_sha256': sha256_file(selected),
                  'best_attempt_for_visual_comparison': best_attempt,
                  'candidate_trajectories': {m: str(p.resolve()) for m,p in paths.items()},
                  'rigid_gauge_normalization': gauge_records,
                  'incumbent_origin': 'Prior all-session exploratory placement; inherited lobby/floor2 joint hypothesis. Comparison baseline, not a new independent solve.',
                  'navigation_approved': False}
        record['database_unchanged'] = record['database_sha256_after'] == db_before
        result['sessions'][session] = record
        print(json.dumps({'session': session, 'selected': chosen, 'fixed_patches': len(common),
                          'best_attempt': best_attempt}), flush=True)
    from evaluate_submap_candidates import write_csv
    write_csv(run/'geometry_validation.csv', all_geometry_rows)
    original = read(SOURCE_GRAPH)
    cross = [e for e in original['edges'] if e['type'] == 'cross_session']
    result['merge_consideration'] = {'combined_cloud_created': False,
        'next_step': 'Remeasure spatially separated cross-session RGB-D regions after per-session choices; jointly optimize only connections that survive heldout checks.',
        'existing_cross_session_edges': cross,
        'outside_to_bridge': 'no validated cross-session edge in canonical graph',
        'bridge_to_lobby': 'no validated cross-session edge in canonical graph',
        'lobby_to_floor2': '3 training edges, 1 heldout; repeated spatial region correlation remains',
        'lobby_to_substairs': 'one training edge; insufficient distributed support',
        'floor2_to_substairs': 'no validated cross-session edge in canonical graph',
        'all_five_connected_with_validated_evidence': False}
    write(run/'review.json', result)


def export_clouds(run):
    report = read(run/'review.json')
    env = os.environ.copy()
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    jobs = []
    for session, record in report['sessions'].items():
        for label, pose in [('selected', record['selected_trajectory']),
                            ('attempt', record['candidate_trajectories'][record['best_attempt_for_visual_comparison']])]:
            out = run/'clouds'/session/label
            if out.exists():
                raise FileExistsError(out)
            out.mkdir(parents=True)
            jobs.append((session, label, record, pose, out))
    def one(job):
        session, label, record, pose, out = job
        db = Path(record['database'])
        before = sha256_file(db)
        cmd = [sys.executable, str(ROOT/'src/export_rtabmap_db.py'), str(db), '--pose-csv', pose,
               '--missing-pose-policy', 'skip', '--frame-stride', '1', '--pixel-stride', '1',
               '--min-depth', '.25', '--max-depth', '4', '--min-confidence', '50', '--voxel', '.05',
               '--cloud', str(out/'cloud.ply'), '--trajectory', str(out/'trajectory.csv'),
               '--summary', str(out/'summary.json')]
        with (out/'export.log').open('x') as stream:
            subprocess.run(cmd, stdout=stream, stderr=subprocess.STDOUT, env=env, check=True)
        summary = read(out/'summary.json')
        actual = sorted(load_pose_csv(out/'trajectory.csv'))
        if actual != wanted_ids(session) or summary['decoded_depth_frames'] != len(actual):
            raise ValueError('dense export did not use every identical A/B frame')
        after = sha256_file(db)
        if before != after:
            raise ValueError('database changed')
        return {'session': session, 'label': label, 'command': cmd,
                'cloud': str((out/'cloud.ply').resolve()), 'cloud_sha256': sha256_file(out/'cloud.ply'),
                'pose_sha256': sha256_file(Path(pose)), 'summary': summary,
                'database_sha256': after, 'frame_ids': actual}
    with ThreadPoolExecutor(max_workers=2) as pool:
        records = list(pool.map(one, jobs))
    write(run/'clouds/manifest.json', {'created_utc': utc_now(), 'exports': records,
        'all_database_hashes_unchanged': True, 'identical_AB_support_per_session': True,
        'session_clouds_merged': False})


def render(run):
    review = read(run/'review.json')
    figures = run/'figures'
    figures.mkdir(exist_ok=False)
    for session, record in review['sessions'].items():
        clouds = [read_ply(run/'clouds'/session/label/'cloud.ply') for label in ('selected', 'attempt')]
        bounds = cloud_bounds(clouds)
        fig, axes = plt.subplots(2, 2, figsize=(13, 10))
        for col, (label, cloud) in enumerate(zip(('selected', 'attempt'), clouds)):
            stage = record['selected_label'] if label == 'selected' else record['best_attempt_for_visual_comparison']
            for row, (a,b) in enumerate((('x','y'),('x','z'))):
                render_projection(axes[row,col], cloud, a,b,bounds, title=f'{label}: {stage}', max_points=180000)
        fig.suptitle(f'{session}: {record["used_AB_frames"]} identical RGB-D frames; independent capture\nActual depth reprojection; common axes and height scale')
        fig.tight_layout(rect=(0,0,1,.94)); fig.savefig(figures/f'{session}_clouds.png', dpi=160); plt.close(fig)
        fig, axes = plt.subplots(1,2,figsize=(13,5))
        for label,path in record['candidate_trajectories'].items():
            poses=load_pose_csv(Path(path)); xyz=np.array([poses[n][:3,3] for n in sorted(poses)])
            axes[0].plot(xyz[:,0],xyz[:,1],label=label,lw=1,alpha=.7)
            axes[1].plot(sorted(poses),xyz[:,2],label=label,lw=1,alpha=.7)
        axes[0].set(xlabel='X (m)',ylabel='Y (m)'); axes[0].set_aspect('equal',adjustable='datalim')
        axes[1].set(xlabel='Frame ID',ylabel='Camera Z (m)'); axes[1].legend(fontsize=8)
        for ax in axes:ax.grid(alpha=.2)
        fig.suptitle(f'{session}: independent optimization portfolio'); fig.tight_layout()
        fig.savefig(figures/f'{session}_trajectories.png',dpi=160);plt.close(fig)
    fig, axes = plt.subplots(1,3,figsize=(16,5))
    sessions=['outside','lobby','floor2']
    for ax,session in zip(axes,sessions):
        stats=review['sessions'][session]['fixed_patch_statistics']
        stages=list(stats); x=np.arange(len(stages))
        for metric,marker in [('median_m','o'),('p90_m','s'),('maximum_m','^')]:
            ax.plot(x,[100*stats[m][metric] for m in stages],marker+'-',label=metric)
        ax.set(xticks=x,xticklabels=stages,ylabel='fixed patch separation (cm)',
               title=f'{session}: same {stats[stages[0]]["count"]} patches')
        ax.tick_params(axis='x',rotation=45); ax.grid(alpha=.2)
    axes[0].legend(fontsize=8);fig.suptitle('Independent session comparison: unchanged finite patches, conditional development evidence')
    fig.tight_layout();fig.savefig(figures/'fixed_patch_summary.png',dpi=180);plt.close(fig)
    write(figures/'manifest.json',{'created_utc':utc_now(),'source_sha256':sha256_file(Path(__file__)),
        'review_sha256':sha256_file(run/'review.json'),
        'cloud_manifest_sha256':sha256_file(run/'clouds/manifest.json'),
        'figures':{p.name:sha256_file(p) for p in sorted(figures.glob('*.png'))}})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--stage',choices=['prepare','review','export','render'],required=True)
    a=p.parse_args();run=a.run.resolve()
    snapshot=run/'source_snapshots'/f'{a.stage}_{sha256_file(Path(__file__))[:16]}.py'
    snapshot.parent.mkdir(exist_ok=True)
    if not snapshot.exists():shutil.copyfile(Path(__file__),snapshot)
    {'prepare':prepare,'review':review,'export':export_clouds,'render':render}[a.stage](run)


if __name__ == '__main__':
    main()
