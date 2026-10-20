#!/usr/bin/env python3
"""Floor2-only frame-level PGO, with an optional local RGB-D measurement ablation.

All selected original frame poses are variables (one gauge per component), not
anchor interpolations. Canonical loop roles and covariances are lifted back to
their original frames. Native sequential odometry is retained with explicit
boundary/quality uncertainty. New local factors never replace a heldout and
cannot connect separate quality runs. This is NOT bundle adjustment: there are
no jointly optimized image landmarks, intrinsic parameters or depth scale.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import shutil
import sys

import numpy as np

from build_robust_submap_graph import adjoint, edge_covariance
from export_rtabmap_db import load_pose_csv
from optimize_robust_submaps import optimize_manifest, parse_manifest
from optimize_session import inverse, relative_error, sha256_file, trajectory, utc_now

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'outputs/student_union_20261017_submap_robust'


def node_key(node):
    return f'floor2:frame:{int(node):05d}'


def lift_anchor_edge(edge, anchors, initial):
    """Undo deterministic frame-to-anchor conversion, retaining uncertainty.

    Mroot = La Mframe inv(Lb), Croot = Ad(Lb) Cframe Ad(Lb)^T.
    Quality inflation already present in Croot is intentionally retained.
    """
    endpoints = edge['provenance']['frame_endpoints']
    sa, a, sb, b = endpoints
    if sa != 'floor2' or sb != 'floor2':
        raise ValueError('Cross-session factor cannot enter floor2 frame graph')
    aa, bb = anchors[edge['from']], anchors[edge['to']]
    la = inverse(initial[aa]) @ initial[a]
    lb = inverse(initial[bb]) @ initial[b]
    measurement = inverse(la) @ np.asarray(edge['measurement']) @ lb
    mapping = adjoint(inverse(lb))
    covariance = mapping @ np.asarray(edge['covariance']) @ mapping.T
    return {'id': edge['id'], 'from': node_key(a), 'to': node_key(b),
            'measurement': measurement.tolist(),
            'covariance': ((covariance+covariance.T)/2).tolist(),
            'type': 'loop', 'role': edge['role'], 'region_group': edge['region_group'],
            'provenance': {**edge['provenance'], 'lifted_from_anchor_factor': True}}


def build_frame_graph(canonical, initial, native, frames, submaps, remeasured=None):
    missing = sorted(set(frames)-set(initial))
    if set(initial)-set(frames) or any(frames[n]['quality_grade'] != 'C' for n in missing):
        raise ValueError('Usable initial pose and quality frame inventories differ')
    anchors = {s['id']: int(s['anchor_node_id']) for s in canonical['submaps']
               if s['session_id'] == 'floor2'}
    # Ensure the canonical graph and frame pose source are the same initialization.
    for s in canonical['submaps']:
        if s['session_id'] == 'floor2' and not np.allclose(
                np.asarray(s['pose']), initial[int(s['anchor_node_id'])], atol=1e-8, rtol=0):
            raise ValueError('Canonical anchor and selected frame initialization differ')
    runs = {s['submap_id']: s['run_id'] for s in submaps['submaps']}
    model = np.asarray(canonical['metadata']['odometry_model_sigma_by_session']['floor2'])
    if model.shape != (6,) or np.any(model <= 0):
        raise ValueError('Invalid canonical odometry model')
    boundaries = {n for n, r in frames.items() if str(r['hard_boundary']).lower() in ('true', '1')}
    nodes = [{'id': node_key(n), 'session_id': 'floor2', 'frame_id': n,
              'pose': initial[n].tolist(), 'quality': frames[n]['quality_grade'],
              'gauge': n == min(initial)} for n in sorted(initial)]
    edges = []
    for e in native['odometry']:
        a, b = int(e['from_id']), int(e['to_id'])
        if a not in initial or b not in initial:
            raise ValueError('Native odometry endpoint missing')
        m = np.asarray(e['measurement'])
        covariance, diagnostic = edge_covariance(e, True)
        mismatch = relative_error(m, initial[a], initial[b])
        covariance += np.diag(model**2 + (.5*np.abs(mismatch))**2)
        qa, qb = frames[a], frames[b]
        crosses = any(min(a,b) < n <= max(a,b) for n in boundaries)
        low_quality = qa['quality_grade'] == 'C' or qb['quality_grade'] == 'C'
        weak = crosses or low_quality
        if weak:
            covariance *= 25.
        edges.append({'id': f'frame_odometry:{a}:{b}', 'from': node_key(a), 'to': node_key(b),
                      'measurement': m.tolist(), 'covariance': covariance.tolist(),
                      'type': 'structural' if weak else 'odometry',
                      'role': 'heldout' if e.get('held_out') else 'train',
                      'region_group': f'frame_continuity:{a}:{b}',
                      'provenance': {'source': 'native_frame_odometry',
                          'frame_endpoints': ['floor2',a,'floor2',b],
                          'hard_boundary': crosses, 'low_quality_endpoint': low_quality,
                          'uncertainty_multiplier': 25. if weak else 1.,
                          'source_covariance': diagnostic, 'model_sigma': model.tolist()}})
    for edge in canonical['edges']:
        if edge['type'] == 'loop' and edge['from'] in anchors and edge['to'] in anchors:
            edges.append(lift_anchor_edge(edge, anchors, initial))
    reserved_pairs = {tuple(sorted([int(e['from_id']), int(e['to_id'])]))
                      for e in native['verified_loops'] if e.get('held_out')}
    reserved_nodes = {n for pair in reserved_pairs for center in pair
                      for n in range(center-10, center+11)}
    added = []
    if remeasured is not None:
        if remeasured.get('complete') is not True or remeasured.get('inputs_unchanged') is not True:
            raise ValueError('Remeasurements incomplete or original inputs changed')
        seen = set()
        for row in remeasured['records']:
            if row.get('accepted') is not True:
                continue
            a, b = int(row['a']), int(row['b'])
            key = tuple(sorted((a,b)))
            if key in seen or a == b:
                raise ValueError('Duplicate/self local factor')
            seen.add(key)
            if a not in initial or b not in initial or a in reserved_nodes or b in reserved_nodes:
                raise ValueError('Local factor enters reserved/missing frame')
            pa, pb = frames[a]['primary_submap_id'], frames[b]['primary_submap_id']
            if (not pa or not pb or runs[pa] != runs[pb]
                    or frames[a]['quality_grade'] not in ('A','B')
                    or frames[b]['quality_grade'] not in ('A','B')):
                raise ValueError('Local factor crosses run or low-quality frame')
            if any(min(a,b) < n <= max(a,b) for n in boundaries):
                raise ValueError('Local factor crosses explicit hard boundary')
            covariance = edge_covariance(row)
            edge_id = f'local_rgbd:{a}:{b}'
            edges.append({'id': edge_id, 'from': node_key(a), 'to': node_key(b),
                          'measurement': row['T_a_b'], 'covariance': covariance.tolist(),
                          'type': 'registration', 'role': 'train',
                          'region_group': f'local_rgbd:{runs[pa]}:{pa}',
                          'provenance': {'source': 'new_local_training_only_rgbd_registration',
                              'frame_endpoints': ['floor2',a,'floor2',b],
                              'module': runs[pa], 'correlation_control': 'average per source primary submap',
                              'not_statistically_independent_of_odometry': True}})
            added.append(edge_id)
    graph = {'schema_version': 1, 'submaps': nodes, 'edges': edges,
             'metadata': {'created_utc': utc_now(), 'session': 'floor2',
                 'node_kind': 'original_frame_not_submap_anchor',
                 'accepted': False, 'navigation': False,
                 'added_local_factor_ids': added,
                 'cross_session_edges': 0, 'absolute_height_constraints': 0,
                 'intrinsics_or_depth_scale_optimized': False,
                 'bundle_adjustment': False,
                 'quality_C_policy': 'retain poses with weak continuity, omit from dense cloud export',
                 'quality_frames_without_initial_pose_omitted': missing,
                 'loop_policy': 'exact canonical loop identities/roles lifted to original frames',
                 'local_holdout_exclusion': 'all original heldout endpoints +/-10 node IDs',
                 'boundary_covariance_multiplier': 25.,
                 'limitations': ['One sensor; local registrations and odometry are correlated.',
                     'Noise floors and boundary inflation are modeling choices, not calibrated covariance.',
                     'Only floor2 capture changes; it includes lower lobby and stairs.',
                     'No surveyed floor labels or absolute stair heights; no plane flattening.']}}
    parse_manifest(graph)  # Validate all transforms and positive covariance before solving.
    return graph


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--canonical',type=Path,default=RUN/'graph_input_v7/manifest.json')
    p.add_argument('--initial',type=Path,default=ROOT/'outputs/student_union_20261017_geometry_review/graph_final/floor2/initial.csv')
    p.add_argument('--native',type=Path,default=ROOT/'outputs/student_union_20261017_added_reviewed/floor2/validated_edges.json')
    p.add_argument('--quality',type=Path,default=ROOT/'outputs/student_union_20261017_submap_quality/floor2')
    p.add_argument('--remeasurements',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--max-evaluations',type=int,default=200)
    p.add_argument('--outer-iterations',type=int,default=8)
    a = p.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    source_paths = [Path(__file__),ROOT/'src/optimize_robust_submaps.py',
        ROOT/'src/build_robust_submap_graph.py',ROOT/'src/optimize_session.py',
        a.canonical,a.initial,a.native,a.quality/'frames.csv',a.quality/'submaps.json']
    if a.remeasurements: source_paths.append(a.remeasurements)
    hashes = {str(f.resolve()):sha256_file(f) for f in source_paths}
    frames = {int(r['node_id']): r for r in csv.DictReader((a.quality/'frames.csv').open())}
    initial = load_pose_csv(a.initial)
    canonical = json.loads(a.canonical.read_text())
    measured = json.loads(a.remeasurements.read_text()) if a.remeasurements else None
    if measured:
        # Remeasurements must come from this exact frozen pose and quality source.
        source_hashes = measured.get('input_hashes', measured.get('input_sha256', {}))
        for f in (a.initial,a.quality/'frames.csv',a.quality/'submaps.json'):
            if source_hashes.get(str(f.resolve())) != sha256_file(f):
                raise ValueError(f'Remeasurement provenance does not match {f}')
    graph = build_frame_graph(canonical,initial,json.loads(a.native.read_text()),frames,
                             json.loads((a.quality/'submaps.json').read_text()),measured)
    graph['metadata']['input_hashes'] = hashes
    a.output.mkdir(parents=True)
    (a.output/'manifest.json').write_text(json.dumps(graph,indent=2)+'\n')
    for f in source_paths[:4]: shutil.copyfile(f,a.output/('source_'+f.name))
    print(json.dumps({'stage':'solve','nodes':len(graph['submaps']),
        'edges':dict(Counter(e['type'] for e in graph['edges'])),
        'new_local':len(graph['metadata']['added_local_factor_ids'])}),flush=True)
    report = optimize_manifest(graph,methods=['baseline_huber'],
        max_evaluations=a.max_evaluations,outer_iterations=a.outer_iterations,
        include_leave_one_group=False,translation_bound=5.,rotation_bound_degrees=20.)
    v = report['variants']['baseline_huber']
    from optimize_robust_submaps import _as_transform
    poses = {n:_as_transform(v['poses'][node_key(n)],str(n)) for n in initial}
    stamps = {n:float(r['stamp']) for n,r in frames.items()}
    trajectory(a.output/'trajectory.csv',poses,stamps)
    usable = {n:t for n,t in poses.items() if frames[n]['quality_grade'] in ('A','B')}
    trajectory(a.output/'trajectory_AB.csv',usable,stamps)
    report.update(command=sys.argv,source_hashes=hashes,
        input_hashes_unchanged=all(sha256_file(Path(f))==h for f,h in hashes.items()),
        trajectory_sha256=sha256_file(a.output/'trajectory.csv'),
        trajectory_AB_sha256=sha256_file(a.output/'trajectory_AB.csv'),
        full_frame_count=len(poses),export_AB_count=len(usable),
        other_sessions_modified=False,default_rviz_replaced=False,
        graph_manifest_sha256=sha256_file(a.output/'manifest.json'))
    if not report['input_hashes_unchanged']: raise RuntimeError('Input changed during solve')
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'output':str(a.output),'accepted':False,'navigation':False,
        'frames':len(poses),'components':v['component_count'],
        'last_solver':v['solver_iterations'][-1]}),flush=True)


if __name__ == '__main__': main()
