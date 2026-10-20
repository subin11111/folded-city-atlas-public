#!/usr/bin/env python3
"""Read-only source, coordinate, covariance, role and cycle audit.

This supplements the nonlinear conflict audit. Passing algebraic consistency
does not establish physical correctness. Native-chain disagreement can be
caused by odometry drift, a wrong revisit measurement, or both.
"""
from collections import Counter, defaultdict
from dataclasses import replace
import argparse
import csv
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation

from audit_constraint_conflicts import source_specs, graph_bridges
from build_robust_submap_graph import (adjoint, edge_covariance,
    extra_constraint_role, transform_frame_edge)
from export_rtabmap_db import load_pose_csv
from floor2_vectorized_solver import build_residual
from optimize_robust_submaps import (_as_transform, _edge_chi2, _odometry_path,
    _robust_weights, connected_components, parse_manifest, select_gauges)
from continue_floor2_graph import encode_warm_start
from optimize_session import inverse, sha256_file, utc_now
from revalidate_loop_constraints import SPECS

ROOT=Path(__file__).resolve().parents[1]


def read(p): return json.loads(Path(p).read_text())


def pair_key(endpoint):
    sa,a,sb,b=endpoint
    return tuple(sorted(((sa,int(a)),(sb,int(b)))))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    canonical_path=ROOT/'outputs/student_union_20261017_submap_robust/graph_input_v7/manifest.json'
    quality_path=ROOT/'outputs/student_union_20261017_submap_quality/manifest.json'
    extra_path=ROOT/'outputs/student_union_20261017_geometry_review/graph_final/used_constraints.json'
    canonical=read(canonical_path);quality=read(quality_path);extra=read(extra_path)
    inputs={canonical_path,quality_path,extra_path,Path(__file__)}
    local={};frames={};native={};sources={};extra_roles=[];errors=[]
    for session,q in quality['sessions'].items():
        pose_path=Path(q['frozen_pose_csv']);frame_path=Path(q['artifacts']['frames_csv'])
        native_path=ROOT/'outputs'/SPECS[session][1]/session/'validated_edges.json'
        inputs.update([pose_path,frame_path,native_path])
        local[session]=load_pose_csv(pose_path)
        frames[session]={int(r['node_id']):r for r in csv.DictReader(frame_path.open())}
        native[session]=read(native_path)
        for row in native[session]['verified_loops']:
            key=f'{session}:native_loop:{row["from_id"]}:{row["to_id"]}'
            sources[key]={'T_a_b':row['measurement'],'effective_information':row['effective_information'],
                          'held_out':row.get('held_out',False)}
    for index,row in enumerate(extra):
        key=str(row.get('edge_id') or f'extra:{row["session_a"]}:{row["node_a"]}:{row["session_b"]}:{row["node_b"]}:{index}')
        role=extra_constraint_role(row);sources[key]=row
        extra_roles.append({'id':key,'role':role,'eligible_for_joint':row.get('eligible_for_joint'),
            'experimental_or_hypothesis_eligible':bool(row.get('eligible_for_experimental_graph') or row.get('hypothesis_eligible')),
            'local_geometry_verified':row.get('local_geometry_verified'),'source':row.get('source_file')})
    source_checks=[]
    for edge in canonical['edges']:
        if edge['type'] not in ('loop','cross_session','registration'): continue
        p=edge['provenance'];sa,a,sb,b=p['frame_endpoints'];aa,bb=p['mapped_anchor_nodes']
        la=inverse(local[sa][aa])@local[sa][a];lb=inverse(local[sb][bb])@local[sb][b]
        original=sources[edge['id']];cov,diagnostic=edge_covariance(original,True)
        measurement,mapped_cov=transform_frame_edge(np.asarray(original['T_a_b']),cov,la,lb)
        inflation=max(.1,min(p['endpoint_quality_weights']))
        expected_cov=mapped_cov/inflation
        recovered=inverse(la)@np.asarray(edge['measurement'])@lb
        mapping=adjoint(inverse(lb))
        recovered_cov=mapping@np.asarray(edge['covariance'])@mapping.T*inflation
        expected_role=extra_constraint_role(original) if 'session_a' in original else ('heldout' if original['held_out'] else 'train')
        row={'id':edge['id'],'measurement_max_abs_error':float(np.max(np.abs(measurement-edge['measurement']))),
             'covariance_max_abs_error':float(np.max(np.abs(expected_cov-edge['covariance']))),
             'roundtrip_measurement_max_abs_error':float(np.max(np.abs(recovered-original['T_a_b']))),
             'roundtrip_covariance_relative_error':float(np.linalg.norm(recovered_cov-cov)/np.linalg.norm(cov)),
             'expected_role':expected_role,'actual_role':edge['role'],'covariance_source':diagnostic['source']}
        row['passed']=(row['measurement_max_abs_error']<1e-8 and row['covariance_max_abs_error']<1e-8
                       and row['roundtrip_measurement_max_abs_error']<1e-8
                       and row['roundtrip_covariance_relative_error']<1e-8 and expected_role==edge['role'])
        if not row['passed']:errors.append(row)
        source_checks.append(row)
    domains={}
    for name,spec in source_specs().items():
        inputs.update([spec['graph'],spec['report']]);graph=read(spec['graph']);report=read(spec['report'])
        nodes,edges=parse_manifest(graph);train=[e for e in edges if e.used_for_fit]
        warm=report['variants'][spec['method']];poses={k:_as_transform(v,k) for k,v in warm['poses'].items()}
        params=warm['parameters'];weights,_=_robust_weights(spec['method'],train,poses,
            **{k:params[k] for k in ('huber_delta','dcs_phi','group_switch_prior')})
        gauges=set(select_gauges(nodes,connected_components([n.id for n in nodes],train)).values())
        x=encode_warm_start(nodes,gauges,warm['poses'],params['translation_correction_bound_m'],params['rotation_correction_bound_deg'])
        residual=build_residual(nodes,train,gauges,weights)(x).reshape(-1,6)
        predicted=np.array([_edge_chi2(e,poses)[1]*weights[e.id] for e in train])
        whitening_error=float(np.max(np.abs(np.sum(residual**2,axis=1)-predicted)))
        continuity=[e for e in train if e.edge_type in ('odometry','structural')]
        cycles=[];endpoint_groups=defaultdict(list);source_rows={r['id']:r for r in graph['edges']}
        validation=[]
        for edge in edges:
            endpoint=source_rows[edge.id].get('provenance',{}).get('frame_endpoints')
            if endpoint: endpoint_groups[pair_key(endpoint)].append({'id':edge.id,'role':edge.role,'type':edge.edge_type})
            if edge.edge_type not in ('loop','registration','cross_session'):continue
            path,ids=_odometry_path(edge,continuity)
            row={'id':edge.id,'role':edge.role,'native_continuity_path_available':path is not None,
                 'path_edge_ids':ids,'path_includes_weak_boundary':any(e.id in ids and e.edge_type=='structural' for e in continuity)}
            if path is not None:
                error=inverse(edge.measurement)@path
                row.update(translation_m=float(np.linalg.norm(error[:3,3])),
                           rotation_deg=float(np.degrees(Rotation.from_matrix(error[:3,:3]).magnitude())))
            cycles.append(row)
            if endpoint and endpoint[0]==endpoint[2] and ':native_loop:' in edge.id:
                session=endpoint[0];revalidated_path=ROOT/'outputs/student_union_20261017_revalidated/intra_session'/session/'constraints.json'
                inputs.add(revalidated_path)
                candidates=[r for r in read(revalidated_path)['records'] if {r['a'],r['b']}=={endpoint[1],endpoint[3]}]
                validation.append({'id':edge.id,'role':edge.role,'rgbd_revalidation_found':bool(candidates),
                    'rgbd_revalidation_passed':any(r.get('accepted') for r in candidates),
                    'reasons':[r.get('reason') for r in candidates]})
        collisions=[{'endpoints':list(key),'edges':rows} for key,rows in endpoint_groups.items() if len(rows)>1]
        heldout_overlap=[r for r in collisions if {e['role'] for e in r['edges']}=={'heldout','train'}]
        components=connected_components([n.id for n in nodes],train)
        domains[name]={'whitened_objective_max_abs_error':whitening_error,
            'information_matrices_clipped_count':sum(e.information_diagnostic['was_clipped'] for e in edges),
            'duplicate_endpoint_groups':collisions,'same_pair_train_heldout_overlap':heldout_overlap,
            'cycle_disagreements':cycles,'native_loop_revalidation_comparison':validation,
            'components':[[n for n in component] for component in components],
            'component_session_counts':[dict(Counter(next(n.session_id for n in nodes if n.id==key) for key in component)) for component in components],
            'factor_type_counts':dict(Counter(e.edge_type for e in edges))}
        if heldout_overlap:errors.append({'domain':name,'same_pair_train_heldout_overlap':heldout_overlap})
        if whitening_error>1e-6:errors.append({'domain':name,'whitening_error':whitening_error})
    hashes={str(p.resolve()):sha256_file(p) for p in inputs}
    result={'created_utc':utc_now(),'status':'ALGEBRA_AND_PROVENANCE_AUDIT_NOT_PHYSICAL_VALIDATION',
        'canonical_revisit_source_roundtrips':source_checks,'extra_role_contract':extra_roles,'domains':domains,
        'errors':errors,'input_hashes':hashes,'source_graph_or_database_modified':False,
        'limitations':['A failed feature revalidation does not prove different physical places.',
            'A native-chain cycle mismatch does not distinguish odometry drift from a wrong loop.',
            'Experimental eligibility is not final joint-map acceptance.',
            'No gravity/floor factor is present unless explicitly counted; local surface fitting compressed to SE3 is not a whole-floor prior.']}
    args.output.mkdir(parents=True);shutil.copyfile(Path(__file__),args.output/'source_snapshot.py')
    (args.output/'report.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'roundtrips':len(source_checks),'errors':len(errors),'domains':len(domains)}))
    if errors:raise RuntimeError('Audit found consistency errors; inspect report')


if __name__=='__main__':main()
