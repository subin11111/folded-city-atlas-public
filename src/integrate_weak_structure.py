#!/usr/bin/env python3
"""Trial a sensitivity-weighted structure factor in the floor2 capture only.

This explicitly weak development factor may be fitted despite failing the
strong-constraint gates. Every replaced measurement is archived; no duplicate
vote, cross-session edge, or automatic map promotion is introduced.
"""
import argparse
import copy
import csv
import json
from pathlib import Path
import shutil
import sys
import numpy as np

from continue_floor2_graph import continue_graph
from optimize_floor2_frames import node_key
from optimize_robust_submaps import parse_manifest
from optimize_session import sha256_file, trajectory, utc_now

ROOT=Path(__file__).resolve().parents[1]


def add_factor(graph,measurement):
    if not measurement.get('fit_executed') or not measurement['original_databases_unchanged']:
        raise ValueError('Missing structure fit or changed input DB')
    solver=measurement['solver']
    if not solver['success'] or solver['active_bounds'] or solver['pose_data_rank_after_landmark_elimination']<6:
        raise ValueError('Numerically invalid weak factor')
    before,after=measurement['withheld_cloud_before'],measurement['withheld_cloud_after']
    if after['clipped_rmse_m']>before['clipped_rmse_m']+.005 or after['overlap']<before['overlap']-.03:
        raise ValueError('Weak factor regresses withheld geometry')
    before,after=measurement['test_before'],measurement['test_after']
    if after['pixel_observations']<8 or after['pixel_median_px']>before['pixel_median_px'] or after['pixel_p90_px']>before['pixel_p90_px']:
        raise ValueError('Weak factor has no favorable withheld reprojection evidence')
    an=measurement['annotation']
    if an['session_a']!='floor2' or an['session_b']!='floor2':
        raise ValueError('Only within-floor2 constraints are allowed')
    a,b=node_key(an['node_a']),node_key(an['node_b']);pair={a,b}
    result=copy.deepcopy(graph)
    if not pair<={n['id'] for n in graph['submaps']}:raise ValueError('Missing endpoint')
    archived=[e for e in result['edges'] if {e['from'],e['to']}==pair and e['type']!='odometry']
    if any(e['role']=='heldout' for e in archived):raise ValueError('Cannot promote a heldout pair')
    result['edges']=[e for e in result['edges'] if e not in archived]
    sigma_t=max(.20,max(r['translation_m'] for r in measurement['click_sensitivity']))
    sigma_r=max(5.,max(r['rotation_deg'] for r in measurement['click_sensitivity']))
    covariance=np.diag([sigma_t**2]*3+[np.radians(sigma_r)**2]*3)
    edge={'id':'manual_structure_weak:548:981','from':a,'to':b,'type':'registration','role':'train',
          'measurement':measurement['T_a_b'],'covariance':covariance.tolist(),
          'region_group':'manual_structure:floor2:548_981',
          'provenance':{'frame_endpoints':['floor2',an['node_a'],'floor2',an['node_b']],
                        'source':'Manual RGB landmarks + measured finite wall/floor normals; fixed within-visit cameras',
                        'evidence_class':'weak_correlated_development_factor_not_independently_validated',
                        'sigma_translation_m':sigma_t,'sigma_rotation_deg':sigma_r,
                        'covariance_interpretation':'Conservative isotropic model floor enlarged to maximum click-perturbation displacement; not statistical covariance or calibrated confidence.',
                        'all_views_count_as_one_region':True,'archived_factor_ids':[e['id'] for e in archived]}}
    result['edges'].append(edge)
    result['metadata'].update(created_utc=utc_now(),weak_manual_structure_factor=edge,
                              archived_structure_replaced_factors=archived,accepted=False,navigation=False)
    parse_manifest(result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--measurement',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    original=json.loads((a.source/'manifest.json').read_text());prior=json.loads((a.source/'report.json').read_text())
    if prior['graph_manifest_sha256']!=sha256_file(a.source/'manifest.json'):raise ValueError('Warm graph binding differs')
    measured=json.loads(a.measurement.read_text());graph=add_factor(original,measured)
    quality=ROOT/'outputs/student_union_20261017_submap_quality/floor2/frames.csv'
    inputs=[a.source/'manifest.json',a.source/'report.json',a.measurement,quality,Path(__file__),
            ROOT/'src/continue_floor2_graph.py',ROOT/'src/floor2_vectorized_solver.py',ROOT/'src/optimize_robust_submaps.py']
    hashes={str(p.resolve()):sha256_file(p) for p in inputs}
    a.output.mkdir(parents=True)
    for path in inputs[4:]:shutil.copyfile(path,a.output/('source_'+path.name))
    (a.output/'manifest.json').write_text(json.dumps(graph,indent=2)+'\n')
    def checkpoint(row,poses,x):
        with (a.output/'iterations.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
        print(json.dumps(row),flush=True)
    nodes,edges=parse_manifest(graph)
    poses,variant=continue_graph(nodes,edges,prior['variants']['baseline_huber'],
        outer_iterations=20,max_evaluations=500,linear_maxiter=10000,jacobian_scheme='3-point',checkpoint=checkpoint)
    frames={int(r['node_id']):r for r in csv.DictReader(quality.open())}
    ids={n['id']:n['frame_id'] for n in graph['submaps']}
    integer={ids[k]:v for k,v in poses.items()};ab={n:v for n,v in integer.items() if frames[n]['quality_grade'] in ('A','B')}
    stamps={n:float(r['stamp']) for n,r in frames.items()}
    trajectory(a.output/'trajectory.csv',integer,stamps);trajectory(a.output/'trajectory_AB.csv',ab,stamps)
    unchanged=all(sha256_file(Path(p))==h for p,h in hashes.items())
    report={'complete':True,'accepted':False,'navigation':False,'created_utc':utc_now(),
            'input_hashes':hashes,'input_hashes_unchanged':unchanged,'command':sys.argv,
            'graph_manifest_sha256':sha256_file(a.output/'manifest.json'),
            'trajectory_sha256':sha256_file(a.output/'trajectory.csv'),
            'trajectory_AB_sha256':sha256_file(a.output/'trajectory_AB.csv'),
            'full_frame_count':len(integer),'export_AB_count':len(ab),
            'variants':{'baseline_huber':variant},'default_rviz_replaced':False,'other_sessions_modified':False}
    if not unchanged:raise ValueError('Inputs changed during solve')
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
