#!/usr/bin/env python3
"""Five independent preprocessed optimization portfolios; never auto-promote maps."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import copy
import csv
import json
import multiprocessing
from pathlib import Path
import shutil
import sqlite3
import sys

import numpy as np

from audit_constraint_conflicts import source_specs
from build_robust_submap_graph import transform_frame_edge
from continue_floor2_graph import continue_graph
from expand_submap_poses import expand_session
from export_rtabmap_db import load_pose_csv
from optimize_geometry_stage import optimize_four_dof, optimize_normals
from optimize_robust_submaps import (_as_transform, _robust_weights, connected_components,
    parse_manifest, transform_record)
from optimize_session import (Edge, apply, inverse, saved_app_poses, sha256_file, trajectory, utc_now)
from revalidate_loop_constraints import SPECS
from session_preprocessing import BASE, RGBD, local_refinement
from evaluate_map_surfaces import Settings, fit_planes

ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT/'outputs/student_union_20261017_independent_v1'
QUALITY=ROOT/'outputs/student_union_20261017_submap_quality'


def read(p):return json.loads(Path(p).read_text())


def dump(p,d):
    p.parent.mkdir(parents=True,exist_ok=True)
    if p.exists():raise FileExistsError(p)
    p.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n')


def normalize(pp,reference,ids):
    if not set(ids)<=set(pp):raise ValueError('A candidate misses common source frames')
    gauge=reference[min(ids)]@inverse(pp[min(ids)])
    return {n:gauge@pp[n] for n in ids},gauge


def expand(name,graph,poses,quality):
    rows,_,_=expand_session(name,quality['sessions'][name],QUALITY,
        Path(graph['metadata']['initial_graph']),{n['id']:n for n in graph['submaps']},poses)
    from scipy.spatial.transform import Rotation
    result={}
    for row in rows:
        t=np.eye(4);t[:3,:3]=Rotation.from_quat([row[k] for k in ('qx','qy','qz','qw')]).as_matrix()
        t[:3,3]=[row[k] for k in ('x','y','z')];result[int(row['id'])]=t
    return result


def fuse(rgbd,node,profile,poses,run_by_frame):
    arrays=[]
    for n in [node-1,node,node+1]:
        if n not in poses or run_by_frame.get(n)!=run_by_frame.get(node):continue
        points=rgbd.get(n,profile,cache=True)['train']
        arrays.append(apply(points,inverse(poses[node])@poses[n]))
    if not arrays:return np.empty((0,3))
    points=np.concatenate(arrays);vox=np.floor(points/.06).astype(np.int32)
    _,ids=np.unique(vox,axis=0,return_index=True)
    return points[ids]


def frame_factor_to_anchor(edge,anchor_nodes,initial):
    sa,a,sb,b=edge['provenance']['frame_endpoints']
    na=anchor_nodes.get(a);nb=anchor_nodes.get(b)
    if na is None or nb is None or na==nb:return None
    aa,bb=anchor_nodes['_anchors'][na],anchor_nodes['_anchors'][nb]
    la=inverse(initial[aa])@initial[a];lb=inverse(initial[bb])@initial[b]
    measurement,cov=transform_frame_edge(np.asarray(edge['measurement']),np.asarray(edge['covariance']),la,lb)
    return {**copy.deepcopy(edge),'from':na,'to':nb,'measurement':measurement.tolist(),'covariance':cov.tolist(),
        'provenance':{**edge['provenance'],'mapped_anchor_nodes':[aa,bb],
                       'portfolio_conversion':'Current frame measurement transferred to fixed historical submap anchors; covariance inflation retained.'}}


def session_job(job):
    name,output=job;output=Path(output)/name;output.mkdir(parents=True,exist_ok=False)
    quality=read(QUALITY/'manifest.json');q=quality['sessions'][name];database=Path(q['source_database'])
    config=read(ROOT/'config/student_union_session_preprocessing.json')['sessions'][name]
    dbhash=sha256_file(database)
    if dbhash!=q['database_sha256_before']:raise ValueError('Source DB differs')
    rows={int(r['node_id']):r for r in csv.DictReader((QUALITY/name/'frames.csv').open())}
    ids=sorted(n for n,r in rows.items() if r['quality_grade'] in ('A','B') and r['primary_submap_id'])
    submaps=read(QUALITY/name/'submaps.json')['submaps'];runs={r['submap_id']:r['run_id'] for r in submaps}
    frame_runs={n:runs.get(r['primary_submap_id']) for n,r in rows.items() if r['primary_submap_id']}
    oldreview=read(OLD/'review.json')['sessions'][name]
    reference=load_pose_csv(Path(oldreview['candidate_trajectories']['incumbent_preview']))
    graph_path=OLD/'isolation'/name/'manifest.json';report_path=OLD/'optimization'/name/'report.json'
    graph=read(graph_path);backend=read(report_path)
    anchors={n['id']:int(n['anchor_node_id']) for n in graph['submaps']}
    anchor_nodes={n:r['primary_submap_id'] for n,r in rows.items()};anchor_nodes['_anchors']=anchors
    initial=load_pose_csv(Path(graph['metadata']['initial_graph'])/name/'initial.csv')
    seed=copy.deepcopy(backend['variants']['dcs']);seed_poses={k:_as_transform(v,k) for k,v in seed['poses'].items()}
    source_inputs={graph_path,report_path,QUALITY/'manifest.json',QUALITY/name/'frames.csv',
        QUALITY/name/'submaps.json',ROOT/'config/student_union_session_preprocessing.json',Path(__file__),
        ROOT/'src/session_preprocessing.py',ROOT/'src/continue_floor2_graph.py',ROOT/'src/optimize_session.py',
        ROOT/'src/optimize_geometry_stage.py',ROOT/'src/expand_submap_poses.py'}
    if name=='floor2':
        latest_path=OLD/'structure_integration/manifest.json'
        excluded_path=ROOT/'outputs/student_union_20261017_constraint_audit_v1/ablations/floor2/252192abef272aed.json'
        latest=read(latest_path);excluded=read(excluded_path);source_inputs.update([latest_path,excluded_path])
        source_edges=[e for e in graph['edges'] if e['type'] in ('odometry','structural')]
        for edge in latest['edges']:
            if edge['type'] in ('odometry','structural') or edge['id']=='extra:floor2:644:floor2:1035:19':continue
            converted=frame_factor_to_anchor(edge,anchor_nodes,initial)
            if converted:source_edges.append(converted)
        graph['edges']=source_edges
        seed_poses={k:_as_transform(excluded['poses'][f'floor2:frame:{a:05d}'],k) for k,a in anchors.items()}
        graph['metadata']['audited_exclusion']='644:1035 excluded only in this new experimental portfolio; source preserved'
    seed['poses']={k:transform_record(v) for k,v in seed_poses.items()}
    frame_seed=expand(name,graph,seed_poses,quality)
    rgbd=RGBD(database)
    candidates=[]
    for edge in graph['edges']:
        if edge['role']!='train' or edge['type'] not in ('loop','registration'):continue
        if 'manual_structure' in edge['id'] or 'revisit_ba' in edge['id']:continue
        ep=edge.get('provenance',{}).get('frame_endpoints')
        if ep and ep[1] in frame_seed and ep[3] in frame_seed:
            candidates.append({'edge_id':edge['id'],'a':ep[1],'b':ep[3],'kind':'existing_revisit'})
    sequential=[e for e in graph['edges'] if e['type']=='odometry' and e['role']=='train'
                and anchors[e['from']] in frame_seed and anchors[e['to']] in frame_seed]
    if sequential:
        for index in np.unique(np.linspace(0,len(sequential)-1,min(12,len(sequential))).astype(int)):
            e=sequential[index];candidates.append({'edge_id':e['id'],'a':anchors[e['from']],'b':anchors[e['to']],'kind':'local_sequence'})
    trials=[]
    for profile in config['profiles']:
        results=[];counts=[]
        for candidate in candidates:
            a,b=candidate['a'],candidate['b']
            aa,bb=rgbd.get(a,BASE,cache=True),rgbd.get(b,BASE,cache=True)
            target=fuse(rgbd,a,profile,frame_seed,frame_runs);source=fuse(rgbd,b,profile,frame_seed,frame_runs)
            refined=local_refinement(target,source,aa['test'],bb['test'],inverse(frame_seed[a])@frame_seed[b])
            results.append({**candidate,**refined})
            for node in (a,b):
                sample=rgbd.get(node,profile,cache=True);counts.append(sample['retained_count']/max(1,sample['baseline_count']))
        comparable=[r for r in results if 'before' in r]
        rmse=np.mean([r['after' if r['passed'] else 'before']['clipped_rmse_m'] for r in comparable]) if comparable else 99.
        retention=float(np.mean(counts)) if counts else 0.
        trials.append({'profile':profile,'fixed_test_mean_clipped_rmse_m':float(rmse),'mean_retention':retention,
            'eligible_for_trial':retention>=.65,'refinements':results,
            'selection_scope':'Same baseline test voxels and pose-initialized local associations; development tuning, not ground truth.'})
    eligible=[t for t in trials if t['eligible_for_trial']]
    selected=min(eligible,key=lambda x:(x['fixed_test_mean_clipped_rmse_m'],-x['mean_retention'])) if eligible else trials[0]
    profile=selected['profile'];archived=[];changes=[]
    for refinement in selected['refinements']:
        if not refinement['passed']:continue
        old=next(e for e in graph['edges'] if e['id']==refinement['edge_id'])
        aa,bb=refinement['a'],refinement['b'];ep=[name,aa,name,bb]
        sigma_t=.20 if refinement['kind']=='local_sequence' else .15;sigma_r=6. if refinement['kind']=='local_sequence' else 5.
        frame_edge={'id':'preprocessed:'+old['id'],'from':'unused','to':'unused','type':'registration','role':'train',
            'measurement':refinement['measurement'],'covariance':np.diag([sigma_t**2]*3+[np.radians(sigma_r)**2]*3).tolist(),
            'region_group':old['region_group'] if refinement['kind']=='existing_revisit' else 'preprocessed_local:'+old['id'],
            'provenance':{'frame_endpoints':ep,'source':'Pose-initialized multiframe trimmed ICP of individually preprocessed RGB-D',
                'evidence_class':'correlated_local_development_refinement_not_independent_loop_discovery',
                'model_sigma_m_deg':[sigma_t,sigma_r],'sigma_note':'Explicit conservative isotropic trial model; not fitted sensor covariance.'}}
        converted=frame_factor_to_anchor(frame_edge,anchor_nodes,initial)
        if converted is None:continue
        if refinement['kind']=='existing_revisit':
            graph['edges'].remove(old);archived.append(old)
        # Native odometry is never deleted, even if a local depth fit disagrees.
        graph['edges'].append(converted);changes.append(converted['id'])
    graph['metadata'].update(preprocessing=profile,accepted=False,navigation=False,archived_replaced_factors=archived,
        new_refinements=changes,cross_session_optimization=False)
    graph_file=output/'graph.json';dump(graph_file,graph);nodes,edges=parse_manifest(graph)
    stages=[]
    def save_stage(key,label,poses,source,solver=None,fresh=False):
        normalized,gauge=normalize(poses,reference,ids);path=output/'poses'/(key+'.csv')
        trajectory(path,normalized,{n:float(rows[n]['stamp']) for n in ids})
        stages.append({'id':key,'label':label,'trajectory':str(path.resolve()),'trajectory_sha256':sha256_file(path),
            'source':source,'newly_computed':fresh,'solver':solver,'display_gauge':gauge.tolist(),
            'preprocessing':'base' if key=='O00' else profile['name']})
    app=saved_app_poses(rgbd.connection)
    stored_app=ROOT/'outputs'/SPECS[name][1]/'audit'/name/'app_optimized_trajectory.csv'
    stored=load_pose_csv(stored_app)
    if not set(ids)<=set(app):raise ValueError('Phone saved solution lacks requested common A/B support')
    if any(not np.allclose(app[n],stored[n],atol=2e-6,rtol=0) for n in ids):raise ValueError('Archived app solution differs from Admin')
    source_inputs.add(stored_app)
    save_stage('O00','Phone app optimized / original',app,str(stored_app))
    save_stage('P00','Preprocessing ONLY / phone poses unchanged',app,'O00 exact poses; only depth pixel removal')
    save_stage('H00','Previous RViz incumbent',reference,oldreview['candidate_trajectories']['incumbent_preview'])
    for code,key,label in [('H01','reviewed_4dof','Previous 4DoF'),('H02','baseline_huber','Previous independent Huber'),
                           ('H03','dcs','Previous independent DCS'),('H04','group_switch','Previous independent group switch')]:
        path=Path(oldreview['candidate_trajectories'][key]);source_inputs.add(path)
        save_stage(code,label,load_pose_csv(path),str(path))
    for code,method in [('N01','baseline_huber'),('N02','dcs'),('N03','group_switch')]:
        warm=copy.deepcopy(seed);warm['method']=method
        print(json.dumps({'session':name,'starting':code,'profile':profile['name'],'new_factors':len(changes)}),flush=True)
        poses,solver=continue_graph(nodes,edges,warm,outer_iterations=12,max_evaluations=200,linear_maxiter=5000,jacobian_scheme='3-point')
        dump(output/'solves'/(code+'.json'),{'graph_sha256':sha256_file(graph_file),'variant':solver})
        save_stage(code,'NEW '+{'baseline_huber':'6DoF Huber','dcs':'6DoF DCS','group_switch':'6DoF group switch'}[method],
                   expand(name,graph,poses,quality),str(graph_file),
                   {'inner_success':solver['solver_iterations'][-1]['success'],'active_bounds':solver['solver_iterations'][-1]['active_bounds'],
                    'strict_outer_converged':solver['strict_converged']},True)
    # Reuse the actual previous 4DoF and 6DoF attitude/normal solver implementations.
    lookup={node.id:k for k,node in enumerate(nodes)};numeric={lookup[k]:v for k,v in seed_poses.items()}
    group_count={}
    for e in edges:
        if e.used_for_fit and not e.is_odometry:group_count[e.region_group]=group_count.get(e.region_group,0)+1
    legacy=[Edge(lookup[e.source],lookup[e.target],0 if e.is_odometry else 1,e.measurement,e.information,
                 e.information/(group_count[e.region_group] if not e.is_odometry and e.used_for_fit else 1),not e.used_for_fit) for e in edges]
    components=[[lookup[n] for n in c] for c in connected_components([n.id for n in nodes],edges)]
    normals=[]
    if name in ('lobby','floor2','bridge_12_sub'):
        settings=Settings(max_planes_per_kind=1)
        for index in np.unique(np.linspace(0,len(nodes)-1,min(30,len(nodes))).astype(int)):
            node=nodes[index];frame=anchors[node.id]
            if frame not in frame_seed:continue
            data=rgbd.get(frame,profile,cache=True);rotation=seed_poses[node.id][:3,:3]
            planes,_=fit_planes(data['train']@rotation.T,data['test']@rotation.T,'horizontal',settings,20261017+frame)
            good=[p for p in planes if -2.4<p['center'][2]<-.4]
            if good:normals.append({'global_id':lookup[node.id],'normal_local':(rotation.T@good[0]['normal']).tolist()})
    for code,label,normal_rows in [('N04','NEW legacy 4DoF',None),('N05','NEW SE3 + baseline attitude prior',[])]+(
            [('N06','NEW SE3 + attitude + local horizontal normals',normals)] if normals else []):
        print(json.dumps({'session':name,'starting':code,'normals':len(normal_rows or [])}),flush=True)
        if normal_rows is None:pp,solver=optimize_four_dof(numeric,legacy,components,max_evaluations=300)
        else:pp,solver=optimize_normals(numeric,legacy,components,normal_rows,max_evaluations=300)
        poses={key:pp[value] for key,value in lookup.items()}
        dump(output/'solves'/(code+'.json'),{'graph_sha256':sha256_file(graph_file),'solver':solver,'poses':{k:transform_record(v) for k,v in poses.items()}})
        save_stage(code,label,expand(name,graph,poses,quality),str(graph_file),solver,True)
    if name=='floor2':
        for code,label,path in [('F01','Previous multi-frame RGB-D BA + frame PGO',ROOT/'outputs/student_union_20261017_floor2_revisit_v2/integration_nominal/trajectory_AB.csv'),
                                ('F02','Previous weak manual structure + frame PGO',OLD/'structure_integration/trajectory_AB.csv')]:
            source_inputs.add(path);save_stage(code,label,load_pose_csv(path),str(path))
        excluded_path=ROOT/'outputs/student_union_20261017_constraint_audit_v1/ablations/floor2/252192abef272aed.json'
        pp={int(k.rsplit(':',1)[1]):_as_transform(v,k) for k,v in read(excluded_path)['poses'].items()}
        save_stage('F03','Previous conflict audit / 644 factor excluded',pp,str(excluded_path))
    # Surface/attitude variants previously executed on the complete joint graph are
    # shown as historical only; never relabel them independent or freshly solved.
    for code,key in [('J01','pgo4'),('J02','pgo6_attitude'),('J03','pgo6_normals'),('J04','pgo6_planes')]:
        path=ROOT/'outputs/student_union_20261017_geometry_review/graph_final'/name/(key+'.csv')
        if path.exists():
            source_inputs.add(path);save_stage(code,'HISTORICAL joint '+key,load_pose_csv(path),str(path))
    all_inputs={str(p.resolve()):sha256_file(p) for p in source_inputs}
    database_after=sha256_file(database);rgbd.connection.close()
    result={'session':name,'created_utc':utc_now(),'accepted':False,'navigation':False,'database':str(database),
        'database_sha256':dbhash,'database_unchanged':database_after==dbhash,'input_hashes':all_inputs,
        'frame_ids':ids,'frame_count':len(ids),'profile_trials':trials,'selected_profile':profile,'profile_reason':config['reason'],
        'new_refinement_count':len(changes),'replaced_original_factors':len(archived),'normal_factor_count':len(normals),
        'stages':stages,'main_three_stage_ids':['O00','P00','N02'],
        'main_third_is_illustrative_not_selected_winner':True,'default_rviz_changed':False,
        'limitations':['Historical poses are reused, not freshly re-executed; fresh methods are marked N.',
            'NEW methods operate on independently isolated submaps with fixed within-submap shape, not joint frame BA.',
            'Profiles chosen using correlated local development depth tests; not globally optimal preprocessing.',
            'Local depth refinement does not establish new repeated-place identity.',
            'Horizontal normals are conditional surface hypotheses, not measured gravity; stairs are never flattened.',
            'No automatic winner or cross-session merge; user chooses per scenario.']}
    dump(output/'report.json',result)
    if not result['database_unchanged']:raise ValueError('Database changed')
    return {'session':name,'report':str((output/'report.json').resolve()),'stage_count':len(stages),'new_solves':sum(s['newly_computed'] for s in stages)}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--workers',type=int,default=3);a=p.parse_args();out=a.output.resolve()
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True);shutil.copyfile(Path(__file__),out/'source_snapshot.py')
    results=[]
    with ProcessPoolExecutor(max_workers=a.workers,mp_context=multiprocessing.get_context('fork')) as pool:
        futures={pool.submit(session_job,(name,str(out))):name for name in SPECS}
        for future in as_completed(futures):
            result=future.result();results.append(result);print(json.dumps({'completed':result}),flush=True)
    dump(out/'manifest.json',{'created_utc':utc_now(),'sessions':results,'accepted':False,'navigation':False,
                            'default_rviz_changed':False,'source_sha256':sha256_file(Path(__file__))})


if __name__=='__main__':main()
