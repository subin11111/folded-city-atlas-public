#!/usr/bin/env python3
"""Inventory every active factor; screen influence and ablate revisit groups.

This is a diagnostic, not a map repair. Source DBs/graphs/poses/RViz are never
replaced. Complete physical-region exclusions are matched to graph-continuation
controls. Linear factor-exclusion influence is a screening approximation, not
proof of measurement truth. A sole graph bridge cannot validate itself.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
import copy
import hashlib
import json
import multiprocessing
from pathlib import Path
import shutil
import sqlite3
import sys
import time

import numpy as np
from scipy.optimize._numdiff import approx_derivative
from scipy.sparse import eye
from scipy.sparse.linalg import splu

from continue_floor2_graph import continue_graph, encode_warm_start
from expand_submap_poses import expand_session
from export_rtabmap_db import load_pose_csv
from floor2_vectorized_solver import build_residual
from optimize_robust_submaps import (_as_transform, _edge_chi2, _robust_weights,
    connected_components, cycle_diagnostics, parse_manifest, select_gauges, transform_record)
from optimize_session import Clouds, geometric_score, inverse, sha256_file, utc_now
from evaluate_submap_candidates import fixed_patches
from evaluate_map_surfaces import pair_metrics
from revalidate_loop_constraints import SPECS

ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'outputs/student_union_20261017_independent_v1'
QUALITY=ROOT/'outputs/student_union_20261017_submap_quality'
CANONICAL=ROOT/'outputs/student_union_20261017_submap_robust'
STATE={}
CACHE={}
SAMPLES={}
QUALITY_DATA={}
PATCH_ROWS={}


def read(path):return json.loads(Path(path).read_text())


def dump(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():raise FileExistsError(path)
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')


def graph_bridges(node_ids,edges):
    """Tarjan with edge identities, so parallel measurements are handled."""
    sys.setrecursionlimit(max(sys.getrecursionlimit(),20000))
    adjacency={n:[] for n in node_ids}
    for e in edges:
        adjacency[e.source].append((e.target,e.id));adjacency[e.target].append((e.source,e.id))
    entry={};low={};bridges=set();clock=0
    def visit(node,parent_edge=None):
        nonlocal clock
        entry[node]=low[node]=clock;clock+=1
        for other,key in adjacency[node]:
            if key==parent_edge:continue
            if other not in entry:
                visit(other,key);low[node]=min(low[node],low[other])
                if low[other]>entry[node]:bridges.add(key)
            else:low[node]=min(low[node],entry[other])
    for node in node_ids:
        if node not in entry:visit(node)
    return bridges


def exclusion_step(a,residual,solve_hessian,base_step):
    """Woodbury factor deletion in the damped, fixed-weight linearization."""
    x=solve_hessian(a.T.toarray())
    k=np.eye(a.shape[0])-np.asarray(a@x)
    k=(k+k.T)/2
    eigen=np.linalg.eigvalsh(k)
    if eigen[0]<1e-9:return None,float(eigen[0])
    correction=x@np.linalg.solve(k,residual+np.asarray(a@base_step).ravel())
    return correction,float(eigen[0])


def source_specs():
    specs={}
    for session in SPECS:
        if session=='floor2':
            specs[session]=dict(graph=RUN/'structure_integration/manifest.json',
                report=RUN/'structure_integration/report.json',method='baseline_huber',mode='frames',sessions=[session])
        else:
            specs[session]=dict(graph=RUN/'isolation'/session/'manifest.json',
                report=RUN/'optimization'/session/'report.json',method='dcs',mode='submaps',sessions=[session])
    specs['canonical_joint']=dict(graph=CANONICAL/'graph_input_v7/manifest.json',
        report=CANONICAL/'optimization_dcs_v7_extended/report.json',method='dcs',mode='submaps',sessions=list(SPECS))
    return specs


def bind_sources():
    specs=source_specs();inputs=set()
    for name,spec in specs.items():
        graph=read(spec['graph']);report=read(spec['report'])
        canonical=hashlib.sha256(json.dumps(graph,sort_keys=True,separators=(',',':'),ensure_ascii=False).encode()).hexdigest()
        if spec['mode']=='frames':
            if report['graph_manifest_sha256']!=sha256_file(spec['graph']):raise ValueError('Frame graph binding differs')
        elif report['provenance']['canonical_manifest_sha256']!=canonical:raise ValueError('Submap graph binding differs')
        nodes,edges=parse_manifest(graph)
        warm=report['variants'][spec['method']]
        if {e.id for e in edges if e.used_for_fit}!=set(warm['fit_edge_ids']):raise ValueError('Training inventory differs')
        pp={n:_as_transform(p,n) for n,p in warm['poses'].items()}
        if set(pp)!={n.id for n in nodes}:raise ValueError('Pose/node ids differ')
        STATE[name]={**spec,'graph_data':graph,'nodes':nodes,'edges':edges,'warm':warm,'poses':pp}
        inputs.update([spec['graph'],spec['report']])
    return inputs


def frame_poses(domain,poses):
    state=STATE[domain]
    if state['mode']=='frames':
        return {'floor2':{int(n.rsplit(':',1)[1]):p for n,p in poses.items()}}
    graph=state['graph_data'];metadata=graph['metadata'];result={}
    for session in state['sessions']:
        rows,_,_=expand_session(session,QUALITY_DATA['sessions'][session],QUALITY,
            Path(metadata['initial_graph']),{n['id']:n for n in graph['submaps']},poses)
        result[session]={}
        for row in rows:
            t=np.eye(4);from scipy.spatial.transform import Rotation
            t[:3,:3]=Rotation.from_quat([row[k] for k in ['qx','qy','qz','qw']]).as_matrix()
            t[:3,3]=[row[k] for k in ['x','y','z']];result[session][int(row['id'])]=t
    return result


def prepare_observations():
    global QUALITY_DATA
    QUALITY_DATA=read(QUALITY/'manifest.json')
    historical=read(RUN/'evaluation/report.json')['comparisons']
    for session in SPECS:
        ids=set(int(row['node_id']) for row in __import__('csv').DictReader((QUALITY/session/'frames.csv').open())
                if row['quality_grade'] in ('A','B') and row['primary_submap_id'])
        PATCH_ROWS[session]=[r for r in historical if r['session']==session and {r['a'],r['b']}<=ids]
        raw=load_pose_csv(ROOT/'outputs'/SPECS[session][1]/session/'raw_main_trajectory.csv')
        endpoint_ids={n for r in PATCH_ROWS[session] for n in (r['a'],r['b'])}
        for state in STATE.values():
            for row in state['graph_data']['edges']:
                endpoints=row.get('provenance',{}).get('frame_endpoints')
                if endpoints and row['type'] in ('loop','cross_session','registration'):
                    for s,n in [(endpoints[0],endpoints[1]),(endpoints[2],endpoints[3])]:
                        if s==session:endpoint_ids.add(int(n))
        database=Path(QUALITY_DATA['sessions'][session]['source_database'])
        if sha256_file(database)!=QUALITY_DATA['sessions'][session]['database_sha256_before']:
            raise ValueError('Source DB changed')
        with sqlite3.connect(f'file:{database}?mode=ro',uri=True) as con:
            clouds=Clouds(con,None)
            patch_ids={n for r in PATCH_ROWS[session] for n in (r['a'],r['b'])}
            for node in sorted(endpoint_ids):
                if node in patch_ids:CACHE[session,node]=fixed_patches(clouds,raw,node)
                SAMPLES[session,node]=clouds.get(node)[1]


def statistics(values):
    return {'count':len(values),'median_m':float(np.median(values)) if values else None,
            'p90_m':float(np.percentile(values,90)) if values else None,
            'maximum_m':float(max(values)) if values else None}


def score(domain,poses):
    frames=frame_poses(domain,poses);patches=[];geometries=[]
    for session,pp in frames.items():
        for row in PATCH_ROWS[session]:
            a,b=row['a'],row['b'];kind=row['kind']
            if a not in pp or b not in pp:continue
            metric=pair_metrics(CACHE[session,a][kind,row['plane_a']],CACHE[session,b][kind,row['plane_b']],pp[a],pp[b])
            patches.append({'session':session,'a':a,'b':b,'kind':kind,'plane_a':row['plane_a'],'plane_b':row['plane_b'],
                'value_m':metric['symmetric_plane_abs_median_m'],'incumbent_m':row['stage_plane_abs_median_m']['previous_preview']})
    for row in STATE[domain]['graph_data']['edges']:
        if row['role']!='heldout':continue
        endpoint=row.get('provenance',{}).get('frame_endpoints')
        if not endpoint:continue
        sa,a,sb,b=endpoint
        if a not in frames.get(sa,{}) or b not in frames.get(sb,{}):continue
        t=inverse(frames[sa][a])@frames[sb][b]
        metric=geometric_score(SAMPLES[sa,a],SAMPLES[sb,b],t,.20)
        geometries.append({'id':row['id'],**metric})
    return {'fixed_patch_statistics':statistics([r['value_m'] for r in patches]),
            'fixed_patch_severe_vs_incumbent_count':sum(r['value_m']-r['incumbent_m']>.05 for r in patches),
            'fixed_patches':patches,'heldout_geometry':geometries,
            'heldout_clipped_rmse_median_m':float(np.median([r['clipped_rmse_m'] for r in geometries])) if geometries else None,
            'heldout_overlap_median':float(np.median([r['overlap'] for r in geometries])) if geometries else None}


def edge_inventory(name):
    state=STATE[name];train=[e for e in state['edges'] if e.used_for_fit]
    bridges=graph_bridges([n.id for n in state['nodes']],train)
    weights,_=_robust_weights(state['method'],train,state['poses'],
        **{k:state['warm']['parameters'][k] for k in ('huber_delta','dcs_phi','group_switch_prior')})
    source={r['id']:r for r in state['graph_data']['edges']};rows=[]
    for edge in state['edges']:
        error,chi2=_edge_chi2(edge,state['poses']);cov=np.linalg.inv(edge.information)
        sigma=np.sqrt(np.maximum(np.diag(cov),0));provenance=source[edge.id].get('provenance',{})
        row={'id':edge.id,'type':edge.edge_type,'role':edge.role,'region_group':edge.region_group,
            'graph_bridge':edge.id in bridges,'translation_residual_m':float(np.linalg.norm(error[:3])),
            'rotation_residual_deg':float(np.degrees(np.linalg.norm(error[3:]))),'chi2':chi2,
            'effective_information_weight':weights.get(edge.id),'translation_sigma_m':sigma[:3].tolist(),
            'rotation_sigma_deg':np.degrees(sigma[3:]).tolist(),'covariance_condition_mixed_units':float(np.linalg.cond(cov)),
            'frame_endpoints':provenance.get('frame_endpoints'),'source':provenance.get('source'),
            'uncalibrated_model_possible':True,'screening_flags':[]}
        if row['translation_residual_m']>.15:row['screening_flags'].append('physical_translation_residual_over_15cm')
        if row['rotation_residual_deg']>3:row['screening_flags'].append('physical_rotation_residual_over_3deg')
        if max(sigma[:3])>2:row['screening_flags'].append('translation_sigma_over_2m_in_graph_model')
        if edge.id in bridges:row['screening_flags'].append('sole_topological_support_not_self_validating')
        if edge.role=='heldout' and edge.id in weights:raise ValueError('Heldout entered weights')
        rows.append(row)
    return rows,bridges


def influence_screen(name,bridges):
    state=STATE[name];train=[e for e in state['edges'] if e.used_for_fit];nodes=state['nodes']
    components=connected_components([n.id for n in nodes],train)
    gauges=set(select_gauges(nodes,components).values());params=state['warm']['parameters']
    x=encode_warm_start(nodes,gauges,state['warm']['poses'],params['translation_correction_bound_m'],params['rotation_correction_bound_deg'])
    weights,_=_robust_weights(state['method'],train,state['poses'],**{k:params[k] for k in ('huber_delta','dcs_phi','group_switch_prior')})
    problem=build_residual(nodes,train,gauges,weights)
    residual=problem(x);jac=approx_derivative(problem,x,method='3-point',sparsity=problem.jac_sparsity).tocsr()
    h=(jac.T@jac).tocsc();damping=max(1e-10,float(np.median(h.diagonal()))*1e-10)
    factor=splu(h+eye(h.shape[0],format='csc')*damping)
    base=factor.solve(-np.asarray(jac.T@residual).ravel());rows=[]
    for i,edge in enumerate(train):
        row={'id':edge.id,'type':edge.edge_type,'graph_bridge':edge.id in bridges}
        if edge.id in bridges:
            row['status']='unobservable_after_removal';rows.append(row);continue
        correction,eig=exclusion_step(jac[i*6:(i+1)*6],residual[i*6:(i+1)*6],factor.solve,base)
        row['minimum_schur_eigenvalue']=eig
        if correction is None:row['status']='near_unobservable_linear_exclusion'
        else:
            delta=correction.reshape(-1,6)
            row.update(status='linear_influence_estimate',
                maximum_translation_change_m=float(np.max(np.linalg.norm(delta[:,:3],axis=1))),
                maximum_rotation_change_deg=float(np.degrees(np.max(np.linalg.norm(delta[:,3:],axis=1)))),
                norm_mixed_units=float(np.linalg.norm(correction)))
        rows.append(row)
    return {'method':'Damped fixed-weight Jacobian factor exclusion using Woodbury; not a nonlinear solve or map accuracy test',
            'damping':damping,'jacobian':'3-point finite differences, exact original residual/sparsity',
            'all_training_edges_screened':len(rows),'rows':rows}


def job_groups(name,bridges,influence):
    state=STATE[name];train=[e for e in state['edges'] if e.used_for_fit]
    groups=defaultdict(list)
    for edge in train:
        if edge.edge_type in ('loop','registration','cross_session'):groups[edge.region_group].append(edge.id)
        elif edge.edge_type=='structural' and state['mode']=='submaps':groups['continuity:'+edge.id].append(edge.id)
    jobs=[{'domain':name,'label':'control','remove_ids':[],'kind':'matched_control'}]
    skipped=[]
    before=len(connected_components([n.id for n in state['nodes']],train))
    for key,ids in sorted(groups.items()):
        remaining=[e for e in train if e.id not in ids]
        components=len(connected_components([n.id for n in state['nodes']],remaining))
        if components>before:
            skipped.append({'group':key,'ids':ids,'reason':'removal_disconnects_training_graph','component_count':components})
        else:jobs.append({'domain':name,'label':key,'remove_ids':ids,'kind':'physical_region_or_transition_exclusion'})
    # Every individual continuity edge was screened above. Nonlinear checks
    # focus on the largest predicted effects; all revisit regions are checked.
    for kind in ('odometry','structural'):
        selected=sorted([r for r in influence['rows'] if r['type']==kind and r.get('status')=='linear_influence_estimate'],
                        key=lambda r:r['maximum_translation_change_m'],reverse=True)[:3]
        for row in selected:
            if any(j['remove_ids']==[row['id']] for j in jobs):continue
            jobs.append({'domain':name,'label':'influence:'+row['id'],'remove_ids':[row['id']],
                         'kind':'top_linear_continuity_influence'})
    spans=[]
    if state['mode']=='frames':
        for edge in sorted([e for e in train if e.edge_type=='structural'],key=lambda e:int(e.source.rsplit(':',1)[1])):
            a,b=sorted([int(edge.source.rsplit(':',1)[1]),int(edge.target.rsplit(':',1)[1])])
            if spans and spans[-1]['end']==a:spans[-1]['end']=b;spans[-1]['ids'].append(edge.id)
            else:spans.append({'start':a,'end':b,'ids':[edge.id]})
        for span in spans:
            count=len(connected_components([n.id for n in state['nodes']],[e for e in train if e.id not in span['ids']]))
            span['components_after_removal']=count
            span['status']='unobservable_after_span_removal' if count>before else 'cycle_supported_span'
            if count==before:jobs.append({'domain':name,'label':f'weak_span:{span["start"]}:{span["end"]}',
                                         'remove_ids':span['ids'],'kind':'weak_span_exclusion'})
    return jobs,skipped,spans


def worker(job):
    state=STATE[job['domain']];removed=set(job['remove_ids'])
    edges=[e for e in state['edges'] if e.id not in removed]
    start=time.monotonic()
    poses,solver=continue_graph(state['nodes'],edges,state['warm'],outer_iterations=6,max_evaluations=150,
        linear_maxiter=3000,jacobian_scheme='2-point')
    result={**job,'elapsed_seconds':time.monotonic()-start,'score':score(job['domain'],poses),
            'solver':{k:solver[k] for k in ('strict_converged','solver_iterations','component_count','heldout_edge_count_in_objective','initial_weighted_cost','final_weighted_cost')},
            'poses':{k:transform_record(p) for k,p in poses.items()}}
    return result


def revalidation_inventory():
    data={}
    for session in SPECS:
        path=ROOT/'outputs/student_union_20261017_revalidated/intra_session'/session/'constraints.json'
        payload=read(path);records=payload['records']
        data[session]={'source':str(path),'sha256':sha256_file(path),'count':len(records),
            'passed':sum(bool(r.get('accepted')) for r in records),
            'records':[{'a':r['a'],'b':r['b'],'accepted':r.get('accepted'),
                        'held_out':r.get('held_out'),'reason':r.get('reason')} for r in records]}
    return data


def aggregate(output,inventory,job_results):
    controls={r['domain']:r for r in job_results if r['kind']=='matched_control'};rankings={}
    for domain in STATE:
        control=controls[domain]['score'];rank=[]
        for result in job_results:
            if result['domain']!=domain or result['kind']=='matched_control':continue
            current=result['score'];before=control['fixed_patch_statistics'];after=current['fixed_patch_statistics']
            p90_gain=before['p90_m']-after['p90_m'] if before['count'] else None
            rms_gain=(control['heldout_clipped_rmse_median_m']-current['heldout_clipped_rmse_median_m']
                      if control['heldout_clipped_rmse_median_m'] is not None else None)
            overlap_change=(current['heldout_overlap_median']-control['heldout_overlap_median']
                            if control['heldout_overlap_median'] is not None else None)
            heldout_regression=(rms_gain is not None and rms_gain<-.005) or (overlap_change is not None and overlap_change<-.03)
            rank.append({'label':result['label'],'kind':result['kind'],'removed_ids':result['remove_ids'],
                'p90_gain_m':p90_gain,'heldout_rmse_gain_m':rms_gain,'heldout_overlap_change':overlap_change,
                'heldout_geometry_regressed':heldout_regression,'after':after,
                'severe_before':control['fixed_patch_severe_vs_incumbent_count'],'severe_after':current['fixed_patch_severe_vs_incumbent_count'],
                'inner_success':result['solver']['solver_iterations'][-1]['success'],
                'active_bounds':result['solver']['solver_iterations'][-1]['active_bounds'],
                'strict_converged':result['solver']['strict_converged'],
                'record':result['record_path']})
        rank.sort(key=lambda r:(r['p90_gain_m'] is None,-(r['p90_gain_m'] or 0),-(r['heldout_rmse_gain_m'] or 0)))
        rankings[domain]=rank
    summary={'created_utc':utc_now(),'accepted':False,'navigation':False,'status':'CONSTRAINT_CONFLICT_DIAGNOSTIC_ONLY',
             'inventory':str((output/'inventory.json').resolve()),'domains':{},'rankings':rankings,
             'nonlinear_job_count':len(job_results),'default_map_replaced':False,
             'limitations':['Repeated historical/development observations; no surveyed truth.',
                'A helpful exclusion is evidence of incompatible influence, not proof of wrong physical place identity.',
                'Graph bridges and disconnected exclusions remain unverifiable from the same graph.',
                'All training factors get a linear influence screen; nonlinear exclusions cover every connected revisit group and strongest continuity predictions.',
                'Budgeted nonlinear fits can fail strict outer convergence; solver status is reported per exclusion.',
                'No measured constraint is deleted from a source graph by this audit.']}
    for name,inv in inventory['domains'].items():
        summary['domains'][name]={k:inv[k] for k in ('node_count','edge_count','training_edge_count','heldout_edge_count','training_component_count','graph_bridge_count','edge_type_counts','scheduled_nonlinear_jobs','disconnected_group_exclusions','weak_span_counts')}
        summary['domains'][name]['control_score']=controls[name]['score']
    dump(output/'report.json',summary)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=3)
    args=p.parse_args();output=args.output.resolve()
    if output.exists():raise FileExistsError(output)
    inputs=bind_sources();prepare_observations()
    inputs.update([QUALITY/'manifest.json',Path(__file__),ROOT/'src/continue_floor2_graph.py',
                   ROOT/'src/optimize_robust_submaps.py',ROOT/'src/floor2_vectorized_solver.py'])
    for session,q in QUALITY_DATA['sessions'].items():
        inputs.update([Path(q['source_database']),Path(q['artifacts']['frames_csv']),Path(q['artifacts']['submaps_json'])])
        inputs.add(ROOT/'outputs'/SPECS[session][1]/session/'raw_main_trajectory.csv')
        inputs.add(ROOT/'outputs/student_union_20261017_geometry_review/graph_final'/session/'initial.csv')
        inputs.add(ROOT/'outputs/student_union_20261017_revalidated/intra_session'/session/'constraints.json')
    inputs.update([RUN/'evaluation/report.json',ROOT/'outputs/student_union_20261017_geometry_review/graph_final/used_constraints.json'])
    hashes={str(path.resolve()):sha256_file(path) for path in inputs}
    output.mkdir(parents=True);shutil.copyfile(Path(__file__),output/'source_snapshot.py')
    inventory={'created_utc':utc_now(),'input_hashes':hashes,'domains':{},'old_rgbd_revalidations':revalidation_inventory()}
    jobs=[]
    for name,state in STATE.items():
        edge_rows,bridges=edge_inventory(name);influence=influence_screen(name,bridges)
        scheduled,skipped,spans=job_groups(name,bridges,influence);jobs.extend(scheduled)
        inv={'source_graph':str(state['graph']),'source_report':str(state['report']),'method':state['method'],
            'mode':state['mode'],'sessions':state['sessions'],'node_count':len(state['nodes']),
            'edge_count':len(state['edges']),'training_edge_count':sum(e.used_for_fit for e in state['edges']),
            'heldout_edge_count':sum(not e.used_for_fit for e in state['edges']),
            'training_component_count':len(connected_components([n.id for n in state['nodes']],state['edges'])),
            'graph_bridge_count':len(bridges),'edge_type_counts':dict(Counter(e.edge_type+':'+e.role for e in state['edges'])),
            'scheduled_nonlinear_jobs':len(scheduled),'disconnected_group_exclusions':skipped,
            'weak_span_counts':dict(Counter(r['status'] for r in spans)),
            'weak_spans':spans,'edges':edge_rows,'influence_screen':influence}
        inventory['domains'][name]=inv
        dump(output/'domains'/name/'inventory.json',inv)
        print(json.dumps({'domain':name,'edges':len(edge_rows),'bridges':len(bridges),'jobs':len(scheduled),'disconnected_groups':len(skipped)}),flush=True)
    extra_path=ROOT/'outputs/student_union_20261017_geometry_review/graph_final/used_constraints.json'
    inventory['curated_extra_constraints']=[{k:r.get(k) for k in ('session_a','node_a','session_b','node_b','accepted','eligible_for_joint','eligible_for_experimental_graph','hypothesis_eligible','local_geometry_verified','joint_role','held_out','source_file')}
                                            for r in read(extra_path)]
    dump(output/'inventory.json',inventory);dump(output/'jobs.json',jobs)
    results=[]
    with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('fork')) as pool:
        futures={pool.submit(worker,job):job for job in jobs}
        for f in as_completed(futures):
            job=futures[f]
            try:result=f.result()
            except Exception as error:
                dump(output/'failures'/f'{job["domain"]}_{hashlib.sha256(job["label"].encode()).hexdigest()[:12]}.json',
                     {'job':job,'error':repr(error)})
                raise
            slug=hashlib.sha256(result['label'].encode()).hexdigest()[:16]
            path=output/'ablations'/result['domain']/(slug+'.json')
            result['record_path']=str(path.resolve());dump(path,result);results.append(result)
            s=result['score']['fixed_patch_statistics']
            print(json.dumps({'completed':len(results),'total':len(jobs),'domain':result['domain'],'label':result['label'],
                              'p90_m':s['p90_m'],'inner_success':result['solver']['solver_iterations'][-1]['success']}),flush=True)
    aggregate(output,inventory,results)
    unchanged=all(sha256_file(Path(p))==h for p,h in hashes.items())
    dump(output/'completion.json',{'complete':True,'input_hashes_unchanged':unchanged,'jobs_completed':len(results),
        'source_graphs_and_databases_replaced':False,'finished_utc':utc_now()})
    if not unchanged:raise RuntimeError('Audit inputs changed')


if __name__=='__main__':main()
