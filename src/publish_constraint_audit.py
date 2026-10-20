#!/usr/bin/env python3
"""Publish actual observations and matched constraint-exclusion diagnostics."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import subprocess

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from check_session_overlap import Session
from evaluate_submap_candidates import fixed_patches
from export_rtabmap_db import load_pose_csv
from optimize_robust_submaps import _as_transform, _edge_chi2, parse_manifest
from optimize_session import Clouds, apply, inverse, sha256_file, utc_now
from revalidate_loop_constraints import SPECS

ROOT=Path(__file__).resolve().parents[1]


def read(p):return json.loads(Path(p).read_text())


def write(p,d):
    if p.exists():raise FileExistsError(p)
    p.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n')


def history_inventory():
    paths=subprocess.check_output(['rg','--files','outputs','-g','manifest.json'],cwd=ROOT,text=True).splitlines()
    records=[];history=defaultdict(list)
    for name in sorted(paths):
        path=ROOT/name;payload=read(path)
        if not isinstance(payload,dict) or 'submaps' not in payload or 'edges' not in payload:continue
        if not isinstance(payload['edges'],list):continue
        rows=payload['edges'];nodes=payload['submaps']
        records.append({'path':name,'sha256':sha256_file(path),'nodes':len(nodes),'edges':len(rows),
            'roles':dict(Counter(r.get('role','unspecified') for r in rows)),
            'types':dict(Counter(r.get('type','unspecified') for r in rows)),
            'status':'historical inventory only; not a new nonlinear solve'})
        for row in rows:
            if row.get('type') in ('odometry','structural'):continue
            fingerprint=hashlib.sha256(json.dumps({k:row.get(k) for k in ('from','to','measurement','covariance','information','role')},sort_keys=True).encode()).hexdigest()
            history[row['id']].append({'graph':name,'role':row.get('role'),'fingerprint':fingerprint})
    return {'graphs':records,'revisit_factor_history':dict(history),
        'warning':'Superseded candidates, sensitivity graphs and isolated copies are not simultaneous constraints.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args();run=a.run.resolve();out=a.output.resolve()
    if out.exists():raise FileExistsError(out)
    completion=read(run/'completion.json')
    if not completion['complete'] or not completion['input_hashes_unchanged']:raise ValueError('Incomplete audit')
    report=read(run/'report.json');inventory=read(run/'inventory.json');provenance=read(run/'provenance/report.json')
    out.mkdir(parents=True);figures=[];input_hashes={}
    for path in [run/'completion.json',run/'report.json',run/'inventory.json',run/'provenance/report.json',Path(__file__)]:
        input_hashes[str(path.resolve())]=sha256_file(path)
    def save(fig,name,caption):
        fig.tight_layout(rect=(0,0,1,.965));path=out/(name+'.png');fig.savefig(path,dpi=160);plt.close(fig)
        with Image.open(path) as im:im.verify()
        figures.append({'file':path.name,'sha256':sha256_file(path),'synthetic':False,'caption':caption})
    jobs={};controls={};detailed={};factor_csv=[]
    for name,inv in inventory['domains'].items():
        jobs[name]=[read(p) for p in sorted((run/'ablations'/name).glob('*.json'))]
        control=next(r for r in jobs[name] if r['label']=='control');controls[name]=control
        graph=read(inv['source_graph']);nodes,edges=parse_manifest(graph);lookup={e.id:e for e in edges}
        cp={k:_as_transform(v,k) for k,v in control['poses'].items()};detailed[name]=[]
        for row in inv['edges']:factor_csv.append({'domain':name,**{k:row.get(k) for k in ('id','type','role','region_group','graph_bridge','translation_residual_m','rotation_residual_deg','chi2','effective_information_weight')},
                                                   'max_translation_sigma_m':max(row['translation_sigma_m'])})
        for case in jobs[name]:
            if case['label']=='control':continue
            pp={k:_as_transform(v,k) for k,v in case['poses'].items()};removed=[]
            for key in case['remove_ids']:
                error,chi2=_edge_chi2(lookup[key],pp)
                removed.append({'id':key,'translation_m':float(np.linalg.norm(error[:3])),
                    'rotation_deg':float(np.degrees(np.linalg.norm(error[3:]))),'chi2':chi2})
            before=control['score'];after=case['score'];patch_change=[]
            for old,new in zip(before['fixed_patches'],after['fixed_patches']):
                assert (old['session'],old['a'],old['b'],old['kind'],old['plane_a'],old['plane_b'])==(new['session'],new['a'],new['b'],new['kind'],new['plane_a'],new['plane_b'])
                patch_change.append({'session':new['session'],'a':new['a'],'b':new['b'],'kind':new['kind'],
                                     'change_m':new['value_m']-old['value_m']})
            old_h={r['id']:r for r in before['heldout_geometry']};held_changes=[]
            for row in after['heldout_geometry']:
                old=old_h[row['id']]
                held_changes.append({'id':row['id'],'clipped_rmse_change_m':row['clipped_rmse_m']-old['clipped_rmse_m'],
                                     'overlap_change':row['overlap']-old['overlap']})
            detailed[name].append({'label':case['label'],'removed_ids':case['remove_ids'],
                'excluded_measurement_residuals':removed,
                'maximum_node_translation_change_m':max(float(np.linalg.norm(pp[k][:3,3]-cp[k][:3,3])) for k in pp),
                'maximum_node_rotation_change_deg':max(float(np.degrees(Rotation.from_matrix(cp[k][:3,:3].T@pp[k][:3,:3]).magnitude())) for k in pp),
                'patch_changes':patch_change,'heldout_geometry_changes':held_changes,
                'source_result':case['record_path']})
    with (out/'all_factors.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(factor_csv[0]));writer.writeheader();writer.writerows(factor_csv)
    names=list(inventory['domains']);x=np.arange(len(names));fig,ax=plt.subplots(figsize=(11,5))
    for offset,key,label in [(-.28,'training_edge_count','Training factors'),(0,'heldout_edge_count','Reserved factors'),(.28,'graph_bridge_count','Sole topological supports')]:
        ax.bar(x+offset,[inventory['domains'][n][key] for n in names],width=.27,label=label)
    ax.set_xticks(x,names,rotation=15);ax.set(ylabel='Factor occurrences',title='Audit scope: five captures + historical canonical joint comparison')
    ax.legend();ax.grid(axis='y',alpha=.2)
    save(fig,'inventory','Canonical joint duplicates some isolated evidence; occurrences are not unique physical measurements.')
    fig,axes=plt.subplots(2,3,figsize=(17,9))
    for ax,name in zip(axes.ravel(),names):
        rows=inventory['domains'][name]['edges']
        for kind,color in [('odometry','#69798a'),('structural','#d7992e'),('loop','#3275b0'),('registration','#9b50b2'),('cross_session','#318860')]:
            selected=[r for r in rows if r['type']==kind and r['role']=='train']
            if selected:ax.scatter([r['translation_residual_m']*100 for r in selected],[max(1e-8,r['chi2']**.5) for r in selected],s=15,label=kind,alpha=.65)
        if name=='floor2':
            r=next(r for r in rows if r['id']=='extra:floor2:644:floor2:1035:19')
            ax.scatter([r['translation_residual_m']*100],[r['chi2']**.5],marker='*',s=170,color='#c33d3d',label='644 / 1035 ICP')
        ax.axhline(3,ls='--',color='#c33d3d',lw=1);ax.set_yscale('log')
        ax.set(title=name,xlabel='Physical translation residual (cm)',ylabel='Whitened 6D residual norm');ax.legend(fontsize=7);ax.grid(alpha=.2)
    fig.suptitle('A small normalized residual is not proof of a geometrically correct map')
    save(fig,'physical_vs_normalized','Residuals at original warm poses; line=norm 3, not a universally calibrated acceptance gate.')
    fig,axes=plt.subplots(1,3,figsize=(17,7))
    for ax,name in zip(axes,['outside','lobby','floor2']):
        ranked=report['rankings'][name];selected=ranked[:4]+ranked[-3:]
        labels=[r['label'].replace(name+'__'+name+':','').replace('continuity:transition:','').replace('influence:','') for r in selected]
        gains=[r['p90_gain_m']*100 for r in selected]
        ax.barh(np.arange(len(gains)),gains,color=['#428c76' if v>0 else '#c6615a' for v in gains]);ax.axvline(0,color='black',lw=.7)
        ax.set_yticks(np.arange(len(labels)),labels,fontsize=8);ax.invert_yaxis();ax.set(title=name,xlabel='P90 improvement after exclusion (cm)')
        ax.grid(axis='x',alpha=.2)
    fig.suptitle('Matched exclusion screens: improvement alone does NOT authorize deletion')
    save(fig,'exclusion_summary','Top four and bottom three P90 changes; full rankings include per-heldout geometry and solver status.')
    f_control=controls['floor2'];f_case=next(r for r in jobs['floor2'] if r['remove_ids']==['extra:floor2:644:floor2:1035:19'])
    bad=[i for i,r in enumerate(f_control['score']['fixed_patches']) if r['value_m']-r['incumbent_m']>.05]
    rows=[f_control['score']['fixed_patches'][i] for i in bad]
    fig,ax=plt.subplots(figsize=(11,5));x=np.arange(len(rows))
    ax.bar(x-.26,[r['incumbent_m']*100 for r in rows],width=.26,label='Incumbent RViz')
    ax.bar(x,[r['value_m']*100 for r in rows],width=.26,label='Unchanged continuation')
    ax.bar(x+.26,[f_case['score']['fixed_patches'][i]['value_m']*100 for i in bad],width=.26,label='Exclude 644 / 1035 only')
    ax.set_xticks(x,[f'{r["a"]}/{r["b"]}\n{r["kind"]}' for r in rows],fontsize=9)
    ax.set(ylabel='Finite-patch plane-distance (cm)',title='Floor2: the seven inherited regressions; same observations in every column');ax.legend();ax.grid(axis='y',alpha=.2)
    save(fig,'floor2_regression_causality','Repeated development patch diagnostics, not surveyed truth; excluded poses are not installed.')
    # Show actual fixed depth samples, never flattening a floor or deleting a wall.
    ss=Session(ROOT/'data/raw/rtabmap'/SPECS['floor2'][0],'floor2')
    raw_path=ROOT/'outputs'/SPECS['floor2'][1]/'floor2/raw_main_trajectory.csv';raw=load_pose_csv(raw_path)
    input_hashes[str(raw_path)]=sha256_file(raw_path)
    cloud=Clouds(ss.connection,None);cache={}
    for index in bad:
        row=f_control['score']['fixed_patches'][index];aa,bb,kind=row['a'],row['b'],row['kind']
        for node in (aa,bb):
            if node not in cache:cache[node]=fixed_patches(cloud,raw,node)
        fig,axes=plt.subplots(2,2,figsize=(11,8));samples=[]
        for col,case in enumerate([f_control,f_case]):
            for node,plane,color in [(aa,row['plane_a'],'#3b89be'),(bb,row['plane_b'],'#dd8545')]:
                points=cache[node][kind,plane]['testing']
                pose=_as_transform(case['poses'][f'floor2:frame:{node:05d}'],str(node));world=apply(points,pose);samples.append(world)
                for rr,(i,j) in enumerate([(0,1),(0,2)]):axes[rr,col].scatter(world[:,i],world[:,j],s=2,alpha=.5,color=color,label=f'frame {node}')
        joined=np.concatenate(samples)
        for rr,(i,j) in enumerate([(0,1),(0,2)]):
            lo=joined[:,[i,j]].min(0)-.12;hi=joined[:,[i,j]].max(0)+.12
            for cc in range(2):
                ax=axes[rr,cc];ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='X (m)',ylabel=['Y (m)','Z (m)'][rr]);ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.2)
        axes[0,0].set_title('Unchanged graph');axes[0,1].set_title('644 / 1035 excluded (diagnostic only)');axes[0,0].legend()
        fig.suptitle(f'Actual fixed depth patch: {aa} / {bb} {kind}; identical samples and axes')
        save(fig,f'floor2_patch_{aa}_{bb}_{kind}_{row["plane_a"]}_{row["plane_b"]}','Original finite depth samples reprojected using each diagnostic pose; no surface synthesis.')
    ss.connection.close()
    # All five active canonical inter-session measurements in original RGB.
    q=read(ROOT/'outputs/student_union_20261017_submap_quality/manifest.json')
    sessions={name:Session(Path(r['source_database']),name) for name,r in q['sessions'].items()}
    graph=read(inventory['domains']['canonical_joint']['source_graph'])
    pairs=[r for r in graph['edges'] if r['type']=='cross_session']
    fig,axes=plt.subplots(len(pairs),2,figsize=(12,3.5*len(pairs)))
    for axes_row,edge in zip(axes,pairs):
        sa,aa,sb,bb=edge['provenance']['frame_endpoints']
        for ax,(session,node) in zip(axes_row,[(sa,aa),(sb,bb)]):
            ax.imshow(cv2.cvtColor(sessions[session].image(node),cv2.COLOR_BGR2RGB));ax.axis('off');ax.set_title(f'{session}:{node} | {edge["role"]}')
    fig.suptitle('All canonical cross-session pairs: original RGB, not new correspondence validation')
    save(fig,'cross_session_original_rgb','Five canonical cross-session measurements. Adjacent or similar-looking structure is not sufficient for independent geometric validation.')
    pairs=[('floor2',644,1035),('floor2',620,1025),('floor2',639,1029),('floor2',548,981),('lobby',82,515)]
    fig,axes=plt.subplots(len(pairs),2,figsize=(12,3.5*len(pairs)))
    for axes_row,(session,aa,bb) in zip(axes,pairs):
        for ax,node in zip(axes_row,[aa,bb]):
            ax.imshow(cv2.cvtColor(sessions[session].image(node),cv2.COLOR_BGR2RGB));ax.axis('off');ax.set_title(f'{session}:{node}')
    fig.suptitle('Original RGB for conflict review; no fabricated correspondences')
    save(fig,'suspect_pair_original_rgb','Original RGB of target/inherited regressions and a lobby revisit; visual identification remains conditional.')
    for session in sessions.values():session.connection.close()
    history=history_inventory();write(out/'history_inventory.json',history)
    summary={k:report[k] for k in ('created_utc','status','nonlinear_job_count','domains','rankings','limitations')}
    summary.update(accepted=False,navigation=False,default_map_replaced=False,input_hashes=input_hashes,
                   excluded_prediction_and_pose_changes=detailed,provenance_errors=provenance['errors'],
                   source_roundtrips=len(provenance['canonical_revisit_source_roundtrips']),
                   factor_occurrence_count=len(factor_csv),historical_graph_inventory_count=len(history['graphs']))
    write(out/'summary.json',summary)
    write(out/'manifest.json',{'created_utc':utc_now(),'input_hashes':input_hashes,'figures':figures,
        'summary_sha256':sha256_file(out/'summary.json'),'test_status':'Run full pytest separately; see experiment document.',
        'default_map_replaced':False,'synthetic_images':False})
    print(json.dumps({'output':str(out),'figures':len(figures),'factor_occurrences':len(factor_csv)}))


if __name__=='__main__':main()
