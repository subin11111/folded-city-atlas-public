#!/usr/bin/env python3
"""Evaluate fixed, pre-experiment finite patches across geometry graph stages.

Historical patch IDs and held-out samples are reproduced before examining new
poses. No planes/correspondences are fitted to optimized maps. A withheld depth
sample on a graph-training frame is not an independent graph validation pair.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sqlite3
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from evaluate_map_surfaces import Settings, fit_planes, pair_metrics
from export_rtabmap_db import load_pose_csv
from optimize_session import Clouds, apply, checked_transform, geometric_score, inverse, sha256_file, utc_now
from refine_geometry_surfaces import SPECS

ROOT=Path(__file__).resolve().parents[1]
PAIRS={'outside':[(69,323),(113,351)],'lobby':[(595,650),(445,495),(82,290)],
       'floor2':[(644,1035),(548,981),(656,1055),(608,1023),(503,961)]}
FITTING_PAIRS={('outside',69,323),('outside',113,351),('lobby',595,650),
               ('lobby',445,495),('floor2',644,1035)}


def fixed_patches(clouds,raw,node):
    train,test=clouds.get(node)
    rotation=raw[node][:3,:3]
    settings=Settings()
    result={}
    for kind,offset in [('horizontal',0),('wall',100000)]:
        planes,_=fit_planes(train@rotation.T,test@rotation.T,kind,settings,settings.seed+node+offset)
        for p in planes:
            p['normal']=rotation.T@p['normal']
            p['center']=rotation.T@p['center']
            p['training']=p['training']@rotation
            p['testing']=p['testing']@rotation
            result[kind,p['plane_index']]=p
    return result


def plot_patch(path,a,b,transforms,metrics,title):
    # Stable view from the original fitted patch, common to all map variants.
    normal=a['normal']
    _,_,vt=np.linalg.svd(a['training']-a['center'],full_matrices=False)
    basis=np.vstack([vt[0],vt[1],normal])
    target=(a['testing']-a['center'])@basis.T
    moving={k:(apply(b['testing'],t)-a['center'])@basis.T for k,t in transforms.items()}
    total=np.vstack([target,*moving.values()])
    lo,hi=np.min(total,axis=0),np.max(total,axis=0)
    pad=np.maximum(.03,.04*(hi-lo))
    fig,axes=plt.subplots(2,len(transforms),figsize=(4.8*len(transforms),7.8),squeeze=False)
    for col,(label,points) in enumerate(moving.items()):
        for row,tangent in enumerate([0,1]):
            ax=axes[row,col]
            ax.scatter(target[:,tangent],target[:,2],s=3,c='#147ba7',alpha=.65,label='fixed visit A')
            ax.scatter(points[:,tangent],points[:,2],s=3,c='#dc702c',alpha=.65,label='visit B')
            ax.axhline(0,color='#606060',lw=.7)
            ax.set(xlim=(lo[tangent]-pad[tangent],hi[tangent]+pad[tangent]),
                ylim=(lo[2]-pad[2],hi[2]+pad[2]),xlabel=f'Patch tangent {tangent+1} (m)',
                ylabel='Signed normal offset (m)')
            ax.grid(alpha=.2)
        m=metrics[label]['symmetric_plane_abs_median_m']
        axes[0,col].set_title(f'{label}\nmedian separation {m*100:.1f} cm',fontsize=10)
    axes[0,0].legend(fontsize=8)
    fig.suptitle(title+'\nIdentical original finite-patch samples; fixed patch axes and limits; no after-map refit',fontsize=12)
    fig.tight_layout(rect=(0,0,1,.91));fig.savefig(path,dpi=155);plt.close(fig)


def graph_role(name,a,b,native,extra):
    hits=[r for r in extra if r.get('session_a')==name and r.get('session_b')==name
          and {r['node_a'],r['node_b']}=={a,b}]
    if hits:
        return ('reserved_graph_holdout' if any(r.get('held_out') for r in hits) else 'graph_training'),hits[-1]
    hits=[r for r in native['verified_loops'] if {r['from_id'],r['to_id']}=={a,b}]
    if hits:return ('reserved_graph_holdout' if any(r.get('held_out') for r in hits) else 'native_graph_training'),None
    return 'not_a_graph_edge',None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--graph',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scan-all-historical',action='store_true',help='Scan every historical finite-patch correspondence, render requested critical pairs only')
    args=parser.parse_args()
    graph=args.graph.resolve();output=args.output.resolve()
    stages={'previous_preview':None,'selected_initial':'initial','pgo4':'pgo4'}
    if (graph/'floor2/pgo6_attitude.csv').exists():stages['pgo6_attitude']='pgo6_attitude'
    stages['pgo6_normals']='pgo6_normals'
    if (graph/'floor2/pgo6_planes.csv').exists():stages['pgo6_planes']='pgo6_planes'
    for name in PAIRS:
        for stage,stem in stages.items():
            path=(ROOT/'outputs/student_union_20261017_all_preview/bundle'/f'{name}_trajectory.csv'
                  if stem is None else graph/name/f'{stem}.csv')
            if not path.exists():raise FileNotFoundError(f'Graph not complete: {path}')
    if output.exists():raise FileExistsError('Use a fresh evaluation output')
    output.mkdir(parents=True)
    extra=json.loads((graph/'used_constraints.json').read_text())
    rows=[];hashes={};figures=[];pose_hashes={};relative_rows=[]
    for name,pairs in PAIRS.items():
        filename,prior_root,metric_sub=SPECS[name]
        database=ROOT/'data/raw/rtabmap'/filename
        hashes[name]={'before':sha256_file(database)}
        previous=ROOT/'outputs'/prior_root
        raw=load_pose_csv(previous/name/'raw_main_trajectory.csv')
        app=load_pose_csv(previous/'audit'/name/'app_optimized_trajectory.csv')
        native=json.loads((previous/name/'validated_edges.json').read_text())
        poses={}
        for stage,stem in stages.items():
            path=(ROOT/'outputs/student_union_20261017_all_preview/bundle'/f'{name}_trajectory.csv'
                  if stem is None else graph/name/f'{stem}.csv')
            poses[stage]=load_pose_csv(path);pose_hashes[str(path)]=sha256_file(path)
        historical=list(csv.DictReader((ROOT/'outputs/student_union_20261017_revalidated'/metric_sub/name/'plane_pair_metrics.csv').open()))
        historical=[r for r in historical if r['trajectory']=='app']
        if args.scan_all_historical:
            pairs=sorted({(int(r['from_id']),int(r['to_id'])) for r in historical})
        connection=sqlite3.connect(f'file:{database}?mode=ro',uri=True)
        clouds=Clouds(connection,None);cache={}
        directory=output/name;directory.mkdir()
        for a,b in pairs:
            for node in (a,b):
                if node not in cache:cache[node]=fixed_patches(clouds,raw,node)
            role,measured=graph_role(name,a,b,native,extra)
            native_t=checked_transform(connection.execute('SELECT transform FROM Link WHERE from_id=? AND to_id=? AND type IN (1,2)',(a,b)).fetchone()[0])
            transforms={stage:inverse(p[a])@p[b] for stage,p in poses.items()}
            measurements={'stored':native_t}
            if measured and 'T_a_b' in measured:measurements['graph_measurement']=np.asarray(measured['T_a_b'])
            for stage,t in transforms.items():
                row={'session':name,'a':a,'b':b,'graph_role':role,'stage':stage,
                     'new_surface_fitting_pair':(name,a,b) in FITTING_PAIRS,
                     'cloud_geometry':geometric_score(clouds.get(a)[1],clouds.get(b)[1],t,.2)}
                for label,m in measurements.items():
                    delta=inverse(m)@t
                    row[label+'_translation_residual_m']=float(np.linalg.norm(delta[:3,3]))
                    row[label+'_rotation_residual_degrees']=float(np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude()))
                relative_rows.append(row)
            matches=[r for r in historical if r['from_id']==str(a) and r['to_id']==str(b)]
            if not matches:raise ValueError(f'No fixed historical plane matches {name} {a} {b}')
            for historical_row in matches:
                kind=historical_row['kind'];ai=int(historical_row['from_plane']);bi=int(historical_row['to_plane'])
                pa,pb=cache[a][kind,ai],cache[b][kind,bi]
                original=pair_metrics(pa,pb,app[a],app[b])
                reproduction=abs(original['symmetric_plane_abs_median_m']-float(historical_row['symmetric_plane_abs_median_m']))
                if reproduction>5e-5:raise ValueError(f'Historical finite patch not reproduced: {name} {a} {b} {kind} {reproduction}')
                metrics={stage:pair_metrics(pa,pb,p[a],p[b]) for stage,p in poses.items()}
                for stage,m in metrics.items():
                    rows.append({'session':name,'a':a,'b':b,'kind':kind,'plane_a':ai,'plane_b':bi,
                                 'graph_role':role,'new_surface_fitting_pair':(name,a,b) in FITTING_PAIRS,
                                 'stage':stage,'historical_reproduction_error_m':reproduction,**m})
                severe=any(m['symmetric_plane_abs_median_m']>
                    min(metrics['selected_initial']['symmetric_plane_abs_median_m'],
                        metrics['previous_preview']['symmetric_plane_abs_median_m'])+.05
                    for label,m in metrics.items() if label.startswith('pgo'))
                if (a,b) in PAIRS[name] or severe:
                    filename=f'{a}_{b}_{kind}_{ai}_{bi}.png'
                    plot_patch(directory/filename,pa,pb,transforms,metrics,f'{name} {a}/{b}: {kind}; {role}')
                    figures.append(str(directory/filename))
                    print(f'{name} {a}/{b} {kind}: '+', '.join(f'{k}={v["symmetric_plane_abs_median_m"]*100:.2f}cm' for k,v in metrics.items()),flush=True)
        connection.close();hashes[name]['after']=sha256_file(database)
        if hashes[name]['before']!=hashes[name]['after']:raise RuntimeError('Source DB changed')
    comparison=[]
    keys=sorted({(r['session'],r['a'],r['b'],r['kind'],r['plane_a'],r['plane_b']) for r in rows})
    for key in keys:
        group={r['stage']:r for r in rows if (r['session'],r['a'],r['b'],r['kind'],r['plane_a'],r['plane_b'])==key}
        first=group['selected_initial'];result={k:first[k] for k in ['session','a','b','kind','plane_a','plane_b','graph_role','new_surface_fitting_pair']}
        result['stage_plane_abs_median_m']={k:r['symmetric_plane_abs_median_m'] for k,r in group.items()}
        for label in [k for k in stages if k.startswith('pgo')]:
            result[label+'_delta_vs_initial_m']=group[label]['symmetric_plane_abs_median_m']-first['symmetric_plane_abs_median_m']
            result[label+'_severe_regression']=result[label+'_delta_vs_initial_m']>.05
            result[label+'_delta_vs_previous_m']=group[label]['symmetric_plane_abs_median_m']-group['previous_preview']['symmetric_plane_abs_median_m']
            result[label+'_severe_regression_vs_previous']=result[label+'_delta_vs_previous_m']>.05
        comparison.append(result)
    chart_rows=[r for r in comparison if (r['a'],r['b']) in PAIRS[r['session']]]
    fig,ax=plt.subplots(figsize=(max(12,len(chart_rows)*.75),6.5))
    x=np.arange(len(chart_rows));width=.8/len(stages)
    for i,label in enumerate(stages):
        ax.bar(x+(i-(len(stages)-1)/2)*width,[100*r['stage_plane_abs_median_m'][label] for r in chart_rows],width,label=label)
    ax.set(xticks=x,xticklabels=[f"{r['session']}\n{r['a']}/{r['b']}\n{r['kind']}" for r in chart_rows],ylabel='Fixed finite-patch median separation (cm)')
    ax.grid(axis='y',alpha=.2);ax.legend(ncol=4)
    fig.suptitle('Fixed historical surfaces across map stages | graph training and reserved roles in report',fontsize=13)
    fig.tight_layout();fig.savefig(output/'finite_patch_progression.png',dpi=170);plt.close(fig)
    report={'created_utc':utc_now(),'command':sys.argv,'graph':str(graph),'source_sha256':sha256_file(Path(__file__)),
            'pose_sha256':pose_hashes,'database_sha256':hashes,'original_databases_unchanged':True,
            'method':'Historical finite patch IDs, deterministic raw-oriented train/test extraction, no after-map refit, gauge-invariant relative transforms.',
            'limitations':['Reserved depth samples on a graph-training pair are not independent graph validation.',
                '548/981 and horizontal82/290,503/961 are native graph training pairs, despite not used by new surface ICP.',
                'Requested reserved holdout examples include floor2656/1055 and608/1023; all actual pair roles are recorded individually.',
                'Original loop-defined finite-patch correspondence is conditional, not ground-truth survey.',
                'No physically measured stair rise, door width or floor height available.'],
            'scan_all_historical':args.scan_all_historical,'fixed_patch_count':len(keys),'comparisons':comparison,'patch_metrics':rows,'relative_pose_metrics':relative_rows,
            'severe_regressions_by_stage':{stage:sorted([r for r in comparison if r[stage+'_severe_regression']],
                key=lambda r:-r[stage+'_delta_vs_initial_m']) for stage in stages if stage.startswith('pgo')},
            'severe_regressions_vs_previous_by_stage':{stage:sorted([r for r in comparison if r[stage+'_severe_regression_vs_previous']],
                key=lambda r:-r[stage+'_delta_vs_previous_m']) for stage in stages if stage.startswith('pgo')},
            'figures':figures,'navigation_ready':False}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for label,data in [('patch_metrics',rows),('relative_pose_metrics',relative_rows)]:
        with (output/(label+'.csv')).open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=sorted({k for r in data for k in r}));writer.writeheader();writer.writerows(data)


if __name__=='__main__':main()
