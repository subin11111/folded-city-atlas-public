#!/usr/bin/env python3
"""Quantify visual equivalence and the sub-stair fitted closure, without a solve."""
import argparse
import csv
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from analyze_visual_shortlist import ROOT, PORTFOLIO, OLD, read, write
from export_rtabmap_db import load_pose_csv
from optimize_session import Clouds, geometric_score, inverse, sha256_file, utc_now


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--assets',type=Path,required=True)
    a=p.parse_args();assets=a.assets.resolve();report=read(assets/'summary.json');figures=[]
    if (assets/'supplement_manifest.json').exists():raise FileExistsError('Supplement exists')
    def save(fig,name,caption):
        fig.tight_layout(rect=(0,0,1,.94));path=assets/(name+'.png');fig.savefig(path,dpi=160);plt.close(fig)
        figures.append({'file':path.name,'sha256':sha256_file(path),'caption':caption,'synthetic':False})
    pairs=[('outside','J02','J03'),('outside','J02','J04'),('lobby','H02','H03'),
           ('bridge_out1f','H02','H03'),('bridge_12_sub','H02','H03'),('bridge_12_sub','H02','H04')]
    rows=[]
    for name,left,right in pairs:
        d=report['sessions'][name]['candidate_pose_differences'][left+'__'+right]
        rows.append({'session':name,'left':left,'right':right,'maximum_translation_mm':d['translation_m']['maximum']*1000,
                     'maximum_rotation_deg':d['rotation_deg']['maximum']})
    fig,axes=plt.subplots(1,2,figsize=(14,5));labels=[f'{r["session"]} {r["left"]}/{r["right"]}' for r in rows]
    for ax,key,title in zip(axes,['maximum_translation_mm','maximum_rotation_deg'],['Max camera-position difference (mm)','Max camera-orientation difference (deg)']):
        vals=[r[key] for r in rows];ax.barh(range(len(rows)),vals,color='#4c89a6')
        ax.set_yticks(range(len(rows)),labels,fontsize=8);ax.invert_yaxis();ax.set_xscale('symlog',linthresh=1e-5)
        ax.set_xlim(0,max(.1,max(vals)*8) if key=='maximum_rotation_deg' else max(75.,max(vals)*3))
        ax.set_xlabel(title);ax.grid(axis='x',alpha=.2)
        for i,v in enumerate(vals):ax.annotate(f'{v:.4g}',(v,i),xytext=(4,0),textcoords='offset points',va='center',fontsize=8)
    axes[0].axvline(50,color='#bc7150',ls='--',label='Export voxel: 50 mm');axes[0].legend(fontsize=8)
    fig.suptitle('Several visually preferred variants are numerically the same or nearly the same\nValues are differences between fitted candidates, not absolute map accuracy')
    save(fig,'pose_equivalence','All differences use identical A/B frame support and existing common display gauge. Same input rays and 5cm voxels can still have sampling/voxel-boundary display differences.')
    # Source is a fitted extra constraint, not an independently reserved loop.
    name='bridge_12_sub';summary=read(PORTFOLIO/name/'summary.json');stage={r['id']:r for r in summary['stages']}
    graph_path=OLD/'isolation'/name/'manifest.json';graph=read(graph_path)
    edges=[r for r in graph['edges'] if r['type']=='loop' and r['role']=='train']
    closure=[];poses={};source_paths=[assets/'summary.json',graph_path,Path(__file__)]
    database=Path(read(ROOT/'outputs/student_union_20261017_submap_quality/manifest.json')['sessions'][name]['source_database'])
    before=sha256_file(database)
    with sqlite3.connect(f'file:{database}?mode=ro',uri=True) as con:
        clouds=Clouds(con,None)
        for code in ['O00','H02','H03','H04','N05','N06']:
            pose_path=Path(stage[code]['trajectory']);source_paths.append(pose_path);pp=load_pose_csv(pose_path)
            for edge in edges:
                sa,aa,sb,bb=edge['provenance']['frame_endpoints']
                if aa not in pp or bb not in pp:continue
                score=geometric_score(clouds.get(aa)[1],clouds.get(bb)[1],inverse(pp[aa])@pp[bb],.20)
                closure.append({'candidate':code,'a':aa,'b':bb,'edge_id':edge['id'],'role':'TRAINING_CORRELATED_DIAGNOSTIC_NOT_HOLDOUT',**score})
    assert before==sha256_file(database)
    fig,axes=plt.subplots(1,2,figsize=(12,5));labels=[r['candidate'] for r in closure];x=np.arange(len(labels))
    axes[0].bar(x,[r['clipped_rmse_m']*100 for r in closure],color='#5a8ba7');axes[0].set_ylabel('Clipped symmetric RMSE (cm)')
    axes[1].bar(x,[r['overlap']*100 for r in closure],color='#6c9c82');axes[1].set_ylabel('Symmetric overlap (%)')
    for ax in axes:ax.set_xticks(x,labels);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Sub-stair fitted 75 / 209 closure: diagnostic only, NOT an independent validation\nNeither low local error nor a smooth camera-Z trace proves correct stair rise')
    save(fig,'substairs_training_closure','Unfiltered original test voxels of an existing TRAIN factor; correlated same-sensor geometry, not an independent reserved region.')
    write(assets/'supplement.json',{'created_utc':utc_now(),'pose_equivalence':rows,'substairs_training_closure':closure,
        'database_unchanged':True,'default_map_modified':False,'source_hashes':{str(path.resolve()):sha256_file(path) for path in source_paths}})
    write(assets/'supplement_manifest.json',{'figures':figures,'supplement_sha256':sha256_file(assets/'supplement.json'),
                                          'synthetic':False,'default_map_modified':False})
    print({'figures':len(figures),'substairs_closure_rows':len(closure)})


if __name__=='__main__':main()
