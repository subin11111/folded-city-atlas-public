#!/usr/bin/env python3
"""Rebuild joint RGB-D clouds; rejected components require explicit preview mode."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from export_rtabmap_db import voxel_reduce, write_binary_ply
from optimize_session import sha256_file, utc_now
from render_session_report import read_ply


def export_policy(component, unvalidated_preview=False):
    """Preview authorization is not a change to the scientific acceptance guard."""
    if not component['accepted'] and not unvalidated_preview:
        return None
    return {
        'directory': 'preview_clouds' if unvalidated_preview else 'clouds',
        'status': ('UNVALIDATED_PREVIEW_NOT_FOR_NAVIGATION' if unvalidated_preview else
                   'experimental_joint_candidate_passed_internal_guards_not_surveyed'),
        'unvalidated_preview': bool(unvalidated_preview),
        'accepted_by_validation': bool(component['accepted']),
        'navigation_ready': False,
        'original_rejection_reasons': list(component.get('rejection_reasons', [])),
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('joint_run',type=Path)
    parser.add_argument('--config',type=Path,default=Path('config/student_union_all_sessions.json'))
    parser.add_argument('--unvalidated-preview', action='store_true',
                        help='Explicitly export rejected hypotheses into preview_clouds, never approved maps')
    parser.add_argument('--candidate-only', action='store_true',
                        help='Rebuild only the joint candidate, without the initial-placement comparison')
    args=parser.parse_args()
    config=json.loads(args.config.read_text())['sessions']
    report=json.loads((args.joint_run/'joint_report.json').read_text())
    environment=os.environ.copy()
    environment.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='1')
    completed=[]
    rejected_exported=False
    for component in report['components']:
        policy=export_policy(component, args.unvalidated_preview)
        if policy is None:
            continue
        root=Path(component['directory'])
        target=root/policy['directory']
        if target.exists():
            raise ValueError('Choose fresh output; clouds already exist: '+str(target))
        target.mkdir()
        manifest={'started_utc':utc_now(),'component_sessions':component['sessions'],
                  **policy, 'source_joint_run':str(args.joint_run.resolve()),
                  'excluded_disconnected_sessions':report.get('disconnected_sessions',[]),
                  'command':sys.argv,
                  'commands':[],'exports':{},'source_sha256':{str(p):sha256_file(p) for p in [Path(__file__),Path('src/export_rtabmap_db.py')]}}
        labels=('candidate',) if args.candidate_only else ('initial','candidate')
        for label in labels:
            clouds=[]
            for name in component['sessions']:
                ply=target/f'{name}_{label}.ply'
                summary=target/f'{name}_{label}.json'
                command=[sys.executable,'src/export_rtabmap_db.py',config[name]['database'],
                    '--cloud',str(ply),'--trajectory',str(target/f'{name}_{label}_trajectory.csv'),
                    '--summary',str(summary),'--pose-csv',str(root/name/(label+'_trajectory.csv')),
                    '--missing-pose-policy','skip','--frame-stride','1','--pixel-stride','1',
                    '--min-depth','.25','--max-depth','4','--min-confidence','50','--voxel','.05']
                for source in (Path(config[name]['database']),root/name/(label+'_trajectory.csv'),root/'joint_report.json'):
                    manifest['source_sha256'][str(source)]=sha256_file(source)
                start=time.monotonic()
                with (target/f'{name}_{label}.log').open('w') as stream:
                    proc=subprocess.run(command,env=environment,stdout=stream,stderr=subprocess.STDOUT)
                manifest['commands'].append({'argv':command,'exit_code':proc.returncode,'elapsed_seconds':time.monotonic()-start})
                if proc.returncode:
                    raise RuntimeError(f'Cloud export failed: {name}/{label}')
                clouds.append(read_ply(ply))
                manifest['exports'][name+'_'+label]=json.loads(summary.read_text())
                print(f'exported {name}/{label}: {len(clouds[-1])} points',flush=True)
            combined=np.concatenate(clouds)
            xyz=np.column_stack([combined[k] for k in ('x','y','z')])
            rgb=np.column_stack([combined[k] for k in ('r','g','b')])
            xyz,rgb,_=voxel_reduce(xyz,rgb,.05)
            write_binary_ply(target/f'combined_{label}.ply',xyz,rgb)
            manifest[label+'_combined_points']=len(xyz)
        sampled=[]
        for label in labels:
            cloud=read_ply(target/f'combined_{label}.ply')
            sampled.append(cloud[np.linspace(0,len(cloud)-1,min(180000,len(cloud))).astype(int)])
        bounds={k:(min(np.percentile(c[k],.5) for c in sampled),max(np.percentile(c[k],99.5) for c in sampled)) for k in ('x','y','z')}
        fig,axes=plt.subplots(2,len(labels),figsize=(6*len(labels),10),constrained_layout=True,squeeze=False)
        display_labels={'initial':'Independent maps + training-link placement','candidate':'Joint PGO candidate'}
        for col,(cloud,label) in enumerate(zip(sampled,labels)):
            for row,(x,y) in enumerate([('x','y'),('x','z')]):
                axes[row,col].scatter(cloud[x],cloud[y],c=cloud['z'],s=.12,cmap='viridis',vmin=bounds['z'][0],vmax=bounds['z'][1])
                axes[row,col].set(xlim=bounds[x],ylim=bounds[y],xlabel=x+' (m)',ylabel=y+' (m)',title=display_labels[label])
                axes[row,col].set_aspect('equal',adjustable='box')
        warning='UNVALIDATED PREVIEW - NOT FOR NAVIGATION' if args.unvalidated_preview else 'Internal-consistency candidate, not surveyed'
        fig.suptitle(' + '.join(component['sessions'])+'\n'+warning)
        fig.savefig(target/'joint_cloud_comparison.png',dpi=160)
        plt.close(fig)
        manifest['finished_utc']=utc_now()
        (target/'export_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        completed.append(str(target.resolve()))
        rejected_exported |= not component['accepted']
    print(json.dumps({'exported_components':completed,'rejected_components_exported':rejected_exported,
                      'unvalidated_preview':args.unvalidated_preview}),flush=True)


if __name__=='__main__':
    main()
