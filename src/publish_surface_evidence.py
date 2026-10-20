#!/usr/bin/env python3
"""Publish compact actual-data comparison plots and integrity records."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from optimize_session import sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--layers',type=Path,required=True);p.add_argument('--assets',type=Path,required=True)
    p.add_argument('--gaussians',type=Path)
    p.add_argument('--reuse-figures',action='store_true',help='Verify previously published plots without overwriting their pixels')
    a=p.parse_args();report=json.loads((a.run/'manifest.json').read_text());assets=a.assets
    names=list(report['sessions']);methods=['tsdf_5cm','tsdf_3cm','ball_pivoting'];x=np.arange(len(names));figures=[]
    def save(fig,filename):
        path=assets/filename
        if path.exists():
            if not a.reuse_figures:raise FileExistsError(path)
            previous=json.loads((assets/'verification.json').read_text())
            expected=next(r['sha256'] for r in previous['summary_figures'] if r['file']==filename)
            if sha256_file(path)!=expected:raise ValueError('Published plot changed: '+filename)
        else:fig.savefig(path,dpi=170)
        plt.close(fig);Image.open(path).verify()
        figures.append({'file':filename,'sha256':sha256_file(path),'synthetic':False})
    fig,ax=plt.subplots(figsize=(11,5),constrained_layout=True)
    for j,method in enumerate(methods):
        ax.bar(x+(j-1)*.25,[report['sessions'][n]['models'][method]['triangles'] for n in names],width=.25,label=method)
    ax.set(xticks=x,xticklabels=names,yscale='log',ylabel='Full-resolution triangles (log scale)',
           title='Actual reconstruction complexity; display copies are separately limited to 180k triangles')
    ax.legend();ax.grid(axis='y',alpha=.15);save(fig,'mesh_complexity.png')
    fig,axes=plt.subplots(1,2,figsize=(15,5),constrained_layout=True)
    for j,method in enumerate(methods):
        axes[0].bar(x+(j-1)*.25,[report['sessions'][n]['models'][method]['reference_points_within_5cm']*100 for n in names],width=.25,label=method)
        axes[1].bar(x+(j-1)*.25,[report['sessions'][n]['models'][method]['reference_to_surface_p90_m']*100 for n in names],width=.25,label=method)
    for ax in axes:ax.set_xticks(x,names,rotation=20);ax.grid(axis='y',alpha=.15)
    axes[0].set(ylabel='Reference map points within 5cm of reconstructed surface (%)',ylim=(0,100))
    axes[1].set(ylabel='Reference-to-surface P90 (cm)');axes[1].legend()
    fig.suptitle('Same-sensor representation coverage/consistency, NOT new registration or surveyed accuracy')
    save(fig,'representation_consistency.png')
    fig,axes=plt.subplots(1,2,figsize=(13,5),constrained_layout=True)
    axes[0].bar(x,[report['sessions'][n]['texture']['textured_face_fraction']*100 for n in names],color='#3f8999')
    axes[0].set(ylabel='Display faces assigned a depth-checked photo (%)',ylim=(0,100))
    axes[1].bar(x,[report['sessions'][n]['detail_overlay_points'] for n in names],color='#a27747')
    axes[1].set(ylabel='1.5cm visual-detail overlay points')
    for ax in axes:ax.set_xticks(x,names,rotation=20);ax.grid(axis='y',alpha=.15)
    fig.suptitle('Unassigned faces retain RGB; detail candidates are NOT semantic sign detections')
    save(fig,'texture_and_detail_coverage.png')
    # Every source is checked again after reconstruction/rendering/neural work.
    hashes=report['input_hashes_after']
    for path,h in hashes.items():
        if sha256_file(Path(path))!=h:raise ValueError('Source changed: '+path)
    original_verified=0
    for n,s in report['sessions'].items():
        for relative,item in s['outputs'].items():
            path=a.run/n/relative
            if sha256_file(path)!=item['sha256']:raise ValueError('Reconstruction changed: '+str(path))
            original_verified+=1
    layer=json.loads((a.layers/'manifest.json').read_text())
    depth_sheet_hashes={}
    for n,s in layer['sessions'].items():
        for method,item in s['colour_standardized_models'].items():
            if sha256_file(a.layers/n/(method+'.glb'))!=item['sha256']:raise ValueError('Display layer changed')
        sheet=a.layers/n/'depth_sheet_textured.glb'
        if sheet.stat().st_size!=s['depth_sheet']['bytes']:raise ValueError('Depth sheet size changed')
        depth_sheet_hashes[n]=sha256_file(sheet)
    render=json.loads((assets/'manifest.json').read_text())
    for item in render['figures']:
        if sha256_file(assets/item['file'])!=item['sha256']:raise ValueError('Render changed')
    integrity={'created_utc':utc_now(),'all_original_inputs_unchanged':True,'frozen_input_files':len(hashes),
        'reconstruction_files_verified':original_verified,'rendered_figures_verified':len(render['figures']),
        'summary_figures':figures,'source_map_replaced':False,'navigation_ready':False,
        'source_sha256':sha256_file(Path(__file__)),
        'selected_manifest_sha256':render['selected_manifest_sha256'],
        'surface_report_sha256':sha256_file(a.run/'manifest.json'),
        'display_layer_manifest_sha256':sha256_file(a.layers/'manifest.json')}
    integrity['depth_sheet_hashes_at_publication']=depth_sheet_hashes
    if a.gaussians:
        gaussian=json.loads((a.gaussians/'manifest.json').read_text())
        if not gaussian['all_inputs_unchanged']:raise ValueError('Gaussian input integrity failed')
        gaussian_files=0;gaussian_figures=0
        for path,h in gaussian['inputs_sha256'].items():
            if sha256_file(Path(path))!=h:raise ValueError('Gaussian source changed: '+path)
        for n,s in gaussian['sessions'].items():
            for relative,item in s['outputs'].items():
                if sha256_file(a.gaussians/n/relative)!=item['sha256']:raise ValueError('Gaussian output changed')
                gaussian_files+=1
            for item in s['figures']:
                path=ROOT/'docs/assets/gaussian-display-review'/n/item['file']
                if sha256_file(path)!=item['sha256']:raise ValueError('Gaussian image changed')
                Image.open(path).verify();gaussian_figures+=1
        integrity['gaussian_report_sha256']=sha256_file(a.gaussians/'manifest.json')
        integrity['gaussian_sessions']=list(gaussian['sessions'])
        integrity['gaussian_outputs_verified']=gaussian_files
        integrity['gaussian_figures_verified']=gaussian_figures
    (assets/'verification.json').write_text(json.dumps(integrity,indent=2)+'\n')
    # Small report avoids shipping reconstructed PLY/GLB or native photos to Git.
    compact={n:{k:v for k,v in s.items() if k not in ['outputs']} for n,s in report['sessions'].items()}
    (assets/'surface_metrics.json').write_text(json.dumps({'sessions':compact,'limitations':report['limitations'],
        'source_run':str(a.run.resolve()),'selected_map_unchanged':True},indent=2)+'\n')
    print(json.dumps({'verified':True,'source_files':len(hashes),'reconstructed_files':original_verified,
                      'figures':len(render['figures'])+len(figures)}))


if __name__=='__main__':main()
