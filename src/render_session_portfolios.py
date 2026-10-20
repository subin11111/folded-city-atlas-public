#!/usr/bin/env python3
"""Actual RGB-D map gallery: full app, preprocessing only, optimization portfolio."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
import hashlib
import json
import math
import multiprocessing
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from build_session_portfolios import ROOT, OLD, QUALITY, read, dump
from export_rtabmap_db import load_pose_csv, voxel_reduce, write_binary_ply
from export_submap_candidate_clouds import cloud_bounds, render_projection
from evaluate_submap_candidates import fixed_patches
from evaluate_map_surfaces import pair_metrics
from optimize_session import Clouds, apply, geometric_score, inverse, saved_app_poses, sha256_file, trajectory, utc_now
from render_session_report import read_ply
from revalidate_loop_constraints import SPECS
from session_preprocessing import RGBD, BASE


def render_one(job):
    name,run,out=job;run=Path(run);out=Path(out)/name;out.mkdir(parents=True,exist_ok=False)
    source_report=run/name/'report.json';record=read(source_report);db=Path(record['database'])
    if sha256_file(db)!=record['database_sha256']:raise ValueError('DB hash changed')
    rgbd=RGBD(db);stages=record['stages'];poses={s['id']:load_pose_csv(Path(s['trajectory'])) for s in stages}
    ids=record['frame_ids'];allapp=saved_app_poses(rgbd.connection)
    exportable={int(r[0]) for r in rgbd.connection.execute('SELECT n.id FROM Node n JOIN Data d ON d.id=n.id WHERE d.image IS NOT NULL AND d.depth IS NOT NULL AND d.calibration IS NOT NULL')}
    fullids=sorted(set(allapp)&exportable);gauge=np.asarray(stages[0]['display_gauge'])
    fullposes={n:gauge@allapp[n] for n in fullids};fullpath=run/name/'full_app_trajectory.csv'
    stamps={int(n):float(t) for n,t in rgbd.connection.execute('SELECT id,stamp FROM Node')}
    if fullpath.exists():raise FileExistsError(fullpath)
    trajectory(fullpath,fullposes,stamps)
    fullstage={'id':'O_FULL','label':'FULL phone app optimized / before quality filtering','trajectory':str(fullpath),
        'trajectory_sha256':sha256_file(fullpath),'newly_computed':False,'source':'DB Admin.opt_poses','preprocessing':'base',
        'solver':None,'display_gauge':gauge.tolist()}
    stages=[fullstage]+stages;poses['O_FULL']=fullposes
    # Stream each frame once per point policy, transform it for every candidate,
    # batch voxel-reduce; no plane synthesis, cropping of bad regions or ICP here.
    buffers={s['id']:[] for s in stages};reduced={s['id']:[] for s in stages}
    point_hash={key:hashlib.sha256() for key in ('base_full','base_AB','preprocessed')};stats={}
    cloudroot=run/name/'clouds';cloudroot.mkdir(exist_ok=False)
    for index,node in enumerate(fullids):
        baseline=rgbd.get(node,BASE,cache=False);filtered=rgbd.get(node,record['selected_profile'],cache=False) if node in ids else None
        for key,data in [('base_full',baseline),('base_AB',baseline if node in ids else None),('preprocessed',filtered)]:
            if data is None:continue
            point_hash[key].update(np.asarray([node],np.int64).tobytes());point_hash[key].update(data['points'].tobytes())
        for stage in stages:
            key=stage['id']
            if node not in poses[key]:continue
            data=baseline if key in ('O_FULL','O00') else filtered
            points=apply(data['points'],poses[key][node]).astype(np.float32);colors=data['colors']
            stats.setdefault(key,{'frame_ids':[],'input_points':0})
            stats[key]['frame_ids'].append(node);stats[key]['input_points']+=len(points)
            buffers[key].append((points,colors))
        if (index+1)%20==0 or index+1==len(fullids):
            for key in buffers:
                if not buffers[key]:continue
                pp=np.concatenate([r[0] for r in buffers[key]]);cc=np.concatenate([r[1] for r in buffers[key]])
                reduced[key].append(voxel_reduce(pp,cc,.05));buffers[key]=[]
        if (index+1)%150==0:print(json.dumps({'render_session':name,'decoded_frames':index+1,'total_frames':len(fullids)}),flush=True)
    sourcepoint_hash={key:h.hexdigest() for key,h in point_hash.items()}
    clouds={};exports=[]
    for stage in stages:
        key=stage['id'];chunks=reduced.pop(key)
        pp=np.concatenate([r[0] for r in chunks]);cc=np.concatenate([r[1] for r in chunks]);ww=np.concatenate([r[2] for r in chunks])
        pp,cc,ww=voxel_reduce(pp,cc,.05,ww);path=cloudroot/(key+'.ply');write_binary_ply(path,pp,cc)
        clouds[key]=read_ply(path)
        exports.append({'id':key,'cloud':str(path),'sha256':sha256_file(path),'points':len(pp),**stats[key],
                        'camera_points_sha256':sourcepoint_hash['base_full' if key=='O_FULL' else 'base_AB' if key=='O00' else 'preprocessed']})
    # The raw/full app and A/B-only app are separate references. All algorithm
    # candidates receive exactly P00's preprocessed camera-point observations.
    common=stats['P00']['frame_ids'];count=stats['P00']['input_points']
    for stage in stages:
        if stage['id'] in ('O00','O_FULL'):continue
        assert stats[stage['id']]['frame_ids']==common and stats[stage['id']]['input_points']==count
    # Same historical finite patches across candidates; the preprocessing is NOT
    # applied to evaluation patches, so removing a wall cannot improve this metric.
    historical=read(OLD/'evaluation/report.json')['comparisons']
    patches=[r for r in historical if r['session']==name and {r['a'],r['b']}<=set(ids)]
    rawpath=ROOT/'outputs'/SPECS[name][1]/name/'raw_main_trajectory.csv';raw=load_pose_csv(rawpath)
    evaluator=Clouds(rgbd.connection,None);cache={}
    for row in patches:
        for n in (row['a'],row['b']):
            if n not in cache:cache[n]=fixed_patches(evaluator,raw,n)
    evaluation={}
    for stage in stages:
        key=stage['id'];values=[];items=[]
        for row in patches:
            a,b,kind=row['a'],row['b'],row['kind']
            metric=pair_metrics(cache[a][kind,row['plane_a']],cache[b][kind,row['plane_b']],poses[key][a],poses[key][b])
            value=metric['symmetric_plane_abs_median_m'];values.append(value)
            items.append({'a':a,'b':b,'kind':kind,'value_m':value})
        evaluation[key]={'count':len(values),'median_cm':float(np.median(values)*100) if values else None,
                         'p90_cm':float(np.percentile(values,90)*100) if values else None,
                         'maximum_cm':float(max(values)*100) if values else None,'patches':items}
    # Bounds cover the full point extents across ALL candidates, not percentile
    # cropping. A bad candidate cannot be made to look good by omitting its tail.
    bounds={axis:[float(min(c[axis].min() for c in clouds.values())),float(max(c[axis].max() for c in clouds.values()))] for axis in ('x','y','z')}
    for axis in bounds:
        lo,hi=bounds[axis];padding=max(.1,(hi-lo)*.025);bounds[axis]=[lo-padding,hi+padding]
    figures=[]
    def save(fig,filename,caption):
        fig.tight_layout(rect=(0,0,1,.95));path=out/filename;fig.savefig(path,dpi=150);plt.close(fig)
        with Image.open(path) as im:size=list(im.size);im.verify()
        figures.append({'file':filename,'sha256':sha256_file(path),'size_pixels':size,'caption':caption,'synthetic':False})
    def title(stage):
        e=evaluation[stage['id']];status='NEW' if stage.get('newly_computed') else 'REUSED POSES'
        metric=f'fixed-patch P90 {e["p90_cm"]:.2f} cm' if e['count'] else 'NO fixed-patch validation'
        solver=stage.get('solver') or {};warning=''
        if solver.get('active_bounds',0):warning+=' | ACTIVE BOUNDS'
        if solver.get('inner_success',solver.get('success',True)) is False:warning+=' | SOLVER LIMIT'
        if solver.get('strict_outer_converged') is False:warning+=' | outer not converged'
        return f'{stage["id"]}: {stage["label"]}\n{len(poses[stage["id"]])} frames | {status} | {metric}{warning}'
    for stage in stages:
        fig,axes=plt.subplots(1,2,figsize=(13,7))
        for ax,(first,second) in zip(axes,[('x','y'),('x','z')]):
            render_projection(ax,clouds[stage['id']],first,second,bounds,title=f'{first.upper()}-{second.upper()}',max_points=350000)
        fig.suptitle(name+' | '+title(stage),fontsize=11)
        save(fig,stage['id']+'.png','Actual RGB-D reprojected using named poses; common full extents and height colors. Patches evaluate unfiltered fixed observations.')
    for suffix,keys in [('three_stage',['O_FULL','P00','N02']),('matched_three_stage',['O00','P00','N02'])]:
        fig,axes=plt.subplots(2,3,figsize=(19,11))
        for col,key in enumerate(keys):
            stage=next(s for s in stages if s['id']==key)
            for rr,(first,second) in enumerate([('x','y'),('x','z')]):
                render_projection(axes[rr,col],clouds[key],first,second,bounds,
                    title=title(stage) if rr==0 else '',max_points=350000)
        fig.suptitle(name+' | 1 Original app -> 2 Preprocessing ONLY -> 3 NEW DCS example (not a selected winner)\n'+
                     ('Full app includes frames rejected by A/B quality screening.' if suffix=='three_stage' else 'Identical A/B frames; first two use exactly the same phone poses.'),fontsize=13)
        save(fig,suffix+'.png','Third column is N02 for a consistent example, not an automatic winner. Preprocessed optimization candidates use exactly P00 camera points.')
    cols=4;rr=math.ceil(len(stages)/cols)
    for view,(first,second) in [('xy',('x','y')),('xz',('x','z'))]:
        fig,axes=plt.subplots(rr,cols,figsize=(23,5.5*rr),squeeze=False)
        for ax,stage in zip(axes.ravel(),stages):
            render_projection(ax,clouds[stage['id']],first,second,bounds,title=title(stage),max_points=240000)
            ax.title.set_fontsize(8)
        for ax in axes.ravel()[len(stages):]:ax.axis('off')
        fig.suptitle(name+' | Every candidate, '+view.upper()+' | choose by candidate ID; no map promoted',fontsize=15)
        save(fig,'portfolio_'+view+'.png','Each candidate uses the same preprocessed observations except explicit original references; historical joint and frame candidates are labelled, not re-executed.')
    rgbd.connection.close()
    if sha256_file(db)!=record['database_sha256']:raise ValueError('DB changed in export')
    summary={k:record[k] for k in ('session','frame_count','selected_profile','profile_trials','profile_reason','new_refinement_count','normal_factor_count','limitations')}
    summary.update(created_utc=utc_now(),accepted=False,navigation=False,stages=stages,evaluation=evaluation,
        exports=exports,source_report=str(source_report),source_report_sha256=sha256_file(source_report),
        export_settings={'pixel_stride':2,'frame_stride':1,'voxel_m':.05,'common_full_cloud_bounds':bounds,
            'depth_min_m':.25,'baseline_max_depth_m':4.,'baseline_confidence_minimum':50,
            'pose_only_comparison_candidates_share_exact_preprocessed_camera_points':True},
        full_app_frame_count=len(fullids),main_three_stage_ids=['O_FULL','P00','N02'],user_winner=None,
        database_sha256=record['database_sha256'],database_unchanged=True,default_rviz_changed=False)
    dump(out/'summary.json',summary)
    dump(out/'manifest.json',{'created_utc':utc_now(),'figures':figures,'summary_sha256':sha256_file(out/'summary.json'),
        'source_sha256':sha256_file(Path(__file__)),'synthetic':False,'default_rviz_changed':False})
    print(json.dumps({'published_session':name,'figures':len(figures),'clouds':len(exports),'full_app_frames':len(fullids)}),flush=True)
    return {'session':name,'summary':str(out/'summary.json'),'figures':len(figures),'clouds':len(exports)}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=2)
    a=p.parse_args();run=a.run.resolve();out=a.output.resolve()
    if out.exists():raise FileExistsError(out)
    read(run/'manifest.json');out.mkdir(parents=True);results=[]
    shutil.copyfile(Path(__file__),run/'render_source_snapshot.py')
    with ProcessPoolExecutor(max_workers=a.workers,mp_context=multiprocessing.get_context('fork')) as pool:
        futures=[pool.submit(render_one,(name,str(run),str(out))) for name in SPECS]
        for future in as_completed(futures):results.append(future.result())
    # Overview uses the already-verified three-stage PNGs, without changing maps.
    from PIL import ImageOps, ImageDraw
    thumbnails=[]
    for name in SPECS:
        with Image.open(out/name/'three_stage.png') as im:
            thumbnails.append(ImageOps.contain(im.convert('RGB'),(1800,1050)))
    overview=Image.new('RGB',(1800,sum(im.height for im in thumbnails)),color='white');top=0
    for im in thumbnails:overview.paste(im,((1800-im.width)//2,top));top+=im.height
    overview.save(out/'overview.png')
    dump(out/'manifest.json',{'created_utc':utc_now(),'sessions':results,'overview_sha256':sha256_file(out/'overview.png'),
        'accepted':False,'navigation':False,'user_winner':None,'default_rviz_changed':False})
    print(json.dumps({'complete':True,'sessions':len(results),'output':str(out)}),flush=True)


if __name__=='__main__':main()
