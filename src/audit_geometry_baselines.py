#!/usr/bin/env python3
"""Freeze evidence-based starting poses and diagnose duplicated map surfaces.

This read-only audit does not optimize, flatten floors, or infer true building
levels. All candidate trajectories are aligned by their first common node only.
The prior graph's whole-pair reserve is independent of the offline fit, but may
have been used by the app and prior model selection: it is a selection set, NOT
a fresh final validation set. Native loop measurements establish tentative patch
correspondence, with deterministic held-out depth samples for relative geometry.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import sqlite3
import sys
import zlib

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from evaluate_map_surfaces import (Settings, anchor_align, fit_planes,
                                   pair_metrics, patch_correspondence)
from export_rtabmap_db import calibration_from_blob, load_pose_csv
from optimize_session import (Clouds, apply, checked_transform, geometric_score,
                              geometry_metrics, inverse, sha256_file, trajectory,
                              utc_now, write_csv)


def aggregate(rows, field):
    values = np.asarray([r[field] for r in rows if r.get(field) is not None], float)
    return ({'count': 0} if not len(values) else
            {'count': len(values), 'median': float(np.median(values)),
             'p90': float(np.percentile(values, 90)), 'max': float(values.max())})


def select_baseline(metrics):
    """Require a useful selection-set gain; preserve app if evidence is weak.

    App is a conservative operational baseline, not presumed ground truth. A
    candidate must improve median clipped distance >= 1cm, preserve overlap and
    avoid >2cm p90 finite-wall regression. No reserve => no measured promotion.
    """
    reference = metrics['app']
    held = reference['graph_reserve_geometry']
    if held.get('count', 0) < 3:
        return 'app', 'Insufficient whole-pair selection reserve; retain native app baseline provisionally.'
    choices = []
    for label, result in metrics.items():
        if label in ('raw', 'app'):
            continue
        score = result['graph_reserve_geometry']
        if score.get('count') != held['count']:
            continue
        wall, ref_wall = result['wall_plane_abs_m'], reference['wall_plane_abs_m']
        wall_ok = not (wall.get('count', 0) and ref_wall.get('count', 0)) or wall['p90'] <= ref_wall['p90']+.02
        if (score['clipped_rmse_median_m'] <= held['clipped_rmse_median_m']-.01
                and score['overlap_median'] >= held['overlap_median']-.03 and wall_ok):
            choices.append((score['clipped_rmse_median_m'], label))
    if choices:
        return min(choices)[1], 'At least 1cm median selection-set gain; overlap and finite-wall guards satisfied. Requires fresh external validation.'
    return 'app', 'No offline candidate clears predeclared 1cm selection-set gain and nonregression guards; retain app provisionally.'


def calibration_audit(connection, nodes):
    """Inspect per-frame intrinsics, native local transform, float depth encoding."""
    records = []
    for node in nodes:
        calibration, depth_blob, conf_blob, image_blob = connection.execute(
            'SELECT calibration,depth,depth_confidence,image FROM Data WHERE id=?', (node,)).fetchone()
        cw, ch, k, local = calibration_from_blob(calibration)
        encoded = cv2.imdecode(np.frombuffer(depth_blob, np.uint8), cv2.IMREAD_UNCHANGED)
        if encoded is None or encoded.ndim != 3 or encoded.shape[2] != 4:
            raise ValueError('Unexpected depth encoding')
        depth = np.ascontiguousarray(encoded).view('<f4').reshape(encoded.shape[:2])
        conf_bytes = zlib.decompress(conf_blob)
        conf = np.frombuffer(conf_bytes[:depth.size], np.uint8).reshape(depth.shape)
        image = cv2.imdecode(np.frombuffer(image_blob, np.uint8), cv2.IMREAD_COLOR)
        scaled_k = k.copy()
        scaled_k[0] *= depth.shape[1]/cw
        scaled_k[1] *= depth.shape[0]/ch
        records.append({'node_id': node, 'calibration_size': [cw, ch],
            'rgb_size': list(image.shape[1::-1]), 'depth_size': list(depth.shape[::-1]),
            'depth_intrinsics': scaled_k.tolist(), 'local_transform': local.tolist(),
            'confidence_values': np.unique(conf).tolist(),
            'confidence_50_fraction': float((conf >= 50).mean()),
            'valid_025_4m_conf50_fraction': float((np.isfinite(depth) & (depth >= .25) & (depth <= 4.) & (conf >= 50)).mean()),
            'local_det': float(np.linalg.det(local[:3,:3]))})
    local0 = np.asarray(records[0]['local_transform'])
    return {'sampled_frames': len(records), 'records': records,
            'max_local_transform_abs_change': float(max(np.abs(np.asarray(r['local_transform'])-local0).max() for r in records)),
            'max_intrinsic_change_depth_pixels': float(np.ptp(np.array([r['depth_intrinsics'] for r in records]), axis=0).max()),
            'note': 'Variable intrinsic values are read per-frame. Constant native extrinsic does not verify factory calibration; float depth reinterpretation and confidence threshold match existing exporter.'}


def surface_pairs(connection, clouds, raw, loops):
    settings = Settings()
    patches = {}
    for node in sorted({i for a,b,_ in loops for i in (a,b)}):
        tr, te = clouds.get(node)
        rot = raw[node][:3,:3]
        accepted = []
        for kind, offset in [('horizontal', 0), ('wall', 100000)]:
            found, _ = fit_planes(tr @ rot.T, te @ rot.T, kind, settings, settings.seed+node+offset)
            for p in found:
                for key in ('normal', 'center'):
                    p[key] = rot.T @ p[key]
                for key in ('training', 'testing'):
                    p[key] = p[key] @ rot
                accepted.append(p)
        patches[node] = accepted
    result = []
    for a,b,held in loops:
        row = connection.execute('SELECT transform FROM Link WHERE from_id=? AND to_id=? AND type IN (1,2)',(a,b)).fetchone()
        measurement = checked_transform(row[0])
        candidates=[]
        for pa in patches[a]:
            for pb in patches[b]:
                match=patch_correspondence(pa,pb,measurement,settings)
                if match:
                    candidates.append((match['measurement_plane_abs_m'],pa,pb,match))
        useda,usedb=set(),set()
        for _,pa,pb,match in sorted(candidates,key=lambda x:x[0]):
            ka,kb=(pa['kind'],pa['plane_index']),(pb['kind'],pb['plane_index'])
            if ka in useda or kb in usedb:
                continue
            useda.add(ka);usedb.add(kb)
            result.append((a,b,held,pa,pb,match))
    return result


def plot_clouds(directory, name, poses, clouds, stamps, selected):
    labels=['raw','app','independent']
    if selected not in labels:
        labels.append(selected)
    ids=sorted(poses['raw'])[::12]
    xyz, times = {},[]
    for label in labels:
        coords=[]
        for n in ids:
            tr,te=clouds.get(n)
            local=np.vstack((tr,te))[::3]
            coords.append(apply(local,poses[label][n]))
            if label==labels[0]: times.extend([stamps[n]]*len(local))
        xyz[label]=np.concatenate(coords)
    allpoints=np.concatenate(list(xyz.values()))
    limits=[(float(allpoints[:,i].min()-.3),float(allpoints[:,i].max()+.3)) for i in range(3)]
    color=np.asarray(times)-min(times)
    fig,axes=plt.subplots(2,len(labels),figsize=(5*len(labels),9),constrained_layout=True)
    for j,label in enumerate(labels):
        for row,(x,y) in enumerate(((0,1),(0,2))):
            ax=axes[row,j]
            ax.scatter(xyz[label][:,x],xyz[label][:,y],c=color,s=.18,cmap='turbo',rasterized=True)
            ax.set(xlim=limits[x],ylim=limits[y],xlabel='XYZ'[x]+' (m)',ylabel='XYZ'[y]+' (m)',title=label+(' [frozen]' if selected==label else ''))
            ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.suptitle(name+' | identical sampled RGB-D observations; color = acquisition time; no visual selection')
    fig.savefig(directory/'cloud_baselines.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
    for label,p in poses.items():
        ns=sorted(p); xyz=np.array([p[n][:3,3] for n in ns]); ts=np.array([stamps[n] for n in ns]); ts-=ts[0]
        axes[0].plot(xyz[:,0],xyz[:,1],lw=.8,label=label)
        axes[1].plot(ts,xyz[:,2],lw=.8,label=label)
        axes[2].plot(xyz[:,0],xyz[:,2],lw=.8,label=label)
    axes[0].set(xlabel='X (m)',ylabel='Y (m)');axes[1].set(xlabel='Elapsed (s)',ylabel='Camera Z (m)');axes[2].set(xlabel='X (m)',ylabel='Z (m)')
    for ax in axes: ax.grid(alpha=.2)
    axes[0].set_aspect('equal');axes[0].legend(fontsize=7)
    fig.suptitle(name+' | first-node gauge only; actual stair/grade Z preserved')
    fig.savefig(directory/'trajectories.png',dpi=160);plt.close(fig)


def plot_worst_wall(directory, name, rows, pairs, poses, connection, selected):
    candidates=[r for r in rows if r['label']==selected and r['kind']=='wall']
    if not candidates: return None
    worst=max(candidates,key=lambda r:r['symmetric_plane_abs_median_m'])
    item=next(p for p in pairs if p[0]==worst['from_id'] and p[1]==worst['to_id'] and p[3]['kind']=='wall' and p[3]['plane_index']==worst['from_plane'])
    a,b,_,pa,pb,_=item
    fig,axes=plt.subplots(2,3,figsize=(15,9),constrained_layout=True)
    for j,label in enumerate(('raw','app','independent')):
        transform=inverse(poses[label][a])@poses[label][b]
        # Wall-normal / vertical section reveals duplicated parallel wall faces.
        normal=pa['normal']; up=poses['raw'][a][:3,:3].T@np.array([0.,0.,1.])
        aa=pa['testing'];bb=apply(pb['testing'],transform)
        axes[0,j].scatter(aa@normal,aa@up,s=2,alpha=.7,label=f'frame {a}')
        axes[0,j].scatter(bb@normal,bb@up,s=2,alpha=.7,label=f'frame {b}')
        metric=pair_metrics(pa,pb,poses[label][a],poses[label][b])
        axes[0,j].set(title=f'{label}: plane median {metric["symmetric_plane_abs_median_m"]*100:.1f} cm',xlabel='Wall-normal coordinate (m)',ylabel='Up coordinate (m)')
        axes[0,j].legend(fontsize=8);axes[0,j].grid(alpha=.2)
    ranges=[ax.get_xlim() for ax in axes[0]];yranges=[ax.get_ylim() for ax in axes[0]]
    for ax in axes[0]:ax.set_xlim(min(x[0] for x in ranges),max(x[1] for x in ranges));ax.set_ylim(min(x[0] for x in yranges),max(x[1] for x in yranges))
    for ax,node in zip(axes[1,:2],(a,b)):
        blob=connection.execute('SELECT image FROM Data WHERE id=?',(node,)).fetchone()[0]
        rgb=cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_COLOR)
        ax.imshow(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB));ax.set_title(f'Original RGB frame {node}');ax.axis('off')
    axes[1,2].axis('off');axes[1,2].text(0,.9,'Finite wall patch diagnostic\n\nSame local observations for all poses.\nNative loop establishes tentative correspondence.\nGraph reserve: '+str(worst['graph_reserved'])+'\n\nNot surveyed wall truth;\nwrong loop matches remain possible.',va='top',fontsize=11)
    fig.suptitle(name+' | largest selected-baseline wall discrepancy')
    fig.savefig(directory/'worst_wall.png',dpi=160);plt.close(fig)
    return worst


def audit_session(name,entry,output):
    directory=output/name;directory.mkdir(parents=True)
    database=Path(entry['database']).resolve(); reviewed=Path(entry['reviewed_dir'])
    before=sha256_file(database)
    paths={'raw':reviewed/'raw_main_trajectory.csv','app':reviewed.parent/'audit'/name/'app_optimized_trajectory.csv','independent':reviewed/'optimized_main_trajectory.csv'}
    candidates=Path('outputs/student_union_20261017_revalidated/intra_session')/name
    for label in ('nominal','weak','attitude_2deg','attitude_5deg'):
        path=candidates/('candidate_'+label)/'main_trajectory.csv'
        if path.exists():paths['revalidated_'+label]=path
    poses={k:load_pose_csv(v) for k,v in paths.items()};raw=poses['raw']; ids=set(raw)
    gauges={}
    for label,p in poses.items():
        if ids-set(p):raise ValueError(f'{name} {label}: missing main nodes')
        poses[label],anchor,gauge=anchor_align(raw,{n:p[n] for n in sorted(ids)})
        gauges[label]={'anchor':anchor,'left_transform':gauge.tolist()}
    connection=sqlite3.connect(f'file:{database}?mode=ro',uri=True)
    stamps=dict(connection.execute('SELECT id,stamp FROM Node'))
    with (reviewed/'loop_validation.csv').open() as stream:
        loops=[(int(r['from_id']),int(r['to_id']),r['held_out']=='True') for r in csv.DictReader(stream) if r['accepted']=='True']
    loops=[(a,b,h) for a,b,h in loops if a in ids and b in ids]
    clouds=Clouds(connection,None)
    print(name+': finite surface extraction',flush=True)
    patch_pairs=surface_pairs(connection,clouds,raw,loops)
    metrics={};geometry=[];surfaces=[]
    for label,p in poses.items():
        reserve,rr=geometry_metrics([(a,b) for a,b,h in loops if h],p,clouds)
        allmetric,ar=geometry_metrics([(a,b) for a,b,h in loops],p,clouds)
        geometry += [{'label':label,'scope':'selection_reserve',**r} for r in rr]
        geometry += [{'label':label,'scope':'all_prior_accepted',**r} for r in ar]
        rows=[]
        for a,b,h,pa,pb,match in patch_pairs:
            rows.append({'label':label,'from_id':a,'to_id':b,'graph_reserved':h,'kind':pa['kind'],'from_plane':pa['plane_index'],'to_plane':pb['plane_index'],**match,**pair_metrics(pa,pb,p[a],p[b])})
        surfaces+=rows
        metrics[label]={'graph_reserve_geometry':reserve,'all_accepted_geometry':allmetric,
            'wall_plane_abs_m':aggregate([r for r in rows if r['kind']=='wall'],'symmetric_plane_abs_median_m'),
            'horizontal_plane_abs_m':aggregate([r for r in rows if r['kind']=='horizontal'],'symmetric_plane_abs_median_m'),
            'wall_normal_difference_deg':aggregate([r for r in rows if r['kind']=='wall'],'normal_difference_degrees'),
            'camera_z_range_m':float(np.ptp([t[2,3] for t in p.values()]))}
    selected,reason=select_baseline(metrics)
    frozen=output/'frozen'/(name+'_poses.csv');trajectory(frozen,poses[selected],stamps)
    cal=calibration_audit(connection,sorted(ids)[::max(1,len(ids)//24)])
    plot_clouds(directory,name,poses,clouds,stamps,selected)
    worst=plot_worst_wall(directory,name,surfaces,patch_pairs,poses,connection,selected)
    write_csv(directory/'geometry_pairs.csv',geometry);write_csv(directory/'surface_pairs.csv',surfaces)
    with (directory/'calibration.json').open('w') as stream:json.dump(cal,stream,indent=2)
    connection.close();after=sha256_file(database)
    if before!=after:raise RuntimeError('Source DB changed')
    record={'session':name,'selected_label':selected,'selected_pose_csv':str(frozen.resolve()),'reason':reason,
        'frozen_pose_sha256':sha256_file(frozen),'selection_status':'PROVISIONAL_BASELINE_FOR_FURTHER_OPTIMIZATION_NOT_VALIDATED_MAP',
        'main_nodes':len(ids),'database':str(database),'database_sha256':before,'database_unchanged':True,
        'metrics':metrics,'candidate_sources':{k:{'path':str(p.resolve()),'sha256':sha256_file(p)} for k,p in paths.items()},
        'gauge_alignment':gauges,'calibration_summary':{k:v for k,v in cal.items() if k!='records'},
        'worst_selected_wall':worst,
        'worst_selected_horizontal':sorted([r for r in surfaces if r['label']==selected and r['kind']=='horizontal'],key=lambda r:r['symmetric_plane_abs_median_m'],reverse=True)[:5],
        'selected_worst_geometry':sorted([r for r in geometry if r['label']==selected and r['scope']=='all_prior_accepted'],key=lambda r:r['clipped_rmse_m'],reverse=True)[:8],
        'figures':[str(p.resolve()) for p in directory.glob('*.png')]}
    (directory/'report.json').write_text(json.dumps(record,indent=2)+'\n')
    print(name+': selected '+selected+'; '+reason,flush=True)
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path('config/student_union_all_sessions.json'))
    parser.add_argument('--output',type=Path,default=Path('outputs/student_union_20261017_geometry_review/baselines'))
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Use a fresh directory')
    args.output.mkdir(parents=True)
    config=json.loads(args.config.read_text())['sessions']
    report={'created_utc':utc_now(),'source_sha256':sha256_file(Path(__file__)),'argv':sys.argv,
        'algorithms':['Native float32 depth decode and confidence>=50; per-frame scaled camera intrinsics and native local extrinsic.',
            'First-common-node SE(3) gauge alignment only; no best-fit trajectory alignment.',
            'Bidirectional nearest-neighbor reserved-voxel cloud distances clipped at 20cm.',
            'Training-only PCA normals, finite-patch 3-point RANSAC+SVD, held-out plane checks.',
            'Tentative one-to-one surface correspondence from prior accepted native loop measurements.',
            'Predeclared baseline selection: >=3 reserve pairs, >=1cm median gain, overlap nonregression 0.03 and wall p90 nonregression 2cm. Otherwise app baseline.'],
        'selection_not_final_validation':True,
        'limitations':['Prior reserves are reused model-selection data, not new final validation. App optimization may have used these loop pairs.',
            'Local point and plane residuals depend on tentative native loop correspondence; no surveyed truth or physical floor labels.',
            'Adjacent frames and spatial samples are correlated. Calibration consistency does not prove calibration accuracy.',
            'No cross-session fitting, global flattening, measurement scaling, or RGB-D source edits.'],
        'thread_environment':{k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},'sessions':{}}
    for name,entry in config.items():
        report['sessions'][name]=audit_session(name,entry,args.output)
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    report['finished_utc']=utc_now()
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
