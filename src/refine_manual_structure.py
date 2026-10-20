#!/usr/bin/env python3
"""Fit manually identified structures using asymmetric depth and tracked views.

Original assistant annotations are uncertain development labels, not surveyed
landmarks. Within-capture camera poses stay fixed. Only the cross-capture SE(3)
and shared 3-D landmarks are optimized; complete withheld frames are scored
after fitting. Several views of one door remain ONE physical region.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from check_session_overlap import Session, rigid_fit
from export_rtabmap_db import load_pose_csv
from optimize_session import Clouds, apply, geometric_score, inverse, sha256_file, utc_now
from evaluate_submap_candidates import fixed_patches
from recover_geometry_connections import MANUAL_ANNOTATIONS, intrinsics_at_rgb
from refine_geometry_surfaces import reduce_voxels, render_evidence
from revalidate_loop_constraints import SPECS, delta_summary

ROOT = Path(__file__).resolve().parents[1]
SETTINGS = {'train_offsets': [-3,-2,-1,0,1,2,3], 'test_offsets': [-6,-5,5,6],
            'lk_window': 31, 'lk_pyramid_levels': 4, 'lk_forward_backward_max_px': 2.,
            'depth_sigma_m': .06, 'pixel_sigma_px': 5., 'huber_scale': 1.,
            'maximum_evaluations': 1000, 'maximum_translation_delta_m': 1.,
            'maximum_rotation_delta_deg': 15., 'seed': 20261017,
            'minimum_shared_RGB_landmarks': 4, 'sensitivity_trials': 12,
            'depth_policy': 'strict', 'maximum_prior_tracking_error_px': 60.,
            'minimum_patch_correlation': .60, 'landmark_bound_m': 10.,
            'plane_distance_sigma_m': .08, 'plane_normal_sigma_deg': 3.,
            'finite_surface_factors': []}


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists(): raise FileExistsError(path)
    path.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')


def gray(image):
    return cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8)).apply(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY))


def project(points, world_from_optical, intrinsic):
    optical = apply(points, inverse(world_from_optical))
    z = optical[:,2]
    pixels = optical[:,:2]/np.maximum(z[:,None],.05)
    pixels = pixels @ intrinsic[:2,:2].T + intrinsic[:2,2]
    return pixels, z


def sample_depth(session,node,pixels,soft=False):
    """Keep uncertain edge depth weak; confidence=0 never becomes metric data."""
    if not soft:
        return session.unproject(node,pixels),np.full(len(pixels),SETTINGS['depth_sigma_m'])
    depth,confidence,cw,ch,k,local=session.depth(node)
    ih,iw=session.image(node).shape[:2];dh,dw=depth.shape
    result=np.full((len(pixels),3),np.nan);sigma=np.full(len(pixels),np.nan)
    for j,pixel in enumerate(pixels):
        u,v=np.rint(np.array(pixel)*[dw/iw,dh/ih]).astype(int)
        if not (1<=u<dw-1 and 1<=v<dh-1):continue
        patch=depth[v-1:v+2,u-1:u+2];conf=confidence[v-1:v+2,u-1:u+2]
        good=np.isfinite(patch)&(patch>=.25)&(patch<=5.)&(conf>=50)
        if good.sum()<3:continue
        spread=float(np.ptp(patch[good]))
        if spread>.60:continue
        z=float(np.median(patch[good]));uv=np.array(pixel)*[cw/iw,ch/ih]
        optical=np.array([(uv[0]-k[0,2])*z/k[0,0],(uv[1]-k[1,2])*z/k[1,1],z])
        result[j]=apply(optical[None],local)[0]
        sigma[j]=SETTINGS['depth_sigma_m']+.5*spread
    return result,sigma


def depth_threshold_sweep(first,second,annotation):
    diagnostics={};counts=[]
    for side,ss in [('a',first),('b',second)]:
        node=annotation['node_'+side];depth,conf,*_=ss.depth(node);ih,iw=ss.image(node).shape[:2]
        rows=[]
        for name,pa,pb in annotation['landmarks']:
            p=pa if side=='a' else pb;u,v=np.rint(np.array(p)*[depth.shape[1]/iw,depth.shape[0]/ih]).astype(int)
            patch=depth[v-1:v+2,u-1:u+2];cc=conf[v-1:v+2,u-1:u+2]
            good=np.isfinite(patch)&(patch>=.25)&(patch<=5.)&(cc>=50)
            rows.append({'name':name,'valid_confident_pixels':int(good.sum()),
                         'range_m':float(np.ptp(patch[good])) if good.any() else None,
                         'center_confidence':int(conf[v,u]),'center_depth_m':float(depth[v,u])})
        diagnostics[side]=rows
    for threshold in [.06,.12,.13,.25,.40,.60]:
        valid={side:[r['valid_confident_pixels']>=5 and r['range_m'] is not None and r['range_m']<=threshold
                     for r in rows] for side,rows in diagnostics.items()}
        counts.append({'range_gate_m':threshold,'minimum_valid_pixels':5,
                       'valid_a':sum(valid['a']),'valid_b':sum(valid['b']),
                       'valid_both':sum(a and b for a,b in zip(valid['a'],valid['b']))})
    return {'unchanged_confidence_minimum':50,'per_corner':diagnostics,'sweep':counts,
            'warning':'A wider patch range may mix foreground/background surfaces; more accepted depth is not evidence of better depth.'}


def track_observations(session, center, pixels, poses, side):
    reference = gray(session.image(center))
    p0 = np.asarray(pixels,np.float32).reshape(-1,1,2)
    reference_points,_=sample_depth(session,center,pixels,soft=True)
    rows=[]; tracking=[]
    for offset in sorted(set(SETTINGS['train_offsets']+SETTINGS['test_offsets'])):
        node=center+offset
        if node not in poses or node not in session.ids: continue
        image=session.image(node)
        if offset==0:
            pp=p0[:,0].copy();valid=np.ones(len(pp),bool);fb=np.zeros(len(pp))
        else:
            current=gray(image)
            pp,st,_=cv2.calcOpticalFlowPyrLK(reference,current,p0,None,
                winSize=(SETTINGS['lk_window'],)*2,maxLevel=SETTINGS['lk_pyramid_levels'],
                criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,50,.01))
            back,bs,_=cv2.calcOpticalFlowPyrLK(current,reference,pp,None,
                winSize=(SETTINGS['lk_window'],)*2,maxLevel=SETTINGS['lk_pyramid_levels'],
                criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,50,.01))
            fb=np.linalg.norm(back[:,0]-p0[:,0],axis=1)
            pp=pp[:,0]
            valid=(st.ravel()>0)&(bs.ravel()>0)&np.isfinite(pp).all(1)&(fb<SETTINGS['lk_forward_backward_max_px'])
            valid &= (pp[:,0]>=5)&(pp[:,0]<image.shape[1]-5)&(pp[:,1]>=5)&(pp[:,1]<image.shape[0]-5)
            # Reject a different similar-looking corner even if LK cycles back.
            intrinsic,local=intrinsics_at_rgb(session,node)
            relative=inverse(poses[center])@poses[node]
            known=np.isfinite(reference_points).all(1)
            if known.any():
                predicted,z=project(reference_points[known],relative@local,intrinsic)
                valid[known] &= (z>0)&(np.linalg.norm(predicted-pp[known],axis=1)<SETTINGS['maximum_prior_tracking_error_px'])
            for j in np.flatnonzero(valid):
                original=cv2.getRectSubPix(reference,(15,15),tuple(map(float,p0[j,0]))).astype(float)
                moved=cv2.getRectSubPix(current,(15,15),tuple(map(float,pp[j]))).astype(float)
                correlation=np.corrcoef(original.ravel(),moved.ravel())[0,1] if min(original.std(),moved.std())>1 else 0.
                if not np.isfinite(correlation) or correlation<SETTINGS['minimum_patch_correlation']:valid[j]=False
        points,depth_sigma=sample_depth(session,node,pp,soft=SETTINGS['depth_policy']=='soft')
        intrinsic,local=intrinsics_at_rgb(session,node)
        relative=inverse(poses[center])@poses[node]
        optical_from_node=inverse(local)
        depth=apply(points,optical_from_node)[:,2]
        for k in range(len(pp)):
            tracking.append({'side':side,'node':node,'landmark':k,'valid':bool(valid[k]),
                             'forward_backward_px':float(fb[k]),'pixel':pp[k].tolist(),
                             'role':'train' if offset in SETTINGS['train_offsets'] else 'test'})
            if not valid[k]: continue
            good_depth=np.isfinite(points[k]).all()
            rows.append({'side':side,'node':node,'landmark':k,'pixel':pp[k].tolist(),
                         'role':'train' if offset in SETTINGS['train_offsets'] else 'test',
                         'intrinsic':intrinsic.tolist(), 'center_from_optical':(relative@local).tolist(),
                         'center_point':apply(points[k:k+1],relative)[0].tolist() if good_depth else None,
                         'depth_z':float(depth[k]) if good_depth else None,
                         'depth_sigma_m':float(depth_sigma[k]) if good_depth else None,
                         'pixel_sigma_px':SETTINGS['pixel_sigma_px'],
                         'original_manual_seed':offset==0})
    return rows, tracking


def make_initial_landmarks(observations, transform, count, annotation, first, second):
    initial=[]
    for k in range(count):
        points=[]
        for r in observations:
            if r['landmark']!=k or r['center_point'] is None:continue
            point=np.array(r['center_point'])
            points.append(point if r['side']=='a' else apply(point[None],transform)[0])
        if points:
            initial.append(np.median(points,axis=0));continue
        ka,la=intrinsics_at_rgb(first,annotation['node_a'])
        kb,lb=intrinsics_at_rgb(second,annotation['node_b'])
        pa=ka@inverse(la)[:3];pb=kb@(inverse(lb)@inverse(transform))[:3]
        ua=np.array(annotation['landmarks'][k][1],float).reshape(2,1)
        ub=np.array(annotation['landmarks'][k][2],float).reshape(2,1)
        h=cv2.triangulatePoints(pa,pb,ua,ub)[:,0]
        point=h[:3]/h[3] if abs(h[3])>1e-9 else np.full(3,np.nan)
        _,z=project(point[None],la,ka)
        if not np.isfinite(point).all() or np.linalg.norm(point)>30 or z[0]<=0:
            ray=np.linalg.solve(ka,np.r_[ua.ravel(),1.])*3.
            point=apply(ray[None],la)[0]
        initial.append(point)
    return np.array(initial)


def make_problem(observations, transform, landmarks):
    counts={}
    for r in observations:
        key=(r['side'],r['landmark']);counts[key]=counts.get(key,0)+1
    seed=np.r_[np.zeros(6),landmarks.ravel()]
    def decode(x):
        correction=np.eye(4);correction[:3,:3]=Rotation.from_rotvec(x[3:6]).as_matrix()
        correction[:3,3]=x[:3]
        return correction@transform,x[6:].reshape(-1,3)
    def residual(x):
        t,points=decode(x);rr=[]
        for r in observations:
            point=points[r['landmark']][None]
            if r['side']=='b':point=apply(point,inverse(t))
            predicted,z=project(point,np.array(r['center_from_optical']),np.array(r['intrinsic']))
            weight=1/np.sqrt(counts[r['side'],r['landmark']])
            rr.extend(weight*(predicted[0]-r['pixel'])/r.get('pixel_sigma_px',SETTINGS['pixel_sigma_px']))
            if r['depth_z'] is not None:
                rr.append(weight*(z[0]-r['depth_z'])/r.get('depth_sigma_m',SETTINGS['depth_sigma_m']))
            rr.append(weight*max(0.,.1-z[0])/.02)
        for surface in SETTINGS['finite_surface_factors']:
            ca,cb=np.array(surface['center_a']),np.array(surface['center_b'])
            na,nb=np.array(surface['normal_a']),np.array(surface['normal_b'])
            moved=apply(cb[None],t)[0];normal=t[:3,:3]@nb
            sign=1 if na@normal>=0 else -1
            rr.append(float(na@(moved-ca))/SETTINGS['plane_distance_sigma_m'])
            rr.extend(np.cross(na,sign*normal)/np.radians(SETTINGS['plane_normal_sigma_deg']))
        return np.array(rr)
    bound=np.r_[np.full(3,SETTINGS['maximum_translation_delta_m']),
                np.full(3,np.radians(SETTINGS['maximum_rotation_delta_deg'])),np.full(landmarks.size,SETTINGS['landmark_bound_m'])]
    return seed,residual,decode,(seed-bound,seed+bound)


def fit_bundle(observations, transform, landmarks):
    x0,fun,decode,bounds=make_problem(observations,transform,landmarks)
    fit=least_squares(fun,x0,bounds=bounds,loss='huber',f_scale=SETTINGS['huber_scale'],
                      max_nfev=SETTINGS['maximum_evaluations'],x_scale='jac',
                      ftol=1e-9,xtol=1e-9,gtol=1e-9)
    t,points=decode(fit.x)
    # Eliminate nuisance landmarks from the weighted data Jacobian. No pose
    # prior is inserted to make the rank appear full.
    j=fit.jac;jp,jl=j[:,:6],j[:,6:]
    projected=jp-jl@np.linalg.lstsq(jl,jp,rcond=1e-10)[0]
    singular=np.linalg.svd(projected,compute_uv=False)
    rank=int(np.sum(singular>max(singular[0]*1e-6,1e-8)))
    initial_residual=fun(x0);scale=SETTINGS['huber_scale']
    huber=np.where(np.abs(initial_residual)<=scale,initial_residual**2,
                   2*scale*np.abs(initial_residual)-scale**2)
    return t,points,{'success':bool(fit.success),'message':fit.message,'nfev':int(fit.nfev),
                     'cost':float(fit.cost),'initial_cost':float(.5*np.sum(huber)),
                     'initial_quadratic_cost':float(.5*np.sum(initial_residual**2)),
                     'active_bounds':int(np.count_nonzero(fit.active_mask)),
                     'pose_data_rank_after_landmark_elimination':rank,
                     'pose_singular_values_mixed_parameter_units':singular.tolist(),
                     'rank_caution':'Robust weighted residual Jacobian in translation meters and rotation radians; conditioning is model dependent, not calibrated covariance.'}


def score(observations,transform,points):
    pixel=[];depth=[];behind=0
    for r in observations:
        point=points[r['landmark']][None]
        if r['side']=='b':point=apply(point,inverse(transform))
        predicted,z=project(point,np.array(r['center_from_optical']),np.array(r['intrinsic']))
        pixel.append(float(np.linalg.norm(predicted[0]-r['pixel'])))
        behind+=int(z[0]<=0)
        if r['depth_z'] is not None:depth.append(float(abs(z[0]-r['depth_z'])))
    return {'pixel_observations':len(pixel),'pixel_median_px':float(np.median(pixel)) if pixel else None,
            'pixel_p90_px':float(np.percentile(pixel,90)) if pixel else None,
            'valid_depth_observations':len(depth),'depth_median_m':float(np.median(depth)) if depth else None,
            'depth_p90_m':float(np.percentile(depth,90)) if depth else None,'behind_camera_count':behind}


def tracked_gallery(directory, annotation, first,second,observations,transform,points):
    for role in ('train','test'):
        frames=sorted(set((r['side'],r['node']) for r in observations if r['role']==role))
        if not frames:continue
        fig,axes=plt.subplots(2,int(np.ceil(len(frames)/2)),figsize=(3.1*np.ceil(len(frames)/2),8),squeeze=False)
        for ax,(side,node) in zip(axes.flat,frames):
            ss=first if side=='a' else second
            ax.imshow(cv2.cvtColor(ss.image(node),cv2.COLOR_BGR2RGB))
            for r in observations:
                if (r['side'],r['node'],r['role'])!=(side,node,role):continue
                point=points[r['landmark']][None]
                if side=='b':point=apply(point,inverse(transform))
                predicted,z=project(point,np.array(r['center_from_optical']),np.array(r['intrinsic']))
                observed=np.array(r['pixel']);p=predicted[0]
                ax.plot([observed[0],p[0]],[observed[1],p[1]],color='yellow',lw=.7)
                ax.scatter(*observed,s=15,facecolors='none',edgecolors='lime' if r['depth_z'] is not None else 'orange')
                if z[0]>0:ax.scatter(*p,s=10,c='cyan',marker='+')
                ax.text(*observed,str(r['landmark']+1),fontsize=7,color='white')
            ax.set_title(f'{ss.label} {node}',fontsize=9);ax.axis('off')
        for ax in list(axes.flat)[len(frames):]:ax.axis('off')
        fig.suptitle(f'{role.upper()}: manual seeds + bidirectional LK; circles=observed, cyan=projection\nGreen=valid depth, orange=RGB only; same door is one physical region')
        fig.tight_layout(rect=(0,0,1,.92));fig.savefig(directory/f'{role}_tracked_reprojection.png',dpi=150);plt.close(fig)


def run(annotation,output):
    sa,sb=annotation['session_a'],annotation['session_b'];a,b=annotation['node_a'],annotation['node_b']
    directory=output/f'{sa}_{a}__{sb}_{b}';directory.mkdir()
    databases={s:ROOT/'data/raw/rtabmap'/SPECS[s][0] for s in (sa,sb)}
    hashes={s:sha256_file(p) for s,p in databases.items()}
    first,second=Session(databases[sa],sa),Session(databases[sb],sb)
    paths={s:ROOT/'outputs'/SPECS[s][1]/'audit'/s/'app_optimized_trajectory.csv' for s in (sa,sb)}
    poses={s:load_pose_csv(p) for s,p in paths.items()}
    pp={s:load_pose_csv(ROOT/'outputs/student_union_20261017_all_preview/bundle'/f'{s}_trajectory.csv') for s in (sa,sb)}
    initial=inverse(pp[sa][a])@pp[sb][b]
    SETTINGS['finite_surface_factors']=[]
    if annotation.get('finite_surfaces'):
        ca,cb=Clouds(first.connection,None),Clouds(second.connection,None)
        patches_a=fixed_patches(ca,first.raw_poses,a);patches_b=fixed_patches(cb,second.raw_poses,b)
        for kind,ia,ib in annotation['finite_surfaces']:
            aa,bb=patches_a[kind,ia],patches_b[kind,ib]
            SETTINGS['finite_surface_factors'].append({'kind':kind,'plane_a':ia,'plane_b':ib,
                'center_a':aa['center'].tolist(),'center_b':bb['center'].tolist(),
                'normal_a':aa['normal'].tolist(),'normal_b':bb['normal'].tolist(),
                'training_points_a':len(aa['training']),'training_points_b':len(bb['training']),
                'source':'Deterministic raw-oriented RANSAC/SVD finite patches; only training samples form these normals/centers.',
                'place_identity':'Explicit inspected same grille wall and adjacent flat floor; development label, not surveyed truth.'})
    pa=np.array([r[1] for r in annotation['landmarks']],float);pb=np.array([r[2] for r in annotation['landmarks']],float)
    oa,ta=track_observations(first,a,pa,poses[sa],'a');ob,tb=track_observations(second,b,pb,poses[sb],'b')
    observations=oa+ob;train=[r for r in observations if r['role']=='train'];test=[r for r in observations if r['role']=='test']
    metric_ids=sorted({r['landmark'] for r in train if r['depth_z'] is not None})
    base={'annotation':annotation,'annotation_source':'Assistant visual pixel annotations; now explicitly used as development fitting data.',
          'settings':copy.deepcopy(SETTINGS),'observations':observations,'tracking_audit':ta+tb,
          'depth_threshold_sweep':depth_threshold_sweep(first,second,annotation),
          'training_unique_metric_landmarks':metric_ids,'training_observations':len(train),'test_observations':len(test),
          'physical_regions':1,'eligible_for_joint':False,'navigation':False,
          'within_session_pose_source':{s:{'path':str(p),'sha256':sha256_file(p)} for s,p in paths.items()},
          'T_a_b_incumbent':initial.tolist(), 'database_sha256_before':hashes,
          'algorithm':'Fixed within-session camera poses; shared structure landmarks and relative SE(3), Huber pixel+valid optical-depth residuals; optional finite-wall/floor normal and distance factors. Per-landmark/visit normalization for correlated repeated frames. No invented depths or dimensions.'}
    shared_ids=set(r['landmark'] for r in train if r['side']=='a') & set(r['landmark'] for r in train if r['side']=='b')
    base['shared_RGB_landmarks']=sorted(shared_ids)
    if len(shared_ids)<SETTINGS['minimum_shared_RGB_landmarks']:
        base.update(fit_executed=False,reason='fewer_than_four_shared_RGB_identities_for_full_pose_fit; retain as partial observations')
    else:
        landmarks=make_initial_landmarks(train,initial,len(pa),annotation,first,second)
        t,points,solver=fit_bundle(train,initial,landmarks)
        base.update(fit_executed=True,T_a_b=t.tolist(),landmarks_a=points.tolist(),solver=solver,
                    training_before=score(train,initial,landmarks),training_after=score(train,t,points),
                    test_before=score(test,initial,landmarks),test_after=score(test,t,points),
                    delta_from_incumbent=delta_summary(initial,t))
        sensitivity=[];rng=np.random.default_rng(SETTINGS['seed'])
        for k in range(SETTINGS['sensitivity_trials']):
            perturbed=copy.deepcopy(train)
            # One click uncertainty shift per landmark/visit, shared across all
            # tracked views; repeats do not magically divide seed uncertainty.
            shifts={(side,n):rng.normal(0,SETTINGS['pixel_sigma_px'],2) for side in ('a','b') for n in range(len(pa))}
            for r in perturbed:r['pixel']=(np.array(r['pixel'])+shifts[r['side'],r['landmark']]).tolist()
            st,_,sr=fit_bundle(perturbed,t,points)
            sensitivity.append({'trial':k,**delta_summary(t,st),'success':sr['success']})
        base['click_sensitivity']=sensitivity
        ca,cb=Clouds(first.connection,None),Clouds(second.connection,None)
        def window(clouds,poses,center):
            ns=[center+d for d in SETTINGS['test_offsets'] if center+d in poses]
            points=[apply(np.vstack(clouds.get(n)),inverse(poses[center])@poses[n]) for n in ns]
            return reduce_voxels(np.vstack(points)),ns
        target,ns_a=window(ca,poses[sa],a);source,ns_b=window(cb,poses[sb],b)
        base['withheld_cloud_frames']={'a':ns_a,'b':ns_b}
        base['withheld_cloud_before']=geometric_score(target,source,initial,.20)
        base['withheld_cloud_after']=geometric_score(target,source,t,.20)
        render_evidence(directory/'withheld_doorway_cloud.png',target,source,
                        {'incumbent':initial,'manual_structure':t},poses[sa][a][:3,:3],
                        f'{sa} {a} / {sb} {b}: tracked manual structure candidate')
        tracked_gallery(directory,annotation,first,second,observations,t,points)
        reasons=[]
        if not solver['success'] or solver['active_bounds']:reasons.append('numerical_failure_or_bound')
        if solver['pose_data_rank_after_landmark_elimination']<6:reasons.append('pose_data_rank_deficient')
        if base['training_after']['pixel_p90_px']>15.:reasons.append('training_reprojection_p90_above_15px')
        before,after=base['withheld_cloud_before'],base['withheld_cloud_after']
        if after['clipped_rmse_m']>before['clipped_rmse_m']+.005 or after['overlap']<before['overlap']-.03:
            reasons.append('withheld_cloud_regression')
        before,after=base['test_before'],base['test_after']
        if after['pixel_observations']<8 or after['pixel_p90_px']>15. or after['pixel_p90_px']>before['pixel_p90_px']+1.:
            reasons.append('withheld_reprojection_regression_or_missing')
        if max(r['translation_m'] for r in sensitivity)>.10 or max(r['rotation_deg'] for r in sensitivity)>3.:
            reasons.append('manual_click_sensitivity_above_10cm_or_3deg')
        base['local_support_passed']=not reasons;base['local_rejection_reasons']=reasons
        base['promotion_blocker']='One already inspected physical doorway; no spatially separated confirming cross-session regions. This fit cannot establish a globally connected map.'
    base['database_sha256_after']={s:sha256_file(p) for s,p in databases.items()}
    base['original_databases_unchanged']=base['database_sha256_after']==hashes
    first.connection.close();second.connection.close()
    if not base['original_databases_unchanged']:raise ValueError('original DB changed')
    save(directory/'report.json',base)
    return {k:base.get(k) for k in ('fit_executed','training_unique_metric_landmarks','training_observations','test_observations',
        'solver','test_before','test_after','withheld_cloud_before','withheld_cloud_after','delta_from_incumbent','local_support_passed','local_rejection_reasons','reason')}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--annotations',type=Path)
    p.add_argument('--depth-policy',choices=['strict','soft'],default='strict')
    p.add_argument('--center-only',action='store_true');a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    a.output.mkdir(parents=True);cv2.setNumThreads(1)
    SETTINGS['depth_policy']=a.depth_policy
    if a.center_only:SETTINGS['train_offsets']=[0]
    annotations=json.loads(a.annotations.read_text())['annotations'] if a.annotations else MANUAL_ANNOTATIONS
    results={}
    for annotation in annotations:
        SETTINGS['pixel_sigma_px']=float(annotation['uncertainty_pixels'])
        key=f'{annotation["session_a"]}_{annotation["node_a"]}__{annotation["session_b"]}_{annotation["node_b"]}'
        results[key]=run(annotation,a.output);print(json.dumps({key:results[key]}),flush=True)
    save(a.output/'report.json',{'created_utc':utc_now(),'source_sha256':sha256_file(Path(__file__)),
        'annotation_source_sha256':sha256_file(ROOT/'src/recover_geometry_connections.py'),
        'custom_annotations':str(a.annotations.resolve()) if a.annotations else None,
        'custom_annotations_sha256':sha256_file(a.annotations) if a.annotations else None,
        'settings':SETTINGS,'results':results,'cross_session_graph_modified':False,'eligible_for_joint':False})
    shutil_source=a.output/'source_snapshot.py'
    shutil_source.write_bytes(Path(__file__).read_bytes())


if __name__=='__main__':main()
