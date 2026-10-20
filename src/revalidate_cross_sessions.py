#!/usr/bin/env python3
"""Strict frame-level RGB-D evidence and independent cross-session proposals.

This program never merges clouds or edits a DB. Locally verified measurements
remain available even when global trajectory drift prevents session consensus.
"""
from __future__ import annotations

import argparse
from collections import Counter, OrderedDict
import csv
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import sys
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import check_session_overlap as checker
from export_rtabmap_db import load_pose_csv


SESSION_DATABASES = {
    'outside':'261016-234854-outside.db', 'lobby':'261017-000622-1f.db',
    'bridge_out1f':'261017-011331-bridgeout1f.db', 'floor2':'261017-013501-2f.db',
    'bridge_12_sub':'261017-013904-bridg12_2.db',
}
PRIORITY_PAIRS = [('outside','bridge_out1f'),('lobby','bridge_out1f'),
                  ('lobby','floor2'),('lobby','bridge_12_sub'),('floor2','bridge_12_sub')]
EXTRA_PARAMETERS = {'opencv_threads':2,'reprojection_median_max_px':6.,
                    'reprojection_p90_max_px':15.,'forward_reverse_retrieval':True,
                    'minimum_independent_support':3,'pairwise_consistency_required':True}
FALLBACK_PARAMETERS = {'clahe_clip_limit':2.,'clahe_tile_grid':[8,8],
                       'valid_depth_mask_erosion_pixels':1,
                       'retrieval_ranking':'mutual_ratio_count_over_all_bidirectional_ANN_proposals',
                       'geometry_gates':'unchanged_strict_canonical'}


def atomic_json(path, payload):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(payload,indent=2,ensure_ascii=False),encoding='utf-8')
    temporary.replace(path)


class CachedSession(checker.Session):
    """Cache only deterministic low-resolution descriptors, keyed by DB/source."""
    def __init__(self,path,label,cache_root,strategy='canonical'):
        super().__init__(path,label)
        self.strategy=strategy
        self.low_features=OrderedDict()
        self.db_fingerprint=checker.fingerprint(path)
        feature_keys=('retrieval_max_side','retrieval_features','sift_octave_layers',
                      'sift_contrast_threshold','sift_edge_threshold','sift_sigma')
        specification={'database_sha256':self.db_fingerprint['sha256'],
                       'parameters':{k:checker.DEFAULTS[k] for k in feature_keys},
                       'opencv':cv2.__version__,
                       'checker_source_sha256':checker.fingerprint(Path(checker.__file__))['sha256'],
                       'semantic_masks':False,'strategy':strategy,
                       'fallback_parameters':FALLBACK_PARAMETERS if strategy=='clahe_depth' else None}
        cache_key=hashlib.sha256(json.dumps(specification,sort_keys=True).encode()).hexdigest()[:20]
        self.cache=cache_root/(label+'_'+cache_key)
        self.cache.mkdir(parents=True,exist_ok=True)
        atomic_json(self.cache/'manifest.json',specification)

    def feature(self,node,full=False):
        if full and self.strategy=='canonical':
            return super().feature(node,full=True)
        if full:
            key=(node,full)
            if key not in self.features:
                if len(self.features)>=64:
                    self.features.pop(next(iter(self.features)))
                self.features[key]=self.enhanced_feature(node,full=True)
            return self.features[key]
        if node in self.low_features:
            self.low_features.move_to_end(node)
            return self.low_features[node]
        path=self.cache/f'{node}.npz'
        if path.exists():
            with np.load(path,allow_pickle=False) as saved:
                descriptors=saved['descriptors'].copy()
                scale=float(saved['scale'])
            # Retrieval only consumes descriptors, never low-res keypoints.
        else:
            _,descriptors,scale=(self.enhanced_feature(node) if self.strategy=='clahe_depth' else super().feature(node,full=False))
            np.savez_compressed(path,descriptors=descriptors,scale=scale)
        value=([],descriptors,scale)
        if len(self.low_features)>=256:
            self.low_features.popitem(last=False)
        self.low_features[node]=value
        return value

    def enhanced_feature(self,node,full=False):
        image=self.image(node)
        depth,confidence,*_=self.depth(node)
        feature_mask=valid_depth_feature_mask(depth,confidence,image.shape)
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        scale=1. if full else checker.DEFAULTS['retrieval_max_side']/max(image.shape[:2])
        if scale!=1.:
            gray=cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
            feature_mask=cv2.resize(feature_mask,(gray.shape[1],gray.shape[0]),interpolation=cv2.INTER_NEAREST)
        gray=cv2.createCLAHE(clipLimit=FALLBACK_PARAMETERS['clahe_clip_limit'],
                             tileGridSize=tuple(FALLBACK_PARAMETERS['clahe_tile_grid'])).apply(gray)
        detector=cv2.SIFT_create(nfeatures=checker.DEFAULTS['verification_features'] if full else checker.DEFAULTS['retrieval_features'],
                nOctaveLayers=checker.DEFAULTS['sift_octave_layers'],contrastThreshold=checker.DEFAULTS['sift_contrast_threshold'],
                edgeThreshold=checker.DEFAULTS['sift_edge_threshold'],sigma=checker.DEFAULTS['sift_sigma'])
        points,descriptors=detector.detectAndCompute(gray,feature_mask)
        return points,checker.root_sift(descriptors),scale

    def depth(self,node):
        if node not in self.geometry and len(self.geometry)>=48:
            self.geometry.pop(next(iter(self.geometry)))
        return super().depth(node)


def valid_depth_feature_mask(depth,confidence,image_shape):
    """Detection support only: never alter stored depth or its confidence."""
    valid=np.isfinite(depth)&(depth>=checker.DEFAULTS['min_depth_m'])&(depth<=checker.DEFAULTS['max_depth_m'])&(
        confidence>=checker.DEFAULTS['min_confidence'])
    mask=cv2.erode(valid.astype(np.uint8)*255,np.ones((3,3),np.uint8),iterations=1)
    return cv2.resize(mask,(image_shape[1],image_shape[0]),interpolation=cv2.INTER_NEAREST)


def pose_sources(workspace,name,session):
    reviewed=workspace/'outputs'/('student_union_20261017_reviewed' if name in ('outside','lobby') else 'student_union_20261017_added_reviewed')
    paths={'optimized':reviewed/name/'optimized_main_trajectory.csv',
           'app':reviewed/'audit'/name/'app_optimized_trajectory.csv',
           'raw':reviewed/name/'raw_main_trajectory.csv'}
    poses={}
    for label,path in paths.items():
        if not path.exists():
            raise FileNotFoundError(path)
        poses[label]=load_pose_csv(path)
    common=set.intersection(*(set(p) for p in poses.values()))
    if common!=set(poses['raw']):
        raise ValueError(f'Pose-source main-component coverage differs for {name}')
    return poses,{label:checker.fingerprint(path) for label,path in paths.items()}


def bidirectional_retrieval(first,second,output,limit,strategy='canonical'):
    forward_dir=output/'retrieval_a_to_b';forward_dir.mkdir(parents=True,exist_ok=True)
    reverse_dir=output/'retrieval_b_to_a';reverse_dir.mkdir(parents=True,exist_ok=True)
    forward,nf,ndf=checker.retrieval(first,second,forward_dir)
    reverse,nr,ndr=checker.retrieval(second,first,reverse_dir)
    if strategy=='clahe_depth':
        def full_candidates(directory):
            with (directory/'retrieval_candidates.csv').open() as stream:
                return [{'outside_id':int(row['outside_id']),'lobby_id':int(row['lobby_id']),
                         'retrieval_score':float(row['retrieval_score'])} for row in csv.DictReader(stream)]
        forward,reverse=full_candidates(forward_dir),full_candidates(reverse_dir)
    combined={}
    for candidate in forward:
        row=dict(candidate);row['retrieval_direction']='a_to_b'
        combined[(row['outside_id'],row['lobby_id'])]=row
    for candidate in reverse:
        row={'outside_id':candidate['lobby_id'],'lobby_id':candidate['outside_id'],
             'retrieval_score':candidate['retrieval_score'],'retrieval_direction':'b_to_a'}
        key=(row['outside_id'],row['lobby_id'])
        if key not in combined or combined[key]['retrieval_score']<row['retrieval_score']:
            combined[key]=row
    ranked=sorted(combined.values(),key=lambda row:row['retrieval_score'],reverse=True)
    if strategy=='clahe_depth':
        for index,row in enumerate(ranked):
            _,da,_=first.feature(row['outside_id']);_,db,_=second.feature(row['lobby_id'])
            row['retrieval_mutual_matches']=len(checker.mutual_ratio_matches(da,db,checker.DEFAULTS['lowe_ratio']))
            if index%500==0:
                print(f'rerank {first.label}/{second.label}: {index}/{len(ranked)}',flush=True)
        ranked.sort(key=lambda row:(row['retrieval_mutual_matches'],row['retrieval_score']),reverse=True)
        checker.write_csv(output/'mutual_reranked_candidates.csv',ranked)
    selected=[]
    for candidate in ranked:
        a,b=candidate['outside_id'],candidate['lobby_id']
        if any(abs(first.stamps[a]-first.stamps[c['outside_id']])<checker.DEFAULTS['candidate_temporal_nms_s'] and
               abs(second.stamps[b]-second.stamps[c['lobby_id']])<checker.DEFAULTS['candidate_temporal_nms_s'] for c in selected):
            continue
        selected.append(candidate)
        if len(selected)>=limit:
            break
    return selected,{'forward_ranked':nf,'reverse_ranked':nr,'forward_index_descriptors':ndf,
                     'reverse_index_descriptors':ndr,'unique_selected_before_final_nms':len(ranked)}


def project_node_points(session,node,points):
    _,_,cw,ch,k,local=session.depth(node)
    image=session.image(node)
    optical=checker.apply_transform(np.linalg.inv(local),points)
    uv=np.full((len(points),2),np.nan)
    valid=optical[:,2]>.01
    uv[valid]=optical[valid,:2]/optical[valid,2,None]
    uv[:,0]=uv[:,0]*k[0,0]+k[0,2]
    uv[:,1]=uv[:,1]*k[1,1]+k[1,2]
    return uv*np.array([image.shape[1]/cw,image.shape[0]/ch])


def reprojection_check(first,second,result):
    a,b=result['outside_id'],result['lobby_id']
    ka,da,_=first.feature(a,full=True);kb,db,_=second.feature(b,full=True)
    matches=checker.mutual_ratio_matches(da,db,checker.DEFAULTS['lowe_ratio'])
    pixa=np.array([ka[m.queryIdx].pt for m in matches]);pixb=np.array([kb[m.trainIdx].pt for m in matches])
    target,source=first.unproject(a,pixa),second.unproject(b,pixb)
    valid=np.isfinite(target).all(axis=1)&np.isfinite(source).all(axis=1)
    target,source,pixa,pixb=target[valid],source[valid],pixa[valid],pixb[valid]
    seed=result['ransac_seed']
    indices=np.random.default_rng(seed).permutation(len(source))
    held=indices[:max(5,int(np.ceil(len(source)*checker.DEFAULTS['heldout_fraction'])))]
    transform=np.asarray(result['lobby_node_to_outside_node'])
    pa=project_node_points(first,a,checker.apply_transform(transform,source[held]))
    pb=project_node_points(second,b,checker.apply_transform(np.linalg.inv(transform),target[held]))
    errors=np.concatenate((np.linalg.norm(pa-pixa[held],axis=1),np.linalg.norm(pb-pixb[held],axis=1)))
    finite=bool(np.isfinite(errors).all())
    metrics={'heldout_count':len(held),'both_directions_finite':finite,
             'median_px':float(np.median(errors)) if finite else None,
             'p90_px':float(np.percentile(errors,90)) if finite else None,
             'rmse_px':float(np.sqrt(np.mean(errors**2))) if finite else None,
             'note':'Feature correspondence holdout; dense ICP can include these pixels.'}
    accepted=(finite and metrics['median_px']<=EXTRA_PARAMETERS['reprojection_median_max_px'] and
              metrics['p90_px']<=EXTRA_PARAMETERS['reprojection_p90_max_px'])
    result['heldout_reprojection']=metrics
    if not accepted:
        result['accepted']=False
        result['reason']='additional_heldout_reprojection_check_failed'
    return result


def verify_planar_disjoint(first,second,candidate,output):
    """Method-correction variant: observable planar 3D matches, disjoint holdout.

    No map/stored transform initializes either RANSAC or ICP. The geometric
    residual, overlap and reprojection gates remain the canonical values.
    """
    import revalidate_loop_constraints as intra
    result=dict(candidate,accepted=False,reason='not_evaluated',geometry_variant='planar_disjoint')
    a,b=candidate['outside_id'],candidate['lobby_id']
    ka,da,_=first.feature(a,True);kb,db,_=second.feature(b,True)
    matches=checker.mutual_ratio_matches(da,db,checker.DEFAULTS['lowe_ratio'])
    result['mutual_matches']=len(matches)
    ia,ib=first.image(a),second.image(b)
    display=cv2.drawMatches(ia,ka,ib,kb,matches[:40],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(output/'matches'/f'{a:04d}_{b:04d}.jpg'),cv2.resize(display,(720,480)))
    if len(matches)<checker.DEFAULTS['minimum_mutual_matches']:
        result['reason']='too_few_mutual_visual_matches';return result
    pixa=np.array([ka[m.queryIdx].pt for m in matches]);pixb=np.array([kb[m.trainIdx].pt for m in matches])
    target,source=first.unproject(a,pixa),second.unproject(b,pixb)
    valid=np.isfinite(target).all(axis=1)&np.isfinite(source).all(axis=1)
    target,source,pixa,pixb=target[valid],source[valid],pixa[valid],pixb[valid]
    result['valid_depth_matches']=len(source)
    if len(source)<checker.DEFAULTS['minimum_depth_matches']:
        result['reason']='too_few_valid_depth_matches';return result
    seed=checker.DEFAULTS['seed']+a*10007+b
    train,held,masks=intra.split_correspondences(intra.depth_pixels(first,a,pixa),intra.depth_pixels(second,b,pixb),
                                                first.depth(a)[0].shape,second.depth(b)[0].shape,seed)
    result.update(ransac_seed=seed,train_count=len(train),heldout_count=len(held),
        discarded_duplicate_or_holdout_neighbours=len(source)-len(train)-len(held),
        excluded_depth_pixels=[int(m.sum()) for m in masks],sample_holdout_scope='3-pixel depth neighbourhoods excluded from training and both dense ICP clouds; nearby samples remain correlated')
    evidence=output/'correspondences'/f'{a}_{b}.npz';evidence.parent.mkdir(exist_ok=True)
    np.savez_compressed(evidence,target=target,source=source,rgb_pixels_a=pixa,rgb_pixels_b=pixb,train_indices=train,heldout_indices=held)
    result['correspondence_evidence']=str(evidence.resolve())
    if len(train)<checker.DEFAULTS['minimum_train_inliers'] or len(held)<5:
        result['reason']='too_few_disjoint_training_or_heldout_matches';return result
    visual,inliers,trials=checker.rigid_ransac(source[train],target[train],seed)
    result.update(ransac_valid_trials=trials,train_inliers=int(inliers.sum()),train_inlier_ratio=float(inliers.mean()),visual_T_a_b=visual.tolist())
    if inliers.sum()<checker.DEFAULTS['minimum_train_inliers'] or inliers.mean()<checker.DEFAULTS['minimum_train_inlier_ratio']:
        result['reason']='insufficient_3d_ransac_consensus';return result
    supported=[]
    for points,pixels,shape,label in ((target[train][inliers],pixa[train][inliers],ia.shape,'outside'),
                                    (source[train][inliers],pixb[train][inliers],ib.shape,'lobby')):
        accepted,diagnostic=intra.planar_aware_support(points,pixels,shape)
        result[label+'_geometry']=diagnostic;supported.append(accepted)
    if not all(supported):
        result['reason']='ill_conditioned_or_narrow_visual_support';return result
    ca,cb=intra.cloud_without_holdout(first,a,masks[0]),intra.cloud_without_holdout(second,b,masks[1])
    result['outside_cloud_points'],result['lobby_cloud_points']=len(ca),len(cb)
    if min(len(ca),len(cb))<100:
        result['reason']='insufficient_holdout_excluded_cloud';return result
    forward,fm=checker.refine_icp(ca,cb,visual)
    reverse,rm=checker.refine_icp(cb,ca,np.linalg.inv(visual))
    cycle=intra.delta_summary(np.eye(4),forward@reverse)
    held_score=intra.holdout_score(forward,source[held],target[held])
    reverse_score=intra.holdout_score(reverse,target[held],source[held])
    result.update(icp=fm,reverse_icp=rm,icp_cycle=cycle,heldout_rmse_m=held_score['rmse_m'],
                  heldout_inliers=held_score['inliers'],heldout_inlier_ratio=held_score['inlier_ratio'],
                  reverse_heldout=reverse_score,lobby_node_to_outside_node=forward.tolist(),T_b_a_independent_icp=reverse.tolist())
    pa=project_node_points(first,a,checker.apply_transform(forward,source[held]))
    pb=project_node_points(second,b,checker.apply_transform(reverse,target[held]))
    errors=np.concatenate((np.linalg.norm(pa-pixa[held],axis=1),np.linalg.norm(pb-pixb[held],axis=1)))
    finite=bool(np.isfinite(errors).all())
    result['heldout_reprojection']={'both_directions_finite':finite,'heldout_count':len(held),
        'median_px':float(np.median(errors)) if finite else None,'p90_px':float(np.percentile(errors,90)) if finite else None,
        'note':'Both independent ICP directions; holdout depth neighbourhoods excluded.'}
    failures=[]
    for name,score in [('forward',held_score),('reverse',reverse_score)]:
        if score['inliers']<5 or score['inlier_ratio']<.5 or score['rmse_m']>.07:
            failures.append(name+'_heldout_correspondence_failed')
    for name,score in [('forward',fm),('reverse',rm)]:
        if score['symmetric_overlap']<.4 or score['symmetric_rmse_m']>.07 or score['refinement_m']>.15 or score['refinement_deg']>5:
            failures.append(name+'_icp_validation_failed')
    if cycle['translation_m']>.10 or cycle['rotation_deg']>2:
        failures.append('bidirectional_icp_cycle_failed')
    if not finite or np.median(errors)>6 or np.percentile(errors,90)>15:
        failures.append('heldout_reprojection_failed')
    result.update(accepted=not failures,reason=';'.join(failures) if failures else 'pair_geometry_verified_planar_disjoint')
    result.update(intra.covariance_model())
    return result


def clusters_across_pose_sources(results,first,second,poses_a,poses_b):
    """Keep local evidence independent of whether a global-pose hypothesis agrees."""
    proposals=[]
    available=[dict(row) for row in results if row.get('accepted') and
               row['outside_id'] in poses_a['raw'] and row['lobby_id'] in poses_b['raw']]
    for label in ('optimized','app','raw'):
        remaining=list(available)
        while remaining:
            cluster,enriched=checker.independent_consensus(remaining,first,second,poses_a[label],poses_b[label])
            if len(cluster)<EXTRA_PARAMETERS['minimum_independent_support']:
                break
            ids={(c['outside_id'],c['lobby_id']) for c in cluster}
            proposals.append({'pose_source':label,'independent_support_count':len(cluster),
                              'T_session_a_session_b':cluster[0]['lobby_map_to_outside_map'],
                              'independent_pairs':[list(pair) for pair in sorted(ids)],
                              'support_details':cluster})
            remaining=[r for r in remaining if (r['outside_id'],r['lobby_id']) not in ids]
    return proposals


def edge_records(results,proposals,name_a,name_b,pose_sources_a,pose_sources_b):
    records=[]
    support={}
    selected_index=(max(range(len(proposals)),key=lambda i:proposals[i]['independent_support_count']) if proposals else None)
    for index,proposal in enumerate(proposals):
        proposal['selected_for_joint']=index==selected_index
        independent=sorted(tuple(pair) for pair in proposal['independent_pairs'])
        held_out=set(independent[3::4]) if len(independent)>=4 else set()
        if len(independent)-len(held_out)<3:
            held_out=set()
        training=[pair for pair in independent if pair not in held_out]
        proposal['training_pairs']=[list(pair) for pair in training]
        proposal['held_out_pairs']=[list(pair) for pair in sorted(held_out)]
        proposal['whole_link_holdout_available']=bool(held_out)
        # Use a training measurement only for initialization. Consensus gating
        # used all local evidence, but held-out edges never initialize the solver.
        details={(c['outside_id'],c['lobby_id']):c for c in proposal.get('support_details',[])}
        if training and training[0] in details:
            proposal['T_session_a_session_b']=details[training[0]]['lobby_map_to_outside_map']
            proposal['initialization_pair']=list(training[0])
        for pair in proposal['independent_pairs']:
            support.setdefault(tuple(pair),[]).append(index)
    for result in results:
        if not result.get('accepted'):
            continue
        a,b=result['outside_id'],result['lobby_id']
        membership=support.get((a,b),[])
        in_main=a in pose_sources_a['raw'] and b in pose_sources_b['raw']
        independent_selected=selected_index in membership if selected_index is not None else False
        held_out=bool(independent_selected and [a,b] in proposals[selected_index]['held_out_pairs'])
        records.append({'edge_id':f'{name_a}:{a}__{name_b}:{b}',
                        'session_a':name_a,'node_a':a,'session_b':name_b,'node_b':b,
                        'T_a_b':result['lobby_node_to_outside_node'],
                        'transform_convention':'T_a_b maps node B coordinates into node A coordinates',
                        'local_geometry_verified':True,'nodes_in_main_components':in_main,
                        'eligible_for_joint':bool(independent_selected and in_main),
                        'independent_support_member':independent_selected,
                        'joint_role':('heldout' if held_out else 'train') if independent_selected and in_main else 'local_evidence_only',
                        'joint_holdout':held_out,
                        'held_out':held_out,
                        'initialization_allowed':bool(independent_selected and in_main and not held_out),
                        'session_consistency_cluster_ids':membership,
                        'independent_support_count':max((proposals[i]['independent_support_count'] for i in membership),default=0),
                        'metrics':{k:v for k,v in result.items() if k not in ('lobby_node_to_outside_node',)},
                        'limitation':'Eligibility is geometric evidence, not ground truth; robust joint optimization and separate validation still required.'})
    return records


def drift_hypothesis_records(records,first,second,poses_a,poses_b,visual_reviewed=False):
    """Exploratory graph input, NEVER a verified rigid session alignment.

    Relax only current-global-pose coherence, not local RGB-D gates. At least
    four independent measured links and a whole-link holdout are mandatory.
    """
    candidates=[]
    for row in records:
        a,b=row['node_a'],row['node_b']
        if not row.get('local_geometry_verified') or a not in poses_a or b not in poses_b:
            continue
        transform=poses_a[a]@np.asarray(row['T_a_b'])@np.linalg.inv(poses_b[b])
        tilt=float(np.degrees(np.arccos(np.clip(transform[2,2],-1,1))))
        if tilt<=10:
            candidates.append((row,transform,tilt))
    best=[]
    # Deterministic greedy search from every anchor; pairwise bounds prevent
    # single-linkage chaining between repeated structures or different floors.
    for anchor in candidates:
        selected=[]
        others=sorted(candidates,key=lambda value:(value[0]['node_a'],value[0]['node_b']))
        for item in [anchor]+[v for v in others if v is not anchor]:
            row,transform,_=item;a,b=row['node_a'],row['node_b']
            independent=True
            for old,old_t,_ in selected:
                delta=np.linalg.inv(old_t)@transform
                if (abs(first.stamps[a]-first.stamps[old['node_a']])<8 or
                    abs(second.stamps[b]-second.stamps[old['node_b']])<8 or
                    np.linalg.norm(poses_a[a][:3,3]-poses_a[old['node_a']][:3,3])<.75 or
                    np.linalg.norm(poses_b[b][:3,3]-poses_b[old['node_b']][:3,3])<.75 or
                    np.linalg.norm(delta[:3,3])>1.5 or
                    np.degrees(Rotation.from_matrix(delta[:3,:3]).magnitude())>10):
                    independent=False;break
            if independent:selected.append(item)
        if len(selected)>len(best):best=selected
    summary={'independent_support_count':len(best),'eligible':len(best)>=4 and visual_reviewed,
        'visual_review_passed':visual_reviewed,'coherence_max_m':1.5,'coherence_max_deg':10.,
        'minimum_time_separation_each_session_s':8.,'minimum_position_separation_each_session_m':.75,
        'minimum_independent_links':4,'current_pose_source':'optimized',
        'scientific_status':'candidate_only_for_guarded_joint_optimization_not_verified_rigid_merge',
        'selection_caveat':'Whole-link holdout excluded from initialization/PGO, but participates in support/coherence selection; not a fully blinded benchmark.'}
    if len(best)<4 or not visual_reviewed:
        summary['reason']='insufficient_independent_support_or_visual_review'
        return [],summary
    best.sort(key=lambda item:(item[0]['node_a'],item[0]['node_b']))
    errors=[]
    for index,(_,t,_) in enumerate(best):
        for _,u,_ in best[index+1:]:
            d=np.linalg.inv(t)@u
            errors.append([float(np.linalg.norm(d[:3,3])),float(np.degrees(Rotation.from_matrix(d[:3,:3]).magnitude()))])
    summary['pairwise_coherence_median_m_deg']=np.median(errors,axis=0).tolist()
    summary['pairwise_coherence_max_m_deg']=np.max(errors,axis=0).tolist()
    held_indices=set(range(3,len(best),4))
    result=[]
    for index,(row,transform,tilt) in enumerate(best):
        copied=dict(row);held=index in held_indices
        copied.update(eligible_for_joint=False,hypothesis_eligible=True,independent_support_member=True,
                      independent_support_count=len(best),joint_holdout=held,held_out=held,
                      joint_role='heldout' if held else 'train',initialization_allowed=not held,
                      hypothesis_kind='within_session_drift_correction_candidate',
                      hypothesis_coherence=summary,session_transform_diagnostic=transform.tolist(),
                      session_gravity_disagreement_deg=tilt)
        result.append(copied)
    summary['independent_pairs']=[[r['node_a'],r['node_b']] for r in result]
    summary['training_pairs']=[[r['node_a'],r['node_b']] for r in result if not r['held_out']]
    summary['held_out_pairs']=[[r['node_a'],r['node_b']] for r in result if r['held_out']]
    return result,summary


def expand_drift_hypothesis(backbone,local_records,poses_a,poses_b):
    """Optional correlated secondary constraints, quarter information by sigma.

    The existing whole-link holdout and independent backbone are immutable.
    Secondary links cannot initialize a session and are not independent proof.
    """
    if len(backbone)<4 or not any(row.get('held_out') for row in backbone):
        raise ValueError('An independent backbone with whole-link holdout is required')
    result=[dict(row) for row in backbone]
    used={(r['node_a'],r['node_b']) for r in backbone}
    transforms=[poses_a[r['node_a']]@np.asarray(r['T_a_b'])@np.linalg.inv(poses_b[r['node_b']]) for r in backbone]
    support_count=min(r['independent_support_count'] for r in backbone)
    for row in local_records:
        a,b=row['node_a'],row['node_b']
        if (a,b) in used or not row.get('local_geometry_verified') or a not in poses_a or b not in poses_b:
            continue
        transform=poses_a[a]@np.asarray(row['T_a_b'])@np.linalg.inv(poses_b[b])
        if np.degrees(np.arccos(np.clip(transform[2,2],-1,1)))>10:
            continue
        differences=[np.linalg.inv(t)@transform for t in transforms]
        if any(np.linalg.norm(d[:3,3])>1.5 or np.degrees(Rotation.from_matrix(d[:3,:3]).magnitude())>10 for d in differences):
            continue
        copied=dict(row)
        copied.update(eligible_for_joint=False,hypothesis_eligible=True,independent_support_member=False,
            independent_support_count=support_count,joint_holdout=False,held_out=False,
            joint_role='train',initialization_allowed=False,hypothesis_kind='correlated_secondary_drift_constraint',
            correlation_information_scale=.25,sigma_translation_m=.20,sigma_rotation_deg=6.,
            correlation_weight_note='Quarter information relative to nominal0.10m/3deg, applied through doubled sigma; do not apply factor twice.',
            hypothesis_coherence=backbone[0]['hypothesis_coherence'])
        result.append(copied);used.add((a,b))
    return result


def parse_pair(value):
    fields=value.split(':')
    if len(fields)!=2 or fields[0]==fields[1] or any(name not in SESSION_DATABASES for name in fields):
        raise argparse.ArgumentTypeError('Pair must be two different known session names joined by a colon')
    return tuple(fields)


def export_drift_experiment(args,workspace):
    if not args.visual_review_note:
        raise ValueError('--visual-review-note is required for a reviewed hypothesis export')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if (out/'drift_hypothesis_edges.json').exists():
        raise FileExistsError('Choose a fresh drift-experiment output; prior evidence is preserved')
    records={};sources=[]
    for path in args.export_drift_from:
        sources.append(checker.fingerprint(path))
        for row in json.loads(path.read_text()):
            # Preserve the first verified measurement rather than cherry-picking
            # residuals; variants remain separately available in original reports.
            records.setdefault(row['edge_id'],dict(row,evidence_source=str(path.resolve())))
    values=list(records.values())
    pair={(r['session_a'],r['session_b']) for r in values}
    if len(pair)!=1:raise ValueError('A drift experiment must contain exactly one oriented session pair')
    name_a,name_b=next(iter(pair))
    first=checker.Session(workspace/'data/raw/rtabmap'/SESSION_DATABASES[name_a],name_a)
    second=checker.Session(workspace/'data/raw/rtabmap'/SESSION_DATABASES[name_b],name_b)
    pa,ma=pose_sources(workspace,name_a,first);pb,mb=pose_sources(workspace,name_b,second)
    if args.drift_backbone:
        backbone=json.loads(args.drift_backbone.read_text());sources.append(checker.fingerprint(args.drift_backbone))
        rows=expand_drift_hypothesis(backbone,values,pa['optimized'],pb['optimized'])
        report={'status':'expanded_hypothesis_not_verified_merge','backbone_holdout_preserved':True}
    else:
        rows,report=drift_hypothesis_records(values,first,second,pa['optimized'],pb['optimized'],True)
    report.update(total_edges=len(rows),independent_support_members=sum(r['independent_support_member'] for r in rows),
        whole_link_holdouts=[[r['node_a'],r['node_b']] for r in rows if r['held_out']],
        evidence_sources=sources,visual_review_note=args.visual_review_note,
        pose_sources={name_a:ma,name_b:mb},generator_source=checker.fingerprint(Path(__file__)),
        command=shlex.join([sys.executable,*sys.argv]),cwd=str(workspace),
        secondary_weight_note='Quarter information via sigma0.20m/6deg; do not apply scale twice',
        selection_caveat='Support selection used all measurements; whole-link holdout not used by graph or initialization.')
    atomic_json(out/'drift_hypothesis_edges.json',rows);atomic_json(out/'report.json',report)
    shutil.copy2(Path(__file__),out/'source_snapshot.py')
    print(json.dumps(report,indent=2))
    first.connection.close();second.connection.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/student_union_20261017_revalidated/cross_session'))
    parser.add_argument('--candidate-limit',type=int,default=160)
    parser.add_argument('--strategy',choices=('canonical','clahe_depth'),default='canonical')
    parser.add_argument('--geometry',choices=('canonical','planar_disjoint'),default='canonical')
    parser.add_argument('--candidate-source',type=Path,help='Reuse only candidate selection from a previous pass; all geometry is recomputed')
    parser.add_argument('--export-drift-from',nargs='+',type=Path,help='Export a separate exploratory hypothesis from verified frame-edge files')
    parser.add_argument('--drift-backbone',type=Path,help='Preserve these independent support members and whole-link holdout')
    parser.add_argument('--visual-review-note',help='Record explicit review of the distinctive shared scene, not only repetitive stairs')
    parser.add_argument('--pairs',nargs='+',type=parse_pair)
    parser.add_argument('--all-pairs',action='store_true')
    parser.add_argument('--resume',action='store_true',help='Reuse completed/checkpoint pair verification only after exact provenance matches')
    args=parser.parse_args()
    if args.candidate_limit<1:
        parser.error('--candidate-limit must be positive')
    workspace=Path(__file__).resolve().parents[1]
    os.chdir(workspace)
    if args.export_drift_from:
        export_drift_experiment(args,workspace)
        return
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(EXTRA_PARAMETERS['opencv_threads'])
    checker.DEFAULTS['candidate_limit']=args.candidate_limit
    pairs=args.pairs or list(PRIORITY_PAIRS)
    if args.all_pairs:
        names=list(SESSION_DATABASES)
        for i,a in enumerate(names):
            for b in names[i+1:]:
                if (a,b) not in pairs and (b,a) not in pairs:
                    pairs.append((a,b))
    source_hashes={name:checker.fingerprint(path)['sha256'] for name,path in
                   [('driver',Path(__file__)),('checker',Path(checker.__file__)),
                    ('calibration',workspace/'src/export_rtabmap_db.py')]}
    snapshot=args.output/'source_snapshot.py'
    if not snapshot.exists():
        shutil.copy2(Path(__file__),snapshot)
    if args.geometry=='planar_disjoint':
        import revalidate_loop_constraints as intra
        source_hashes['planar_holdout_helper']=checker.fingerprint(Path(intra.__file__))['sha256']
    sessions={};pose_sets={};pose_metadata={}
    for name in sorted({name for pair in pairs for name in pair}):
        cache_root=(args.candidate_source/'feature_cache') if args.candidate_source else (args.output/'feature_cache')
        session=CachedSession(workspace/'data/raw/rtabmap'/SESSION_DATABASES[name],name,cache_root,args.strategy)
        sessions[name]=session
        pose_sets[name],pose_metadata[name]=pose_sources(workspace,name,session)
    aggregate_edges=[];aggregate_proposals=[];pair_summaries=[]
    for name_a,name_b in pairs:
        start=time.monotonic();first,second=sessions[name_a],sessions[name_b]
        output=args.output/(name_a+'__'+name_b);output.mkdir(parents=True,exist_ok=True)
        (output/'matches').mkdir(exist_ok=True)
        provenance={'inputs':[first.db_fingerprint,second.db_fingerprint],
                    'parameters':dict(checker.DEFAULTS),'additional_parameters':EXTRA_PARAMETERS,
                    'feature_strategy':args.strategy,
                    'fallback_parameters':FALLBACK_PARAMETERS if args.strategy=='clahe_depth' else None,
                    'geometry_variant':args.geometry,
                    'geometry_variant_parameters':intra.SETTINGS if args.geometry=='planar_disjoint' else None,
                    'source_hashes':source_hashes,'semantic_masks':False}
        selection=None
        if args.candidate_source:
            selection_path=args.candidate_source/(name_a+'__'+name_b)/'verification_checkpoint.json'
            selection=json.loads(selection_path.read_text())
            if len(selection['pair_results'])!=len(selection['selected_candidates']):
                raise ValueError('Candidate source must be a completed pass, not a changing checkpoint')
            if selection['provenance']['inputs']!=provenance['inputs'] or selection['provenance'].get('feature_strategy','canonical')!=args.strategy:
                raise ValueError('Candidate selection DB/feature strategy mismatch')
            provenance['candidate_selection_source']=checker.fingerprint(selection_path)
        checkpoint_path=output/'verification_checkpoint.json'
        old=None
        if checkpoint_path.exists():
            if not args.resume:
                raise FileExistsError(f'{checkpoint_path} exists; choose fresh output or --resume')
            old=json.loads(checkpoint_path.read_text())
            if old['provenance']!=provenance:
                raise ValueError('Checkpoint source/input/parameter provenance mismatch; use a fresh output')
        if old:
            selected,counts,results=old['selected_candidates'],old['retrieval_counts'],old['pair_results']
        else:
            if selection:
                selected=selection['selected_candidates'][:args.candidate_limit];counts=selection['retrieval_counts']
            else:
                selected,counts=bidirectional_retrieval(first,second,output,args.candidate_limit,args.strategy)
            results=[]
        for index,candidate in enumerate(selected[len(results):],start=len(results)):
            result=(verify_planar_disjoint(first,second,candidate,output) if args.geometry=='planar_disjoint'
                    else checker.verify_pair(first,second,candidate,output))
            if result['accepted'] and args.geometry=='canonical':
                result=reprojection_check(first,second,result)
            result['session_a']=name_a;result['session_b']=name_b
            results.append(result)
            print(f'{name_a}/{name_b} {index+1}/{len(selected)} {result["outside_id"]}/{result["lobby_id"]} {result["reason"]}',flush=True)
            atomic_json(checkpoint_path,{'provenance':provenance,'selected_candidates':selected,
                                        'retrieval_counts':counts,'pair_results':results})
        proposals=clusters_across_pose_sources(results,first,second,pose_sets[name_a],pose_sets[name_b])
        edges=edge_records(results,proposals,name_a,name_b,pose_sets[name_a],pose_sets[name_b])
        for index,proposal in enumerate(proposals):
            proposal.update(session_a=name_a,session_b=name_b,cluster_id=index,
                            proposal_id=f'{name_a}__{name_b}:{index}')
        summary={'session_a':name_a,'session_b':name_b,'provenance':provenance,
                 'pose_sources':{name_a:pose_metadata[name_a],name_b:pose_metadata[name_b]},
                 'command':shlex.join([sys.executable,*sys.argv]),'cwd':str(workspace),
                 'retrieval_counts':counts,'checked_pairs':len(results),
                 'local_geometry_verified_pairs':len(edges),
                 'eligible_for_joint_pairs':sum(e['eligible_for_joint'] for e in edges),
                 'independent_session_proposals':len(proposals),
                 'failure_counts':dict(Counter(r['reason'] for r in results if not r['accepted'])),
                 'stage_counts':{'descriptor_verification':len(results),
                                 'depth_correspondence':sum('valid_depth_matches' in r for r in results),
                                 'ransac':sum('ransac_valid_trials' in r for r in results),
                                 'icp':sum('icp' in r for r in results),
                                 'reprojection':sum('heldout_reprojection' in r for r in results)},
                 'elapsed_seconds':time.monotonic()-start,
                 'limitations':['No map merge or joint optimization in this program.',
                                'Local verified measurements are retained even when session-global trajectories disagree.',
                                'Session proposals require three independent supports; appearance alone is insufficient.',
                                'Bounded appearance retrieval can miss physical overlap.',
                                'No semantic masks; no independent surveyed ground truth.',
                                ('Held-out feature correspondences can occur in dense ICP clouds.' if args.geometry=='canonical' else
                                 'Held-out depth neighbourhoods excluded from both dense ICP clouds; spatial sensor samples remain correlated.'),
                                'Whole-link holdouts participate in support selection but not initialization or graph optimization.']}
        atomic_json(output/'report.json',summary)
        atomic_json(output/'accepted_frame_edges.json',edges)
        atomic_json(output/'session_alignment_proposals.json',proposals)
        checker.write_csv(output/'candidate_results.csv',results)
        checker.write_csv(output/'rejected_pairs.csv',[r for r in results if not r['accepted']])
        checker.candidate_sheet(results,output)
        aggregate_edges.extend(edges);aggregate_proposals.extend(proposals);pair_summaries.append(summary)
        atomic_json(args.output/'accepted_frame_edges.json',aggregate_edges)
        atomic_json(args.output/'session_alignment_proposals.json',aggregate_proposals)
        atomic_json(args.output/'summary.json',{'status':'complete' if len(pair_summaries)==len(pairs) else 'in_progress',
                    'requested_pairs':pairs,'completed_pairs':len(pair_summaries),'pair_summaries':pair_summaries,
                    'accepted_frame_edges':len(aggregate_edges),'eligible_for_joint_edges':sum(e['eligible_for_joint'] for e in aggregate_edges),
                    'cross_session_map_merge_performed':False})
        print(json.dumps({k:summary[k] for k in ('session_a','session_b','checked_pairs','local_geometry_verified_pairs',
                                                'eligible_for_joint_pairs','failure_counts')}),flush=True)
    for session in sessions.values():
        session.connection.close()


if __name__=='__main__':
    main()
