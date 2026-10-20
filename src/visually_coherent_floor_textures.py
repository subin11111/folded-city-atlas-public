#!/usr/bin/env python3
"""Photographic material plus spatially consistent illumination; visual QA required."""
import argparse
import json
from pathlib import Path
import shutil
import sqlite3
import warnings

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage

from blend_photographic_floors import grid_xy,multiband_pair,scalar_photo_gains,source_floor
from coherent_photo_charts import coherent_labels
from export_rtabmap_db import load_pose_csv
from flatten_horizontal_surfaces import view_samples
from level_floor_materials import replace_floor_references
from optimize_session import apply,sha256_file,utc_now
from presentation_geometry import LevelPresentation
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb
from surface_model_utils import decode_frame,project,select_frames,unproject

ROOT=Path(__file__).resolve().parents[1]


def local_floor_plane(points):
    if len(points)<100:return None
    sample=points[np.linspace(0,len(points)-1,min(1200,len(points))).astype(int)];rng=np.random.default_rng(24);best=None
    for _ in range(100):
        tri=sample[rng.choice(len(sample),3,replace=False)];n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);length=np.linalg.norm(n)
        if length<1e-7:continue
        n/=length
        if abs(n[2])<.985:continue
        count=int((abs((sample-tri[0])@n)<.045).sum())
        if best is None or count>best[0]:best=(count,tri[0],n)
    if best is None or best[0]<len(sample)*.5:return None
    used=abs((points-best[1])@best[2])<.065
    for _ in range(3):
        q=points[used];c=np.median(q,axis=0);_,_,vt=np.linalg.svd(q-c,full_matrices=False);n=vt[-1];used=abs((points-c)@n)<.06
    if used.sum()<100 or abs(n[2])<.985:return None
    return c,n


def prepare(con,node,spec,pose,session,model,group,cfg):
    frame=decode_frame(con,node,{**spec,'max_depth':5.5},pose);loose=decode_frame(con,node,{**spec,'confidence':0,'median_gate':0,'edge_gate':0,'max_depth':5.5},pose);frame['raw_depth']=loose['depth'];frame['session']=session
    xyz=apply(unproject(frame['depth'],frame['k_depth']).reshape(-1,3),frame['world_optical']).reshape((*frame['depth'].shape,3));n=np.cross(xyz[1:-1,2:]-xyz[1:-1,:-2],xyz[2:,1:-1]-xyz[:-2,1:-1]);n/=np.maximum(np.linalg.norm(n,axis=2)[:,:,None],1e-8);horizontal=np.zeros_like(frame['valid']);horizontal[1:-1,1:-1]=abs(n[:,:,2])>.8
    expected=source_floor(model,group,session,xyz[:,:,:2].reshape(-1,2)).reshape(frame['depth'].shape);good=horizontal&frame['valid']&(abs(xyz[:,:,2]-expected)<.5);plane=local_floor_plane(xyz[good]);frame['floor_plane']=plane
    if plane is not None:good=horizontal&frame['valid']&(abs((xyz-plane[0])@plane[1])<.07)
    frame['depth']=np.where(good,frame['depth'],0).astype(np.float32);poly=cfg['reviewed_floor_polygons'].get(f'{session}:{node}');frame['reviewed_floor']=None
    if poly:
        mask=np.zeros(frame['rgb'].shape[:2],np.uint8);cv2.fillPoly(mask,[np.asarray(poly,np.int32)],1);frame['reviewed_floor']=mask
    return frame


def sample(frame,xy,model,group,cfg):
    plane=frame['floor_plane']
    if plane is None:z=source_floor(model,group,frame['session'],xy)
    else:z=plane[0][2]-(xy-plane[0][:2])@plane[1][:2]/plane[1][2]
    points=np.c_[xy,z];rgb,weight=view_samples(frame,points,z,'floor',tolerance=cfg['depth_tolerance_m'])
    if frame['reviewed_floor'] is not None and plane is not None:
        uv,depth=project(points,frame['k_rgb'],frame['world_optical']);rh,rw=frame['rgb'].shape[:2];dh,dw=frame['depth'].shape;x,y=np.rint(np.clip(uv,-1e6,1e6)).astype(int).T;good=(x>=1)&(x<rw-1)&(y>=1)&(y<rh-1)&(depth>.3)&(depth<5.5);ids=np.flatnonzero(good);good[ids]&=frame['reviewed_floor'][y[ids],x[ids]]>0;dx=np.clip(np.rint(uv[:,0]*dw/rw),0,dw-1).astype(int);dy=np.clip(np.rint(uv[:,1]*dh/rh),0,dh-1).astype(int);raw=frame['raw_depth'][dy,dx];good&=(raw<=0)|(raw>=depth-.15);distance=np.linalg.norm(points-frame['world_optical'][:3,3],axis=1);incidence=abs(z-frame['world_optical'][2,3])/np.maximum(distance,.05);extra=np.where(good,.4*incidence**2/(1+.1*distance**2),0);weight=np.maximum(weight,extra)
    linear=srgb_to_linear(rgb);lum=linear@np.array([.2126,.7152,.0722]);weight*=np.clip((.95-lum)/.20,0,1);weight*=np.clip(lum/.015,0,1)
    return linear.astype(np.float32),weight.astype(np.float32)


def masked_gaussian(values,weight,sigma):
    return cv2.GaussianBlur(values*weight,(0,0),sigma)/np.maximum(cv2.GaussianBlur(weight,(0,0),sigma),1e-6)


def illumination_fields(images,weights,gains,pixel_m,cfg):
    lum=np.maximum(np.asarray(images)@np.array([.2126,.7152,.0722]),.003)*gains[:,None,None];valid=weights>.005
    with warnings.catch_warnings():
        warnings.simplefilter('ignore',RuntimeWarning);consensus=np.nanmedian(np.where(valid,np.log(lum),np.nan),axis=0)
    supported=np.isfinite(consensus);near=ndimage.distance_transform_edt(~supported,return_distances=False,return_indices=True);consensus=consensus[near[0],near[1]];sigma=max(1.2,cfg['illumination_radius_m']/pixel_m);target=masked_gaussian(consensus,supported.astype(np.float32),sigma);target=target[near[0],near[1]];fields=[];scores=[]
    for index in range(len(images)):
        own=masked_gaussian(np.log(lum[index]),valid[index].astype(np.float32),sigma);ratio=masked_gaussian(target-own,valid[index].astype(np.float32),sigma);ratio=np.clip(ratio,np.log(cfg['spatial_gain_bounds'][0]),np.log(cfg['spatial_gain_bounds'][1]));fields.append(np.exp(ratio)*gains[index]);residual=np.maximum(np.log(lum[index])-consensus-.2,0);scores.append(weights[index]*np.exp(-residual**2/.15))
    return np.asarray(fields),np.asarray(scores),np.exp(target)


def group_mosaic(group,charts,packs,selected,mp,model,cfg,dest):
    low=np.min([packs[c]['low'] for c in charts],axis=0);high=np.max([packs[c]['low']+np.array(packs[c]['mask'].shape[::-1])*packs[c]['cell'] for c in charts],axis=0);extent=high-low;cw,ch=np.maximum(8,np.rint(extent/extent.max()*cfg['coarse_side_px']).astype(int));xy=grid_xy(low,extent,cw,ch);candidates=[];databases={};poses={}
    for session in ['lobby','floor2','bridge_12_sub']:
        spec=selected['sessions'][session];poses[session]=load_pose_csv(mp.parent/spec['trajectory']);eligible=[n for n,p in poses[session].items() if .6<p[2,3]-source_floor(model,group,session,p[None,:2,3])[0]<2.8];nodes=select_frames(eligible,cfg['maximum_frames_per_session_layer'],cfg['preferred_nodes'].get(session,[]));databases[session]=selected['database_integrity'][session]['database']
        with sqlite3.connect(f'file:{databases[session]}?mode=ro',uri=True) as con:
            for node in nodes:
                frame=prepare(con,node,spec['preprocessing'],poses[session][node],session,model,group,cfg);rgb,weight=sample(frame,xy,model,group,cfg)
                if np.count_nonzero(weight)>20:candidates.append({'session':session,'node':node,'image':rgb.reshape(ch,cw,3),'weight':weight.reshape(ch,cw),'plane':None if frame['floor_plane'] is None else [v.tolist() for v in frame['floor_plane']]})
        print(f'Visual floor candidate {group}/{session}',flush=True)
    if not candidates:raise ValueError('No floor photographs')
    chosen=[];coverage=np.zeros((ch,cw),np.float32)
    for _ in range(min(cfg['maximum_selected_photos'],len(candidates))):
        index=int(np.argmax([(v['weight']/(1+coverage*5)).sum() if i not in chosen else -1 for i,v in enumerate(candidates)]));chosen.append(index);coverage+=candidates[index]['weight']>0
    views=[candidates[i] for i in chosen];images=[v['image'] for v in views];weights=np.asarray([v['weight'] for v in views]);gains,edges=scalar_photo_gains(images,weights,cfg['gain_bounds']);fields,weights,target=illumination_fields(images,weights,gains,max(extent[0]/cw,extent[1]/ch),cfg);labels=coherent_labels(weights,step=2,potts=.65,iterations=8);w,h=np.minimum(cfg['maximum_texture_side_px'],np.maximum(8,np.rint(extent/cfg['metres_per_pixel']).astype(int)));fine_xy=grid_xy(low,extent,w,h);labels=cv2.resize(labels.astype(np.float32),(w,h),interpolation=cv2.INTER_NEAREST).astype(int);colour=np.zeros((h*w,3),np.float32);total=np.zeros(h*w,np.float32)
    for index,v in enumerate(views):
        with sqlite3.connect(f'file:{databases[v["session"]]}?mode=ro',uri=True) as con:frame=prepare(con,v['node'],selected['sessions'][v['session']]['preprocessing'],poses[v['session']][v['node']],v['session'],model,group,cfg)
        feather=cv2.GaussianBlur((labels==index).astype(np.float32),(0,0),6).ravel();ids=np.flatnonzero(feather>1e-4);rgb,score=sample(frame,fine_xy[ids],model,group,cfg);gain=cv2.resize(fields[index].astype(np.float32),(w,h),interpolation=cv2.INTER_LINEAR).ravel()[ids];alpha=feather[ids]*(score>0);colour[ids]+=rgb*gain[:,None]*alpha[:,None];total[ids]+=alpha
    good=total>1e-5;colour[good]/=total[good,None];near=ndimage.distance_transform_edt(~good.reshape(h,w),return_distances=False,return_indices=True);image=colour.reshape(h,w,3)[near[0],near[1]];target=cv2.resize(target.astype(np.float32),(w,h),interpolation=cv2.INTER_LINEAR)
    Image.fromarray(np.uint8(linear_to_srgb(np.clip(image,0,1))*255+.5)).save(dest/(group+'_photos.png'));np.savez_compressed(dest/(group+'_domain.npz'),valid=good.reshape(h,w),target=target,labels=labels,low=low,extent=extent)
    return image,good.reshape(h,w),target,low,extent,{'selected_photos':[{k:v[k] for k in ['session','node','plane']} for v in views],'method':'Same-position photo median illumination, smooth scalar fields, highlight downweighting, coherent source-photo labels; not mean-colour paint'}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',type=Path,default=ROOT/'config/visual_floor_illumination.json');parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();cfg=json.loads(args.config.read_text());base=ROOT/cfg['source_bundle'];run=json.loads((base/'manifest.json').read_text());texture_base=ROOT/cfg['texture_bundle'];pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text());model=LevelPresentation(ROOT,run['presentation_config']);args.output.mkdir(parents=True,exist_ok=False);dest=args.output/'floor_charts';dest.mkdir();packs={};source=ROOT/'outputs/student_union_topology_fixed_v2';inputs={str(p.resolve()):sha256_file(p) for p in [Path(__file__),args.config,base/'manifest.json',texture_base/'manifest.json',ROOT/'src/blend_photographic_floors.py',ROOT/'src/level_floor_materials.py']}
    for chart in cfg['charts']:
        path=source/chart['session']/(chart['id']+'_mask.npz');pack=np.load(path);packs[chart['id']]={k:pack[k].copy() for k in pack.files};inputs[str(path.resolve())]=sha256_file(path)
    groups={g:group_mosaic(g,c,packs,selected,mp,model,cfg,dest) for g,c in cfg['groups'].items()};reports=[];replacements={}
    for chart in cfg['charts']:
        identity=chart['id'];pack=packs[identity];old=texture_base/'floor_charts'/(identity+'.png');rgb=np.asarray(Image.open(old).convert('RGB'));original=srgb_to_linear(rgb/255.).astype(np.float32);h,w=rgb.shape[:2];xy=grid_xy(pack['low'],np.array(pack['mask'].shape[::-1])*pack['cell'],w,h);group=next(g for g,ids in cfg['groups'].items() if identity in ids);image,valid,target,low,extent,record=groups[group];uv=((xy-low)/extent*np.array(image.shape[1::-1])-.5).reshape(h,w,2).astype(np.float32);foreground=cv2.remap(image,uv[:,:,0],uv[:,:,1],cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE);alpha=cv2.remap(valid.astype(np.float32),uv[:,:,0],uv[:,:,1],cv2.INTER_NEAREST);target_luma=cv2.remap(target,uv[:,:,0],uv[:,:,1],cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE);surface=cv2.resize(pack['mask'].astype(np.uint8),(w,h),interpolation=cv2.INTER_NEAREST)>0;alpha*=surface;pixel_m=max(pack['mask'].shape[1]*pack['cell']/w,pack['mask'].shape[0]*pack['cell']/h);own=np.maximum(original@np.array([.2126,.7152,.0722]),.003);own_low=masked_gaussian(np.log(own),surface.astype(np.float32),max(2,.4/pixel_m));correction=np.exp(np.clip(np.log(np.maximum(target_luma,.003))-own_low,np.log(cfg['fallback_gain_bounds'][0]),np.log(cfg['fallback_gain_bounds'][1])));background=np.clip(original*correction[:,:,None],0,1);result=multiband_pair(foreground,background,alpha,cfg['multiband_levels']);result[~surface]=original[~surface];path=dest/(identity+'.png');Image.fromarray(np.uint8(linear_to_srgb(result)*255+.5)).save(path);digest=sha256_file(old);expected=next(v['new_texture_sha256'] for v in run['floor_material_reports'] if v['id']==identity);assert digest==expected;replacements[digest]=path;reports.append({**chart,'source_texture_sha256':digest,'new_texture_sha256':sha256_file(path),'uniform_colour_replacement':False,'projection':record});inputs[str(old.resolve())]=digest
    sessions={};cleanup={};old_cleanup=json.loads((base/'cleanup.json').read_text())['sessions']
    for name,session in run['sessions'].items():
        folder=args.output/name;folder.mkdir();shutil.copyfile(base/name/'clean_structural.ply',folder/'clean_structural.ply');hits=replace_floor_references(base/name/'clean_textured.glb',folder/'clean_textured.glb',replacements)
        for path in (base/name).iterdir():
            if path.is_file() and path.name not in ['clean_structural.ply','clean_textured.glb','finish_appended.glb','finish_retained.glb']:shutil.copyfile(path,folder/path.name)
        sessions[name]={**session,'floor_material_references_changed':hits};cleanup[name]={**old_cleanup[name],'ply_sha256':sha256_file(folder/'clean_structural.ply'),'glb_sha256':sha256_file(folder/'clean_textured.glb')}
        for path in [base/name/'clean_structural.ply',base/name/'clean_textured.glb']:inputs[str(path.resolve())]=sha256_file(path)
    for path in base.iterdir():
        if path.is_file() and path.name not in ['manifest.json','cleanup.json','geometry_audit.json','material_audit.json']:shutil.copyfile(path,args.output/path.name)
    manifest={**run,'created_utc':utc_now(),'sessions':sessions,'inputs_sha256':{**run['inputs_sha256'],**inputs},'visual_floor_illumination':True,'floor_material_source_manifest':str((base/'manifest.json').resolve()),'previous_material_manifest':str((base/'manifest.json').resolve()),'floor_material_config':cfg,'floor_material_reports':reports,'scope':cfg['scope']};(args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(args.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(args.output/'manifest.json')},indent=2)+'\n')
    for path,digest in manifest['inputs_sha256'].items():assert sha256_file(Path(path))==digest,'Input changed: '+path


if __name__=='__main__':main()
