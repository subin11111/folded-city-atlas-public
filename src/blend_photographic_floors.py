#!/usr/bin/env python3
"""Shared RGB-D floor photographs and multiband seams, without uniform paint."""
import argparse
import json
from pathlib import Path
import shutil
import sqlite3

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage

from coherent_photo_charts import coherent_labels
from export_rtabmap_db import load_pose_csv
from flatten_horizontal_surfaces import view_samples
from level_floor_materials import replace_floor_references
from optimize_session import apply, sha256_file, utc_now
from presentation_geometry import LevelPresentation
from reconcile_multilevel_surfaces import sample_lattice
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb
from surface_model_utils import decode_frame, select_frames, unproject

ROOT = Path(__file__).resolve().parents[1]


def multiband_pair(foreground, background, alpha, levels=5):
    """Blend photographic bands, never replace either input by a mean colour."""
    fg = [np.asarray(foreground, np.float32)]
    bg = [np.asarray(background, np.float32)]
    masks = [np.asarray(alpha, np.float32)]
    for _ in range(levels-1):
        if min(fg[-1].shape[:2]) < 4:
            break
        fg.append(cv2.pyrDown(fg[-1]))
        bg.append(cv2.pyrDown(bg[-1]))
        masks.append(cv2.pyrDown(masks[-1]))
    result = fg[-1]*masks[-1][:,:,None]+bg[-1]*(1-masks[-1][:,:,None])
    for i in range(len(fg)-2, -1, -1):
        shape = (fg[i].shape[1], fg[i].shape[0])
        first = fg[i]-cv2.pyrUp(fg[i+1], dstsize=shape)
        second = bg[i]-cv2.pyrUp(bg[i+1], dstsize=shape)
        result = cv2.pyrUp(result, dstsize=shape)+first*masks[i][:,:,None]+second*(1-masks[i][:,:,None])
    # Outside the narrow pyramid transition, preserve the background exactly.
    radius = 2**len(fg)*2
    far = ndimage.distance_transform_edt(alpha <= 0) > radius
    result[far] = background[far]
    return np.clip(result, 0, 1)


def scalar_photo_gains(images, weights, bounds):
    """Robust overlap log-luminance graph; one scalar, not a new RGB material."""
    count = len(images)
    luminance = [np.maximum(im@np.array([.2126,.7152,.0722]),.004) for im in images]
    rows, values, records = [], [], []
    for i in range(count):
        for j in range(i):
            good = (weights[i]>.015)&(weights[j]>.015)&(luminance[i]>.015)&(luminance[j]>.015)&(luminance[i]<.85)&(luminance[j]<.85)
            if good.sum() < 60:
                continue
            delta = np.log(luminance[i][good])-np.log(luminance[j][good])
            median = float(np.median(delta)); mad = float(np.median(abs(delta-median)))
            if mad > .35 or abs(median) > .9:
                continue
            row = np.zeros(count); row[i] = 1; row[j] = -1
            rows.append(row); values.append(-median)
            records.append({'photos':[i,j], 'samples':int(good.sum()), 'log_ratio':median, 'mad':mad})
    if not rows:
        return np.ones(count), records
    matrix = np.asarray(rows); target = np.asarray(values); weight = np.ones(len(rows))
    anchor = np.zeros((1,count)); anchor[0,int(np.argmax([w.sum() for w in weights]))] = 5
    for _ in range(6):
        root = np.sqrt(weight)
        design = np.r_[matrix*root[:,None], np.eye(count)*.15, anchor]
        rhs = np.r_[target*root, np.zeros(count+1)]
        offset = np.linalg.lstsq(design, rhs, rcond=None)[0]
        residual = matrix@offset-target
        weight = np.minimum(1., .08/np.maximum(abs(residual),1e-8))
    return np.exp(np.clip(offset, np.log(bounds[0]), np.log(bounds[1]))), records


def source_floor(model, group, session, xy):
    key = 'upper' if group == 'upper' else ('sub_lower' if session == 'bridge_12_sub' else 'lower')
    return sample_lattice(model.models[key], xy)


def sample_floor(frame, xy, model, group, cfg):
    z = source_floor(model, group, frame['session'], xy)
    return view_samples(frame, np.c_[xy,z], z, 'floor', tolerance=cfg['depth_tolerance_m'])


def prepare_frame(con, node, profile, pose, session, model, group, cfg):
    frame = decode_frame(con, node, {**profile,'max_depth':5.5}, pose)
    xyz = apply(unproject(frame['depth'],frame['k_depth']).reshape(-1,3),frame['world_optical']).reshape((*frame['depth'].shape,3))
    normal = np.cross(xyz[1:-1,2:]-xyz[1:-1,:-2],xyz[2:,1:-1]-xyz[:-2,1:-1])
    normal /= np.maximum(np.linalg.norm(normal,axis=2)[:,:,None],1e-8)
    horizontal = np.zeros_like(frame['valid']); horizontal[1:-1,1:-1] = abs(normal[:,:,2])>.75
    z = source_floor(model,group,session,xyz[:,:,:2].reshape(-1,2)).reshape(frame['depth'].shape)
    good = horizontal&frame['valid']&(abs(xyz[:,:,2]-z)<cfg['floor_height_tolerance_m'])
    frame['depth'] = np.where(good,frame['depth'],0).astype(np.float32)
    frame['session'] = session
    return frame


def grid_xy(low, extent, width, height):
    yy,xx = np.indices((height,width))
    return low+np.c_[(xx.ravel()+.5)*extent[0]/width,(yy.ravel()+.5)*extent[1]/height]


def reconstruct_group(group, charts, packs, selected, mp, model, cfg, dest):
    low = np.min([packs[c]['low'] for c in charts],axis=0)
    high = np.max([packs[c]['low']+np.array(packs[c]['mask'].shape[::-1])*packs[c]['cell'] for c in charts],axis=0)
    extent = high-low; coarse = np.maximum(8,np.rint(extent/extent.max()*cfg['coarse_side_px']).astype(int))
    cw,ch = coarse; xy = grid_xy(low,extent,cw,ch)
    candidates = []; frames = {}; databases = {}; pose_sets = {}
    for session in ['lobby','floor2','bridge_12_sub']:
        spec = selected['sessions'][session]; poses = load_pose_csv(mp.parent/spec['trajectory']); pose_sets[session] = poses
        eligible = [node for node,pose in poses.items() if .6 < pose[2,3]-source_floor(model,group,session,pose[None,:2,3])[0] < 2.8]
        nodes = select_frames(eligible,cfg['maximum_frames_per_session_layer'],cfg['preferred_nodes'].get(session,[]))
        db = Path(selected['database_integrity'][session]['database']); databases[session] = db
        with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:
            for node in nodes:
                frame = prepare_frame(con,node,spec['preprocessing'],poses[node],session,model,group,cfg)
                rgb,weight = sample_floor(frame,xy,model,group,cfg)
                if np.count_nonzero(weight) < 20:
                    continue
                candidates.append({'session':session,'node':node,'image':srgb_to_linear(rgb).reshape(ch,cw,3).astype(np.float32),'weight':weight.reshape(ch,cw).astype(np.float32)})
        print(f'Floor photos {group}/{session}: {len(nodes)} examined, {len(candidates)} cumulative candidates',flush=True)
    if not candidates:
        raise ValueError('No supported photographic floor observations: '+group)
    chosen = []; coverage = np.zeros((ch,cw),np.float32)
    for _ in range(min(cfg['maximum_selected_photos'],len(candidates))):
        scores = [(v['weight']/(1+coverage*5)).sum() if i not in chosen else -1 for i,v in enumerate(candidates)]
        index = int(np.argmax(scores)); chosen.append(index); coverage += candidates[index]['weight']>0
    views = [candidates[i] for i in chosen]; weights = np.array([v['weight'] for v in views])
    labels = coherent_labels(weights,step=2,potts=.55,iterations=6)
    gains,edges = scalar_photo_gains([v['image'] for v in views],weights,cfg['gain_bounds'])
    dims = np.minimum(cfg['maximum_texture_side_px'],np.maximum(8,np.rint(extent/cfg['metres_per_pixel']).astype(int)))
    width,height = dims; fine_xy = grid_xy(low,extent,width,height)
    labels = cv2.resize(labels.astype(np.float32),(width,height),interpolation=cv2.INTER_NEAREST).astype(int)
    colour = np.zeros((height*width,3),np.float32); total = np.zeros(height*width,np.float32)
    for index,view in enumerate(views):
        session,node = view['session'],view['node']; spec = selected['sessions'][session]
        with sqlite3.connect(f'file:{databases[session]}?mode=ro',uri=True) as con:
            frame = prepare_frame(con,node,spec['preprocessing'],pose_sets[session][node],session,model,group,cfg)
        region = cv2.GaussianBlur((labels==index).astype(np.float32),(0,0),cfg['photo_label_feather_px']).ravel()
        ids = np.flatnonzero(region>1e-4)
        rgb,weight = sample_floor(frame,fine_xy[ids],model,group,cfg)
        alpha = region[ids]*(weight>0)
        colour[ids] += srgb_to_linear(rgb)*gains[index]*alpha[:,None]; total[ids] += alpha
        frames[(session,node)] = frame
    good = total>1e-5; colour[good] /= total[good,None]
    if not good.any():
        raise ValueError('Fine photographic projection empty')
    # Valid source colours, not a mean floor material, seed pyramid boundaries.
    near = ndimage.distance_transform_edt(~good.reshape(height,width),return_distances=False,return_indices=True)
    image = colour.reshape(height,width,3); image = image[near[0],near[1]]
    Image.fromarray(np.uint8(linear_to_srgb(np.clip(image,0,1))*255+.5)).save(dest/(group+'_photo_mosaic.png'))
    np.savez_compressed(dest/(group+'_photo_domain.npz'),valid=good.reshape(height,width),labels=labels,low=low,extent=extent)
    return image,good.reshape(height,width),labels,low,extent,{'candidate_photos':len(candidates),'selected_photos':[{'session':v['session'],'node':v['node'],'scalar_gain':float(g)} for v,g in zip(views,gains)],'exposure_edges':edges,'shape':[int(width),int(height)],'exposure_gain_bounds':cfg['gain_bounds']}


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--config',type=Path,default=ROOT/'config/photographic_surface_finish.json'); parser.add_argument('--output',type=Path,required=True); args = parser.parse_args()
    cfg = json.loads(args.config.read_text()); base = ROOT/cfg['source_bundle']; run = json.loads((base/'manifest.json').read_text()); pointer = json.loads((ROOT/'config/current_map.json').read_text()); mp = ROOT/pointer['manifest']; selected = json.loads(mp.read_text()); model = LevelPresentation(ROOT,run['presentation_config'])
    args.output.mkdir(parents=True,exist_ok=False); dest = args.output/'floor_charts'; dest.mkdir(); source = ROOT/'outputs/student_union_topology_fixed_v2'
    paths = [Path(__file__),args.config,base/'manifest.json',mp,ROOT/'src/level_floor_materials.py',ROOT/'src/coherent_photo_charts.py',ROOT/'src/surface_model_utils.py']; inputs = {str(p.resolve()):sha256_file(p) for p in paths}; packs = {}; chart_by_id = {v['id']:v for v in cfg['charts']}
    for chart in cfg['charts']:
        path = source/chart['session']/(chart['id']+'_mask.npz'); pack = np.load(path); packs[chart['id']] = {k:pack[k].copy() for k in pack.files}; inputs[str(path.resolve())] = sha256_file(path)
    groups = {group:reconstruct_group(group,charts,packs,selected,mp,model,cfg,dest) for group,charts in cfg['groups'].items()}
    reports = []; replacements = {}
    for chart in cfg['charts']:
        identity = chart['id']; pack = packs[identity]; path = source/chart['session']/(identity+'_texture.png'); original = np.asarray(Image.open(path).convert('RGB')); h,w = original.shape[:2]; xy = grid_xy(pack['low'],np.array(pack['mask'].shape[::-1])*pack['cell'],w,h)
        group = next(g for g,ids in cfg['groups'].items() if identity in ids); image,valid,labels,low,extent,record = groups[group]; uv = (xy-low)/extent*np.array(image.shape[1::-1])-.5; maps = uv.reshape(h,w,2).astype(np.float32)
        foreground = cv2.remap(image,maps[:,:,0],maps[:,:,1],cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE); alpha = cv2.remap(valid.astype(np.float32),maps[:,:,0],maps[:,:,1],cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
        surface = cv2.resize(pack['mask'].astype(np.uint8),(w,h),interpolation=cv2.INTER_NEAREST)>0; alpha *= surface
        result = multiband_pair(foreground,srgb_to_linear(original/255.),alpha,cfg['multiband_levels']); result[~surface] = srgb_to_linear(original[~surface]/255.)
        target = dest/(identity+'.png'); Image.fromarray(np.uint8(linear_to_srgb(result)*255+.5)).save(target); coverage_path = source/chart['session']/(identity+'_photo_coverage.png'); old_coverage = np.asarray(Image.open(coverage_path))==255
        digest = sha256_file(path); replacements[digest] = target; reports.append({**chart,'source_texture_sha256':digest,'new_texture_sha256':sha256_file(target),'prior_direct_photo_fraction':float(old_coverage[surface].mean()),'new_direct_photo_fraction':float(alpha[surface].mean()),'uniform_colour_replacement':False,'group':group,'group_report':record})
        for p in [path,coverage_path]: inputs[str(p.resolve())] = sha256_file(p)
    cleanup_before = json.loads((base/'cleanup.json').read_text())['sessions']; sessions = {}; cleanup = {}
    for name,session in run['sessions'].items():
        folder = args.output/name; folder.mkdir(); shutil.copyfile(base/name/'clean_structural.ply',folder/'clean_structural.ply'); hits = replace_floor_references(base/name/'clean_textured.glb',folder/'clean_textured.glb',replacements)
        for path in (base/name).iterdir():
            if path.is_file() and path.name not in ['clean_structural.ply','clean_textured.glb','retained.glb']: shutil.copyfile(path,folder/path.name)
        sessions[name] = {**session,'floor_material_references_changed':hits}; cleanup[name] = {**cleanup_before[name],'ply_sha256':sha256_file(folder/'clean_structural.ply'),'glb_sha256':sha256_file(folder/'clean_textured.glb')}
        for p in [base/name/'clean_structural.ply',base/name/'clean_textured.glb']: inputs[str(p.resolve())] = sha256_file(p)
    for path in base.iterdir():
        if path.is_file() and path.name not in ['manifest.json','cleanup.json','geometry_audit.json','material_audit.json']: shutil.copyfile(path,args.output/path.name)
    for name in ['lobby','floor2','bridge_12_sub']:
        for p in [Path(selected['database_integrity'][name]['database']),mp.parent/selected['sessions'][name]['trajectory']]: inputs[str(p.resolve())] = sha256_file(p)
    manifest = {**run,'created_utc':utc_now(),'sessions':sessions,'inputs_sha256':{**run['inputs_sha256'],**inputs},'floor_photo_preservation':False,'photo_seam_reconstruction':True,'floor_material_strategy':'original_RGBD_shared_photos_bounded_scalar_gains_multiband','floor_material_source_manifest':str((base/'manifest.json').resolve()),'previous_material_manifest':str((base/'manifest.json').resolve()),'floor_material_config':cfg,'floor_material_reports':reports,'scope':cfg['scope']}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n'); (args.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(args.output/'manifest.json')},indent=2)+'\n')
    for path,digest in manifest['inputs_sha256'].items(): assert sha256_file(Path(path))==digest,'Input changed: '+path
    print('Shared photographic floors finished: no geometry or uniform paint.',flush=True)


if __name__ == '__main__': main()
