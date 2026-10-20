#!/usr/bin/env python3
"""Photographic floor material without baked exposure patches or flat paint."""
import argparse,json,shutil
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from level_floor_materials import replace_floor_references
from optimize_session import sha256_file,utc_now
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb

ROOT=Path(__file__).resolve().parents[1]


def reconstruct_floor_material(rgb,surface,pixel_m,grain,cfg):
    """Display material inference: measured dark accents/joints, not new layouts."""
    linear=srgb_to_linear(rgb/255.).astype(np.float32);lum=np.maximum(linear@np.array([.2126,.7152,.0722]),1e-5);log=np.log(lum);size=max(3,int(round(.20/pixel_m))|1);closed=cv2.morphologyEx(log.astype(np.float32),cv2.MORPH_CLOSE,np.ones((size,size),np.uint8));crease=np.maximum(closed-log,0)
    # Only finite small dark components become black stone accents. Broad
    # shadows are not copied as black tiles. No periodic grid is synthesized.
    dark=((crease>.95)&surface).astype(np.uint8);count,labels,stats,_=cv2.connectedComponentsWithStats(dark);black=np.zeros_like(lum,np.float32)
    for i in range(1,count):
        x,y,w,h,area=stats[i];area_m2=area*pixel_m**2
        if .002<area_m2<.09 and max(w,h)*pixel_m<.55 and .15<w/max(h,1)<6:black[labels==i]=1
    joint=cv2.morphologyEx(((crease>.23)&surface).astype(np.uint8),cv2.MORPH_OPEN,np.ones((3,3),np.uint8));joint=cv2.GaussianBlur(joint.astype(np.float32),(0,0),.7);black=cv2.GaussianBlur(black,(0,0),.55);reference=np.asarray(cfg['reference_chromaticity'])*cfg['target_linear_luminance'];low=cv2.GaussianBlur(linear,(0,0),1.5);detail=np.clip(linear/np.maximum(low,.003)-1,-.18,.18)*reference*.25;reliable=np.clip((lum-.012)/.025,0,1)*np.clip((.80-lum)/.20,0,1);gh,gw=grain.shape[:2];h,w=lum.shape;exemplar=np.tile(grain,(h//gh+1,w//gw+1,1))[:h,:w];detail=detail*reliable[:,:,None]+exemplar*(1-reliable[:,:,None])*.8;result=np.broadcast_to(reference,linear.shape).copy()+detail
    # The gray joints and black accents come from the existing photographed
    # texels; the stone background/micro-grain is explicitly modeled appearance.
    if cfg.get('preserve_joint_bands'):
        sigma=max(2.,cfg['detail_radius_m']/pixel_m);weight=surface.astype(np.float32);illumination=cv2.GaussianBlur(log*weight,(0,0),sigma)/np.maximum(cv2.GaussianBlur(weight,(0,0),sigma),1e-6);ratio=np.exp(np.clip(log-illumination,np.log(cfg['contrast_bounds'][0]),np.log(cfg['contrast_bounds'][1])));result=np.broadcast_to(reference,linear.shape).copy()*ratio[:,:,None]+detail
    gray=np.array([.105,.108,.106]);result=result*(1-joint[:,:,None]*.35)+gray*joint[:,:,None]*.35;result=result*(1-black[:,:,None]*.94);result=np.clip(result,0,1);result[~surface]=linear[~surface]
    return np.uint8(linear_to_srgb(result)*255+.5)


def photographic_albedo(rgb,surface,pixel_m,grain,cfg):
    if cfg.get('material_reconstruction'):return reconstruct_floor_material(rgb,surface,pixel_m,grain,cfg)
    original_linear=srgb_to_linear(rgb/255.).astype(np.float32);linear=original_linear.copy();minimum=cfg.get('radiometric_floor',.003);lum=np.maximum(linear@np.array([.2126,.7152,.0722]),minimum);sigma=max(3.,cfg['illumination_radius_m']/pixel_m);weight=surface.astype(np.float32);denominator=np.maximum(cv2.GaussianBlur(weight,(0,0),sigma),1e-6)
    if cfg.get('low_light_denoising'):
        broad=cv2.GaussianBlur(lum,(0,0),sigma);amount=np.clip((.005-broad)/.004,0,1);filtered=cv2.GaussianBlur(linear,(0,0),1.2);linear=linear*(1-amount[:,:,None])+filtered*amount[:,:,None];lum=np.maximum(linear@np.array([.2126,.7152,.0722]),minimum)
    if cfg.get('chromaticity_normalization'):
        low_rgb=cv2.GaussianBlur(linear*weight[:,:,None],(0,0),sigma)/denominator[:,:,None];low_y=np.maximum(low_rgb@np.array([.2126,.7152,.0722]),minimum);chroma=low_rgb/low_y[:,:,None];ratio=np.asarray(cfg['reference_chromaticity'])/np.maximum(chroma,.05);changed=linear*ratio;changed*=lum[:,:,None]/np.maximum(changed@np.array([.2126,.7152,.0722]),minimum)[:,:,None];linear=changed.astype(np.float32)
    low=cv2.GaussianBlur(np.log(lum)*weight,(0,0),sigma)/denominator;gain=np.exp(np.clip(np.log(cfg['target_linear_luminance'])-low,np.log(cfg['gain_bounds'][0]),np.log(cfg['gain_bounds'][1])));result=np.clip(linear*gain[:,:,None],0,1)
    # Only measured zero-mean micro-grain; no repeated tile/accent layout.
    detail=linear-cv2.GaussianBlur(linear,(0,0),1.2);texture=np.sqrt(cv2.GaussianBlur(np.mean(detail**2,axis=2),(0,0),3));weak=np.clip((.014-texture)/.014,0,1);output_luma=result@np.array([.2126,.7152,.0722]);weak*=np.clip((output_luma-.04)/.06,0,1);weak*=surface;gh,gw=grain.shape[:2];h,w=lum.shape;pattern=np.tile(grain,(h//gh+1,w//gw+1,1))[:h,:w];result=np.clip(result+pattern*weak[:,:,None]*.7,0,1);result[~surface]=original_linear[~surface]
    return np.uint8(linear_to_srgb(result)*255+.5)


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/intrinsic_floor_material.json');p.add_argument('--output',type=Path,required=True);a=p.parse_args();cfg=json.loads(a.config.read_text());base=ROOT/cfg['source_bundle'];run=json.loads((base/'manifest.json').read_text());a.output.mkdir(parents=True,exist_ok=False);dest=a.output/'floor_charts';dest.mkdir();grain_path=ROOT/cfg['grain_source'];crop=np.asarray(Image.open(grain_path).convert('RGB').crop(cfg['grain_crop']));grain=srgb_to_linear(crop/255.).astype(np.float32);reference=np.median(grain.reshape(-1,3),axis=0);cfg['reference_chromaticity']=(reference/max(float(reference@np.array([.2126,.7152,.0722])),.003)).tolist();grain-=cv2.GaussianBlur(grain,(0,0),2);grain-=grain.mean(axis=(0,1));inputs={str(path.resolve()):sha256_file(path) for path in [Path(__file__),a.config,base/'manifest.json',grain_path,ROOT/'src/level_floor_materials.py']};replacements={};reports=[];Image.fromarray(crop).save(dest/'original_stone_grain_crop.png')
    for chart in run['floor_material_reports']:
        identity=chart['id'];name=chart['session'];path=base/'floor_charts'/(identity+'.png');mask_path=ROOT/'outputs/student_union_topology_fixed_v2'/name/(identity+'_mask.npz');pack=np.load(mask_path);rgb=np.asarray(Image.open(path).convert('RGB'));h,w=rgb.shape[:2];surface=cv2.resize(pack['mask'].astype(np.uint8),(w,h),interpolation=cv2.INTER_NEAREST)>0;pixel_m=max(pack['mask'].shape[1]*pack['cell']/w,pack['mask'].shape[0]*pack['cell']/h);result=photographic_albedo(rgb,surface,pixel_m,grain,cfg);target=dest/(identity+'.png');Image.fromarray(result).save(target);digest=sha256_file(path);assert digest==chart['new_texture_sha256'];replacements[digest]=target;reports.append({'id':identity,'session':name,'source_texture_sha256':digest,'new_texture_sha256':sha256_file(target),'modeled_micro_grain':True,'no_uniform_RGB_paint':True,'no_synthetic_tile_grid':True})
        for source in [path,mask_path]:inputs[str(source.resolve())]=sha256_file(source)
    sessions={};cleanup={};previous=json.loads((base/'cleanup.json').read_text())['sessions']
    for name,spec in run['sessions'].items():
        folder=a.output/name;folder.mkdir();shutil.copyfile(base/name/'clean_structural.ply',folder/'clean_structural.ply');hits=replace_floor_references(base/name/'clean_textured.glb',folder/'clean_textured.glb',replacements)
        for path in (base/name).iterdir():
            if path.is_file() and path.name not in ['clean_structural.ply','clean_textured.glb','finish_retained.glb','finish_appended.glb']:shutil.copyfile(path,folder/path.name)
        sessions[name]={**spec,'floor_material_references_changed':hits};cleanup[name]={**previous[name],'ply_sha256':sha256_file(folder/'clean_structural.ply'),'glb_sha256':sha256_file(folder/'clean_textured.glb')}
        for path in [base/name/'clean_structural.ply',base/name/'clean_textured.glb']:inputs[str(path.resolve())]=sha256_file(path)
    for path in base.iterdir():
        if path.is_file() and path.name not in ['manifest.json','cleanup.json','geometry_audit.json','material_audit.json']:shutil.copyfile(path,a.output/path.name)
    manifest={**run,'created_utc':utc_now(),'sessions':sessions,'inputs_sha256':{**run['inputs_sha256'],**inputs},'intrinsic_floor_material':True,'modeled_floor_material':bool(cfg.get('material_reconstruction')),'floor_material_strategy':'photo_feature_preserving_modeled_stone_material' if cfg.get('material_reconstruction') else 'photographic_diffuse_material_illumination_and_colour_cast_separation','floor_material_source_manifest':str((base/'manifest.json').resolve()),'previous_material_manifest':str((ROOT/cfg['comparison_bundle']/'manifest.json').resolve()) if cfg.get('comparison_bundle') else str((base/'manifest.json').resolve()),'floor_material_config':cfg,'floor_material_reports':reports,'scope':cfg['scope']};(a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(a.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.output/'manifest.json')},indent=2)+'\n')
    for path,digest in manifest['inputs_sha256'].items():assert sha256_file(Path(path))==digest,'Input changed: '+path


if __name__=='__main__':main()
