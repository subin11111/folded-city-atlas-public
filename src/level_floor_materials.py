#!/usr/bin/env python3
"""Repair fragmented floor colour charts without changing a vertex or footprint."""
import argparse,copy,hashlib,json,shutil,struct
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from scipy import ndimage
from optimize_session import sha256_file,utc_now
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb

ROOT=Path(__file__).resolve().parents[1]


def consistent_floor_colour(rgb,reliable,surface,pixel_m,cfg):
    linear=srgb_to_linear(np.asarray(rgb)/255.).astype(np.float32);luma=np.maximum(linear@np.array([.2126,.7152,.0722]),.002);good=reliable&surface&(luma>.015)&(luma<.92)
    if good.sum()<100:raise ValueError('Insufficient original floor-photo support')
    sigma=max(2.,cfg['lighting_radius_m']/pixel_m);weight=good.astype(np.float32);normalizer=np.maximum(cv2.GaussianBlur(weight,(0,0),sigma),1e-8);low=cv2.GaussianBlur(linear*weight[:,:,None],(0,0),sigma)/normalizer[:,:,None];nearest=ndimage.distance_transform_edt(~good,return_distances=False,return_indices=True);low=low[nearest[0],nearest[1]]
    # The inferred appearance contains no fabricated tile lines or black
    # accents. It is a robust photographed stone colour, deliberately smooth.
    lo,hi=np.quantile(luma[good],[.25,.85]);stone=good&(luma>=lo)&(luma<=hi);base=np.median(linear[stone],axis=0);base*=cfg['target_linear_luminance']/max(float(base@np.array([.2126,.7152,.0722])),.005);rgb_detail=np.log(np.maximum(linear,.002))-np.log(np.maximum(low,.002));low_luma=np.maximum(low@np.array([.2126,.7152,.0722]),.002);luma_detail=np.log(luma)-np.log(low_luma);chroma_confidence=np.clip((luma-.05)/.15,0,1);detail=np.clip(chroma_confidence[:,:,None]*rgb_detail+(1-chroma_confidence[:,:,None])*luma_detail[:,:,None],*cfg['log_detail_bounds']);fixed=np.clip(base*np.exp(detail),0,1);fallback=np.broadcast_to(base,fixed.shape).copy();distance=ndimage.distance_transform_edt(reliable&surface)*pixel_m;blend=np.clip(distance/cfg['photo_edge_feather_m'],0,1);blend*=reliable;result=fixed*blend[:,:,None]+fallback*(1-blend[:,:,None]);result[~surface]=fallback[~surface];out=np.uint8(np.clip(linear_to_srgb(np.clip(result,0,1)),0,1)*255+.5)
    return out,{'reliable_photo_fraction':float(reliable[surface].mean()),'fallback_colour_srgb':np.uint8(linear_to_srgb(base)*255+.5).tolist(),'log_detail_bounds':cfg['log_detail_bounds'],'target_linear_luminance':cfg['target_linear_luminance'],'normalization':'Photographed high-frequency detail on one robust photographed stone colour; dark-photo chroma suppressed to avoid amplifying colour noise. Low-frequency exposure removed.','dark_chroma_luminance_ramp':[.05,.20],'unreliable_texels_use_smooth_measured_colour':True,'invented_tile_pattern':False}


def replace_floor_references(source,target,replacements):
    raw=Path(source).read_bytes();size=struct.unpack_from('<I',raw,12)[0];doc=json.loads(raw[20:20+size]);payload=bytearray(raw[28+size:]);hashes={}
    for index,image in enumerate(doc.get('images',[])):
        if 'bufferView' not in image:continue
        view=doc['bufferViews'][image['bufferView']];begin=view.get('byteOffset',0);hashes[index]=hashlib.sha256(payload[begin:begin+view['byteLength']]).hexdigest()
    materials={};hits={key:0 for key in replacements}
    for mesh in doc['meshes']:
        for primitive in mesh['primitives']:
            old=primitive.get('material')
            if old is None:continue
            material=doc['materials'][old];texture=material.get('pbrMetallicRoughness',{}).get('baseColorTexture',{}).get('index')
            if texture is None:continue
            digest=hashes.get(doc['textures'][texture]['source'])
            if digest not in replacements:continue
            if old not in materials:
                path=replacements[digest];blob=Path(path).read_bytes();payload.extend(b'\0'*(-len(payload)%4));start=len(payload);payload.extend(blob);doc['bufferViews'].append({'buffer':0,'byteOffset':start,'byteLength':len(blob)});doc['images'].append({'bufferView':len(doc['bufferViews'])-1,'mimeType':'image/png'});sampler=doc['textures'][texture].get('sampler',0);doc['textures'].append({'source':len(doc['images'])-1,'sampler':sampler});changed=copy.deepcopy(material);changed['pbrMetallicRoughness']['baseColorTexture']['index']=len(doc['textures'])-1;doc['materials'].append(changed);materials[old]=len(doc['materials'])-1
            primitive['material']=materials[old];hits[digest]+=1
    # GLB chunks require four-byte alignment, including the final appended PNG.
    payload.extend(b'\0'*(-len(payload)%4))
    doc['buffers'][0]['byteLength']=len(payload);encoded=json.dumps(doc,separators=(',',':')).encode();encoded+=b' '*(-len(encoded)%4)
    with Path(target).open('xb') as f:
        f.write(struct.pack('<4sII',b'glTF',2,28+len(encoded)+len(payload)));f.write(struct.pack('<I4s',len(encoded),b'JSON'));f.write(encoded);f.write(struct.pack('<I4s',len(payload),b'BIN\0'));f.write(payload)
    return {k:v for k,v in hits.items() if v}


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/floor_material_continuity.json');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True);cfg=json.loads(a.config.read_text());base=ROOT/cfg['source_bundle'];run=json.loads((base/'manifest.json').read_text());chart_dir=a.output/'floor_charts';chart_dir.mkdir();source=ROOT/'outputs/student_union_topology_fixed_v2';replacements={};reports=[];inputs={str(p.resolve()):sha256_file(p) for p in [Path(__file__),a.config,base/'manifest.json',ROOT/'src/refine_surface_display.py',ROOT/'src/repair_visual_appearance.py']}
    for chart in cfg['charts']:
        folder=source/chart['session'];identity=chart['id'];texture=folder/(identity+'_texture.png');coverage=folder/(identity+'_photo_coverage.png');mask_path=folder/(identity+'_mask.npz');im=np.asarray(Image.open(texture).convert('RGB'));pack=np.load(mask_path);surface=cv2.resize(pack['mask'].astype(np.uint8),(im.shape[1],im.shape[0]),interpolation=cv2.INTER_NEAREST)>0;reliable=np.asarray(Image.open(coverage))==255;pixel=max(pack['mask'].shape[1]*pack['cell']/im.shape[1],pack['mask'].shape[0]*pack['cell']/im.shape[0]);out,report=consistent_floor_colour(im,reliable,surface,pixel,cfg);dest=chart_dir/(identity+'.png');Image.fromarray(out).save(dest);digest=sha256_file(texture);replacements[digest]=dest;reports.append({**chart,**report,'source_texture_sha256':digest,'new_texture_sha256':sha256_file(dest)})
        for f in [texture,coverage,mask_path]:inputs[str(f.resolve())]=sha256_file(f)
    sessions={};cleanup={}
    for name,s in run['sessions'].items():
        print('Coherent floor material: '+name,flush=True);dest=a.output/name;dest.mkdir();shutil.copyfile(base/name/'clean_structural.ply',dest/'clean_structural.ply');matches=replace_floor_references(base/name/'clean_textured.glb',dest/'clean_textured.glb',replacements)
        for f in (base/name).iterdir():
            if f.name in ['clean_structural.ply','clean_textured.glb','retained.glb']:continue
            if f.is_file():shutil.copyfile(f,dest/f.name)
        sessions[name]={**s,'floor_material_references_changed':matches};cleanup[name]={'before_faces':json.loads((base/'cleanup.json').read_text())['sessions'][name]['before_faces'],'after_faces':json.loads((base/'cleanup.json').read_text())['sessions'][name]['after_faces'],'removed_faces':json.loads((base/'cleanup.json').read_text())['sessions'][name]['removed_faces'],'ply_sha256':sha256_file(dest/'clean_structural.ply'),'glb_sha256':sha256_file(dest/'clean_textured.glb')}
        for f in [base/name/'clean_structural.ply',base/name/'clean_textured.glb']:inputs[str(f.resolve())]=sha256_file(f)
    for f in base.iterdir():
        if f.is_file() and f.name not in ['manifest.json','cleanup.json','geometry_audit.json']:shutil.copyfile(f,a.output/f.name)
    manifest={**run,'created_utc':utc_now(),'sessions':sessions,'inputs_sha256':{**run['inputs_sha256'],**inputs},'floor_material_continuity':True,'floor_material_source_manifest':str((base/'manifest.json').resolve()),'floor_material_config':cfg,'floor_material_reports':reports,'scope':run['scope']+' Floor colour only: original photographic details retained on reliable texels, inferred smooth photographed stone colour elsewhere; no invented tile layout.'};(a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(a.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.output/'manifest.json')},indent=2)+'\n')
    for f,h in manifest['inputs_sha256'].items():
        if sha256_file(Path(f))!=h:raise ValueError('Source binding changed: '+f)
    print('Floor material continuity finished without geometry changes.',flush=True)


if __name__=='__main__':main()
