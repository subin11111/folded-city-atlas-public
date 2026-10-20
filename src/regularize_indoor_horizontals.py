#!/usr/bin/env python3
"""Continuous observed ceiling sheet, bounded holes, and overlapping floor shards.

Presentation-only derivative. Never creates a roof from a floor footprint and
never dilates a ceiling boundary. Original maps and retained coordinates stay.
"""
import argparse,gc,json,shutil,sqlite3
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from scipy import ndimage
from append_plane_glb import append_planes
from clean_structural_fragments import filter_glb
from coherent_photo_charts import coherent_labels,flatten_photo_lighting
from complete_service_slabs import mask_lookup,prune_small_components
from flatten_horizontal_surfaces import build_sheet,protected_points
from optimize_session import sha256_file,utc_now
from presentation_geometry import LevelPresentation
from reconcile_multilevel_surfaces import remap_mask
from reconstruct_continuous_planes import solve_log_gains
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb
from surface_model_utils import project

ROOT=Path(__file__).resolve().parents[1]


def bounded_holes(observed,allowed,cell,maximum_area,maximum_distance):
    enclosed=ndimage.binary_fill_holes(observed)&~observed;labels,count=ndimage.label(enclosed);distance=ndimage.distance_transform_edt(~observed)*cell;result=observed.copy()
    for i in range(1,count+1):
        q=labels==i
        if q.sum()*cell**2<=maximum_area+1e-9 and allowed[q].all() and distance[q].max()<=maximum_distance+1e-9:result[q]=True
    return result


def roof_mask(records,cell,cfg):
    points=np.concatenate([np.asarray(b) for r in records for b in r['boundaries_xy']]);low=np.floor(points.min(0)/cell)*cell-cell;high=np.ceil(points.max(0)/cell)*cell+cell;w,h=np.ceil((high-low)/cell).astype(int);observed=np.zeros((h,w),bool);support=np.zeros((h,w),np.uint16)
    for record in records:
        current=np.zeros((h,w),np.uint8)
        for boundary in record['boundaries_xy']:cv2.fillPoly(current,[np.rint((np.asarray(boundary)-low)/cell).astype(np.int32)],1)
        observed|=current>0;support+=current
    allowed=ndimage.binary_fill_holes(observed);mask=bounded_holes(observed,allowed,cell,cfg['maximum_ceiling_hole_area_m2'],cfg['maximum_ceiling_hole_distance_m']);mask=prune_small_components(mask,max(1,int(cfg['minimum_ceiling_component_area_m2']/cell**2)))
    return mask,observed,support,low


def photo_projection(record,xy):
    centre=np.asarray(record['source_plane_centre']);normal=np.asarray(record['source_plane_normal']);z=centre[2]-(xy-centre[:2])@normal[:2]/normal[2];points=np.c_[xy,z];pose=np.asarray(record['world_optical']);px,depth=project(points,np.asarray(record['K']),pose);x,y=np.rint(np.clip(px,-1e6,1e6)).astype(int).T;image_mask=np.zeros((960,720),np.uint8)
    for poly in record['polygons_rgb']:cv2.fillPoly(image_mask,[np.rint(poly).astype(np.int32)],1)
    valid=(depth>.25)&(depth<8)&(x>=1)&(x<719)&(y>=1)&(y<959);ii=np.flatnonzero(valid);valid[ii]&=image_mask[y[ii],x[ii]]>0;direction=points-pose[:3,3];distance=np.linalg.norm(direction,axis=1);incidence=abs(direction@normal)/np.maximum(distance,1e-8);weight=np.where(valid,incidence**2/(1+.12*distance**2),0.).astype(np.float32)
    return px,weight


def sample_rgb(rgb,px):
    h,w=rgb.shape[:2];x=np.clip(px[:,0],0,w-1);y=np.clip(px[:,1],0,h-1);x0=np.floor(x).astype(int);y0=np.floor(y).astype(int);x1=np.minimum(x0+1,w-1);y1=np.minimum(y0+1,h-1);a=(x-x0)[:,None];b=(y-y0)[:,None];return ((1-b)*((1-a)*rgb[y0,x0]+a*rgb[y0,x1])+b*((1-a)*rgb[y1,x0]+a*rgb[y1,x1]))/255.


def roof_texture(records,mask,low,cell,cfg,db,dest):
    h,w=mask.shape;cw,ch=min(w,180),min(h,180);yy,xx=np.indices((ch,cw));xy=low+np.c_[(xx.ravel()+.5)*w*cell/cw,(yy.ravel()+.5)*h*cell/ch];weights=np.array([photo_projection(r,xy)[1].reshape(ch,cw) for r in records]);coarse_mask=cv2.resize(mask.astype(np.uint8),(cw,ch),interpolation=cv2.INTER_NEAREST)>0;weights*=coarse_mask;chosen=[];coverage=np.zeros((ch,cw),float)
    for _ in range(min(cfg['maximum_texture_frames'],len(records))):
        scores=np.array([(weight/(1+coverage*4)).sum() if i not in chosen else -1 for i,weight in enumerate(weights)]);index=int(scores.argmax())
        if scores[index]<=0:break
        chosen.append(index);coverage+=weights[index]>0
    if not chosen:raise ValueError('No original ceiling photographs cover the observed roof')
    con=sqlite3.connect(f'file:{db}?mode=ro',uri=True);cache={}
    def rgb(index):
        if index not in cache:
            blob=con.execute('SELECT image FROM Data WHERE id=?',(records[index]['node'],)).fetchone()[0];cache[index]=cv2.cvtColor(cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_COLOR),cv2.COLOR_BGR2RGB)
        return cache[index]
    coarse_images=[]
    for i in chosen:coarse_images.append(srgb_to_linear(sample_rgb(rgb(i),photo_projection(records[i],xy)[0])).reshape(ch,cw,3))
    gains=solve_log_gains(coarse_images,[weights[i] for i in chosen]);gains=np.clip(gains,.7,1.5);small=coherent_labels(weights[chosen],step=2,potts=.55,iterations=6);tw=min(cfg['texture_maximum_side_px'],max(8,int(w*cell/cfg['texture_metres_per_pixel'])));th=min(cfg['texture_maximum_side_px'],max(8,int(h*cell/cfg['texture_metres_per_pixel'])));labels=cv2.resize(small.astype(np.float32),(tw,th),interpolation=cv2.INTER_NEAREST).astype(int);yy,xx=np.indices((th,tw));fine_xy=low+np.c_[(xx.ravel()+.5)*w*cell/tw,(yy.ravel()+.5)*h*cell/th];mask_tex=cv2.resize(mask.astype(np.uint8),(tw,th),interpolation=cv2.INTER_NEAREST)>0;colour=np.zeros((th*tw,3),np.float32);total=np.zeros(th*tw,np.float32)
    for label,index in enumerate(chosen):
        blend=cv2.GaussianBlur((labels==label).astype(np.float32),(0,0),2.5).ravel();ids=np.flatnonzero((blend>1e-4)&mask_tex.ravel());px,score=photo_projection(records[index],fine_xy[ids]);weight=blend[ids]*np.sqrt(score);values=srgb_to_linear(sample_rgb(rgb(index),px))*gains[label];colour[ids]+=values*weight[:,None];total[ids]+=weight
    good=total>1e-7;colour[good]/=total[good,None]
    # Only uncovered texels use a nearby measured photo colour, never a
    # generated image or new geometry. Large unknown roof areas stay unmeshed.
    fallback=np.zeros((ch*cw,3),np.float32);best=weights.argmax(0).ravel();valid=weights.max(0).ravel()>0
    for i in np.unique(best[valid]):
        ids=np.flatnonzero((best==i)&valid);px,_=photo_projection(records[int(i)],xy[ids]);fallback[ids]=srgb_to_linear(sample_rgb(rgb(int(i)),px))
    nearest=ndimage.distance_transform_edt(~valid.reshape(ch,cw),return_distances=False,return_indices=True);fallback=fallback.reshape(ch,cw,3)[nearest[0],nearest[1]];fallback=cv2.resize(fallback,(tw,th),interpolation=cv2.INTER_LINEAR).reshape(-1,3);colour[~good]=fallback[~good];colour,lighting=flatten_photo_lighting(colour.reshape(th,tw,3),mask_tex,max(w*cell/tw,h*cell/th));texture=dest/'continuous_2f_roof.png';Image.fromarray(np.uint8(linear_to_srgb(np.clip(colour,0,1))*255+.5)).save(texture);Image.fromarray(np.where(mask_tex,np.where(good.reshape(th,tw),255,80),0).astype(np.uint8)).save(dest/'roof_photo_coverage.png');Image.fromarray(labels.astype(np.uint8)).save(dest/'roof_photo_labels.png');con.close()
    return texture,{'selected_photo_nodes':[records[i]['node'] for i in chosen],'all_observation_frames':len(records),'per_photo_gains':gains.tolist(),'photo_fraction_on_surface':float(good.reshape(th,tw)[mask_tex].mean()),'texture_shape':[tw,th],'lighting':lighting,'labeling':'Potts ICM on coarse source-photo coverage, bounded gains, 2.5px boundary feather; not graph-cut optimality','fallback':'Nearest original-photo colour only in uncovered texels; no generated imagery'}


def floor_layer(layer,meshes,cell):
    packs=[np.load(ROOT/p) for p in layer['masks']];low=np.floor(np.min([p['low'] for p in packs],0)/cell)*cell;high=np.max([p['low']+np.array(p['mask'].shape[::-1])*p['cell'] for p in packs],0);shape=tuple(np.ceil((high-low)/cell).astype(int)[::-1]);allowed=np.zeros(shape,bool);forbidden=np.zeros(shape,bool)
    for pack in packs:
        allowed|=remap_mask(pack['mask'],pack['low'],float(pack['cell']),low,cell,shape)
        forbidden|=remap_mask(pack['stair_guard']|pack['free_guard'],pack['low'],float(pack['cell']),low,cell,shape)
    coverage=np.zeros(shape,bool)
    for name,mesh in meshes.items():
        if name not in ['lobby','floor2','bridge_12_sub']:continue
        xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)];flat=np.max(abs(xyz[:,:,2]-layer['height_m']),axis=1)<2e-5;q=xyz[flat].mean(1);ij=np.floor((q[:,:2]-low)/cell).astype(int);x,y=ij.T;valid=(x>=0)&(x<shape[1])&(y>=0)&(y<shape[0]);coverage[y[valid],x[valid]]=True
    return {**layer,'low':low,'coverage':coverage,'allowed':allowed&~forbidden,'forbidden':forbidden,'texture_low':packs[0]['low'],'texture_extent':np.array(packs[0]['mask'].shape[::-1])*cell}


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/indoor_horizontal_continuity.json');p.add_argument('--observations',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True);cfg=json.loads(a.config.read_text());base=ROOT/cfg['source_bundle'];run=json.loads((base/'manifest.json').read_text());observations=json.loads(a.observations.read_text());assert observations['inputs_sha256'][str(a.config.resolve())]==sha256_file(a.config);records=observations['records'];cell=cfg['cell_m'];mask,observed,support,low=roof_mask(records,cell,cfg);height=observations['target_display_height_m'];ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text());db=Path(selected['database_integrity']['floor2']['database']);model=LevelPresentation(ROOT,run['presentation_config']);old_guard=json.loads((ROOT/'outputs/student_union_verified_wall_delivery_v2/manifest.json').read_text());inputs={str(p.resolve()):sha256_file(p) for p in [Path(__file__),a.config,a.observations,base/'manifest.json',ROOT/'src/append_plane_glb.py',ROOT/'src/coherent_photo_charts.py',ROOT/'src/clean_structural_fragments.py',ROOT/'outputs/student_union_verified_wall_delivery_v2/manifest.json']};inputs.update(observations['inputs_sha256'])
    import open3d as o3d
    meshes={n:o3d.io.read_triangle_mesh(str(base/n/'clean_structural.ply')) for n in run['sessions']};layers=[floor_layer(layer,meshes,cell) for layer in cfg['floor_layers']]
    for layer in layers:
        filled=bounded_holes(layer['coverage'],layer['allowed'],cell,cfg['maximum_floor_hole_area_m2'],.4);layer['holes']=filled&~layer['coverage'];np.savez_compressed(a.output/(layer['id']+'_floor_mask.npz'),coverage=layer['coverage'],allowed=layer['allowed'],holes=layer['holes'],forbidden=layer['forbidden'],low=layer['low'],cell=cell,height=layer['height_m'])
        for f in layer['masks']+[layer['texture']]:inputs[str((ROOT/f).resolve())]=sha256_file(ROOT/f)
    np.savez_compressed(a.output/'roof_mask.npz',mask=mask,observed=observed,support=support,low=low,cell=cell,height=height);sessions={};cleanup={}
    for name,mesh in meshes.items():
        print('Continuous indoor surfaces: '+name,flush=True);dest=a.output/name;dest.mkdir();mesh.compute_triangle_normals();xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32);centres=xyz.mean(1);nz=abs(np.asarray(mesh.triangle_normals)[:,2]);keep=np.ones(len(xyz),bool);floor_remove=np.zeros(len(xyz),bool);roof_remove=np.zeros(len(xyz),bool);added=[];uvs=[];textures=[];reports=[]
        if name in ['lobby','floor2','bridge_12_sub']:
            guard=old_guard['sessions'][name]['flat_stair_guard'];c=np.asarray(guard['centres']);half=np.asarray(guard['half'])
            if len(c):c=model.warp(c,name);protected=protected_points(xyz.reshape(-1,3),c,half+[.1,.1,.5]).reshape(-1,3).any(1)
            else:protected=np.zeros(len(xyz),bool)
            for layer in layers:
                coverage=layer['coverage']|layer['holes'];inside=mask_lookup(coverage,layer['low'],cell,centres[:,:2]);permitted=mask_lookup(layer['allowed'],layer['low'],cell,centres[:,:2]);near=np.max(abs(xyz[:,:,2]-layer['height_m']),axis=1)<cfg['floor_fragment_band_m'];exact=np.max(abs(xyz[:,:,2]-layer['height_m']),axis=1)<2e-5;floor_remove|=inside&permitted&near&~exact&(nz>.55)&~protected
                if name==layer['owner'] and layer['holes'].any():
                    patch,uv=build_sheet(o3d,layer['holes'],layer['low'],layer['height_m'],cell,'floor');points=np.asarray(patch.vertices);uv=(points[:,:2]-layer['texture_low'])/layer['texture_extent'];added.append(patch);uvs.append(uv[np.asarray(patch.triangles)]);textures.append(ROOT/layer['texture']);reports.append({'id':layer['id']+'_bounded_floor_holes','kind':'floor','height_m':layer['height_m'],'added_area_m2':float(layer['holes'].sum()*cell**2),'triangles':len(patch.triangles)})
            inside=mask_lookup(mask,low,cell,centres[:,:2]);near=abs(centres[:,2]-height)<cfg['ceiling_fragment_band_m'];roof_remove=inside&near&(nz>.5);keep&=~(floor_remove|roof_remove)
        if name=='floor2':
            texture,photo=roof_texture(records,mask,low,cell,cfg,db,dest);patch,uv=build_sheet(o3d,mask,low,height,cell,'ceiling');added.append(patch);uvs.append(uv);textures.append(texture);reports.append({'id':'continuous_observed_2f_ceiling','kind':'ceiling','height_m':height,'area_m2':float(mask.sum()*cell**2),'bounded_hole_fill_m2':float((mask&~observed).sum()*cell**2),'triangles':len(patch.triangles),'observed_roof_frames':len(records),'no_outward_boundary_dilation':True,**photo})
        np.save(dest/'retained_source_faces.npy',keep);np.savez_compressed(dest/'removal_categories.npz',floor=floor_remove,roof=roof_remove);filter_glb(base/name/'clean_textured.glb',dest/'retained.glb',keep);mesh.remove_triangles_by_mask(~keep);mesh.remove_unreferenced_vertices();retained=len(mesh.triangles)
        for patch in added:mesh+=patch
        mesh.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'clean_structural.ply'),mesh);append_planes(dest/'retained.glb',dest/'clean_textured.glb',added,uvs,textures);sessions[name]={**run['sessions'][name],'retained_source_triangles':retained,'horizontal_continuity':reports,'removed_floor_overlap_triangles':int(floor_remove.sum()),'removed_roof_fragment_triangles':int(roof_remove.sum()),'bounds_min_m':mesh.get_min_bound().tolist(),'bounds_max_m':mesh.get_max_bound().tolist(),'source_mesh':str((base/name/'clean_structural.ply').resolve()),'source_texture':str((base/name/'clean_textured.glb').resolve())};cleanup[name]={'before_faces':len(keep),'after_faces':len(mesh.triangles),'removed_faces':int((~keep).sum()),'ply_sha256':sha256_file(dest/'clean_structural.ply'),'glb_sha256':sha256_file(dest/'clean_textured.glb')}
        for f in [base/name/'clean_structural.ply',base/name/'clean_textured.glb']:inputs[str(f.resolve())]=sha256_file(f)
        for obj in run['sessions'][name].get('semantic_objects',[]):
            for f in [obj.get('source_rgb'),obj.get('geometry',{}).get('texture')]:
                if f and (base/name/f).exists():shutil.copyfile(base/name/f,dest/f)
        del xyz;gc.collect()
    for f in ['semantic_objects.json','reviewed_atrium.npz']:shutil.copyfile(base/f,a.output/f)
    manifest={**run,'created_utc':utc_now(),'sessions':sessions,'inputs_sha256':{**run['inputs_sha256'],**inputs},'base_display_manifest':str((base/'manifest.json').resolve()),'horizontal_continuity_completion':True,'horizontal_continuity_config':cfg,'roof_observations':str(a.observations.resolve()),'retained_geometry_already_normalized':True,'scope':'Continuous observed 2F ceiling and bounded interior floor hole patches; remove overlapping near-floor/roof fragments. No source pose changes, no outward roof growth, no new blanket lower ceiling. Not navigation/surveyed geometry.'};(a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(a.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.output/'manifest.json')},indent=2)+'\n')
    for f,h in manifest['inputs_sha256'].items():
        if sha256_file(Path(f))!=h:raise ValueError('Source binding changed: '+f)
    print(f'Finished continuous roof: {mask.sum()*cell**2:.2f}m2, internal filled holes {(mask&~observed).sum()*cell**2:.2f}m2',flush=True)


if __name__=='__main__':main()
