#!/usr/bin/env python3
"""Local closed presentation slabs and omitted original-photo bank signs.

Shared underside/top/side footprints are bounded by BOTH photographed ceilings
and existing complete upper-floor cells. Original pose graphs stay immutable.
"""
import argparse,gc,json,shutil,sqlite3
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree

from append_plane_glb import append_planes
from build_unified_presentation import bounded_photo_polygon,ceiling_fit
from clean_structural_fragments import filter_glb
from complete_verified_upper_walls import inside_polygon
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from photo_semantic_surfaces import fitted_plane,lift_pixels,photo_mesh
from presentation_geometry import LevelPresentation
from probe_upper_wall_support import polygon_points
from surface_model_utils import decode_frame,project

ROOT=Path(__file__).resolve().parents[1]


def mask_lookup(mask,low,cell,xy):
    ij=np.floor((np.asarray(xy)-low)/cell).astype(int);x,y=ij.T;h,w=mask.shape;valid=(x>=0)&(x<w)&(y>=0)&(y<h);result=np.zeros(len(x),bool);result[valid]=mask[y[valid],x[valid]];return result


def fill_bounded_holes(mask,allowed,maximum_cells):
    holes=ndimage.binary_fill_holes(mask)&~mask;labels,count=ndimage.label(holes)
    result=mask.copy()
    for i in range(1,count+1):
        q=labels==i
        if q.sum()<=maximum_cells and allowed[q].all():result[q]=True
    return result


def remove_diagonal_contacts(mask):
    """Shrink checkerboard contacts; never expand a boundary or seal a void."""
    result=mask.copy()
    while True:
        a,b,c,d=result[:-1,:-1],result[:-1,1:],result[1:,:-1],result[1:,1:]
        first=np.argwhere(a&d&~b&~c);second=np.argwhere(b&c&~a&~d)
        if not len(first) and not len(second):return result
        for y,x in first:result[y,x]=False
        for y,x in second:result[y,x+1]=False


def prune_small_components(mask,minimum_cells):
    labels,count=ndimage.label(mask);sizes=np.bincount(labels.ravel());keep=sizes>=minimum_cells;keep[0]=False;return keep[labels]


def closed_slab_arrays(mask,low,cell,bottom,top):
    """One welded solid: paired horizontal faces plus each perimeter edge once."""
    h,w=mask.shape;y,x=np.indices((h+1,w+1));xy=low+np.c_[x.ravel(),y.ravel()]*cell;n=len(xy)
    vertices=np.r_[np.c_[xy,np.full(n,bottom)],np.c_[xy,np.full(n,top)]];ids=np.arange(n).reshape(h+1,w+1);lower=[];upper=[];side=[]
    for j,i in zip(*np.where(mask)):
        a,b,c,d=ids[j,i],ids[j,i+1],ids[j+1,i],ids[j+1,i+1]
        upper.extend([[a+n,b+n,c+n],[b+n,d+n,c+n]])
        lower.extend([[a,c,b],[b,c,d]])
        for exposed,first,last in [(j==0 or not mask[j-1,i],a,b),(i==w-1 or not mask[j,i+1],b,d),(j==h-1 or not mask[j+1,i],d,c),(i==0 or not mask[j,i-1],c,a)]:
            if exposed:side.extend([[first,last,last+n],[first,last+n,first+n]])
    return vertices,[np.asarray(v,dtype=int).reshape(-1,3) for v in [lower,upper,side]]


def actual_upper_floor_ids(base,run):
    prior=json.loads(Path(run['base_display_manifest']).read_text());spec=prior['sessions']['floor2'];cursor=spec['retained_previous_prefix_faces'];keep=np.load(base/'floor2/retained_source_faces.npy');rank=np.cumsum(keep)-1
    for layer in spec['flat_sheets']:
        if not layer.get('accepted'):continue
        end=cursor+layer['triangles']
        if layer['id']=='floor2_floor_2f':return rank[np.arange(cursor,end)[keep[cursor:end]]]
        cursor=end
    raise ValueError('No accepted upper-floor display sheet')


def raster_photo_regions(records,frames,height,low,cell,shape):
    mask=np.zeros(shape,np.uint8)
    for record in records:
        region=record['region'];frame=frames[region['id']];poly=bounded_photo_polygon(region['polygon'],frame,height)
        if len(poly)<3:continue
        xy,t=lift_pixels(poly,frame['k_rgb'],frame['world_optical'],[0,0,height],[0,0,1])
        if not np.isfinite(xy).all() or not ((t>.25)&(t<8)).all():raise ValueError('Unbounded ceiling rays')
        cv2.fillPoly(mask,[np.rint((xy[:,:2]-low)/cell).astype(np.int32)],1)
    return mask>0


def texture_chart(records,frames,height,low,cell,shape,target):
    h,w=shape;tw,th=max(8,int(w*cell/.025)),max(8,int(h*cell/.025));y,x=np.indices((th,tw));xy=low+np.c_[(x.ravel()+.5)*w*cell/tw,(y.ravel()+.5)*h*cell/th];world=np.c_[xy,np.full(len(xy),height)];out=np.full((len(xy),3),210.,float);seen=np.zeros(len(xy),bool);gains=[]
    for rec in records:
        region=rec['region'];f=frames[region['id']];px,z=project(world,f['k_rgb'],f['world_optical']);good=inside_polygon(px,region['polygon'])&(z>.25)&(z<8);rgb=cv2.remap(f['rgb'],px[:,0].reshape(th,tw).astype(np.float32),px[:,1].reshape(th,tw).astype(np.float32),cv2.INTER_LINEAR).reshape(-1,3).astype(float);gain=float(np.clip(175/max(np.median(rgb[good].mean(1)) if good.any() else 175,1),.85,1.5));rgb*=gain;overlap=good&seen
        if overlap.sum()>100:rgb*=np.clip(np.median(out[overlap].mean(1))/max(np.median(rgb[overlap].mean(1)),1),.8,1.25)
        use=good&~seen;out[use]=rgb[use];seen|=good;gains.append(gain)
    Image.fromarray(np.uint8(np.clip(out.reshape(th,tw,3),0,255))).save(target)
    return gains


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/verified_service_slabs.json');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);cfg=json.loads(a.config.read_text());base=ROOT/cfg['source_bundle'];prior=json.loads((base/'manifest.json').read_text());ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text());model=LevelPresentation(ROOT,prior['presentation_config']);inputs={str(p.resolve()):sha256_file(p) for p in [Path(__file__),a.config,base/'manifest.json',ROOT/cfg['upper_floor_support'],ROOT/cfg['atrium_mask'],ROOT/'src/photo_semantic_surfaces.py',ROOT/'src/append_plane_glb.py',ROOT/'src/clean_structural_fragments.py']}
    import open3d as o3d
    source_meshes={n:o3d.io.read_triangle_mesh(str(base/n/'clean_structural.ply')) for n in prior['sessions']};upper_ids=actual_upper_floor_ids(base,prior);upper_mesh=source_meshes['floor2'];upper_xyz=np.asarray(upper_mesh.vertices)[np.asarray(upper_mesh.triangles)[upper_ids]];upper_grid=np.load(ROOT/cfg['upper_floor_support']);low=upper_grid['low'];cell=float(upper_grid['cell']);assert abs(cell-cfg['cell_m'])<1e-9;shape=upper_grid['mask'].shape;counts=np.zeros(shape,int);ij=np.floor((upper_xyz.mean(1)[:,:2]-low)/cell).astype(int);np.add.at(counts,(ij[:,1],ij[:,0]),1);allowed=(counts==2)&upper_grid['mask']&~upper_grid['stair_guard']&~upper_grid['free_guard'];atrium=np.load(ROOT/cfg['atrium_mask']);y,x=np.indices(shape);xy=low+np.c_[x.ravel()+.5,y.ravel()+.5]*cell;forbidden=mask_lookup(atrium['core'],atrium['low'],float(atrium['cell']),xy).reshape(shape);allowed&=~forbidden
    frames={};records=[];poses_cache={}
    regions=cfg['ceiling_regions']+cfg['bank_objects']
    # Include the two already reviewed original restroom ceiling charts.
    regions+= [r for r in prior['presentation_config']['regions'] if r['id'] in prior['presentation_config']['soffit_group']]
    for region in regions:
        name=region['session'];spec=selected['sessions'][name]
        if name not in poses_cache:poses_cache[name]=load_pose_csv(mp.parent/spec['trajectory'])
        db=Path(selected['database_integrity'][name]['database']);inputs[str(db)]=sha256_file(db)
        with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:frame=decode_frame(con,region['node'],{**spec['preprocessing'],'max_depth':5.5},poses_cache[name][region['node']])
        frames[region['id']]=frame
        if region.get('kind')=='ceiling':
            plane,fit=ceiling_fit(polygon_points(frame,region['polygon']))
            if plane is None:raise ValueError('No coherent physical ceiling '+region['id'])
            records.append({'region':region,'plane':plane,'fit':fit})
    continuous=ROOT/'outputs/student_union_continuous_selected_v4/lobby';plane_spec=json.loads((continuous/'summary.json').read_text())['continuous_planes'];cloud=o3d.io.read_point_cloud(str(mp.parent/selected['sessions']['lobby']['cloud']));tree=cKDTree(np.asarray(cloud.points));inputs[str(continuous/'summary.json')]=sha256_file(continuous/'summary.json');sign_planes={}
    for obj in cfg['bank_objects']:
        if not obj.get('add_surface'):continue
        plane,fit=fitted_plane(frames[obj['id']],obj,plane_spec[obj['plane_index']],plane_spec,tree)
        if plane is None:raise ValueError('Bank sign plane unsupported: '+obj['id'])
        sign_planes[obj['id']]=(plane,fit)
    groups=[('restroom',[r for r in records if 'soffit' in r['region']['id']]),('bank',[r for r in records if 'bank_roof' in r['region']['id']])];slabs=[];all_masks=np.zeros(shape,bool)
    for name,group in groups:
        source_h=float(np.median([r['plane'][0][2] for r in group]));target=float(np.median([model.warp(r['plane'][0][None],'lobby')[0,2] for r in group]))
        if name=='restroom':
            old=prior['sessions']['lobby']['architectural_regions'][0];source_h=old['source_height_m'];target=old['display_joint_height_m']
        photographed=raster_photo_regions(group,frames,source_h,low,cell,shape);local_allowed=allowed.copy()
        if name=='bank':
            centre,normal=sign_planes['lobby_shinhan_heading'][0];camera=frames['lobby_shinhan_heading']['world_optical'][:3,3];toward=np.sign((camera-centre)@normal);interior=toward*((xy-centre[:2])@normal[:2]+(source_h-centre[2])*normal[2])>=.01;local_allowed&=interior.reshape(shape)
        mask=fill_bounded_holes(photographed&local_allowed,local_allowed,int(cfg['maximum_internal_hole_area_m2']/cell**2));mask&=~all_masks;mask=remove_diagonal_contacts(mask);before_prune=int(mask.sum());mask=prune_small_components(mask,int(round(cfg['minimum_connected_slab_area_m2']/cell**2)))
        if mask.sum()<10:raise ValueError('Insufficient upper-floor supported photographed ceiling: '+name)
        all_masks|=mask;slabs.append({'id':name,'records':group,'source_height_m':source_h,'bottom_m':target,'top_m':cfg['upper_floor_m'],'mask':mask,'photographed_mask':photographed,'area_m2':float(mask.sum()*cell**2),'removed_isolated_cells':before_prune-int(mask.sum()),'connected_components':int(ndimage.label(mask)[1]),'discarded_non_upper_floor_or_wall_exterior_area_m2':float((photographed&~local_allowed).sum()*cell**2)})
    semantics=json.loads((base/'semantic_objects.json').read_text());sessions={};cleanup={};top_texture=ROOT/'outputs/student_union_topology_fixed_v2/floor2/floor2_floor_2f_texture.png';inputs[str(top_texture)]=sha256_file(top_texture);add_semantics=[]
    for name,source in source_meshes.items():
        print('Service slab derivative: '+name,flush=True);dest=a.output/name;dest.mkdir();source.compute_triangle_normals();xyz=np.asarray(source.vertices)[np.asarray(source.triangles)].astype(np.float32);centres=xyz.mean(1);normals=np.asarray(source.triangle_normals);keep=np.ones(len(xyz),bool);meshes=[];uvs=[];textures=[];reports=[]
        if name=='floor2':
            in_slab=mask_lookup(all_masks,low,cell,centres[upper_ids,:2]);keep[upper_ids[in_slab]]=False
        if name=='lobby':
            # Replace the earlier cantilevered soffit by one bounded slab underside.
            start=prior['sessions']['lobby']['retained_source_triangles'];count=prior['sessions']['lobby']['architectural_regions'][0]['triangles'];keep[start:start+count]=False
            for slab in slabs:
                inside=mask_lookup(slab['photographed_mask'],low,cell,centres[:,:2]);near=abs(centres[:,2]-slab['bottom_m'])<.45;keep&=~(inside&near&(abs(normals[:,2])>.8))
                vertex,faces=closed_slab_arrays(slab['mask'],low,cell,slab['bottom_m'],slab['top_m']);uv=(vertex[:,:2]-low)/(np.array(shape[::-1])*cell);roof_texture=dest/(slab['id']+'_underside.png');gains=texture_chart(slab['records'],frames,slab['source_height_m'],low,cell,shape,roof_texture);side_texture=dest/(slab['id']+'_side.png');Image.new('RGB',(8,8),(64,69,77)).save(side_texture)
                # All three charts use the same welded geometric grid/footprint.
                for part,tri,tex in zip(['underside','top','side'],faces,[roof_texture,top_texture,side_texture]):
                    mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(vertex);mesh.triangles=o3d.utility.Vector3iVector(tri);mesh.compute_vertex_normals();meshes.append(mesh);uvs.append(uv[tri] if part!='side' else np.zeros((len(tri),3,2)));textures.append(tex)
                np.savez_compressed(dest/(slab['id']+'_slab_mask.npz'),mask=slab['mask'],allowed=allowed,photographed=slab['photographed_mask'],low=low,cell=cell,bottom=slab['bottom_m'],top=slab['top_m']);reports.append({k:v for k,v in slab.items() if k not in ['records','mask','photographed_mask']});reports[-1].update(triangles=sum(map(len,faces)),parts=[len(t) for t in faces],mask_file=slab['id']+'_slab_mask.npz',exposure_gains=gains,ceiling_fits=[r['fit'] for r in slab['records']],scope='Presentation solid, not measured slab or pipe geometry. No horizontal footprint expansion beyond retained upper-floor cells.')
            for obj in cfg['bank_objects']:
                if not obj.get('add_surface'):continue
                frame=frames[obj['id']];plane,fit=sign_planes[obj['id']]
                result,geometry=photo_mesh(o3d,frame,obj,plane,dest)
                if result is None:raise ValueError('Bank sign projection failed: '+str(geometry))
                patch,uv,texture=result;patch.vertices=o3d.utility.Vector3dVector(model.warp(np.asarray(patch.vertices),name));patch.compute_vertex_normals();meshes.append(patch);uvs.append(uv);textures.append(texture)
                display_frame={**frame,'world_optical':frame['world_optical'].copy()};display_frame['world_optical'][:3,3]=model.warp(display_frame['world_optical'][:3,3][None],name)[0];px,z=project(centres,frame['k_rgb'],display_frame['world_optical']);camera=display_frame['world_optical'][:3,3];centre=model.warp(plane[0][None],name)[0];normal=plane[1];inside=inside_polygon(px,obj['polygon']);distance=abs((centres-centre)@normal);clear=inside&(z>.25)&(z<8)&(distance<.35)&(abs(normals[:,2])<.65);keep&=~clear
                Image.fromarray(frame['rgb']).save(dest/(obj['id']+'_original.png'));geometry['world_centre']=model.warp(np.array(geometry['world_centre'])[None],name)[0].astype(float).tolist();geometry['source_world_boundary']=geometry['world_boundary'];geometry['world_boundary']=model.warp(np.array(geometry['world_boundary']),name).astype(float).tolist();add_semantics.append({**obj,'geometry':geometry,'plane_fit':fit,'world_optical':frame['world_optical'].tolist(),'K':frame['k_rgb'].tolist(),'source_rgb':obj['id']+'_original.png','source_RGB_sha256':sha256_file(dest/(obj['id']+'_original.png')),'presentation_geometry_only':True,'passability_verified':False,'superseded_source_faces':int(clear.sum())})
        np.save(dest/'retained_source_faces.npy',keep);filter_glb(base/name/'clean_textured.glb',dest/'retained.glb',keep);source.remove_triangles_by_mask(~keep);source.remove_unreferenced_vertices();retained=len(source.triangles)
        for mesh in meshes:source+=mesh
        source.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'clean_structural.ply'),source);append_planes(dest/'retained.glb',dest/'clean_textured.glb',meshes,uvs,textures)
        for obj in semantics['objects']:
            if obj['session']==name:
                for file in [obj.get('source_rgb'),obj.get('geometry',{}).get('texture')]:
                    if file and (base/name/file).exists():shutil.copyfile(base/name/file,dest/file)
        sessions[name]={**prior['sessions'][name],'retained_source_triangles':retained,'superseded_source_faces':int((~keep).sum()),'service_slabs':reports,'bounds_min_m':source.get_min_bound().tolist(),'bounds_max_m':source.get_max_bound().tolist(),'source_mesh':str((base/name/'clean_structural.ply').resolve()),'source_texture':str((base/name/'clean_textured.glb').resolve()),'additional_vertex_coordinate_changes':0};cleanup[name]={'before_faces':len(keep),'after_faces':len(source.triangles),'removed_faces':int((~keep).sum()),'ply_sha256':sha256_file(dest/'clean_structural.ply'),'glb_sha256':sha256_file(dest/'clean_textured.glb')};inputs[str((base/name/'clean_structural.ply').resolve())]=sha256_file(base/name/'clean_structural.ply');inputs[str((base/name/'clean_textured.glb').resolve())]=sha256_file(base/name/'clean_textured.glb');del xyz,source;gc.collect()
    semantics['objects']+=add_semantics;shutil.copyfile(base/'reviewed_atrium.npz',a.output/'reviewed_atrium.npz');(a.output/'semantic_objects.json').write_text(json.dumps(semantics,indent=2,ensure_ascii=False)+'\n');sessions['lobby']['semantic_objects']+=[o for o in add_semantics];manifest={**prior,'created_utc':utc_now(),'sessions':sessions,'inputs_sha256':{**prior['inputs_sha256'],**inputs},'base_display_manifest':str((base/'manifest.json').resolve()),'service_slab_completion':True,'service_slab_config':cfg,'retained_geometry_already_normalized':True,'scope':'Photo-bounded closed service slabs and bank sign restoration. Original source poses unchanged. Presentation only, not surveyed/navigation geometry.'};(a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(a.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.output/'manifest.json')},indent=2)+'\n')
    for path,h in manifest['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Source changed during generation: '+path)
    print('Service slab and bank completion finished.',flush=True)


if __name__=='__main__':main()
