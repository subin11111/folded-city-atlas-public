#!/usr/bin/env python3
"""Verify actual closed slab geometry, exact retained source and bank visibility."""
import argparse,json,sqlite3
from pathlib import Path
import numpy as np
from PIL import Image
from scipy import ndimage
from audit_structural_delivery import glb_triangle_positions
from audit_unified_presentation import embedded_images
from complete_service_slabs import closed_slab_arrays
from export_rtabmap_db import load_pose_csv
from complete_verified_upper_walls import inside_polygon
from optimize_session import sha256_file,utc_now
from presentation_geometry import LevelPresentation
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());base=Path(run['base_display_manifest']).parent;model=LevelPresentation(ROOT,run['presentation_config']);atrium=np.load(a.bundle/'reviewed_atrium.npz');results={};slab_reports=[]
    import open3d as o3d
    scene=o3d.t.geometry.RaycastingScene();geometry_ids={}
    for name,s in run['sessions'].items():
        old=o3d.io.read_triangle_mesh(str(base/name/'clean_structural.ply'));new=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'));old_xyz=np.asarray(old.vertices)[np.asarray(old.triangles)].astype(np.float32);xyz=np.asarray(new.vertices)[np.asarray(new.triangles)].astype(np.float32);keep=np.load(a.bundle/name/'retained_source_faces.npy');count=s['retained_source_triangles']
        assert np.array_equal(xyz[:count],old_xyz[keep]),'Retained geometry changed: '+name
        assert np.array_equal(xyz,glb_triangle_positions(a.bundle/name/'clean_textured.glb')),'GLB and PLY disagree'
        images=embedded_images(base/name/'clean_textured.glb');assert images==embedded_images(a.bundle/name/'clean_textured.glb')[:len(images)]
        cursor=count
        for slab in s['service_slabs']:
            d=np.load(a.bundle/name/slab['mask_file']);mask=d['mask'];assert not (mask&~d['allowed']).any();components=int(ndimage.label(mask)[1]);assert components==1,'Disconnected slab islands';vertices,parts=closed_slab_arrays(mask,d['low'],d['cell'],d['bottom'],d['top']);faces=np.concatenate(parts);actual=xyz[cursor:cursor+len(faces)];expected=vertices[faces].astype(np.float32);assert np.array_equal(actual,expected)
            edges=np.sort(np.r_[faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]],axis=1);_,multiplicity=np.unique(edges,axis=0,return_counts=True);assert (multiplicity==2).all(),'Unclosed or nonmanifold slab'
            volume=float(abs(np.einsum('ij,ij->i',vertices[faces[:,0]],np.cross(vertices[faces[:,1]],vertices[faces[:,2]])).sum()/6));expected_volume=float(mask.sum()*d['cell']**2*(d['top']-d['bottom']));assert abs(volume-expected_volume)<1e-7
            y,x=np.where(mask);xy=d['low']+np.c_[x+.5,y+.5]*d['cell'];ij=np.floor((xy-atrium['low'])/atrium['cell']).astype(int);h,w=atrium['core'].shape;valid=(ij[:,0]>=0)&(ij[:,0]<w)&(ij[:,1]>=0)&(ij[:,1]<h);assert not atrium['core'][ij[valid,1],ij[valid,0]].any()
            slab_reports.append({'id':slab['id'],'area_m2':float(mask.sum()*d['cell']**2),'bottom_m':float(d['bottom']),'top_m':float(d['top']),'connected_components':components,'boundary_edges':int((multiplicity==1).sum()),'nonmanifold_edges':int((multiplicity>2).sum()),'volume_m3':volume,'volume_matches_closed_extrusion':True,'single_flat_underside_and_top':True,'all_cells_within_existing_complete_upper_floor':True,'new_ceilings_inside_open_atrium_core':0,'actual_GLB_PLY_slab_coordinates_verified':True});cursor+=len(faces)
        geometry_ids[name]=scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(new));results[name]={'retained_source_coordinates_exact':True,'actual_GLB_PLY_equal':True,'preserved_embedded_source_images':len(images),'retained_triangles':count,'removed_triangles':int((~keep).sum()),'added_triangles':len(xyz)-count,'ply_sha256':sha256_file(a.bundle/name/'clean_structural.ply'),'glb_sha256':sha256_file(a.bundle/name/'clean_textured.glb')};print('Verified service-slab derivative: '+name,flush=True)
    selected_pointer=json.loads((ROOT/'config/current_map.json').read_text());selected_path=ROOT/selected_pointer['manifest'];selected=json.loads(selected_path.read_text());semantic=json.loads((a.bundle/'semantic_objects.json').read_text());signs=[];cursor=run['sessions']['lobby']['retained_source_triangles']+sum(s['triangles'] for s in run['sessions']['lobby']['service_slabs'])
    for obj in [v for v in semantic['objects'] if 'shinhan' in v['id']]:
        poly=np.asarray(obj['polygon']);lo=poly.min(0);hi=poly.max(0);x,y=np.meshgrid(np.arange(lo[0]+8,hi[0]-8,8),np.arange(lo[1]+8,hi[1]-8,8));pixels=np.c_[x.ravel(),y.ravel()];pixels=pixels[inside_polygon(pixels,poly)];k=np.asarray(obj['K']);pose=np.asarray(obj['world_optical']);camera=model.warp(pose[:3,3][None],obj['session'])[0];rays=np.c_[(pixels[:,0]-k[0,2])/k[0,0],(pixels[:,1]-k[1,2])/k[1,1],np.ones(len(pixels))]@pose[:3,:3].T;cast=np.c_[np.broadcast_to(camera,rays.shape),rays].astype(np.float32);hit=scene.cast_rays(o3d.core.Tensor(cast));ids=hit['primitive_ids'].numpy();g=hit['geometry_ids'].numpy();count=obj['geometry']['triangles'];visible=(g==geometry_ids[obj['session']])&(ids>=cursor)&(ids<cursor+count)
        spec=selected['sessions'][obj['session']];poses=load_pose_csv(selected_path.parent/spec['trajectory']);db=selected['database_integrity'][obj['session']]['database']
        with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:frame=decode_frame(con,obj['node'],{**spec['preprocessing'],'max_depth':5.5},poses[obj['node']])
        assert np.array_equal(frame['rgb'],np.asarray(Image.open(a.bundle/obj['session']/obj['source_rgb']).convert('RGB'))),'Original DB RGB changed'
        slab=next(s for s in run['sessions']['lobby']['service_slabs'] if s['id']=='bank');d=np.load(a.bundle/'lobby'/slab['mask_file']);y,x=np.where(d['mask']);xy=d['low']+np.c_[x+.5,y+.5]*d['cell'];centre=np.asarray(obj['geometry']['plane_centre']);normal=np.asarray(obj['geometry']['plane_normal']);toward=np.sign((pose[:3,3]-centre)@normal);inside_distance=toward*((xy-centre[:2])@normal[:2]+(slab['source_height_m']-centre[2])*normal[2]);assert inside_distance.min()>=.01-1e-6,'Ceiling passes through bank wall'
        blocked={name:int(((g==gid)&~visible).sum()) for name,gid in geometry_ids.items()};blocked['miss']=int((~np.isfinite(hit['t_hit'].numpy())).sum());blocked['lobby_new_slab']=int(((g==geometry_ids['lobby'])&(ids>=run['sessions']['lobby']['retained_source_triangles'])&(ids<cursor)).sum())
        colours=frame['rgb'][pixels[:,1].astype(int),pixels[:,0].astype(int)].astype(float);blue=(colours[:,2]>colours[:,0]*1.25+8)&(colours[:,2]>colours[:,1]*1.15+5)
        signs.append({'id':obj['id'],'source_node':obj['node'],'source_RGB_pixels_exact':True,'source_RGB_sha256':obj['source_RGB_sha256'],'sampled_photo_rays':len(pixels),'new_sign_first_hit_fraction':float(visible.mean()),'blue_heading_first_hit_fraction':float(visible[blue].mean()) if blue.any() else None,'blocked_rays_by_geometry':blocked,'scope':'Visibility from vertically adjusted virtual source camera, not physical pose accuracy.'});cursor+=count
    for file,digest in run['inputs_sha256'].items():assert sha256_file(Path(file))==digest,'Source binding changed: '+file
    ptr=json.loads((ROOT/'config/current_map.json').read_text());assert sha256_file(ROOT/ptr['manifest'])==run['selected_manifest_sha256'];selected=json.loads((ROOT/ptr['manifest']).read_text());dbs={n:sha256_file(Path(s['database'])) for n,s in selected['database_integrity'].items()};assert all(dbs[n]==s['sha256'] for n,s in selected['database_integrity'].items())
    result={'created_utc':utc_now(),'bundle_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'selected_map_unchanged':True,'selected_map_manifest_sha256':run['selected_manifest_sha256'],'source_databases_unchanged':dbs,'input_bindings_valid':True,'sessions':results,'slabs':slab_reports,'bank_signs':signs,'audit_source_sha256':sha256_file(Path(__file__)),'scope':'Closed presentation volumes, no further retained coordinate changes, not surveyed slab dimensions or navigation geometry.'};a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
